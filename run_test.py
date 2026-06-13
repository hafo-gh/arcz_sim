#!/usr/bin/env python3
"""Run one PX4 + Gazebo Docker SITL scenario and collect artifacts."""

from __future__ import annotations

import argparse
import json
import os
import re
import shutil
import signal
import subprocess
import sys
import threading
import time
import uuid
from dataclasses import dataclass
from pathlib import Path
from typing import Any

try:
    import yaml
except ImportError:  # pragma: no cover - JSON manifests still work.
    yaml = None

try:
    from pymavlink import mavutil
except ImportError as exc:  # pragma: no cover
    raise SystemExit("pymavlink is required: python3 -m pip install pymavlink") from exc


ROOT = Path(__file__).resolve().parent
REQUIRED_FIELDS = ("vehicle", "world", "px4_params", "mission_plan", "extra_images", "output_dir")
PX4_CUSTOM_MAIN_MODE_AUTO = 4
PX4_CUSTOM_SUB_MODE_AUTO_MISSION = 4


@dataclass
class TestConfig:
    vehicle: str
    world: str
    px4_params: Path
    mission_plan: Path
    extra_images: list[str]
    output_dir: Path
    px4_image: str = "px4io/px4-sitl-gazebo:latest"
    timeout_s: int = 90
    mavlink_url: str = "udpin:0.0.0.0:14540"
    ros_domain_id: int = 0


@dataclass
class LogFollower:
    process: subprocess.Popen[bytes]
    thread: threading.Thread


def resolve_path(value: str | Path, manifest_dir: Path) -> Path:
    path = Path(value)
    if path.is_absolute():
        return path
    manifest_relative = manifest_dir / path
    if manifest_relative.exists():
        return manifest_relative.resolve()
    return (ROOT / path).resolve()


def resolve_output_path(value: str | Path, manifest_dir: Path) -> Path:
    path = Path(value)
    if path.is_absolute():
        return path
    return (manifest_dir / path).resolve()


def load_manifest(path: Path) -> TestConfig:
    text = path.read_text(encoding="utf-8")
    if path.suffix.lower() == ".json":
        data = json.loads(text)
    else:
        if yaml is None:
            raise SystemExit("YAML manifest requires PyYAML, or use JSON.")
        data = yaml.safe_load(text)

    missing = [field for field in REQUIRED_FIELDS if field not in data]
    if missing:
        raise SystemExit(f"Manifest missing required fields: {', '.join(missing)}")

    manifest_dir = path.parent.resolve()
    return TestConfig(
        vehicle=str(data["vehicle"]),
        world=str(data["world"]),
        px4_params=resolve_path(data["px4_params"], manifest_dir),
        mission_plan=resolve_path(data["mission_plan"], manifest_dir),
        extra_images=list(data.get("extra_images") or []),
        output_dir=resolve_output_path(data["output_dir"], manifest_dir),
        px4_image=str(data.get("px4_image", "px4io/px4-sitl-gazebo:latest")),
        timeout_s=int(data.get("timeout_s", 90)),
        mavlink_url=str(data.get("mavlink_url", "udpin:0.0.0.0:14540")),
        ros_domain_id=int(data.get("ros_domain_id", 0)),
    )


def run(cmd: list[str], **kwargs: Any) -> subprocess.CompletedProcess[str]:
    return subprocess.run(cmd, text=True, check=False, **kwargs)


def append_log(log_path: Path, message: str) -> None:
    with log_path.open("a", encoding="utf-8") as log:
        log.write(message.rstrip() + "\n")


def parse_params(path: Path) -> dict[str, float]:
    params: dict[str, float] = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        stripped = line.strip()
        if not stripped or stripped.startswith("#"):
            continue
        parts = stripped.split()
        if len(parts) < 2:
            raise ValueError(f"Invalid parameter line in {path}: {line}")
        params[parts[0]] = float(parts[1])
    return params


def qgc_plan_items(path: Path) -> list[Any]:
    plan = json.loads(path.read_text(encoding="utf-8"))
    items = plan["mission"]["items"]
    mission_items = []
    for seq, item in enumerate(items):
        if item.get("type") != "SimpleItem":
            continue
        raw_params = item.get("params", [])
        params = [(0 if value is None else value) for value in raw_params]
        while len(params) < 7:
            params.append(0)
        mission_items.append(
            mavutil.mavlink.MAVLink_mission_item_int_message(
                target_system=0,
                target_component=0,
                seq=seq,
                frame=int(item["frame"]),
                command=int(item["command"]),
                current=1 if seq == 0 else 0,
                autocontinue=1 if item.get("autoContinue", True) else 0,
                param1=float(params[0]),
                param2=float(params[1]),
                param3=float(params[2]),
                param4=float(params[3]),
                x=int(float(params[4]) * 1e7),
                y=int(float(params[5]) * 1e7),
                z=float(params[6]),
                mission_type=mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
            )
        )
    return mission_items


def connect_mavlink(url: str, log_path: Path, deadline_s: int = 45) -> Any | None:
    append_log(log_path, f"[runner] Waiting for MAVLink heartbeat on {url}")
    master = mavutil.mavlink_connection(url, autoreconnect=True)
    deadline = time.monotonic() + deadline_s
    while time.monotonic() < deadline:
        heartbeat = master.wait_heartbeat(timeout=2)
        if heartbeat:
            append_log(
                log_path,
                f"[runner] MAVLink heartbeat: system={master.target_system} component={master.target_component}",
            )
            return master
    append_log(log_path, "[runner] MAVLink heartbeat not received before deadline")
    return None


def upload_params(master: Any, params: dict[str, float], log_path: Path) -> None:
    for name, value in params.items():
        encoded_name = name.encode("ascii")
        master.mav.param_set_send(
            master.target_system,
            master.target_component,
            encoded_name,
            float(value),
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
        )
        ack = master.recv_match(type="PARAM_VALUE", blocking=True, timeout=2)
        status = "ack" if ack else "no ack"
        append_log(log_path, f"[runner] param {name}={value:g}: {status}")


def upload_mission(master: Any, mission_items: list[Any], log_path: Path) -> bool:
    if not mission_items:
        append_log(log_path, "[runner] mission has no SimpleItem entries to upload")
        return False

    master.mav.mission_clear_all_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
    )
    time.sleep(0.5)
    master.mav.mission_count_send(
        master.target_system,
        master.target_component,
        len(mission_items),
        mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
    )

    sent = 0
    deadline = time.monotonic() + 20
    while sent < len(mission_items) and time.monotonic() < deadline:
        request = master.recv_match(
            type=["MISSION_REQUEST", "MISSION_REQUEST_INT"],
            blocking=True,
            timeout=3,
        )
        if request is None:
            continue
        seq = int(request.seq)
        if seq >= len(mission_items):
            continue
        item = mission_items[seq]
        item.target_system = master.target_system
        item.target_component = master.target_component
        master.mav.send(item)
        sent += 1
        append_log(log_path, f"[runner] mission item {seq} sent")

    ack = master.recv_match(type="MISSION_ACK", blocking=True, timeout=5)
    ok = bool(ack and ack.type == mavutil.mavlink.MAV_MISSION_ACCEPTED)
    append_log(log_path, f"[runner] mission upload {'accepted' if ok else 'not accepted'}")
    return ok


def start_mission(master: Any, log_path: Path) -> None:
    master.set_mode_px4(
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        PX4_CUSTOM_MAIN_MODE_AUTO,
        PX4_CUSTOM_SUB_MODE_AUTO_MISSION,
    )
    time.sleep(1)
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1,
        0,
        0,
        0,
        0,
        0,
        0,
    )
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_MISSION_START,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
    )
    append_log(log_path, "[runner] requested AUTO.MISSION mode, arm, and mission start")


def docker_start(config: TestConfig, output_dir: Path, log_path: Path) -> str:
    container = f"arcz-sim-{uuid.uuid4().hex[:12]}"
    cmd = [
        "docker",
        "run",
        "--detach",
        "--name",
        container,
        "--network",
        "host",
        "-e",
        "HEADLESS=1",
        "-e",
        f"PX4_SIM_MODEL={config.vehicle}",
        "-e",
        f"PX4_GZ_WORLD={config.world}",
        "-e",
        f"ROS_DOMAIN_ID={config.ros_domain_id}",
        "-v",
        f"{output_dir}:/test_output",
        config.px4_image,
    ]
    append_log(log_path, "[runner] " + " ".join(cmd))
    result = run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    append_log(log_path, result.stdout or "")
    if result.returncode != 0:
        raise RuntimeError(f"docker run failed with exit code {result.returncode}")
    return container


ANSI_RE = re.compile(rb"\x1b\[[0-9;?]*[ -/]*[@-~]")
PXH_PROMPT_RE = re.compile(rb"(?:\r?\x1b\[2K)?\r?pxh> ?")


def sanitize_log_chunk(chunk: bytes) -> str:
    chunk = ANSI_RE.sub(b"", chunk)
    chunk = PXH_PROMPT_RE.sub(b"", chunk)
    return chunk.decode("utf-8", errors="replace")


def clean_log_file(log_path: Path) -> None:
    if not log_path.exists():
        return
    text = log_path.read_text(encoding="utf-8", errors="replace")
    text = re.sub(r"\x1b\[[0-9;?]*[ -/]*[@-~]", "", text)
    text = text.replace("\r", "")
    text = text.replace("pxh>", "")
    text = text.replace("pxh[runner]", "[runner]")
    text = text.replace("p[runner]", "[runner]")
    text = text.replace("[r[runner]", "[runner]")
    text = text.replace(" runner]", "[runner]")
    text = text.replace("unner]", "[runner]")
    text = re.sub(r"[ \t]+\n", "\n", text)
    text = re.sub(r"\n{4,}", "\n\n\n", text)
    log_path.write_text(text, encoding="utf-8")


def docker_logs(container: str, log_path: Path) -> LogFollower:
    process = subprocess.Popen(
        ["docker", "logs", "--follow", container],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    )
    assert process.stdout is not None

    def pump() -> None:
        with log_path.open("a", encoding="utf-8") as log_file:
            while True:
                chunk = process.stdout.read(4096)
                if not chunk:
                    break
                cleaned = sanitize_log_chunk(chunk)
                if cleaned:
                    log_file.write(cleaned)
                    log_file.flush()

    thread = threading.Thread(target=pump, name=f"{container}-logs", daemon=True)
    thread.start()
    return LogFollower(process=process, thread=thread)


def collect_ulg(container: str, output_dir: Path, log_path: Path) -> str | None:
    find_cmd = "find /root /home /tmp -type f -name '*.ulg' 2>/dev/null | sort | tail -n 1"
    result = run(["docker", "exec", container, "sh", "-lc", find_cmd], stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    candidate = (result.stdout or "").strip().splitlines()[-1:] or [""]
    remote_path = candidate[0]
    if not remote_path:
        append_log(log_path, "[runner] no .ulg file found in container")
        return None
    copy_result = run(["docker", "cp", f"{container}:{remote_path}", str(output_dir / "px4.ulg")], stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    append_log(log_path, copy_result.stdout or "")
    if copy_result.returncode == 0:
        append_log(log_path, f"[runner] copied {remote_path} to px4.ulg")
        return remote_path
    append_log(log_path, f"[runner] failed to copy ULG from {remote_path}")
    return None


def write_metadata(path: Path, data: dict[str, Any]) -> None:
    path.write_text(json.dumps(data, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("manifest", type=Path)
    args = parser.parse_args()

    config = load_manifest(args.manifest.resolve())
    for needed in (config.px4_params, config.mission_plan):
        if not needed.exists():
            raise SystemExit(f"Input file does not exist: {needed}")

    output_dir = config.output_dir
    output_dir.mkdir(parents=True, exist_ok=True)
    log_path = output_dir / "run.log"
    if log_path.exists():
        log_path.unlink()

    metadata: dict[str, Any] = {
        "started_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "manifest": str(args.manifest.resolve()),
        "config": {
            "vehicle": config.vehicle,
            "world": config.world,
            "px4_params": str(config.px4_params),
            "mission_plan": str(config.mission_plan),
            "extra_images": config.extra_images,
            "output_dir": str(config.output_dir),
            "px4_image": config.px4_image,
            "timeout_s": config.timeout_s,
            "mavlink_url": config.mavlink_url,
            "ros_domain_id": config.ros_domain_id,
        },
        "status": "running",
        "artifacts": {"run_log": "run.log", "px4_ulg": None},
    }
    write_metadata(output_dir / "metadata.json", metadata)

    shutil.copy2(config.px4_params, output_dir / "input.params")
    shutil.copy2(config.mission_plan, output_dir / "input.plan")

    container = ""
    log_follower: LogFollower | None = None
    exit_code = 0
    try:
        container = docker_start(config, output_dir, log_path)
        metadata["container"] = container
        write_metadata(output_dir / "metadata.json", metadata)
        log_follower = docker_logs(container, log_path)

        master = connect_mavlink(config.mavlink_url, log_path)
        if master is not None:
            upload_params(master, parse_params(config.px4_params), log_path)
            if upload_mission(master, qgc_plan_items(config.mission_plan), log_path):
                start_mission(master, log_path)

        append_log(log_path, f"[runner] letting scenario run for {config.timeout_s}s")
        time.sleep(config.timeout_s)
        ulg_path = collect_ulg(container, output_dir, log_path)
        metadata["artifacts"]["px4_ulg"] = "px4.ulg" if ulg_path else None
        metadata["container_ulg_path"] = ulg_path
        metadata["status"] = "completed"
    except KeyboardInterrupt:
        metadata["status"] = "interrupted"
        exit_code = 130
    except Exception as exc:
        append_log(log_path, f"[runner] ERROR: {exc}")
        metadata["status"] = "failed"
        metadata["error"] = str(exc)
        exit_code = 1
    finally:
        metadata["finished_at"] = time.strftime("%Y-%m-%dT%H:%M:%S%z")
        if container:
            if metadata["artifacts"]["px4_ulg"] is None:
                ulg_path = collect_ulg(container, output_dir, log_path)
                metadata["artifacts"]["px4_ulg"] = "px4.ulg" if ulg_path else None
                metadata["container_ulg_path"] = ulg_path
            run(["docker", "stop", "--time", "10", container], stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
            run(["docker", "rm", container], stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        if log_follower and log_follower.process.poll() is None:
            log_follower.process.send_signal(signal.SIGTERM)
            try:
                log_follower.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                log_follower.process.kill()
        if log_follower:
            log_follower.thread.join(timeout=5)
        clean_log_file(log_path)
        write_metadata(output_dir / "metadata.json", metadata)

    print(f"Artifacts written to {output_dir}")
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
