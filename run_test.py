#!/usr/bin/env python3
"""Run one PX4 + Gazebo Docker SITL scenario and collect artifacts."""

from __future__ import annotations

import argparse
import json
import os
import re
import select
import shutil
import signal
import socket
import subprocess
import sys
import termios
import threading
import time
import tty
import uuid
import xml.etree.ElementTree as ET
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
REQUIRED_FIELDS = ("vehicle", "px4_params", "mission_plan", "extra_images", "output_dir")
PX4_CUSTOM_MAIN_MODE_AUTO = 4
PX4_CUSTOM_SUB_MODE_AUTO_MISSION = 4
PX4_READY_TIMEOUT_S = 90
PX4_ARM_TIMEOUT_S = 12
PX4_ARM_ATTEMPTS = 3

REQUIRED_PREFLIGHT_SENSORS = (
    ("gyro", mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_GYRO),
    ("accelerometer", mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_ACCEL),
    ("magnetometer", mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_MAG),
    ("barometer", mavutil.mavlink.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE),
    ("gps", mavutil.mavlink.MAV_SYS_STATUS_SENSOR_GPS),
)

@dataclass
class TestConfig:
    vehicle: str
    world_name: str | None
    world_file: Path | None
    px4_params: Path
    mission_plan: Path | None
    extra_images: list[str]
    output_dir: Path
    px4_image: str = "px4io/px4-sitl-gazebo:latest"
    timeout_s: int | None = 90
    mavlink_url: str = "udpin:0.0.0.0:14540"
    ros_domain_id: int = 0
    px4_net_interface: str | None = None
    qgc_host: str | None = None
    qgc_port: int = 14550
    qgc_mode: str = "unicast"
    qgc_proxy_px4_port: int = 14551
    gpu: str = "none"
    serial_bridge: "SerialBridgeConfig | None" = None


@dataclass
class SerialBridgeConfig:
    device: Path
    baudrate: int
    px4_host: str
    px4_port: int
    bind_host: str
    bind_port: int


@dataclass
class SerialBridge:
    config: SerialBridgeConfig
    process: subprocess.Popen[str] | None
    stop_event: threading.Event
    thread: threading.Thread


@dataclass
class LogFollower:
    process: subprocess.Popen[bytes]
    thread: threading.Thread


@dataclass
class GCSHeartbeat:
    stop_event: threading.Event
    thread: threading.Thread


@dataclass
class QGCProxy:
    stop_event: threading.Event
    thread: threading.Thread
    public_socket: socket.socket
    px4_socket: socket.socket


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


def sdf_world_name(path: Path) -> str:
    try:
        root = ET.parse(path).getroot()
    except ET.ParseError as exc:
        raise SystemExit(f"Could not parse world_file as XML/SDF: {path}: {exc}") from exc

    world = root.find("world")
    if world is None:
        raise SystemExit(f"world_file does not contain a top-level <world> element: {path}")
    name = world.get("name")
    if not name:
        raise SystemExit(f"world_file <world> element is missing a name attribute: {path}")
    return name


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
    if "world_name" in data and "world_file" in data:
        raise SystemExit("Manifest must use only one of world_name or world_file.")
    if "world_name" not in data and "world_file" not in data and "world" not in data:
        raise SystemExit("Manifest must define either world_name or world_file.")

    manifest_dir = path.parent.resolve()
    world_file = None
    world_name = None
    if "world_file" in data:
        world_file = resolve_path(data["world_file"], manifest_dir)
        if world_file.suffix.lower() not in (".world", ".sdf"):
            raise SystemExit(f"world_file must end in .world or .sdf: {world_file}")
    else:
        world_name = str(data.get("world_name", data.get("world", "default")))

    serial_bridge = load_serial_bridge(data.get("serial_bridge"), manifest_dir)
    mission_plan = None
    if data["mission_plan"] is not None:
        mission_plan = resolve_path(data["mission_plan"], manifest_dir)

    return TestConfig(
        vehicle=str(data["vehicle"]),
        world_name=world_name,
        world_file=world_file,
        px4_params=resolve_path(data["px4_params"], manifest_dir),
        mission_plan=mission_plan,
        extra_images=list(data.get("extra_images") or []),
        output_dir=resolve_output_path(data["output_dir"], manifest_dir),
        px4_image=str(data.get("px4_image", "px4io/px4-sitl-gazebo:latest")),
        timeout_s=parse_timeout(data["timeout_s"]) if "timeout_s" in data else 90,
        mavlink_url=str(data.get("mavlink_url", "udpin:0.0.0.0:14540")),
        ros_domain_id=int(data.get("ros_domain_id", 0)),
        px4_net_interface=str(data["px4_net_interface"]) if data.get("px4_net_interface") else None,
        qgc_host=str(data["qgc_host"]) if data.get("qgc_host") else None,
        qgc_port=int(data.get("qgc_port", 14550)),
        qgc_mode=parse_qgc_mode(data.get("qgc_mode", "unicast")),
        qgc_proxy_px4_port=int(data.get("qgc_proxy_px4_port", 14551)),
        gpu=parse_gpu_mode(data.get("gpu", "none")),
        serial_bridge=serial_bridge,
    )


def parse_qgc_mode(value: Any) -> str:
    mode = str(value).strip().lower()
    if mode not in ("unicast", "multicast", "proxy"):
        raise SystemExit("qgc_mode must be one of: unicast, multicast, proxy")
    return mode


def parse_gpu_mode(value: Any) -> str:
    mode = str(value).strip().lower()
    if mode not in ("none", "auto", "nvidia"):
        raise SystemExit("gpu must be one of: none, auto, nvidia")
    return mode


def parse_timeout(value: Any) -> int | None:
    if value is None:
        return None
    if isinstance(value, str) and value.strip().lower() in ("none", "null", "unlimited", "infinite"):
        return None
    timeout = int(value)
    if timeout <= 0:
        return None
    return timeout


def load_serial_bridge(data: Any, manifest_dir: Path) -> SerialBridgeConfig | None:
    if data is None:
        return None
    if not isinstance(data, dict):
        raise SystemExit("serial_bridge must be a mapping.")
    if not bool(data.get("enabled", False)):
        return None
    return SerialBridgeConfig(
        device=resolve_output_path(str(data.get("device", "/dev/ttySIM0")), manifest_dir),
        baudrate=int(data.get("baudrate", 921600)),
        px4_host=str(data.get("px4_host", "127.0.0.1")),
        px4_port=int(data.get("px4_port", 18570)),
        bind_host=str(data.get("bind_host", "127.0.0.1")),
        bind_port=int(data.get("bind_port", 14550)),
    )


def run(cmd: list[str], **kwargs: Any) -> subprocess.CompletedProcess[str]:
    return subprocess.run(cmd, text=True, check=False, **kwargs)


def locked_send(send_lock: threading.Lock | None, send: Any) -> None:
    if send_lock is None:
        send()
        return
    with send_lock:
        send()


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


def docker_is_running(container: str) -> bool:
    result = run(
        ["docker", "inspect", "-f", "{{.State.Running}}", container],
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
    )
    return result.returncode == 0 and (result.stdout or "").strip() == "true"


def connect_mavlink(url: str, log_path: Path, container: str | None = None, deadline_s: int = 45) -> Any | None:
    append_log(log_path, f"[runner] Waiting for MAVLink heartbeat on {url}")
    master = mavutil.mavlink_connection(url, autoreconnect=True)
    deadline = time.monotonic() + deadline_s
    while time.monotonic() < deadline:
        if container and not docker_is_running(container):
            append_log(log_path, "[runner] PX4 container exited before MAVLink heartbeat")
            return None
        heartbeat = master.wait_heartbeat(timeout=2)
        if heartbeat:
            append_log(
                log_path,
                f"[runner] MAVLink heartbeat: system={master.target_system} component={master.target_component}",
            )
            return master
    append_log(log_path, "[runner] MAVLink heartbeat not received before deadline")
    return None


def start_gcs_heartbeat(master: Any, log_path: Path, send_lock: threading.Lock) -> GCSHeartbeat:
    stop_event = threading.Event()

    def pump() -> None:
        while not stop_event.is_set():
            try:
                locked_send(
                    send_lock,
                    lambda: master.mav.heartbeat_send(
                        mavutil.mavlink.MAV_TYPE_GCS,
                        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                        0,
                        0,
                        mavutil.mavlink.MAV_STATE_ACTIVE,
                    ),
                )
            except Exception as exc:
                append_log(log_path, f"[runner] GCS heartbeat send failed: {exc}")
            stop_event.wait(1.0)

    thread = threading.Thread(target=pump, name="gcs-heartbeat", daemon=True)
    thread.start()
    append_log(log_path, "[runner] sending MAVLink GCS heartbeats")
    return GCSHeartbeat(stop_event=stop_event, thread=thread)


def stop_gcs_heartbeat(heartbeat: GCSHeartbeat | None) -> None:
    if heartbeat is None:
        return
    heartbeat.stop_event.set()
    heartbeat.thread.join(timeout=3)


def upload_params(
    master: Any,
    params: dict[str, float],
    log_path: Path,
    send_lock: threading.Lock | None = None,
) -> None:
    for name, value in params.items():
        encoded_name = name.encode("ascii")
        locked_send(
            send_lock,
            lambda: master.mav.param_set_send(
                master.target_system,
                master.target_component,
                encoded_name,
                float(value),
                mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
            ),
        )
        ack = master.recv_match(type="PARAM_VALUE", blocking=True, timeout=2)
        status = "ack" if ack else "no ack"
        append_log(log_path, f"[runner] param {name}={value:g}: {status}")


def upload_mission(
    master: Any,
    mission_items: list[Any],
    log_path: Path,
    send_lock: threading.Lock | None = None,
) -> bool:
    if not mission_items:
        append_log(log_path, "[runner] mission has no SimpleItem entries to upload")
        return False

    locked_send(
        send_lock,
        lambda: master.mav.mission_clear_all_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
        ),
    )
    time.sleep(0.5)
    locked_send(
        send_lock,
        lambda: master.mav.mission_count_send(
            master.target_system,
            master.target_component,
            len(mission_items),
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
        ),
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
        locked_send(send_lock, lambda item=item: master.mav.send(item))
        sent += 1
        append_log(log_path, f"[runner] mission item {seq} sent")

    ack = master.recv_match(type="MISSION_ACK", blocking=True, timeout=5)
    ok = bool(ack and ack.type == mavutil.mavlink.MAV_MISSION_ACCEPTED)
    append_log(log_path, f"[runner] mission upload {'accepted' if ok else 'not accepted'}")
    return ok


def missing_sensor_health(sys_status: Any | None) -> list[str]:
    if sys_status is None:
        return [name for name, _bit in REQUIRED_PREFLIGHT_SENSORS]
    health = int(sys_status.onboard_control_sensors_health)
    return [name for name, bit in REQUIRED_PREFLIGHT_SENSORS if not (health & bit)]


def message_is_fresh(message: Any | None, now: float, max_age_s: float = 5.0) -> bool:
    if message is None:
        return False
    return now - float(message._timestamp) <= max_age_s


def wait_for_vehicle_ready(master: Any, log_path: Path, timeout_s: int = PX4_READY_TIMEOUT_S) -> bool:
    """Wait until simulated sensors and global position have settled before arming."""
    append_log(log_path, f"[runner] waiting up to {timeout_s}s for PX4 sensor and position readiness")
    deadline = time.monotonic() + timeout_s
    status: Any | None = None
    gps: Any | None = None
    global_position: Any | None = None
    local_position: Any | None = None
    attitude: Any | None = None
    last_report = 0.0

    while time.monotonic() < deadline:
        now = time.monotonic()
        message = master.recv_match(
            type=[
                "SYS_STATUS",
                "GPS_RAW_INT",
                "GLOBAL_POSITION_INT",
                "LOCAL_POSITION_NED",
                "ATTITUDE",
            ],
            blocking=True,
            timeout=1,
        )
        if message is not None:
            message_type = message.get_type()
            if message_type == "SYS_STATUS":
                status = message
            elif message_type == "GPS_RAW_INT":
                gps = message
            elif message_type == "GLOBAL_POSITION_INT":
                global_position = message
            elif message_type == "LOCAL_POSITION_NED":
                local_position = message
            elif message_type == "ATTITUDE":
                attitude = message

        missing_sensors = missing_sensor_health(status)
        gps_ready = message_is_fresh(gps, now) and int(gps.fix_type) >= 3
        global_ready = (
            message_is_fresh(global_position, now)
            and int(global_position.lat) != 0
            and int(global_position.lon) != 0
        )
        local_ready = message_is_fresh(local_position, now)
        attitude_ready = message_is_fresh(attitude, now)

        if not missing_sensors and gps_ready and global_ready and local_ready and attitude_ready:
            append_log(log_path, "[runner] PX4 sensor and position readiness confirmed")
            return True

        if now - last_report >= 5:
            missing_parts = []
            if missing_sensors:
                missing_parts.append("sensor health: " + ", ".join(missing_sensors))
            if not gps_ready:
                fix_type = getattr(gps, "fix_type", "none") if gps else "none"
                missing_parts.append(f"GPS fix >= 3: current={fix_type}")
            if not global_ready:
                missing_parts.append("global position")
            if not local_ready:
                missing_parts.append("local position")
            if not attitude_ready:
                missing_parts.append("attitude")
            append_log(log_path, "[runner] PX4 not ready yet: " + "; ".join(missing_parts))
            last_report = now

    append_log(log_path, "[runner] PX4 readiness wait timed out; arming may be denied")
    return False


def wait_for_armed(master: Any, log_path: Path, timeout_s: int = PX4_ARM_TIMEOUT_S) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        heartbeat = master.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if heartbeat and int(heartbeat.base_mode) & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
            append_log(log_path, "[runner] PX4 reports vehicle armed")
            return True
    append_log(log_path, "[runner] PX4 did not report armed before timeout")
    return False


def start_mission(master: Any, log_path: Path, send_lock: threading.Lock | None = None) -> bool:
    wait_for_vehicle_ready(master, log_path)
    locked_send(
        send_lock,
        lambda: master.set_mode_px4(
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            PX4_CUSTOM_MAIN_MODE_AUTO,
            PX4_CUSTOM_SUB_MODE_AUTO_MISSION,
        ),
    )
    time.sleep(1)
    armed = False
    for attempt in range(1, PX4_ARM_ATTEMPTS + 1):
        locked_send(
            send_lock,
            lambda: master.mav.command_long_send(
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
            ),
        )
        append_log(log_path, f"[runner] arm command sent (attempt {attempt}/{PX4_ARM_ATTEMPTS})")
        if wait_for_armed(master, log_path):
            armed = True
            break
        wait_for_vehicle_ready(master, log_path, timeout_s=15)

    if not armed:
        append_log(log_path, "[runner] vehicle did not arm; mission start not requested")
        return False

    locked_send(
        send_lock,
        lambda: master.mav.command_long_send(
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
        ),
    )
    append_log(log_path, "[runner] requested AUTO.MISSION mode, arm, and mission start")

    return True


def docker_gui_args() -> list[str]:
    display = os.environ.get("DISPLAY")
    if not display:
        raise RuntimeError("show_simulation requires DISPLAY to be set on the host.")

    args = [
        "-e",
        f"DISPLAY={display}",
        "-e",
        "QT_X11_NO_MITSHM=1",
        "-v",
        "/tmp/.X11-unix:/tmp/.X11-unix:rw",
    ]

    if Path("/dev/dri").exists():
        args.extend(["--device", "/dev/dri"])

    return args


def docker_gpu_args(mode: str, log_path: Path) -> list[str]:
    if mode == "none":
        return []

    has_nvidia_smi = shutil.which("nvidia-smi") is not None
    info = run(["docker", "info", "--format", "{{json .Runtimes}}"], stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
    has_nvidia_runtime = info.returncode == 0 and '"nvidia"' in (info.stdout or "")

    if not has_nvidia_smi or not has_nvidia_runtime:
        message = "NVIDIA GPU requested but nvidia-smi or Docker nvidia runtime is not available"
        if mode == "nvidia":
            raise RuntimeError(message)
        append_log(log_path, f"[runner] {message}; continuing without GPU")
        return []

    append_log(log_path, "[runner] enabling NVIDIA GPU access for Gazebo container")
    return [
        "--gpus",
        "all",
        "--runtime",
        "nvidia",
        "-e",
        "NVIDIA_VISIBLE_DEVICES=all",
        "-e",
        "NVIDIA_DRIVER_CAPABILITIES=compute,graphics,utility,display",
        "-e",
        "__GLX_VENDOR_LIBRARY_NAME=nvidia",
    ]


def px4_qgc_target(config: TestConfig) -> tuple[str, int] | None:
    if config.qgc_mode == "proxy":
        return "127.0.0.1", config.qgc_proxy_px4_port
    if config.qgc_host:
        return config.qgc_host, config.qgc_port
    return None


def px4_mavlink_override_args(config: TestConfig, output_dir: Path, log_path: Path) -> list[str]:
    target = px4_qgc_target(config)
    if target is None:
        return []
    target_host, target_port = target

    script_path = output_dir / "px4-rc.mavlink"
    script = f"""#!/bin/sh
# Generated by arcz_sim for explicit QGroundControl {config.qgc_mode}.
# shellcheck disable=SC2154

udp_offboard_port_local=$((14580+px4_instance))
udp_offboard_port_remote=$((14540+px4_instance))
[ "$px4_instance" -gt 9 ] && udp_offboard_port_remote=14549
udp_onboard_payload_port_local=$((14280+px4_instance))
udp_onboard_payload_port_remote=$((14030+px4_instance))
udp_onboard_gimbal_port_local=$((13030+px4_instance))
udp_onboard_gimbal_port_remote=$((13280+px4_instance))
udp_gcs_port_local=$((18570+px4_instance))

mavlink_network_interface_arg=""
if [ -n "$PX4_NET_INTERFACE" ]; then
    mavlink_network_interface_arg="-n $PX4_NET_INTERFACE"
fi

# GCS link: send to QGroundControl, which should listen on UDP {config.qgc_port}.
gcs_interface_arg=""
if [ "{config.qgc_mode}" = "multicast" ]; then
    gcs_interface_arg="$mavlink_network_interface_arg -p"
fi
mavlink start -x -u $udp_gcs_port_local -r 4000000 -f -t {target_host} -o {target_port} $gcs_interface_arg
mavlink stream -r 50 -s POSITION_TARGET_LOCAL_NED -u $udp_gcs_port_local
mavlink stream -r 50 -s LOCAL_POSITION_NED -u $udp_gcs_port_local
mavlink stream -r 50 -s GLOBAL_POSITION_INT -u $udp_gcs_port_local
mavlink stream -r 50 -s ATTITUDE -u $udp_gcs_port_local
mavlink stream -r 50 -s ATTITUDE_QUATERNION -u $udp_gcs_port_local
mavlink stream -r 50 -s ATTITUDE_TARGET -u $udp_gcs_port_local
mavlink stream -r 50 -s SERVO_OUTPUT_RAW_0 -u $udp_gcs_port_local
mavlink stream -r 20 -s RC_CHANNELS -u $udp_gcs_port_local
mavlink stream -r 10 -s OPTICAL_FLOW_RAD -u $udp_gcs_port_local

# API/Offboard link
mavlink start -x -u $udp_offboard_port_local -r 4000000 -f -m onboard -o $udp_offboard_port_remote $mavlink_network_interface_arg

# Onboard link to camera
mavlink start -x -u $udp_onboard_payload_port_local -r 4000 -f -m onboard -o $udp_onboard_payload_port_remote $mavlink_network_interface_arg

# Onboard link to gimbal
mavlink start -x -u $udp_onboard_gimbal_port_local -r 400000 -f -m gimbal -o $udp_onboard_gimbal_port_remote $mavlink_network_interface_arg
"""
    script_path.write_text(script, encoding="utf-8")
    script_path.chmod(0o755)
    append_log(log_path, f"[runner] PX4 QGC MAVLink {config.qgc_mode} target: {target_host}:{target_port}")
    return ["-v", f"{script_path}:/opt/px4-gazebo/etc/init.d-posix/px4-rc.mavlink:ro"]


def docker_start(config: TestConfig, output_dir: Path, log_path: Path, show_simulation: bool) -> str:
    container = f"arcz-sim-{uuid.uuid4().hex[:12]}"
    gz_partition = container
    world_name = config.world_name or "default"
    world_mount: list[str] = []
    if config.world_file:
        world_name = sdf_world_name(config.world_file)
        world_mount = [
            "-e",
            "PX4_GZ_WORLDS=/scenario/worlds",
            "-v",
            f"{config.world_file}:/scenario/worlds/{world_name}.sdf:ro",
        ]

    display_args = docker_gui_args() if show_simulation else ["-e", "HEADLESS=1"]
    gpu_args = docker_gpu_args(config.gpu, log_path)
    mavlink_override_args = px4_mavlink_override_args(config, output_dir, log_path)
    net_interface_args = ["-e", f"PX4_NET_INTERFACE={config.px4_net_interface}"] if config.px4_net_interface else []
    cmd = [
        "docker",
        "run",
        "--detach",
        "--name",
        container,
        "--network",
        "host",
        *gpu_args,
        *display_args,
        *net_interface_args,
        *mavlink_override_args,
        "-e",
        f"PX4_SIM_MODEL={config.vehicle}",
        "-e",
        f"PX4_GZ_WORLD={world_name}",
        "-e",
        f"ROS_DOMAIN_ID={config.ros_domain_id}",
        "-e",
        "GZ_IP=127.0.0.1",
        "-e",
        f"GZ_PARTITION={gz_partition}",
        *world_mount,
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


def ensure_serial_bridge_can_start(config: SerialBridgeConfig) -> None:
    if shutil.which("socat") is None:
        raise RuntimeError("serial_bridge requires socat to be installed on the host.")
    if config.device.exists() or config.device.is_symlink():
        if config.device.is_symlink():
            config.device.unlink()
        else:
            raise RuntimeError(f"serial_bridge device already exists and is not a symlink: {config.device}")
    parent = config.device.parent
    parent.mkdir(parents=True, exist_ok=True)
    if not os.access(parent, os.W_OK):
        raise RuntimeError(
            f"Cannot create {config.device}. Run the test as a user that can write to {parent}, "
            "or choose a writable serial_bridge.device path."
        )


def start_serial_bridge(config: SerialBridgeConfig, log_path: Path) -> SerialBridge:
    ensure_serial_bridge_can_start(config)
    stop_event = threading.Event()
    ready_event = threading.Event()
    bridge = SerialBridge(config=config, process=None, stop_event=stop_event, thread=threading.Thread())

    def command() -> list[str]:
        pty_options = [
            f"link={config.device}",
            "raw",
            "echo=0",
            f"b{config.baudrate}",
            "mode=660",
            "waitslave",
            "ignoreeof",
        ]
        udp_options = [
            f"bind={config.bind_host}:{config.bind_port}",
            "reuseaddr",
        ]
        return [
            "socat",
            "-d",
            "-d",
            "PTY," + ",".join(pty_options),
            f"UDP4:{config.px4_host}:{config.px4_port}," + ",".join(udp_options),
        ]

    def supervise() -> None:
        announced = False
        while not stop_event.is_set():
            if config.device.is_symlink():
                config.device.unlink()
            cmd = command()
            append_log(log_path, "[runner] " + " ".join(cmd))
            process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
            bridge.process = process
            while process.poll() is None and not stop_event.is_set():
                if config.device.exists() and not ready_event.is_set():
                    message = (
                        f"Gazebo PX4 MAVLink serial bridge is available at "
                        f"{config.device}:{config.baudrate}"
                    )
                    print(message, flush=True)
                    append_log(log_path, f"[runner] {message}")
                    ready_event.set()
                    announced = True
                time.sleep(0.1)
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
            output = process.stdout.read() if process.stdout else ""
            if output.strip():
                append_log(log_path, "[serial_bridge] " + output.strip())
            if not stop_event.is_set():
                append_log(log_path, f"[runner] serial bridge exited with code {process.returncode}; restarting")
                if announced:
                    ready_event.clear()
                time.sleep(0.5)

    thread = threading.Thread(target=supervise, name="serial-bridge", daemon=True)
    bridge.thread = thread
    thread.start()
    if not ready_event.wait(timeout=10):
        stop_serial_bridge(bridge, log_path)
        raise RuntimeError(f"serial_bridge did not create {config.device} before timeout")
    return bridge


def stop_serial_bridge(bridge: SerialBridge | None, log_path: Path) -> None:
    if bridge is None:
        return
    bridge.stop_event.set()
    if bridge.process and bridge.process.poll() is None:
        bridge.process.terminate()
        try:
            bridge.process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            bridge.process.kill()
    bridge.thread.join(timeout=5)
    if bridge.config.device.is_symlink():
        bridge.config.device.unlink()
        append_log(log_path, f"[runner] removed serial bridge device {bridge.config.device}")


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


def start_qgc_proxy(config: TestConfig, log_path: Path) -> QGCProxy | None:
    if config.qgc_mode != "proxy":
        return None

    public_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    px4_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        public_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        px4_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        public_socket.bind(("0.0.0.0", config.qgc_port))
        px4_socket.bind(("127.0.0.1", config.qgc_proxy_px4_port))
        public_socket.setblocking(False)
        px4_socket.setblocking(False)
    except OSError:
        public_socket.close()
        px4_socket.close()
        raise

    stop_event = threading.Event()
    clients: dict[tuple[str, int], float] = {}
    client_ttl_s = 120.0
    px4_addr = ("127.0.0.1", 18570)

    def loop() -> None:
        append_log(
            log_path,
            f"[runner] QGC proxy listening on 0.0.0.0:{config.qgc_port}; "
            f"PX4 side 127.0.0.1:{config.qgc_proxy_px4_port} -> 127.0.0.1:18570",
        )
        while not stop_event.is_set():
            try:
                readable, _, _ = select.select([public_socket, px4_socket], [], [], 0.25)
            except (OSError, ValueError):
                break
            now = time.monotonic()
            for sock in readable:
                try:
                    data, addr = sock.recvfrom(65535)
                except BlockingIOError:
                    continue
                except OSError:
                    return
                if sock is public_socket:
                    if addr not in clients:
                        append_log(log_path, f"[runner] QGC proxy learned client {addr[0]}:{addr[1]}")
                    clients[addr] = now
                    px4_socket.sendto(data, px4_addr)
                    continue

                expired = [client for client, seen_at in clients.items() if now - seen_at > client_ttl_s]
                for client in expired:
                    clients.pop(client, None)
                for client in list(clients):
                    public_socket.sendto(data, client)

    thread = threading.Thread(target=loop, name="qgc-proxy", daemon=True)
    thread.start()
    return QGCProxy(stop_event=stop_event, thread=thread, public_socket=public_socket, px4_socket=px4_socket)


def stop_qgc_proxy(proxy: QGCProxy | None, log_path: Path) -> None:
    if proxy is None:
        return
    proxy.stop_event.set()
    proxy.public_socket.close()
    proxy.px4_socket.close()
    proxy.thread.join(timeout=2)
    append_log(log_path, "[runner] QGC proxy stopped")


def wait_for_run_stop(timeout_s: int | None, log_path: Path) -> str:
    if timeout_s is None:
        message = "Scenario is running without a timeout. Press q to stop and collect artifacts."
        deadline = None
    else:
        message = f"Scenario is running for {timeout_s}s. Press q to stop early and collect artifacts."
        deadline = time.monotonic() + timeout_s
    print(message, flush=True)
    append_log(log_path, f"[runner] {message}")

    if not sys.stdin.isatty():
        if timeout_s is None:
            while True:
                time.sleep(1)
        time.sleep(timeout_s)
        return "timeout"

    old_settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while True:
            remaining = None if deadline is None else max(0.0, deadline - time.monotonic())
            if remaining == 0.0:
                return "timeout"
            wait_s = 0.25 if remaining is None else min(0.25, remaining)
            readable, _, _ = select.select([sys.stdin], [], [], wait_s)
            if not readable:
                continue
            key = sys.stdin.read(1)
            if key.lower() == "q":
                append_log(log_path, "[runner] stop requested by q key")
                return "user"
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("manifest", type=Path)
    parser.add_argument(
        "run_mode",
        nargs="?",
        choices=("show_simulation",),
        help="Use show_simulation to start Gazebo with its GUI instead of headless mode.",
    )
    args = parser.parse_args()

    config = load_manifest(args.manifest.resolve())
    show_simulation = args.run_mode == "show_simulation"
    needed_inputs = [config.px4_params]
    if config.mission_plan:
        needed_inputs.append(config.mission_plan)
    for needed in needed_inputs:
        if not needed.exists():
            raise SystemExit(f"Input file does not exist: {needed}")
    if config.world_file and not config.world_file.exists():
        raise SystemExit(f"World file does not exist: {config.world_file}")

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
            "world_name": config.world_name,
            "world_file": str(config.world_file) if config.world_file else None,
            "px4_params": str(config.px4_params),
            "mission_plan": str(config.mission_plan) if config.mission_plan else None,
            "extra_images": config.extra_images,
            "output_dir": str(config.output_dir),
            "px4_image": config.px4_image,
            "timeout_s": config.timeout_s,
            "mavlink_url": config.mavlink_url,
            "ros_domain_id": config.ros_domain_id,
            "px4_net_interface": config.px4_net_interface,
            "qgc_host": config.qgc_host,
            "qgc_port": config.qgc_port,
            "qgc_mode": config.qgc_mode,
            "qgc_proxy_px4_port": config.qgc_proxy_px4_port,
            "gpu": config.gpu,
            "show_simulation": show_simulation,
            "serial_bridge": {
                "device": str(config.serial_bridge.device),
                "baudrate": config.serial_bridge.baudrate,
                "px4_endpoint": f"{config.serial_bridge.px4_host}:{config.serial_bridge.px4_port}",
                "bind_endpoint": f"{config.serial_bridge.bind_host}:{config.serial_bridge.bind_port}",
            }
            if config.serial_bridge
            else None,
        },
        "status": "running",
        "artifacts": {"run_log": "run.log", "px4_ulg": None},
    }
    write_metadata(output_dir / "metadata.json", metadata)

    shutil.copy2(config.px4_params, output_dir / "input.params")
    if config.mission_plan:
        shutil.copy2(config.mission_plan, output_dir / "input.plan")
    elif (output_dir / "input.plan").exists():
        (output_dir / "input.plan").unlink()

    container = ""
    log_follower: LogFollower | None = None
    serial_bridge: SerialBridge | None = None
    gcs_heartbeat: GCSHeartbeat | None = None
    qgc_proxy: QGCProxy | None = None
    exit_code = 0
    try:
        if show_simulation:
            append_log(log_path, "[runner] starting Gazebo with GUI because show_simulation was requested")
        qgc_proxy = start_qgc_proxy(config, log_path)
        container = docker_start(config, output_dir, log_path, show_simulation)
        metadata["container"] = container
        write_metadata(output_dir / "metadata.json", metadata)
        log_follower = docker_logs(container, log_path)

        master = connect_mavlink(config.mavlink_url, log_path, container=container)
        if master is None and container and not docker_is_running(container):
            raise RuntimeError("PX4 container exited before MAVLink heartbeat")
        if master is not None:
            mavlink_send_lock = threading.Lock()
            gcs_heartbeat = start_gcs_heartbeat(master, log_path, mavlink_send_lock)
            if config.serial_bridge:
                serial_bridge = start_serial_bridge(config.serial_bridge, log_path)
            upload_params(master, parse_params(config.px4_params), log_path, send_lock=mavlink_send_lock)
            if config.mission_plan is None:
                append_log(log_path, "[runner] no mission_plan configured; leaving vehicle idle")
            elif upload_mission(master, qgc_plan_items(config.mission_plan), log_path, send_lock=mavlink_send_lock):
                start_mission(master, log_path, send_lock=mavlink_send_lock)

        stop_reason = wait_for_run_stop(config.timeout_s, log_path)
        metadata["stop_reason"] = stop_reason
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
        stop_gcs_heartbeat(gcs_heartbeat)
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
        stop_serial_bridge(serial_bridge, log_path)
        stop_qgc_proxy(qgc_proxy, log_path)
        clean_log_file(log_path)
        write_metadata(output_dir / "metadata.json", metadata)

    print(f"Artifacts written to {output_dir}")
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
