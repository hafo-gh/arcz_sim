# AR-CZ PX4/Gazebo Simulation Test Runner

Minimal automated test harness for running one PX4 + Gazebo SITL scenario in Docker and collecting deterministic artifacts.

## Current scope

- One PX4/Gazebo run per invocation.
- One manifest file in YAML or JSON.
- One PX4 parameter file and one QGroundControl `.plan` mission file.
- Headless Gazebo via upstream `px4io/px4-sitl-gazebo`.
- Artifacts in the requested output directory:
  - `run.log`
  - `px4.ulg` when PX4 produces one
  - `metadata.json`

`extra_images` is accepted in the manifest but intentionally not launched yet. It is part of the Phase 1 contract for the later ROS 2 companion-container phase.

## Requirements

- Linux host with Docker.
- Python 3.
- Python packages: `PyYAML`, `pymavlink`.

Both packages are already present in this workspace environment.

## Run the example

```bash
python3 run_test.py scenarios/0_example/scenario.yaml
```

The example writes artifacts under `scenarios/0_example/recent_output/`.

## Manifest fields

```yaml
vehicle: gz_x500
world: default
px4_params: inputs/hello.params
mission_plan: inputs/hello.plan
extra_images: []
output_dir: recent_output
```

Optional fields currently supported by the runner:

- `px4_image`: Docker image, default `px4io/px4-sitl-gazebo:latest`.
- `timeout_s`: total run time after container start, default `90`.
- `mavlink_url`: MAVLink endpoint used by the runner, default `udpin:0.0.0.0:14540`.
- `ros_domain_id`: exported into the PX4 container, default `0`.

## Notes

The runner uses Docker host networking. This keeps UDP ports explicit and practical for PX4 SITL, QGroundControl, MAVSDK, and future ROS 2 containers on a Linux test host.
