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

## Run With Point-To-Home

For companion-container testing, use PX4 SITL MAVLink over UDP. Start the companion first, then start the simulation:

```bash
cd /home/radek/pth
docker compose up -d arcz_point_to_home

cd /home/radek/arcz_sim
python3 run_test.py scenarios/0_example/scenario.yaml
```

The point-to-home container is configured to listen on:

```text
udpin:0.0.0.0:14540
```

The sim runner uses a separate PX4 MAVLink endpoint for its own parameter and mission upload:

```text
udpin:0.0.0.0:14030
```

## Manifest fields

```yaml
vehicle: gz_x500
# Use a built-in PX4/Gazebo world by name instead of world_file:
# world_name: default
world_file: inputs/hello.world
px4_params: inputs/hello.params
mission_plan: inputs/hello.plan
extra_images: []
output_dir: recent_output

px4_image: px4io/px4-sitl-gazebo:latest
timeout_s: 45
mavlink_url: udpin:0.0.0.0:14030
ros_domain_id: 0
```

Optional fields currently supported by the runner:

- `px4_image`: Docker image, default `px4io/px4-sitl-gazebo:latest`.
- `timeout_s`: total run time after container start, default `90`.
- `mavlink_url`: MAVLink endpoint used by the runner, default `udpin:0.0.0.0:14540`. Use `14030` when a companion container is listening on PX4's `14540` onboard stream.
- `ros_domain_id`: exported into the PX4 container, default `0`.

World selection:

- `world_name`: built-in PX4/Gazebo world name, for example `default`.
- `world_file`: scenario-local `.world` or `.sdf` file. The runner mounts it into the PX4 container as a Gazebo SDF world.

## Notes

The runner uses Docker host networking. This keeps UDP ports explicit and practical for PX4 SITL, QGroundControl, MAVSDK, and future ROS 2 containers on a Linux test host.

The old pseudo-serial bridge is intentionally not used by the example. Direct UDP is simpler and matches PX4 SITL's native networking model.
