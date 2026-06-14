# AR-CZ PX4/Gazebo Simulation Test Runner

Automated test harness for running one PX4 + Gazebo SITL scenario in Docker and collecting deterministic artifacts.

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

While a scenario is running, press `q` in the runner terminal to stop PX4/Gazebo cleanly and collect artifacts.

To show the Gazebo GUI instead of running headless:

```bash
python3 run_test.py scenarios/0_example/scenario.yaml show_simulation
```

This requires a local X11 display. If Docker cannot connect to the display, allow local root-owned containers to use your X session before starting the scenario:

```bash
xhost +SI:localuser:root
```

In GUI mode the runner intentionally leaves `HEADLESS` unset because the PX4 Gazebo startup script treats any value of `HEADLESS`, even `0`, as headless mode.

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

## Virtual Drone Scenario

For a long-running virtual drone with no uploaded mission, use:

```bash
python3 run_test.py scenarios/1_no_mission/scenario.yaml
```

This scenario sets:

```yaml
mission_plan: null
timeout_s: null
```

It keeps PX4/Gazebo running until you press `q`. This is useful when connecting QGroundControl or manually controlling the simulated vehicle via MAVLink.

The scenario parameter file sets `COM_RC_IN_MODE 1`, which tells PX4 to use MAVLink manual-control input instead of a physical RC receiver. In QGroundControl, enable the virtual joystick in the application settings, then arm and switch to a manual-capable mode such as Position or Altitude before trying to fly with the joystick.

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
- `timeout_s: null`: run until `q` is pressed.
- `mavlink_url`: MAVLink endpoint used by the runner, default `udpin:0.0.0.0:14540`. Use `14030` when a companion container is listening on PX4's `14540` onboard stream.
- `ros_domain_id`: exported into the PX4 container, default `0`.
- `mission_plan: null`: do not upload or start a mission.

World selection:

- `world_name`: built-in PX4/Gazebo world name, for example `default`.
- `world_file`: scenario-local `.world` or `.sdf` file. The runner mounts it into the PX4 container as a Gazebo SDF world.

## Reducing CPU Load

To reduce the CPU load, you can play with followeing params in the world definitions...

```xml
<max_step_size>0.004</max_step_size>
<real_time_factor>0.5</real_time_factor>
<real_time_update_rate>125</real_time_update_rate>
```

This targets about half real time instead of full real time. Missions and QGC interaction still work, but simulated time advances more slowly and Gazebo asks less from the host. To go lighter, reduce `real_time_factor` and `real_time_update_rate` together, for example `0.25` and `62.5`. To restore PX4's usual default, use `1.0` and `250`.

## Notes

The runner uses Docker host networking. This keeps UDP ports explicit and practical for PX4 SITL, QGroundControl, MAVSDK, and future ROS 2 containers on a Linux test host.
