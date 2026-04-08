
# VR Teleoperation of Franka FR3 (ROS 2 + MoveIt Servo + Meta Quest)

This repository contains our FR3 teleoperation stack with:
- Franka ROS 2 control (`franka_hardware` + controllers)
- MoveIt Servo velocity control
- Meta Quest bridge based on Oculus Reader
- One-click shell workflow to clean, start arm side, and start VR side

## Features
- Real robot control through `fr3_arm_controller`
- MoveIt Servo online filtering and singularity-aware slowdown
- VR pose bridge (`set_target_pose` service pipeline)
- Scripted startup and cleanup workflow (`scripts/run_franka_vr.sh`)

## Environment Setup (Our Tested Setting)

The commands below match the setup used during development and debugging.

### 1. Host requirements
- Ubuntu host with Docker
- Real-time kernel recommended by Franka docs
- Meta Quest connected via USB for ADB authorization

Reference for base container setup: [libfranka-docker](https://github.com/ZorAttC/libfranka-docker/blob/main/docker_launch_files/docker-compose.yml)

### 2. Container and workspace layout
Our active container name:

```bash
u22.04-franka_ros2_vrnew
```

Main paths used inside container:
- Franka ROS 2 overlay: `/docker_volume/ros2_ws/install/setup.bash`
- MoveIt workspace: `/ws_moveit2`
- This package source path: `/ws_moveit2/src/franka_vr`

### 3. Install dependencies in container

```bash
docker exec -it u22.04-franka_ros2_vrnew bash

source /opt/ros/humble/setup.bash
cd /ws_moveit2
rosdep install -r --from-paths src --ignore-src --rosdistro humble -y
```

Python dependencies required by VR bridge:

```bash
pip3 install --break-system-packages pure-python-adb tf-transformations termcolor
```

ADB tool (if missing):

```bash
apt-get update && apt-get install -y android-tools-adb
```

### 4. Build

```bash
docker exec u22.04-franka_ros2_vrnew bash -lc '
  source /opt/ros/humble/setup.bash && \
  source /docker_volume/ros2_ws/install/setup.bash && \
  cd /ws_moveit2 && \
  colcon build --packages-select franka_vr --cmake-args -DCMAKE_BUILD_TYPE=Release
'
```

## Start Teleoperation (Using Our Shell Script)

The script is:

```bash
scripts/run_franka_vr.sh
```

Default parameters inside the script:
- `CONTAINER_NAME=u22.04-franka_ros2_vrnew`
- `ROS_DOMAIN_ID_VALUE=66`
- `ROBOT_IP=192.168.1.11`

### Step 1. Cleanup stale processes

```bash
cd /home/franka/Collect_demo/franka_vr
./scripts/run_franka_vr.sh cleanup
```

### Step 2. Start arm side (terminal A)

```bash
cd /home/franka/Collect_demo/franka_vr
./scripts/run_franka_vr.sh arm
```

This launches:
- `franka_twist.launch.py`
- ros2_control + MoveIt Servo pipeline
- optional RViz (`USE_RVIZ=true` by default)

### Step 3. Start VR side (terminal B)

```bash
cd /home/franka/Collect_demo/franka_vr
./scripts/run_franka_vr.sh vr
```

Script behavior:
- checks ADB availability
- ensures a Quest device is authorized (`adb devices` must show `device`)
- avoids duplicate VR bridge startup
- locates and runs `start_vr.sh`

### Control mapping
- Hold `rightTrig` to send arm motion commands
- Hold `rightGrip` to close gripper
- Release `rightGrip` to open gripper

## Useful Script Commands

```bash
./scripts/run_franka_vr.sh check
```

Prints ROS graph diagnostics (`node list`, `/tf`, `/tf_static`) from inside the container.

## Troubleshooting (From Real Runs)

1. `No ADB device detected`
- Reconnect Quest USB and allow authorization popup in headset.
- Verify: `adb devices` shows one line with state `device`.

2. VR script starts but arm does not move
- Confirm trigger is actually pressed (`rightTrig > 0`), otherwise commands are gated.

3. Build succeeds but launch cannot find executable link
- Rebuild `franka_vr` and verify binary exists under install tree.
- If needed, refresh the symlink used by launch path:

```bash
docker exec u22.04-franka_ros2_vrnew bash -lc '
  mkdir -p /ws_moveit2/install/moveit_servo/lib/moveit_servo && \
  ln -sf /ws_moveit2/install/franka_vr/lib/franka_vr/demo_franka_vr_vel \
         /ws_moveit2/install/moveit_servo/lib/moveit_servo/demo_franka_vr_vel
'
```

4. Arm aborts with `cartesian_reflex`
- This is a hardware safety reflex (too aggressive command/jump), not only a launch problem.
- Reduce motion aggressiveness and check VR target continuity.

## References
- [MoveIt Servo Tutorial](https://moveit.picknik.ai/main/doc/examples/realtime_servo/realtime_servo_tutorial.html)
- [MoveIt 2](https://github.com/moveit/moveit2)
- [Oculus Reader](https://github.com/rail-berkeley/oculus_reader)
- [Franka ROS 2](https://github.com/frankaemika/franka_ros2)
- [Franka Installation Guide](https://frankaemika.github.io/docs/installation_linux.html)
