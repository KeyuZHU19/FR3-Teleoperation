#!/usr/bin/env bash
set -euo pipefail

CONTAINER_NAME="${CONTAINER_NAME:-u22.04-franka_ros2_vrnew}"
ROS_DOMAIN_ID_VALUE="${ROS_DOMAIN_ID_VALUE:-66}"
ROBOT_IP="${ROBOT_IP:-192.168.1.11}"
USE_RVIZ="${USE_RVIZ:-true}"
MOVEIT_WS="${MOVEIT_WS:-/ws_moveit2}"
FRANKA_WS_SETUP="${FRANKA_WS_SETUP:-/docker_volume/ros2_ws/install/setup.bash}"

usage() {
  cat <<EOF
Usage:
  $0 cleanup
  $0 arm
  $0 vr
  $0 check

Optional environment variables:
  CONTAINER_NAME     (default: ${CONTAINER_NAME})
  ROS_DOMAIN_ID_VALUE(default: ${ROS_DOMAIN_ID_VALUE})
  ROBOT_IP           (default: ${ROBOT_IP})
  USE_RVIZ           (default: ${USE_RVIZ})
EOF
}

require_container() {
  if ! docker ps --format '{{.Names}}' | grep -qx "${CONTAINER_NAME}"; then
    echo "Container not running: ${CONTAINER_NAME}" >&2
    exit 1
  fi
}

run_in_container() {
  local cmd="$1"
  docker exec "${CONTAINER_NAME}" bash -lc "${cmd}"
}

cleanup_stale() {
  require_container
  run_in_container "
    export ROS_DOMAIN_ID=${ROS_DOMAIN_ID_VALUE}
    source /opt/ros/humble/setup.bash
    source ${FRANKA_WS_SETUP}
    source ${MOVEIT_WS}/install/setup.bash

    kill_pattern() {
      local pattern=\"\$1\"
      local pids
      pids=\$(pgrep -f \"\$pattern\" || true)
      for pid in \$pids; do
        if [ \"\$pid\" != \"\$\$\" ] && [ \"\$pid\" != \"\$PPID\" ]; then
          kill \"\$pid\" 2>/dev/null || true
        fi
      done
      sleep 0.2
      pids=\$(pgrep -f \"\$pattern\" || true)
      for pid in \$pids; do
        if [ \"\$pid\" != \"\$\$\" ] && [ \"\$pid\" != \"\$PPID\" ]; then
          kill -9 \"\$pid\" 2>/dev/null || true
        fi
      done
    }

    # Stop launch parents first
    kill_pattern 'ros2 launch franka_vr franka_twist.launch.py'

    # Stop VR bridge
    kill_pattern 'start_franka_vr.py'
    kill_pattern 'bash ./start_vr.sh'

    # Stop nodes spawned by franka_twist.launch.py
    kill_pattern '/opt/ros/humble/lib/controller_manager/ros2_control_node'
    kill_pattern '/opt/ros/humble/lib/controller_manager/spawner'
    kill_pattern '/opt/ros/humble/lib/robot_state_publisher/robot_state_publisher'
    kill_pattern '/opt/ros/humble/lib/tf2_ros/static_transform_publisher'
    kill_pattern '/docker_volume/ros2_ws/install/franka_gripper/lib/franka_gripper/franka_gripper_node'
    kill_pattern 'static_transform_publisher.*world base'
    kill_pattern 'franka_gripper_node'
    kill_pattern 'fake_gripper_state_publisher.py'
    kill_pattern '/ws_moveit2/install/franka_vr/lib/franka_vr/demo_franka_vr_vel'

    # Refresh graph cache
    ros2 daemon stop || true
    sleep 1
    ros2 daemon start || true
  "
  echo "Cleanup finished."
}

start_arm() {
  require_container
  run_in_container "
    export ROS_DOMAIN_ID=${ROS_DOMAIN_ID_VALUE}
    source /opt/ros/humble/setup.bash
    source ${FRANKA_WS_SETUP}
    source ${MOVEIT_WS}/install/setup.bash
    cd ${MOVEIT_WS}
    ros2 launch franka_vr franka_twist.launch.py robot_ip:=${ROBOT_IP} use_fake_hardware:=false use_rviz:=${USE_RVIZ}
  "
}

start_vr() {
  require_container
  run_in_container "
    export ROS_DOMAIN_ID=${ROS_DOMAIN_ID_VALUE}
    source /opt/ros/humble/setup.bash
    source ${FRANKA_WS_SETUP}
    source ${MOVEIT_WS}/install/setup.bash

    if ! command -v adb >/dev/null 2>&1; then
      echo 'adb is not installed in container. Install with: apt-get install -y android-tools-adb' >&2
      exit 1
    fi

    adb_start_output=$(adb start-server 2>&1 || true)
    if [ -n "\$adb_start_output" ]; then
      echo "\$adb_start_output"
    fi

    adb_devices=$(adb devices | awk "NR>1 && \$2==\"device\" {print \$1}")
    if [ -z "\$adb_devices" ]; then
      echo 'No ADB device detected. Connect/authorize Quest first (adb devices should show one device).' >&2
      exit 1
    fi

    vr_running=false
    for pid in \$(pgrep -f 'start_franka_vr.py' || true); do
      if [ "\$pid" = "\$\$" ] || [ "\$pid" = "\$PPID" ]; then
        continue
      fi
      comm=\$(ps -p "\$pid" -o comm= 2>/dev/null | tr -d '[:space:]')
      if [ "\$comm" = "python3" ]; then
        vr_running=true
        break
      fi
    done
    if [ "\${vr_running:-false}" = "true" ]; then
      echo 'Oculus VR bridge is already running. Skip duplicate start.'
      exit 0
    fi

    VR_DIR=''
    for d in \
      ${MOVEIT_WS}/src/franka_vr/franka_vr/oculus_reader \
      /workspaces/franka_vr/franka_vr/oculus_reader \
      /workspaces/franka_vr/oculus_reader; do
      if [ -f \"\$d/start_vr.sh\" ]; then
        VR_DIR=\"\$d\"
        break
      fi
    done

    if [ -z \"\$VR_DIR\" ]; then
      echo 'Cannot find start_vr.sh in known paths.' >&2
      echo 'Searched:' >&2
      echo '  ${MOVEIT_WS}/src/franka_vr/franka_vr/oculus_reader' >&2
      echo '  /workspaces/franka_vr/franka_vr/oculus_reader' >&2
      echo '  /workspaces/franka_vr/oculus_reader' >&2
      exit 1
    fi

    cd \"\$VR_DIR\"
    bash ./start_vr.sh
  "
}

check_graph() {
  require_container
  run_in_container "
    export ROS_DOMAIN_ID=${ROS_DOMAIN_ID_VALUE}
    source /opt/ros/humble/setup.bash
    source ${FRANKA_WS_SETUP}
    source ${MOVEIT_WS}/install/setup.bash
    echo '=== Node duplicates ==='
    ros2 node list | sort | uniq -c
    echo '=== /tf info (first 120 lines) ==='
    ros2 topic info /tf -v | sed -n '1,120p'
    echo '=== /tf_static info (first 120 lines) ==='
    ros2 topic info /tf_static -v | sed -n '1,120p'
  "
}

main() {
  local action="${1:-}"

  case "${action}" in
    cleanup)
      cleanup_stale
      ;;
    arm)
      start_arm
      ;;
    vr)
      start_vr
      ;;
    check)
      check_graph
      ;;
    *)
      usage
      exit 1
      ;;
  esac
}

main "$@"
