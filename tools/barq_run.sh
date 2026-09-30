#!/usr/bin/env bash
# Run a command (default: interactive bash) in the barq:dev container, with GPU, host network,
# shared /dev/shm (FastDDS — mandatory for cross-container ROS), and the Jetson's :0 display
# (the VNC-viewed desktop). Replaces the pre-reflash ~/run_barq.sh + ~/run_barq_gui.sh.
#   tools/barq_run.sh                         # shell
#   tools/barq_run.sh -n barq_sim ros2 launch barq_bringup sim.launch.py gait:=true gui:=true
#   -n NAME  run detached as a named container (docker logs NAME / docker stop NAME)
set -euo pipefail
NAME=""; if [ "${1:-}" = "-n" ]; then NAME=$2; shift 2; fi
XAUTH=/run/user/$(id -u)/gdm/Xauthority
ARGS=(--runtime nvidia --network host --ipc host --shm-size=8g
      -v /dev/shm:/dev/shm -v "$HOME/barq_ws:/root/barq_ws"
      -e DISPLAY=:0 -e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix
      -e XAUTHORITY=/tmp/.Xauthority -v "$XAUTH:/tmp/.Xauthority:ro")
[ -n "${ROS_DOMAIN_ID:-}" ] && ARGS+=(-e ROS_DOMAIN_ID)
SRC='source /opt/ros/humble/setup.bash; [ -f /root/barq_ws/install/setup.bash ] && source /root/barq_ws/install/setup.bash'
if [ -n "$NAME" ]; then
  docker rm -f "$NAME" >/dev/null 2>&1 || true
  docker run -d --name "$NAME" "${ARGS[@]}" barq:dev bash -lc "$SRC; exec $*"
elif [ $# -gt 0 ]; then
  docker run --rm -t "${ARGS[@]}" barq:dev bash -lc "$SRC; $*"
else
  docker run --rm -it "${ARGS[@]}" barq:dev bash -l
fi
