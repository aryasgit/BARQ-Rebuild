#!/usr/bin/env bash
# Record a real-speed MP4 of a Gazebo walk (host-side, Jetson :0 display, needs ffmpeg + xdotool).
#   tools/record_walk.sh <name> "<label>" "<extra sim.launch args>" [VX=0.15 DUR=15]
# Launches a fresh GUI sim, frames a chase camera, grabs the 3D view locally at 30 fps (no VNC
# in the path), drives sim_walk_metric, then RE-TIMES the clip by the measured real-time factor
# (sim seconds / wall seconds) so motion plays at true physical speed. Output:
#   ~/barq_ws/artifacts/videos/<name>.mp4  (+ the WALK/ATT/TORQ lines in <name>.txt)
set -euo pipefail
NAME=$1; LABEL=$2; EXTRA=${3:-}; VX=${VX:-0.15}; DUR=${DUR:-15}
OUT=~/barq_ws/artifacts/videos; mkdir -p "$OUT"
export DISPLAY=:0 XAUTHORITY=/run/user/$(id -u)/gdm/Xauthority
RUN="$(dirname "$0")/barq_run.sh"
SRC='source /opt/ros/humble/setup.bash; source /root/barq_ws/install/setup.bash'
dex() { docker exec barq_sim bash -lc "$SRC; $*"; }
simtime() { dex "timeout 5 ign topic -e -t /stats -n 1" | awk '/sim_time/{f=1} f&&$1=="sec:"{s=$2} f&&$1=="nsec:"{printf "%.3f\n", s+$2/1e9; exit}'; }

"$RUN" -n barq_sim ros2 launch barq_bringup sim.launch.py gait:=true gui:=true $EXTRA >/dev/null
for _ in $(seq 60); do docker logs barq_sim 2>&1 | grep -q 'activated joint_group_position_controller' && break; sleep 2; done
for _ in $(seq 30); do W=$(xdotool search --name '^Gazebo$' 2>/dev/null | head -1) && [ -n "$W" ] && break; sleep 1; done
sleep 3
xdotool windowactivate "$W" windowsize "$W" 1850 1013 windowmove "$W" 70 27; sleep 1
sleep 8   # GUI must finish loading or /gui/follow is accepted but ignored
for _ in 1 2; do dex "ign service -s /gui/follow/offset --reqtype ignition.msgs.Vector3d --reptype ignition.msgs.Boolean --timeout 3000 --req 'x: -0.45, y: -0.62, z: 0.28' >/dev/null; ign service -s /gui/follow --reqtype ignition.msgs.StringMsg --reptype ignition.msgs.Boolean --timeout 3000 --req 'data: \"barq\"' >/dev/null"; sleep 2; done
xdotool mousemove 1910 1070   # park the cursor outside the view
sleep 3

S0=$(simtime); T0=$(date +%s.%N)
ffmpeg -hide_banner -loglevel error -y -f x11grab -framerate 30 -video_size 1404x790 -i :0.0+86,216 \
  -c:v libx264 -preset ultrafast -crf 18 -pix_fmt yuv420p "$OUT/$NAME.raw.mp4" &
FF=$!
dex "timeout -k 2 $((DUR * 3 + 30)) python3 /root/barq_ws/src/diagnostics/sim_walk_metric.py --vx $VX --duration $DUR" \
  2>&1 | grep -E '^(WALK|ATT|TORQ)' | tee "$OUT/$NAME.txt"
S1=$(simtime); T1=$(date +%s.%N)
kill -INT $FF; wait $FF 2>/dev/null || true
RTF=$(python3 -c "r=($S1-$S0)/($T1-$T0); assert 0.05<r<1.5, r; print(round(r,4))")
echo "real-time factor during capture: $RTF" | tee -a "$OUT/$NAME.txt"
FONT=/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf
LBL=$(printf "%s" "$LABEL" | sed "s/[:\\']/\\\\&/g")   # drawtext: escape : \ '
ffmpeg -hide_banner -loglevel error -y -i "$OUT/$NAME.raw.mp4" \
  -vf "setpts=PTS*$RTF,fps=30,scale=1280:720,drawtext=fontfile=$FONT:text='$LBL':x=24:y=20:fontsize=34:fontcolor=white:box=1:boxcolor=black@0.55:boxborderw=12,drawtext=fontfile=$FONT:text='real speed (sim RTF $RTF corrected)':x=24:y=h-50:fontsize=20:fontcolor=white:box=1:boxcolor=black@0.45:boxborderw=8" \
  -c:v libx264 -preset medium -crf 20 -pix_fmt yuv420p -movflags +faststart "$OUT/$NAME.mp4"
# raw capture kept (re-label / re-cut without re-recording)
echo "wrote $OUT/$NAME.mp4"
