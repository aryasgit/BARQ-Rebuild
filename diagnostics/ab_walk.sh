#!/usr/bin/env bash
# A/B walk benchmark on FRESH spawns (host-side; needs barq:dev + built workspace).
#   diagnostics/ab_walk.sh "label|<extra sim.launch args>" ...   [VX=0.15 DUR=10 REPS=1]
# Prints the WALK + ATT lines per config (sim_walk_metric.py), one fresh sim per run.
set -uo pipefail
RUN="$(dirname "$0")/../tools/barq_run.sh"
VX=${VX:-0.15}; DUR=${DUR:-10}; REPS=${REPS:-1}; GUI=${GUI:-false}
SRC='source /opt/ros/humble/setup.bash; source /root/barq_ws/install/setup.bash'
for spec in "$@"; do
  label=${spec%%|*}; extra=${spec#*|}; [ "$extra" = "$spec" ] && extra=""
  for r in $(seq 1 "$REPS"); do
    docker rm -f barq_ab >/dev/null 2>&1
    "$RUN" -n barq_ab ros2 launch barq_bringup sim.launch.py gait:=true gui:=$GUI $extra >/dev/null
    for _ in $(seq 1 60); do
      docker logs barq_ab 2>&1 | grep -q 'activated joint_group_position_controller' && break; sleep 2
    done
    sleep 4
    out=$(docker exec barq_ab bash -lc "$SRC; timeout -k 2 120 python3 \
      /root/barq_ws/src/diagnostics/sim_walk_metric.py --vx $VX --duration $DUR" 2>&1 | grep -E '^(WALK|ATT)')
    echo "== $label (rep $r) [$extra]"; echo "$out"
    docker rm -f barq_ab >/dev/null 2>&1
  done
done
