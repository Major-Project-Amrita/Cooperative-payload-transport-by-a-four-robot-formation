#!/bin/bash
# Real ROS 2 (Humble) lockstep validation of the proposed PA-VFH, the learned
# DAgger policy and MPC: plant node + controller node as two ROS 2 processes,
# each compared with the in-process run. Run on a machine with ROS 2 Humble.
#   bash tools/ros_humble_validation.sh 2>&1 | tee results/ros_humble_validation.log
set -u
cd "$(dirname "$0")/.."
for cfg in "mpc on_top false false truth" "nn on_top false false truth" \
           "pavfh on_top false false truth" "pavfh_nonn on_top false false truth" \
           "pavfh on_top false true learned" "nn on_top false true learned" \
           "pavfh virtual false true sensor"; do
  set -- $cfg
  echo "=== $1 payload=$2 phase=$4 gate_source=$5"
  bash tools/ros_lockstep_test.sh "$@" 2>&1 | grep -v -i warn | tail -11
done
