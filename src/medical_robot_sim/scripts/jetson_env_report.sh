#!/usr/bin/env bash

# Day14: Jetson/Edge environment report
# - Must not fail on non-Jetson environments
# - Must always print arch=...

set -u

arch="$(uname -m 2>/dev/null || echo unknown)"
echo "arch=${arch}"

echo "kernel=$(uname -sr 2>/dev/null || echo unknown)"

echo "hostname=$(hostname 2>/dev/null || echo unknown)"

if [ -f /etc/os-release ]; then
  # shellcheck disable=SC1091
  . /etc/os-release
  echo "os_name=${NAME:-unknown}"
  echo "os_version=${VERSION_ID:-unknown}"
else
  echo "os_name=unknown"
  echo "os_version=unknown"
fi

# ROS distro (if present)
echo "ros_distro=${ROS_DISTRO:-unknown}"

# Python version (best-effort)
python3_version="$(python3 -c 'import sys; print("%d.%d.%d" % sys.version_info[:3])' 2>/dev/null || echo unknown)"
echo "python3=${python3_version}"

# Jetson detection: /etc/nv_tegra_release exists on Jetson (L4T)
if [ -f /etc/nv_tegra_release ]; then
  nv_rel_raw="$(cat /etc/nv_tegra_release 2>/dev/null | tr -d '\r' | head -n 1)"
  # Keep it one-line and safe for grep
  nv_rel="$(echo "${nv_rel_raw}" | tr '\t' ' ' | tr -s ' ')"
  echo "jetson_nv_tegra_release=${nv_rel}"
else
  echo "N/A: not a Jetson (/etc/nv_tegra_release missing)"
fi

exit 0
