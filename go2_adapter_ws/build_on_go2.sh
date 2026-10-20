#!/usr/bin/env bash
# Build only this interface package in an EXISTING Go2 Foxy environment.
# Installs no dependencies and launches no robot-control nodes.
set -eo pipefail
ADAPTER_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
if [[ "${ROS_DISTRO:-}" != foxy ]]; then
  echo '[go2-adapter] 先加载原来能看到 Go2 数据的 Foxy / 宇树 DDS 环境。' >&2
  exit 2
fi
/usr/bin/python3 - <<'PY'
import sys
import rclpy
import tf2_ros
import launch_ros
import yaml
import colcon_core
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from unitree_api.msg import Request
from rclpy.serialization import serialize_message

request = Request()
request.header.identity.api_id = 1008
request.header.identity.id = 1
request.header.lease.id = 0
request.parameter = '{"x":0.0,"y":0.0,"z":0.0}'
serialize_message(request)
print('[go2-adapter] Existing Python / ROS / Unitree request schema: PASS', sys.version)
PY
cd "$ADAPTER_WS"
/usr/bin/python3 -c 'from colcon_core.command import main; raise SystemExit(main())' \
  build --symlink-install \
  --packages-select go2_pct_adapter --event-handlers console_direct+ "$@"
set +u
source "$ADAPTER_WS/install/setup.bash"
set -u
/usr/bin/python3 - <<'PY'
import importlib
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
for name in ('state_adapter', 'sport_bridge', 'preflight'):
    importlib.import_module('go2_pct_adapter.' + name)
share = Path(get_package_share_directory('go2_pct_adapter'))
for name in ('config/go2.yaml', 'launch/interfaces.launch.py', 'launch/navigation.launch.py'):
    if not (share / name).is_file():
        raise RuntimeError('Missing installed resource: ' + name)
print('[go2-adapter] Installed node imports / ament resources: PASS')
PY
ros2 pkg executables go2_pct_adapter
echo '[go2-adapter] Build complete. No robot-control nodes were launched.'
echo '[go2-adapter] Next: source install/setup.bash; ros2 run go2_pct_adapter preflight'
