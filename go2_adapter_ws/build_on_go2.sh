#!/usr/bin/env bash
# Build the interfaces, C++ map node and generated messages in EXISTING Foxy.
# Installs no dependencies and launches no robot-control nodes.
set -eo pipefail
ADAPTER_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
GO2_BUILD_MAP_ONLY=0
GO2_BUILD_TARGET=go2_pct_adapter
if [[ "${1:-}" == --map-only ]]; then
  GO2_BUILD_MAP_ONLY=1
  GO2_BUILD_TARGET=go2_map_maintenance
  shift
fi
if [[ "${ROS_DISTRO:-}" != foxy ]]; then
  echo '[go2-adapter] 先加载原来能看到 Go2 数据的 Foxy / 宇树 DDS 环境。' >&2
  exit 2
fi
/usr/bin/python3 - "$GO2_BUILD_MAP_ONLY" <<'PY'
import sys
import rclpy
import colcon_core
if sys.argv[1] == '0':
    import tf2_ros
    import launch_ros
    import yaml
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
    print('[go2-adapter] Existing Unitree request schema: PASS')
print('[go2-adapter] Existing Python / ROS build imports: PASS', sys.version)
PY
cd "$ADAPTER_WS"
/usr/bin/python3 -c 'from colcon_core.command import main; raise SystemExit(main())' \
  build --symlink-install \
  --packages-up-to "$GO2_BUILD_TARGET" --event-handlers console_direct+ \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
  -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3 "$@"
set +u
source "$ADAPTER_WS/install/setup.bash"
set -u
/usr/bin/python3 - "$GO2_BUILD_MAP_ONLY" <<'PY'
import sys
import importlib
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
from go2_map_maintenance.msg import MapSnapshot
from rclpy.serialization import serialize_message
serialize_message(MapSnapshot())
if sys.argv[1] == '0':
    for name in ('state_adapter', 'sport_bridge', 'preflight'):
        importlib.import_module('go2_pct_adapter.' + name)
    share = Path(get_package_share_directory('go2_pct_adapter'))
    for name in ('config/go2.yaml', 'launch/interfaces.launch.py', 'launch/navigation.launch.py',
                 'launch/online_navigation.launch.py'):
        if not (share / name).is_file():
            raise RuntimeError('Missing installed resource: ' + name)
print('[go2-adapter] Installed node imports / ament resources: PASS')
PY
if [[ "$GO2_BUILD_MAP_ONLY" == 0 ]]; then ros2 pkg executables go2_pct_adapter; fi
ros2 pkg executables go2_map_maintenance
echo '[go2-adapter] Build complete. No robot-control nodes were launched.'
if [[ "$GO2_BUILD_MAP_ONLY" == 1 ]]; then
  echo '[go2-adapter] Map-only build; see COMMANDLIST.md section 0.1.'
else
  echo '[go2-adapter] Next: source install/setup.bash; ros2 run go2_pct_adapter preflight'
fi
