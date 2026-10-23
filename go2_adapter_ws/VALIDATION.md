# Verification status

Date: 2026-10-21 (Asia/Shanghai).
Host: Ubuntu 20.04 x86_64, system Python 3.8.10, ROS Noetic only.
No ROS 2, Unitree SDK or system dependencies were installed.

## Deployment documentation correction, 2026-10-23

- Confirmed that the Git repository is the full `PCT-SCAN-ROS2/` directory.
  Its root `src/` contains the algorithms; its nested `go2_adapter_ws/src/`
  contains the adapter. The sibling standalone adapter is a historical copy.
- Standardized deployment docs on the nested workspace, removed the instruction
  to copy it outside the repository, and separated online/offline map startup.
- Checked that all 11 documented source/script/config entry paths exist and
  that relative Markdown links resolve. All 45 Bash blocks across the four
  entry documents passed `bash -n`; the commands were not executed.
- No algorithm or adapter source changes, ROS installation, Foxy compilation
  or robot execution were part of this documentation correction.

## Actually run for the online-map change

- 81 Python tests passed: 77 adapter/online tests plus 4 existing bridge helper
  tests. Includes real CPU tomography on synthetic room geometry, unknown
  holes/body-radius buffers, missing-ceiling policy, CUDA-unavailable CPU fallback,
  map limits, padded/big-endian PointCloud2 decoding, rigid transforms, in-memory
  loading and both existing Building/Plaza pickle formats.
- Callback tests use fake ROS transport. They cover stale/superseded worker
  results, invalid maps, cancellation during a build, pre-map and unobserved
  goals, no route, goal completion and replanning without automatically enabling
  motion. Launch tests use fake launch objects, not ROS launch.
- One pre-existing NumPy warning from the visualization helper's NaN comparison
  was emitted; all tests passed.
- Compiled the real native PCT A* and its standalone online-replan regression
  using C++17, Eigen, -Wall -Wextra and UndefinedBehaviorSanitizer. All five
  assertions passed, including a reachable request after a failed search.
- Python 3.8 byte-compilation of the adapter, PCT Python package, tomography
  package and offline converter passed. The old demo's Python 3.10 match
  statement was converted to if/elif for Foxy/Python 3.8 compatibility.
- Adapter setup.py strict check, build and actual installation under
  /tmp/go2_online_install_20261021 passed. All five executable entries, three
  launch files, ament marker, XML, YAML and LICENSE were checked.
- Tomography build and installation under /tmp/pct_tomography_install_20261021
  included the shared CPU backend.
- Offline converter --help, XML/YAML parsing, Bash syntax and git diff checks
  passed. CPU equations were extracted from the existing offline converter
  into tomography.cpu_backend, shared by both entry points.

Reproduce from the repository root in the same non-ROS test environment:

```bash
PYTHONNOUSERSITE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
PYTHONPATH=go2_adapter_ws/src/go2_pct_adapter:src/pct/tomography:src/pct/pct_planner:src/integration/pct_scan_bridge \
  /usr/bin/python3 -m pytest -q \
  go2_adapter_ws/src/go2_pct_adapter/test src/integration/pct_scan_bridge/test

g++ -std=c++17 -O2 -Wall -Wextra -fsanitize=undefined \
  -I/usr/include/eigen3 -Isrc/pct/pct_planner/lib/src \
  src/pct/pct_planner/lib/test/test_online_replan.cc \
  src/pct/pct_planner/lib/src/a_star/a_star_search.cc \
  -o /tmp/pct_online_replan_test
/tmp/pct_online_replan_test
```

## Previous interface checks

On 2026-10-19 the original 42 adapter tests, Python build/install and native
tracking-check helper (15 standalone C++ assertions) passed. That helper did not
compile the complete ROS controller. These interface tests remain in the suite.

Base upstream commit: 1cf45a1c12ca785ce533293302be09d51daf0ccb.
pct_scan_foxy.patch is the historical 2026-10-19 compatibility patch, not a
standalone patch for this online-map revision. Use the full Git repository for
the online version; do not reapply the historical patch to these sources.

## Not verified here

- Full Foxy colcon/native dependency build; real rclpy/launch imports, TF lookup,
  DDS/QoS delivery and Unitree message serialization.
- CUDA/CuPy execution or CPU timing/memory usage on the Go2 expansion computer.
- Actual /uslam/cloud_map type, accumulated contents, frame and update stamps;
  actual odom origin/twist convention, TF alignment and SLAM loop-closure behavior.
- PCT terrain traversability versus SCAN collision-model consistency, including
  stairs and overhangs; firmware-specific leases and physical motion.
- Frontier exploration or navigation through unobserved regions. Such goals
  are preserved while waiting for later maps; the node does not explore.

Deployment entry: [ONLINE_GO2.md](ONLINE_GO2.md). Existing preview and motion
watchdogs are retained. These checks are not a claim of Go2 deployment readiness.
