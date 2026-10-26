# Verification status

Date: 2026-10-21 (Asia/Shanghai).
Host: Ubuntu 20.04 x86_64, system Python 3.8.10, ROS Noetic only.
No ROS 2, Unitree SDK or system dependencies were installed.

## Root command reference, 2026-10-26

- Added COMMANDLIST.md at the repository root, linked prominently from both
  README entry points and DEPLOY_GO2.md. It covers acquisition/version checks,
  read-only inspection, ordinary dependencies, CMake, both builds, tests,
  configuration, conditional uslam commands, runtime/RViz/goals/enable/cancel,
  optional offline conversion, troubleshooting and Git updates.
- All 48 Bash code blocks passed bash -n; internal section anchors, relative
  document links and referenced entry-script paths were checked. The documented
  Python command ran: 81 tests passed with the existing visualization warning.
  The exact documented standalone C++ block compiled and passed both A* and
  tracking assertions. Converter/inspector CLI parameter names were checked.
- Height input is explicit; the new launch examples do not supply an assumed
  0.4 m body height. The document distinguishes the physical body-height offset
  from odom z and from the offline tomogram ground-height parameter.
- No apt/pip dependency installation, Foxy build, uslam command or robot-control
  command was executed for this documentation change. The optional Python 3.8
  offline conversion environment installation was documented, not installed here.

## Unified input collection, 2026-10-26

- Added standalone inspect_navigation.py, which runs before any adapter build.
  It records odom/cloud summaries, observed receive rates, timestamp ages,
  publisher/subscriber QoS, TF edges and latest/message-time transform queries.
  It creates no control publisher, calls no business services and starts no SLAM.
- Candidate full/incremental-map semantics, the physical odom reference point,
  body_height, loop closure and lease access remain explicitly unverified.
  Observed topic types alone do not imply a publisher exists; endpoints and
  received counts are recorded separately.
- Python 3.8 compilation, --help, invalid-duration and missing-rclpy handling
  passed. Three temporary fake-ROS transport scenarios passed: normal samples
  with a retained map and partial TF, no input messages, and missing TF. They
  also covered paths with spaces and nonfinite odometry serialization. These
  checks do not validate real Foxy/DDS reception or TF correctness on the robot.
- Checked the ROS 2 Foxy rclpy Node and tf2_ros Buffer source APIs. No ROS was
  installed and no navigation, SLAM or motion command was executed on Go2.

## Deployment commands and environment helper, 2026-10-26

- Expanded DEPLOY_GO2.md with existing-Foxy prerequisites, dependency/planner/
  adapter build commands and logs, per-terminal setup, conditional uslam client
  commands, actual input/TF checks, RViz/CLI goals, preview, enable and cancellation.
- Added source-only env_on_go2.bash to locate the nested workspace and load both
  overlays. Temporary shell fixtures passed for paths with spaces, overlay order,
  custom/missing configuration, missing build outputs, wrong ROS distribution,
  failed sourcing, errexit and preservation of the caller's nounset setting.
- All 61 Bash blocks in the five entry documents passed bash -n (not executed).
  Relative document links and git diff --check passed.
- inspect_uslam.py ran on Python 3.8 with temporary configuration and fake ROS CLI
  fixtures, including the three candidate command interfaces. These are read-only
  collection checks, not real DDS or robot results; details are in USLAM_AUDIT.md.
- Checked the Foxy ros2topic implementation for --once/--keep-alive and OSQP 1.0.0
  sources for the minimum CMake version of 3.18. No dependency installer, ROS build,
  uslam command, navigation node or motion command was executed on the robot.
- The current planner still uses one planning_frame and transforms map snapshots
  into it. A separate global map/local odom architecture with explicit handling
  of loop-closure corrections remains unimplemented and unverified.

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
