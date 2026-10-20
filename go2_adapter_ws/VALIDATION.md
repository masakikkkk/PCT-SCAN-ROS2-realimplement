# Verification status

Date: 2026-10-19 (Asia/Shanghai)

Host: Ubuntu 20.04 x86_64, /usr/bin/python3 3.8.10, ROS Noetic only.
No ROS 2, SDK or system dependencies were installed.

## Interface package: actually run and passed

- `setup.py check --strict` (fixed missing URL metadata).
- `setup.py build --build-base /tmp/go2_pct_adapter_build_20261019`.
- `setup.py install --prefix /tmp/go2_pct_adapter_install_20261019
  --single-version-externally-managed --record ...`.
- Installation output includes state_adapter, sport_bridge, live_pct_bridge,
  preflight, ament package marker, package.xml, LICENSE, YAML and both launch files.
- 42 Python tests: 33 ROS-independent tests plus 9 fake-transport callback tests.
  Covers command conversion, readiness, preview isolation, invalid routes,
  cancellation, missing lease, watchdogs, timestamps and quaternion math.
- Native tracking_checks test compiled with g++ C++17, -Wall -Wextra -Werror
  and UndefinedBehaviorSanitizer; 15 assertions passed. This is a ROS-independent
  helper test, not a compilation of the complete ROS controller node.
- Python 3.8 syntax / byte-compilation; package XML, YAML and Bash syntax checks.

These are real Python build/install checks, not a successful Foxy colcon build.
Foxy's rclpy, generated messages and launch are absent on this host, so their
runtime imports are checked by build_on_go2.sh on the robot.

## Upstream source: reviewed / patched, not compiled here

Base PCT-SCAN-ROS2 commit: `1cf45a1c12ca785ce533293302be09d51daf0ccb`.

Local changes:

- build/runtime scripts select the existing Foxy/Humble environment.
- optional PCT_BUILD_TESTING flag for deployment builds.
- TF2 .h/.hpp include compatibility for SCAN controllers and kinematic simulator.
- Controller validation of cubic spline knots / finite points, odometry timeout,
  bounded trajectory lifetime, reference-path cancellation and preview time freeze.
- Navigation requires a valid PCT route and a live tracker heartbeat. New goals
  first disable motion; failed plans and cancellation cannot approve the old route.

Patch is included as `pct_scan_foxy.patch`. The helper has compiled standalone,
and diff/shell checks pass; the full C++ dependency graph has NOT been built in Foxy.

## Still requires Go2 evidence

- `bash build_on_go2.sh`: real colcon, node imports, Unitree message serialization.
- `ros2 run go2_pct_adapter preflight`: graph, samples, timestamp freshness, TF.
- Actual odom origin / twist convention and map alignment.
- Firmware-specific lease acquisition/renewal when required.
- Full PCT/SCAN build, own-scene tomogram, goal planning, tracking and physical motion.
