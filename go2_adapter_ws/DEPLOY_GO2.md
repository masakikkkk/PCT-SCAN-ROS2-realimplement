# Go2 上的部署顺序

**在线地图版本：** 直接复用 `/uslam/cloud_map` 的最新步骤见
[ONLINE_GO2.md](ONLINE_GO2.md)。下面保留离线地图与接口单独检查流程。

使用你们已经正常工作的 Foxy、SLAM 和宇树 DDS 环境，不安装 ROS。

## 1. 复制并构建接口包

建议将源码解压到 `~/go2_nav/`，其下有 `go2_adapter_ws/` 和 `PCT-SCAN-ROS2/`。
只验接口时只需前一个目录。

打开终端，按照你们原来的方法加载 Foxy、宇树消息包和 DDS，然后：

```bash
echo "$ROS_DISTRO"
ros2 topic list -t
cd ~/go2_nav/go2_adapter_ws
bash build_on_go2.sh
source install/setup.bash
```

必须显示 foxy，并能看到原来的 odom、点云和 Sport 话题。构建脚本使用系统 Python，
检查 rclpy、TF、unitree_api 导入与真实消息序列化，运行 colcon，再检查安装结果。
成功末尾显示 `Build complete`。脚本不安装依赖、不启动控制节点。
若缺少消息包，先 source 你们原来的宇树工作区；不要混用 Conda Python 或 Noetic。

## 2. 核对配置与输入

配置文件：`src/go2_pct_adapter/config/go2.yaml`。默认选 `/utlidar/robot_odom` 和
`/utlidar/cloud_deskewed`，frame 名和外参是待核对模板。先做只读检查：

```bash
ros2 run go2_pct_adapter preflight \
  --config "$PWD/src/go2_pct_adapter/config/go2.yaml"
```

报告会列出实际消息类型、odom 的 frame_id / child_frame_id、点云 frame、时间戳和
base-to-sensor TF。根据报告修改配置，再检查至 `result: PASS`。

| 字段 | 如何设置 |
| --- | --- |
| `odom_topic` | 一路稳定的机体里程计，不要把雷达/IMU 原点当成机体 |
| `planning_frame` | 实际 odom.header.frame_id，两个节点中必须相同 |
| `base_frame` | 实际机体 odom.child_frame_id，两个节点中必须相同 |
| `sensor_frame` | 真实雷达坐标系；局部雷达点云的 header.frame_id |
| `cloud_is_world` | 实时点云已在规划坐标系时 true，否则 false |
| `odom_twist_frame` | 标准 child 坐标速度用 child，确认世界速度才用 world |
| `use_tf_extrinsic` | 默认 true，要求现有 base->sensor TF |

缺少 TF 时可填写测量的 `body_to_sensor_xyz` 和 `body_to_sensor_xyzw`，设置
`use_tf_extrinsic: false`、`extrinsics_confirmed: true`。默认零值不是 Go2 标定值。
若 odom 原点实际是雷达/IMU，需先转换为机体里程计，不能只改 frame 名字。

preflight 不证明控制权、地图对齐或运动执行成功。保持 `enable_motion: false`。

## 3. 启动接口，查看请求预览

```bash
ros2 launch go2_pct_adapter interfaces.launch.py \
  config:="$PWD/src/go2_pct_adapter/config/go2.yaml"
```

新终端也需加载相同 Foxy/宇树环境并 source 本工作区，分别查看：

```bash
ros2 topic hz /pct_go2/body_pose
ros2 topic hz /pct_go2/cloud
ros2 topic echo /pct_go2/bridge_status
```

这些命令各自长期运行，查看后 Ctrl+C。无速度输入时 `no_valid_command` 正常。
独立验证速度映射时，在另一终端运行：

```bash
ros2 topic pub --rate 10 /pct_go2/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.1}, angular: {z: 0.0}}'
```

再查看 `/pct_go2/sport_request_preview`，应为 api_id=1008，parameter 中 x=0.1、
y=0、z=0，状态 motion_enabled=false。本桥尚不向实际 Sport 话题发送请求。
停止测试速度发布器，约 0.25 s 后预览应转为 api_id=1003。
本步骤仅验证接口，不代表规划器已经启动。

## 4. 构建并连接完整规划器

完整算法仍需在 Foxy 实编。源码完整包内已带 Foxy 脚本和 TF2 头文件修正；若另从
GitHub clone 原版，则需应用随接口包附带的 `pct_scan_foxy.patch`，不要重复应用。

先准备上游 GTSAM 4.2、OSQP 1.0 等依赖。上游
`scripts/setup_pct_dependencies.sh` 会联网下载并在项目 `.deps` 内构建依赖，
不负责安装 ROS。依赖齐备后，在现有 Foxy 环境中运行：

```bash
cd ~/go2_nav/PCT-SCAN-ROS2
PCT_ROS_DISTRO=foxy PCT_BUILD_TESTING=OFF bash scripts/build_workspace.sh \
  --packages-up-to pct_scan_bridge scan_planner
source scripts/_pct_scan_env.sh
source ~/go2_nav/go2_adapter_ws/install/setup.bash
```

这是部署构建入口，不是已经通过的整栈 Foxy 构建。若仍有 C++ 或依赖错误，保留完整
报错；不要通过混入 Humble 环境解决。现有全局点云还要生成你们场地的 tomogram，
并与定位坐标系对齐；仓库 Building/Plaza 地图不能用于真实场地导航。

结束第 3 步的接口启动及测试速度发布器，避免重复节点，然后一次启动完整链路：

```bash
ros2 launch go2_pct_adapter navigation.launch.py \
  config:=/home/YOUR_USER/go2_nav/go2_adapter_ws/src/go2_pct_adapter/config/go2.yaml \
  tomogram_path:=/absolute/path/your_environment.pickle body_height:=0.4
```

换成实际路径，`body_height` 换成机体参考点的实际离地高度。目标输入
`/pct_go2/goal` 是 PoseStamped，frame 与地图一致，z 是目标地面高度。可将 RViz
目标工具的话题设置为 `/pct_go2/goal`，Fixed Frame 设为 planning_frame。

依次观察 `/pct_go2/global_path`、`/pct_go2/bspline`、`/pct_go2/cmd_vel` 和
`/pct_go2/sport_request_preview`，确认路线对齐且有连续输出。默认仍不发运动请求。

新版本还要求 `/pct_go2/route_valid` 为 true、`/pct_go2/tracking_valid` 持续为 true。
没有有效路线或跟踪心跳时，bridge_status 会显示 route_unavailable、tracking_unavailable
或 tracking_timeout，并阻止旧速度继续发送。预览不推进轨迹执行时间，但轨迹仍有
有效期；预览过久出现失效时，重新发送目标，不要增大超时来掩盖问题。

每次发送 `/pct_go2/goal` 都会先撤销使能，包括目标无效或规划失败的情况。确认新路线
正常后，需要再次发送 motion_enable=true。这是逐目标批准执行的流程。

## 5. 最后启用真实速度发送

保留宇树自带运控。若固件要求租赁，先接现有 SDK 的申请/续租管理，配置
`require_lease: true`，持续提供有效 `/pct_go2/lease_id`。本包不猜测租赁申请协议。

在已有正常站立、运控模式和控制权时发送：

```bash
ros2 topic pub --once /pct_go2/motion_enable std_msgs/msg/Bool '{data: true}'
```

这一步开始发送实际 Sport 请求，先验证短距离平地目标。不要保留第 3 步的测试
速度发布器。撤销发送：

```bash
ros2 topic pub --once /pct_go2/motion_enable std_msgs/msg/Bool '{data: false}'
```

老版 CLI 若不支持 --once，用 --rate 1，收到后 Ctrl+C。使能后需新收到有效速度才
执行；速度或状态超时会尝试 StopMove。进程崩溃、断网后的停车及遥控接管，仍需底层
SDK/控制器保障。上下楼需另行验证步态、地形和跟踪能力。

完整取消当前目标（同时清空旧路线、撤销使能）：

```bash
ros2 topic pub --once /pct_go2/cancel std_msgs/msg/Empty '{}'
```

取消后需要新目标及再次使能；单独发送 motion_enable=true 不会恢复已取消的目标。
