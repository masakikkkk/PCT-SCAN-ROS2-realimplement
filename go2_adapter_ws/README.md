# Go2 EDU 的 PCT / SCAN 接口

**所有执行命令统一入口：[仓库根目录 COMMANDLIST.md](../COMMANDLIST.md)。**
远端文件与文档一致性核查见 [DELIVERY_AUDIT.md](DELIVERY_AUDIT.md)。

部署步骤和实际目录见 [DEPLOY_GO2.md](DEPLOY_GO2.md)；在线累计地图原理见
[ONLINE_GO2.md](ONLINE_GO2.md)：默认使用 `/uslam/cloud_map`，
启动 `online_navigation.launch.py`，不需要预先生成 PCD 或 tomogram 文件。
以下原有 `navigation.launch.py` 示例默认是离线地图模式。

代码在现有 Foxy 环境中使用；本工作区不安装 ROS、SDK、SLAM，也不启动底层控制器。

完整部署、uslam 准备、RViz 目标和执行命令已统一写在 [DEPLOY_GO2.md](DEPLOY_GO2.md)。
完成构建后，每个新终端在完整仓库根目录 `source go2_adapter_ws/env_on_go2.bash`
加载两套工作区。uslam 参数及协议核查见 [USLAM_AUDIT.md](USLAM_AUDIT.md)。

本工作区位于完整仓库内部，算法源码在 `../src/`，构建脚本在 `../scripts/`。
仓库以原上游提交 `1cf45a1c12ca785ce533293302be09d51daf0ccb` 为基础加入实机接口和
在线地图功能；该哈希不是当前版本。上游要求 Humble；本地额外修正了构建/
运行脚本对 Humble 的硬编码，以及 SCAN 的 TF2 .h/.hpp 头文件兼容入口。这些是
源码层面的已知 Foxy 问题修正，不代表整个上游已经完成 Foxy 编译验证。

## 当前交付与验证边界

- `state_adapter`：选择一路已有机体里程计和一路实时点云，检查类型契约、坐标系、
  时间戳，组合雷达外参，输出 SCAN 私有输入。
- `sport_bridge`：将机体系 `Twist` 转为 `unitree_api/msg/Request`；支持预览、限速、
  使能、输入超时停止、外部运动租赁 ID。
- `live_pct_bridge`：收到目标时，以最新机体里程计计算 PCT 起点，避免沿用演示起点。
- `navigation.launch.py`：把上述接口、PCT、SCAN 和仓库现有闭环跟踪器连接起来。
  不启动模拟器，不重启你们的 SLAM，不写入宇树 `/uslam/navigation/global_path`。

当前主机只有 Noetic，未安装或修改 ROS 2。在线版本通过 Python 3.8 构建和 /tmp
前缀安装检查，五个入口及三个 launch 文件均已安装；81 项 Python 测试通过
（77 项接口/在线地图测试和 4 项桥接辅助测试）。原生 A* 的失败后重规划回归也已
独立编译运行。尚未运行 rclpy 节点、Foxy colcon 构建、CUDA 或实机运动验证。
详情及可复现命令见 [VALIDATION.md](VALIDATION.md)。

按步骤上机见 `DEPLOY_GO2.md`。`build_on_go2.sh` 在既有 Foxy 环境构建本包并检查真实
消息序列化及节点导入，不安装依赖、不启动控制；`preflight` 在 Go2 上只读检查传感器、
坐标系和 TF。

## 数据契约

默认值是待核对的配置模板，不声称是这只 Go2 的实际 frame_id / child_frame_id。
统一修改 `src/go2_pct_adapter/config/go2.yaml`。

| 输入 | 默认选择 | 类型 / 要求 |
| --- | --- | --- |
| 机体里程计 | `/utlidar/robot_odom` | `nav_msgs/msg/Odometry`，位置为机体参考点 |
| 实时点云 | `/utlidar/cloud_deskewed` | `sensor_msgs/msg/PointCloud2`，含 XYZ，不是累计全局地图 |
| 雷达外参 | 现有 TF 的 `base_frame -> sensor_frame` | 也可显式提供经过测量的 XYZ / xyzw |
| 导航目标 | `/pct_go2/goal` | `geometry_msgs/msg/PoseStamped`，与 tomogram 相同坐标系 |
| 跟踪速度 | `/pct_go2/cmd_vel` | `geometry_msgs/msg/Twist`，机体系 vx / vy / yaw_rate |

| 输出 | 含义 |
| --- | --- |
| `/pct_go2/body_pose` | SCAN 机体位姿；twist 转为规划坐标系速度 |
| `/pct_go2/sensor_pose` | 同一规划坐标系内的传感器位姿 |
| `/pct_go2/cloud` | 保留原始数据和时间戳的点云 |
| `/pct_go2/global_path` | PCT 输出的地表路径 |
| `/pct_go2/bspline` | SCAN 输出的局部轨迹 |
| `/pct_go2/sport_request_preview` | 实际序列化的运动请求预览 |
| `/pct_go2/bridge_status` | 使能状态、停止原因、租赁可用状态 |
| `/pct_go2/route_valid` | 当前目标的 PCT 路线是否有效 |
| `/pct_go2/tracking_valid` | 当前轨迹的有效性心跳 |
| `/pct_go2/execution_enabled` | 是否允许推进轨迹执行时间 |
| `/api/sport/request` | 使能后发送给 Go2 的高层运动请求 |

不要同时订阅所有 odom。选择与地图稳定对齐、确实表示机体位姿的一路。如果
`/uslam/localization/odom` 更适合，可更改 `odom_topic`。若该消息表示雷达/IMU 原点，
必须先用正确的外参变换成机体里程计，不能只改 `child_frame_id`。

`planning_frame` 必须等于输入里程计的 `header.frame_id`，`base_frame` 必须等于
`child_frame_id`。本接口会拒绝不一致输入，不伪造坐标变换。用于 PCT 的静态地图必须
与该规划坐标系对齐；重启或重定位导致 odom 原点改变时需要重新对齐。

标准 Odometry 的 twist 位于 child frame，配置 `odom_twist_frame: child`；若你们的
SLAM 明确发布世界坐标速度，则设置 `world`。SCAN 源码直接将该速度用作世界坐标
轨迹边界，所以 `/pct_go2/body_pose` 是针对该规划器的私有契约，不能替代通用里程计。
其 covariance 仅沿用源消息，本接口未变换 covariance；SCAN 不使用这些 covariance。

局部雷达点云设置 `cloud_is_world: false`，消息 frame 必须是 `sensor_frame`。
若是已经变换到规划坐标系的实时点云，则设为 `true`；消息 frame 必须是
`planning_frame`，但雷达射线原点仍由真实传感器位姿给出。不要把累计全局点云地图
送入这个实时避障入口。`grid_map`、`height_map`、`voxel_map` 的名字不表示它们可以
直接作为 SCAN 输入；本方案由实时点云构建 SCAN 自己的局部地图。

本适配器使用最新里程计样本估算传感器位姿，检查与点云的时间差（默认上限 0.10 s），
不做位姿插值，也不负责点云去畸变。上游 SCAN 的 sensor_pose / cloud 是独立回调，
并非严格同步。运动速度较高时应进一步做逐帧时间配对，不能把本适配器当作精确同步器。

如果没有可用 TF，设置 `use_tf_extrinsic: false`，填写测量的 `body_to_sensor_xyz` 和
`body_to_sensor_xyzw`，并设置 `extrinsics_confirmed: true`。默认零值不是 Go2 标定值。

## 在已有 Foxy 中构建接口包

先加载你们原来能够看到 Go2 数据的 Foxy / DDS / `unitree_api` 环境。不要更换 DDS
配置，也不要为了本接口重新安装 ROS。然后在本工作区执行：

```bash
cd ~/PCT-SCAN-ROS2/go2_adapter_ws
bash build_on_go2.sh
source install/setup.bash
ros2 run go2_pct_adapter preflight
ros2 launch go2_pct_adapter interfaces.launch.py
```

复制到机器人后，将 `cd` 路径换成复制后的路径。只构建这一个接口包不需要编译 PCT、
GTSAM、OSQP 或 SCAN，也不要求它们已经安装。rclpy、TF 和宇树消息包由现有 Foxy 提供。

可指定配置副本：

```bash
ros2 launch go2_pct_adapter interfaces.launch.py config:=/absolute/path/go2.yaml
```

`enable_motion: false` 时不会向 Sport 控制话题发送请求。可以查看输出：

```bash
ros2 topic echo /pct_go2/bridge_status
ros2 topic echo /pct_go2/sport_request_preview
```

在另一个已加载相同环境的终端，可给私有入口发送一条测试速度：

```bash
ros2 topic pub --rate 10 /pct_go2/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.1}, angular: {z: 0.0}}'
```

默认仍是预览。需要同时收到有效机体里程计和点云，否则预览为 StopMove。预览正常、
机器人运动模式及控制权已经确认后，可用下面的接口切换发送状态：

```bash
# 允许向 Sport 发送；需在此之后继续收到新的有效速度指令。
ros2 topic pub --once /pct_go2/motion_enable std_msgs/msg/Bool '{data: true}'
# 撤销发送，并在本节点曾接管时发送 StopMove。
ros2 topic pub --once /pct_go2/motion_enable std_msgs/msg/Bool '{data: false}'
```

若你们的 Foxy CLI 不支持 `--once`，使用 `--rate 1`，收到后 Ctrl+C；重复的 true
不会清除使能状态。使能是本接口的开关，不等于宇树平台授予控制权。本节点不会自动
站起、切换运控模式、禁用遥控器或关闭避障。

输入默认限速为 0.2 m/s、横向 0.1 m/s、转向 0.3 rad/s。默认速度超时 0.25 s、
里程计超时 0.25 s、点云超时 0.5 s，使用单调时钟判定到达间隔并检查传感器 header
时间戳。非有限值或不支持的 linear.z / angular.x / angular.y 会使旧速度失效。
桥进程崩溃或网络断开后的停车需要底层自身的 watchdog；退出时发送停止仅是尽力而为。
遥控器接管请沿用现有 SDK/控制权管理，并把接管事件接到 `motion_enable=false`。

## 租赁与控制协议

官方 ROS 2 `Move` 请求为 `api_id=1008`，`parameter={"x": vx, "y": vy, "z": yaw_rate}`；
`StopMove` 为 `api_id=1003`。JSON 的 z 不是爬升速度。

`sport_lease` 话题存在，不足以证明此固件下所有 Move 都必须租赁。官方 SDK2 的
SportClient 构造参数默认 `enableLease=false`，但应以你们机器的 SDK/固件为准。
若实际需要租赁，把 `require_lease` 改为 true，由你们的 SDK 控制权管理节点获取并
续租，把有效 ID 以 `std_msgs/msg/Int64` 送到 `/pct_go2/lease_id`，刷新周期小于
`lease_timeout_sec`。桥把它写入 Request.header.lease.id，缺失/过期时只尝试 StopMove。
仅重复一个已过期的 ID 不会完成续租，外部管理节点必须检查 SDK 续租成功。

这里没有实现猜测的租赁申请/续约消息，也不会抢占现有控制权。若固件确实要求租赁，
这一外部 SDK 对接是上实机前仍需完成的一步；错误或失效的租赁也可能导致停止请求
被拒绝。先用 `inspect_go2.sh` 记录实际类型，再接你们提供的 SDK。

协议参考：
- https://github.com/unitreerobotics/unitree_ros2/blob/master/example/src/src/common/ros2_sport_client.cpp
- https://github.com/unitreerobotics/unitree_ros2/blob/master/cyclonedds_ws/src/unitree/unitree_api/msg/RequestHeader.msg
- https://github.com/unitreerobotics/unitree_sdk2/blob/main/include/unitree/robot/go2/sport/sport_client.hpp

## 接入完整 PCT / SCAN

在线导航按 [DEPLOY_GO2.md](DEPLOY_GO2.md) 操作。下面只演示已有离线 tomogram 的
入口。在完整仓库根目录，先加载算法工作区，再加载仓库内的接口工作区：

```bash
cd ~/PCT-SCAN-ROS2
source go2_adapter_ws/env_on_go2.bash
read -r -p '输入实际场地 pickle 绝对路径：' GO2_TOMOGRAM_PATH
read -r -p '输入确认后的 body_height（米）：' GO2_BODY_HEIGHT
ros2 launch go2_pct_adapter navigation.launch.py \
  config:="$GO2_NAV_CONFIG" tomogram_path:="${GO2_TOMOGRAM_PATH:?}" \
  body_height:="${GO2_BODY_HEIGHT:?}"
```

不要在不同 ROS 发行版之间混用 setup.bash。本地修正后的上游 build_workspace.sh /
_pct_scan_env.sh 支持现有 `ROS_DISTRO` 或显式 `PCT_ROS_DISTRO=foxy`；TF2 头文件入口
也已兼容 Foxy 的 .h。上游的其他 C++ 与依赖仍需在现有 Foxy 上实编验证；脚本不会
自动安装 ROS。本次没有在当前主机运行完整算法编译。

`body_height` 是机体参考点离地高度，需要按真实模型设置，不是 odom 的 z。
launch 源码仍保留上游默认值 0.4，但上面的部署命令会显式传入确认后的实际值。
PCT 起点使用机体 z 减去这个高度，目标 z 表示目标地表高度；SCAN 再加回相同高度。
这种偏移用于初步接口接入，并不等于楼梯上的精确地面估计。

目标为 `/pct_go2/goal`，`PoseStamped.header.frame_id` 必须与规划地图一致；
通过它触发规划时会使用最新实机位置作为起点。本入口禁用了 RViz 演示自动起点和
交互标记。每个新目标先撤销运动使能并清空旧路线；规划失败不会批准旧路线继续执行。
成功后仍需查看预览并重新使能。本节的离线入口需要提前生成 tomogram 文件；
`online_navigation.launch.py` 则在运行时将累计点云转换为 tomogram，无需提前生成文件。

本地跟踪器补充了里程计超时、三次样条结构校验及轨迹有效期检查。有效期按源时间戳
和轨迹时长计算，使用单调时钟监测接收间隔。navigation.launch.py 强制同时检查路线
和跟踪心跳；跟踪进程退出、心跳超时或轨迹失效时，桥不会继续发旧速度。
预览时轨迹执行时间冻结，但不会无限延长轨迹有效期；预览过久后需重新发送目标。
发布 std_msgs/msg/Empty 到 `/pct_go2/cancel` 会撤销目标并停止发送运动。
这些改动仍需在完整 Foxy 链路验证；本次未证明楼梯轨迹能被现有步态执行。

## 只读采集实际接口

在 Go2 的现有 Foxy/宇树终端，进入完整仓库后执行，不需要先编译接口包：

```bash
cd ~/PCT-SCAN-ROS2
/usr/bin/python3 go2_adapter_ws/inspect_navigation.py --output go2_navigation_report.json
```

路径不同就替换 `cd`。默认采集 20 秒，报告包含候选 odom/点云/地图的类型、
发布者 QoS、frame、首末消息摘要、接收频率、时间戳、TF 边及实际时间变换查询，
以及 Sport/lease 接口的类型和节点信息。缺少话题或 TF 会记录在报告中。
频率是本机观察值；只在固定窗口采样，未收到不等于系统不具备该功能。
对发现的全为 TRANSIENT_LOCAL 的点云发布者，会使用同样的持久性接收历史地图；
若发布者在采集过程中才出现，其历史消息可能无法收到，可在其运行后重新采集。

脚本只创建订阅并读取 ROS 图，不发布控制指令、不启动 SLAM，也不修改配置。
`body_height` 不会自动猜测：仍需确认 odom 的物理参考点，并测量其到脚下地面的
高度差。点云点数不能证明消息是完整地图，TF 存在不能证明回环修正生效，lease
话题存在也不能证明固件强制要求租赁。

若 uslam 尚未运行，先保留一次报告；用你们原有 client 启动建图后再采一次：

```bash
/usr/bin/python3 go2_adapter_ws/inspect_navigation.py --output go2_navigation_report_uslam.json
```

需要同时查 uslam 安装配置时，额外运行以下已有脚本；在扩展电脑上找不到机身主机
的文件不代表没有配置。详情见 [USLAM_AUDIT.md](USLAM_AUDIT.md)。

```bash
/usr/bin/python3 go2_adapter_ws/inspect_uslam.py > uslam_inventory.txt 2>&1
```

原 `inspect_go2.sh` 仍可用于单独导出消息定义；上述采集报告不代替部署时的 preflight。

## 本地检查（不需要 ROS）

从完整仓库根目录运行：

```bash
PYTHONNOUSERSITE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
PYTHONPATH=go2_adapter_ws/src/go2_pct_adapter:src/pct/tomography:src/pct/pct_planner:src/integration/pct_scan_bridge \
  /usr/bin/python3 -m pytest -q \
  go2_adapter_ws/src/go2_pct_adapter/test src/integration/pct_scan_bridge/test
```

live_pct_bridge 继承 GPL 的 PCT 桥，本接口包整体按 GPL-2.0-or-later 标注，原项目
及第三方授权见各仓库许可证。
