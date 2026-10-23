# Go2 实机部署：使用完整仓库

本说明统一使用“完整仓库内包含接口工作区”的布局。沿用 Go2 已有 Foxy、
宇树消息/DDS、SLAM 和 Sport 运控，不安装 ROS，不启动另一套 SLAM。

## 1. 先分清目录

完整源码应是下面这个结构；仓库名字和存放位置可以变化，内部结构不变：

```text
PCT-SCAN-ROS2/                       ← 完整仓库根目录，GO2_NAV_ROOT 指这里
├── scripts/
│   ├── setup_pct_dependencies.sh   ← 构建 GTSAM / OSQP
│   ├── build_workspace.sh          ← 构建 PCT / SCAN / tomography
│   └── _pct_scan_env.sh            ← 加载算法运行环境
├── src/                            ← 算法源码
│   ├── pct/{pct_planner,tomography}/
│   ├── integration/pct_scan_bridge/
│   └── planner/plan_manage/        ← scan_planner 包
└── go2_adapter_ws/                 ← 仓库内的接口工作区
    ├── DEPLOY_GO2.md
    ├── build_on_go2.sh             ← 只构建接口包
    └── src/go2_pct_adapter/
        ├── config/go2.yaml
        └── launch/online_navigation.launch.py
```

`go2_adapter_ws/src` 只有接口源码，不能代替根目录的算法 `src`。
**只传 `go2_adapter_ws` 到扩展电脑，无法启动完整导航。** 请传整个仓库，或在扩展
电脑上克隆自己的 Fork。不要再把仓库内的接口工作区复制到外面。

2026-10-23 在开发电脑核对到的最新源码是：
`/home/yeyu/go2 planner/PCT-SCAN-ROS2`。
同级 `/home/yeyu/go2 planner/go2_adapter_ws` 是旧的独立接口副本；
`*_20261019.tar.gz` 及当时的无日期压缩包也不含最新在线版本。
这是开发电脑的核查结果，不代表已读取 Go2 文件系统。

若扩展电脑尚无完整仓库，在选好的父目录执行一次：

```bash
git clone https://github.com/masakikkkk/PCT-SCAN-ROS2-realimplement.git PCT-SCAN-ROS2
cd PCT-SCAN-ROS2
```

已有仓库就直接 `cd` 到它，不必重新克隆。开发电脑的对应命令是：

```bash
cd '/home/yeyu/go2 planner/PCT-SCAN-ROS2'
```

上述两种方式选一种。进入完整仓库根目录后，记录位置并检查必要文件：

```bash
export GO2_NAV_ROOT="$PWD"
ls "$GO2_NAV_ROOT/scripts/build_workspace.sh" \
   "$GO2_NAV_ROOT/src/pct/pct_planner/package.xml" \
   "$GO2_NAV_ROOT/src/planner/plan_manage/package.xml" \
   "$GO2_NAV_ROOT/go2_adapter_ws/src/go2_pct_adapter/package.xml"
```

四个文件都应存在。后续命令使用这个变量，不再假定 `~/go2_nav` 等另一套路径。
路径中有空格时保留双引号。

## 2. 在扩展电脑上构建一次

先按你们原来的方法加载能够看到 Go2 数据的 Foxy、宇树消息包和 DDS 环境：

```bash
echo "$ROS_DISTRO"
ros2 topic list -t
```

应为 `foxy`，并能看到已有 odom、点云和 Sport 话题。当前开发电脑只有 Noetic，
不在开发电脑执行以下 Foxy 构建。不要混入 Noetic、其他 ROS 2 版本或 Conda 环境。

算法还需要 GTSAM 4.2、OSQP 1.0，以及各包声明的构建/运行依赖。
若尚未在仓库默认 `.deps/install` 下准备前两者，可运行：

```bash
bash "$GO2_NAV_ROOT/scripts/setup_pct_dependencies.sh"
```

这一步会联网下载并编译 GTSAM/OSQP，不安装 ROS；脚本需要已有 git、CMake、Ninja、
C++ 编译器和相关开发库，不能替代系统依赖准备。若已经按本脚本构建完成，可跳过。
随后分别构建算法和接口：

```bash
PCT_ROS_DISTRO=foxy PCT_BUILD_TESTING=OFF \
  bash "$GO2_NAV_ROOT/scripts/build_workspace.sh" \
  --packages-up-to pct_scan_bridge scan_planner tomography
source "$GO2_NAV_ROOT/scripts/_pct_scan_env.sh"
bash "$GO2_NAV_ROOT/go2_adapter_ws/build_on_go2.sh"
source "$GO2_NAV_ROOT/go2_adapter_ws/install/setup.bash"
```

每条命令成功后再执行下一条。第一步构建结果在根目录 `install/`，接口构建结果在
`go2_adapter_ws/install/`，两者都要加载。在线入口必须包含 `tomography` 包。
`build_on_go2.sh` 只构建接口，不能单独完成 PCT/SCAN 构建。
现有源码已包含 Foxy 兼容修改，无需再应用历史 `pct_scan_foxy.patch`。

完整 Foxy 构建尚未在这台开发电脑上验证；若遇到缺依赖或 C++ 编译错误，保留实际
报错处理，不能将本说明视为已经通过 Go2 编译的记录。

## 3. 加载环境，检查已有数据

**每开一个终端**，先加载原来的 Foxy/宇树/DDS 环境，再进入扩展电脑上的完整仓库
根目录，执行下面这段。无需重新编译：

```bash
export GO2_NAV_ROOT="$PWD"
source "$GO2_NAV_ROOT/scripts/_pct_scan_env.sh"
source "$GO2_NAV_ROOT/go2_adapter_ws/install/setup.bash"
export GO2_NAV_CONFIG="$GO2_NAV_ROOT/go2_adapter_ws/src/go2_pct_adapter/config/go2.yaml"
```

`$PWD` 必须是完整仓库根目录，不能是 `go2_adapter_ws` 或它下面的 `src`。
先运行只读检查：

```bash
ros2 run go2_pct_adapter preflight --config "$GO2_NAV_CONFIG"
```

默认复用 `/utlidar/robot_odom`、`/utlidar/cloud_deskewed` 和已有 TF。
检查消息类型、frame 和时间戳是否与配置一致；已有 TF 正确时直接复用，不需要重新
标定雷达。若报告不一致，修改上面这份 `go2.yaml`，不要只改消息 frame 名字。
通过接口检查不等于累计地图或运动验证通过。

在线累计地图默认 `/uslam/cloud_map`：它需要是持续更新、时间戳有效、每条包含累计
XYZ 地图的 `sensor_msgs/msg/PointCloud2`。话题名字本身不能证明这些条件满足。
地图坐标系不同于规划坐标系时，使用现有 TF 变换。参数含义及状态见
[ONLINE_GO2.md](ONLINE_GO2.md)。保持 `enable_motion: false`。

## 4. 启动在线导航，查看路线

```bash
ros2 launch go2_pct_adapter online_navigation.launch.py \
  config:="$GO2_NAV_CONFIG" body_height:=0.4
```

`body_height` 换成机体参考点的实际离地高度，0.4 只是示例。
本入口同时启动接口、在线 PCT、SCAN 和跟踪器；不要再同时启动
`interfaces.launch.py` 或手动测试速度发布器，以免重复节点/速度源。
无需提供 PCD 或 pickle：累计点云会在运行时转换成内存 tomogram。
首份地图处理完成前等待属于正常行为，只支持已观测且连通的区域，不会自主探索。

本 launch **不会自动打开 RViz**。另开已完成第 3 步环境加载的终端运行 `rviz2`。
将 Fixed Frame 设为配置中的 `planning_frame`，添加：

- PointCloud2：`/pct_go2/tomogram`，也可查看 `/uslam/cloud_map`。
- Path：`/pct_go2/global_path`。
- 将目标工具（如 2D Goal Pose）的 Topic 改为 `/pct_go2/goal`，然后点击目标。

起点来自当前里程计。目标 z 表示地面高度；二维点击适用于对应高度的平面，
跨楼层需显式指定目标 XYZ。观察以下话题，各命令在单独终端运行，Ctrl+C 退出查看：

```bash
ros2 topic echo /pct_go2/online_status
ros2 topic echo /pct_go2/bridge_status
ros2 topic echo /pct_go2/sport_request_preview
```

先确认地图/路线位置正确，有 SCAN 轨迹和持续跟踪速度。
`route_ready` 只表示全局路线已输出，还要满足跟踪心跳和输入有效性。
默认仅预览，机器狗不会因点击目标自动运动。

## 5. 对当前目标启用运动，或取消

使用现有 Sport 运控，已有正常站立状态、运动模式和控制权后，对已检查的短距离
平地目标发送：

```bash
ros2 topic pub --once /pct_go2/motion_enable std_msgs/msg/Bool '{data: true}'
```

此时才向 `/api/sport/request` 发高层速度请求。若固件要求租赁，需先连接已有 SDK
申请/续租管理，设置 `require_lease: true` 并持续提供有效 `/pct_go2/lease_id`；本包
没有实现自动获取租赁。不要仅凭 lease 话题存在就判断本固件必需租赁。

每次用户发送新目标都会撤销使能，检查新路线后需再次使能；同一目标的自动地图更新
和内部重规划不要求反复使能。暂停运动请求可发送：

```bash
ros2 topic pub --once /pct_go2/motion_enable std_msgs/msg/Bool '{data: false}'
```

清除目标和路线、同时撤销使能：

```bash
ros2 topic pub --once /pct_go2/cancel std_msgs/msg/Empty '{}'
```

旧 Foxy CLI 若无 `--once`，使用 `--rate 1`，收到后 Ctrl+C。
传感器、速度或跟踪失效会阻止旧速度继续发送；断网/桥进程崩溃后的停车和遥控接管
仍依赖底层控制器。上下楼和在线地图处理性能尚须实测，不能由平地接口检查推断。

## 已有离线 tomogram 时

完成相同构建和环境加载后，可改用以下入口：

```bash
ros2 launch go2_pct_adapter navigation.launch.py \
  config:="$GO2_NAV_CONFIG" \
  tomogram_path:=/absolute/path/your_environment.pickle body_height:=0.4
```

只有这个离线模式需要提前生成场地 tomogram；不要使用演示地图代替真实场地。
验证记录见 [VALIDATION.md](VALIDATION.md)，详细接口契约见 [README.md](README.md)。
