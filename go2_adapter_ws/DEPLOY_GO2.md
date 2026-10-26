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

如果目前只核查接口，**不用先构建，也不用启动导航**。在能看到狗上话题的现有
Foxy/宇树终端，进入完整仓库，运行下面一条采集命令即可（约 20 秒）：

```bash
cd ~/PCT-SCAN-ROS2
/usr/bin/python3 go2_adapter_ws/inspect_navigation.py --output go2_navigation_report.json
```

已有其他路径就替换 `cd`。报告覆盖 frame、频率、时间戳、TF、地图消息摘要和
控制话题类型。`body_height` 需要确认物理参考点后测量，完整地图/增量地图和租赁
权限仍需接口证据；脚本不会自动判断这些语义。uslam 未启动时可先采一次，再用原有
client 启动后换输出文件名重采。完整说明见 [README.md](README.md#只读采集实际接口)。

以下约定扩展电脑上的完整仓库位于 `~/PCT-SCAN-ROS2`，与第 1 步 clone 的名字一致。
已有其他安装位置时，只替换每个终端开头的 `cd`，无需移动源码。
开发电脑对应位置为 `/home/yeyu/go2 planner/PCT-SCAN-ROS2`；它只有 Noetic，
不在开发电脑执行这些 Foxy 构建，也不安装 ROS 2。

先进入你们原来能看到 Go2 数据的 Foxy/宇树/DDS 终端；若登录有 ROS 选择，选 Foxy。
沿用原来的宇树环境加载方式，不猜测其 setup.sh 路径，不重写 DDS 配置。

```bash
cd ~/PCT-SCAN-ROS2
echo "$ROS_DISTRO"
ros2 topic list -t
/usr/bin/python3 -c 'import rclpy, tf2_ros, yaml, numpy, scipy, colcon_core; from unitree_api.msg import Request; print("Foxy / Python / Unitree imports: PASS")'
cmake --version
```

应显示 `foxy` 并能看到机器人原有话题。PCT 原生依赖包括 GTSAM 4.2、OSQP 1.0。
OSQP 1.0 要求 **CMake >= 3.18**；如果当前是 3.16，先按文末的 CMake 工具步骤处理。

若缺少普通系统开发依赖，下面只安装编译工具、C++ 库及 Python 库，不安装 ROS 发行版。
已经齐备可跳过这一块。ROS 包与宇树消息包仍由现有 Foxy 环境提供。

```bash
sudo apt-get update
sudo apt-get install -y \
  build-essential git cmake ninja-build python3-dev python3-pip python3-venv \
  python3-numpy python3-scipy python3-yaml \
  libboost-all-dev libeigen3-dev libpcl-dev libopencv-dev \
  libarmadillo-dev libglm-dev libglew-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev
```

若已有 rosdep，可先检查包声明依赖；这是检查命令，不执行安装：

```bash
rosdep check --from-paths src go2_adapter_ws/src --ignore-src \
  --rosdistro foxy --skip-keys unitree_api
```

`unitree_api` 由原有宇树工作区提供，已在上面的 Python 导入检查中验证。
若缺少 ament、colcon、pybind11_vendor 等，需按实际报告补齐现有 Foxy 的开发依赖。
不要套用仓库后半部分的 Humble 演示安装命令。

下面一块在子 shell 中执行，任一步失败即停止，并把构建输出存入仓库 `log/`。
GTSAM/OSQP 会联网下载并构建到 `.deps/install`；若那里已有可用安装，可跳过
`setup_pct_dependencies.sh` 这一行。不要使用另一台电脑的 build/install 产物。

```bash
bash -eo pipefail <<'BUILD'
cd ~/PCT-SCAN-ROS2
export GO2_NAV_ROOT="$PWD"
test "${ROS_DISTRO:-}" = foxy
mkdir -p "$GO2_NAV_ROOT/log"
PCT_BUILD_JOBS=2 bash scripts/setup_pct_dependencies.sh \
  2>&1 | tee "$GO2_NAV_ROOT/log/go2_dependencies.log"
PCT_ROS_DISTRO=foxy PCT_BUILD_TESTING=OFF bash scripts/build_workspace.sh \
  --executor sequential --packages-up-to pct_scan_bridge scan_planner tomography \
  2>&1 | tee "$GO2_NAV_ROOT/log/go2_planner_build.log"
source scripts/_pct_scan_env.sh
bash go2_adapter_ws/build_on_go2.sh \
  2>&1 | tee "$GO2_NAV_ROOT/log/go2_adapter_build.log"
BUILD
```

如果仓库路径不同，子 shell 里的 `cd` 也要替换。第一套构建结果在根目录 `install/`，
接口结果在 `go2_adapter_ws/install/`。在线模式必须构建 `tomography`。
`build_on_go2.sh` 单独只构建接口，不能代替完整算法构建；无需再应用历史 patch。
完整 Foxy 编译、真实 DDS/TF、在线处理性能及物理运动仍待实机验证。

## 3. 每个新终端的环境加载

每个终端先进入原有 Foxy/宇树/DDS 环境，再执行：

```bash
cd ~/PCT-SCAN-ROS2
source go2_adapter_ws/env_on_go2.bash
```

新脚本按自身位置查找仓库，加载算法和接口两套工作区，导出 `GO2_NAV_ROOT` 与
`GO2_NAV_CONFIG`。默认配置文件是仓库内
`go2_adapter_ws/src/go2_pct_adapter/config/go2.yaml`。若要使用自己的配置，先
`export GO2_NAV_CONFIG=/实际位置/go2.yaml` 再 source；带空格路径需加引号。
这个脚本不启动 SLAM、导航或运动，也不自动寻找/改写宇树 DDS 环境。

可检查已安装的入口及原生库：

```bash
ros2 pkg prefix pct_planner
ros2 pkg prefix tomography
ros2 pkg prefix scan_planner
ros2 pkg prefix go2_pct_adapter
ros2 pkg executables go2_pct_adapter
/usr/bin/python3 -c 'import pct_planner.lib.a_star, pct_planner.lib.ele_planner; from tomography.cpu_backend import cpu_point2map; import go2_pct_adapter.online_pct_bridge; print("Online planner imports: PASS")'
```

## 4. 终端 A：准备 uslam 地图输入

优先使用你们原有 client。你记得的 `mapstart` 是启动建图的线索，并不能唯一确认
协议或回环配置。先查看接口和日志：

```bash
ros2 topic type /uslam/client_command
ros2 topic info /uslam/client_command --verbose
timeout 8s ros2 topic echo /uslam/server_log
```

以下字符串命令对应公开 uslam 客户端，详见 [USLAM_AUDIT.md](USLAM_AUDIT.md)。
**仅在实际接口类型为 `std_msgs/msg/String`、且原 client 确认使用该协议时执行。**
`/api/slam_operate/request` 或 `/qt_command` 类型不同，不能向它们照发这些字符串。
先查询建图状态：

```bash
ros2 topic pub --once --keep-alive 1.0 \
  /uslam/client_command std_msgs/msg/String '{data: mapping/get_status}'
```

若准备在新场地开始一份新地图，且尚未建图，再发送一次：

```bash
ros2 topic pub --once --keep-alive 1.0 \
  /uslam/client_command std_msgs/msg/String '{data: mapping/start}'
```

已经在建图就不要重复发送开始命令；开始新建图可能替换当前地图。
建图过程本身也会估计位置，未知场地在线建图不要求再额外开启 `localization/start`。
已有保存地图的定位是另一条流程，通常还需加载地图、设置初始位姿；不要为了运行
本入口直接填零位姿或启动原厂 navigation/patrol。

可选的单次取图请求（仍须先确认该固件支持）：

```bash
ros2 topic pub --once --keep-alive 1.0 \
  /uslam/client_command std_msgs/msg/String '{data: mapping/get_cloud_map}'
```

请求地图不等于已经有了持续地图流，也不证明返回的是 PointCloud2 或经过回环优化。
不要把请求周期发布、旧地图重打时间戳作为在线更新的替代。
官方 Foxy CLI 支持 `--once`/`--times 1`；若机器中的定制 CLI 不支持，使用原 client
发送一次，不要以持续重复 `mapping/start` 作为替代。

检查实际输入：

```bash
ros2 topic info /uslam/cloud_map --verbose
timeout 5s ros2 topic echo /uslam/cloud_map --no-arr
timeout 5s ros2 topic echo /uslam/frontend/odom --no-arr
timeout 5s ros2 topic echo /utlidar/robot_odom --no-arr
timeout 5s ros2 topic echo /utlidar/cloud_deskewed --no-arr
```

当前在线入口要求 `map_topic` 是含 XYZ 的 `sensor_msgs/msg/PointCloud2`，每条消息
已经包含要规划的累计地图，时间戳有效并持续更新；默认十秒超时。不满足时需适配地图
输出，或使用离线 tomogram 入口。不能直接将单帧 `cloud_world_ds` 或 `cloud_deskewed`
改作 `map_topic` 来代替累计地图。只收到单次地图，也不能据此宣称在线链路已就绪。

## 5. 终端 B：配置坐标系并运行只读检查

先完成第 3 步环境加载，编辑实际配置：

```bash
nano "$GO2_NAV_CONFIG"
```

按消息和已有 TF 核对下列字段；它们是模板，不是实机确认值：

| 配置位置 | 核对内容 |
| --- | --- |
| `go2_state_adapter.odom_topic` | 一路确实表示机体参考点的稳定里程计 |
| state 与 sport 的 `planning_frame` | 均等于所选 odom 的 `header.frame_id` |
| state 与 sport 的 `base_frame` | 均等于所选机体 odom 的 `child_frame_id` |
| state 的 `cloud_is_world` | 实时点云已在 planning_frame 中为 true；在雷达系中为 false |
| state 的 `sensor_frame` | 实际雷达坐标系；现有 TF 中能查到 base 到 sensor |
| online 的 `map_topic` | 已确认满足上一步数据要求的地图话题 |
| sport 的 `enable_motion` | 保持 false，先看预览 |

**当前代码只使用一个 `planning_frame`。**默认保留 utlidar 机体里程计和 `odom`；
在线节点在接收地图时，将不同 frame 的地图按消息时刻的 TF 转入 planning_frame。
之前讨论的“PCT 在 map、SCAN 在 odom，路线动态转换并处理回环跳变”尚未完整实现。
所以当前命令可用于集成验证，不代表长距离回环一致性已经解决。

若地图实际 frame 是 `map`、规划 frame 是 `odom`，可检查两者连通性：

```bash
timeout 10s ros2 run tf2_ros tf2_echo odom map
```

将两个 frame 名替换成消息里的真实值；这里查询的是地图到规划系的变换。
最新 TF 连通还不能证明历史消息时刻可查，在线节点会按地图时间戳再次检查。
已有雷达 TF 正确时复用即可，不需要重新标定；不能通过改 frame_id 强行对齐。
若点云已在 odom 下，不要再次按雷达局部点云变换。

```bash
ros2 run go2_pct_adapter preflight --config "$GO2_NAV_CONFIG"
```

报告 `result: PASS` 后继续。preflight 主要检查里程计、实时点云、时间戳、雷达 TF 和
请求序列化；不检查累计地图全貌、回环效果、控制权或物理运动。查看完整 uslam 信息可运行：

```bash
/usr/bin/python3 "$GO2_NAV_ROOT/go2_adapter_ws/inspect_uslam.py" \
  > "$GO2_NAV_ROOT/log/uslam_inventory.txt" 2>&1
```

## 6. 终端 B：启动 PCT → SCAN → Sport 预览

```bash
ros2 launch go2_pct_adapter online_navigation.launch.py \
  config:="$GO2_NAV_CONFIG" body_height:=0.4
```

`body_height` 换成机体参考点的真实离地高度，0.4 是示例。
该 launch 同时启动接口、在线 PCT、SCAN 和跟踪器。不要再同时启动
`interfaces.launch.py` 或测试速度发布器，避免重复节点或速度源。
无需预先提供 PCD/pickle：输入地图被转换为内存 tomogram。
地图、里程计或 TF 不可用时会等待/拒绝路线，不会凭空得到全局地图。
只支持已观测且连通的区域；未知目标会等待新地图，没有自主探索模块。

## 7. 终端 C：打开 RViz，设置目标

在有桌面的电脑上完成第 3 步环境加载，然后：

```bash
rviz2
```

当前 launch **不自动打开 RViz**。在 RViz 设置：

1. Fixed Frame 使用配置中的 `planning_frame`，模板是 `odom`。
2. 添加 PointCloud2 `/pct_go2/tomogram`、`/pct_go2/cloud` 和 Path `/pct_go2/global_path`。
3. 在目标工具（如 2D Goal Pose）的 Tool Properties 中，把 Topic 设为 `/pct_go2/goal`。
4. 给出目标；起点由当前里程计提供，无需手动设置。

目标 z 表示地面高度。普通二维点击适用于相应高度平面，不能自动选择其他楼层。
没有 RViz 时，可以在第 3 步环境加载后，用 CLI 给一个明确 XYZ 的目标：

```bash
export GO2_GOAL_FRAME="$(/usr/bin/python3 -c 'import os,yaml; print(yaml.safe_load(open(os.environ["GO2_NAV_CONFIG"]))["go2_state_adapter"]["ros__parameters"]["planning_frame"])')"
ros2 topic pub --once --keep-alive 1.0 /pct_go2/goal geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: '$GO2_GOAL_FRAME'}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}"
```

XYZ=(1,0,0) 只是规划坐标系下的示例目标，换成地图中已观测、可通行的位置，
不是相对机器狗前进一米的指令。新目标会撤销运动使能。

## 8. 终端 D：观察状态，再决定执行当前目标

先完成第 3 步环境加载。下列查看命令分别运行，Ctrl+C 结束当前查看后可运行下一条：

```bash
ros2 topic echo /pct_go2/online_status
ros2 topic echo /pct_go2/bridge_status
ros2 topic echo /pct_go2/sport_request_preview
```

也可查看全局路线、局部轨迹和速度是否有输出：

```bash
ros2 topic hz /pct_go2/global_path
ros2 topic hz /pct_go2/bspline
ros2 topic hz /pct_go2/cmd_vel
```

路线是事件驱动输出，`global_path` 没有固定频率不等于故障。
`online_status.reason=route_ready` 只表示全局路线已就绪，还要满足有效跟踪心跳和输入。
确认地图和路线对齐、轨迹及请求预览正常后，在正常站立、运动模式和控制权已准备好的
情况下，发送下列命令才开始真实 Sport 请求：

```bash
ros2 topic pub --once --keep-alive 1.0 /pct_go2/motion_enable std_msgs/msg/Bool '{data: true}'
```

每次用户新目标后需重新使能；同一目标的地图刷新和内部重规划保持已有使能。
预览过久轨迹可能过期，此时重新给目标，不要增大超时掩盖问题。
若固件要求运动租赁，需已有 SDK 管理有效租赁，并设置 `require_lease: true`、
持续提供 `/pct_go2/lease_id`；本包没有自动申请租赁。

暂停运动请求：

```bash
ros2 topic pub --once --keep-alive 1.0 /pct_go2/motion_enable std_msgs/msg/Bool '{data: false}'
```

取消目标并清空路线，同时撤销使能：

```bash
ros2 topic pub --once --keep-alive 1.0 /pct_go2/cancel std_msgs/msg/Empty '{}'
```

关闭导航前先取消目标，再在终端 B Ctrl+C。停止 uslam 建图、保存地图等操作继续使用
原 client，避免将停止建图当作暂停机器狗的命令。桥进程崩溃、断网后的停车和遥控接管
仍依赖底层控制器；上下楼及 CPU/CUDA 性能需另外实测。

## 已有离线 tomogram 时

完成相同构建和环境加载后，可改用以下入口：

```bash
ros2 launch go2_pct_adapter navigation.launch.py \
  config:="$GO2_NAV_CONFIG" \
  tomogram_path:=/absolute/path/your_environment.pickle body_height:=0.4
```

只有这个离线模式需要提前生成场地 tomogram；不要使用演示地图代替真实场地。
验证记录见 [VALIDATION.md](VALIDATION.md)，详细接口契约见 [README.md](README.md)。


## CMake 低于 3.18 时的独立工具安装

仅当第 2 步发现 CMake 过旧时使用。它只在 `.deps` 下准备构建工具，
不安装 ROS、不激活 Python 虚拟环境，也不替换系统 Python。需要已有 python3-venv。

```bash
cd ~/PCT-SCAN-ROS2
/usr/bin/python3 -m venv .deps/cmake-tools
.deps/cmake-tools/bin/python -m pip install 'cmake==3.27.9'
export GO2_CMAKE_BIN="$(.deps/cmake-tools/bin/python -c 'import pathlib,cmake; print(pathlib.Path(cmake.__file__).resolve().parent / "data" / "bin")')"
export PATH="$GO2_CMAKE_BIN:$PATH"
cmake --version
```

在同一终端继续第 2 步构建；以后重新构建时，在该终端重新设置上述两条 export。
正常运行导航不需要这套 CMake 工具。
