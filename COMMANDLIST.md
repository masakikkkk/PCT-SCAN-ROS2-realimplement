# Go2 EDU / ROS 2 Foxy 命令总表

这份文件放在**完整仓库根目录**，与最外层 `README.md`、`src/`、`scripts/` 同级。
用于本 Fork 的 Go2 接入；沿用狗上已有 Foxy、宇树 SDK/DDS、SLAM 和 Sport 运控。
不需要安装另一套 ROS。更新日期：2026-10-26。

| 要做的事 | 位置 |
| --- | --- |
| 下载、更新和确认版本 | [1](#1-获取代码和确认版本) |
| 只查询狗上的接口，暂不安装或编译 | [2](#2-只读查询接口不用先编译) |
| 安装普通依赖、升级构建用 CMake | [3](#3-install安装普通依赖) |
| 编译算法和 Go2 接口 | [4](#4-build编译两套工作区) |
| 单元测试、C++ 测试、构建后的检查 | [5](#5-test测试与构建检查) |
| 修改话题、坐标系和高度配置 | [6](#6-config修改实际配置) |
| 查询或启动 uslam | [7](#7-uslam查询与建图入口) |
| 日常启动、RViz、目标、运动使能、停止 | [8](#8-use日常在线导航) |
| 离线 PCD 转 tomogram 和离线导航 | [9](#9-可选离线地图模式) |
| 排错、导出报告 | [10](#10-debug排错与导出报告) |
| 修改后重新构建、上传 GitHub | [11](#11-update更新代码与上传修改) |

首次部署按 **1 → 2 → 3 → 4 → 5 → 6 → 7 → 8**；只采接口就做到第 2 节。
已经装好后，日常使用看第 8 节。每个代码块按所在章节的条件执行，不要把整份文档
作为一个脚本运行；尤其不要一起执行“开始运动”和“取消目标”。

## 0. 路径和终端约定

目前开发电脑的完整仓库位置是：

```bash
cd '/home/yeyu/go2 planner/PCT-SCAN-ROS2'
```

下面的 Go2 命令统一假设完整仓库克隆到了 `~/PCT-SCAN-ROS2`。你如果放在别处，
把每个新终端开头的 `cd` 换成真实路径即可。**不要进入外面的旧独立
`go2_adapter_ws`；当前接口在完整仓库内部。**

Foxy 命令在**原来就能看到狗上话题的终端**执行；沿用你们原来的宇树环境加载方式。
开发电脑只有 Noetic，不在它上面执行 Foxy 构建或导航，也不为此安装 ROS 2。
第 5.1、5.2 节的独立测试可以在开发电脑运行。

## 1. 获取代码和确认版本

Go2 扩展电脑上尚未下载完整仓库时，在家目录执行一次：

```bash
cd ~
git clone https://github.com/masakikkkk/PCT-SCAN-ROS2-realimplement.git PCT-SCAN-ROS2
cd PCT-SCAN-ROS2
```

已有完整仓库则更新，不需要重新 clone：

```bash
cd ~/PCT-SCAN-ROS2
git status --short
git pull --ff-only origin main
git log -1 --oneline
git ls-remote origin refs/heads/main
```

若 pull 提示本地修改冲突，先保留并处理自己的修改；不要直接覆盖或清理源码。
最后两条用于对照本地和远端版本。确认总表和脚本确实存在：

```bash
ls COMMANDLIST.md \
   scripts/build_workspace.sh \
   go2_adapter_ws/inspect_navigation.py \
   go2_adapter_ws/src/go2_pct_adapter/config/go2.yaml
```

在终端阅读本文件：

```bash
less COMMANDLIST.md
```

按 `q` 退出。GitHub 首页 README 顶部也有本文件入口。

## 2. 只读查询接口：不用先编译

### 2.1 检查现有 Foxy 环境

```bash
cd ~/PCT-SCAN-ROS2
echo "$ROS_DISTRO"
ros2 topic list -t
/usr/bin/python3 -c 'import rclpy, tf2_ros; from nav_msgs.msg import Odometry; from sensor_msgs.msg import PointCloud2; print("Existing ROS imports: PASS")'
```

应显示 `foxy` 并能看到原有狗上话题。看不到时，先恢复原有 ROS/DDS 环境；
不要通过修改话题名来解决网络或环境问题。

### 2.2 统一采集：默认约 20 秒

```bash
/usr/bin/python3 go2_adapter_ws/inspect_navigation.py \
  --output go2_navigation_report.json
```

报告在**执行命令时的当前目录**。本节从仓库根目录执行，因此它就在
`~/PCT-SCAN-ROS2/go2_navigation_report.json`。包含里程计、实时点云、候选地图的
frame、时间戳、接收频率、QoS、TF 及控制话题元数据。脚本不需要构建接口包。

uslam 未启动时也可以先采；用原有 client 启动建图后，再采一份对照：

```bash
/usr/bin/python3 go2_adapter_ws/inspect_navigation.py \
  --output go2_navigation_report_uslam.json
```

如果实际话题与默认不同，可以指定。下面的名称是当前候选值，替换成查询到的真实名称：

```bash
/usr/bin/python3 go2_adapter_ws/inspect_navigation.py \
  --duration 30 \
  --odom-topic /utlidar/robot_odom \
  --cloud-topic /utlidar/cloud_deskewed \
  --map-topic /uslam/cloud_map \
  --output go2_navigation_report_custom.json
```

若已知道物理雷达 TF frame，还可加 `--sensor-frame 实际雷达frame`。
查看参数或报告：

```bash
/usr/bin/python3 go2_adapter_ws/inspect_navigation.py --help
less go2_navigation_report.json
```

### 2.3 补充导出 uslam 配置和消息定义

```bash
/usr/bin/python3 go2_adapter_ws/inspect_uslam.py > uslam_inventory.txt 2>&1
bash go2_adapter_ws/inspect_go2.sh > go2_interfaces.txt 2>&1
```

这三个脚本均不发布运动指令，也不启动 SLAM。机身主机上的 uslam 配置可能无法在
扩展电脑读取；相关说明见 [USLAM_AUDIT.md](go2_adapter_ws/USLAM_AUDIT.md)。
`body_height`、地图是否每条完整累计、实际 lease 权限不能由这些报告自动认定。

## 3. Install：安装普通依赖

### 3.1 已有环境检查

本节在 Go2 扩展电脑执行，已有的库无需重装：

```bash
cd ~/PCT-SCAN-ROS2
/usr/bin/python3 -c 'import rclpy, tf2_ros, launch_ros, yaml, numpy, scipy, colcon_core; from unitree_api.msg import Request; print("Foxy / Unitree / build imports: PASS")'
cmake --version
```

`unitree_api` 来自你们原有宇树工作区；导入失败时先加载正确的 setup 环境。
这里不下载替代 SDK，也不安装 ROS 发行版。

### 3.2 普通系统开发库和 Python 测试工具

确认缺少时执行：

```bash
sudo apt-get update
sudo apt-get install -y \
  build-essential git cmake ninja-build \
  python3-dev python3-pip python3-venv python3-setuptools python3-wheel \
  python3-numpy python3-scipy python3-yaml python3-pytest \
  libboost-all-dev libeigen3-dev libpcl-dev libopencv-dev \
  libarmadillo-dev libglm-dev libglew-dev libglfw3-dev \
  libgl1-mesa-dev libglu1-mesa-dev
```

已有 rosdep 时可查询缺少的 ROS 开发依赖；这条只检查，不安装：

```bash
rosdep check --from-paths src go2_adapter_ws/src --ignore-src \
  --rosdistro foxy --skip-keys unitree_api
```

若缺少 colcon、ament、pybind11_vendor 等，按结果补齐现有 Foxy 的开发环境。
不要执行上游 Humble 的整套安装命令。在线 CPU tomogram 不需要 Open3D 或 CUDA。

### 3.3 CMake 低于 3.18 时才执行

OSQP 1.0 构建需要 CMake 至少 3.18。以下准备独立的 CMake 3.27.9 工具，不改变
系统 Python，也不激活虚拟环境：

```bash
/usr/bin/python3 -m venv .deps/cmake-tools
.deps/cmake-tools/bin/python -m pip install 'cmake==3.27.9'
export GO2_CMAKE_BIN="$(.deps/cmake-tools/bin/python -c 'import pathlib,cmake; print(pathlib.Path(cmake.__file__).resolve().parent / "data" / "bin")')"
export PATH="$GO2_CMAKE_BIN:$PATH"
cmake --version
```

在这个终端继续构建。以后新终端中重新构建，若仍需这套工具，就重新执行上面两条
`export`；正常运行导航无需它。

## 4. Build：编译两套工作区

### 4.1 首次完整构建

在已经加载原有 Foxy/宇树环境的终端执行。若升级过 CMake，使用同一终端。
下面子 shell 任一步失败即停止，并保存日志：

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

第一步联网构建 GTSAM 4.2 / OSQP 1.0 到 `.deps/install/`；算法产物在根目录
`install/`，接口产物在 `go2_adapter_ws/install/`。只运行最后一条接口构建不够。
若路径不同，也要替换子 shell 里的 `cd`。无需重新打历史 `pct_scan_foxy.patch`。

### 4.2 每个新终端加载运行环境

构建完成后，每个用于导航、RViz、发目标或查看状态的新终端都先执行：

```bash
cd ~/PCT-SCAN-ROS2
source go2_adapter_ws/env_on_go2.bash
```

它自动设置 `GO2_NAV_ROOT`、`GO2_NAV_CONFIG` 并加载两套工作区；仍需终端原本已有
Foxy/宇树/DDS 环境。不能用 `bash env_on_go2.bash` 替代 `source`。

## 5. Test：测试与构建检查

### 5.1 Python 单元测试，无需 ROS 2

可在开发电脑或具备 NumPy/SciPy/PyYAML/pytest 的扩展电脑执行，从完整仓库根目录运行：

```bash
PYTHONNOUSERSITE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
PYTHONPATH=go2_adapter_ws/src/go2_pct_adapter:src/pct/tomography:src/pct/pct_planner:src/integration/pct_scan_bridge \
  /usr/bin/python3 -m pytest -q \
  go2_adapter_ws/src/go2_pct_adapter/test src/integration/pct_scan_bridge/test
```

这些测试包含模拟 ROS 传输，不会启动 Sport 或 SLAM；通过不代表实机编译与导航验收。

### 5.2 独立 C++ 回归测试，无需 ROS 2

需要已有 C++17 编译器和 Eigen。使用子 shell 保证编译失败时不运行旧二进制：

```bash
bash -e <<'TEST'
g++ -std=c++17 -O2 -Wall -Wextra -fsanitize=undefined \
  -I/usr/include/eigen3 -Isrc/pct/pct_planner/lib/src \
  src/pct/pct_planner/lib/test/test_online_replan.cc \
  src/pct/pct_planner/lib/src/a_star/a_star_search.cc \
  -o /tmp/go2_pct_astar_test
/tmp/go2_pct_astar_test
g++ -std=c++17 -O2 -Wall -Wextra -fsanitize=undefined \
  -Isrc/planner/plan_manage/include \
  src/planner/plan_manage/test/test_tracking_checks.cpp \
  -o /tmp/go2_tracking_checks_test
/tmp/go2_tracking_checks_test
TEST
```

正常退出表示断言通过；覆盖 A* 重规划和跟踪有效性检查，不会启动 ROS 控制节点。

### 5.3 Foxy 构建产物检查

Go2 扩展电脑加载第 4.2 节环境后：

```bash
ros2 pkg prefix pct_planner
ros2 pkg prefix tomography
ros2 pkg prefix scan_planner
ros2 pkg prefix go2_pct_adapter
ros2 pkg executables go2_pct_adapter
/usr/bin/python3 -c 'import pct_planner.lib.a_star, pct_planner.lib.ele_planner; from tomography.cpu_backend import cpu_point2map; import go2_pct_adapter.online_pct_bridge; print("Online planner imports: PASS")'
```

### 5.4 已编译 PCT 的演示地图 A* 冒烟测试，可选

仍在扩展电脑或已有对应 ROS 构建环境的电脑，先加载环境，再运行：

```bash
bash scripts/test_pct_building_astar.sh --force
bash scripts/test_pct_plaza_astar.sh --force
```

读取仓库自带演示 tomogram，只计算并导出路线，不驱动机器狗；`--force` 允许替换
同名演示路线 CSV。演示地图不等于你的真实场地地图。

## 6. Config：修改实际配置

加载第 4.2 节环境后，查看路径并编辑：

```bash
echo "$GO2_NAV_CONFIG"
nano "$GO2_NAV_CONFIG"
```

默认文件是 `go2_adapter_ws/src/go2_pct_adapter/config/go2.yaml`。按第 2 节报告修改：

| 字段 | 填写依据 |
| --- | --- |
| state 的 `odom_topic` | 确认能表示机身参考点位置的里程计 |
| state / sport 的 `planning_frame` | 同一值，匹配所选 odom 的 `header.frame_id` |
| state / sport 的 `base_frame` | 同一值，匹配已确认的机身 `child_frame_id` |
| state 的 `cloud_topic` / `cloud_is_world` | 实时点云；已在 planning_frame 为 true，雷达系为 false |
| state 的 `sensor_frame` / `use_tf_extrinsic` | 真实雷达 frame，优先复用已有机身到雷达 TF |
| online 的 `map_topic` | 持续更新、每条完整累计、含 XYZ 的 PointCloud2 地图 |
| sport 的 `enable_motion` | 保持 false，启动后先预览 |
| sport 的 `require_lease` | 由固件/SDK 的真实控制权限要求决定 |

`body_height` 在启动时传入：它是机身参考点到脚下地面的高度偏移，**不是 odom 的 z**。
必须先确认参考点再测量；文档不再将 0.4 米作为可直接用于真机的值。

只读 preflight：

```bash
ros2 run go2_pct_adapter preflight --config "$GO2_NAV_CONFIG"
```

`result: PASS` 仅证明该次接口、时间和配置检查通过；不证明累计地图、回环或控制权。
当前实现使用一个 `planning_frame`；完整的全局 map / 局部 odom 回环处理架构尚未完成。

## 7. uslam：查询与建图入口

### 7.1 优先继续使用你们现有 client

你记得的 `mapstart` 尚未对应到已确认的完整命令。这里不编造 client 可执行文件名。
先读取实际入口的类型和订阅者：

```bash
ros2 topic type /uslam/client_command
ros2 topic info /uslam/client_command --verbose
timeout 5s ros2 topic echo /uslam/server_log
```

### 7.2 以下字符串仅在固件协议已确认时使用

条件：上述类型为 `std_msgs/msg/String`，并确认原 client 使用同一协议。
公开来源及适用范围见 [USLAM_AUDIT.md](go2_adapter_ws/USLAM_AUDIT.md)。下面逐条按需
执行，不要整节连续粘贴；默认的部署启动不会替你发送这些请求。

查询前端和建图状态：

```bash
ros2 topic pub --once --keep-alive 1.0 /uslam/client_command std_msgs/msg/String '{data: frontend/get_status}'
ros2 topic pub --once --keep-alive 1.0 /uslam/client_command std_msgs/msg/String '{data: mapping/get_status}'
```

尚未建图，且准备在当前场地开始一份新地图时，发送一次：

```bash
ros2 topic pub --once --keep-alive 1.0 /uslam/client_command std_msgs/msg/String '{data: mapping/start}'
```

正在建图就不要重复开始；不要为未知场地建图额外执行 `localization/start`。
请求一次地图（不保证它成为持续地图流）：

```bash
ros2 topic pub --once --keep-alive 1.0 /uslam/client_command std_msgs/msg/String '{data: mapping/get_cloud_map}'
```

检查实际地图和定位输出：

```bash
ros2 topic info /uslam/cloud_map --verbose
timeout 5s ros2 topic echo /uslam/cloud_map --no-arr
timeout 5s ros2 topic echo /uslam/frontend/odom --no-arr
timeout 5s ros2 topic echo /uslam/localization/odom --no-arr
```

在线入口要求持续、新鲜、完整累计的地图，默认十秒超时。单帧去畸变点云或一份旧地图
不能直接替代它；不满足时要适配地图源，或用第 9 节离线入口。也不能只改 frame 名。

## 8. Use：日常在线导航

前提：已编译，配置通过 preflight，原有 Sport 运控就绪，且已准备符合第 7 节要求的
累计地图输入。首次到现场先完成接口采集；以下启动命令不代表这些前提已在真机验证。

### 8.1 终端 A：启动导航，默认预览

```bash
cd ~/PCT-SCAN-ROS2
source go2_adapter_ws/env_on_go2.bash
ros2 run go2_pct_adapter preflight --config "$GO2_NAV_CONFIG"
```

确认结果 PASS 后，再执行下一块。输入**已确认的机身参考点离地高度，单位米**：

```bash
read -r -p '输入确认后的 body_height（米）：' GO2_BODY_HEIGHT
ros2 launch go2_pct_adapter online_navigation.launch.py \
  config:="$GO2_NAV_CONFIG" body_height:="${GO2_BODY_HEIGHT:?请先输入实际高度}"
```

该入口启动接口、在线 PCT、SCAN 和局部跟踪器；默认不使能运动。
不要同时再启动 `interfaces.launch.py` 或另一个速度发布器。
在线模式不需要 PCD/pickle 文件，只能规划已观测且连通的区域，不会自主探索未知区域。

### 8.2 终端 B：打开 RViz 和给目标

有桌面的扩展电脑，或已接入同一 ROS/DDS 图的可视化电脑：

```bash
cd ~/PCT-SCAN-ROS2
source go2_adapter_ws/env_on_go2.bash
rviz2
```

launch 不自动打开 RViz。RViz 中：

1. Fixed Frame 设为配置中的 `planning_frame`。
2. 添加 PointCloud2 `/pct_go2/cloud`、`/pct_go2/tomogram`，及 Path `/pct_go2/global_path`。
3. 目标工具（如 2D Goal Pose）在 Tool Properties 中将 Topic 设为 `/pct_go2/goal`。
4. 在已观测的可通行地面给目标。目标 z 是地面高度；二维点击不能自动选其他楼层。

起点使用当前里程计，不需要手动设置。只收到地图尚不会自动开始运动。

### 8.3 没有 RViz 时，用终端给明确 XYZ 目标

新终端先加载第 4.2 节环境，下面交互输入规划坐标系内的**绝对目标**：

```bash
export GO2_GOAL_FRAME="$(/usr/bin/python3 -c 'import os,yaml; print(yaml.safe_load(open(os.environ["GO2_NAV_CONFIG"]))["go2_state_adapter"]["ros__parameters"]["planning_frame"])')"
read -r -p '输入目标 x y 地面z（米，空格分隔）：' GO2_GOAL_X GO2_GOAL_Y GO2_GOAL_Z
ros2 topic pub --once --keep-alive 1.0 /pct_go2/goal geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: '$GO2_GOAL_FRAME'}, pose: {position: {x: ${GO2_GOAL_X:?}, y: ${GO2_GOAL_Y:?}, z: ${GO2_GOAL_Z:?}}, orientation: {w: 1.0}}}"
```

例如 x=1 并不代表相对机器人前进一米。新用户目标会撤销运动使能，需再次检查后使能。

### 8.4 终端 C：查看状态

```bash
cd ~/PCT-SCAN-ROS2
source go2_adapter_ws/env_on_go2.bash
timeout 5s ros2 topic echo /pct_go2/online_status
timeout 5s ros2 topic echo /pct_go2/bridge_status
timeout 5s ros2 topic echo /pct_go2/sport_request_preview
```

`route_ready` 表示全局路线就绪，实际运动还要有有效跟踪、输入和控制权限。
`waiting_for_map` / `map_timeout` 先查地图源；`start_not_observed` /
`goal_not_observed` 查地面、目标高度和坐标对齐；`no_known_route` 表示已知地图未找到连通路线。
监视命令被 `timeout` 结束时可能返回 124，这是限时采样的正常退出方式。

### 8.5 执行当前目标：这一条会允许真实运动

路线、局部轨迹和请求预览确认正确，机器狗已正常站立且原有运控模式/控制权就绪后，
在已加载环境的新终端执行一次：

```bash
ros2 topic pub --once --keep-alive 1.0 /pct_go2/motion_enable std_msgs/msg/Bool '{data: true}'
```

每次新的用户目标后需重新使能；同一目标的地图刷新和内部重规划保留原有使能。
若固件要求 lease，本包不自动申请：由已有 SDK 维护有效租赁，配置
`require_lease: true` 并持续提供 `/pct_go2/lease_id`。不能凭话题存在就任填 lease ID。

### 8.6 暂停、取消、关闭

暂停本导航的运动使能：

```bash
ros2 topic pub --once --keep-alive 1.0 /pct_go2/motion_enable std_msgs/msg/Bool '{data: false}'
```

取消目标、清空路线并撤销使能：

```bash
ros2 topic pub --once --keep-alive 1.0 /pct_go2/cancel std_msgs/msg/Empty '{}'
```

关闭导航前先取消，再到终端 A 按 Ctrl+C；RViz 可另外关闭。重新开始时重新给目标并使能。
停 uslam、保存地图继续按原有 client 流程；停止建图不是暂停运动命令。
遥控接管和底层停车沿用机器人原有机制。

## 9. 可选：离线地图模式

在线模式正常工作时**不用执行本节**。只有已经保存了与定位对齐的场地点云/地图，
且准备使用固定 tomogram 时才需要。以下 PCD 转换建议在 x86 Ubuntu 20.04 开发电脑
完成，再把场地 pickle 传到 Go2；不假定 Jetson/ARM 有 Open3D 预编译包。

### 9.1 在开发电脑准备离线转换环境

从完整仓库根目录，用 Python 3.8 的兼容版本单独建环境，不激活它：

```bash
/usr/bin/python3 -m venv .venv-tomography
.venv-tomography/bin/python -m pip install \
  'numpy==1.24.4' 'scipy==1.10.1' 'open3d==0.18.0'
```

### 9.2 PCD 转 tomogram，查看转换参数

交互填写真实输入、输出绝对路径与地图中的基准地面 z。地面 z 用于切片范围，
不是机身的 `body_height`。同名输出已存在时默认拒绝覆盖：

```bash
read -r -p '输入场地 PCD 绝对路径：' GO2_PCD_PATH
read -r -p '输入输出 pickle 绝对路径：' GO2_TOMOGRAM_PATH
read -r -p '输入地图基准地面 z（米）：' GO2_MAP_GROUND_Z
bash scripts/generate_tomogram.sh \
  --input "${GO2_PCD_PATH:?}" --output "${GO2_TOMOGRAM_PATH:?}" \
  --ground-height "${GO2_MAP_GROUND_Z:?}" --backend cpu
```

查看参数和生成结果：

```bash
bash scripts/generate_tomogram.sh --help
.venv-tomography/bin/python scripts/inspect_pct_tomogram.py "${GO2_TOMOGRAM_PATH:?}"
```

转换参数（分辨率、台阶、净空等）需按场地和机器人能力确认。全局地图必须与当前
定位对齐，生成 pickle 本身不会完成定位或坐标配准。

### 9.3 Go2 上启动离线入口

与在线模式二选一，不同时启动。场地 pickle 已放在 Go2 后，在扩展电脑执行：

```bash
cd ~/PCT-SCAN-ROS2
source go2_adapter_ws/env_on_go2.bash
read -r -p '输入 Go2 上场地 pickle 的绝对路径：' GO2_TOMOGRAM_PATH
read -r -p '输入确认后的 body_height（米）：' GO2_BODY_HEIGHT
ros2 launch go2_pct_adapter navigation.launch.py \
  config:="$GO2_NAV_CONFIG" tomogram_path:="${GO2_TOMOGRAM_PATH:?}" \
  body_height:="${GO2_BODY_HEIGHT:?}"
```

后续 RViz、目标、预览、使能与取消仍按第 8 节；离线模式不用 `/pct_go2/online_status`。

## 10. Debug：排错与导出报告

### 10.1 图、输入、输出与频率

加载第 4.2 节环境后，按需查询；点云使用 `--no-arr`，避免打印整个二进制数据：

```bash
ros2 node list
ros2 topic list -t
timeout 5s ros2 topic echo /utlidar/robot_odom --no-arr
timeout 5s ros2 topic echo /utlidar/cloud_deskewed --no-arr
timeout 5s ros2 topic hz /utlidar/robot_odom
timeout 5s ros2 topic hz /utlidar/cloud_deskewed
timeout 5s ros2 topic hz /uslam/cloud_map
timeout 5s ros2 topic echo /pct_go2/body_pose --no-arr
timeout 5s ros2 topic echo /pct_go2/global_path --no-arr
timeout 5s ros2 topic echo /pct_go2/bspline --no-arr
timeout 5s ros2 topic echo /pct_go2/cmd_vel
```

上述原始输入名是默认候选值；配置改过时对应替换。路线是事件驱动，不保证固定频率。

### 10.2 TF 查询

两个 frame 从报告取真实名称；输入源坐标系中的点将被变换到目标坐标系：

```bash
read -r -p '输入目标 frame（规划坐标系）：' GO2_TF_TARGET
read -r -p '输入源 frame（地图或雷达坐标系）：' GO2_TF_SOURCE
timeout 10s ros2 run tf2_ros tf2_echo "${GO2_TF_TARGET:?}" "${GO2_TF_SOURCE:?}"
ros2 topic info /tf --verbose
ros2 topic info /tf_static --verbose
```

最新 TF 可查不代表历史消息时间戳可查；统一采集报告会另外记录消息时刻的查询结果。

### 10.3 导出状态与构建日志

```bash
mkdir -p log
ros2 run go2_pct_adapter preflight --config "$GO2_NAV_CONFIG" \
  > log/go2_preflight.json 2> log/go2_preflight_stderr.txt
timeout 5s ros2 topic echo /pct_go2/online_status > log/go2_online_status.txt 2>&1
timeout 5s ros2 topic echo /pct_go2/bridge_status > log/go2_bridge_status.txt 2>&1
ls -lh log
```

优先提供：第 2 节的 `go2_navigation_report*.json`、`uslam_inventory.txt`，以及
相应报错的构建日志/状态。第 4.1 节已保存构建日志，无需重新编译来取得旧日志。

## 11. Update：更新代码与上传修改

### 11.1 Go2 上拉取更新后重新构建

先按第 8.6 节取消并关闭导航。打开原有 Foxy/宇树终端：

```bash
cd ~/PCT-SCAN-ROS2
git status --short
git pull --ff-only origin main
git log -1 --oneline
```

仅文档或独立采集脚本变化，不用重编译。只改 Go2 Python 接口时：

```bash
export PCT_ROS_DISTRO=foxy
source scripts/_pct_scan_env.sh
bash go2_adapter_ws/build_on_go2.sh
```

算法、launch 资源或不确定变化范围时，重新构建算法和接口，不必重复安装已有依赖：

```bash
bash -eo pipefail <<'REBUILD'
export PCT_ROS_DISTRO=foxy
PCT_BUILD_TESTING=OFF bash scripts/build_workspace.sh \
  --executor sequential --packages-up-to pct_scan_bridge scan_planner tomography
source scripts/_pct_scan_env.sh
bash go2_adapter_ws/build_on_go2.sh
REBUILD
```

若系统 CMake 过旧，先恢复第 3.3 节的工具 PATH。构建成功后，新开运行终端加载
第 4.2 节环境，再做 preflight 和启动。不要复制另一架构电脑的 build/install 产物。

### 11.2 把自己的修改上传到 Fork

从自己修改过的完整仓库根目录执行，先检查内容：

```bash
git status --short
git diff
git remote -v
```

例如只修改了 Go2 配置，就只提交这个文件；其他文件同理，不要把采集报告或构建产物
一起提交。确认当前分支是要上传的 `main` 后：

```bash
git branch --show-current
git add go2_adapter_ws/src/go2_pct_adapter/config/go2.yaml
git diff --cached
git commit -m "Configure navigation topics and frames for our Go2"
git push origin main
```

HTTPS clone 后，push 需要 GitHub 已配置的凭据；不要用 GitHub 登录密码代替令牌。
已经设置 SSH 密钥的开发电脑可继续使用原来的 SSH origin，无需更换。

部署原理与参数说明见 [DEPLOY_GO2.md](go2_adapter_ws/DEPLOY_GO2.md)，在线地图契约见
[ONLINE_GO2.md](go2_adapter_ws/ONLINE_GO2.md)，实际验证范围见
[VALIDATION.md](go2_adapter_ws/VALIDATION.md)。完整 Foxy 编译、真实 DDS/TF 和物理导航
仍需在 Go2 上完成验证；本表中的实机命令不是已执行成功的记录。
