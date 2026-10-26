# Go2 在线累计点云导航

本入口复用 Go2 已有 SLAM、TF 和 Sport 运控。默认将 `/uslam/cloud_map` 视为
`sensor_msgs/msg/PointCloud2` 累计点云地图，每两秒至多启动一次后台全量 tomogram
重建；完成后在内存切换 PCT 地图，重新选择切片、搜索当前目标。
`/utlidar/cloud_deskewed` 继续只供 SCAN 实时局部感知。没有启动第二套 SLAM。

`/uslam/cloud_map` 是尚待实机核实的候选输入，建图 client 的启动方式和地图发布
条件见 [DEPLOY_GO2.md](DEPLOY_GO2.md) 第 4 步及 [USLAM_AUDIT.md](USLAM_AUDIT.md)。
本入口不启动 uslam，也不实现连续扫描累积。当前地图会转入统一的 `planning_frame`；
PCT/SCAN 分用 map/odom 并处理回环跳变的完整方案尚未实现。

这是无先验地图的在线规划入口，第一版只导航至已观测且连通的区域。目标尚不可达时，
保留原目标并等待后续地图，不把远目标裁剪到地图边缘，也不驱动机器人自主探索。
更新地图和内部重规划会清空旧路线、暂停跟踪，可能出现停走；同一目标已获得的运动
使能不会被反复撤销。新用户目标、取消和到达目标仍会撤销使能。

## 1. 目录、构建和环境加载

统一按 [DEPLOY_GO2.md](DEPLOY_GO2.md) 操作：使用完整仓库内的
`go2_adapter_ws`，先构建算法（包含 tomography），再构建接口，并加载两份
`install/setup.bash`（新终端可 `source go2_adapter_ws/env_on_go2.bash`）。
只复制接口包不足以运行在线导航。

以下命令假设已经按部署文档设置 `GO2_NAV_ROOT`、`GO2_NAV_CONFIG` 并加载环境。

在线入口需要 `tomography` 包，CPU 后端需要已有的 NumPy/SciPy。`auto` 优先选择
可用的 CUDA/CuPy 后端，否则使用同一套 PCT 方程的 CPU 实现。未验证 Go2 上的处理
速度，不应将默认两秒更新间隔视为性能保证。已知 CUDA 后端不可用时可设置 `cpu`。

## 2. 配置和启动

文件：`$GO2_NAV_CONFIG`，即仓库内的
`go2_adapter_ws/src/go2_pct_adapter/config/go2.yaml`。

- `go2_state_adapter`：原有机体 odom、实时雷达点云、规划坐标系和 TF 配置。
- `go2_online_planner.map_topic`：默认 `/uslam/cloud_map`，必须每条消息包含累计地图。
- 地图 frame 与 `planning_frame` 不同，节点会查消息时间戳对应的现有 TF 并变换点云。
  找不到 TF 时拒绝使用，不通过改 frame 名强行对齐。
- 地图消息需要当前时间戳并持续更新。默认消息时间和本地接收时间超时均为十秒；
  静态一次性发布或保留旧时间戳的地图不满足本入口契约，需调整源消息或使用离线入口。
- `resolution` / `slice_height`、机器人净空与台阶能力仍是模板参数，并非实机标定。
- `require_ceiling_observation: true` 是保守的室内默认：缺少顶部观测的网格被阻断。
  在开阔室外会因此没有路线；设为 false 意味着允许未观测到顶部的空间，部署前需
  确认相应净空策略。两种模式都会阻断缺失地面的区域，并为未知区域保留机体半径缓冲。
- 默认最多接收二百万点、构建二百万初始网格；超过限制会报告错误并阻断旧路线，
  可按实际算力提高分辨率数值或调整上限。上限不是实际内存峰值的精确估计。

仍可先运行原有只读接口检查，然后启动在线全链路，不需要 `tomogram_path`：

```bash
ros2 run go2_pct_adapter preflight \
  --config "$GO2_NAV_CONFIG"
```

结果 PASS 后，填写实际机身参考点离地高度并启动：

```bash
read -r -p '输入确认后的 body_height（米）：' GO2_BODY_HEIGHT
ros2 launch go2_pct_adapter online_navigation.launch.py \
  config:="$GO2_NAV_CONFIG" body_height:="${GO2_BODY_HEIGHT:?请先输入实际高度}"
```

`body_height` 换成实际机体参考点离地高度。preflight 检查 odom 和实时点云；
累计地图由在线节点在运行时检查。先保持默认预览，不自动使能运动。

## 3. 给目标和观察状态

当前 `online_navigation.launch.py` **不自动启动 RViz**。在有桌面的扩展坞或可接入
同一 ROS 2/DDS 图的电脑上，加载相同 ROS 环境后另开终端：

```bash
rviz2
```

在 RViz 中：

1. Global Options → Fixed Frame 设为配置中的 `planning_frame`（模板默认 `odom`）。
2. Add → By topic，添加 PointCloud2 `/pct_go2/tomogram` 和 Path `/pct_go2/global_path`；
   也可以添加 `/uslam/cloud_map` 查看原始累计地图。
3. Panels → Tool Properties，找到目标工具（如 `2D Goal Pose`），将 Topic 设为
   `/pct_go2/goal`。如果工具栏没有目标工具，可通过添加工具按钮选择它。
4. 使用目标工具在地图上给位置和方向。起点自动使用机器狗当前里程计，无须手动给起点。

目标 PoseStamped 发往 `/pct_go2/goal`，frame 必须等于实际 `planning_frame`。
跨楼层时还必须给出正确的目标地面高度 z；普通二维目标操作不能代替三维楼层选择，
可以由程序或 CLI 发布包含明确 XYZ 的 PoseStamped。
目标 z 表示地面高度。目标可先于首份地图到达，后续地图就绪会自动尝试规划。
若二维目标工具输出 z=0，而目标地面在规划坐标系内并非 z=0，请使用命令总表的
CLI 目标入口明确填写地面高度；代码不会自动从目标 z 扣除 `body_height`。
端点只允许默认 0.4 m 水平、0.25 m 高度容差的吸附，防止吸附到其他楼层或遥远边界。

```bash
ros2 topic echo /pct_go2/online_status
```

重点状态：

| reason | 含义 |
| --- | --- |
| `waiting_for_map` / `map_timeout` | 没有可用或足够新的累计地图 |
| `invalid_map` / `map_build_failed` | 输入、TF、资源限制或构建出错；查看 error 字段 |
| `waiting_for_odometry` | 尚无新鲜机体里程计 |
| `start_not_observed` / `goal_not_observed` | 没找到端点附近可通行地面，保留目标等下一份地图 |
| `no_known_route` | 当前地图没有起终点之间的已知连接 |
| `route_ready` | 已输出 PCT 路线，仍须满足原有跟踪、输入和运动使能条件 |
| `goal_reached` / `cancelled` | 已撤销运动使能并清空目标 |

`building`、`backend`、`map_sequence`、`built_sequence` 显示处理进度。
输入使用深度 1 的 best-effort/volatile 订阅，接受持续发布的 reliable 或 best-effort
地图；不会依赖先前保留的单次地图。`built_sequence` 记录最近完成或失败的构建尝试。
RViz 可显示 `/pct_go2/tomogram` 和 `/pct_go2/global_path`。

确认路线、SCAN 轨迹和运动预览正常后，按原有流程使能这一个目标：

```bash
ros2 topic pub --once /pct_go2/motion_enable std_msgs/msg/Bool '{data: true}'
```

地图自动刷新不需要每次重新使能。失效的路线会清空并停止跟踪；下一份有效路线
可在同一目标已有使能下继续。完全取消（同时撤销使能）：

```bash
ros2 topic pub --once /pct_go2/cancel std_msgs/msg/Empty '{}'
```

标准 Foxy CLI 支持 `--once`，也可用 `--times 1`；不要持续重复发布 uslam 的开始建图命令。
控制权、SDK 租赁和底层断网停车要求沿用 [DEPLOY_GO2.md](DEPLOY_GO2.md)。
累计点云的可靠地面/顶部观测、SLAM 回环与定位一致性、PCT 台阶通行模型和 SCAN
碰撞模型的一致性，以及物理运动效果仍须在机器人上验证。

## 实现参考

在线内存地图入口参考了 [dimensionalOS/PCT_planner](https://github.com/dimensionalOS/PCT_planner)
的 SLAM 模式思路；本实现复用当前仓库的 PCT CPU/CUDA 代码，并新增周期更新与目标
状态管理，没有替换现有 PCT→SCAN→Sport 分工。CPU 和回调测试不等同于 Foxy/实机验收。
