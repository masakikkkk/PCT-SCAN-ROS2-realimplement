# Go2 开机点云在线导航

更新日期：2026-11-11。默认使用开机已有的 `/utlidar/cloud_deskewed` 和
`/utlidar/robot_odom`，不再等待 `/uslam/cloud_map`，不启动另一套 SLAM 或原厂导航。
完整命令见 [COMMANDLIST.md](../COMMANDLIST.md)。

## 数据链路

```text
cloud_deskewed + robot_odom
          ↓ 已有 state_adapter（实际外参、时间检查仍需通过）
   /pct_go2/cloud、body_pose、sensor_pose
          ├── SCAN：实时局部地图与局部轨迹
          └── 新 C++ 地图维护节点：有界静态场景累计
                    ↓ MapSnapshot（地图、版本、健康状态、会话）
             CPU/CUDA tomogram → PCT 路线 → SCAN → Sport 预览
```

新增 `go2_map_maintenance` 位于 `go2_adapter_ws/src/`，与 Python 接口同一工作区。
`build_on_go2.sh` 会构建两个包和生成的 MapSnapshot 消息；不安装另一套 ROS。
依赖已有 Foxy 的 rclcpp、rosidl_default_generators 等开发组件，缺少时按构建报告补齐。
完整算法构建仍按部署文档操作。

## 第一版的实际行为

- 只接受规划系内的点；用户报告中点云为 `odom`，默认改为 `cloud_is_world: true`。
  不重复变换端点，不混用 uslam 前端 odom。
- 三维稀疏体素默认 10 cm；每体素每帧最多一票，3 个不同时间戳的帧确认后输出。
  确认前平均每帧代表点；确认后冻结代表点，原地重复命中不改变地图版本。
- 保留地面和顶部。只清理 5 秒未复现的未确认候选，不按时间删除已确认结构。
- **只做静态场景累积，没有射线清除、动态物体识别或回环优化。** 人或移动物体可能
  留下旧点；第一轮在静态、小范围场地验证，场景变化后取消目标并重置地图。
- 范围以首次接纳的机身位置为中心，水平 ±10 m、垂直 ±4 m。范围外的点不进入地图；
  机器人越界、体素数量超过 200000 时锁定故障，要求显式重置。
- 默认约 5 Hz 积分、1 Hz 发布，只保留最新待处理帧。点云与位姿时间差不超过 0.1 秒，
  输入超时 0.5 秒。不将旧点云重打当前时间来维持健康。
- 里程计相邻消息位置步进超过 0.5 m、姿态步进超过约 45 度，或点云时间倒退时锁定。
  这是保守异常检测，不是完整的漂移/重定位识别。

PCT 仍默认 `require_ceiling_observation: true`，缺少顶部观测可能没有可通行路线。
只有确认场地实际净空后才考虑改为 false；该选项不表示自动证明未知空间可通行。
地面观测缺失仍会阻断路线。分辨率、机体净空和台阶参数均需按现场数据检查。

## 几何版本与健康状态

`/pct_go2/map_snapshot` 使用 `go2_map_maintenance/msg/MapSnapshot`，同一消息包含
会话、版本、最新接纳的输入时间戳、健康状态与完整地图，避免跨话题错配。

相同版本且几何一致、持续有新鲜有效输入时，PCT 仅更新健康状态，不重新构建。
新增稳定表面后才后台重建 tomogram；切换时沿用原来的暂停/重规划逻辑。
维护节点退出、输入超时或故障时撤销路线。失败的构建不能靠同版本心跳恢复，旧工作
结果不能跨故障/会话复活路线。默认维护心跳和输入时间检查均为 1.5 秒。

地图重置或维护节点重启改变会话，PCT 撤销旧目标与使能，需要重新给目标。
在完整节点首次启动、首份地图之前给出的目标仍可等待地图。

## 上机启动

先完成 [DEPLOY_GO2.md](DEPLOY_GO2.md) 的构建。在已有 Foxy/宇树环境的新终端执行：

```bash
cd ~/PCT-SCAN-ROS2
source go2_adapter_ws/env_on_go2.bash
ros2 run go2_pct_adapter preflight --config "$GO2_NAV_CONFIG"
```

PASS 后输入实际机身参考点离地高度并启动。若单独开过地图节点，先 Ctrl+C 结束它；
下面的 launch 会自动启动维护节点，不要同时运行两个：

```bash
read -r -p '输入确认后的 body_height（米）：' GO2_BODY_HEIGHT
ros2 launch go2_pct_adapter online_navigation.launch.py \
  config:="$GO2_NAV_CONFIG" body_height:="${GO2_BODY_HEIGHT:?请先输入实际高度}"
```

不需要启动 uslam 或提供 PCD/pickle，默认仍预览。原报告中的约 24.9 天时基差、
缺少机身到雷达 TF/固定外参仍需按实际安装处理；本次没有填假外参、关闭时间检查或
默认开启运动来绕过这些条件。

另一个加载过环境的终端查看：

```bash
timeout 5s ros2 topic echo /pct_go2/map_status
timeout 5s ros2 topic echo /pct_go2/online_status
timeout 5s ros2 topic echo /pct_go2/bridge_status
```

`map_status.healthy=true`、`reason=ready` 表示地图输入就绪；还需全局路线、局部跟踪及
控制权限满足。`input_timeout_or_waiting_for_input`、`collecting_confirmed_surface`、
定位跳变/容量错误分别对应输入、确认数量和锁定故障。

RViz 仍手动启动，Fixed Frame 为实际 planning_frame，可添加：

| 话题 | 显示 |
| --- | --- |
| `/pct_go2/cloud` | 原实时点云 |
| `/pct_go2/accumulated_cloud` | 维护节点的累计表面点，仅健康时发布 |
| `/pct_go2/tomogram` | 代价地图 |
| `/pct_go2/global_path` | PCT 地表路线 |

目标工具 Topic 为 `/pct_go2/goal`，z 是地面高度。二维工具给出 z=0 而真实地面并非
z=0 时，用命令总表的 CLI 填写明确地面高度。使能、暂停和取消仍按总表操作。

## 显式清空地图

先取消目标，再清空累计地图：

```bash
ros2 topic pub --once --keep-alive 1.0 /pct_go2/cancel std_msgs/msg/Empty '{}'
ros2 topic pub --once --keep-alive 1.0 /pct_go2/reset_map std_msgs/msg/Empty '{}'
```

再次完成地图确认后，需要重新给目标并使能。旧表面没有动态删除功能，SCAN 仍独立
使用实时局部感知。

## 可选外部地图

若已验证原厂提供持续、新鲜、完整累计 PointCloud2，可设置
`go2_online_planner.map_source: external`，`map_topic: /uslam/cloud_map`。
此时同一 launch 不启动维护节点，恢复外部地图输入（默认十秒超时）。地图 frame
不同时查询消息时间戳 TF；原厂接口资料保留在 [USLAM_AUDIT.md](USLAM_AUDIT.md)。
离线 tomogram 入口继续保留。

## 实际验证范围

没有 ROS 2 的开发电脑已验证累计核心、点云解码、真实 CPU tomogram 和原生 PCT A*
的合成链路，以及 Python 回调/启动配置回归。未安装 ROS 2。Foxy 消息生成、C++ ROS
节点链接、DDS/TF 与物理运动仍需目标机验证，见 [VALIDATION.md](VALIDATION.md)。
累计地图在 odom 中，会继承定位漂移；本版本不包含自主探索。
