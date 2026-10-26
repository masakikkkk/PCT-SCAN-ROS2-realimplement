# Go2 uslam 配置核查

核查日期：2026-10-26（Asia/Shanghai）。本文件记录证据，不是实机配置结果。

当前开发电脑中未找到 Go2 内置 uslam 的实际配置或安装目录，未连接机器人。
因此回环开关、frame、地图更新策略和实际输入源均未确认，不将其他 SLAM 的参数
视为本机 uslam 配置。PCT/SCAN 配置保持原样，`/uslam/cloud_map` 仍只是候选地图输入。

用户补充：记得通过 client 启动，完整命令暂未确认。该线索不足以唯一确定入口：
原始话题列表同时有 `/uslam/client_command`、`/api/slam_operate/request` 和
`/qt_command`。应读取其消息类型、订阅者以及实际进程命令行，不凭 client 一词
选择协议。采集脚本已加入三类入口和 `/uslam/server_log`，请求入口只查询元数据。
客户端发送“开始建图”与启动服务进程也不等价，应分别确认。

## 公开代码能确认的运行接口

与用户给出的 `/uslam/*` 话题对应的社区客户端为
[unitree_ui](https://github.com/legion1581/unitree_ui)。其
[命令实现](https://github.com/legion1581/unitree_ui/blob/main/src/ui/components/mapping-page.ts)
通过 `rt/uslam/client_command` 发送字符串，列出以下入口：

| 功能 | 公开客户端列出的命令 |
| --- | --- |
| 前端 | `frontend/start`、`frontend/stop`、`frontend/get_status` |
| 建图 | `mapping/start`、`mapping/stop`、`mapping/get_status` |
| 取地图 | `mapping/get_cloud_map` |
| 定位 | `localization/start`、`localization/stop`、`localization/get_status` |
| 初始位姿 | `localization/set_initial_pose/{x}/{y}/{yaw}` |

这证明客户端具备分阶段控制入口，不证明上述命令在用户固件中的行为，也不证明建图与
定位可以同时运行。核查脚本不发送这些命令。

[作者的接口记录](https://github.com/legion1581/unitree_ui/blob/main/docs/slam.md#topics)
把 WebRTC 侧的 `frontend/cloud_world_ds` 记录为建图实时点云，把 `cloud_map`
记录为停止建图后的最终地图；同时客户端存在主动取地图命令。仅凭这些信息，不能
认定 ROS/DDS 侧的 `/uslam/cloud_map` 是定频、每条完整累计的 PointCloud2，也不能
断言它在建图期间绝对无法获取。WebRTC 二进制封装不能直接用于推断 ROS 消息类型。

社区记录的可执行文件候选位置：
`/unitree/module/unitree_lidar_slam/bin/uslam_server`。
这是查找线索，尚未在这只 Go2 上确认；不要据此编造 config.yaml 文件名。

## 尚待实际配置或运行证据确认

- 可执行文件版本、真正加载的配置文件，以及服务运行在机身主机还是扩展电脑。
- 输入点云/IMU/里程计及其配准方式，是否复用 utlidar 前端。
- 回环检测和后端优化是否存在、启用，优化后的地图怎样重新发布。
- frontend、mapping、localization 模式能否同时工作；位姿是否包含后端修正。
- 每路 cloud/odom 的类型、frame、参考点、时间戳和实际发布条件。
- `map -> odom` 是否由系统提供，以及地图定位修正后怎样变化。

官方 [unitree_slam](https://github.com/unitreerobotics/unitree_slam) 也提供建图和重定位
接口示例，但其示例使用 `qt_command` 与 `lio_sam_ros2` 等话题。未建立版本对应关系前，
不把这一套接口、LIO-SAM 的回环配置或外置 MID360 的参数移植成 uslam 的“已知配置”。

## 取得这只机器的证据

将本目录的 `inspect_uslam.py` 放到相应电脑；加载已有 Foxy/宇树环境后运行：

```bash
python3 inspect_uslam.py > uslam_inventory.txt 2>&1
```

它只读取候选配置、相关进程、ROS 图和短时间消息样本，不启动/停止 SLAM、不发布控制
命令、不调用业务服务、不修改参数。文件扫描最多深入五层、六十个候选文件；跳过
地图数据和日志目录，不复制地图或二进制。ROS 节点提供参数服务时才尝试读取参数。

若安装位置不同，可明确传入实际模块目录；不要求安装 ROS：

```bash
python3 inspect_uslam.py --module-dir /actual/module/directory --skip-ros
```

`/actual/module/directory` 必须替换成真实位置。扩展电脑找不到候选目录，并不等于
机身主机没有 uslam；应在拥有该进程/目录的电脑上采集文件信息。服务尚未运行时，
脚本也不会替用户启动；缺少消息样本只表示本次没有观测到，不能推断算法能力。

结果需结合用户实际启动命令核对。脚本显示的文件是候选配置，存在某个字段不代表
进程已加载它；没有找到 loop 字段也不证明二进制中没有回环。

## 本地验证范围

Python 3.8 实际运行通过。临时配置与模拟 ROS CLI 检查了带空格路径、字段提取、
私密键过滤、损坏 JSON、地图数据目录排除、缺少模块及五个只读 CLI 查询。
在开发电脑实际运行仅确认没有对应目录/进程。未在 Go2 上读取配置，未执行真实
ROS 2 查询；以上模拟结果不等于 Foxy/DDS 兼容性验证。
