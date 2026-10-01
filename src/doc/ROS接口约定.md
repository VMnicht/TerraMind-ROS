# ROS 接口约定

默认所有节点使用相对话题名，可在一致的命名空间下运行。发布者与订阅者的控制 QoS 为 reliable、volatile、keep last 1。第一阶段仅支持单个上层控制源，未实现多源仲裁。不要在同一命名空间并行启动 real、sim 和 serial_test；运行多个机器人或比较模式时应使用隔离的 ROS 域。

| 接口 | 类型 | 方向和语义 |
|---|---|---|
| cmd_vel | geometry_msgs/TwistStamped | 上层至控制管理；仅 linear.x 为 m/s，angular.z 为 rad/s |
| implements/command | terramind_interfaces/ImplementCommand | 上层至控制管理；所有作业装置的完整目标 |
| control/set_enabled | std_srvs/SetBool | 使能/撤销使能；使能前需要健康、及时的板卡状态 |
| control/stop | std_srvs/Trigger | 停止并锁定；恢复必须显式使能 |
| control/status | terramind_interfaces/ControlStatus | PC 使能状态、链路就绪状态、连接标识、原因 |
| mcu/command | terramind_interfaces/ControlCommand | 控制管理独占发布；每 20 ms 完整快照 |
| mcu/state | terramind_interfaces/McuState | 后端发布；对应完整有效状态帧，不重复旧状态来伪装新鲜数据 |
| diagnostics | diagnostic_msgs/DiagnosticArray | 链路和协议诊断，正常约 1 Hz，连接失败即时报告 |
| sim/ground_truth | nav_msgs/Odometry | 直接仿真专用，真值，无测量噪声；不发布到 odom |
| joint_states | sensor_msgs/JointState | 直接仿真车轮转角与角速度 |
| sim/reset | std_srvs/Trigger | 清零模型并更换连接标识，需要重新使能 |
| sim/drop_commands | std_srvs/SetBool | 仿真故障注入，丢弃后端至控制板模型的指令 |
| sim/drop_status | std_srvs/SetBool | 仿真故障注入，丢弃模型至后端的状态 |

ControlCommand 中 connection_id 必须匹配当前 McuState。控制帧自身 uint16 序号由后端生成，不由业务层指定。Header 时间戳按 PC/ROS 时间填写；上层周期推荐 20 ms。cmd_vel 的 frame_id 允许为空（约定 base_link）或 base_link，其他坐标系将被拒绝。真实超时计时使用 steady clock，软件发送周期不是硬实时保证。

`terramind_panel/control_panel` 发布 `cmd_vel`、`implements/command` 并调用使能/停止服务；订阅 `mcu/state`、`control/status`、`mcu/command`、`diagnostics`。它不发布 `mcu/command`，也不直接打开串口。启动前和发送中检查额外的上层发布者及重复后端，冲突时拒绝或停止发送；这只是面板的冲突检查，不构成系统级控制权仲裁。多个机器人使用独立 ROS 域，或给所有节点一致设置命名空间。

完整作业指令显式使用 left_rpm、right_rpm、mower_percent、lift_mm、sprayer_percent 等单位字段。装置关闭后模型和固件将其执行目标置零，字段本身仍需要满足协议范围。PC 控制管理节点拒绝非法值，不通过裁剪来悄悄改变上层指令。

McuState.header.stamp 是 PC 接收/仿真生成时间，不是传感器采样时间。uptime_ms 是板卡运行时间。link_ready 是 PC 的会话健康判断，不代表每个传感器独立有效。simulated 是启动后端的标识，不是网络认证机制。真实轮速暂缺少新鲜度标志，刀盘状态仅有设定油门。

协议保留字段原样上报，PC 自身的超时、连接等原因由 ControlStatus 和 diagnostics 表达。当前 CAN 故障位和命令超时位会阻止运行；接收缓冲区历史溢出位保留并上报，不能单凭这一历史位永久禁止重新使能。
