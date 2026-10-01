# TerraMind 割草机器人 PC

ROS 2 Humble 工作区，包含 USART3 控制板通讯、控制管理、二维差速仿真、虚拟串口控制板和 Qt 控制面板。PC 按协议发送底盘线速度与角速度，真实下位机负责左右轮分配。所有启动入口默认停机，需要显式使能和持续更新的控制指令。

## 构建

开发环境：Ubuntu 22.04、ROS 2 Humble、C++17。Python 辅助工具使用系统 Python，不需要 pyserial。依赖由各包的 `package.xml` 声明；标准 ROS desktop 环境还需要 colcon、ament_cmake_gtest、ament_cmake_pytest、python3-yaml、python3-pyqt5、python3-pytest。若缺少面板依赖，可用 `sudo apt install python3-pyqt5 python3-pytest ros-humble-ament-cmake-pytest` 安装。

```bash
cd /home/tang/TerraMind
source /opt/ros/humble/setup.bash
MAKEFLAGS="-j2 -l2" colcon build --symlink-install --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
```

后续每个新终端均需加载 ROS 和本工作区的 `install/setup.bash`。

## 无硬件二维仿真

一键启动，无需提前加载 ROS 或工作区环境：

```bash
/home/tang/TerraMind/start_sim.sh
```

脚本自动定位工程，启动控制管理节点、仿真接口、机器人模型、RViz 和控制面板；缺少构建产物时自动编译。按 `Ctrl+C` 退出。无显示环境会自动关闭所有图形窗口，也可显式指定：

```bash
/home/tang/TerraMind/start_sim.sh --headless
```

修改源码后使用 `start_sim.sh --build` 重新编译并启动。其他参数会传给原启动入口，例如 `description:=false`。脚本默认将 ROS 日志放在工程的 `log/ros/`，也支持已有的 `ROS_LOG_DIR` 设置。

已有 ROS 环境的终端仍可直接运行 `ros2 launch terramind_bringup sim.launch.py`。

RViz 默认使用 **Move Camera** 工具和 **Orbit** 视角。在三维画面内操作：左键拖动旋转视角，按住 **Shift + 左键** 拖动或中键拖动平移，滚轮缩放，也可以按住右键上下拖动缩放。Views 面板可调整距离、焦点或切换视角；若切换到选择工具，点击工具栏的 **Move Camera** 即可恢复视角操作。这些操作只移动观察视角，不控制机器人。

若使用命令行控制，请关闭面板（或启动时加 `--no-panel`），再在另一个终端运行下面的命令，显式使能并发送 3 秒的新鲜指令，最后请求停止：

```bash
ros2 run terramind_sim control_cli.py --linear 0.10 --angular 0.2 --duration 3
```

同一工具也可控制真实后端，执行前应确认正在使用的启动入口。默认作业装置均关闭；可以显式指定 `--left-rpm`、`--right-rpm`、`--mower-percent`。工具每 20 ms 更新时间戳，避免手写一个固定时间戳的命令被有效期检查拒绝。

```bash
ros2 topic echo /mcu/state
ros2 topic echo /control/status
ros2 topic echo /sim/ground_truth
ros2 service call /control/stop std_srvs/srv/Trigger '{}'
```

## 控制面板

测试全部可控装置的一键入口：

```bash
./start_sim.sh capabilities:=63
```

`63`（`0x003F`）使仿真支持全部六类能力。默认 `15`（`0x000F`）与当前控制板一致，升降和喷洒显示为不支持。该参数只作用于模拟后端，真实控制始终按板卡回传能力判断。

已有后端运行时，可单独打开面板，不需要再次启动后端：

```bash
./start_panel.sh
# 或在已加载工作区环境的终端：
ros2 run terramind_panel control_panel
```

面板连接同一 `ROS_DOMAIN_ID`、同一命名空间里的后端，仿真和实机共用同一套操作。不要同时运行其他控制面板或 `control_cli.py`；面板在启动及运行中检查控制源冲突，发现冲突即拒绝运行或停止。面板不实现多源仲裁。

| 可控字段 | 控制范围 |
|---|---|
| 系统使能、软件停止 | 开始发送 / 撤销使能 / 停止全部 |
| 底盘线速度、角速度 | −0.35～0.35 m/s；−2～2 rad/s |
| 左、右播撒 | 各自开关；−500～500 RPM |
| 割草刀盘 | 开关；0～100% |
| 刀盘升降 | 开关；0～1000 mm，需后端支持 |
| 喷洒装置 | 开关；0～100%，需后端支持 |

操作顺序：设置底盘数值、勾选需要开启的作业装置并填写目标，点击 **开始发送 / 使能**。面板先确认停止请求，再请求使能，避免旧停止请求影响新一轮控制；收到使能状态确认后，以 50 Hz 持续发送完整目标。运行中编辑属于草稿，点击 **应用修改** 后才生效。未勾选装置发送关闭和零目标；**清零草稿** 仅修改编辑值，实际停车请点击红色 **停止全部**。撤销使能和停止都会停止全部输出，再次运行需明确点击开始。

右侧显示板卡状态、能力位、最近命令序号、接纳结果、故障、指令年龄、各装置设定和反馈；“PC 完整指令”显示管理节点的输出，不是串口原始字节或逐帧执行回执。协议序号、CRC、保留位自动生成；播撒舵机角度仅有反馈，不是 v1 可控字段。

窗口关闭、界面阻塞、状态过期、连接更换或控制管理撤销使能后，不会自动恢复原目标。面板的 ROS 回调和发送心跳由 Qt 同一事件循环驱动，界面卡住时不会由后台线程继续续发旧命令。软件停止的服务响应只表示 PC 已处理请求；机械停转仍需实际反馈确认。

## 虚拟串口联调

```bash
ros2 launch terramind_bringup serial_test.launch.py
# 使用面板，并让虚拟板支持所有字段：
ros2 launch terramind_bringup serial_test.launch.py panel:=true capabilities:=63
```

该入口自动创建私有 PTY，运行真实 `mcu_serial_node` 和控制板模拟器，无需 USB 设备。上层仍使用同一个控制工具。此模式验证串口和协议，不发布二维位置或车轮动画。

## 连接真实控制板

连接 USB 串口后，直接启动即可自动识别控制板端口，无需填写 `ttyUSB` 编号：

```bash
ros2 launch terramind_bringup real.launch.py panel:=true
```

默认 `device: auto`。驱动优先扫描 `/dev/serial/by-id/*`，再扫描 `/dev/ttyUSB*` 和 `/dev/ttyACM*`；同一设备的多个别名只检查一次。扫描时以 115200 8N1 打开候选端口，在默认 0.6 秒的共同监听窗口中读取数据，不向候选端口发送控制报文。只有收到完整、CRC 正确且序号和运行时间持续前进的 v1 状态帧，才认为是控制板；导航文本、普通日志和无响应端口不会被当作控制板。

发现唯一匹配后，保留该端口的已打开句柄，清除扫描期间的接收缓存，执行未使能零指令握手。面板显示实际选择的设备路径；没有设备、权限不足、端口被占用或发现多个控制板时，也会显示原因。未找到时默认每隔 1 秒重试；拔插或端口号变化后重新扫描，连接恢复仍保持停机，需要再次手动开始发送。

如果本轮扫描发现多个符合协议的控制板，驱动不会任意选择或向它们发送控制帧。可缩小 `discovery_patterns` 的范围，或明确指定一个稳定设备路径：

```bash
ros2 launch terramind_bringup real.launch.py device:=/dev/serial/by-id/实际设备名称 panel:=true
```

手动路径和扫描范围都可以保存到 `src/terramind_bringup/config/serial.yaml`。指定具体 `device` 路径时直接使用原有连接握手流程，不做自动筛选。进程需拥有设备读写权限；串口库使用建议锁，跳过已经被本工程驱动占用的设备。自动识别依赖协议约定的每 50 ms 主动状态上报；如果固件尚未实现该行为，可先使用手动端口联调。v1 没有唯一板卡身份字段，自动识别的是控制板协议，不是特定机器人的身份认证。

启动后的第一帧合法零控制快照会使 USART3 接管，下位机本次上电不会自动退回 UART5。看到 `/control/status` 的 `link_ready: true` 后，可显式使能并持续发布指令。停止服务返回表示 PC 接受停止请求；下位机接纳结果应查看 `/mcu/state`。刀盘协议只有油门设定反馈，不能据此确认刀盘机械上已经停转。

```bash
ros2 service call /control/set_enabled std_srvs/srv/SetBool '{data: true}'
```

单独使能后必须在 150 ms 内开始发布新鲜 `/cmd_vel`，否则自动撤销使能。通常使用前述控制工具完成这两个操作。

## 参数

| 文件 | 内容 |
|---|---|
| `src/terramind_bringup/config/serial.yaml` | 自动/手动串口、候选路径、扫描监听窗口、115200 波特率、20 ms 发送周期、重连间隔 |
| `src/terramind_bringup/config/control.yaml` | 指令与状态的 150 ms 有效期 |
| `src/terramind_bringup/config/robot.yaml` | 真实轮径、轮距及反馈口径，当前待标定 |
| `src/terramind_bringup/config/sim.yaml` | 明确标注的演示尺寸、运动响应和能力位 |

你后续需要填写真实 `wheel_diameter_m` 和 `wheel_separation_m`。这些参数不参与当前真实串口发送，真实运动控制无需等待机械参数。`robot.yaml` 当前是标定记录，尚未启用真实轮式里程计；其中的零值表示未知，绝不能作为实际尺寸。仿真使用独立的 0.20 m 轮径和 0.45 m 轮距演示配置，这些值不是实际机器人测量值。

## 验证

```bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
ros2 run terramind_sim serial_loopback_test.py
ros2 run terramind_sim serial_discovery_test.py
ros2 run terramind_sim ros_smoke_test.py --mode sim
ros2 run terramind_sim ros_smoke_test.py --mode serial
ros2 run terramind_panel panel_integration_test.py --mode sim
ros2 run terramind_panel panel_integration_test.py --mode serial
ros2 run terramind_panel panel_integration_test.py --mode sim --capabilities 15
```

ROS 进程测试自动创建隔离的 ROS 域，只启动仿真或 PTY 后端，测试结束关闭其启动的进程；不会使用真实串口。`ros_smoke_test.py` 验证控制、作业指令独立超时、停机锁定、非法指令、链路恢复以及控制管理进程崩溃。`panel_integration_test.py` 使用无显示 Qt 窗口操作真实控件，验证所有字段、编辑/应用、停止/撤销使能、界面冻结、链路恢复、使能与停止竞争和关窗停机；可加 `--screenshot /tmp/panel.png` 保存运行截图。

`serial_discovery_test.py` 将扫描范围限定在临时目录内的私有 PTY，验证无设备等待、别名去重、两块控制板歧义、导航端口不接收控制报文、自动握手和控制、拔插后端口重新编号，以及恢复后必须重新使能。

## 当前边界

- 这是运行于真实时间、1 倍速的二维运动模型，`use_sim_time` 必须为 false；没有 Gazebo、碰撞、打滑或草叶切割物理仿真。
- `/sim/ground_truth` 和 `sim_world → base_link` 仅用于仿真观察。当前不发布真实 `/odom`，也未实现导航融合。
- 仿真轮速按车轮轴计算；真实状态的 RPM 轴侧、减速比、线速度反馈语义仍待固件核对。v1 没有独立轮速反馈新鲜度标志。
- 仿真播撒舵机角度暂为 0，占位表示尚未建模；接收错误计数采用饱和计数，具体固件统计口径仍需对照固件。虚拟 UART5 输入固定为零。
- 升降与喷洒在默认 `0x000F` 能力配置下关闭。真实串口没有绕过能力检查的控制入口。
- 物理急停和真实机械停机距离不由这个软件验证。固件 250 ms 保护依赖其控制任务持续运行。
- 本次软件验证与真实控制板联调是两件事；实际 USB 串口、电机方向和真实执行反馈需要现场验证。

架构与接口分别见 `src/doc/PC软件架构.md`、`src/doc/ROS接口约定.md`。原始协议保留不变。

本次验证结果见 `src/doc/首版实现验证记录.md`。
