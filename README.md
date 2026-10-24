# TerraMind 割草机器人 PC

ROS 2 Humble 工作区，包含 USART3 控制板通讯、导航板接收与坐标换算、控制管理、二维差速仿真、虚拟串口控制板和 Qt 控制面板。PC 按协议发送底盘线速度与角速度，真实下位机负责左右轮分配。所有启动入口默认停机，需要显式使能和持续更新的控制指令。

## 构建

开发者请先阅读 [开发指南](src/doc/开发指南.md) 和 [节点与数据流](src/doc/节点与数据流.md)。前者包含新电脑克隆、依赖安装、源码入口、参数与扩展流程；后者说明三种后端和线程/进程边界。接口类型见 [ROS 接口约定](src/doc/ROS接口约定.md)，验证方法见 [联调与测试说明](src/doc/联调与测试说明.md)。

导航板 v1 已接入：独立 460800 串口、80 字节流式解包、WGS84→固定 ENU、姿态换轴和安装杆臂。启动、原点/外参配置、时间语义及测试见 [导航接入与坐标换算](src/doc/导航接入与坐标换算.md)。导航首版发布独立定位话题，不发布动态 TF；实机安装参数仍需标定。

开发环境：Ubuntu 22.04、ROS 2 Humble、C++17。Python 辅助工具使用系统 Python，不需要 pyserial。依赖由各包的 `package.xml` 声明；标准 ROS desktop 环境还需要 colcon、ament_cmake_gtest、ament_cmake_pytest、python3-yaml、python3-pyqt5、python3-pytest。若缺少面板依赖，可用 `sudo apt install python3-pyqt5 python3-pytest fonts-noto-cjk ros-humble-ament-cmake-pytest` 安装，中文字体用于避免界面显示方框。

```bash
cd TerraMind  # 切换到克隆得到的工作区根目录，路径以本机为准
source /opt/ros/humble/setup.bash
MAKEFLAGS="-j2 -l2" colcon build --symlink-install --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
```

后续每个新终端均需加载 ROS 和本工作区的 `install/setup.bash`。

实际部署到另一台 Linux 主机时，按 [Linux 主机接板部署](src/doc/开发指南.md#linux-主机接板部署) 安装依赖、在目标机重新构建并填写两个串口路径。该流程包含串口权限、导航参数保存和有/无图形界面的启动方式。

本次 Ubuntu22-E（WSL）验证中，默认 Fast DDS 的本机发现/服务通信不稳定，切换 Cyclone DDS 后串口发现和 ROS 控制链路测试通过。该环境安装 `sudo apt install ros-humble-rmw-cyclonedds-cpp`；三个启动脚本会在 WSL 中优先使用已安装的 Cyclone DDS，并保留显式设置的 `RMW_IMPLEMENTATION`。直接运行 ROS 命令或测试前执行 `source scripts/native_gui_env.sh`，确保同一组节点使用一致的中间件。

### Snap 终端中的图形程序启动失败

如果出现 `/snap/core20/.../libpthread.so.0: undefined symbol: __libc_pthread_init, version GLIBC_PRIVATE`，说明系统程序混用了 Snap 内的运行库。Snap 编辑器终端继承的 `GTK_PATH`、GIO/Qt 插件路径或动态库路径可能导致此问题。`start_panel.sh`、`start_sim.sh` 和 `start_debug.sh` 会在加载 ROS 前过滤相关变量中的 Snap 路径，保留其他路径；清理仅作用于启动进程及其子进程，不修改终端或系统配置。脚本修改后直接重新启动，无需编译。

如果直接运行 `ros2` 命令，可先在当前终端执行 `source scripts/native_gui_env.sh`，再加载 ROS 和工作区环境。`Ignoring XDG_SESSION_TYPE=wayland on Gnome` 是 Qt 选择 X11 后端的提示，本身不会造成上述库符号错误，不需要通过强制 Wayland 来处理。

## 图形化调试总面板

日常调试可以直接打开工程中的 `TerraMind-Debug.desktop`（当前工作区路径为 `/home/tang/TerraMind-ROS`），或运行 `./start_debug.sh`。总面板本身只依赖系统 Python 和 PyQt5，无需提前 source ROS，也无需先构建工作区。

- **启动调试环境**：选择二维仿真、虚拟串口或真实控制板，点击“启动所选后端”。可选择同时打开控制面板、RViz、模拟能力位和 ROS 域；真实串口可使用 `auto` 或填写 `/dev/` 下的路径。
- **编译 / 更新工作区**：修改源码后点击编译。启动功能时如缺少关键构建产物，会先编译，成功后自动继续；编译失败或被停止时取消后续启动。
- **运行所选测试**：提供全部包单元测试、串口回环、自动发现、两种 ROS 链路测试、当前板卡/旧板卡/全能力控制面板测试，以及总面板自身的启动/停止联调。进程测试使用隔离 ROS 域和模拟后端；总面板测试还用不存在的设备路径检查手动端口覆盖，不接触真实控制板。
- **查看话题**：选择板卡状态、控制状态、完整命令、诊断或仿真真值，输出显示在下方日志中。
- **程序与日志**：选择任务查看实时输出与退出结果；完整输出保存在 `log/debug/`。停止所选、停止全部和关闭窗口会先请求程序退出，必要时终止该任务的整个进程组；总面板异常退出后，独立监督进程也会清理其启动的任务。

同一工作区只允许一个总面板。总面板启动的三个后端互斥，重复控制面板会被拦截；构建、测试需要先停止其他任务。它只管理自己启动的程序，不会接管其他终端已经启动的节点。“运行中”表示进程正在运行，链路是否就绪以控制面板反馈为准。启动实机模式会打开串口并执行未使能零指令握手，运动仍需在控制面板中显式使能。

停止程序按钮用于结束调试进程；运行中的设备控制仍使用控制面板的“停止全部”。将工程移动到其他路径后，应更新 `.desktop` 文件中的 `Exec` 和 `Path`。

## 无硬件二维仿真

一键启动，无需提前加载 ROS 或工作区环境：

```bash
./start_sim.sh
```

脚本自动定位工程，启动控制管理节点、仿真接口、机器人模型、RViz 和控制面板；缺少构建产物时自动编译。按 `Ctrl+C` 退出。无显示环境会自动关闭所有图形窗口，也可显式指定：

```bash
./start_sim.sh --headless
```

修改源码后使用 `start_sim.sh --build` 重新编译并启动。其他参数会传给原启动入口，例如 `description:=false`。脚本默认将 ROS 日志放在工程的 `log/ros/`，也支持已有的 `ROS_LOG_DIR` 设置。

已有 ROS 环境的终端仍可直接运行 `ros2 launch terramind_bringup sim.launch.py`。

RViz 默认使用 **Move Camera** 工具和 **Orbit** 视角。在三维画面内操作：左键拖动旋转视角，按住 **Shift + 左键** 拖动或中键拖动平移，滚轮缩放，也可以按住右键上下拖动缩放。Views 面板可调整距离、焦点或切换视角；若切换到选择工具，点击工具栏的 **Move Camera** 即可恢复视角操作。这些操作只移动观察视角，不控制机器人。

若使用命令行控制，请关闭面板（或启动时加 `--no-panel`），再在另一个终端运行下面的命令，显式使能并发送 3 秒的新鲜指令，最后请求停止：

```bash
ros2 run terramind_sim control_cli.py --linear 0.10 --angular 0.2 --duration 3
```

同一工具也可控制真实后端，执行前应确认正在使用的启动入口。默认作业装置均关闭；可以显式指定 `--left-rpm`、`--right-rpm`、`--mower-percent`、`--sprayer-percent`。工具每 20 ms 更新时间戳，避免手写一个固定时间戳的命令被有效期检查拒绝。

```bash
ros2 topic echo /mcu/state
ros2 topic echo /control/status
ros2 topic echo /sim/ground_truth
ros2 service call /control/stop std_srvs/srv/Trigger '{}'
```

## 控制面板

### 矩形区域作业

在二维仿真中运行 `./start_sim.sh capabilities:=63`，打开控制面板的“区域作业”页：

1. 停稳并收到至少 0.4 秒稳定定位后，点击“以当前位置建区”。当前位置和朝向锁定为初始位姿，后续实时位置变化不会拖动区域。
2. 按住鼠标左键拖动对角点；矩形坐标 X 沿建区时的车头方向，Y 向左，支持四个拖拽方向。滚轮缩放，也可精确填写对角 X/Y。
3. 输入车宽/有效割幅及速度，点击“生成路线”。横向等分为 `ceil(区域宽度/割幅)` 行，行距不超过割幅；相邻行通过半圆掉头连接。矩形是作业覆盖范围，车身和掉头允许越出区域。
4. 在手动页设置所需刀盘油门与升降缩短量，再回区域页核对“启动设定”。未开启刀盘时仅行驶验证。覆盖直线开启所选刀盘，进入/掉头/返航关闭刀盘；升降目标在任务中保持，停止时按固件行为回零。播撒和喷洒不自动开启。
5. 点击“开始区域作业”。完成覆盖后返回记录的初始位置，再恢复初始朝向；最终位置和角度连续 0.8 秒同时满足容差才完成并撤销使能。规划终点严格等于初始位姿，实际到位使用容差。

默认位置容差 0.03 m、角度容差 0.035 rad（约 2°），在 `control.yaml` 的 `coverage_position_tolerance_m` / `coverage_yaw_tolerance_rad` 配置，重启面板生效。停止全部、定位过期、会话/原点/连接改变、偏离路径和界面卡顿均停止任务；不会在恢复连接后自动续跑。运行中不能应用手动目标或修改区域。

真实板卡使用“导航板 · 实机定位”，订阅现有 `navigation/pose` 和 `navigation/status`。需导航有效、原点就绪、安装外参已标定且位姿参考点为 `base_link`；真实控制板拒绝仿真真值和模拟导航。实机入口与串口配置沿用 [Linux 主机接板部署](src/doc/开发指南.md#linux-主机接板部署)，随后在同一面板操作。独立面板连接仿真时可在区域页手动选择“二维仿真 · 模型真值”；实机默认不使用该来源。

规划、跟踪和定位门控分别位于 `terramind_panel` 的 `coverage.py`、`coverage_tracking.py`、`coverage_runtime.py`，画布位于 `coverage_widget.py`。所有运动指令仍经原控制管理和串口保护链路。

```bash
# 无硬件验收：真实 Qt 控件 + ROS 控制管理 + 带加速度限制的模型。
ros2 run terramind_panel coverage_integration_test.py
# 以导航接入话题运行，同步注入 200 ms 测量延迟和定位失效。
ros2 run terramind_panel coverage_integration_test.py --pose-source navigation
```

测试中的导航输入是模型测量转换的测试消息，不是真实 GNSS；导航串口解包和坐标换算由独立 `navigation_integration_test.py` 验证。运行期间系统时钟需稳定；时间跳变会触发已有新鲜度保护停机，不会忽略时间异常继续作业。

当前 F4 固件已接入升降：初始化成功时上报 bit4，喷洒初始化成功时上报 bit5。面板按实际能力开放控件。`control.yaml` 中 `terramind_panel.lift_max_mm` 限制当前机构可输入的缩短量（默认 38.56 mm，可设更小值）；协议本身仍允许 0～1000 mm。关闭升降或停止全部后，固件逐步回到初始位置，不是保持当前位置。升降与喷洒没有有效测量反馈，模拟后端也按此返回无效反馈，不显示目标为实际位置或流量。

实机模式可在调试总面板勾选导航，指定端口、选择和编辑原点/外参 YAML，并打开 RViz 位姿轨迹。控制面板“导航状态”页显示质量与参考点。独立导航、固定原点复用和参数优先级见 [导航接入与坐标换算](src/doc/导航接入与坐标换算.md)。

测试全部可控装置的一键入口：

```bash
./start_sim.sh capabilities:=63
```

`63`（`0x003F`）使仿真支持全部六类能力。默认 `47`（`0x002F`）包含新增的喷洒模块，刀盘升降仍不支持。旧板卡兼容测试可选择 `15`（`0x000F`），此时升降和喷洒均不支持。该参数只作用于模拟后端，真实控制始终按板卡回传能力判断。

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
| 刀盘升降 | 开关；当前机构缩短量 0～38.56 mm，需后端支持；关闭/停止后逐步回零 |
| 喷洒装置 | 开关；0～100%，需后端支持 |

喷洒模块沿用 v1 的 `0x50` 控制块：勾选“喷洒装置”，设置 0～100% 目标，开始发送或应用修改即可。实机只有在状态帧上报能力 bit5 时才开放控件；`feedback_valid=false` 时显示“反馈无效”，不会把设定值当成实测反馈。

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

导航板位于前轴时，可通过 `navigation.yaml` 的 `imu_translation_in_base_m` 将定位换算到后轴中心；默认值是占位，尚未启用。新增 `ros2 run terramind_panel calibrate_lever_arm --config /绝对路径/navigation.yaml --output /绝对路径/navigation-identified.yaml`，正反各自转一圈辨识水平偏移。高度和安装旋转保留为手动参数。启动、应用结果及纯仿真验证见[后轴偏移与两圈自转辨识](src/doc/导航接入与坐标换算.md#后轴偏移配置与两圈自转辨识)。

| 文件 | 内容 |
|---|---|
| `src/terramind_bringup/config/serial.yaml` | 自动/手动串口、候选路径、扫描监听窗口、115200 波特率、20 ms 发送周期、重连间隔 |
| `src/terramind_bringup/config/control.yaml` | 指令与状态的 150 ms 有效期 |
| `src/terramind_bringup/config/robot.yaml` | 真实轮径、轮距及反馈口径，当前待标定 |
| `src/terramind_bringup/config/sim.yaml` | 明确标注的演示尺寸、运动响应和能力位 |

`robot.yaml` 已记录当前 F4 固件配置：轮径 0.10 m、轮距 0.32 m、减速比 19.2032 和方向修正；这些是源码配置，不代表本轮已完成实测标定。它不参与真实串口发送，尚未启用真实轮式里程计，输出轴到车轮的传动关系仍待现场核对。仿真使用独立的 0.20 m 轮径和 0.45 m 轮距演示配置，这些值不是实际机器人测量值。

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
ros2 run terramind_panel panel_integration_test.py --mode sim --capabilities 47
ros2 run terramind_panel panel_integration_test.py --mode serial --capabilities 47
ros2 run terramind_panel panel_integration_test.py --mode sim --capabilities 15
ros2 run terramind_panel debug_launcher_integration_test.py
```

ROS 进程测试自动创建隔离的 ROS 域，只启动仿真或 PTY 后端，测试结束关闭其启动的进程；不会使用真实串口。`ros_smoke_test.py` 验证控制、作业指令独立超时、停机锁定、非法指令、链路恢复以及控制管理进程崩溃。`panel_integration_test.py` 使用无显示 Qt 窗口操作真实控件，验证所有字段、编辑/应用、停止/撤销使能、界面冻结、链路恢复、使能与停止竞争和关窗停机；可加 `--screenshot /tmp/panel.png` 保存运行截图。

`serial_discovery_test.py` 将扫描范围限定在临时目录内的私有 PTY，验证无设备等待、别名去重、两块控制板歧义、导航端口不接收控制报文、自动握手和控制、拔插后端口重新编号，以及恢复后必须重新使能。

## 当前边界

- 这是运行于真实时间、1 倍速的二维运动模型，`use_sim_time` 必须为 false；没有 Gazebo、碰撞、打滑或草叶切割物理仿真。
- `/sim/ground_truth` 和 `sim_world → base_link` 仅用于仿真观察。当前不发布真实 `/odom`，也未实现导航融合。
- 仿真轮速按车轮轴计算；真实 RPM 为转子 RPM 除以 19.2032 后的减速器输出轴反馈，保留固件左右符号；线速度、角速度是板端应用目标。v1 没有独立轮速反馈新鲜度标志。
- 仿真播撒舵机角度暂为 0，占位表示尚未建模；接收错误计数采用饱和计数，具体固件统计口径仍需对照固件。虚拟 UART5 输入固定为零。
- 模拟默认 `0x002F` 保留无升降兼容场景；`0x003F` 可测试全部装置。当前固件在升降、喷洒初始化成功时上报相应能力，真实串口始终按回传检查。
- 物理急停和真实机械停机距离不由这个软件验证。固件 250 ms 保护依赖其控制任务持续运行。
- 本次软件验证与真实控制板联调是两件事；实际 USB 串口、电机方向和真实执行反馈需要现场验证。

架构与接口分别见 `src/doc/PC软件架构.md`、`src/doc/ROS接口约定.md`。v1 报文字节布局保持不变，协议说明已同步喷洒能力。

本次验证结果见 `src/doc/首版实现验证记录.md`。
