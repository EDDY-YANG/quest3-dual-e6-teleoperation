# Quest / E6 SDK：平台条件与五步代码导读

> 本文保留自单独维护的完整 SDK，描述的是该 SDK 的源码与验证记录。本公开仓库只提供展示网页、文档和统计分析，不包含下文引用的机器人控制源码或 Windows DLL；下载本仓库不能直接运行遥操作。

本导读针对当前交付目录的源码，核对于 2026-11-17。此次只阅读代码、运行无设备的软件检查，不连接 E6/Quest、不发送运动。

## 先回答：Mac / Windows 都只安装 VS Code 就能用吗？

**不能。当前交付是 Windows x64 源码 SDK。VS Code 用于阅读与编辑；程序仍需要 Python、第三方依赖和匹配平台的原生库。**

| 使用层级 | Windows x64 | macOS |
|---|---|---|
| 查看本包源码、文档 | VS Code 即可 | VS Code 即可 |
| 运行完整 SDK、网页与 RangedIK | Python 3.10 x64 + setup 安装固定依赖 + 随包 Windows DLL | 当前包不能直接作为完整 SDK 运行 |
| 纯 Python / NumPy 数学模块 | 安装依赖后可离线运行 | 有移植基础，但尚未在 Mac 实测 |
| 离线混合原型及绘图 | 另装 requirements-offline.txt；读取明确的历史输入 | 原生库、启动与字体路径仍需适配 |
| 真机遥操作 | 还需真实网卡绑定、设备身份、Quest 配对/AR/标定 | 尚未提供完整平台适配及验收 |

本次重新核对的阻碍：

- RangedIK 构建清单 (`packages/05_ranged_ik/files/runtime/rangedik_native/build-manifest.json`) 第 28 行明确为 `x86_64-pc-windows-msvc`；二进制头也实际读到 Windows PE、AMD64（0x8664）。包里没有 macOS `.dylib`。
- 启动器 (SDK source: `sdk.ps1`) 与 START_SDK.cmd (SDK source: `START_SDK.cmd`) 调用 PowerShell、`python`、Windows `.venv/Scripts/python.exe`；包里没有附带 Python 解释器。
- 设备身份核对 (`packages/09_features/files/start_positions.py`) 第 58 行 `neighbor_identity()` 在非 Windows 系统明确报错，并调用 `Get-NetIPAddress / Get-NetNeighbor`。
- USB 路线使用 Windows `adb.exe`；离线绘图有 `C:/Windows/Fonts/msyh.ttc` 路径。
- 当前这个交付目录还没有 `app/.venv/Scripts/python.exe`。此前安装验证用的是独立测试副本；在此目录使用一键启动时仍需首次 setup。

本次无设备复查：使用本机**已安装的 Python 与依赖**，真实 DLL 加载、FK、名义小目标 RangedIK、传统 DLS 基本求解均通过；46 个隔离 HTTP/前端资源路径通过，没有创建机器人驱动。没有 Mac 电脑实测，不能把 Windows 检查外推为 Mac 通过。

若以后要做到“发给别人即可运行”，应分别提供 Windows x64、macOS Apple Silicon（以及明确支持的 Intel Mac）构建产物，统一跨平台启动/身份核对，并选择自带运行时的使用版或自动安装依赖的开发版。VS Code 应当是可选编辑器，不是程序运行前提。以上跨平台工作尚未实施。

## 阅读规则：改 packages，运行统一入口

- `packages/<包名>/files/`：正式编辑位置。
- `app/`：assemble 生成的运行目录，保留原 import 与资源路径。
- 根目录 `sdk.ps1`：你的入口；不要逐个运行 `left_z_trial.py`、`ranged_ik_adapter.py` 或 `dobot_safe_sdk.py`。
- 在 VS Code 中打开下方链接，再按 **Ctrl+G** 输入所写行号。行号对应本次 SDK 快照。

```text
sdk.ps1 start
  → app/sdk.ps1
  → start-rangedik.ps1 设置 E6_RANGEDIK_PROFILE
  → 原启动脚本启动 server.py
  → server.make_app() 装配前端、会话、每臂控制器与功能

Quest 手柄 → quest.js → WebSocket → QuestState
          → Grip 锚点与固定方向映射 → 目标位姿
          → RangedIK 候选 → 控制检查 → ServoJ → ACK
                                                   ↘ 实测反馈回到控制器
```

## 1. 连接设备

先打开 config.json (`packages/10_sdk_runtime/files/config.json`)。这里的目标 IP 与本机源 IP 决定每条链路的地址；两台同目标 IP 的 E6 必须使用分开的物理链路和正确源绑定。保存的地址/备注不能证明当下接线身份。

接着看：

1. SDK 启动分发 (`packages/10_sdk_runtime/files/sdk.ps1`)，44 行：`start` 进入 `start-rangedik.ps1`。
2. server.py (`packages/02_backend/files/server.py`)，448 行 `make_app()`；1123 行 `prepare_one()` 构造对应侧驱动、反馈读取器及 `LeftZTrial`。
3. quest_lan.py (`packages/02_backend/files/quest_lan.py`)：HTTPS/WSS、配对码、来源与会话校验。
4. dobot_safe_sdk.py (`packages/08_robot_io/files/dobot_safe_sdk.py`)，171 行：29999 命令连接，192 行显式 `source_address`。
5. e6_feedback.py (`packages/08_robot_io/files/e6_feedback.py`)，139 行 `FeedbackReader`：30004 反馈读取与有效性判断。

以后实际运行时，在 SDK 根目录依次执行 `sdk.ps1 setup`（首次）、`doctor`、`start`。Wi-Fi 首次还要用 `lan-config -Ip <实际PC地址> -Prefix <实际前缀>` 生成证书，然后 `lan-enable`、`lan-info`。尖括号部分是需要替换的说明，不能原样执行。

设备接好后，`sdk.ps1 connect-feedback -Side left/both` 显式连接反馈；`sdk.ps1 prepare -Side auto` 查询/准备当前在线臂。这里的 left/both 表示二选一参数，不是字面字符串。**现在没接设备，不执行这些连接/准备动作也能继续阅读。**

## 2. 读取 VR 输入

打开 web/quest.js (`packages/01_frontend/files/web/quest.js`)：

- 93 行 `serialiseHand()`：从 `gripSpace` 读取真实控制器位置/朝向，从 `gamepad` 读取按钮和摇杆。
- 139 行 `packet()`：打包左右手、序号、参考空间、可见性和时间戳。
- 175 行 `send()`：通过 WebSocket 发送；积压时丢弃旧采样，避免排队重放。
- 262 行 `frameLoop()`：XR 帧回调；常规发送约 30 Hz，Grip 状态改变可即时发送。
- 后端 server.py (`packages/02_backend/files/server.py`) 的 57 行 `validate_frame()`、135 行 `QuestState`、861 行 `websocket()` 接收和保存最新有效输入。

输入数据主要是位置 **m**、姿态四元数 **xyzw**、Grip/Trigger、`tracking_valid`、`visibility`。四元数（Quaternion）用四个数表达三维旋转，这里不能把其中三项直接当欧拉角。

该 JS 在 **Quest 自己的浏览器**进入 AR 后运行，不在 PC 上用 Python 运行。PC 读取观察入口是 `sdk.ps1 quest`，或在 `app/` 执行 `.venv/Scripts/python.exe -m examples.watch_quest`。后者仅 2 Hz 打印十秒，是观察脚本，不是实际采集频率。

## 3. 坐标映射

打开 relative_pose.py (`packages/03_pose_mapping/files/relative_pose.py`)，193 行 `anchored_pose_target()`；由 left_z_trial.py (`packages/07_motion_control/files/left_z_trial.py`) 1102 行调用。

末端控制点（Tool Center Point，TCP）是机器人控制器用于描述末端目标的位置/姿态点。这里将手柄相对本次握持起点的变化，映射到机器人在握持瞬间的实测 TCP：

```text
p_goal_mm = p_robot_grip_mm + 1000 × C × (p_hand_now_m - p_hand_grip_m)
R_goal    = C × (R_hand_now × R_hand_gripᵀ) × Cᵀ × R_robot_grip
```

`C` 是固定的 XR 世界方向到该机器人基座的方向映射；`1000` 是 m→mm 换算，不是灵敏度旋钮。当前平移/旋转采用 1:1 映射；松开再握持会重新取锚点，旧手柄位移不重放。

方向标定在 direction_calibration.py (`packages/03_pose_mapping/files/direction_calibration.py`)：63 行开始处理，98 行构造矩阵，116 行提供当前有效映射；共享操作者前方在 operator_frame.py (`packages/03_pose_mapping/files/operator_frame.py`)。当前只采该臂基座 +X，固定重力竖直，要求基座水平。

映射后还会经过 tool_orientation_cone.py (`packages/07_motion_control/files/tool_orientation_cone.py`) 的朝下锥约束，然后进入求解。因此要区分“手柄原始旋转意图”和“经过约束后的机器人目标”。

这一阶段由后端自动调用，无独立启动命令。要修改方向、平移比例或旋转跟随规则，应改这里的映射逻辑并明确新增参数，不应靠改 IP、IK 权重或单位换算实现。

## 4. 算法求解

逆运动学（Inverse Kinematics，IK）将末端目标位姿和当前关节状态变成候选关节角；正运动学（Forward Kinematics，FK）把候选关节角算回末端位姿，用来核对误差。

先看 left_z_trial.py (`packages/07_motion_control/files/left_z_trial.py`) 717 行 `_produce_target()`：739 行有 RangedIK 时调用 `ranged.propose()`；745 行否则调用传统 DLS 的 `PoseModel.solve()`。**这是分支，不是默认串联。**

当前在线 v7 路径：

```text
ranged_ik_profile.py 读取配置
  → ranged_ik_adapter.py:propose() / _propose_multistart()
  → ranged_ik_backend.py:NativeRangedIK
  → native_bridge.rs / Rust 原生库
  → 候选 q、误差、是否在范围内、preview ID
```

### RangedIK 参数优先改这个 JSON

profiles/rangedik_world_anchor.json (`packages/05_ranged_ik/files/profiles/rangedik_world_anchor.json`)：

| 位置 | 参数与当前值 | 工程含义 |
|---|---|---|
| 8 行 | `position_tolerance_mm = [1,1,1]` | 目标坐标系三个位置分量范围，不等于位置误差范数 ≤1 mm |
| 13–14 行 | `GEODESIC_ANGLE`、`orientation_tolerance_deg = 2` | 整体旋转角误差范围，不是三个欧拉角各 ±2° |
| 15 行 | `joint_search_radius_deg = [60,…]` | 以实测关节角为中心的搜索范围，不是速度上限 |
| 51 行 | `objective_weights` | 各项优化代价的权重 |
| 62–64 行 | velocity 0.7 / acceleration 0.5 / jerk 0.3 | 软平滑偏好，不是硬性 deg/s 或 deg/s² 限速 |
| 67 行 | `orientation_center = 5` | 靠近姿态中心的偏好；与范围权重分开 |
| 4、6 行 | 多初值 / direct | 求解策略和输出路径，应按对应代码理解后修改 |

求解流程在 ranged_ik_adapter.py (`packages/05_ranged_ik/files/ranged_ik_adapter.py`)：184 行 `propose()`、250 行多初值策略、20–21 行中间姿态比例与 0.08 s 总预算。修改数学目标或原生优化实现才进入 native_bridge.rs (`packages/05_ranged_ik/files/tools/rangedik_native/native_bridge.rs`) 及 Rust objective.rs (`packages/05_ranged_ik/files/vendor/rangedik-source-snapshot/src/groove/objective.rs`)，并按构建文档更新源码/桥接、重编 DLL 与 manifest；不能只改旁观用源码快照就认为 DLL 已变化。

### DLS 分清两个实现

阻尼最小二乘法（Damped Least Squares，DLS）是局部 IK 方法，通过阻尼抑制接近奇异位置时过大的数值增量；不会自动保证脱离奇异或避碰。

**传统 DLS：**controller_kinematics.py (`packages/04_kinematics_dls/files/controller_kinematics.py`)。165 行 `PoseModel.solve()`；20–24 行位置/角度容差、旋转权重、单次数值迭代步长；222 行阻尼候选 `(0.01, 0.1, 1.0, 10.0)`；默认最多 40 次迭代。这里的 2° 是求解器一次迭代步长，不是实机命令限制。位置/姿态容差也用于 `identify()` 的模型检查，不能随意当成单纯响应速度参数。

**离线混合 DLS：**experiments/ranged_dls/controllers.py (`packages/11_offline_algorithms/files/experiments/ranged_dls/controllers.py`)，51 行 `Settings`：

- 53–56 行：名义执行 50 Hz、参考规划 10 Hz、关节速度 60 deg/s、加速度 120 deg/s²。
- 60–62 行：位置/姿态/构型增益。
- 66–68 行：`damping_floor=.002`、`damping_max=.05`、`damping_sigma_width=.05`。
- 80 行 `ReferencePlanner`、114 行 `ConstrainedDLS`；它使用包 11 的 offline_hybrid_v4.json (`packages/11_offline_algorithms/files/profiles/offline_hybrid_v4.json`)。

离线原型的历史位置跟踪与实时性未通过，仍不能接入默认在线控制。正常运行 SDK 会自动运行在线求解模块；离线原型应单独按 `SDK_ALGORITHMS.md` 运行，不能把它放进真机路径试验。

## 5. 下发控制与反馈

打开 left_z_trial.py (`packages/07_motion_control/files/left_z_trial.py`)：

- 819 行：direct 路径保留已检查的 RangedIK 候选；旧 QP 路径在 else 分支。
- 849 行 `before_send()`：再次检查真实反馈、输入年龄、命令跨度、工具姿态与路径。
- 882 行：`self._call("ServoJ", *target_q, t=0.15, aheadtime=50, gain=200, guard=before_send)`。
- 895 行：收到控制器 ACK 并通过队列检查后，才 `ranged.commit(result)` 更新求解历史。

关节伺服目标指令（ServoJ）把六个关节目标交给 E6 控制器；它不是 PC 直接输出电机力矩。通过 dobot_safe_sdk.py (`packages/08_robot_io/files/dobot_safe_sdk.py`) 225 行 `_transaction()` 传输；格式化来自随包官方 `dobot_api.py`。`t=0.15` 是点位运行时间，源代码说明 aheadtime/gain 是控制器参数，不能把 aheadtime 数字直接当网络毫秒。

随后 e6_feedback.py (`packages/08_robot_io/files/e6_feedback.py`) 250 行持续解析真实反馈。teleop_group.py (`packages/07_motion_control/files/teleop_group.py`) 管理参与集合、统一启动与组停止。确认回执（Acknowledgement，ACK）说明控制器回复了命令；真实到达/静止必须看后续反馈。

运行入口是 `sdk.ps1 arm -Side auto` 或 VR 中“开始遥操作”；这可能使能机器人，之后真实 Grip 可以触发运动。`sdk.ps1 stop` 请求停止。**本次没有执行这些命令。**

时间参数不要混淆：`left_z_trial.py` 1157 行约 20 ms 是状态检查等待；1081–1084 行有 0.1 s 下发/窗口间隔；求解、队列、反馈检查还会延后发送。VR 约 30 Hz、状态循环约 50 Hz、实际命令更新率是不同量。

## 常用功能修改索引

| 想修改 | 源码位置 |
|---|---|
| VR 按钮、HUD | quest_hud.mjs (`packages/01_frontend/files/web/quest_hud.mjs`)、quest.js；新增动作还要改后端协议/路由 |
| PC 监控页面 | pc_monitor.mjs (`packages/01_frontend/files/web/pc_monitor.mjs`)、对应 HTML/CSS |
| 新增功能 API | server.py (`packages/02_backend/files/server.py`) + Python SDK client.py (`packages/10_sdk_runtime/files/quest_e6_sdk/client.py`) |
| 方向/Grip 映射 | 包 03；不要修改底层命令字符串代替坐标变换 |
| RangedIK 容差、权重 | 包 05 的 rangedik_world_anchor.json |
| RangedIK 多初值/候选策略 | 包 05 的 ranged_ik_adapter.py |
| 传统 DLS 参数 | 包 04 的 controller_kinematics.py |
| 离线混合 DLS 参数 | 包 11 的 controllers.py:Settings |
| 工具朝下锥 / 初始角条件 | tool_orientation_cone.py (`packages/07_motion_control/files/tool_orientation_cone.py`) 10–11 行，当前 45°/5° |
| 输入过期判断 | headset_bridge.py (`packages/02_backend/files/headset_bridge.py`) 11–12 行，准入 400 ms / 运动 200 ms |
| ServoJ 参数、队列、停止 | 包 07 的 left_z_trial.py、teleop_group.py |
| 回位 / 起点 | start_positions.py (`packages/09_features/files/start_positions.py`)、start_position_presets.json |
| 数据记录 | teleop_recording.py (`packages/09_features/files/teleop_recording.py`) |

## 修改后如何运行

只改 `packages/.../files/`，保存后在根目录运行 `sdk.ps1 assemble`；程序已经运行时先通过所属实例正常 Stop / shutdown。然后 `doctor`、相关离线检查、`start`，刷新 PC/Quest。改纯 Python/JSON 不需要重编 DLL；改 Rust 才需要原生构建。

开发检查只选择本次改动相关的一项或一组。无设备基础检查可在 `app/` 用已安装依赖的 Python 执行 `tools/verify_sdk_delivery.py`；它只允许隔离回环 HTTP 和离线真实求解，不会连接机器人。测试通过仍不证明实体控制表现。

当前是源码导读与平台核查，没有调整任何运动或算法参数，也没有把本包改造成 macOS 发行版。
