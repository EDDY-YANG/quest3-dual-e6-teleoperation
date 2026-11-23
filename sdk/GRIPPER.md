# 双 E6 · Quest Trigger 连续夹爪（v0.2.0）

**当前方案为每个夹爪各一块 Arduino Uno、每块板驱动一只 XINHUI 鑫辉 20KG 升级款位置舵机。** 默认只计算虚拟开度。未来 PCB 可实现同一驱动接口、更换传输方式；本版实现 USB 串口至 Uno。

## 下载后先看哪里

### 本机独立夹爪的松手和可选位置 PI

独立模式处理中，食指读数降至 0.02 或以下时立即请求标定的全开脉宽，超过 0.05 后恢复比例目标。行程中段沿用原来的线性脉宽映射。驱动只保留最新有效目标；全开无需等待闭合到位。STOP、断连、跟踪失效或 E6 保护优先撤销输出。需要重新操作时，先恢复故障，再显式启用。夹爪释放与 E6 回开始位是两项独立操作。

三线舵机的默认配置使用直接脉宽。可选的比例积分（PI）外环需要一个能测出夹指实际行程的传感器。当前实现支持线性模拟位置反馈。校准时分别记录两端的实测指尖间距和 ADC 读数，方向可反。读数应位于 ADC 两个电源轨之间，传感器接口应使断线有可识别的电平。单通道模拟量无法识别仍落在有效量程内的所有故障。

接入传感器时，在 Uno 草图已有的板型、侧别、D2、脉宽端点编译参数上增加 `-DGRIPPER_FEEDBACK_PIN=A0`（仅当现场接 A0）。用 `python -m gripper_bench adc --side right --port COM_PORT` 可在 SDK 已停止、舵机输出关闭的台架上读取原始码。默认固件不会读取未接的模拟脚。

设置位于 `runtime/grippers.local.json` 对应侧的 `position_pi` 对象。省略此项或设 `{"enabled":false}` 即使用原驱动。启用时填写 `enabled=true`、`closed_adc`、`open_adc`、`closed_gap_mm`、`open_gap_mm`、`kp_us`、`ki_us_s`、`max_trim_us`。ADC 与间距来自实测。增益按满行程归一化误差计算，单位分别为 µs 和 µs/s。先用 `ki_us_s=0` 验证修正方向，再针对小的稳态残差调积分。保留 `independent_trigger=true`、`slew_us_s=0`。

Uno 大约每 20 ms 读取位置，在原前馈上加有界修正，指令到来时立即更新目标。积分采用抗饱和，反向误差、全开/全闭、停机均清除积累。全开和全闭直接使用现场标定端点。回执分别报告接受的基准脉宽、实际写入的脉宽和实测 ADC；PC 换算毫米并检查新鲜度。传感器越界或指令超时会停止脉冲。接触物体时没有力反馈，PI 不能充当夹持力控制器。

### 启动 SDK

1. Windows x64 安装 Python 3.10 和 VS Code，完整解压后双击 `OPEN_IN_VSCODE.cmd`。在工作区根目录的 PowerShell 启动服务：
   ```powershell
   powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\sdk.ps1 start -NoBrowser
   ```
2. 电脑打开 [夹爪装配预览](http://127.0.0.1:8765/asset-preview)，独立拖动左、右 Trigger。两侧展示厂家 E6 模型、夹爪细节和模拟输入。这里走只读几何求值接口，不会写串口，也不会虚构关节反馈。
3. 打开 [PC 监控](http://127.0.0.1:8765/monitor) 或 [头显镜像](http://127.0.0.1:8765/hud-preview)。实际 Quest 通过配对网页进入 AR/VR，使用相同左右 E6 装配。对准空处先松一下 Trigger，再渐按；松开对应张开、按到底对应闭合。Grip 仍控制机械臂跟随。指向中央 HUD 时 Trigger 归界面使用，离开后需要看到释放再开始下一次夹取。
4. 运行无设备检查：
   ```powershell
   .\app\.venv\Scripts\python.exe -X utf8 .\app\tools\verify_sdk_delivery.py
   Set-Location .\app
   .\.venv\Scripts\python.exe -X utf8 -m unittest discover -s tests -p 'test_gripper*.py' -v
   ```

## 装配和开度如何得到

`packages/12_virtual_assets` 是单一虚拟资产目录。E6 的末端位置来自 Link6 视觉网格的安装平面（局部 +Z 19.0 mm）。夹爪使用用户提供的 STEP，安装面位于原 STEP Z = −67.6 mm。两组四孔中心均约为 (±17.68, ±17.68) mm，名义节圆直径 50 mm。两臂各有一个独立模型实例，连接在各自 Link6，移动和旋转都继承第六轴。当前采用孔对齐的零偏航装法；重复四孔的 90° 分度、实物接插件方向与真实工具偏置需要现场确认。SDK 的控制 TCP、IK 和避碰模型保持基线定义，渲染网格仅用于可视化。

28 个 STEP 零件实例按固定件、曲柄、两侧滑动件和两根侧连杆分组，视频用于辨认装配方向和开合形式。轴销中心从 STEP 圆柱面提取。偏心半径约 25.0 mm、连杆中心距约 29.228 mm，滑块沿 X 轴运动。后端对单调装配分支求逆，使扳机的一半行程对应一半**模型指距**，并同步更新滑块、爪指、轴承和连杆。虚拟工作段取设计闭合姿态至曲柄转动 75°，仅为 CAD 预览行程。实际舵盘连杆位置、夹持间距、载荷和硬限位需标定。CAD 里的舵机外壳仍是设计者使用的 MG996R 包络，实装型号来自用户提供的 XINHUI 产品照片。

## 接真实 Uno（可选，先无负载）

台架识别、逐步点动和卡尺记录表见 [BENCH_TEST.md](BENCH_TEST.md)。虚拟资产的登记和版本规则见 [ASSETS.md](ASSETS.md)。

每块板的一路 D9 输出舵机控制信号，板卡通过 USB 与 PC 相连。舵机使用独立、限流且与实际型号匹配的电源，信号地与对应 Uno 共参考。商品图标注 5–8.4 V，8.4 V 时单只堵转电流标称 2.3 A。不要把舵机功率输入接 Arduino 的 5 V/USB 或 E6 的 24 V 输出。20KG 是产品扭矩标识，不能换算为已经验证的夹持力。此版本不能用 E6 的硬件急停代替独立舵机断电措施。

1. 确认是目标 Uno 型号，本版编译验证对象为 ATmega328P Uno（`arduino:avr:uno`）。在 Arduino IDE 打开 `app/firmware/quest_gripper/quest_gripper.ino`。左板设置 `GRIPPER_SIDE 1`，右板设置 `GRIPPER_SIDE 2`，分别选择对应端口后由操作员上传。默认 0 不接受 Arm。需要 Arduino AVR core 和 Arduino Servo 库。
2. 空载逐侧测出可靠的闭合/张开脉宽，留机械余量，记录每秒允许变化的脉宽。软件允许的保守边界是 1000–2000 μs，实际使用范围和方向由各自机构确定。首次使能前将机构置于已标定的张开端；无位置传感器，首次加脉冲不能从未知实测位置平滑起步。
3. 在 SDK 根目录创建本地配置。新版 setup 已安装 pySerial 3.5；下方 pip 命令也可用于单独修复驱动依赖。
   ```powershell
   .\app\.venv\Scripts\python.exe -m pip install -r .\app\requirements-gripper.txt
   New-Item -ItemType Directory -Force .\app\runtime | Out-Null
   if (-not (Test-Path .\app\runtime\grippers.local.json)) { Copy-Item .\app\grippers.example.json .\app\runtime\grippers.local.json }
   ```
   只在本机文件填写每侧不同的 COM 口、`closed_us`、`open_us`、`slew_us_s`，准备好后把对应侧 `enabled` 设为 true，正常停止再启动该 SDK 服务。模板没有端口或舵机端点，默认不会探测、打开或刷写任何板卡。
4. 物理输出另需原有遥操作组显式 Arm、参与侧就绪、有效新鲜 E6 反馈、有效头显输入和释放 Trigger。板卡握手验证 L/R。首次连接等待松开扳机，首先发送已标定的张开脉宽。每个板卡独立看门狗 300 ms，输入超时或停止会撤销新指令，断开时看门狗停止输出脉冲。这可能释放负载，请按空载方案验收。异常 ACK 不重放，重新检查后显式停止/重新 Arm。
5. 按释放、中间和全按行程分别观察**两只**夹爪，用卡尺记录间距，同时检查 UI 点击不合爪、断线后不保持旧运动指令。一般三线位置舵机没有独立夹爪编码器，本版 `measured_gap_mm` 始终为空。串口 ACK 表示 Uno 接受了目标 PWM，不能证明机械到位或力矩。

## 数据接口

- `GET /api/assets` 返回版本、内容哈希、E6、Touch Plus、夹爪和渲染库资源，以及左右安装关系；`GET /api/assets/{asset_id}/{name}` 仅提供登记文件。PC 与已配对的 Quest 使用同一份目录。旧 `/static/robot-assets/…` URL 继续从同一目录解析。
- WebXR `xr-standard` 的 `buttons[0].value` 是同侧模拟 Trigger，`buttons[1]` 是 Grip；旧版客户端缺少 UI 占用标记时夹爪不接受它的输入。只有新鲜、可见且位置有效的控制器能更新目标。
- PC `/api/monitor/status`、`/api/status`、头显 `/api/headset/status` 与 WSS 遥测都含 `grippers.arms.left/right`。关键字段为 `trigger`、`target_open_fraction`、`model_gap_mm`、`model_pose`、`drive.status`、`drive.ack_us`、`measured_gap_mm`。正常 SDK 录制经已有缓冲队列按侧保存 `gripper_reference`，附资产 revision 和输入序号；连续数值变化最多约 10 Hz 记录，状态切换另记。
- 驱动边界为 `offer(lease, input_seq, crank_fraction, expires, neutral)` / `snapshot()` / `close()`。后续 PCB 可以实现这三个操作并返回自己的反馈证据。当前唯一实物后端是每侧一个 USB 串口的 Uno。
- 公开包分发渲染所需的 STEP 派生网格、关节定义和资源哈希。原始 CAD、原视频、商品截图和任何本机串口配置均未打包。

[WebXR gamepad 映射](https://www.w3.org/TR/webxr-gamepads-module-1/#xr-standard-gamepad-mapping) · [Arduino Servo 库](https://docs.arduino.cc/libraries/servo/)

## 固件开发验证

用 Arduino CLI 和 Servo 库可分别编译左右 Uno。以下为编译命令，不涉及刷写或选择真实串口（从 SDK 根目录执行）。

~~~powershell
arduino-cli core install arduino:avr@1.8.6
arduino-cli lib install Servo
arduino-cli compile --fqbn arduino:avr:uno --build-property compiler.cpp.extra_flags=-DGRIPPER_SIDE=1 --output-dir .\build\uno-left .\app\firmware\quest_gripper
arduino-cli compile --fqbn arduino:avr:uno --build-property compiler.cpp.extra_flags=-DGRIPPER_SIDE=2 --output-dir .\build\uno-right .\app\firmware\quest_gripper
~~~

开发者还可将**实际编译的 HEX** 放入 AVR8js 的 ATmega328P 指令集模拟器，由 Python 的 Uno 串口协议逐条驱动模拟 UART，并读出模拟 D9 的脉冲。Node.js 与 AVR8js 仅用于此验证。

~~~powershell
npm install --prefix .\build\avr-test --ignore-scripts avr8js@0.21.1
.\app\.venv\Scripts\python.exe -X utf8 .\app\tools\verify_gripper_firmware.py --left .\build\uno-left\quest_gripper.ino.hex --right .\build\uno-right\quest_gripper.ino.hex --avr8js-path .\build\avr-test\node_modules\avr8js
~~~

本次验证涵盖三个不同的模拟脉宽、错侧拒绝、重复序号和看门狗。模拟波形属于固件软件验收，不等同于示波器实测。
