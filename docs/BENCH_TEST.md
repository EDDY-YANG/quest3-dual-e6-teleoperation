# 明日空载验收 · 左右各一块 Uno

按左板、右板、双侧、最后才是 E6 跟随的顺序做。运行时将真实 COM 口和测量结果保存在自己的工作区；本表不预填任何实物测量数值。

| 现场记录 | 左夹爪 | 右夹爪 |
|---|---|---|
| 实物板型 / 端口 / 固件侧别 | 待填 | 待填 |
| 选定供电电压 / 电流限制 | 待填 | 待填 |
| 安全张开 PWM / 实测指距 | 待填 | 待填 |
| 安全闭合 PWM / 实测指距 | 待填 | 待填 |
| 中间 PWM / 实测指距 | 待填 | 待填 |
| 法兰绕轴装向 / 装夹间隙 | 待填 | 待填 |
| 失联响应 / 独立断电验证 | 待填 | 待填 |

1. **断电查装配。** 核对 CAD 四孔方向、固定与防松、滑轨顺畅、两指不与腕部碰撞。模型假定 +Z 从法兰向指尖、沿 X 开合。四孔不能唯一确定实物偏航；若相差 90°，在资产目录中修改相应实例的 `mount.rpy_rad` 后重建。TCP、质量/质心与实物避碰属于另外的实机核对项。
2. **单板逻辑验收。** E6 保持停止，退出占用同一 Uno 端口的 SDK 和串口监视器。每个 Uno 只接它自己的 D9 信号和信号地。先断开舵机功率，确认独立供电、板型、线序、极性和急停/断电手段。把左/右固件上传到相应板，再运行以下命令。`identify` 打开串口会使典型 Uno 复位，验证侧别并使脉冲输出关闭。

```powershell
Set-Location .\app
.\.venv\Scripts\python.exe -m gripper_bench ports
$leftPort = Read-Host '左板实际 COM 口'
$rightPort = Read-Host '右板实际 COM 口'
.\.venv\Scripts\python.exe -m gripper_bench identify --side left --port $leftPort
.\.venv\Scripts\python.exe -m gripper_bench identify --side right --port $rightPort
```

3. **单只舵机卸载试动。** 初次确定方向建议先卸下舵盘或脱开连杆。有条件时用示波器先确认 D9 脉冲。从实物说明或已确认的空载位置取得一个安全起始脉宽，设置小范围单向边界。以下示例只在输入 `JOG LEFT` 后使能；`+` 每次只朝边界迈一步，`-` 向起始值退一步，空格/`q` 停止。4 秒无人操作会停脉冲。这是独立台架模式，不要求先 Arm E6，也不应与运行中遥操作并用。

```powershell
$startUs = [int](Read-Host '已验证的空载起始脉宽，微秒')
$limitUs = [int](Read-Host '本次确认的单向小范围边界，微秒')
.\.venv\Scripts\python.exe -m gripper_bench jog --side left --port $leftPort --start-us $startUs --limit-us $limitUs --step-us 5
```

右侧同理改成 `--side right --port $rightPort`，按右侧实际端点重新输入起始值与边界。安装舵盘后先用空载小步观察齿向和机械余量，用卡尺逐侧记录三个位置。若卡住、发热、欠压复位或碰到硬限位，立即切断舵机电源。

4. **保存两侧标定。** 创建模板后填入各自的 `port`、`open_us`、`closed_us`、`slew_us_s` 并按需设 `enabled: true`。开闭数值允许正向或反向排列。接线记录与校准值只在本机保存。

```powershell
.\.venv\Scripts\python.exe -m gripper_bench init
code .\runtime\grippers.local.json
.\.venv\Scripts\python.exe -m gripper_bench check
```

5. **先虚拟，再双侧空载。** 正常运行 SDK，在预览页确认左右扳机独立。Quest 中指向面板做 UI 点击、释放、移出面板，核对没有把 UI 点击当成夹取。用现场确认过的 E6 反馈、校准与 Arm 流程使能对应侧，先松开 Trigger。打印或观察 `drive.ack_us`，用卡尺对照真实间距。做断开 USB、追踪丢失、显式 Stop 和重新 Arm 试验时确保无负载。若 PWM 停止后夹爪松脱，属于当前舵机断脉冲模式的可能结果。USB 断电、机械臂急停与舵机功率切断的关系应单独实测。

6. **最后再测真实移动。** 用低风险小位移验证 Grip 只管手臂、Trigger 只管同侧手爪。检查新增夹爪质量、末端长度、TCP 与两臂工作空间。机构卡死、电流、实际夹持力、夹持物保持和实体停止距离均以实测为准。

软件参考验收见 [GRIPPER.md](GRIPPER.md)。实际源码和 [版本变更记录](../CHANGELOG.md) 随 ZIP 提供。
