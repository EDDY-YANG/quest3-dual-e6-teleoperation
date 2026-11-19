# 01–06 · 在 VS Code 的 PowerShell 终端运行示例

**运行方式是 `python -m`，使用本项目的 `app/.venv`。** 先双击 SDK 根目录的 `OPEN_IN_VSCODE.cmd`。需要装环境请看[安装命令与权限说明](../docs/INSTALL.md)。在 VS Code 选 **终端 → 新建终端 → PowerShell**。以下块按目录和用途分开，05 的 Arm 只在准备进行实机操作时手动执行。

**只在终端起始时做一次。** 新终端应位于含有 `sdk.ps1` 的工作区根目录。先启动本机 SDK 服务，再进入 `app`。如果终端已经在 `app` 且该服务已由你启动，可直接从 01 开始。`Test-Path` 应为 `True`，服务地址为 `127.0.0.1:8765`。

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\sdk.ps1 start -NoBrowser
Set-Location .\app
Test-Path .\.venv\Scripts\python.exe
```

**下面 01–06 的每段命令，都从上一步的 `app` 目录运行。** 同一台 PC 的服务必须保持运行。没有 E6 / Quest 时可以先看 01–03 的未连接状态，或运行安装指南中的离线 DLL 自检。

## 01 读取服务状态

```powershell
.\.venv\Scripts\python.exe -X utf8 -m examples.read_status
```

只读本机 HTTP 状态，输出 `mode`、`teleop` 和 `profile_id`。硬件未接也可运行。若提示无法连接，先核对上面的服务是否已在本机启动。

## 02 观察 Quest 手柄输入

```powershell
.\.venv\Scripts\python.exe -X utf8 -m examples.watch_quest
```

每秒读取两次，共读约 10 秒，输出 `status`、`age_ms`、`valid` 和 `hands`。看到真实手柄位姿需要头显已配对、进入 XR 且追踪有效。未连接时显示空值或等待状态。

## 03 查看左右机械臂反馈

```powershell
.\.venv\Scripts\python.exe -X utf8 -m examples.read_feedback
```

只读后端缓存。想看实测关节角和工具中心点（TCP）位姿，要先填好本人设备地址并接好硬件。例如**只连接左臂反馈**时，显式执行下一行后再运行本节第一条。右臂或两臂按实际接线把 `left` 改成 `right` 或 `both`。

```powershell
.\.venv\Scripts\python.exe -X utf8 -m quest_e6_sdk connect-feedback --side left
```

## 04 准备已连接的机械臂

```powershell
.\.venv\Scripts\python.exe -X utf8 -m examples.prepare_robot --side auto
```

这会对自己配置的真实机械臂建立或重检连接，查询状态并准备模型。检查 `participants` 和每臂的预检结果。单臂可明确传 `--side left` 或 `--side right`。此步骤不会 Arm；尚未接设备时可先运行 `.\.venv\Scripts\python.exe -X utf8 -m examples.prepare_robot --help` 查看参数。

## 05 允许跟随与停止

**05a · Arm（允许遥操作）**。只有在 04 预检通过、头显内相应标定完成、手柄 Grip 已释放并确认现场可进行实机操作后执行。该命令可能使能机械臂，之后有效 Grip 可以触发实际运动。日常也可在头显完成这一步。

```powershell
.\.venv\Scripts\python.exe -X utf8 -m examples.teleop_control arm --side auto
```

**05b · Stop（请求停止）**。需要停止时单独执行。应看到 `stop_evidence.feedback_confirmed` 为 `true`；若命令报停止未确认，应查看现场、程序状态与反馈。

```powershell
.\.venv\Scripts\python.exe -X utf8 -m examples.teleop_control stop
```

## 06 保存短时诊断记录

```powershell
.\.venv\Scripts\python.exe -X utf8 -m examples.capture_diagnostic
```

会写入一段约 2 秒的本地状态记录。正常遥操作已在记录时后端会拒绝重复占用。稍后运行下面一行，在返回结果的 `recording` 中核对 `SAVED` 与 `directory`。没有硬件时保存的是未连接状态。默认目录是当前用户 `Documents/E6_Teleop_Recordings`。

```powershell
.\.venv\Scripts\python.exe -X utf8 -m quest_e6_sdk status
```

完成上机操作、已确认停止并准备关闭自己的本地服务时，在当前 `app` 终端执行以下命令。各例子的编辑源在 `packages/10_sdk_runtime/files/examples/`，修改后使用根目录的 `sdk.ps1 assemble`。

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File ..\sdk.ps1 shutdown
```
