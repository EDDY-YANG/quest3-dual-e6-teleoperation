# Windows 安装与 VS Code 终端

本发行包面向 Windows x64，已验证 Python 3.10.11 x64。双击入口使用 Python 自带的 `venv`、`pip` 和锁定的依赖文件。日常在 VS Code 的 **PowerShell** 终端使用随包环境，命令无需 `uv run` 或 Linux `sudo`。

| 使用场景 | 需要什么 |
|---|---|
| 阅读并运行 Windows SDK | Windows x64、Python 3.10 x64、Windows PowerShell（系统自带）。桌面 VS Code 和 Microsoft Python 扩展提供本指南使用的编辑体验。首次 setup 要能取得 Python 包。 |
| 本地服务、Quest Wi-Fi、在线求解 | `setup` 自动安装 aiohttp、NumPy、Requests、cryptography、pySerial 3.5 及其锁定依赖。WebXR 前端、Three.js 资源与 RangedIK Windows DLL 已装入 ZIP。 |
| 额外的离线研究回放 / 开发检查 | 离线研究按需安装 SciPy 和 Matplotlib。Ruff 为可选开发依赖。普通使用无需 Rust、VS C++、ROS、CUDA、Node 或 uv。 |
| 真机运行 | 需要自己的 E6 控制器、匹配的网卡和地址、Quest 3 与控制器。局域网模式需要 PC WLAN 地址和本站证书，USB 方式才需要 ADB。 |

## 1. 安装系统工具（新 Windows 电脑）

已装好 VS Code 与 Python 3.10 x64 的读者直接跳到第 2 步。其余读者在 **普通 PowerShell** 中执行下面两条 WinGet 命令。若系统没有 `winget`，可用 [微软 VS Code 安装程序](https://code.visualstudio.com/download) 和 [Python 官方 Windows x64 安装程序](https://www.python.org/downloads/release/python-31011/)；安装 Python 时选中添加到 PATH 或安装 Python Launcher。

```powershell
winget install --exact --id Microsoft.VisualStudioCode --scope user --source winget
winget install --exact --id Python.Python.3.10 --version 3.10.11 --architecture x64 --scope user --source winget
```

安装完成后重新打开终端，让 PATH 生效。下面的检查应显示 `3.10.x` 和 `win-amd64`。若安装时只启用了 `python` 命令，可把第一行的 `py -3.10` 换成 `python`。

```powershell
py -3.10 -c "import sys, sysconfig; print(sys.version.split()[0], sysconfig.get_platform())"
code --version
code --install-extension ms-python.python
```

## 2. 解压、组装并自检

完整解压到自己的新目录，打开里面含有 `OPEN_IN_VSCODE.cmd` 和 `sdk.ps1` 的文件夹。双击 `OPEN_IN_VSCODE.cmd` 即可。也可以在该目录的 PowerShell 输入同一入口：

```powershell
.\OPEN_IN_VSCODE.cmd
```

如只想在终端安装而暂不打开编辑器，在同一根目录运行下列命令。第一行已包含首次组装、建立 `app/.venv`、按锁安装、`doctor` 检查；第二行查询依赖一致性。无需手动激活虚拟环境。

```powershell
powershell.exe -NoLogo -NoProfile -ExecutionPolicy Bypass -File .\tools\setup_vscode.ps1
.\app\.venv\Scripts\python.exe -m pip check
```

`-ExecutionPolicy Bypass` 只作用于本次启动的 PowerShell 进程。打开工作区后，如果顶端显示“受限模式”，请先核对 ZIP 来源和校验值；确认信任后在横幅的“管理”中信任此工作区以使用任务。工作区推荐 `ms-python.python`。若状态栏选错 Python，用 `Python: Select Interpreter` 指向 `app/.venv/Scripts/python.exe`。

## 3. 没有硬件时能运行哪些检查

在**工作区根目录**运行。第一条检查环境与文件，第二条会在本机加载真实 DLL，测试公开 URDF 的名义求解和隔离的回环页面。首次安装时设备地址、ADB 和 WLAN 证书未配置会出现提示。

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\sdk.ps1 doctor
.\app\.venv\Scripts\python.exe -X utf8 .\app\tools\verify_sdk_delivery.py
```

启动本地服务后即可运行例 01–03，未连接设备时应如实显示等待、无有效手柄或无反馈。六个例子的逐条命令在[示例运行指南](SDK_EXAMPLES.md)。

## 4. 按使用方式补装

以下命令均从 SDK **根目录**执行。常规 Wi-Fi 遥操作只用第 2 步的依赖。研究回放需要使用者自己的离线模型与记录。

```powershell
# 仅做约束 DLS / RangedIK 离线实验时
.\app\.venv\Scripts\python.exe -m pip install -r .\app\requirements-offline.txt

# 仅安装开发检查工具时
.\app\.venv\Scripts\python.exe -m pip install -r .\app\requirements-dev.txt

# 仅需 USB / ADB 方式时，验证官方 Platform Tools 归档后才解压
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\sdk.ps1 setup -IncludeUsb
```

Qt、Visual Studio C++ 与 Rust 都不是运行预编译 DLL 的前提。需要自行**重新编译** RangedIK 时，使用带 MSVC、Windows SDK 和兼容 Rust 工具链的 x64 开发者终端，见 `packages/05_ranged_ik/files/tools/rangedik_native/build_native.ps1`。

## 5. Windows 的管理员命令与 Ubuntu 的 sudo

Windows 的普通安装与自检用当前用户权限。给自己的 Quest 配置 WLAN 证书和监听时，先确认 PC 的实际 WLAN IPv4 和子网前缀，然后在 SDK 根目录运行：

```powershell
$wlanAddress = Read-Host 'This PC WLAN IPv4'
$wlanPrefix = [int](Read-Host 'WLAN subnet prefix length')
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\sdk.ps1 lan-config -Ip $wlanAddress -Prefix $wlanPrefix
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\sdk.ps1 start -NoBrowser
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\sdk.ps1 lan-enable
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\sdk.ps1 lan-info
```

仅在 Windows 防火墙阻断本人已配置的 Quest 监听时，才在**管理员 PowerShell** 回到同一 SDK 根目录运行下面一条。脚本按 `lan-config` 生成的地址、子网、端口和程序路径设置规则。配对码需要更新时，用普通终端按操作手册运行 `sdk.ps1 lan-renew`，当前会话可能需要重新配对。

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\app\2026-10-29-enable-lan-firewall.ps1
```

如果你坐在 **Ubuntu** 电脑上，只想解压、阅读源码或编辑文档，可在 Ubuntu 终端安装下列工具。这是 Ubuntu 的 `sudo` 安装命令。Windows DLL 和 PowerShell 设备功能仍要在 Windows x64 上运行；未在 Ubuntu 验收该 SDK 的实机功能。

```bash
sudo apt update
sudo apt install -y unzip snapd
sudo snap install --classic code
code --install-extension ms-python.python
```

[WinGet 安装选项](https://learn.microsoft.com/en-us/windows/package-manager/winget/install) · [Python 3.10.11 的 WinGet 清单](https://github.com/microsoft/winget-pkgs/blob/master/manifests/p/Python/Python/3/10/3.10.11/Python.Python.3.10.installer.yaml) · [VS Code 的 Linux 安装说明](https://code.visualstudio.com/docs/setup/linux)
