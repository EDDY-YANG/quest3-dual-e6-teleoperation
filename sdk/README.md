# Quest 3 / Dual DOBOT E6 SDK for Windows x64

**v0.3.0 · Yang Taige · The University of Hong Kong · ME capstone project**

[双臂夹爪与两块 Uno](GRIPPER.md) · [逐侧空载测试和标定表](BENCH_TEST.md) · [统一资产接口](ASSETS.md) · [更新日志](CHANGELOG.md)

## 首次打开

新电脑的[完整 Windows 安装命令与可选 sudo 说明](packages/10_sdk_runtime/files/docs/INSTALL.md)和 [01–06 的 VS Code 终端命令](packages/10_sdk_runtime/files/examples/README.md)随包提供。

1. 安装桌面版 [VS Code](https://code.visualstudio.com/download) 和 [Windows x64 Python 3.10](https://www.python.org/downloads/release/python-31011/)（本次参考版本为 3.10.11）。完整解压到一个新目录，例如 `C:\QuestE6`。首次安装依赖需要可用的 Python 包下载源。
2. 在含有 `sdk.ps1` 的目录双击 **`OPEN_IN_VSCODE.cmd`**。它找到兼容的解释器，把 13 个源码包组装到 `app/`，建立 `app/.venv`，安装锁定依赖并运行 `doctor`，然后打开 `Quest_E6_SDK.code-workspace`。
3. 在 VS Code 里按需确认工作区信任并安装推荐的 Microsoft Python 扩展。解释器应为 `app/.venv/Scripts/python.exe`。`Terminal → Run Task → SDK: Verify native solver offline (Windows x64)` 可实际加载 DLL 并检查本地页面。此步骤使用厂家公开 URDF 的名义法兰模型，适合无设备验收。
4. 日常编辑直接打开工作区。`packages/` 是编辑源，`app/` 是运行副本。服务停止后用 `powershell -NoProfile -ExecutionPolicy Bypass -File .\sdk.ps1 assemble` 重新组装。

## 绑定自己的设备

刚解压时没有机器人地址、回位目标、证书或配对码。`doctor` 会提示设备绑定、USB ADB 或局域网证书尚未准备好，文件/依赖检查可独立完成。先在 SDK 根目录安装，再运行下面命令生成本人使用的本地文件。

```powershell
.\app\.venv\Scripts\python.exe .\app\device_settings.py --init
```

打开 `app/runtime/devices.local.json`。对每个使用的机械臂，`left_e6_ip` / `right_e6_ip` 填机器人控制器地址，`left_e6_local_ip` / `right_e6_local_ip` 填同侧 PC 网口地址。整侧不用就保留两个 `null`。如果两台控制器地址相同，必须接在相互隔离的物理链路上并使用不同的 PC 源地址。用下列命令检查自己的配置。

```powershell
.\app\.venv\Scripts\python.exe .\app\device_settings.py --check
powershell -NoProfile -ExecutionPolicy Bypass -File .\sdk.ps1 doctor
```

在准备好现场操作后启动服务，按[详细操作手册](https://eddy-yang.github.io/quest3-dual-e6-teleoperation/docs/OPERATIONS.html)操作 PC 与 Quest。同侧机器人身份、工具和工区需在现场核对。先在 PC 控制台保存该机械臂的**实测**开始位，头显内才可请求回到该起点。左右臂独立保存。`START_SDK.cmd` 是运行服务的入口，启动 VS Code 用 `OPEN_IN_VSCODE.cmd`。

## 代码与数据

从 [CODE_TOUR.md](CODE_TOUR.md) 按连接、VR 输入、映射、求解、下发、反馈与记录的顺序读。`packages/05_ranged_ik` 提供预编译 Windows x64 DLL、锁定的 Rust 源码与构建说明。`packages/11_offline_algorithms` 保留受约束 DLS / RangedIK 回放代码，输入使用研究者自己的模型和录制，见 [OFFLINE.md](OFFLINE.md)。在线默认仍是 RangedIK v7 direct。

机器人运行日志、设备地址、实测回位、设备识别模型及会话密钥均由使用者在自己的机器上生成。物理运动、避碰、实机回位和停止性能需要每个现场单独验收。macOS 可以阅读源码，随包原生库面向 Windows x64。

原创代码采用 [MIT](LICENSE)。第三方模型、数值库与依赖的许可见 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md) 和 `licenses/`。
