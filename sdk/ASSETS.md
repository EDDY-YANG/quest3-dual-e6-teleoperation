# 虚拟资产目录

入口是 `packages/12_virtual_assets/catalog.sources.json`。它保存型号 ID、模型加载类型、单位、待登记文件和左右实例的父坐标系及安装变换。生成的 `app/web/asset_catalog.json` 是运行时目录，`GET /api/assets` 同时供 PC 与已配对 Quest 使用。面板、机械臂运动学和舵机固件各自保留独立职责。

| 型号 ID | 资源 | 每侧状态 |
|---|---|---|
| `e6` | 官方 URDF 的 STL 和关节树 | 每臂自己的实测关节角或显式离线参考姿态 |
| `quest-touch-plus` | 厂家 profile 对应的左右 GLB | `xr-standard` 的按钮、追踪和姿态；匹配型号时显示真实模型及 Trigger 动画 |
| `gripper-v1` | 28 个 STEP 实例构成的 GLB、曲柄/连杆/滑块几何 | 左右各一份场景实例，依据同侧目标开度更新。合法硬件 ACK 与测量值单独表示 |
| `three` | 本地版 Three.js 及加载器 | 两侧共享资源 |

## 添加或更换模型

1. 在包 12 的 `files/web/robot-assets/<asset-id>/` 放受审阅的模型和来源/许可。维护 `catalog.sources.json` 中的 `resource_manifest` 或 `include` 文件名单。同种格式的型号可以直接沿用现有 loader。新格式在共享 `asset_store.mjs` 实现一次。
2. 通过 `instances.left/right` 指定 `arm`、`controller`、`tool`。安装变换 `xyz_m` 相对于 `mount.parent`，`rpy_rad` 使用固定轴 XYZ（矩阵 Rz Ry Rx）。显示位置单位为米，当前夹爪机构参数及 TCP 读数使用毫米。
3. 从 SDK 根目录生成目录，再在已停止的本地 SDK 副本中组装。若添加了新的文件，先将其目标相对路径加入对应 `package.json` 的 `files`，以便组装器收录。
   ~~~powershell
   py -3.10 .\packages\12_virtual_assets\tools\build_catalog.py
   powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\sdk.ps1 assemble
   ~~~
4. 浏览器通过 `loadAsset(id)` 和 `loadArmAssembly(side)` 获取资源，PC / HUD 共享 `E6SideModels`。文件从 `/api/assets/<asset-id>/<filename>` 读取，旧静态 URL 从同一清单解析。记录中的 `asset_revision` 标识该次使用的目录和文件内容。更换 mesh 或安装变换会生成新 revision。
5. 到 `/asset-preview` 核对父子装配与开合，再检查 `/monitor` 和 `/hud-preview`。公开分发时复核变更与 `RELEASE_FILES.json`。运行时不服务磁盘上所有文件，原始 STEP、实验视频、密钥和串口配置均在分发清单外。

此型号的三维轨迹由 `mechanism.json` 定义；从 CAD 重建网格需要自行提供有权使用的原始 STEP 和 OpenCascade。安装方法、测试和物理边界见 [GRIPPER.md](GRIPPER.md)。
