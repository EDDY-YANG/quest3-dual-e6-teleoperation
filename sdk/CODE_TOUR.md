# 源码阅读与修改顺序

先打开 `Quest_E6_SDK.code-workspace`。按下表在 `packages/<包名>/files/` 内查看；同名的 `app/` 文件由组装程序生成。

| 模块 | 主要文件与作用 |
|---|---|
| 01 前端 | `web/quest.js` 读取 WebXR 手柄，`web/quest_hud.mjs` 绘制操作面板，`web/e6_urdf_view.mjs` 根据反馈更新左右模型。 |
| 02 后端 | `server.py` 编排连接，`quest_lan.py` 绑定头显会话，`headset_bridge.py` 处理按钮和状态。 |
| 03 坐标映射 | `direction_calibration.py` 采集每臂水平 +X，`operator_frame.py` 保存前方，`relative_pose.py` 从每次 Grip 锚点生成 TCP 目标。 |
| 04 运动学、05 RangedIK、06 QP | `controller_kinematics.py` 提供 FK 和本地 DLS；`ranged_ik_adapter.py` 与 `profiles/rangedik_world_anchor.json` 定义 v7 求解；`joint_reference_qp.py` 提供历史 QP 参考策略。 |
| 07 运控、08 IO | `left_z_trial.py` 中的每臂实例验证候选，`teleop_group.py` 编排参与侧；`dobot_safe_sdk.py` 发送经校验的命令，`e6_feedback.py` 解析反馈。 |
| 09 功能 | `start_positions.py` 保存现场测量的起点并管理回位，`teleop_recording.py` 写入新会话记录。 |
| 10 运行环境 | `device_settings.py` 检查使用者地址，`sdk.ps1` 和 `quest_e6_sdk/` 提供命令与离线自检。 |
| 11 离线实验 | `experiments/ranged_dls/controllers.py` 中 `ConstrainedDLS` 实现有约束的局部更新，参见 [OFFLINE.md](OFFLINE.md)。 |

更改求解参数后，先停止正在使用该副本的服务、重新组装，并执行工作区离线验证任务。首次实机操作见 [SDK 操作手册](https://eddy-yang.github.io/quest3-dual-e6-teleoperation/docs/OPERATIONS.html)。

## 虚拟资产和夹爪

包 12 集中 E6、Touch Plus 和夹爪网格；`asset_catalog.json` 登记文件与两侧法兰的名义安装。包 13 处理扳机连续值、连杆运动学、Uno 串口和固件。见 [GRIPPER.md](GRIPPER.md)。
