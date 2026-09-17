# SNP 打磨仿真 · 换成 Dobot CR12A 机械臂（cr12a 分支）

在 [SNP Automate 2023](https://github.com/ros-industrial-consortium/snp_automate_2023) 打磨仿真里，
**把机器人从安川 Motoman HC10DT 换成越疆 Dobot CR12A**，并在新构型下重算扫描轨迹、调快打磨节拍。

> 本分支工作日期：2026-09-17
> 上一个分支（zhixingceng，工件替换为坐面）的记录见 [`docs/zhixingceng_seat_polishing.md`](docs/zhixingceng_seat_polishing.md)

---

## 快速启动

```bash
cd snp-automate-2023-polishing-simulation
bash scripts/restart_demo.sh
```

仿真模式默认全开：`sim_robot=true` / `sim_vision=true` / `bypass_execution=true`。

> ⚠️ 启动后**等约 5 秒**（`start_reconstruction` 服务就绪）再点按钮，否则报 `unreachable`。

---

## 本次做了什么

### 1. 机械臂模型替换

| 项 | 原来 | 现在 |
|---|---|---|
| 机器人 | Motoman HC10DT | Dobot CR12A |
| URDF | `urdf/hc10dt_macro.xacro` | `urdf/cr12_macro.xacro`（新增） |
| 关节名 | `joint_1_s` … `joint_6_t` | `joint1` … `joint6` |
| 模型网格 | 复用 | `meshes/cr12/`（`base_link0.ply` + `j1..j6.ply`） |

`urdf/cr12_macro.xacro` 是 Dobot 官方仓库
[`DOBOT_6Axis_ROS2_V4`](https://github.com/Dobot-Arm/DOBOT_6Axis_ROS2_V4) 里 CR12 URDF 的忠实移植：
去掉官方模型里的 `dummy_link`，补上 `tool0` / `flange` 两个坐标系，并参数化 `prefix`。

配套改动：

- `urdf/workcell.xacro` —— 换 `<xacro:include>`、换机器人实例名，重新对 `table_to_base` 定位
- `config/workcell.srdf` —— `<robot name="cr12_robot">`，`manipulator` 组链改为 `base_link → tool0`
- `config/controllers.yaml`、`config/app.rviz`、`config/rviz_base_config.rviz` —— 关节名同步
- `meshes/cr12_stl/` —— 官方原始 STL（本地保存，未提交，用于重新导出 PLY）

**关节限位收窄**：官方 CR12 URDF 里 J1/J2/J4/J5/J6 写的是 ±6.28 rad（即 ±360°），
对 MoveIt / Descartes 来说这种「能转整圈」的关节会让解空间炸开、求解变慢甚至无解。
本分支按「单圈可达足够用」的原则统一收窄到 **±π**，J3 保持官方 ±2.79：

| 关节 | lower / upper | velocity |
|---|---|---|
| joint1 / joint2 | ±3.14159265 | 3.14 |
| joint3 | ±2.79 | 3.89 |
| joint4 / joint5 / joint6 | ±3.14159265 | 3.89 |

effort 统一 100。`urdf/ros2_control.xacro` 的 `command_interface` 上下限同步。

### 2. home 位统一为「全零位」

按需求，扫描**从零位启动、结束时回到零位**，打磨也**从零位接着开始**。

这一步踩了个大坑：home 位在工程里被 **4 个地方**分别定义，任何一处不一致，
行为树节点 `UpdateTrajectoryStartState` 就会报
`Joint 'joint2' difference from start state (0.5204 radians) exceeds tolerance (0.00174533 radians)` 而失败。

| # | 文件 | 参数 |
|---|---|---|
| 1 | `launch/start.launch.xml` | `home_state_joint_positions` |
| 2 | `launch/test.launch.xml` | `joint_state_publisher_gui` 的 `zeros.jointN` |
| 3 | `urdf/ros2_control.xacro` | `<state_interface name="position">` 的 `initial_value` |
| 4 | `config/scan_traj.yaml` | 轨迹的**首点和尾点** |

**改 home 位时这 4 处必须一起改。** 目前 4 处均为 `[0, 0, 0, 0, 0, 0]`。

> **关于全零位是奇异构型**：全零位下雅可比秩只有 3（J2/J3/J4/J6 的轴同向），
> 几何上是个奇异点。**已确认保持全零位不变**——它只作扫描/打磨的起终点，
> 不参与笛卡尔规划，实际跑不受影响。
> 仅当以后需要**绕零位附近做笛卡尔运动**时才需要理会
> （届时可改用 `[0, 0, 0.05, 0, 0.10, 0]`，秩 6 且末端位置几乎不变）。

### 3. 向前重算扫描轨迹

**问题**：`config/scan_traj.yaml` 里原第 2–9 个航点还是 HC10DT 的关节值。
代入 CR12A 正运动学核算后，`tool0` 全落在 X 负半区（−0.38 ~ −0.52 m），
也就是**机器人基座后方**；而工件实际在 X ≈ 0.66 ~ 0.94 m 的**正前方**。
照原样跑，CR12A 会朝着空气扫。

**关键发现**：从 `config/calibration.yaml` 反解出相机在法兰坐标系下的姿态后，
相机光轴（+Z）在法兰系里指向 **+X**。所以「相机垂直朝下」要求法兰的 +X 轴竖直向下。

**重算方法**：以「相机光轴竖直向下、镜头对准坐面上方固定高度」为约束，
用正运动学求目标位姿、再用阻尼最小二乘（DLS）数值逆解求关节角，
逐个航点迭代到残差 < 1e-4。

**结果**：相机在坐面上方 **Z = 0.42 m**（离坐面顶面 10.3 cm）做蛇形扫描，
X 方向覆盖 0.68 → 0.90 m，Y 方向 ±0.06 ~ 0.08 m。共 10 个点：

```
P1  [0,0,0,0,0,0]  ← 零位（起点）
P2..P9  8 个扫描航点，X 由 0.68 递增到 0.90 再折返
P10 [0,0,0,0,0,0]  ← 零位（终点）
```

关于可达性边界（供以后调工件位置参考）：这台 CR12A 在「相机朝下」姿态下，
末端 X 越远能压得越低 —— X ≈ 0.86 m 时 Z 最低 0.52 m；0.90 m 时约 0.45 m；0.92 m 时约 0.42 m。**再远就够不着了。**

### 4. 节拍调快

| 项 | 原来 | 现在 |
|---|---|---|
| 扫描轨迹总时长 | ~60 s | **13.2 s** |
| 打磨 TCP 平移速度 | 0.05 m/s | **0.15 m/s** |
| 打磨 TCP 平移加速度 | 0.10 m/s² | **0.50 m/s²** |
| 打磨 TCP 旋转速度 | 1.571 rad/s | 3.14 rad/s |
| 打磨 TCP 旋转加速度 | 3.14 rad/s² | 6.28 rad/s² |

扫描时长是按关节速度上限的 25%（下限 1.2 s）自动分配的，比原来快约 4.5 倍。

打磨速度做成了 launch 参数，不用改文件就能调：

```bash
ros2 launch snp_automate_2023 start.launch.xml max_translational_vel:=0.25
```

> ⚠️ 别一次提太高。速度上限直接进 Descartes 的 LadderGraphSolver，
> 提过头会报 `LadderGraphSolver failed to build graph`（找不到可行解）。
> 想调回保守值就传 `0.05`。

### 5. Docker 挂载 urdf 目录

`docker/compose.sim.yml` 原来只挂了 `config/`、`launch/`、`meshes/`，
`urdf/` 走的是镜像内的旧副本。补上一行，本地改 URDF 才生效：

```yaml
- ../urdf:/opt/snp_automate_2023/install/snp_automate_2023/share/snp_automate_2023/urdf:ro
```

---

## 关键文件对照

| 文件 | 作用 |
|---|---|
| `urdf/cr12_macro.xacro` | CR12A 模型宏（新增） |
| `urdf/workcell.xacro` | 工作台 + 机器人实例装配 |
| `urdf/ros2_control.xacro` | 仿真控制接口与关节限位 |
| `config/workcell.srdf` | MoveIt 规划组定义 |
| `config/scan_traj.yaml` | 扫描轨迹（关节空间，10 点） |
| `config/tpp.yaml` | 打磨工具路径参数（线/点间距、IK 超时） |
| `launch/start.launch.xml` | 主入口：home 位、速度参数、各节点 |
| `launch/test.launch.xml` | 机器人描述 / 关节状态 / RViz |
| `docker/compose.sim.yml` | 容器挂载 |

---

## 已知隐患 / 待办

### 下一步：把坐面换回原本的半圆工件

当前工件是**坐面坐板**（`meshes/part_scan.ply`，约占 X 0.658~0.938 / Y −0.095~0.115 / Z 0.302~0.317），
打磨的是它的顶面。**后续要把这个坐面换回原本的那个半圆工件。**

换回去时注意：

1. 扫描轨迹（`config/scan_traj.yaml` 的 P2–P9）是**按当前坐面的位置和尺寸算的**，
   换工件后必须重算，否则相机扫不到目标。
2. 打磨路径由 `config/tpp.yaml` + 圈选 ROI 现场生成，工件换了要重新圈选。
3. 如果新工件比坐面高或更远，先对照上面的可达性边界确认够不够得着。

### 其它遗留项

| 项 | 说明 |
|---|---|
| **相机标定还是 HC10 时代的** | `config/calibration.yaml` 里的 `camera_mount_to_camera` 未重新标定，只是沿用 |
| **打磨头支架未换** | `urdf/workcell.xacro` 的末端执行器仍引用 `hc10_standoff.ply`，位置尺寸按 HC10 来的 |
| **TCP `sand_tcp` 未标定** | 法兰到打磨头的变换仍是旧值 |
| `motoros2/r1/flange` 帧名 | 仿真里靠 static_transform_publisher 桥接到 `flange`，名字是安川时代的遗留，功能正常 |
| `generate_motion_plan` 内存增长 | 长时间跑会持续涨内存，详见 `docs/TROUBLESHOOTING_CN.md` |

---

## 相关文档

| 文档 | 内容 |
|---|---|
| [`docs/CR12A_Migration_Analysis.md`](docs/CR12A_Migration_Analysis.md) | CR12A 替换可行性分析、DH 参数、工作空间核算 |
| [`docs/zhixingceng_seat_polishing.md`](docs/zhixingceng_seat_polishing.md) | 坐面工件替换 + 只打磨顶面（上一分支） |
| [`docs/RUN_GUIDE_CN.md`](docs/RUN_GUIDE_CN.md) | 运行指南 |
| [`docs/TROUBLESHOOTING_CN.md`](docs/TROUBLESHOOTING_CN.md) | 常见问题排查 |
| [`docs/PROJECT_WORKFLOW_CN.md`](docs/PROJECT_WORKFLOW_CN.md) | 流程说明 |

## 相关链接

- 上游项目：https://github.com/ros-industrial-consortium/snp_automate_2023
- Dobot 官方 ROS2 仓库：https://github.com/Dobot-Arm/DOBOT_6Axis_ROS2_V4
