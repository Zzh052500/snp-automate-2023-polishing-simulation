# 越疆 CR12A 机械臂替换分析（整合版）

> **文档说明**
> 本文档整合了原有三份独立分析（总体分析 / DH参数分析 / 工作空间分析），
> 并新增「官方资源评估」章节。
> 所有硬件数据均已通过官方渠道核实，替代了早期版本中的推测值。

**分析对象：** 将当前 `Motoman HC10DT` 替换为 `越疆 CR12A`
**分析日期：** 2026-09-17
**预计工作量：** 方案A（验证）约 10h / 方案B（生产）约 28h
**总体风险等级：** 🟡 **中等**（原判"高"，因官方资源可获取而下调）

---

## 目录

1. [摘要与结论](#一摘要与结论)
2. [数据来源与可信度](#二数据来源与可信度)
3. [硬件规格对比](#三硬件规格对比)
4. [官方资源评估](#四官方资源评估)
5. [风险分析](#五风险分析)
6. [实施方案](#六实施方案)
7. [工作量与风险矩阵](#七工作量与风险矩阵)
8. [验证方法](#八验证方法)
9. [附录](#九附录)

---

## 一、摘要与结论

### 核心结论

```
✅ 可行，且比初版评估乐观

三个关键事实：

1. 工作空间大小完全一致
   HC10DT 有效半径 1200mm  =  CR12A 工作半径 1200mm
   工件位于 0.73~1.00m，两者余量相同（17%~39%）
   → 位置层面不构成风险

2. 官方 ROS2 资源完整可获取
   Dobot 官方仓库提供 URDF / SRDF / 关节限位 / 质量惯量 / MoveIt 配置
   → 省去约 8-11 小时的自建工作

3. 真正的技术风险是「±360° 关节导致 IK 多解」
   官方 URDF 实证：joint1/2/4/5/6 限位均为 ±6.27 rad（±359.24°）
   → 相邻路径点可能出现 360° 跳变，触发 Descartes 求解失败
```

### 初版文档的更正

| 项目 | 初版（推测） | 已核实 |
|------|-------------|--------|
| HC10DT 有效半径 | ~1.4m | **1200mm** |
| CR12A 工作半径 | ~1.2m | **1200mm** |
| 半径差异 | 减少 200mm ⚠️ | **完全一致** ✅ |
| CR12A 关节范围 | ±180° | **±360°**（J3: ±160°） |
| 风险方向（关节） | "超限无法到达" | **"解过多导致缠绕"** |
| 重复定位精度 | ±0.08 / ±0.05 | **±0.1 / ±0.03** |
| 工作空间失败概率 | 40% 🔴 | **~5%** 🟢 |

---

## 二、数据来源与可信度

| 数据项 | 来源 | 可信度 |
|--------|------|--------|
| HC10DT 规格 | Yaskawa 官方产品页 | ✅ 官方 |
| CR12A 规格 | Dobot 官方 + 授权经销商 | ✅ 官方 |
| CR12A 关节链 / 限位 / 惯量 | **Dobot 官方 ROS2 仓库** | ✅ 官方代码 |
| 工件几何 | 本项目 `results_mesh.ply` 实测 | ✅ 实测 |
| 工作空间参数 | 本项目 `start.launch.xml` | ✅ 实测 |
| 安装件依赖 | 本项目 `urdf/workcell.xacro` | ✅ 实测 |

> ⚠️ 本文档中标注「待实测」的项目，指需要现场测量或索取图纸确认。

---

## 三、硬件规格对比

### HC10DT（当前机器人）

| 参数 | 数值 |
|------|------|
| 最大工作范围 | 1,379 mm（S/L 旋转中心 → R/T 旋转中心） |
| **有效到达距离** | **1,200 mm** |
| 有效负载 | 10 kg |
| 重复定位精度 | ±0.1 mm |
| 本体重量 | 48 kg |
| 电源容量 | 1 kVA |
| 安装方式 | 地面 / 墙面 / 天花板 |

> 当前仿真实际型号 `motoman_hc10dt`，见 `urdf/workcell.xacro:4`

### CR12A（目标机器人）

| 参数 | 数值 |
|------|------|
| **工作半径** | **1,200 mm** |
| 有效负载 | 12 kg |
| 重复定位精度 | ±0.03 mm |
| 本体重量 | 39.5 kg |
| 关节活动范围 | J1/J2/J4/J5/J6 = **±360°**，J3 = **±160°** |
| 关节最大速度 | J1/J2: 150°/s，J3: 180°/s，J4/J5/J6: 223°/s |
| IP 防护 | IP54 |
| 典型功耗 | 350 W |

### 对比总览

| 指标 | HC10DT | CR12A | 判定 |
|------|--------|-------|------|
| 有效工作半径 | 1200mm | 1200mm | ✅ 一致 |
| 关节数 | 6 | 6 | ✅ 一致 |
| 关节范围 | 约 ±180° | **±360°** | 🔴 需处理 |
| 有效负载 | 10kg | 12kg | ✅ 有余量 |
| 重复定位精度 | ±0.1mm | ±0.03mm | ✅ 更优 |
| 本体重量 | 48kg | 39.5kg | ✅ 更轻 |

---

## 四、官方资源评估

### 4.1 仓库概况

**仓库：** [`Dobot-Arm/DOBOT_6Axis_ROS2_V4`](https://github.com/Dobot-Arm/DOBOT_6Axis_ROS2_V4)
**规模：** 1119 个文件
**协议：** 见仓库 LICENSE

```
DOBOT_6Axis_ROS2_V4/
├── cr12_moveit/               ⭐ 完整 MoveIt 配置包（最核心）
├── cra_description/           ⭐ URDF xacro + STL 网格
├── dobot_rviz/                ⭐ 纯 URDF + STL 网格
├── dobot_kinematics_plugin/   ⭐ 官方运动学插件
├── dobot_bringup_v4/          ← 驱动层
├── dobot_gazebo/              ← Gazebo 仿真
├── dobot_msgs_v4/             ← 消息定义
├── dobot_demo/ servo_action/  ← 示例
└── cr3/cr5/cr7/cr10/cr16/cr20/cr30h/me6/nova*/  ← 其他型号（可参考）
```

### 4.2 `cr12_moveit/config/` 内容清单

| 文件 | 作用 | 价值 |
|------|------|------|
| `cr12_robot.urdf.xacro` | 机器人 URDF（xacro 包装） | ⭐⭐⭐ |
| `cr12_robot.srdf` | 语义定义（规划组/碰撞豁免） | ⭐⭐⭐ |
| `cr12_robot.ros2_control.xacro` | 控制器配置 | ⭐⭐ |
| `kinematics.yaml` | **IK 求解器配置** | ⭐⭐⭐ |
| `joint_limits.yaml` | **关节限位/速度/加速度** | ⭐⭐⭐ |
| `ompl_planning.yaml` | OMPL 规划参数 | ⭐⭐ |
| `initial_positions.yaml` | 初始位姿 | ⭐ |
| `moveit_controllers.yaml` / `ros2_controllers.yaml` | 控制器 | ⭐⭐ |
| `pilz_cartesian_limits.yaml` | 笛卡尔限位 | ⭐ |
| `moveit.rviz` | RViz 配置 | ⭐ |
| 8 个 launch 文件 | 启动脚本 | ⭐⭐ |

### 4.3 网格资源

```
cra_description/meshes/cr12/    以及    dobot_rviz/meshes/cr12/
├── base_link0.STL
├── J1.STL ~ J6.STL
```

共 7 个 STL，与 URDF 中 7 个连杆一一对应。

### 4.4 关节链实测数据（等效 DH 参数）

从官方 URDF 提取：

| 关节 | parent → child | origin xyz (m) | axis | 说明 |
|------|---------------|----------------|------|------|
| joint1 | base_link → Link1 | (0, 0, **0.1765**) | (0,0,1) | 底座高度 176.5mm |
| joint2 | Link1 → Link2 | (0, 0, 0) | (0,0,1) | — |
| joint3 | Link2 → Link3 | (**-0.557**, 0, 0.066) | (0,0,1) | **大臂 557mm** |
| joint4 | Link3 → Link4 | (**-0.518**, 0, 0.125) | (0,0,1) | **小臂 518mm** |
| joint5 | Link4 → Link5 | (0, -0.125, 0) | (0,0,1) | — |
| joint6 | Link5 → Link6 | (0, 0.1114, 0) | (0,0,1) | — |

**几何校验：** 大臂 557 + 小臂 518 = **1075mm**，加腕部约 **1200mm**
→ ✅ 与官方工作半径 1200mm 吻合

### 4.5 关节限位（官方实证）

| 关节 | lower (rad) | upper (rad) | 角度 | effort | velocity |
|------|------------|------------|------|--------|----------|
| joint1 | -6.2700 | 6.2700 | **±359.24°** | 100 | 3.14 |
| joint2 | -6.2700 | 6.2700 | **±359.24°** | 100 | 3.14 |
| joint3 | -2.7900 | 2.7900 | **±159.86°** | 100 | 3.89 |
| joint4 | -6.2700 | 6.2700 | **±359.24°** | 100 | 3.89 |
| joint5 | -6.2700 | 6.2700 | **±359.24°** | 100 | 3.89 |
| joint6 | -6.2700 | 6.2700 | **±359.24°** | 100 | 3.89 |

> 🔴 **关键实证**：CR12A 的 joint1/2/4/5/6 限位确为 ±360° 量级。
> 这不是参数配置失误，而是**官方设计意图**。
> 这就是第五章「IK 多解风险」的根源。

`joint_limits.yaml` 补充的速度/加速度限制：

```yaml
default_velocity_scaling_factor: 0.3
default_acceleration_scaling_factor: 0.3
joint1/joint2:  max_velocity 3.14 rad/s,  max_acceleration 6.28
joint3~joint6:  max_velocity 3.89 rad/s,  max_acceleration 7.78
```

### 4.6 质量与惯量（STEP 中完全缺失）

| 连杆 | 质量 (kg) |
|------|----------|
| base_link | 1.8265 |
| Link1 | 4.1649 |
| Link2 | 10.7566 |
| Link3 | 4.4449 |
| Link4 | 1.1688 |
| Link5 | 1.2230 |
| Link6 | 0.2071 |
| **总计** | **23.79** |

完整惯性张量（ixx/ixy/ixz/iyy/iyz/izz）也包含在 URDF 中。

### 4.7 IK 求解器配置（官方）

```yaml
cr12_group:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.005        # 仅 5ms
```

### 4.8 SRDF 规划组定义（官方）

```xml
<group name="cr12_group">
    <chain base_link="dummy_link" tip_link="Link6"/>
</group>

<group_state name="home" group="cr12_group">
    <joint name="joint1" value="0"/>
    <joint name="joint2" value="0.5204"/>
    <joint name="joint3" value="-1.4335"/>
    <joint name="joint4" value="-0.5204"/>
    <joint name="joint5" value="1.7695"/>
    <joint name="joint6" value="0"/>
</group_state>
```

### 4.9 ⚠️ 官方资源的四个坑

| # | 坑 | 说明 | 应对 |
|---|----|------|------|
| 1 | **dummy_link / dummy_joint** | SolidWorks 导出器产物，SRDF 中作为 chain 起点 | 清理或保留但明确其角色 |
| 2 | **IK 插件体系不同** | 官方用 MoveIt `KDLKinematicsPlugin`；本项目用 Tesseract `KDLInvKinChainLMA` | ❌ **不能照搬**，需重写配置 |
| 3 | **缺 flange/tool0** | URDF 到 `Link6` 结束，无工具法兰坐标系 | 需自行添加 |
| 4 | **timeout 差 2000 倍** | 官方 5ms vs 本项目 10000ms | 说明求解路径不同，需重新调参 |

### 4.10 与前次识别的资料缺口对照

| 前次判定为「缺失」 | 官方仓库是否提供 |
|-------------------|-----------------|
| 关节轴向 / 原点 | ✅ **提供**（URDF 完整） |
| 关节限位 | ✅ **提供**（URDF + joint_limits.yaml） |
| 质量 / 惯量 | ✅ **提供**（URDF 完整） |
| 网格几何 | ✅ **提供**（STL，7个） |
| 语义定义 SRDF | ✅ **提供** |
| IK 求解器配置 | ⚠️ 提供但**体系不同**（MoveIt vs Tesseract） |
| 工具法兰 / TCP | ❌ **仍缺**，需自行定义 |
| 控制器通信接口 | ✅ 提供（dobot_bringup_v4） |

---

## 五、风险分析

### 5.1 风险一：±360° 关节导致 IK 多解 🔴

**这是最重要的技术风险。**

```
HC10DT 关节范围（约 ±180°）:
  目标姿态对应 θ2 = 10°
  → 搜索空间 [-180°, 180°]
  → 唯一合理解: 10°
  → 解确定 ✅

CR12A 关节范围（±360°，官方实证）:
  目标姿态对应 θ2 = 10°
  → 搜索空间 [-360°, 360°]
  → 合理解: 10° / 370° / -350° / 730° ...
  → 解不唯一 ⚠️
```

#### 故障模式 1：关节绕整圈

```
打磨路径相邻两点（相距仅 30mm）:

  Point_1 求解 → θ2 = 355°
  Point_2 求解 → θ2 = 5°      (数学等价，物理差 350°)

规划器看到:
  θ2: 355° → 5°  =  -350° 的大幅旋转 ⚠️

实际现象:
  ❌ 机械臂在打磨过程中突然"绕一整圈"
  ❌ 路径执行时间暴增
  ❌ 可能与工件/夹具/桌面碰撞
```

#### 故障模式 2：Descartes 求解器建图失败

```
Descartes 要求相邻路径点的关节解"连续"
  ↓
±360° 下 KDL 可能返回不连续解（相差 360°）
  ↓
LadderGraphSolver failed to build graph
```

> 📌 **这是本项目已经踩过的坑**
> `README.md` 记载：当前 HC10 配置曾报 `LadderGraphSolver failed to build graph`，
> 解法是"对工具路径做法向过滤，把法向量写死为朝上 (0,0,1)"。
>
> 换成 CR12A 后，**±360° 会让这个问题更容易复现**，
> 原有的"法向量写死"解法可能不足。

#### 故障模式 3：奇异点位置改变

```
HC10DT 可正常求解的关节配置
  ↓
CR12A 可能落入奇异点（雅可比矩阵行列式 → 0）
  ↓
IK 无法反演，返回无解
```

#### 三种模式对比

| 模式 | 触发条件 | 症状 | 严重度 |
|------|---------|------|--------|
| 关节绕整圈 | ±360° 多解 | 路径时间暴增、碰撞 | 🔴 高 |
| Descartes 建图失败 | 解不连续 | `LadderGraphSolver failed` | 🔴 高 |
| 奇异点 | 几何构型 | IK 返回无解 | 🟡 中 |

#### 应对方案

**方案 A：收窄 URDF 关节范围（推荐）**

```xml
<joint name="joint1" type="revolute">
  <limit lower="-3.14159265" upper="3.14159265"
         effort="100" velocity="3.14"/>
  <!-- 从 ±6.27 收窄到 ±π，消除多圈歧义 -->
</joint>

<!-- joint3 保持官方值 -->
<joint name="joint3" type="revolute">
  <limit lower="-2.7900" upper="2.7900" .../>
</joint>
```

**方案 B：Tesseract 关节连续性约束**（保留全部自由度，配置复杂）

**方案 C：后处理关节角 unwrap**

```python
def unwrap_joint_trajectory(trajectory):
    """保证相邻路径点的关节角差不超过 π"""
    for i in range(1, len(trajectory)):
        for j in range(6):
            delta = trajectory[i][j] - trajectory[i-1][j]
            if delta > np.pi:
                trajectory[i][j] -= 2*np.pi
            elif delta < -np.pi:
                trajectory[i][j] += 2*np.pi
    return trajectory
```

| 方案 | 工作量 | 保留自由度 | 推荐度 |
|------|--------|-----------|--------|
| A. 收窄 URDF | 1-2h | 50% | ⭐⭐⭐ **推荐** |
| B. 连续性约束 | 4-6h | 100% | ⭐⭐ |
| C. 后处理 unwrap | 3-5h | 100% | ⭐⭐ |

---

### 5.2 风险二：安装接口 `hc10_standoff` 必须重做 🔴

在 `urdf/workcell.xacro` 中发现**硬件专用件**：

```xml
<!-- 第 108、144 行 -->
<mesh filename="package://snp_automate_2023/meshes/collision/hc10_standoff.ply"/>
```

工具链结构：

```
ATI 力控主轴 (ati_spindle / ati_tool / ati_hose)
    ↓
ati_bracket + ati_standoff
    ↓
hc10_standoff          ← 🔴 HC10 专用！换 CR12A 必须重做
    ↓
sensor_bracket
    ↓
tool0
```

#### 需要确认的接口参数

| 参数 | HC10DT | CR12A | 动作 |
|------|--------|-------|------|
| 法兰直径 | 待实测 | 待实测 | 索取图纸 |
| 法兰螺栓孔 | 待实测 | 待实测 | 索取图纸 |
| 定位销 | 待实测 | 待实测 | 实际测量 |
| 法兰到 tool0 偏移 | 由 hc10_standoff 决定 | **未知** | **重新设计** |

#### 影响

```
真实场景：❌ 工具无法物理安装
仿真场景：❌ flange → tool0 变换错误
         → TCP 位置偏移 → 打磨接触点错位
```

**预计工作量：3-5 小时**（含建模 + 仿真验证）

---

### 5.3 风险三：工作空间大小 🟢（已排除）

#### 有效半径对比

```
HC10DT  ├──────────────────┤ 1200mm
CR12A   ├──────────────────┤ 1200mm
                      完全一致
```

#### 工件实测位置

从 `runtime/snp_home/snp/meshes/results_mesh.ply`（1276 顶点）：

```
凳面坐板包围盒:
  X: 0.6577 ~ 0.9377 m   (尺寸 0.280 m)
  Y: -0.0950 ~ 0.1150 m  (尺寸 0.210 m)
  Z: 0.3020 ~ 0.3170 m   (尺寸 0.015 m)

  ↑ 0.28 × 0.21 m 与坐面实际尺寸吻合
```

#### 可达性计算

```
工件距底座原点（base_link）距离:
  最近角点 (0.6577, 0, 0.302)      → 0.730 m
  最远角点 (0.9377, 0.115, 0.317)  → 0.997 m
  范围: 0.73 ~ 1.00 m
```

| 机器人 | 有效半径 | 最近点余量 | 最远点余量 | 判定 |
|--------|---------|-----------|-----------|------|
| HC10DT | 1200 mm | 39.2% | 17.0% | ✅ 可达 |
| CR12A | 1200 mm | 39.2% | 17.0% | ✅ 可达 |

**结论：工件完全落在两种机器人的可达范围内，余量相同，位置层面无风险。**

#### 关于 `ir_max_x = 1.5m` 的说明

```xml
<param name="ir_max_x" value="1.5"/>
```

⚠️ 这**不是机械臂可达边界**，而是**工业重建（IR）的体素裁剪边界**：

- 1.5m 是扫描点云的保留范围，可以大于机械臂工作半径
- 机械臂可达半径（1.2m）限制的是**执行阶段**
- 两者含义不同，不冲突

---

### 5.4 风险四：Z 轴工作范围偏移 🟡

```
当前配置:
  ir_min_z: 0.0889 m
  ir_max_z: 0.527 m
```

这两个值与**底座安装高度**直接相关。官方 URDF 显示 CR12A 的
`base_link → joint1` 偏移为 **176.5mm**，若与 HC10DT 不同：

```
新的可达 Z 范围 = [0.0889 + Δh, 0.527 + Δh]
工件 Z = 0.302~0.317 m（固定不变）
```

#### 需实测

```bash
ros2 run tf2_ros tf2_echo base_link tool0
grep -A5 "base_link" urdf/workcell.xacro | grep -i origin
```

**预计工作量：1-2 小时**

---

## 六、实施方案

### 方案 A：快速验证（推荐先做）

**目标：** 验证 CR12A 是否可行，判断是否值得全面投入

```
步骤 1  获取官方资源                                       0.5h
        └─ 拉取 DOBOT_6Axis_ROS2_V4 的 cr12 相关目录

步骤 2  清理 URDF + 添加 flange                             2h
        ├─ 清理/保留 dummy_link
        ├─ 添加 tool0 / sand_tcp 坐标系
        └─ 验证 URDF 可解析

步骤 3  收窄关节范围 ±6.27 → ±π                             1h
        └─ 消除 IK 多解（方案A）

步骤 4  Tesseract IK 配置改写                               2-4h
        └─ 从 MoveIt KDL 插件转为 KDLInvKinChainLMA

步骤 5  工作空间参数标定                                    1h
        └─ 实测底座高度，调整 ir_min_z / ir_max_z

步骤 6  集成测试                                            3h
        ├─ 打磨路径规划
        ├─ 运动规划
        └─ 干涉检查
```

**总时间：约 10 小时**

### 方案 B：完整集成（生产级）

```
第1阶段（2天）：准备
  ├─ 获取官方全套资源
  ├─ 物理测量 CR12A 法兰接口
  ├─ 重新设计 hc10_standoff 替代件
  └─ 建立标定流程

第2阶段（2天）：集成
  ├─ URDF 完整替换 + 清理
  ├─ SRDF 重写（适配 SNP 规划组）
  ├─ IK 求解器配置（Tesseract）
  ├─ 碰撞模型建立
  └─ ros2_control 配置

第3阶段（2天）：验证
  ├─ ±360° 多解问题处理与验证
  ├─ 工作空间可视化对比
  ├─ 打磨路径全点可达性检查
  └─ 干涉检测

第4阶段（1天）：优化与文档
  ├─ 参数微调
  ├─ 文档更新
  └─ 回滚方案确认
```

**总时间：约 28 小时（约 3.5 工作日）**

---

## 七、工作量与风险矩阵

### 7.1 工作量估计（含官方资源后的修正）

| 工作项 | 初版估计 | **修正后** | 说明 |
|--------|---------|-----------|------|
| 获取资料 | 2-3h | **0.5h** | ⬇️ 官方仓库直接获取 |
| 关节轴/限位/惯量自建 | 6-9h | **0h** | ⬇️ 官方 URDF 已提供 |
| URDF 替换与清理 | 1-2h | **2-3h** | ⬆️ 需清理 dummy_link + 加 flange |
| IK 求解器配置 | 4-6h | **4-6h** | — 插件体系不同，仍需重写 |
| 工作空间标定 | 3-5h | **1-2h** | ⬇️ 半径一致，仅需标定 Z |
| ±360° 多解处理 | 未识别 | **1-2h** | ⬆️ 新识别（方案A） |
| TCP / 安装件 | 5-8h | **5-8h** | — hc10_standoff 仍需重做 |
| 集成测试 | 3-8h | **3-8h** | — |
| 文档更新 | 1-3h | **1-3h** | — |
| **合计** | **25-44h** | **17-33h** | **⬇️ 约省 8-11h** |

### 7.2 风险矩阵（修正后）

| 风险点 | 概率 | 影响 | 修复难度 | 等级 |
|--------|------|------|---------|------|
| **安装接口不匹配** | **80%** | 🔴 严重 | 🟡 中等 | 🔴 **高** |
| **±360° 关节 IK 多解** | **60%** | 🔴 严重 | 🔴 困难 | 🔴 **高** |
| IK/DH 参数重配 | 30% | 🔴 严重 | 🔴 困难 | 🔴 **高** |
| TCP 位置偏移 | 50% | 🟡 中等 | 🟡 中等 | 🟡 **中等** |
| Z 轴范围偏移 | 20% | 🟡 中等 | 🟢 容易 | 🟡 **中等** |
| ~~工作空间大小不足~~ | ~~40%~~ → **5%** | 🟢 轻微 | 🟢 容易 | 🟢 **低** |

### 7.3 资源需求

**人力：** 1× 高级 ROS/MoveIt/Tesseract 工程师
**软件：** ROS 2 + Tesseract + MoveIt 2 + 官方 Dobot 仓库
**硬件（方案B）：** CR12A 实体机器人、量具

---

## 八、验证方法

### 8.1 IK 往返一致性测试

```python
#!/usr/bin/env python3
"""验证 CR12A 的 IK 求解是否稳定（重点检测 360° 跳变）"""
import numpy as np

def test_ik_roundtrip(ik_solver, fk_func, num_samples=200, tol=1e-3):
    failures, jumps = [], []

    for i in range(num_samples):
        q_true = np.random.uniform(-np.pi, np.pi, 6)
        pose = fk_func(q_true)
        q_solved = ik_solver.solve(pose)

        if q_solved is None:
            failures.append((i, 'no solution'))
            continue

        diff = q_solved - q_true
        if np.any(np.abs(diff) > np.pi):
            jumps.append((i, diff))

        if not np.allclose(fk_func(q_solved), pose, atol=tol):
            failures.append((i, 'pose mismatch'))

    print(f"失败数: {len(failures)}/{num_samples}")
    print(f"360°跳变数: {len(jumps)}/{num_samples}")

    if jumps:
        print("⚠️  检测到关节跳变 → 需收窄关节范围或启用连续性约束")
    return failures, jumps
```

**预期结果：**
- 未收窄关节范围：跳变数 > 0（验证问题存在）
- 收窄后：跳变数应为 **0**

### 8.2 打磨路径连续性检查

```python
def check_path_continuity(trajectory, max_delta=np.pi/2):
    """检查打磨路径相邻点的关节角变化是否合理"""
    problems = []
    for i in range(1, len(trajectory)):
        for j in range(6):
            delta = abs(trajectory[i][j] - trajectory[i-1][j])
            if delta > max_delta:
                problems.append({'segment': i, 'joint': j,
                                 'delta_deg': np.degrees(delta)})

    if problems:
        print(f"⚠️  发现 {len(problems)} 处异常跳变:")
        for p in problems[:10]:
            print(f"  路径段 {p['segment']}, J{p['joint']+1}: {p['delta_deg']:.1f}°")
    else:
        print("✅ 路径连续性正常")
    return problems
```

### 8.3 几何校验（已知数据）

```
理论工作半径 = 大臂 557mm + 小臂 518mm + 腕部 ≈ 1200mm
官方标称工作半径 = 1200mm
→ ✅ 吻合，可用于验证 URDF 加载正确性
```

### 8.4 命令速查

```bash
# 查看 TF 树
ros2 run tf2_ros tf2_echo base_link tool0

# 检查 URDF 解析
check_urdf cr12_robot.urdf

# 验证 MoveIt 加载
ros2 launch cr12_moveit demo.launch.py
```

---

## 九、附录

### 9.1 需要修改的文件（本项目）

```
✏️ urdf/workcell.xacro          机器人宏替换
✏️ config/workcell.srdf         规划组重写
✏️ config/workcell_plugins.yaml IK 求解器配置
✏️ launch/start.launch.xml      工作空间参数
✏️ README.md                    文档更新
```

### 9.2 需要创建的文件

```
✨ config/cr12a_macro.xacro      若无现成包
✨ meshes/collision/cr12a_standoff.ply   替代 hc10_standoff
✨ docs/TCP_Calibration_Procedure.md
```

### 9.3 官方资源地址

| 资源 | 地址 |
|------|------|
| Dobot ROS2 仓库 | https://github.com/Dobot-Arm/DOBOT_6Axis_ROS2_V4 |
| CR12 URDF | `dobot_rviz/urdf/cr12_robot.urdf` |
| CR12 MoveIt 配置 | `cr12_moveit/config/` |
| CR12 网格 | `cra_description/meshes/cr12/` |
| 本机 STEP 文件 | `~/Documents/xwechat_files/.../CR12A_asm.stp` |

### 9.4 本地 STEP 文件评估

**文件：** `CR12A_asm.stp`（7.6MB / 157,099 行）

| 项目 | 状态 |
|------|------|
| 格式 | STEP AP214 (AUTOMOTIVE_DESIGN) |
| 来源 | Creo Parametric，2026-06-10 |
| 单位 | MILLI METRE (mm) |
| 部件 | `BASE-CR12A` + `LINK1~LINK6-CR12A`（7个）✅ |
| 几何 | 2085 面 / 39 闭合壳 ✅ |
| 圆柱特征 | 420 个（可用于反推关节轴） |
| **运动学定义** | ❌ 无（MECHANISM=0） |
| **关节限位** | ❌ 无 |
| **质量/惯量** | ❌ 无（MASS/DENSITY=0） |

**包围盒实测：**
```
X: -93.40 ~ 193.78 mm    (跨度  287.18 mm)
Y: -304.37 ~ 105.30 mm   (跨度  409.67 mm)
Z: -93.00 ~ 1349.83 mm   (跨度 1442.83 mm)
```

> **结论：** STEP 文件的价值是**几何建模参考**；
> 运动学参数（关节轴/限位/惯量）**由官方 URDF 提供，无需从 STEP 反推**。

### 9.5 参考来源

- [Yaskawa HC10DT 官方产品页](https://www.yaskawa.co.uk/products/robots/collaborative/productdetail/product/hc10dt_681)
- [Dobot CR12A - RobotShop](https://eu.robotshop.com/products/dobot-cr12a-6-axis-collaborative-robot-arm-12kg-1200mm)
- [Dobot CR12A - RBTX](https://rbtx.de/en-GB/components/robots/dobot-cr12a-6dof-1200mm-12kg)
- [越疆 CR12 官方页面](https://www.dobot.cn/products/cr-series/cr12.html)
- [Dobot ROS2 仓库](https://github.com/Dobot-Arm/DOBOT_6Axis_ROS2_V4)

---

## 版本历史

| 版本 | 日期 | 作者 | 变更 |
|------|------|------|------|
| v1.0 | 2026-09-17 | 浮浮酱 | 初始版本（含推测数据） |
| v1.1 | 2026-09-17 | 浮浮酱 | 修正工作空间与关节范围数据 |
| v2.0 | 2026-09-17 | 浮浮酱 | **整合版**：合并 DH 参数分析、工作空间分析，新增官方资源评估 |

---

**最后更新：** 2026-09-17
**维护者：** 浮浮酱
