# DH参数改变详细分析

> ## ⚠️ 数据更正说明（2026-09-17）
>
> 本文档初版部分数据基于推测，经核实官方规格后更正：
>
> | 项目 | 初版（❌） | 已核实（✅） |
> |------|-----------|-------------|
> | CR12A 关节范围 | ±180° | **±360°**（J3 为 ±160°） |
> | 风险方向 | "关节超限无法到达" | **"解过多导致缠绕"** |
> | 臂长差异 | HC10 1.4m / CR12A 1.2m | 两者有效半径**均 1200mm** |
>
> **核心修正：** CR12A 的关节范围**大于** HC10DT，不会出现超限问题，
> 反而因为解不唯一导致 360° 跳变。详见第三章。

---

## 一、什么是DH参数？

DH参数（Denavit-Hartenberg参数）是描述机械臂几何结构的4个参数组合，用来定义相邻两个关节之间的变换关系。

```
对于每个关节 i，有4个DH参数：
├─ a_i     : 连杆长度 (两个关节轴之间的距离)
├─ α_i     : 连杆扭角 (两个关节轴之间的夹角)
├─ d_i     : 关节距离 (沿关节轴方向的偏移)
└─ θ_i     : 关节角度 (可变，运动时改变)
```

### 为什么这4个参数是"运动学的地基"

```
每个关节的变换矩阵 T_i 由DH参数唯一确定：

     ┌ cosθᵢ  -sinθᵢcosαᵢ   sinθᵢsinαᵢ   aᵢcosθᵢ ┐
Tᵢ = │ sinθᵢ   cosθᵢcosαᵢ  -cosθᵢsinαᵢ   aᵢsinθᵢ │
     │ 0       sinαᵢ        cosαᵢ        dᵢ      │
     └ 0       0             0             1      ┘

整个机械臂：T_total = T₁ × T₂ × T₃ × T₄ × T₅ × T₆

aᵢ 或 dᵢ 改变 → Tᵢ 改变 → T_total 改变 → 末端位置改变
```

---

## 二、HC10DT vs CR12A 的几何参数对比

### HC10DT（当前机器人）

| 参数 | 数值 |
|------|------|
| 最大工作范围 | 1,379 mm（S/L 旋转中心 → R/T 旋转中心） |
| **有效到达距离** | **1,200 mm**（S/L 旋转中心 → Point P） |
| 关节数 | 6 |
| 重复定位精度 | ±0.1 mm |
| 本体重量 | 48 kg |

### CR12A（目标机器人）

| 参数 | 数值 |
|------|------|
| **工作半径** | **1,200 mm** |
| 关节数 | 6 |
| 关节范围 | J1/J2/J4/J5/J6 = **±360°**，J3 = **±160°** |
| 重复定位精度 | ±0.03 mm |
| 本体重量 | 39.5 kg |

### 关键差异总结

```
✅ 有效工作半径:  1200mm  vs  1200mm   → 完全一致
✅ 关节数:        6       vs  6        → 一致
🔴 关节范围:      约±180° vs  ±360°     → CR12A 明显更宽
✅ 重复精度:      ±0.1mm  vs  ±0.03mm  → CR12A 更好
```

> ⚠️ **注意**：初版文档曾假设"CR12A 臂长 1.2m 比 HC10 的 1.4m 短"，
> 这是**错误的**。两者官方有效工作半径均为 1200mm。

---

## 三、影响链路：DH参数改变 → 运动学模型改变 → IK行为改变

### 步骤1：DH参数改变的本质

```
机械臂的运动学模型建立在DH参数之上

HC10DT 的 DH 参数 → 得到 T₁, T₂, ..., T₆
CR12A  的 DH 参数 → 得到 T₁', T₂', ..., T₆'

两者仅在关节数上相同，几何参数完全不同
```

### 步骤2：替换URDF后发生什么

```xml
<!-- 当前 workcell.xacro:3-4 -->
<xacro:include filename="$(find motoman_hc10_support)/urdf/hc10dt_macro.xacro" />
<xacro:motoman_hc10dt prefix=""/>

<!-- 替换为 -->
<xacro:include filename="$(find cr12a_support)/urdf/cr12a_macro.xacro" />
<xacro:cr12a_macro prefix=""/>
```

```
MoveIt 读取新 URDF
  ↓
加载 CR12A 的运动学参数
  ↓
重新构建 KDL 运动学链
  ↓
Forward/Inverse Kinematics 全部改变
```

### 步骤3：IK求解的行为变化

```
IK求解器（KDLInvKinChainLMA，当前配置）的工作方式：

  1. 从当前关节角 θ_start 出发
  2. 迭代调整 θ，直到末端到达目标位姿
  3. 受关节限制约束
  4. 返回第一个满足条件的解

关键：解的搜索空间由「关节范围」决定
```

---

## 四、真正的风险：±360° 导致 IK 多解 🔴

### 这是最重要的技术风险

```
HC10DT 关节范围（约 ±180°）:
  目标姿态对应 θ₂ = 10°
  → 搜索空间 [-180°, 180°]
  → 唯一合理解: 10°
  → 解确定 ✅

CR12A 关节范围（±360°）:
  目标姿态对应 θ₂ = 10°
  → 搜索空间 [-360°, 360°]
  → 合理解: 10° / 370° / -350° / 730° ...
  → 解不唯一 ⚠️
```

### 故障模式 1：关节绕整圈

```
打磨路径相邻两点（相距仅 30mm）:

  Point_1 求解 → θ₂ = 355°
  Point_2 求解 → θ₂ = 5°      (数学等价，物理上差 350°)

规划器看到:
  θ₂: 355° → 5°  =  -350° 的大幅旋转 ⚠️

实际现象:
  ❌ 机械臂在打磨过程中突然"绕一整圈"
  ❌ 路径执行时间暴增（原本 1 秒变成 5 秒）
  ❌ 可能与工件/夹具/桌面发生碰撞
```

### 故障模式 2：Descartes 求解器建图失败

```
Descartes 要求相邻路径点的关节解是"连续的"
  ↓
±360° 下 KDL 可能返回不连续解（相差 360°）
  ↓
LadderGraphSolver failed to build graph
```

> 📌 **这是本项目已经踩过的坑！**
> `docs/../README.md` 记载：当前 HC10 配置曾报 `LadderGraphSolver failed to build graph`，
> 解法是"对工具路径做法向过滤，把法向量写死为朝上 (0,0,1)"。
>
> 换成 CR12A 后，**±360° 会让这个问题更容易复现**，
> 且原有的"法向量写死"解法可能不足以解决。

### 故障模式 3：奇异点位置改变（依然存在）

```
HC10DT 可以正常求解的关节配置
  ↓
CR12A 可能落入奇异点（雅可比矩阵行列式 → 0）
  ↓
IK 求解器无法反演
  ↓
返回无解
```

### 三种模式的对比

| 故障模式 | 触发条件 | 症状 | 严重度 |
|---------|---------|------|--------|
| 关节绕整圈 | ±360° 多解 | 路径时间暴增、碰撞 | 🔴 高 |
| Descartes 建图失败 | 解不连续 | `LadderGraphSolver failed` | 🔴 高 |
| 奇异点 | 几何构型 | IK 返回无解 | 🟡 中 |

---

## 五、解决方案

### 方案 A：收窄 URDF 关节范围（推荐）

```xml
<!-- 在 CR12A 的 URDF 中人为限制关节范围 -->
<joint name="cr12a_joint_1" type="revolute">
  <limit lower="-3.14159265" upper="3.14159265"
         effort="150.0" velocity="2.618"/>
  <!-- ±π (±180°) 而非 ±2π (±360°)，消除多圈歧义 -->
</joint>

<!-- J3 保持官方值 ±160° -->
<joint name="cr12a_joint_3" type="revolute">
  <limit lower="-2.7925268" upper="2.7925268" .../>
</joint>
```

| 优点 | 缺点 |
|------|------|
| 实现简单，改 URDF 即可 | 损失约 50% 的关节自由度 |
| 与现有 Descartes 配置兼容 | 极端姿态可能不可达 |
| 立即消除 360° 跳变 | — |

**预计工作量：1-2 小时**

### 方案 B：启用关节连续性约束

```yaml
# Tesseract 运动规划配置
kinematic_plugins:
  ...
  isometry_
# 在 Descartes 中配置关节连续性
```

| 优点 | 缺点 |
|------|------|
| 保留全部自由度 | 配置复杂，文档少 |
| 更优雅 | 调试困难 |

**预计工作量：4-6 小时**

### 方案 C：后处理关节角 unwrap

```python
def unwrap_joint_trajectory(trajectory, limits=2*np.pi):
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

| 优点 | 缺点 |
|------|------|
| 保留全部自由度 | 需要自研代码并接入规划流程 |
| 可控性最强 | 需处理边界情况 |

**预计工作量：3-5 小时**

### 方案对比

| 方案 | 工作量 | 保留自由度 | 推荐度 |
|------|--------|-----------|--------|
| A. 收窄URDF | 1-2h | 50% | ⭐⭐⭐ **推荐** |
| B. 连续性约束 | 4-6h | 100% | ⭐⭐ |
| C. 后处理unwrap | 3-5h | 100% | ⭐⭐ |

---

## 六、验证方法

### 验证 1：IK 往返一致性测试

```python
#!/usr/bin/env python3
"""验证 CR12A 的 IK 求解是否稳定"""
import numpy as np

def test_ik_roundtrip(ik_solver, fk_func, num_samples=200, tol=1e-3):
    """从随机关节角出发，做 FK→IK 往返，检查是否回到原点"""
    failures = []
    jumps = []

    for i in range(num_samples):
        # 随机生成合法的关节角
        q_true = np.random.uniform(-np.pi, np.pi, 6)

        # FK: 关节角 → 末端位姿
        pose = fk_func(q_true)

        # IK: 末端位姿 → 关节角
        q_solved = ik_solver.solve(pose)

        if q_solved is None:
            failures.append((i, 'no solution'))
            continue

        # 检查是否有 360° 跳变
        diff = q_solved - q_true
        if np.any(np.abs(diff) > np.pi):
            jumps.append((i, diff))

        # 检查 FK 结果是否一致（关节角可能不同，但位姿应相同）
        if not np.allclose(fk_func(q_solved), pose, atol=tol):
            failures.append((i, 'pose mismatch'))

    print(f"失败数: {len(failures)}/{num_samples}")
    print(f"360°跳变数: {len(jumps)}/{num_samples}")

    if jumps:
        print("⚠️  检测到关节跳变，建议收窄关节范围或启用连续性约束")
    if len(failures) > num_samples * 0.1:
        print("⚠️  失败率 > 10%，IK 配置可能有问题")

    return failures, jumps
```

**预期结果：**
- 未收窄关节范围时：跳变数 > 0（正常现象，说明问题存在）
- 收窄后：跳变数应为 **0**

### 验证 2：打磨路径连续性检查

```python
def check_path_continuity(trajectory, max_delta=np.pi/2):
    """检查打磨路径相邻点的关节角变化是否合理"""
    problems = []

    for i in range(1, len(trajectory)):
        for j in range(6):
            delta = abs(trajectory[i][j] - trajectory[i-1][j])
            if delta > max_delta:
                problems.append({
                    'segment': i,
                    'joint': j,
                    'delta_deg': np.degrees(delta)
                })

    if problems:
        print(f"⚠️  发现 {len(problems)} 处异常跳变:")
        for p in problems[:10]:
            print(f"  路径段 {p['segment']}, 关节 J{p['joint']+1}: "
                  f"{p['delta_deg']:.1f}°")
    else:
        print("✅ 路径连续性正常")

    return problems
```

### 验证 3：零位位置精度对比

```bash
# 启动仿真后，对比 FK 零位
ros2 run tf2_ros tf2_echo base_link tool0

# 记录输出，与理论值对比
```

---

## 七、结论

```
DH参数改变是🔴最高风险因素之一，但风险的具体形态与初版判断不同：

❌ 原以为的风险：臂长缩短、关节超限
   → 已证伪（两者半径相同、CR12A关节范围更大）

✅ 实际的风险：±360° 关节范围导致 IK 多解
   → 相邻路径点可能出现 360° 跳变
   → 触发 Descartes 求解失败（与已踩过的坑同源）

建议：
  1. 优先采用「方案A：收窄URDF关节范围」，1-2小时即可验证
  2. 用「验证1」的往返测试确认跳变是否消除
  3. 保留 HC10DT 配置作为回滚点
```

---

## 参考来源

- [Yaskawa HC10DT 官方产品页](https://www.yaskawa.co.uk/products/robots/collaborative/productdetail/product/hc10dt_681)
- [Dobot CR12A - RobotShop](https://eu.robotshop.com/products/dobot-cr12a-6-axis-collaborative-robot-arm-12kg-1200mm)
- [越疆 CR12 官方页面](https://www.dobot.cn/products/cr-series/cr12.html)

---

## 版本历史

| 版本 | 日期 | 作者 | 变更 |
|------|------|------|------|
| v1.0 | 2026-09-17 | 浮浮酱 | 初始版本 |
| v1.1 | 2026-09-17 | 浮浮酱 | 修正关节范围数据（±180° → ±360°），更正风险方向，补充验证方法 |

---

**最后更新：** 2026-09-17
