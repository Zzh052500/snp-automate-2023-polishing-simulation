# 打磨路径规划优化指南

## 问题描述

在 CR12A 打磨仿真中，圈选大面积打磨区域后，虽然能生成坐标点和路径，但执行打磨时报错**"无法规划"**（`LadderGraphSolver failed to build graph`）。

---

## 根本原因分析

### 1. **路径点过多导致组合爆炸**

当圈选大面积区域时，根据 `config/tpp.yaml` 的配置：
- `line_spacing: 0.03` (线间距 3cm)
- `point_spacing: 0.03` (点间距 3cm，经过 UniformSpacing 后实际是 1.5cm)

**一个 20cm × 20cm 的区域就会生成约 200 个路径点**。

Descartes 的 LadderGraphSolver 需要为每个路径点求解多个 IK 分支（通常 4-8 个），然后在这些分支之间搜索最优路径。路径点越多，解空间呈指数增长：

```
解空间 = (IK分支数)^路径点数
```

**大面积 → 路径点多 → 解空间过大 → 求解超时或内存不足 → 规划失败**

---

### 2. **速度加速度限制过严**

当前配置（`launch/start.launch.xml`）：
- `max_translational_vel: 0.15` m/s
- `max_translational_acc: 0.50` m/s²

这些参数会传入 Descartes 的运动学约束检查。如果相邻路径点之间的距离/方向变化要求的速度超过限制，该分支就会被剪枝。

**速度限制 + 密集路径点 = 可行解急剧减少**

README 中已经明确警告：
> ⚠️ 别一次提太高。速度上限直接进 Descartes 的 LadderGraphSolver，提过头会报 `LadderGraphSolver failed to build graph`（找不到可行解）。

---

### 3. **CR12A 工作空间限制**

根据 README 第 3.1 节：
> 这台 CR12A 在「相机朝下」姿态下，末端 X 越远能压得越低 —— X ≈ 0.86 m 时 Z 最低 0.52 m；0.90 m 时约 0.45 m；0.92 m 时约 0.42 m。**再远就够不着了。**

如果圈选的区域超出可达范围，或者姿态要求过于严格（例如刀具必须严格垂直表面），IK 可能无解。

---

### 4. **碰撞检测过严**

`launch/start.launch.xml` 配置了碰撞检查：
```xml
<arg name="scan_disabled_contact_links" default="[table, base_link, floor, scan]"/>
<arg name="scan_reduced_contact_links" default="[sand_tcp]"/>
```

如果打磨路径经过的某些点会导致**打磨头 ↔ 大臂**或**连杆 ↔ 台面**距离过近，这些 IK 解会被过滤掉，进一步减少可行解。

---

## 优化方案

### 方案 1：调整路径规划参数（推荐）⭐

**目标**：减少路径点数量，降低解空间复杂度

修改 `config/tpp.yaml`：

```yaml
tool_path_planner:
  name: PlaneSlicerLegacy
  direction_generator:
    name: LocatedVectorDirection
    service_name: get_located_vector
  origin_generator:
    name: Centroid
  line_spacing: 0.05      # 原 0.03 → 加大到 5cm，减少扫描线数量
  point_spacing: 0.05     # 原 0.03 → 加大到 5cm
  min_hole_size: 0.15     # 原 0.10 → 加大，忽略小孔洞
  min_segment_size: 0.15  # 原 0.10 → 加大，过滤短线段
  bidirectional: true

tool_path_modifiers:
  - name: MovingAverageOrientationSmoothing
    window_size: 5
  - name: UniformSpacing
    point_spacing: 0.025   # 原 0.015 → 加大到 2.5cm，减少点数
    spline_degree: 2
    include_endpoints: false
  - name: SnakeOrganization
```

**效果估算**：
- 原配置：20×20cm 区域 ≈ 200 点
- 新配置：20×20cm 区域 ≈ 70 点（减少 65%）

---

### 方案 2：分区域打磨

将大面积区域手动拆分成多个小区域，逐个圈选并打磨。

**优点**：每个区域路径点少，规划成功率高  
**缺点**：需要手动操作多次，效率低

---

### 方案 3：降低速度要求

如果路径点减少后仍然失败，可以尝试**降低速度加速度限制**，让更多 IK 分支满足运动学约束：

修改 `launch/start.launch.xml` 的启动参数：

```bash
ros2 launch snp_automate_2023 start.launch.xml \
  max_translational_vel:=0.10 \
  max_translational_acc:=0.30
```

或者在 `start.launch.xml` 中直接修改默认值：

```xml
<arg name="max_translational_vel" default="0.10" />  <!-- 原 0.15 -->
<arg name="max_translational_acc" default="0.30" />  <!-- 原 0.50 -->
```

---

### 方案 4：调整 IK 求解器容忍度（进阶）

如果上游 `snp_motion_planning` 暴露了 Descartes 的配置参数，可以尝试：
- 增加 IK 求解超时时间
- 降低姿态约束（允许刀具轻微倾斜，而非严格垂直）
- 增加 LadderGraph 的搜索时间限制

**这需要查看 `snp_motion_planning` 的源码或文档，确认可调参数。**

---

### 方案 5：使用更粗糙的初筛 + 精细化打磨

1. **第一遍**：用粗间距（5-8cm）快速规划并打磨整个大面积
2. **第二遍**：对需要精细打磨的局部区域，用细间距（1.5-3cm）重新圈选

**优点**：兼顾覆盖率和精度  
**缺点**：需要两次操作

---

## 诊断方法

### 1. 查看规划日志

运行仿真后，查看终端输出，搜索关键词：
```bash
grep -i "LadderGraphSolver\|failed to build\|no valid solution" runtime/logs/*.log
```

如果看到：
- `LadderGraphSolver failed to build graph` → 解空间过大或速度限制过严
- `No valid IK solution` → 工作空间超限或碰撞
- `Timeout` → 路径点过多，求解超时

---

### 2. 检查路径点数量

在 RViz 中观察生成的刀具路径（Tool Path）：
- **绿色线**：规划成功的路径段
- **红色标记**：无法规划的点

如果红色标记密集出现，说明该区域的 IK 求解大量失败。

---

### 3. 验证工作空间可达性

使用 `scripts/check_scan_traj.py` 类似的思路，编写一个脚本检查：
- 打磨路径的所有点是否在机器人可达范围内
- 末端姿态（刀具垂直向下）在该位置是否有 IK 解

---

## 快速验证步骤

1. **备份原配置**
   ```bash
   cp config/tpp.yaml config/tpp.yaml.bak
   ```

2. **应用方案 1 的修改**（修改 `config/tpp.yaml`）

3. **重启仿真**
   ```bash
   bash scripts/restart_demo.sh
   ```

4. **测试小面积区域**
   - 先圈选一个 **10cm × 10cm** 的小区域
   - 点击 `Generate Tool Paths`
   - 观察是否成功生成路径

5. **逐步扩大面积**
   - 如果小区域成功，尝试 **15cm × 15cm**
   - 再尝试 **20cm × 20cm**
   - 找到当前配置下的**最大可规划面积**

6. **调整参数**
   - 如果仍然失败，继续加大 `line_spacing` 和 `point_spacing`
   - 或者降低速度限制（方案 3）

---

## 推荐配置（保守方案）

如果希望**优先保证规划成功**，使用以下配置：

**`config/tpp.yaml`**（粗间距）：
```yaml
line_spacing: 0.06
point_spacing: 0.06
min_hole_size: 0.20
min_segment_size: 0.20

tool_path_modifiers:
  - name: MovingAverageOrientationSmoothing
    window_size: 5
  - name: UniformSpacing
    point_spacing: 0.03  # 最终点间距 3cm
    spline_degree: 2
    include_endpoints: false
  - name: SnakeOrganization
```

**`launch/start.launch.xml`**（低速）：
```xml
<arg name="max_translational_vel" default="0.08" />
<arg name="max_translational_acc" default="0.25" />
```

**效果**：
- 可规划的最大面积：约 **25cm × 25cm**
- 路径点数：约 **50-80 点**
- 打磨精度：线间距 6cm，点间距 3cm（适合粗打磨）

---

## 后续改进方向

1. **升级到 Tesseract 规划器**：Descartes 对大规模路径支持有限，Tesseract 的 TrajOpt 求解器对密集路径点优化更好
2. **自适应采样**：在曲率大的区域密集采样，平坦区域稀疏采样
3. **多分辨率规划**：先用粗网格规划整体路径，再局部细化

---

## 相关文件

| 文件 | 作用 |
|---|---|
| `config/tpp.yaml` | 打磨路径规划参数（线/点间距、孔洞过滤） |
| `launch/start.launch.xml` | 速度加速度限制（第 19-22 行） |
| `scripts/check_scan_traj.py` | 碰撞检查脚本（可参考用于验证打磨路径） |

---

## 总结

**核心问题**：大面积 → 路径点多 → Descartes 求解器负担过重 → 规划失败

**核心解法**：
1. ⭐ **加大线/点间距**（最直接有效）
2. 降低速度要求（增加可行解）
3. 分区域打磨（降低单次复杂度）

建议**先实施方案 1**，如果还不行再叠加方案 3。保守起见，可以先用粗间距验证可行性，再根据需要逐步细化。

---

*生成日期：2026-09-18*
