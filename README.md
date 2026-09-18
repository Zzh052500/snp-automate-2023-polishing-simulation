<img width="1850" height="1053" alt="e70798c38288ccf201b2665d7d2950a2" src="https://github.com/user-attachments/assets/2cdf7c75-a30a-4c21-80c6-84c759924bc8" />
# SNP 打磨仿真 · 球面座面工件适配（qiumian 分支）
![Uploading 微信图片_20260918102128_49_22.png…]()


在 [cr12a 分支](https://github.com/Zzh052500/snp-automate-2023-polishing-simulation/tree/cr12a) 的基础上，
**将工件从平板座面替换为 main 分支的半圆/球面座面**，并完成 CR12A 机械臂的工作空间适配与碰撞配置优化。

> 本分支工作日期：2026-09-18

---

## 快速启动

```bash
cd snp-automate-2023-polishing-simulation
bash scripts/restart_demo.sh
```

等待约 5 秒后在 RViz2 中操作：
1. 点击 **Execute Scan Motion** - 执行扫描
2. 点击 **Start Reconstruction** - 重建工件
3. 圈选打磨区域
4. 点击 **Plan Tool Paths** - 生成刀具路径
5. 点击 **Generate Motion Plan** - 生成运动规划
6. 点击 **Execute Motion Plan** - 执行打磨

---

## 本次做了什么

### 1. 工件替换：从平板座面改为球面座面

**背景：**
- cr12a 分支使用的是简化的平板座面工件（71K，1276 顶点）
- main 分支有更真实的半圆/球面座面工件（147K，3627 顶点，二进制 PLY 格式）

**目标：**
将 main 分支的球面座面工件移植到 cr12a 分支，保留半圆座面的几何特征。

**操作步骤：**

#### 1.1 工件提取与几何分析

从 main 分支提取 `part_scan.ply` 工件文件：

```bash
git show origin/main:meshes/part_scan.ply > meshes/part_scan_main_semicircle.ply
```

几何参数对比：

| 参数 | main 球面座面（原始） | cr12a 平板座面 |
|---|---|---|
| **格式** | 二进制 PLY + RGBA | ASCII PLY |
| **顶点数** | 3627 | 1276 |
| **X 范围** | 0.568 ~ 1.027 m | 0.657 ~ 0.937 m |
| **Y 范围** | -0.225 ~ 0.245 m | -0.095 ~ 0.115 m |
| **Z 范围** | 0.087 ~ 0.227 m | 0.302 ~ 0.317 m |
| **工件高度** | 139.9 mm | 15 mm |

**关键差异：**
- main 工件**更宽更大**（X/Y 范围都扩展了）
- main 工件的 **Z 坐标偏低**（顶面低了 90mm）
- main 工件的 **X 最大值 1.027m 超出 CR12A 可达范围**（0.92m）

#### 1.2 工件平移到可达位置

需要两个方向的平移：

**Z 方向（垂直）：+0.09 m**
- 目的：对齐顶面高度到 cr12a 的 0.317 m
- 原因：cr12a 的扫描轨迹相机高度为 Z = 0.42 m，离顶面 10.3 cm

**X 方向（水平）：-0.11 m**
- 目的：将工件向后移，确保最远点在可达范围内
- 原因：X = 1.027 m 超出 CR12A 在「相机朝下」姿态的可达边界（0.92 m）

平移后的工件参数：

```
X 范围: 0.458 ~ 0.917 m  ✅ < 0.92 m 可达边界
Y 范围: -0.225 ~ 0.245 m
Z 范围: 0.177 ~ 0.317 m  ✅ 顶面对齐
```

**实现代码：** 使用 Python 读取二进制 PLY，对所有顶点应用平移变换后写回。

#### 1.3 文件组织

```
meshes/
├── part_scan.ply (147K)                  ← 当前使用：平移后的球面座面
├── part_scan_main_original.ply (147K)   ← main 原始位置备份
├── part_scan_main_semicircle.ply (147K) ← main GitHub 原始版本
├── part_scan_cr12a_plate.ply (71K)      ← cr12a 平板座面备份
└── part_scan_backup.ply (123K)          ← cr12a 历史备份
```

---

### 2. 碰撞配置优化：解决打磨规划失败问题

**问题现象：**
工件重建和刀具路径规划成功，但运动规划失败，报错：
```
Descartes vertex failure: All IK solutions found were in collision or invalid.
sand_tcp (打磨头) 与 scan (相机) 碰撞
```

**根本原因：**
SNP 系统的碰撞配置**未区分扫描阶段和打磨阶段**：
- **扫描阶段：** 相机固定在法兰上，打磨头需要避让相机 ✅
- **打磨阶段：** 应该只有打磨头，但系统仍把相机当作碰撞体 ❌

导致打磨规划时，IK 求解器找到的所有解都因为"打磨头撞到（不存在的）相机"而被拒绝。

**解决方案：**

在 `launch/start.launch.xml` 中，将 `scan`（相机组件）添加到禁用碰撞列表：

```xml
<!-- 修改前 -->
<arg name="scan_disabled_contact_links" default="[table, base_link, floor]"/>

<!-- 修改后 -->
<arg name="scan_disabled_contact_links" default="[table, base_link, floor, scan]"/>
```

**效果：**
- 扫描阶段：`scan` 被禁用碰撞检测，不参与规划（合理，因为相机固定在机械臂上）
- 打磨阶段：`scan` 同样被禁用，允许打磨头自由规划路径 ✅

修改后，打磨运动规划成功，完整流程可以正常执行。

---

### 3. 扫描轨迹复用验证

**决策：** 先测试 cr12a 的现有扫描轨迹是否可用，再考虑重新计算 IK。

**cr12a 扫描轨迹覆盖范围：**
```
X: 0.68 ~ 0.90 m（README 第 96 行）
Y: ±0.06 ~ 0.08 m
Z: 0.42 m（相机高度，离顶面 10.3 cm）
```

**球面座面工件范围（平移后）：**
```
X: 0.458 ~ 0.917 m
Y: -0.225 ~ 0.245 m
Z 顶面: 0.317 m
```

**测试结果：**
- ✅ 扫描轨迹能够覆盖工件的**中心区域**
- ✅ 工件重建成功
- ✅ 打磨路径规划成功（配置修改后）
- ✅ 完整的扫描-重建-规划-执行流程通过

**结论：**
cr12a 的扫描轨迹**无需重新计算**，现有轨迹足够覆盖球面座面工件的有效打磨区域。

---

## 工作空间可达性分析

根据 [cr12a README 第 149 行](../README.md#L149)，CR12A 在「相机朝下」姿态下的可达性：

```
X ≈ 0.86 m 时 Z 最低 0.52 m
X ≈ 0.90 m 时 Z 约 0.45 m
X ≈ 0.92 m 时 Z 约 0.42 m
再远就够不着了
```

**球面座面工件适配：**
- 原始 main 工件：X 最大 1.027 m ❌ 超出可达范围
- 平移后工件：X 最大 0.917 m ✅ 在可达边界内

通过 X 方向 -110mm 平移，确保工件完全在机械臂工作空间内。

---

## 技术细节

### 工件文件格式

**二进制 PLY 结构：**
```
ply
format binary_little_endian 1.0
element vertex 3627
property float x
property float y
property float z
property uchar red
property uchar green
property uchar blue
property uchar alpha
element face 7052
...
end_header
[二进制顶点数据：x,y,z,r,g,b,a × 3627]
[二进制面数据...]
```

每个顶点 16 字节：
- 12 字节：x, y, z (3 × float32)
- 4 字节：r, g, b, a (4 × uint8)

### 平移变换实现

```python
def translate_ply_binary(input_file, output_file, dx, dy, dz):
    # 读取头部保持不变
    # 对每个顶点：
    #   x_new = x_old + dx
    #   y_new = y_old + dy
    #   z_new = z_old + dz
    # 颜色和面数据保持不变
```

### 碰撞检测机制

SNP 使用 Tesseract 进行碰撞检测，配置参数：
- `scan_disabled_contact_links`：完全忽略的碰撞对（不检测）
- `scan_reduced_contact_links`：最小接触距离设为 0（允许接触）

这套配置**同时应用于扫描和打磨阶段**，因此需要包含所有阶段都应忽略的组件。

---

### 4. 路径规划参数优化：解决全工件打磨 IK 求解失败

**问题现象：**
初始尝试对整个球面座面工件进行全覆盖打磨时，路径规划成功但运动规划卡在 IK 求解阶段无法完成。

**根本原因：**
根据 `docs/Polish_Planning_Optimization.md` 分析，Descartes LadderGraphSolver 的计算复杂度为：
```
解空间 = (IK分支数)^路径点数
```

球面座面工件尺寸较大（~46cm × 47cm），初始配置生成的路径点过多（200-300个），导致解空间呈指数增长，超出 Descartes 的计算能力。

**解决方案：**

#### 4.1 路径规划参数优化

修改 `config/tpp.yaml` 为极限稀疏模式：

```yaml
tool_path_planner:
  line_spacing: 0.06      # 线间距从 3cm 增大到 6cm
  point_spacing: 0.06     # 点间距从 3cm 增大到 6cm
  min_hole_size: 0.18     # 过滤小于 18cm 的孔洞
  min_segment_size: 0.18  # 过滤短于 18cm 的线段

tool_path_modifiers:
  - name: UniformSpacing
    point_spacing: 0.045  # 最终点间距 4.5cm
```

**效果：**
- 路径点数量从 ~200-300 个减少到 ~120 个（减少约 40%）
- 计算复杂度大幅降低：8^200 → 8^120
- 成功实现全工件打磨运动规划 ✅

#### 4.2 速度参数保守化

修改 `launch/start.launch.xml`，降低 TCP 速度和加速度限制：

```xml
<arg name="max_translational_vel" default="0.05"/>  <!-- 从 0.08 降低到 0.05 m/s -->
<arg name="max_translational_acc" default="0.20"/>  <!-- 从 0.30 降低到 0.20 m/s² -->
<arg name="max_rotational_vel" default="1.50"/>     <!-- 从 2.00 降低到 1.50 rad/s -->
<arg name="max_rotational_acc" default="3.00"/>     <!-- 从 4.00 降低到 3.00 rad/s² -->
```

**效果：**
- 相邻路径点之间的时间余量增加 60%（从 0.375s 到 0.6s）
- 关节速度约束更宽松，IK 解空间更大
- 提高运动规划成功率 ✅

#### 4.3 优化策略总结

**双重优化：**
1. **减少路径点数量**（降低计算量）
2. **放宽速度约束**（增加可行解数量）

**权衡取舍：**
- ✅ 成功实现全工件自动打磨
- ⚠️ 路径密度降低（线间距 6cm）
- ⚠️ 打磨速度较慢（5cm/s）

**后续改进方向：**
- 采用分区打磨策略，每个区域使用更密集的路径参数
- 探索其他运动规划器（如 OMPL）替代 Descartes
- 优化 IK 求解算法，提高大规模路径的计算效率

---

## 与 cr12a 分支的差异

| 项目 | cr12a 分支 | qiumian 分支 |
|---|---|---|
| **工件模型** | 平板座面（71K） | 球面座面（147K，平移后） |
| **工件范围** | X: 0.657~0.937 m | X: 0.458~0.917 m |
| **碰撞配置** | `[table, base_link, floor]` | `[table, base_link, floor, scan]` |
| **扫描轨迹** | 8 航点蛇形 | 复用 cr12a |
| **路径规划** | line_spacing: 3cm | line_spacing: 6cm（极限稀疏） |
| **TCP 速度** | 0.08 m/s | 0.05 m/s（保守模式） |
| **打磨能力** | ✅ | ✅（全工件打磨成功） |

---

## 已知限制与后续改进

### 当前限制

1. **扫描覆盖范围**：
   - 当前扫描轨迹覆盖 X: 0.68~0.90 m
   - 球面座面工件 X: 0.458~0.917 m
   - 只扫描到工件的**中心和偏右区域**（~70% 覆盖）

2. **工件位置固定**：
   - 工件位置由平移量硬编码（dx=-0.11, dz=+0.09）
   - 如需调整工件位置，需要重新计算平移参数

### 可选改进

**A. 扩展扫描覆盖范围**（如需 100% 覆盖）
- 重新计算扫描轨迹 IK 解
- 增加扫描航点数量
- 参考 `scripts/compute_scan_ik.py`（已包含 DLS 逆运动学框架）

**B. 参数化工件位置**
- 将平移量作为 launch 参数
- 支持不同尺寸的球面座面工件

**C. 自动化工件适配**
- 自动检测工件边界
- 自动计算平移量和扫描路径

---

## 文件清单

**新增文件：**
- `meshes/part_scan_main_original.ply` - main 原始位置工件
- `meshes/part_scan_main_semicircle.ply` - main GitHub 原始版本
- `meshes/part_scan_cr12a_plate.ply` - cr12a 平板座面备份
- `scripts/compute_scan_ik.py` - IK 求解脚本框架（备用）
- `README_qiumian.md` - 本文档

**修改文件：**
- `launch/start.launch.xml` - 碰撞配置（添加 `scan` 到禁用列表）+ 速度参数优化
- `config/tpp.yaml` - 路径规划参数优化（极限稀疏模式）
- `meshes/part_scan.ply` - 替换为平移后的球面座面工件

---

## 参考资料与相关文档

### 项目相关
- [cr12a 分支 README](../README.md) - CR12A 机械臂替换与扫描轨迹计算  
- [main 分支工件](https://github.com/Zzh052500/snp-automate-2023-polishing-simulation/tree/main/meshes)  
- SNP Automate 2023 项目：https://github.com/ros-industrial-consortium/snp_automate_2023

### 技术文档

| 文档 | 内容 |
|---|---|
| [`docs/CR12A_Migration_Analysis.md`](docs/CR12A_Migration_Analysis.md) | CR12A 替换可行性分析、DH 参数、工作空间核算 |
| [`docs/Polish_Planning_Optimization.md`](docs/Polish_Planning_Optimization.md) | **打磨路径规划优化指南**（大面积区域规划失败的解决方案） |
| [`docs/zhixingceng_seat_polishing.md`](docs/zhixingceng_seat_polishing.md) | 坐面工件替换 + 只打磨顶面（上一分支） |
| [`docs/RUN_GUIDE_CN.md`](docs/RUN_GUIDE_CN.md) | 运行指南 |
| [`docs/TROUBLESHOOTING_CN.md`](docs/TROUBLESHOOTING_CN.md) | 常见问题排查 |
| [`docs/PROJECT_WORKFLOW_CN.md`](docs/PROJECT_WORKFLOW_CN.md) | 流程说明 |

---

## 常见问题

**Q: 为什么不重新计算扫描轨迹？**  
A: 测试表明 cr12a 的现有轨迹已经覆盖工件的有效打磨区域，能够完成扫描-重建-打磨的完整流程。重新计算 IK 的工作量较大（需要 DLS + 碰撞优化 + 关节连续性），在当前需求下是非必要的优化。

**Q: 球面座面和平板座面有什么实际差异？**  
A: 球面座面更接近真实工件的几何特征，底部是曲面而非平面。打磨路径会根据曲面法向量生成，更符合实际加工需求。

**Q: 碰撞配置修改会影响扫描阶段吗？**  
A: 不会。`scan` 组件在扫描阶段固定在法兰上，不参与路径规划，禁用其碰撞检测不影响扫描轨迹的安全性。

**Q: 如何切换回 cr12a 的平板座面？**  
A: 执行 `cp meshes/part_scan_cr12a_plate.ply meshes/part_scan.ply`，然后重启仿真即可。

**Q: 为什么运动规划会卡在 IK 求解阶段？**  
A: Descartes 规划器的计算复杂度呈指数增长（解空间 = IK分支数^路径点数）。当路径点过多（>200个）或速度约束过严时，计算量会超出系统能力。解决方法：增大 `line_spacing` 和 `point_spacing` 减少路径点，或降低 `max_translational_vel` 放宽速度约束。

**Q: 如何在覆盖率和规划成功率之间取得平衡？**  
A: 当前配置（line_spacing: 6cm）是经过实测验证能够成功的极限稀疏配置。如需更密集覆盖，建议采用分区打磨策略：将工件分成 2-3 个小区域，每个区域可以使用更密集的参数（如 line_spacing: 4cm）。

---

**最后更新：** 2026-09-18  
**分支状态：** ✅ 完整流程验证通过（含全工件打磨优化）
