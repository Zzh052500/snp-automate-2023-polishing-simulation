# SNP 打磨仿真 - 执行层优化分支 (zhixingceng)

## 分支介绍

本分支是在原 SNP Automate 2023 打磨仿真基础上，**集成新工件坐面** 并进行 **执行层参数优化** 的工作分支。

## 主要改动

### 1. 工件替换与坐面提取

**目标**：只打磨坐面顶面，不生成底面打磨路径。

**坐标系关键点**（踩坑记录）：

| | OBJ 原始坐标 | 仿真内 PLY 坐标 |
|---|---|---|
| X | 0 ~ 0.28 | 0.6577 ~ 0.9377 |
| Y | 0 ~ 0.23 | -0.095 ~ 0.115 |
| Z | 0 ~ 0.21 | 0.2702 ~ 0.3170 |
| 坐面朝向 | **+Y**（凳子未立起） | **+Z** |

> ⚠️ 仿真内的 mesh 坐标经过平移+旋转，**不能直接把 OBJ 坐标写进 PLY**，否则位置完全错乱。

**坐面识别方法**：在仿真 PLY **自身坐标系内**做平面检测，取**面积最大的朝上平面**：
- 法向量 (0, 0, 1)，Z = 0.317002
- 面积 0.05880 m²（= 0.28 × 0.21，与坐面尺寸吻合）

> ⚠️ `part_scan.ply` 是 **ASCII PLY**（`format ascii 1.0`），不是二进制。
> 用 `struct.unpack` 按二进制解析会得到垃圾数据。

**实体薄板重建**（关键）：单面网格（无厚度）在仿真中**无法工作** —— 显示为黑色，且规划报错。
因此将坐面重建为**封闭实体薄板**：

| 参数 | 值 | 说明 |
|---|---|---|
| 网格间距 | 10 mm | 原坐面网格为 26~35mm，过疏 |
| 板厚 | 15 mm | 需明显大于网格间距 |
| 顶点/面数 | 1276 / 2548 | 顶面朝上 + 底面朝下 + 侧壁朝外 |

### 2. 规划参数调整

#### ROISelection 参数

```yaml
mesh_modifiers:
  - name: ROISelection
    max_cluster_size: 1000000000
    min_cluster_size: 5
    cluster_tolerance: 0.012
    plane_distance_threshold: 0.012
  - name: NormalsFromMeshFaces
```

**参数窗口原理**（本次解决方案的核心）：

```
网格点间距 10mm  <  参数 12mm  <  板厚 15mm
      ↑                            ↑
  能聚成一类                 能区分顶/底面
```

- 参数 **过大**（如 30mm > 板厚）→ 顶底被当成同一平面 → 法向量冲突
  → `Error invoking mesh modifier at index 1 ... Failed to concatenate normals into mesh vertex cloud`
- 参数 **过小** → 网格点聚不成类 → 圈选结果为空，同样报错

#### 运动规划参数
```
ompl_max_planning_time: 15.0 秒 (从 5.0 秒)
cartesian_tolerance: [0.02, 0.02, 0.02, 0.1, 0.1, 6.28]
(从 [0.01, 0.01, 0.01, 0.05, 0.05, 6.28])
```

### 3. Docker 配置更新
- 添加 meshes 目录挂载到容器

### 4. 启动脚本修复
- 修复 `test.launch.xml` 中 joint_state_publisher 的配置
- 处理 Windows 换行符问题

## 当前状态 ✅

| 阶段 | 状态 | 备注 |
|------|------|------|
| 工件集成 | ✅ 成功 | 实体薄板已集成并正常显示 |
| 圈选功能 | ✅ 成功 | ROISelection 只圈选顶面，无底面 |
| 工具路径规划 | ✅ 成功 | 仅顶面生成打磨路径 |
| 运动规划 | ✅ 成功 | 生成可行轨迹 |
| 执行层 | ⚠️ 待优化 | 打磨头在执行时卡住 |

## 已知问题

### 1. 打磨头执行卡住

**症状**：路径规划成功，但开始执行时卡住。

**根本原因**：Descartes 求解器无法在给定速度/加速度约束下找到可行解
```
Error: LadderGraphSolver failed to build graph.
```

### 2. 仿真启动竞争

**症状**：重启后立即操作，报 `Service 'start_reconstruction' is unreachable`。

**原因**：RViz 抢在 `reconstruction_sim_node` 启动完成前调用服务（约差 2 秒）。

**规避**：**等仿真完全启动（约 5 秒）再操作**。

## 备用文件

| 文件 | 说明 |
|---|---|
| `meshes/part_scan.ply` | 当前生效版本（实体薄板 10mm 网格 / 15mm 厚） |
| `meshes/part_scan_backup.ply` | 原始完整工件（切掉桌腿，仅坐面） |
| `meshes/part_scan_plate_5mm_OK.ply` | 5mm 薄板版本（显示正常但参数不匹配） |
| `meshes/part_scan_plate_dense15mm.ply` | 当前版本的备份 |

## 后续优化方向

### 1️⃣ 降低机械臂约束（推荐优先）
- 降低关节最大速度 / 加速度
- 增加加速度缓冲时间

### 2️⃣ 增加轨迹平滑处理
- 实施轨迹后处理（trajectory smoothing）
- 减少关节角度突变

### 3️⃣ 调整工具路径生成（TPP 优化）
- 调整 `line_spacing` / `point_spacing`
- 调整方向生成器算法

## 测试环境

```
操作系统：Ubuntu 24.04 / WSL2
ROS 版本：ROS 2 Jazzy
Docker：用于容器化仿真环境
机械臂：Motoman HC10（模拟器）
```

## 使用方法

### 启动仿真

```bash
cd snp-automate-2023-polishing-simulation
bash scripts/restart_demo.sh
```

> ⚠️ 启动后等待约 5 秒，等 `start_reconstruction` 服务就绪再操作。

### 切换到本分支

```bash
git checkout zhixingceng
```

## 相关链接

- 原工件仓库：https://github.com/Zzh052500/snp-workpiece-swap
- 原项目：https://github.com/ros-industrial-consortium/snp_automate_2023
- 主分支：main

## 备注

本分支是在原同事仓库基础上独立分支进行的工作，现已迁移到自己的 GitHub 账户，便于独立维护和优化。

---

**最后更新**：2026-09-16
**分支持有人**：Zzh052500
