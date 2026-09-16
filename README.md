# SNP 打磨仿真 - 执行层优化分支 (zhixingceng)

## 分支介绍

本分支是在原 SNP Automate 2023 打磨仿真基础上，**集成新工件坐面** 并进行 **执行层参数优化** 的工作分支。

## 主要改动

### 1. 工件替换
- **新工件**：`seat_only.ply`（来自 https://github.com/Zzh052500/snp-workpiece-swap）
- **尺寸**：0.28 × 0.21 × 0.05 m（比原工件小得多）
- **顶点数**：2256，面数：4082
- **位置**：`meshes/part_scan.ply`

### 2. 规划参数调整

#### ROISelection 参数优化
由于新坐面较小，调整了圈选参数以适配：
```yaml
mesh_modifiers:
  - name: ROISelection
    max_cluster_size: 1000000000
    min_cluster_size: 10          # 从 100 → 10
    cluster_tolerance: 0.05       # 从 0.1 → 0.05
    plane_distance_threshold: 0.05 # 从 0.1 → 0.05
```

#### 运动规划参数优化
增加求解器时间和放宽容差：
```
ompl_max_planning_time: 15.0 秒 (从 5.0 秒)
cartesian_tolerance: [0.02, 0.02, 0.02, 0.1, 0.1, 6.28]
(从 [0.01, 0.01, 0.01, 0.05, 0.05, 6.28])
```

### 3. Docker 配置更新
- 添加 meshes 目录挂载到容器
- 确保新坐面文件在容器内可用

### 4. 启动脚本修复
- 修复 `test.launch.xml` 中 joint_state_publisher 的配置
- 处理 Windows 换行符问题

## 当前状态 ✅

| 阶段 | 状态 | 备注 |
|------|------|------|
| 工件集成 | ✅ 成功 | 新坐面已集成并可视化 |
| 圈选功能 | ✅ 成功 | ROISelection 能识别坐面 |
| 工具路径规划 | ✅ 成功 | 生成法线和打磨路径 |
| 运动规划 | ✅ 成功 | 生成可行轨迹 |
| 执行层 | ⚠️ 待优化 | 打磨头在执行时卡住 |

## 已知问题

### 打磨头执行卡住

**症状**：
- 路径规划成功生成轨迹
- 开始执行时卡住，无法完成运动

**根本原因**：
新坐面（0.28 × 0.21 × 0.05 m）相比原工件结构完全不同，导致生成的打磨路径具有以下特点：
1. 路径点更密集
2. 关节角度变化幅度大
3. 局部加速度超出约束

**现象**：
Descartes 求解器无法在给定的速度/加速度约束下找到可行解
```
Error: LadderGraphSolver failed to build graph.
```

## 后续优化方向

### 1️⃣ 调整工具路径生成（TPP 优化）
- 增加工具路径点间距（降低密集程度）
- 调整方向生成器算法
- 改变平面分割策略

### 2️⃣ 降低机械臂约束（推荐优先）
- 降低关节最大速度
- 降低关节最大加速度
- 增加加速度缓冲时间

### 3️⃣ 增加轨迹平滑处理
- 实施轨迹后处理（trajectory smoothing）
- 减少关节角度的突变
- 实现 S 曲线加速度规划

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
chmod +x scripts/*.sh
./scripts/run_simulation.sh
```

### 切换到本分支

```bash
git checkout zhixingceng
```

## 提交信息

**主提交**：`08f8e62`
- 集成新坐面seat_only.ply并调整规划参数
- 更新 Docker 配置和启动脚本
- 优化 ROISelection 参数以适配新坐面

## 相关链接

- 原工件仓库：https://github.com/Zzh052500/snp-workpiece-swap
- 原项目：https://github.com/ros-industrial-consortium/snp_automate_2023
- 主分支：main

## 备注

本分支是在原同事仓库基础上独立分支进行的工作。现已迁移到主人自己的 GitHub 账户，便于独立维护和优化。

---

**最后更新**：2026-09-16  
**分支持有人**：Zzh052500
