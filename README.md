# SNP 打磨仿真 · 只打磨坐面顶面（zhixingceng 分支）

在 SNP Automate 2023 打磨仿真中，把工件替换为**凳面坐板**，并让走刀路径**只打磨坐面顶面、不走底面**。

## 今天做了什么（2026-09-16）

1. **工件替换**：把仿真工件从完整椅子换成凳面坐板（切掉桌腿，只保留坐面）。
2. **只走顶面**：通过 `ROISelection` 圈选 + 实体薄板重建，让路径规划只生成坐面顶面的打磨路径，不再出现底面路径。
3. **全链路跑通**：圈选 ✅ → 工具路径规划 ✅ → 运动规划 ✅ → 执行打磨 ✅。

## 关键踩坑记录

| 坑 | 现象 | 解决 |
|---|---|---|
| ASCII PLY 被当二进制读 | 坐标读出垃圾数据，误判成"单位问题" | `part_scan.ply` 是 `format ascii 1.0`，按文本解析 |
| OBJ 坐标 ≠ 仿真坐标 | 直接把 OBJ 坐标写进 PLY 导致位置错乱 | 在仿真 PLY **自身坐标系内**做平面检测 |
| 单面网格不能用 | RViz 显示黑色 + 报 `Failed to concatenate normals` | 重建为**有厚度的封闭实体**（10mm 网格 / 15mm 厚） |
| ROISelection 参数不对 | 顶底被当同一平面，或圈选为空 | 参数落在「点间距 10mm < 参数 12mm < 板厚 15mm」窗口内 |

## 坐面识别方法

在仿真 PLY 自身坐标系内，取**面积最大的朝上平面**：

- 法向量 (0, 0, 1)，Z = 0.317002
- 面积 0.05880 m²（= 0.28 × 0.21，与坐面尺寸吻合）

## 当前状态

| 阶段 | 状态 | 备注 |
|------|------|------|
| 工件集成 | ✅ | 实体薄板正常显示 |
| 圈选 | ✅ | 只圈选顶面，无底面 |
| 工具路径规划 | ✅ | 仅顶面生成路径 |
| 运动规划 | ✅ | 生成可行轨迹 |
| 执行层 | ✅ | 打磨头正常执行，可完整打磨 |

## 执行层（已解决）

曾报 `LadderGraphSolver failed to build graph`（Descartes 求解器无可行解），现已跑通。

**解决方式**：对工具路径做**法向过滤**，并把法向量**写死为朝上 (0,0,1)** —— 工具姿态一致后，路径点均可到达，Descartes 正常求解。

## 快速启动

```bash
cd snp-automate-2023-polishing-simulation
bash scripts/restart_demo.sh
```

> ⚠️ 启动后等约 5 秒（`start_reconstruction` 服务就绪）再操作，否则报 `unreachable`。

## 备用文件

| 文件 | 说明 |
|---|---|
| `meshes/part_scan.ply` | 当前生效版本（实体薄板 10mm 网格 / 15mm 厚） |
| `meshes/part_scan_backup.ply` | 原始完整工件（切掉桌腿，仅坐面），回滚点 |
| `meshes/part_scan_plate_dense15mm.ply` | 当前版本备份 |

## 相关链接

- 原项目：https://github.com/ros-industrial-consortium/snp_automate_2023
