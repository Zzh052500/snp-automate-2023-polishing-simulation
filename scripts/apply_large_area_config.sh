#!/bin/bash
# 应用大面积打磨路径规划优化配置
# 用法：bash scripts/apply_large_area_config.sh [restore]

set -e

CONFIG_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/config"
BACKUP_FILE="${CONFIG_DIR}/tpp.yaml.bak"
ORIGINAL_FILE="${CONFIG_DIR}/tpp.yaml"
OPTIMIZED_FILE="${CONFIG_DIR}/tpp_large_area.yaml"

# 恢复原始配置
if [ "$1" == "restore" ]; then
    if [ -f "$BACKUP_FILE" ]; then
        echo "✅ 恢复原始 tpp.yaml 配置..."
        cp "$BACKUP_FILE" "$ORIGINAL_FILE"
        echo "✅ 已恢复到原始配置"
        echo ""
        echo "当前配置："
        grep -E "line_spacing|point_spacing" "$ORIGINAL_FILE" | head -3
    else
        echo "❌ 备份文件不存在: $BACKUP_FILE"
        echo "   无法恢复原始配置"
        exit 1
    fi
    exit 0
fi

# 应用优化配置
echo "========================================"
echo "  应用大面积打磨路径规划优化配置"
echo "========================================"
echo ""

# 1. 备份原始配置
if [ ! -f "$BACKUP_FILE" ]; then
    echo "📦 备份原始配置到: tpp.yaml.bak"
    cp "$ORIGINAL_FILE" "$BACKUP_FILE"
else
    echo "ℹ️  备份文件已存在，跳过备份"
fi

# 2. 应用优化配置
echo "🔧 应用优化配置..."
cp "$OPTIMIZED_FILE" "$ORIGINAL_FILE"

echo ""
echo "✅ 配置已更新！"
echo ""
echo "主要变化："
echo "  - line_spacing:  0.03 → 0.05 m  (线间距增大)"
echo "  - point_spacing: 0.015 → 0.025 m (点间距增大)"
echo "  - min_hole_size: 0.10 → 0.15 m  (忽略更小的孔洞)"
echo ""
echo "效果："
echo "  - 路径点数量减少约 65%"
echo "  - 规划成功率提高"
echo "  - 适合 20cm × 20cm 或更大区域"
echo ""
echo "⚠️  注意："
echo "  - 打磨精度会降低（线间距 5cm，点间距 2.5cm）"
echo "  - 如需精细打磨，请分区域操作或恢复原配置"
echo ""
echo "恢复原配置命令："
echo "  bash scripts/apply_large_area_config.sh restore"
echo ""
echo "现在请重启仿真："
echo "  bash scripts/restart_demo.sh"
echo "========================================"
