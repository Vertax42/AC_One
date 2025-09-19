#!/bin/bash

# 评估脚本使用示例

echo "🚀 开始模型评估示例..."

# 设置环境
cd /home/Xense/AC_One/act

# 激活conda环境
source /home/Xense/miniconda3/etc/profile.d/conda.sh
conda activate act

# 基础评估 (无temporal aggregation)
echo ""
echo "🔍 运行基础评估 (无temporal aggregation)..."
python eval.py \
    --data data/config.yaml \
    --ckpt_dir weights \
    --dataset_path datasets/episode_0.hdf5 \
    --camera_names head left_wrist right_wrist \
    --policy_class ACT \
    --chunk_size 50 \
    --hidden_dim 512 \
    --dim_feedforward 3200 \
    --kl_weight 10 \
    --lr 1e-5

echo ""
echo "🔍 运行Temporal Aggregation评估..."
python eval.py \
    --data data/config.yaml \
    --ckpt_dir weights \
    --dataset_path datasets/episode_0.hdf5 \
    --camera_names head left_wrist right_wrist \
    --policy_class ACT \
    --chunk_size 50 \
    --hidden_dim 512 \
    --dim_feedforward 3200 \
    --kl_weight 10 \
    --lr 1e-5 \
    --temporal_agg \
    --max_predictions 20

echo ""
echo "✅ 评估完成!"
