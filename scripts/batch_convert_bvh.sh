#!/bin/bash
# 批量转换 BVH 文件到 NPZ 格式

cd /home/qingzhe/workspace/Projects/roboflow-engine

# 需要转换的文件列表（行走和跑步）
FILES=(
    "walk1_subject2"
    "walk1_subject5"
    "walk2_subject1"
    "walk2_subject3"
    "walk2_subject4"
    "walk3_subject1"
    "walk3_subject2"
    "walk3_subject3"
    "walk3_subject4"
    "run1_subject5"
    "run2_subject1"
    "run2_subject4"
)

for name in "${FILES[@]}"; do
    bvh="external/lafan/lafan1/bvh_data/${name}.bvh"
    npz="assets/motions/${name}.npz"
    
    if [ -f "$bvh" ] && [ ! -f "$npz" ]; then
        echo "Converting: $name"
        python3 scripts/bvh_to_npz.py --bvh_file "$bvh" --robot unitree_g1 --save_path "$npz" --no_viewer
        echo ""
    else
        if [ -f "$npz" ]; then
            echo "Skipping $name (already exists)"
        else
            echo "Skipping $name (source not found)"
        fi
    fi
done

echo "Done!"
