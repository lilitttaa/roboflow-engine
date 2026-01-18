#!/usr/bin/env python3
"""
BVH to NPZ converter using GMR.
Outputs .npz format that can be directly loaded by MotionFlow.

Usage:
    python3 scripts/bvh_to_npz.py \
        --bvh_file external/lafan/lafan1/bvh_data/walk1_subject1.bvh \
        --robot unitree_g1 \
        --save_path assets/motions/walk1_subject1.npz
"""

import argparse
import pathlib
import numpy as np
from general_motion_retargeting import GeneralMotionRetargeting as GMR
from general_motion_retargeting.utils.lafan1 import load_lafan1_file
from tqdm import tqdm

# G1 joint names (29 DOF)
G1_JOINT_NAMES = [
    "left_hip_pitch_joint",
    "left_hip_roll_joint",
    "left_hip_yaw_joint",
    "left_knee_joint",
    "left_ankle_pitch_joint",
    "left_ankle_roll_joint",
    "right_hip_pitch_joint",
    "right_hip_roll_joint",
    "right_hip_yaw_joint",
    "right_knee_joint",
    "right_ankle_pitch_joint",
    "right_ankle_roll_joint",
    "waist_yaw_joint",
    "waist_roll_joint",
    "waist_pitch_joint",
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
]


def main():
    parser = argparse.ArgumentParser(description="Convert BVH to NPZ using GMR")
    parser.add_argument("--bvh_file", required=True, type=str, help="Input BVH file")
    parser.add_argument("--robot", default="unitree_g1", 
                        choices=["unitree_g1", "unitree_g1_with_hands", "booster_t1", "stanford_toddy", "fourier_n1", "engineai_pm01"])
    parser.add_argument("--save_path", required=True, type=str, help="Output NPZ file")
    parser.add_argument("--no_viewer", action="store_true", help="Disable visualization")
    args = parser.parse_args()
    
    # Load BVH data
    print(f"Loading BVH: {args.bvh_file}")
    lafan1_data_frames, actual_human_height = load_lafan1_file(args.bvh_file)
    
    # Initialize retargeter
    retargeter = GMR(
        src_human="bvh",
        tgt_robot=args.robot,
        actual_human_height=actual_human_height,
    )
    
    motion_fps = 30
    qpos_list = []
    
    # Retarget each frame
    print(f"Retargeting {len(lafan1_data_frames)} frames...")
    for i in tqdm(range(len(lafan1_data_frames)), desc="Retargeting"):
        smplx_data = lafan1_data_frames[i]
        qpos = retargeter.retarget(smplx_data)
        qpos_list.append(qpos)
    
    # Convert to arrays
    qpos_array = np.array(qpos_list)
    root_pos = qpos_array[:, :3].astype(np.float32)
    # Convert quaternion from wxyz to xyzw
    root_rot_wxyz = qpos_array[:, 3:7]
    root_rot = root_rot_wxyz[:, [1, 2, 3, 0]].astype(np.float32)  # xyzw
    dof_pos = qpos_array[:, 7:].astype(np.float32)
    
    # Save as NPZ
    print(f"Saving to: {args.save_path}")
    np.savez(
        args.save_path,
        fps=np.array([motion_fps], dtype=np.float32),
        root_pos=root_pos,
        root_rot=root_rot,  # xyzw quaternion
        dof_pos=dof_pos,
        joint_names=np.array(G1_JOINT_NAMES),
    )
    
    # Print info
    print(f"  FPS: {motion_fps}")
    print(f"  Frames: {len(qpos_list)}")
    print(f"  Joints: {dof_pos.shape[1]}")
    print(f"  File size: {pathlib.Path(args.save_path).stat().st_size / 1024:.1f} KB")


if __name__ == "__main__":
    main()
