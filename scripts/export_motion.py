#!/usr/bin/env python3
"""
将 gentle-humanoid-training 的重定向数据导出为 roboflow-engine 可读的二进制格式。

数据格式 (.motion 文件):
- Header:
    - magic: 4 bytes "MFMT"
    - version: uint32
    - fps: float32
    - num_frames: uint32
    - num_joints: uint32
    - joint_names_offset: uint32  (从文件开头到关节名称的偏移)
- Joint names (null-terminated strings, concatenated)
- Per-frame data (num_frames times):
    - root_pos: float32 x 3
    - root_quat_wxyz: float32 x 4
    - joint_pos: float32 x num_joints

使用方法:
    python export_motion.py <input.npz> <output.motion>
    
或批量转换:
    python export_motion.py <input_dir> <output_dir> --batch
"""

import argparse
import struct
import numpy as np
from pathlib import Path
import sys

# 添加 gentle-humanoid-training 到路径
sys.path.insert(0, str(Path(__file__).parent.parent.parent / "gentle-humanoid-training"))

# G1 机器人的关节名称（与 gentle-humanoid-training 中的顺序一致）
G1_JOINT_NAMES = [
    'left_hip_pitch_joint', 'left_hip_roll_joint', 'left_hip_yaw_joint',
    'left_knee_joint', 'left_ankle_pitch_joint', 'left_ankle_roll_joint',
    'right_hip_pitch_joint', 'right_hip_roll_joint', 'right_hip_yaw_joint',
    'right_knee_joint', 'right_ankle_pitch_joint', 'right_ankle_roll_joint',
    'waist_yaw_joint', 'waist_roll_joint', 'waist_pitch_joint',
    'left_shoulder_pitch_joint', 'left_shoulder_roll_joint', 'left_shoulder_yaw_joint',
    'left_elbow_joint', 'left_wrist_roll_joint', 'left_wrist_pitch_joint', 'left_wrist_yaw_joint',
    'right_shoulder_pitch_joint', 'right_shoulder_roll_joint', 'right_shoulder_yaw_joint',
    'right_elbow_joint', 'right_wrist_roll_joint', 'right_wrist_pitch_joint', 'right_wrist_yaw_joint'
]

# 从 URDF 名称到 motion 数据名称的映射（如果不同的话）
JOINT_NAME_MAP = {
    # G1_jy URDF 使用的名称 -> motion 数据中的名称
    'leg_l1_joint': 'left_hip_pitch_joint',
    'leg_l2_joint': 'left_hip_roll_joint', 
    'leg_l3_joint': 'left_hip_yaw_joint',
    'leg_l4_joint': 'left_knee_joint',
    'leg_l5_joint': 'left_ankle_pitch_joint',
    'leg_l6_joint': 'left_ankle_roll_joint',
    'leg_r1_joint': 'right_hip_pitch_joint',
    'leg_r2_joint': 'right_hip_roll_joint',
    'leg_r3_joint': 'right_hip_yaw_joint',
    'leg_r4_joint': 'right_knee_joint',
    'leg_r5_joint': 'right_ankle_pitch_joint',
    'leg_r6_joint': 'right_ankle_roll_joint',
    'waist_joint': 'waist_yaw_joint',
    'arm_l1_joint': 'left_shoulder_pitch_joint',
    'arm_l2_joint': 'left_shoulder_roll_joint',
    'arm_l3_joint': 'left_shoulder_yaw_joint',
    'arm_l4_joint': 'left_elbow_joint',
    'arm_r1_joint': 'right_shoulder_pitch_joint',
    'arm_r2_joint': 'right_shoulder_roll_joint',
    'arm_r3_joint': 'right_shoulder_yaw_joint',
    'arm_r4_joint': 'right_elbow_joint',
}

# 反向映射
MOTION_TO_URDF_MAP = {v: k for k, v in JOINT_NAME_MAP.items()}


def export_motion(input_path: Path, output_path: Path, use_urdf_names: bool = True):
    """
    将 npz 文件转换为 .motion 格式
    """
    print(f"Loading {input_path}...")
    data = dict(np.load(input_path, allow_pickle=True))
    
    # 获取数据
    fps = int(data.get('mocap_framerate', data.get('frequency', data.get('fps', 50))))
    root_pos = data['root_pos'].astype(np.float32)  # (T, 3)
    root_rot = data['root_rot'].astype(np.float32)  # (T, 4) xyzw
    dof_pos = data['dof_pos'].astype(np.float32)    # (T, J)
    
    joint_names_raw = data.get('joint_names', [])
    if hasattr(joint_names_raw, 'tolist'):
        joint_names_raw = joint_names_raw.tolist()
    
    num_frames = root_pos.shape[0]
    num_joints_in_data = dof_pos.shape[1]
    
    print(f"  FPS: {fps}")
    print(f"  Frames: {num_frames}")
    print(f"  Joints in data: {num_joints_in_data}")
    print(f"  Joint names: {joint_names_raw[:5]}...")
    
    # 将四元数从 xyzw 转换为 wxyz
    root_quat_wxyz = np.concatenate([root_rot[:, 3:4], root_rot[:, :3]], axis=-1)
    
    # 选择要导出的关节（按 G1_JOINT_NAMES 顺序）
    if use_urdf_names:
        # 使用 URDF 名称
        export_joint_names = []
        export_joint_indices = []
        for urdf_name, motion_name in JOINT_NAME_MAP.items():
            if motion_name in joint_names_raw:
                export_joint_names.append(urdf_name)
                export_joint_indices.append(joint_names_raw.index(motion_name))
    else:
        # 使用 motion 数据中的原始名称
        export_joint_names = []
        export_joint_indices = []
        for name in G1_JOINT_NAMES:
            if name in joint_names_raw:
                export_joint_names.append(name)
                export_joint_indices.append(joint_names_raw.index(name))
    
    num_joints = len(export_joint_names)
    print(f"  Exporting {num_joints} joints: {export_joint_names[:5]}...")
    
    # 提取选中关节的数据
    export_dof_pos = dof_pos[:, export_joint_indices] if export_joint_indices else np.zeros((num_frames, 0), dtype=np.float32)
    
    # 写入文件
    output_path.parent.mkdir(parents=True, exist_ok=True)
    
    with open(output_path, 'wb') as f:
        # Magic
        f.write(b'MFMT')
        
        # Version
        f.write(struct.pack('<I', 1))
        
        # FPS
        f.write(struct.pack('<f', float(fps)))
        
        # Num frames
        f.write(struct.pack('<I', num_frames))
        
        # Num joints
        f.write(struct.pack('<I', num_joints))
        
        # Joint names offset (will write later)
        joint_names_offset_pos = f.tell()
        f.write(struct.pack('<I', 0))  # placeholder
        
        # Frame data
        for i in range(num_frames):
            # root_pos: 3 floats
            f.write(struct.pack('<3f', *root_pos[i]))
            # root_quat_wxyz: 4 floats
            f.write(struct.pack('<4f', *root_quat_wxyz[i]))
            # joint_pos: num_joints floats
            if num_joints > 0:
                f.write(struct.pack(f'<{num_joints}f', *export_dof_pos[i]))
        
        # Joint names
        joint_names_offset = f.tell()
        for name in export_joint_names:
            f.write(name.encode('utf-8') + b'\x00')
        
        # Go back and write the offset
        f.seek(joint_names_offset_pos)
        f.write(struct.pack('<I', joint_names_offset))
    
    file_size = output_path.stat().st_size
    print(f"  Written to {output_path} ({file_size} bytes)")


def main():
    parser = argparse.ArgumentParser(description='Export motion data to roboflow-engine format')
    parser.add_argument('input', help='Input npz file or directory')
    parser.add_argument('output', help='Output .motion file or directory')
    parser.add_argument('--batch', action='store_true', help='Batch convert directory')
    parser.add_argument('--use-motion-names', action='store_true', 
                        help='Use motion data joint names instead of URDF names')
    args = parser.parse_args()
    
    input_path = Path(args.input)
    output_path = Path(args.output)
    
    if args.batch:
        if not input_path.is_dir():
            print(f"Error: {input_path} is not a directory")
            return 1
        
        npz_files = list(input_path.rglob('*.npz'))
        print(f"Found {len(npz_files)} npz files")
        
        for npz_file in npz_files:
            rel_path = npz_file.relative_to(input_path)
            out_file = output_path / rel_path.with_suffix('.motion')
            try:
                export_motion(npz_file, out_file, use_urdf_names=not args.use_motion_names)
            except Exception as e:
                print(f"  Error: {e}")
    else:
        if not input_path.exists():
            print(f"Error: {input_path} does not exist")
            return 1
        
        export_motion(input_path, output_path, use_urdf_names=not args.use_motion_names)
    
    return 0


if __name__ == '__main__':
    sys.exit(main())
