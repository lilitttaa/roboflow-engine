#!/usr/bin/env python3
"""
生成测试用的动作数据文件。
这个脚本会生成一个简单的行走动作，用于测试 roboflow-engine 的动作播放功能。
"""

import struct
import numpy as np
from pathlib import Path

# G1 机器人的关节名称（使用 G1_jy URDF 中的名称）
G1_URDF_JOINT_NAMES = [
    'leg_l1_joint',  # left_hip_pitch
    'leg_l2_joint',  # left_hip_roll
    'leg_l3_joint',  # left_hip_yaw
    'leg_l4_joint',  # left_knee
    'leg_l5_joint',  # left_ankle_pitch
    'leg_l6_joint',  # left_ankle_roll
    'leg_r1_joint',  # right_hip_pitch
    'leg_r2_joint',  # right_hip_roll
    'leg_r3_joint',  # right_hip_yaw
    'leg_r4_joint',  # right_knee
    'leg_r5_joint',  # right_ankle_pitch
    'leg_r6_joint',  # right_ankle_roll
    'waist_joint',   # waist_yaw
    'arm_l1_joint',  # left_shoulder_pitch
    'arm_l2_joint',  # left_shoulder_roll
    'arm_l3_joint',  # left_shoulder_yaw
    'arm_l4_joint',  # left_elbow
    'arm_r1_joint',  # right_shoulder_pitch
    'arm_r2_joint',  # right_shoulder_roll
    'arm_r3_joint',  # right_shoulder_yaw
    'arm_r4_joint',  # right_elbow
]

def generate_walk_motion(fps=50, duration=5.0, output_path='walk.motion'):
    """生成一个简单的行走动作"""
    num_frames = int(fps * duration)
    num_joints = len(G1_URDF_JOINT_NAMES)
    
    # 初始化数据
    root_pos = np.zeros((num_frames, 3), dtype=np.float32)
    root_quat_wxyz = np.zeros((num_frames, 4), dtype=np.float32)
    root_quat_wxyz[:, 0] = 1.0  # w=1, x=y=z=0 (单位四元数)
    joint_pos = np.zeros((num_frames, num_joints), dtype=np.float32)
    
    # 生成行走动作
    walk_freq = 1.0  # 1Hz = 1步/秒
    for i in range(num_frames):
        t = i / fps
        phase = 2 * np.pi * walk_freq * t
        
        # 根位置：向前移动 + 轻微上下浮动
        root_pos[i, 0] = t * 0.5  # 前进速度 0.5 m/s (X方向在Z-up坐标系中)
        root_pos[i, 1] = 0.0      # 左右保持不变
        root_pos[i, 2] = 0.75 + 0.01 * np.sin(phase * 2)  # 高度 + 轻微上下浮动
        
        # 左腿（与右腿相位相反）
        # leg_l1: hip pitch
        joint_pos[i, 0] = 0.3 * np.sin(phase) - 0.2  # 摆动 + 基础弯曲
        # leg_l2: hip roll
        joint_pos[i, 1] = 0.05 * np.sin(phase)
        # leg_l3: hip yaw
        joint_pos[i, 2] = 0.0
        # leg_l4: knee
        joint_pos[i, 3] = 0.5 + 0.3 * max(0, np.sin(phase))  # 膝盖弯曲
        # leg_l5: ankle pitch
        joint_pos[i, 4] = -0.2 - 0.1 * np.sin(phase)
        # leg_l6: ankle roll
        joint_pos[i, 5] = 0.0
        
        # 右腿（与左腿相位相反）
        phase_r = phase + np.pi
        # leg_r1: hip pitch
        joint_pos[i, 6] = 0.3 * np.sin(phase_r) - 0.2
        # leg_r2: hip roll
        joint_pos[i, 7] = 0.05 * np.sin(phase_r)
        # leg_r3: hip yaw
        joint_pos[i, 8] = 0.0
        # leg_r4: knee
        joint_pos[i, 9] = 0.5 + 0.3 * max(0, np.sin(phase_r))
        # leg_r5: ankle pitch
        joint_pos[i, 10] = -0.2 - 0.1 * np.sin(phase_r)
        # leg_r6: ankle roll
        joint_pos[i, 11] = 0.0
        
        # 腰部
        joint_pos[i, 12] = 0.05 * np.sin(phase)  # 轻微扭转
        
        # 左臂（与左腿相位相反，自然摆臂）
        joint_pos[i, 13] = 0.3 + 0.3 * np.sin(phase_r)  # shoulder pitch
        joint_pos[i, 14] = 0.1  # shoulder roll
        joint_pos[i, 15] = 0.0  # shoulder yaw
        joint_pos[i, 16] = 0.5  # elbow
        
        # 右臂（与右腿相位相反）
        joint_pos[i, 17] = 0.3 + 0.3 * np.sin(phase)  # shoulder pitch
        joint_pos[i, 18] = -0.1  # shoulder roll
        joint_pos[i, 19] = 0.0  # shoulder yaw
        joint_pos[i, 20] = 0.5  # elbow
    
    # 写入文件
    output_path = Path(output_path)
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
        
        # Joint names offset (placeholder)
        joint_names_offset_pos = f.tell()
        f.write(struct.pack('<I', 0))
        
        # Frame data
        for i in range(num_frames):
            # root_pos: 3 floats
            f.write(struct.pack('<3f', *root_pos[i]))
            # root_quat_wxyz: 4 floats
            f.write(struct.pack('<4f', *root_quat_wxyz[i]))
            # joint_pos: num_joints floats
            f.write(struct.pack(f'<{num_joints}f', *joint_pos[i]))
        
        # Joint names
        joint_names_offset = f.tell()
        for name in G1_URDF_JOINT_NAMES:
            f.write(name.encode('utf-8') + b'\x00')
        
        # Go back and write the offset
        f.seek(joint_names_offset_pos)
        f.write(struct.pack('<I', joint_names_offset))
    
    file_size = output_path.stat().st_size
    print(f"Generated {output_path}")
    print(f"  FPS: {fps}")
    print(f"  Frames: {num_frames}")
    print(f"  Duration: {duration}s")
    print(f"  Joints: {num_joints}")
    print(f"  File size: {file_size} bytes")


def generate_stand_motion(fps=50, duration=3.0, output_path='stand.motion'):
    """生成一个简单的站立动作（呼吸动画）"""
    num_frames = int(fps * duration)
    num_joints = len(G1_URDF_JOINT_NAMES)
    
    # 初始化数据
    root_pos = np.zeros((num_frames, 3), dtype=np.float32)
    root_quat_wxyz = np.zeros((num_frames, 4), dtype=np.float32)
    root_quat_wxyz[:, 0] = 1.0
    joint_pos = np.zeros((num_frames, num_joints), dtype=np.float32)
    
    for i in range(num_frames):
        t = i / fps
        breath = np.sin(2 * np.pi * 0.3 * t)  # 呼吸频率 0.3 Hz
        
        # 根位置
        root_pos[i, 2] = 0.75 + 0.005 * breath
        
        # 基础站立姿态
        # 膝盖微弯
        joint_pos[i, 3] = 0.3  # left knee
        joint_pos[i, 9] = 0.3  # right knee
        
        # hip pitch
        joint_pos[i, 0] = -0.15
        joint_pos[i, 6] = -0.15
        
        # ankle pitch
        joint_pos[i, 4] = -0.15
        joint_pos[i, 10] = -0.15
        
        # 手臂自然下垂
        joint_pos[i, 13] = 0.2 + 0.05 * breath  # left shoulder pitch
        joint_pos[i, 14] = 0.1   # left shoulder roll
        joint_pos[i, 16] = 0.3   # left elbow
        
        joint_pos[i, 17] = 0.2 + 0.05 * breath  # right shoulder pitch
        joint_pos[i, 18] = -0.1  # right shoulder roll
        joint_pos[i, 20] = 0.3   # right elbow
    
    # 写入文件
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    
    with open(output_path, 'wb') as f:
        f.write(b'MFMT')
        f.write(struct.pack('<I', 1))
        f.write(struct.pack('<f', float(fps)))
        f.write(struct.pack('<I', num_frames))
        f.write(struct.pack('<I', num_joints))
        
        joint_names_offset_pos = f.tell()
        f.write(struct.pack('<I', 0))
        
        for i in range(num_frames):
            f.write(struct.pack('<3f', *root_pos[i]))
            f.write(struct.pack('<4f', *root_quat_wxyz[i]))
            f.write(struct.pack(f'<{num_joints}f', *joint_pos[i]))
        
        joint_names_offset = f.tell()
        for name in G1_URDF_JOINT_NAMES:
            f.write(name.encode('utf-8') + b'\x00')
        
        f.seek(joint_names_offset_pos)
        f.write(struct.pack('<I', joint_names_offset))
    
    file_size = output_path.stat().st_size
    print(f"Generated {output_path}")
    print(f"  FPS: {fps}")
    print(f"  Frames: {num_frames}")
    print(f"  Duration: {duration}s")
    print(f"  Joints: {num_joints}")
    print(f"  File size: {file_size} bytes")


if __name__ == '__main__':
    import argparse
    
    parser = argparse.ArgumentParser(description='Generate test motion files')
    parser.add_argument('--output-dir', default='assets/motions', help='Output directory')
    args = parser.parse_args()
    
    output_dir = Path(args.output_dir)
    
    generate_walk_motion(output_path=output_dir / 'walk.motion')
    generate_stand_motion(output_path=output_dir / 'stand.motion')
    
    print(f"\nTest motion files generated in {output_dir}/")
    print("You can now run the viewer to test motion playback.")
