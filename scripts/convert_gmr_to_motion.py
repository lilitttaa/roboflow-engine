#!/usr/bin/env python3
"""
Convert GMR pickle format to .motion binary format for MotionFlow.

GMR pickle format:
- fps: int
- root_pos: (N, 3) array - root position (x, y, z)
- root_rot: (N, 4) array - root rotation quaternion (xyzw)
- dof_pos: (N, 29) array - joint positions

.motion binary format (MFMT):
- Magic: "MFMT" (4 bytes)
- Version: uint32 (4 bytes)
- FPS: float (4 bytes)
- NumFrames: uint32 (4 bytes)
- NumJoints: uint32 (4 bytes)
- JointNamesOffset: uint32 (4 bytes)
- Frames: NumFrames x Frame
  - Frame:
    - rootPos: 3 floats (x, y, z)
    - rootQuat: 4 floats (w, x, y, z)  <- wxyz order!
    - jointPositions: NumJoints floats
- JointNames: null-terminated strings
"""

import argparse
import pickle
import struct
import numpy as np
from pathlib import Path

# G1 joint names (29 DOF, excluding the pelvis root)
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


def convert_gmr_to_motion(input_path: Path, output_path: Path, max_frames: int = None):
    """Convert GMR pickle to .motion format."""
    
    # Load GMR pickle
    with open(input_path, 'rb') as f:
        data = pickle.load(f)
    
    fps = data['fps']
    root_pos = data['root_pos']  # (N, 3)
    root_rot = data['root_rot']  # (N, 4) - xyzw quaternion from GMR
    dof_pos = data['dof_pos']    # (N, 29)
    
    num_frames = len(root_pos)
    if max_frames and max_frames < num_frames:
        num_frames = max_frames
    
    num_joints = len(G1_JOINT_NAMES)
    
    print(f"Converting {input_path.name}:")
    print(f"  FPS: {fps}")
    print(f"  Frames: {num_frames}")
    print(f"  Joints: {num_joints}")
    
    # Calculate header size and joint names offset
    header_size = 4 + 4 + 4 + 4 + 4 + 4  # magic + version + fps + frames + joints + offset
    frame_size = 3 * 4 + 4 * 4 + num_joints * 4  # rootPos + rootQuat + joints
    joint_names_offset = header_size + num_frames * frame_size
    
    # Write .motion file
    with open(output_path, 'wb') as f:
        # Header
        f.write(b'MFMT')                                    # Magic
        f.write(struct.pack('I', 1))                        # Version
        f.write(struct.pack('f', float(fps)))               # FPS
        f.write(struct.pack('I', num_frames))               # NumFrames
        f.write(struct.pack('I', num_joints))               # NumJoints
        f.write(struct.pack('I', joint_names_offset))       # JointNamesOffset
        
        # Frames
        for i in range(num_frames):
            # Root position (x, y, z)
            f.write(struct.pack('3f', *root_pos[i]))
            
            # Root quaternion - convert from xyzw to wxyz
            qx, qy, qz, qw = root_rot[i]
            f.write(struct.pack('4f', qw, qx, qy, qz))  # wxyz order
            
            # Joint positions
            f.write(struct.pack(f'{num_joints}f', *dof_pos[i]))
        
        # Joint names (null-terminated strings)
        for name in G1_JOINT_NAMES:
            f.write(name.encode('utf-8'))
            f.write(b'\0')
    
    print(f"  Output: {output_path}")
    print(f"  Size: {output_path.stat().st_size / 1024:.1f} KB")


def main():
    parser = argparse.ArgumentParser(description="Convert GMR pickle to .motion format")
    parser.add_argument("input", type=Path, help="Input GMR pickle file or directory")
    parser.add_argument("-o", "--output", type=Path, help="Output .motion file or directory")
    parser.add_argument("--max-frames", type=int, help="Maximum frames to export (for testing)")
    args = parser.parse_args()
    
    if args.input.is_dir():
        # Convert all .pkl files in directory
        input_dir = args.input
        output_dir = args.output or input_dir
        output_dir.mkdir(parents=True, exist_ok=True)
        
        for pkl_file in input_dir.glob("*.pkl"):
            output_file = output_dir / pkl_file.with_suffix('.motion').name
            convert_gmr_to_motion(pkl_file, output_file, args.max_frames)
    else:
        # Convert single file
        input_file = args.input
        output_file = args.output or input_file.with_suffix('.motion')
        convert_gmr_to_motion(input_file, output_file, args.max_frames)


if __name__ == "__main__":
    main()
