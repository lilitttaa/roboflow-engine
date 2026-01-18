#!/usr/bin/env python3
"""
测试流式接收 - 验证 MotionFlow 和 gentle-humanoid 之间的通信
"""

import sys
sys.path.insert(0, 'src')

from common.utils import MotionStreamServer
import time

def main():
    print("=" * 60)
    print("Stream Test - Waiting for MotionFlow connection...")
    print("=" * 60)
    print()
    print("请在 MotionFlow 中:")
    print("  1. 点击 'Connect' 连接")
    print("  2. 勾选 'Enable Streaming'")
    print("  3. 播放动画")
    print()
    
    server = MotionStreamServer("127.0.0.1", 28563)
    server.start()
    
    last_frame_count = 0
    
    try:
        while True:
            stats = server.get_stats()
            frame = server.get_latest_frame()
            
            if stats['frames_received'] > last_frame_count:
                last_frame_count = stats['frames_received']
                
                if frame:
                    print(f"\n[Frame {stats['frames_received']}]")
                    print(f"  Timestamp: {frame['timestamp']:.3f}s")
                    print(f"  Joint[0-5]: {frame['joint_pos'][:6]}")
                    print(f"  Joint[6-11]: {frame['joint_pos'][6:12]}")
                    # 检查是否有任何非零值
                    non_zero = frame['joint_pos'][frame['joint_pos'] != 0]
                    print(f"  Non-zero joints: {len(non_zero)} values")
                    if len(non_zero) > 0:
                        print(f"  Non-zero values: {non_zero[:5]}...")
                    print(f"  Root Quat: {frame['root_quat_w']}")
                    print(f"  Root Pos: {frame['root_pos_w']}")
            
            if not server.is_connected():
                print(".", end="", flush=True)
            
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("\n\nStopping...")
    finally:
        server.stop()

if __name__ == "__main__":
    main()
