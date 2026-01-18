import socket
import struct
import threading
import time
from collections import deque
from typing import Dict, List, Optional, Tuple

import numpy as np
import linuxfd
import select


class DictToClass:
    def __init__(self, data_dict):
        for key, value in data_dict.items():
            setattr(self, key, value)


class Timer(object):
    '''Timer class for accurate loop rate control
    This class does not use Python's built-in thread timing control
    or management. Only use this class on Linux platforms.
    '''
    def __init__(self, interval: float) -> None:
        self.__epl, self.__tfd = self.__create_timerfd(interval)

    @staticmethod
    def __create_timerfd(interval: float):
        '''Produces a timerfd file descriptor from the kernel
        '''
        tfd = linuxfd.timerfd(rtc=True, nonBlocking=True)
        tfd.settime(interval, interval)
        epl = select.epoll()
        epl.register(tfd.fileno(), select.EPOLLIN)
        return epl, tfd

    def sleep(self) -> None:
        '''Blocks the thread holding this func until the next time point
        '''
        events = self.__epl.poll(-1)
        for fd, event in events:
            if fd == self.__tfd.fileno() and event & select.EPOLLIN:
                self.__tfd.read()

# =========================================
# Tiny non-blocking UDP command server
# =========================================
class MotionUDPServer(threading.Thread):
    """Very small UDP server; each datagram is a motion name string."""
    def __init__(self, host: str = "127.0.0.1", port: int = 28562):
        super().__init__(daemon=True)
        self._host = host
        self._port = port
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind((self._host, self._port))
        self._sock.settimeout(0.2)
        self._q = deque()
        self._lock = threading.Lock()
        self._running = True
        print(f"[MotionUDPServer] Listening on udp://{host}:{port}")

    def run(self):
        while self._running:
            try:
                data, _ = self._sock.recvfrom(1024)
                name = data.decode("utf-8", errors="ignore").strip()
                if name:
                    with self._lock:
                        self._q.append(name)
            except socket.timeout:
                continue
            except Exception as e:
                print(f"[MotionUDPServer] Error: {e}")

    def stop(self):
        self._running = False
        try:
            self._sock.close()
        except Exception:
            pass

    def pop_all(self) -> List[str]:
        with self._lock:
            items = list(self._q)
            self._q.clear()
        return items


# =========================================
# Motion Stream Server for real-time data from MotionFlow
# =========================================
class MotionStreamServer(threading.Thread):
    """
    UDP server that receives real-time motion data from MotionFlow engine.
    
    Packet format (148 bytes):
    - Magic: 4 bytes (0x4D465354 = "MFST")
    - Timestamp: 8 bytes (double)
    - Joint positions: 116 bytes (29 * float32)
    - Root quaternion: 16 bytes (4 * float32, wxyz)
    - Root position: 12 bytes (3 * float32)
    """
    MAGIC = 0x4D465354  # "MFST"
    PACKET_SIZE = 148
    
    def __init__(self, host: str = "127.0.0.1", port: int = 28563):
        super().__init__(daemon=True)
        self._host = host
        self._port = port
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((self._host, self._port))
        self._sock.settimeout(0.1)
        
        self._lock = threading.Lock()
        self._running = True
        self._connected = False
        self._last_receive_time = 0.0
        self._frames_received = 0
        
        # Latest frame data
        self._latest_frame: Optional[Dict] = None
        
        # Frame buffer for smoothing (optional)
        self._frame_buffer: deque = deque(maxlen=10)
        
        print(f"[MotionStreamServer] Listening on udp://{host}:{port}")
    
    def run(self):
        while self._running:
            try:
                data, addr = self._sock.recvfrom(256)
                if len(data) >= self.PACKET_SIZE:
                    frame = self._parse_packet(data)
                    if frame is not None:
                        with self._lock:
                            self._latest_frame = frame
                            self._frame_buffer.append(frame)
                            self._last_receive_time = time.time()
                            self._frames_received += 1
                            if not self._connected:
                                self._connected = True
                                print(f"[MotionStreamServer] Connected from {addr}")
            except socket.timeout:
                # Check for disconnect (no data for 5 seconds)
                if self._connected and time.time() - self._last_receive_time > 5.0:
                    self._connected = False
                    print(f"[MotionStreamServer] Disconnected (timeout)")
                continue
            except Exception as e:
                print(f"[MotionStreamServer] Error: {e}")
    
    def _parse_packet(self, data: bytes) -> Optional[Dict]:
        """Parse a UDP packet into a frame dictionary."""
        try:
            offset = 0
            
            # Magic (4 bytes, uint32)
            magic = struct.unpack('<I', data[offset:offset+4])[0]
            offset += 4
            if magic != self.MAGIC:
                return None
            
            # Timestamp (8 bytes, double)
            timestamp = struct.unpack('<d', data[offset:offset+8])[0]
            offset += 8
            
            # Joint positions (29 * 4 = 116 bytes)
            joint_pos = np.array(struct.unpack('<29f', data[offset:offset+116]), dtype=np.float32)
            offset += 116
            
            # Root quaternion (4 * 4 = 16 bytes, wxyz)
            root_quat = np.array(struct.unpack('<4f', data[offset:offset+16]), dtype=np.float32)
            offset += 16
            
            # Root position (3 * 4 = 12 bytes)
            root_pos = np.array(struct.unpack('<3f', data[offset:offset+12]), dtype=np.float32)
            
            return {
                'timestamp': timestamp,
                'joint_pos': joint_pos,
                'root_quat_w': root_quat,  # wxyz format
                'root_pos_w': root_pos,
            }
        except Exception as e:
            print(f"[MotionStreamServer] Parse error: {e}")
            return None
    
    def get_latest_frame(self) -> Optional[Dict]:
        """Get the latest received frame."""
        with self._lock:
            return self._latest_frame.copy() if self._latest_frame else None
    
    def get_buffered_frames(self, count: int = 5) -> List[Dict]:
        """Get recent frames from buffer."""
        with self._lock:
            frames = list(self._frame_buffer)
            return frames[-count:] if len(frames) >= count else frames
    
    def is_connected(self) -> bool:
        """Check if we're receiving data."""
        return self._connected
    
    def get_stats(self) -> Dict:
        """Get server statistics."""
        return {
            'connected': self._connected,
            'frames_received': self._frames_received,
            'last_receive_time': self._last_receive_time,
        }
    
    def stop(self):
        self._running = False
        try:
            self._sock.close()
        except Exception:
            pass
        print(f"[MotionStreamServer] Stopped. Total frames received: {self._frames_received}")


joint_names_29 = ["left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint", "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint", "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint", "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint", "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint", "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint", "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint", "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint", "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint"]

joint_names_23 = ["left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint", "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint", "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint", "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint", "waist_yaw_joint", "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint", "left_elbow_joint", "left_wrist_roll_joint", "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint", "right_elbow_joint", "right_wrist_roll_joint"]

body_names_29 = ["pelvis", "left_hip_pitch_link", "left_hip_roll_link", "left_hip_yaw_link", "left_knee_link", "left_ankle_pitch_link", "left_ankle_roll_link", "right_hip_pitch_link", "right_hip_roll_link", "right_hip_yaw_link", "right_knee_link", "right_ankle_pitch_link", "right_ankle_roll_link", "torso_link", "left_shoulder_pitch_link", "left_shoulder_roll_link", "left_shoulder_yaw_link", "left_elbow_link", "left_wrist_roll_link", "right_shoulder_pitch_link", "right_shoulder_roll_link", "right_shoulder_yaw_link", "right_elbow_link", "right_wrist_roll_link", "head_mimic", "left_hand_mimic", "right_hand_mimic"]

body_names_23 = ["world", "pelvis", "left_hip_pitch_link", "left_hip_roll_link", "left_hip_yaw_link", "left_knee_link", "left_ankle_pitch_link", "left_ankle_roll_link", "right_hip_pitch_link", "right_hip_roll_link", "right_hip_yaw_link", "right_knee_link", "right_ankle_pitch_link", "right_ankle_roll_link", "torso_link", "left_shoulder_pitch_link", "left_shoulder_roll_link", "left_shoulder_yaw_link", "left_elbow_link", "left_wrist_roll_rubber_hand", "right_shoulder_pitch_link", "right_shoulder_roll_link", "right_shoulder_yaw_link", "right_elbow_link", "right_wrist_roll_rubber_hand", "head_mimic", "left_hand_mimic", "right_hand_mimic"]
