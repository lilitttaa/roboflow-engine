#!/usr/bin/env python3
"""
Sim2Sim GUI - 集成的仿真测试应用
一个窗口同时运行仿真器、控制器和动作选择器
"""
import os
import sys
import time
import yaml
import signal
import socket
import threading
import traceback
from pathlib import Path
from multiprocessing import Value
from typing import Optional, Dict, List, Any

import numpy as np
import mujoco
import mujoco.viewer

from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QListWidget, QListWidgetItem, QGroupBox,
    QSlider, QSpinBox, QTextEdit, QSplitter, QFrame, QStatusBar,
    QProgressBar, QComboBox
)
from PyQt6.QtCore import Qt, QTimer, pyqtSignal, QThread
from PyQt6.QtGui import QFont, QColor, QPalette, QTextCursor

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_ as LowStateHG
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_ as LowCmdHG
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_, unitree_hg_msg_dds__LowState_
from unitree_sdk2py.utils.crc import CRC

from common.utils import DictToClass, Timer
from common.remote_controller import RemoteController, KeyMap
from common.joint_mapper import create_real_to_mujoco_mapper, create_isaac_to_real_mapper
from common.command_helper import create_damping_cmd, create_zero_cmd, init_cmd_hg, MotorMode

from policy import TrackingPolicy
from paths import ASSETS_DIR, REAL_G1_ROOT

np.set_printoptions(formatter={'float': lambda x: "{0:0.2f}".format(x)})


def load_yaml_config(path: str) -> DictToClass:
    """加载 YAML 配置文件"""
    p = Path(path)
    if not p.is_absolute():
        p = REAL_G1_ROOT / p
    with open(str(p), 'r') as f:
        return DictToClass(yaml.load(f, Loader=yaml.FullLoader))


def load_motion_options(yaml_path: str) -> List[str]:
    """从配置加载可用的动作列表"""
    p = Path(yaml_path)
    if not p.is_absolute():
        p = REAL_G1_ROOT / p
    
    if not p.exists():
        return ["default"]
    
    with open(str(p), 'r') as f:
        data = yaml.safe_load(f) or {}
    
    opts = []
    seen = set()
    
    for arr_key in ("motion_clips", "motions"):
        arr = data.get(arr_key, []) or []
        for item in arr:
            name = str(item.get("name", "")).strip()
            if name and name not in seen:
                opts.append(name)
                seen.add(name)
    
    if "default" not in seen:
        opts.insert(0, "default")
    
    return opts


class LogWidget(QTextEdit):
    """自定义日志显示组件"""
    append_signal = pyqtSignal(str, str)
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setReadOnly(True)
        self.setFont(QFont("JetBrains Mono", 10))
        self.setStyleSheet("""
            QTextEdit {
                background-color: #1a1a2e;
                color: #eef0f5;
                border: 1px solid #3a3a5e;
                border-radius: 6px;
                padding: 8px;
            }
        """)
        self.append_signal.connect(self._append_log)
    
    def _append_log(self, msg: str, level: str = "info"):
        colors = {
            "info": "#a0d8ef",
            "warn": "#f9c74f",
            "error": "#f94144",
            "success": "#90be6d"
        }
        color = colors.get(level, "#eef0f5")
        timestamp = time.strftime("%H:%M:%S")
        self.append(f'<span style="color:#666;">[{timestamp}]</span> <span style="color:{color};">{msg}</span>')
        self.moveCursor(QTextCursor.MoveOperation.End)
    
    def log(self, msg: str, level: str = "info"):
        self.append_signal.emit(msg, level)


class SimulatorThread(QThread):
    """MuJoCo 仿真器线程"""
    state_updated = pyqtSignal(dict)
    log_signal = pyqtSignal(str, str)
    
    def __init__(self, config: DictToClass, xml_path: str):
        super().__init__()
        self.config = config
        self.xml_path = xml_path
        self.is_alive = True
        self.is_running = False
        
        self.pub_freq = 200
        self.pub_dt = 1. / self.pub_freq
        self.low_level_freq = 200
        self.low_level_dt = 1. / self.low_level_freq
        
        # MuJoCo 模型
        xml_candidate = Path(xml_path)
        if not xml_candidate.is_absolute():
            under_assets = ASSETS_DIR / xml_candidate
            model_path = os.path.abspath(str(under_assets if under_assets.exists() else xml_candidate))
        else:
            model_path = str(xml_candidate)
        
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.model.opt.timestep = self.low_level_dt
        self.data = mujoco.MjData(self.model)
        
        self.ctrl_lower = 0.8 * self.model.actuator_ctrlrange[:, 0]
        self.ctrl_upper = 0.8 * self.model.actuator_ctrlrange[:, 1]
        
        # 关节映射
        self.real_to_mujoco_mapper = create_real_to_mujoco_mapper(
            self.config.real_joint_names,
            self.config.mujoco_joint_names
        )
        
        # 初始位置
        self.data.qpos[7:] = self.real_to_mujoco_mapper.map_state_to_from(self.config.default_qpos_real)
        self.data.qvel[:] = 0.
        mujoco.mj_forward(self.model, self.data)
        
        # 低级命令
        self.__ptargets_real = np.zeros(len(self.config.real_joint_names))
        self.__kp_real = np.zeros(len(self.config.real_joint_names))
        self.__kd_real = np.zeros(len(self.config.real_joint_names))
        
        self.low_cmd = None
        self.low_state = unitree_hg_msg_dds__LowState_()
        
        # DDS 发布/订阅
        self.state_pub = ChannelPublisher(self.config.lowstate_topic, LowStateHG)
        self.state_pub.Init()
        self.cmd_sub = ChannelSubscriber(self.config.lowcmd_topic, LowCmdHG)
        self.cmd_sub.Init(self.cmd_sub_handler)
        
        self.policy_queried = False
        self.loop_count = Value('i', 0)
        
        # 状态机
        self.stage = "wait"  # wait, gantry, control
        self.viewer = None
        self._viewer_tick = 0
        self.viewer_decim = max(1, self.low_level_freq // 30)
        
    def cmd_sub_handler(self, msg):
        self.low_cmd = msg
        self.policy_queried |= self.low_cmd.reserve[0]
        for i in range(len(self.config.real_joint_names)):
            self.__ptargets_real[i] = msg.motor_cmd[i].q
            self.__kp_real[i] = msg.motor_cmd[i].kp
            self.__kd_real[i] = msg.motor_cmd[i].kd
    
    def set_button(self, idx: int):
        """模拟按键"""
        self.low_state.wireless_remote[0] = idx
        self.state_pub.Write(self.low_state)
        QTimer.singleShot(100, lambda: self._clear_button())
    
    def _clear_button(self):
        self.low_state.wireless_remote[0] = 0
        self.state_pub.Write(self.low_state)
    
    def run(self):
        self.is_running = True
        self.log_signal.emit("仿真器启动...", "info")
        
        # 启动状态发布线程
        state_thread = threading.Thread(target=self._state_pub_handler, daemon=True)
        state_thread.start()
        
        # 启动 MuJoCo viewer
        try:
            with mujoco.viewer.launch_passive(
                self.model,
                self.data,
                show_left_ui=False,
                show_right_ui=False,
            ) as viewer:
                self.viewer = viewer
                self.log_signal.emit("MuJoCo 可视化窗口已打开", "success")
                self.log_signal.emit("等待高级控制器连接...", "info")
                
                # 等待控制器连接
                while self.is_alive and self.low_cmd is None:
                    if not viewer.is_running():
                        self.is_alive = False
                        break
                    time.sleep(0.01)
                
                if self.low_cmd is not None:
                    self.log_signal.emit("高级控制器已连接", "success")
                    self.log_signal.emit("点击 '移动到默认姿态' 按钮", "info")
                
                self.stage = "wait"
                
                # 主循环
                timer = Timer(self.low_level_dt)
                while self.is_alive and viewer.is_running():
                    if self.stage == "gantry":
                        self._simulate_gantry_step()
                    elif self.stage == "control":
                        self._simulate_control_step()
                    
                    self._viewer_sync()
                    timer.sleep()
                
        except Exception as e:
            self.log_signal.emit(f"仿真器错误: {e}", "error")
            traceback.print_exc()
        finally:
            self.is_running = False
            self.log_signal.emit("仿真器已停止", "warn")
    
    def _state_pub_handler(self):
        timer = Timer(self.pub_dt)
        while self.is_alive:
            low_state = self.low_state
            joint_qpos_mujoco = self.data.qpos[7:]
            joint_qvel_mujoco = self.data.qvel[6:]
            joint_torque_mujoco = self.data.ctrl.copy()
            
            joint_qpos_real = self.real_to_mujoco_mapper.map_state_to_from(joint_qpos_mujoco)
            joint_qvel_real = self.real_to_mujoco_mapper.map_state_to_from(joint_qvel_mujoco)
            joint_torque_real = self.real_to_mujoco_mapper.map_state_to_from(joint_torque_mujoco)
            
            for i in range(len(self.config.real_joint_names)):
                low_state.motor_state[i].q = joint_qpos_real[i]
                low_state.motor_state[i].dq = joint_qvel_real[i]
                low_state.motor_state[i].tau_est = joint_torque_real[i]
            
            low_state.imu_state.quaternion = self.data.qpos[3:7].copy()
            low_state.imu_state.gyroscope = self.data.qvel[3:6].copy()
            low_state.tick = 1
            low_state.crc = CRC().Crc(low_state)
            self.state_pub.Write(low_state)
            
            # 发送状态更新
            self.state_updated.emit({
                'height': float(self.data.qpos[2]),
                'loop_count': self.loop_count.value,
                'stage': self.stage
            })
            
            timer.sleep()
    
    def _simulate_gantry_step(self):
        ptargets_mujoco = self.real_to_mujoco_mapper.map_action_from_to(self.__ptargets_real)
        self.data.qpos[:7] = [0, 0, 2, 1.0, 0.0, 0.0, 0.0]
        self.data.qpos[7:] = ptargets_mujoco
        mujoco.mj_forward(self.model, self.data)
    
    def _simulate_control_step(self):
        if not self.policy_queried:
            return
        
        ptargets_mujoco = self.real_to_mujoco_mapper.map_action_from_to(self.__ptargets_real)
        kp_mujoco = self.real_to_mujoco_mapper.map_action_from_to(self.__kp_real)
        kd_mujoco = self.real_to_mujoco_mapper.map_action_from_to(self.__kd_real)
        
        qpos_mujoco = self.data.qpos[7:]
        qvel_mujoco = self.data.qvel[6:]
        ctrl = kp_mujoco * (ptargets_mujoco - qpos_mujoco) + kd_mujoco * (0 - qvel_mujoco)
        ctrl = np.clip(ctrl, self.ctrl_lower, self.ctrl_upper)
        self.data.ctrl[:] = ctrl
        
        self._limit_external_forces(max_force=30.0)
        mujoco.mj_step(self.model, self.data)
        self.loop_count.value += 1
    
    def _limit_external_forces(self, max_force=30.0):
        for i in range(self.model.nbody):
            force = self.data.xfrc_applied[i, :3]
            force_magnitude = np.linalg.norm(force)
            if force_magnitude > max_force:
                self.data.xfrc_applied[i, :3] = force * (max_force / force_magnitude)
    
    def _viewer_sync(self):
        if self.viewer is None:
            return
        self._viewer_tick += 1
        if (self._viewer_tick % self.viewer_decim) == 0:
            self.viewer.sync()
    
    def start_gantry(self):
        self.stage = "gantry"
        self.log_signal.emit("开始移动到默认姿态...", "info")
    
    def start_control(self):
        self.data.qpos[2] = 0.78
        mujoco.mj_forward(self.model, self.data)
        self.stage = "control"
        self.loop_count.value = 0
        self.log_signal.emit("开始运行控制循环", "success")
    
    def stop(self):
        self.is_alive = False
        self.set_button(KeyMap.select)


class ControllerThread(QThread):
    """高级控制器线程"""
    log_signal = pyqtSignal(str, str)
    status_signal = pyqtSignal(str)
    
    def __init__(self, config: DictToClass):
        super().__init__()
        self.config = config
        self.is_alive = True
        self.is_running = False
        
        self.remote_controller = RemoteController()
        self.control_dt = 1.0 / self.config.control_freq
        
        self.isaac_to_real_mapper_state = create_isaac_to_real_mapper(
            self.config.isaac_joint_names_state,
            self.config.real_joint_names
        )
        
        self._state_lock = threading.Lock()
        self.dof_size_real = len(self.config.real_joint_names)
        
        self.smoothing_alpha = getattr(self.config, "lowstate_alpha", 0.2)
        self._qj_smooth = np.zeros(self.dof_size_real, dtype=np.float32)
        self._dqj_smooth = np.zeros(self.dof_size_real, dtype=np.float32)
        self._tau_smooth = np.zeros(self.dof_size_real, dtype=np.float32)
        self._quat_smooth = np.zeros(4, dtype=np.float32)
        self._gyro_smooth = np.zeros(3, dtype=np.float32)
        
        self.qj_real = np.zeros(self.dof_size_real, dtype=np.float32)
        self.dqj_real = np.zeros(self.dof_size_real, dtype=np.float32)
        self.tau_real = np.zeros(self.dof_size_real, dtype=np.float32)
        self.quat = np.zeros(4, dtype=np.float32)
        self.gyro = np.zeros(3, dtype=np.float32)
        
        self.qj_isaac = None
        self.dqj_isaac = None
        self.tau_isaac = None
        
        self.default_qpos_real = np.array(self.config.default_qpos_real, dtype=np.float32)
        self.init_qpos_real = np.array(self.config.init_qpos_real, dtype=np.float32)
        self.kps_real = np.array(self.config.kps_real, dtype=np.float32)
        self.kds_real = np.array(self.config.kds_real, dtype=np.float32)
        
        self._prev_buttons = None
        self.btn_rise = None
        self.btn_fall = None
        
        # DDS
        self.low_cmd = unitree_hg_msg_dds__LowCmd_()
        self.low_state = unitree_hg_msg_dds__LowState_()
        self.mode_pr_ = MotorMode.PR
        self.mode_machine_ = 0
        
        self.lowcmd_publisher_ = ChannelPublisher(self.config.lowcmd_topic, LowCmdHG)
        self.lowcmd_publisher_.Init()
        
        self.lowstate_subscriber = ChannelSubscriber(self.config.lowstate_topic, LowStateHG)
        self.lowstate_subscriber.Init(self.LowStateHgHandler, 0)
        
        # 策略
        self.tracking_cfg = load_yaml_config("config/tracking.yaml")
        self.tracking_cfg.udp_enable = False  # 禁用 UDP，我们直接调用
        self.current_policy: Optional[TrackingPolicy] = None
        
        self.stage = "wait"  # wait, zero_torque, default_pose, running
        self._last_target_qpos_real = self.init_qpos_real.copy()
    
    def LowStateHgHandler(self, msg: LowStateHG):
        a = self.smoothing_alpha
        with self._state_lock:
            self.low_state = msg
            
            q = [msg.motor_state[i].q for i in range(self.dof_size_real)]
            dq = [msg.motor_state[i].dq for i in range(self.dof_size_real)]
            tau = [msg.motor_state[i].tau_est for i in range(self.dof_size_real)]
            
            self._qj_smooth[:] = (1 - a) * self._qj_smooth[:] + a * np.array(q, dtype=np.float32)
            self._dqj_smooth[:] = (1 - a) * self._dqj_smooth[:] + a * np.array(dq, dtype=np.float32)
            self._tau_smooth[:] = (1 - a) * self._tau_smooth[:] + a * np.array(tau, dtype=np.float32)
            self._quat_smooth[:] = (1 - a) * self._quat_smooth + a * np.array(msg.imu_state.quaternion, dtype=np.float32)
            self._gyro_smooth[:] = (1 - a) * self._gyro_smooth + a * np.array(msg.imu_state.gyroscope, dtype=np.float32)
            
            self.remote_controller.set_sim2sim(msg.wireless_remote)
            self.mode_machine_ = msg.mode_machine
    
    def send_cmd(self, cmd):
        cmd.crc = CRC().Crc(cmd)
        self.lowcmd_publisher_.Write(cmd)
    
    def process_state(self):
        with self._state_lock:
            self.qj_real[:] = self._qj_smooth
            self.dqj_real[:] = self._dqj_smooth
            self.tau_real[:] = self._tau_smooth
            self.quat[:] = self._quat_smooth
            self.gyro[:] = self._gyro_smooth
            
            now = np.array(self.remote_controller.button, dtype=np.int8)
            if self._prev_buttons is None or len(self._prev_buttons) != len(now):
                self._prev_buttons = now.copy()
                self.btn_rise = np.zeros_like(now, dtype=bool)
                self.btn_fall = np.zeros_like(now, dtype=bool)
            else:
                self.btn_rise = (self._prev_buttons == 0) & (now == 1)
                self.btn_fall = (self._prev_buttons == 1) & (now == 0)
                self._prev_buttons = now
        
        self.qj_isaac = self.isaac_to_real_mapper_state.map_state_to_from(self.qj_real)
        self.dqj_isaac = self.isaac_to_real_mapper_state.map_state_to_from(self.dqj_real)
        self.tau_isaac = self.isaac_to_real_mapper_state.map_state_to_from(self.tau_real)
    
    def run(self):
        self.is_running = True
        self.log_signal.emit("控制器启动...", "info")
        
        # 等待低级状态
        self.log_signal.emit("等待仿真器状态...", "info")
        while self.is_alive and self.low_state.tick == 0:
            time.sleep(self.control_dt)
        
        if not self.is_alive:
            return
        
        self.log_signal.emit("已连接到仿真器", "success")
        init_cmd_hg(self.low_cmd, self.mode_machine_, self.mode_pr_)
        
        timer = Timer(self.control_dt)
        
        try:
            while self.is_alive:
                self.process_state()
                
                if self.btn_rise[KeyMap.select]:
                    self.log_signal.emit("收到退出信号", "warn")
                    break
                
                if self.stage == "zero_torque":
                    create_zero_cmd(self.low_cmd)
                    self.send_cmd(self.low_cmd)
                    
                    if self.btn_rise[KeyMap.start]:
                        self.stage = "move_to_default"
                        self._move_to_default_start_time = time.time()
                        self._move_to_default_init_pos = np.array([
                            self.low_state.motor_state[i].q for i in range(self.dof_size_real)
                        ], dtype=np.float32)
                        self.log_signal.emit("开始移动到默认姿态...", "info")
                
                elif self.stage == "move_to_default":
                    total_time = 2.0
                    elapsed = time.time() - self._move_to_default_start_time
                    alpha = min(elapsed / total_time, 1.0)
                    
                    for i in range(self.dof_size_real):
                        target_pos = self.init_qpos_real[i]
                        self.low_cmd.motor_cmd[i].q = self._move_to_default_init_pos[i] * (1 - alpha) + target_pos * alpha
                        self.low_cmd.motor_cmd[i].qd = 0
                        self.low_cmd.motor_cmd[i].kp = self.kps_real[i]
                        self.low_cmd.motor_cmd[i].kd = self.kds_real[i]
                        self.low_cmd.motor_cmd[i].tau = 0
                    self.send_cmd(self.low_cmd)
                    
                    if alpha >= 1.0:
                        self.stage = "default_pose"
                        self._last_target_qpos_real[:] = self.init_qpos_real[:]
                        self.log_signal.emit("已到达默认姿态，等待启动策略...", "success")
                        self.status_signal.emit("default_pose")
                
                elif self.stage == "default_pose":
                    for i in range(self.dof_size_real):
                        self.low_cmd.motor_cmd[i].q = self.init_qpos_real[i]
                        self.low_cmd.motor_cmd[i].qd = 0
                        self.low_cmd.motor_cmd[i].kp = self.kps_real[i]
                        self.low_cmd.motor_cmd[i].kd = self.kds_real[i]
                        self.low_cmd.motor_cmd[i].tau = 0
                    self.send_cmd(self.low_cmd)
                    
                    if self.btn_rise[KeyMap.A]:
                        self._init_policy()
                        self.stage = "running"
                        self.low_cmd.reserve[0] = 1
                        self.send_cmd(self.low_cmd)
                        self.log_signal.emit("策略已启动", "success")
                        self.status_signal.emit("running")
                
                elif self.stage == "running":
                    if self.current_policy is not None:
                        self.current_policy.update_obs()
                        action_real = self.current_policy.compute_action()
                        self._apply_action_real(action_real)
                    self.send_cmd(self.low_cmd)
                
                timer.sleep()
                
        except Exception as e:
            self.log_signal.emit(f"控制器错误: {e}", "error")
            traceback.print_exc()
        finally:
            create_damping_cmd(self.low_cmd)
            self.send_cmd(self.low_cmd)
            self.is_running = False
            self.log_signal.emit("控制器已停止", "warn")
    
    def _init_policy(self):
        self.current_policy = TrackingPolicy("tracking", self.tracking_cfg, self)
        self.smoothing_alpha = self.current_policy.lowstate_alpha
        if hasattr(self.current_policy, "kps_real") and hasattr(self.current_policy, "kds_real"):
            self.kps_real[:] = self.current_policy.kps_real
            self.kds_real[:] = self.current_policy.kds_real
        self.current_policy.fade_in()
    
    def _apply_action_real(self, action_real_delta: np.ndarray):
        if action_real_delta is None or not np.all(np.isfinite(action_real_delta)):
            return
        
        target = self.default_qpos_real + action_real_delta
        for i in range(self.dof_size_real):
            self.low_cmd.motor_cmd[i].q = float(target[i])
            self.low_cmd.motor_cmd[i].qd = 0.0
            self.low_cmd.motor_cmd[i].kp = float(self.kps_real[i])
            self.low_cmd.motor_cmd[i].kd = float(self.kds_real[i])
            self.low_cmd.motor_cmd[i].tau = 0.0
    
    def request_motion(self, name: str):
        if self.current_policy is not None:
            self.current_policy.request_motion(name)
            self.log_signal.emit(f"请求动作: {name}", "info")
    
    def start_zero_torque(self):
        self.stage = "zero_torque"
        self.status_signal.emit("zero_torque")
        self.log_signal.emit("进入零力矩状态", "info")
    
    def stop(self):
        self.is_alive = False


class Sim2SimGUI(QMainWindow):
    """主 GUI 窗口"""
    
    def __init__(self):
        super().__init__()
        self.setWindowTitle("GentleHumanoid Sim2Sim GUI")
        self.setMinimumSize(900, 700)
        
        # 加载配置
        self.ctrl_config = load_yaml_config("config/controller.yaml")
        self.motion_options = load_motion_options("config/tracking.yaml")
        
        # 初始化 DDS
        ChannelFactoryInitialize(0, 'lo')
        
        # 线程
        self.simulator_thread: Optional[SimulatorThread] = None
        self.controller_thread: Optional[ControllerThread] = None
        
        self._setup_ui()
        self._setup_style()
        
        # 状态更新定时器
        self.status_timer = QTimer()
        self.status_timer.timeout.connect(self._update_status)
        self.status_timer.start(100)
    
    def _setup_style(self):
        self.setStyleSheet("""
            QMainWindow {
                background-color: #16213e;
            }
            QGroupBox {
                background-color: #1a1a2e;
                border: 2px solid #3a3a5e;
                border-radius: 8px;
                margin-top: 12px;
                padding-top: 10px;
                font-weight: bold;
                color: #e2e2e2;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 12px;
                padding: 0 8px;
                background-color: #1a1a2e;
            }
            QPushButton {
                background-color: #0f3460;
                color: #eef0f5;
                border: none;
                border-radius: 6px;
                padding: 10px 20px;
                font-weight: bold;
                font-size: 13px;
            }
            QPushButton:hover {
                background-color: #1a508b;
            }
            QPushButton:pressed {
                background-color: #0d2847;
            }
            QPushButton:disabled {
                background-color: #2a2a3e;
                color: #666;
            }
            QPushButton#startBtn {
                background-color: #2d6a4f;
            }
            QPushButton#startBtn:hover {
                background-color: #40916c;
            }
            QPushButton#stopBtn {
                background-color: #9d0208;
            }
            QPushButton#stopBtn:hover {
                background-color: #d00000;
            }
            QPushButton#actionBtn {
                background-color: #7b2cbf;
            }
            QPushButton#actionBtn:hover {
                background-color: #9d4edd;
            }
            QListWidget {
                background-color: #1a1a2e;
                border: 1px solid #3a3a5e;
                border-radius: 6px;
                color: #eef0f5;
                padding: 4px;
            }
            QListWidget::item {
                padding: 8px;
                border-radius: 4px;
            }
            QListWidget::item:selected {
                background-color: #0f3460;
            }
            QListWidget::item:hover {
                background-color: #1a508b;
            }
            QLabel {
                color: #b8c5d6;
            }
            QLabel#statusLabel {
                color: #90be6d;
                font-weight: bold;
                font-size: 14px;
            }
            QStatusBar {
                background-color: #0f0f23;
                color: #b8c5d6;
            }
        """)
    
    def _setup_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QHBoxLayout(central)
        main_layout.setSpacing(12)
        main_layout.setContentsMargins(12, 12, 12, 12)
        
        # 左侧：控制面板
        left_panel = QWidget()
        left_layout = QVBoxLayout(left_panel)
        left_layout.setSpacing(12)
        
        # 系统控制
        sys_group = QGroupBox("系统控制")
        sys_layout = QVBoxLayout(sys_group)
        
        self.start_btn = QPushButton("🚀 启动系统")
        self.start_btn.setObjectName("startBtn")
        self.start_btn.clicked.connect(self._start_system)
        
        self.stop_btn = QPushButton("⏹ 停止系统")
        self.stop_btn.setObjectName("stopBtn")
        self.stop_btn.setEnabled(False)
        self.stop_btn.clicked.connect(self._stop_system)
        
        sys_layout.addWidget(self.start_btn)
        sys_layout.addWidget(self.stop_btn)
        left_layout.addWidget(sys_group)
        
        # 状态机控制
        state_group = QGroupBox("状态机控制")
        state_layout = QVBoxLayout(state_group)
        
        self.zero_torque_btn = QPushButton("零力矩状态 (等待)")
        self.zero_torque_btn.setEnabled(False)
        self.zero_torque_btn.clicked.connect(self._zero_torque)
        
        self.default_pose_btn = QPushButton("移动到默认姿态 [S]")
        self.default_pose_btn.setEnabled(False)
        self.default_pose_btn.clicked.connect(self._move_to_default)
        
        self.start_policy_btn = QPushButton("启动策略 [A]")
        self.start_policy_btn.setObjectName("actionBtn")
        self.start_policy_btn.setEnabled(False)
        self.start_policy_btn.clicked.connect(self._start_policy)
        
        state_layout.addWidget(self.zero_torque_btn)
        state_layout.addWidget(self.default_pose_btn)
        state_layout.addWidget(self.start_policy_btn)
        left_layout.addWidget(state_group)
        
        # 状态显示
        status_group = QGroupBox("系统状态")
        status_layout = QVBoxLayout(status_group)
        
        self.status_label = QLabel("状态: 未启动")
        self.status_label.setObjectName("statusLabel")
        
        self.height_label = QLabel("高度: --")
        self.loop_label = QLabel("循环计数: --")
        
        status_layout.addWidget(self.status_label)
        status_layout.addWidget(self.height_label)
        status_layout.addWidget(self.loop_label)
        left_layout.addWidget(status_group)
        
        left_layout.addStretch()
        
        # 右侧：动作选择和日志
        right_panel = QWidget()
        right_layout = QVBoxLayout(right_panel)
        right_layout.setSpacing(12)
        
        # 动作选择
        motion_group = QGroupBox("动作选择")
        motion_layout = QVBoxLayout(motion_group)
        
        self.motion_list = QListWidget()
        for motion in self.motion_options:
            item = QListWidgetItem(motion)
            self.motion_list.addItem(item)
        self.motion_list.setCurrentRow(0)
        
        self.send_motion_btn = QPushButton("▶ 发送动作")
        self.send_motion_btn.setEnabled(False)
        self.send_motion_btn.clicked.connect(self._send_motion)
        
        motion_layout.addWidget(self.motion_list)
        motion_layout.addWidget(self.send_motion_btn)
        right_layout.addWidget(motion_group)
        
        # 日志
        log_group = QGroupBox("运行日志")
        log_layout = QVBoxLayout(log_group)
        
        self.log_widget = LogWidget()
        log_layout.addWidget(self.log_widget)
        right_layout.addWidget(log_group)
        
        # 添加到主布局
        splitter = QSplitter(Qt.Orientation.Horizontal)
        splitter.addWidget(left_panel)
        splitter.addWidget(right_panel)
        splitter.setSizes([350, 550])
        main_layout.addWidget(splitter)
        
        # 状态栏
        self.statusBar().showMessage("就绪")
    
    def _start_system(self):
        self.log_widget.log("正在启动系统...", "info")
        
        # 启动仿真器
        xml_path = str(ASSETS_DIR / "g1" / "g1.xml")
        self.simulator_thread = SimulatorThread(self.ctrl_config, xml_path)
        self.simulator_thread.log_signal.connect(self.log_widget.log)
        self.simulator_thread.state_updated.connect(self._on_state_updated)
        self.simulator_thread.start()
        
        # 启动控制器
        self.controller_thread = ControllerThread(self.ctrl_config)
        self.controller_thread.log_signal.connect(self.log_widget.log)
        self.controller_thread.status_signal.connect(self._on_controller_status)
        self.controller_thread.start()
        
        # 更新按钮状态
        self.start_btn.setEnabled(False)
        self.stop_btn.setEnabled(True)
        self.zero_torque_btn.setEnabled(True)
        
        self.status_label.setText("状态: 运行中")
        self.statusBar().showMessage("系统已启动")
    
    def _stop_system(self):
        self.log_widget.log("正在停止系统...", "warn")
        
        if self.controller_thread:
            self.controller_thread.stop()
        if self.simulator_thread:
            self.simulator_thread.stop()
        
        # 更新按钮状态
        self.start_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)
        self.zero_torque_btn.setEnabled(False)
        self.default_pose_btn.setEnabled(False)
        self.start_policy_btn.setEnabled(False)
        self.send_motion_btn.setEnabled(False)
        
        self.status_label.setText("状态: 已停止")
        self.statusBar().showMessage("系统已停止")
    
    def _zero_torque(self):
        if self.controller_thread:
            self.controller_thread.start_zero_torque()
        self.default_pose_btn.setEnabled(True)
    
    def _move_to_default(self):
        if self.simulator_thread:
            self.simulator_thread.start_gantry()
            self.simulator_thread.set_button(KeyMap.start)
    
    def _start_policy(self):
        if self.simulator_thread:
            self.simulator_thread.start_control()
            self.simulator_thread.set_button(KeyMap.A)
    
    def _send_motion(self):
        current_item = self.motion_list.currentItem()
        if current_item and self.controller_thread:
            motion_name = current_item.text()
            self.controller_thread.request_motion(motion_name)
    
    def _on_state_updated(self, state: dict):
        self.height_label.setText(f"高度: {state['height']:.3f} m")
        self.loop_label.setText(f"循环计数: {state['loop_count']}")
    
    def _on_controller_status(self, status: str):
        if status == "default_pose":
            self.start_policy_btn.setEnabled(True)
        elif status == "running":
            self.send_motion_btn.setEnabled(True)
    
    def _update_status(self):
        sim_running = self.simulator_thread and self.simulator_thread.is_running
        ctrl_running = self.controller_thread and self.controller_thread.is_running
        
        if sim_running and ctrl_running:
            self.status_label.setStyleSheet("color: #90be6d;")
        elif sim_running or ctrl_running:
            self.status_label.setStyleSheet("color: #f9c74f;")
        else:
            self.status_label.setStyleSheet("color: #666;")
    
    def closeEvent(self, event):
        self._stop_system()
        event.accept()


def main():
    # 设置多进程启动方法
    try:
        import multiprocessing as mp
        if mp.get_start_method(allow_none=True) is None:
            mp.set_start_method('spawn', force=True)
    except Exception:
        pass
    
    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    
    window = Sim2SimGUI()
    window.show()
    
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
