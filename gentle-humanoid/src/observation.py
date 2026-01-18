import numpy as np
from scipy.spatial.transform import Rotation as R

from common.math_utils import _clamp_indices, _quat_apply_inv
from common.remote_controller import KeyMap

class BaseObs:
    @property
    def size(self) -> int: ...
    def update(self): ...
    def compute(self) -> np.ndarray: ...

class TrackingCommandObs:
    def __init__(self, ctrl, policy):
        self.ctrl = ctrl
        self.policy = policy

        self.future_steps = np.array([0, 2, 4, 8, 16], dtype=np.int32)

        # State
        self.force_limit = 10.0
        self.force_limit_range = (5.0, 15.0)
        self.btn_state = {"up": False, "down": False}

    @property
    def size(self) -> int:
        n_fut = len(self.future_steps)
        n_j = getattr(self.policy, "n_joints", 0)
        return n_fut * 1 + (n_fut - 1) * 2 + (n_fut - 1) * 2 + 1 + n_fut * n_j + n_fut * 3

    def reset(self):
        pass

    def update(self):
        # Button edge-detection for stiffness
        if self.ctrl.remote_controller.button[KeyMap.up] == 1 and not self.btn_state["up"]:
            self.force_limit = min(self.force_limit + 1.0, self.force_limit_range[1])
            print(f"[CommandObs] Force limit increased to {self.force_limit:.2f}")
        elif self.ctrl.remote_controller.button[KeyMap.down] == 1 and not self.btn_state["down"]:
            self.force_limit = max(self.force_limit - 1.0, self.force_limit_range[0])
            print(f"[CommandObs] Force limit decreased to {self.force_limit:.2f}")

        self.btn_state["up"] = self.ctrl.remote_controller.button[KeyMap.up] == 1
        self.btn_state["down"] = self.ctrl.remote_controller.button[KeyMap.down] == 1

    def compute(self) -> np.ndarray:
        # If we don't have a ref yet, return zeros
        if (self.policy.ref_joint_pos is None or
            self.policy.ref_root_height is None or
            self.policy.ref_root_quat_rp is None or
            self.policy.ref_root_quat_yaw is None or
            self.policy.ref_root_pos is None):
            raise ValueError("Ref data not available yet.")

        # Indices for future samples
        base = self.policy.ref_idx
        T = self.policy.ref_len
        fut_idx = _clamp_indices(base + self.future_steps, T)

        # 1) Reference future torso in world (already aligned by policy)
        pos_w = self.policy.ref_root_pos[fut_idx].copy()          # (N,3)
        quat_yaw_w = self.policy.ref_root_quat_yaw[fut_idx].copy()        # (N,4)
        root_height = self.policy.ref_root_height[fut_idx, :].copy()  # (N,1)

        yaw_ref = quat_yaw_w[1:]
        yaw_now = quat_yaw_w[0]

        pos_diff_w = pos_w[1:] - pos_w[0:1]              # (N-1,3)
        pos_diff_b = _quat_apply_inv(yaw_now, pos_diff_w)    #

        heading_rel = np.array([1.0, 0.0, 0.0], dtype=np.float32)
        heading_target_w = R.from_quat(yaw_ref, scalar_first=True).apply(heading_rel)
        heading_target_w = np.asarray(heading_target_w, dtype=np.float32)
        heading_target_b = _quat_apply_inv(yaw_now, heading_target_w)
        heading_target_b = np.asarray(heading_target_b, dtype=np.float32)

        # 4) Future gravity / joints straight from ref-stream
        tgt_joints = self.policy.ref_joint_pos[fut_idx]            # (N, J)

        # Gravity in *ref root* local frame (same as before)
        tgt_quat_root = self.policy.ref_root_quat_rp[fut_idx]         # (N, 4)
        g_world = np.array([0., 0., -1.], dtype=np.float32).reshape(1, 3)
        g_local = _quat_apply_inv(tgt_quat_root, g_world)          # (N, 3)

        # 5) Pack
        obs = np.concatenate(
            [
                root_height.reshape(-1),
                pos_diff_b[:, :2].reshape(-1),
                heading_target_b[:, :2].reshape(-1),
                np.array([self.force_limit], dtype=np.float32),
                tgt_joints.reshape(-1),
                g_local.reshape(-1),
            ],
            axis=-1,
        )
        return obs.astype(np.float32)

class RootAngVelB(BaseObs):
    def __init__(self, ctrl):
        self.ctrl = ctrl

    @property
    def size(self):
        return 3

    def compute(self):
        g = self.ctrl.gyro
        return g

class ProjectedGravityB(BaseObs):
    def __init__(self, ctrl):
        self.ctrl = ctrl

    @property
    def size(self): return 3

    def compute(self):
        quat = self.ctrl.quat
        g_world = np.array([0., 0., -1.], dtype=np.float32)
        g_body = _quat_apply_inv(quat, g_world)
        g_body /= np.linalg.norm(g_body) + 1e-8
        return g_body

class JointPos(BaseObs):
    def __init__(self, ctrl,
                 pos_steps=(0, 1, 2, 4, 8, 16)):
        self.ctrl = ctrl
        self.pos_steps = list(pos_steps)
        
        self.num_joints = len(ctrl.config.isaac_joint_names_state)
        self.max_step = max(self.pos_steps)
        self.hist = np.zeros((self.max_step + 1, self.num_joints), dtype=np.float32)

    @property
    def size(self):
        return len(self.pos_steps) * self.num_joints
    
    def reset(self):
        self.hist[:] = self.ctrl.qj_isaac.copy().reshape(1, -1)

    def update(self):
        self.hist = np.roll(self.hist, 1, axis=0)
        cur = self.ctrl.qj_isaac.copy()
        self.hist[0] = cur

    def compute(self):
        pos = self.hist[self.pos_steps].reshape(-1)
        return pos

class JointTorque(BaseObs):
    def __init__(self, ctrl):
        self.ctrl = ctrl
        self.num_joints = len(self.ctrl.config.isaac_joint_names_state)
        self.tau = np.zeros(self.num_joints, dtype=np.float32)

    @property
    def size(self):
        return self.num_joints
    
    def reset(self):
        self.tau[:] = 0.0

    def update(self):
        self.tau[:] = self.ctrl.tau_isaac

    def compute(self):
        return self.tau.copy()

from policy import Policy
class PrevActions(BaseObs):
    def __init__(self, policy: Policy, steps=1, old_style=False):
        self.policy = policy
        self.steps = steps
        self.action_dim = self.policy.last_action.shape[0]
        self.buf = np.zeros((steps, self.action_dim), dtype=np.float32)
        self.old_style = old_style

    @property
    def size(self):
        return self.action_dim * self.steps

    def reset(self):
        self.buf[:] = 0.0

    def update(self):
        self.buf = np.roll(self.buf, 1, axis=0)
        if self.old_style:
            self.buf[0, :] = self.policy.applied_action_isaac
        else:
            self.buf[0, :] = self.policy.last_action

    def compute(self):
        return self.buf.reshape(-1)

class BootIndicator(BaseObs):
    def __init__(self):
        pass

    @property
    def size(self):
        return 1

    def compute(self):
        return np.array([0.0], dtype=np.float32)
