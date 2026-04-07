#!/usr/bin/env python3
"""
G1 机器人实机部署脚本
====================
将 PyBullet 训练好的 PPO 策略部署到真实 Unitree G1 机器人

用法：
  # 干跑模式（无需机器人，验证逻辑）
  python3 scripts/deploy_real.py --dry-run

  # 实机部署
  python3 scripts/deploy_real.py \
    --model  models/ppo_riding_2000000_steps \
    --norm   models/vec_normalize_final.pkl \
    --network eth0   # 连接 G1 的网卡名称

安全警告：
  !! 部署前必须阅读 docs/DEPLOY.md 中的安全检查清单 !!
  !! 机器人必须绑安全吊绳，旁边有人手持急停 !!
"""

import argparse
import sys
import os
import time
import signal
import threading
import pickle
import numpy as np

# ─────────────────────────────────────────────
#  可选：unitree SDK（只有实机模式需要）
# ─────────────────────────────────────────────
HAS_SDK = False
try:
    from unitree_sdk2py.core.channel import (
        ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize
    )
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
    from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
    from unitree_sdk2py.utils.crc import CRC as _CRC
    from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
    _crc = _CRC()
    HAS_SDK = True
except ImportError:
    pass

HAS_SB3 = False
try:
    from stable_baselines3 import PPO
    HAS_SB3 = True
except Exception:
    pass  # TorchScript 模式不需要 SB3

# ─────────────────────────────────────────────
#  G1 关节索引（29 自由度版本）
# ─────────────────────────────────────────────
class G1JointIdx:
    # 左腿
    L_HIP_PITCH    = 0
    L_HIP_ROLL     = 1
    L_HIP_YAW      = 2
    L_KNEE         = 3
    L_ANKLE_PITCH  = 4
    L_ANKLE_ROLL   = 5
    # 右腿
    R_HIP_PITCH    = 6
    R_HIP_ROLL     = 7
    R_HIP_YAW      = 8
    R_KNEE         = 9
    R_ANKLE_PITCH  = 10
    R_ANKLE_ROLL   = 11
    # 腰
    WAIST_YAW      = 12
    WAIST_ROLL     = 13
    WAIST_PITCH    = 14
    # 左臂
    L_SH_PITCH     = 15
    L_SH_ROLL      = 16
    L_SH_YAW       = 17
    L_ELBOW        = 18
    L_WRIST_ROLL   = 19
    L_WRIST_PITCH  = 20
    L_WRIST_YAW    = 21
    # 右臂
    R_SH_PITCH     = 22
    R_SH_ROLL      = 23
    R_SH_YAW       = 24
    R_ELBOW        = 25
    R_WRIST_ROLL   = 26
    R_WRIST_PITCH  = 27
    R_WRIST_YAW    = 28
    NUM_MOTORS     = 29


# ─────────────────────────────────────────────
#  BALANCE_JOINTS → G1 motor index 映射
#  顺序必须与 pybullet_riding_env.py BALANCE_JOINTS 完全一致！
# ─────────────────────────────────────────────
POLICY_MOTOR_IDX = [
    G1JointIdx.WAIST_YAW,    # [0] waist_yaw_joint
    G1JointIdx.WAIST_ROLL,   # [1] waist_roll_joint   ← 主平衡关节
    G1JointIdx.WAIST_PITCH,  # [2] waist_pitch_joint
    G1JointIdx.L_SH_PITCH,   # [3] left_shoulder_pitch_joint
    G1JointIdx.L_SH_ROLL,    # [4] left_shoulder_roll_joint
    G1JointIdx.L_SH_YAW,     # [5] left_shoulder_yaw_joint
    G1JointIdx.L_ELBOW,      # [6] left_elbow_joint
    G1JointIdx.R_SH_PITCH,   # [7] right_shoulder_pitch_joint
    G1JointIdx.R_SH_ROLL,    # [8] right_shoulder_roll_joint
    G1JointIdx.R_ELBOW,      # [9] right_elbow_joint
]
ACT_DIM = len(POLICY_MOTOR_IDX)  # = 10

# ─────────────────────────────────────────────
#  骑行参考姿态偏置（rad）
#  含义：G1 上电时关节 q=0 是直立姿态，骑行坐姿与之有偏差
#  标定时间：2026-03-27，机器人坐在车上、手握车把骑行姿态
# ─────────────────────────────────────────────
RIDING_POSE_OFFSET = np.zeros(G1JointIdx.NUM_MOTORS, dtype=np.float64)
RIDING_POSE_OFFSET[G1JointIdx.WAIST_YAW]   = -0.0835  # -4.78°
RIDING_POSE_OFFSET[G1JointIdx.WAIST_ROLL]  = -0.0547  # -3.14°
RIDING_POSE_OFFSET[G1JointIdx.WAIST_PITCH] = -0.0719  # -4.12°
RIDING_POSE_OFFSET[G1JointIdx.L_SH_PITCH]  =  0.1920  # +11.00°
RIDING_POSE_OFFSET[G1JointIdx.L_SH_ROLL]   =  0.1360  # +7.79°
RIDING_POSE_OFFSET[G1JointIdx.L_SH_YAW]    = -0.5021  # -28.77°
RIDING_POSE_OFFSET[G1JointIdx.L_ELBOW]     =  1.2655  # +72.51°
RIDING_POSE_OFFSET[G1JointIdx.R_SH_PITCH]  =  0.1950  # +11.18°
RIDING_POSE_OFFSET[G1JointIdx.R_SH_ROLL]   = -0.0726  # -4.16°
RIDING_POSE_OFFSET[G1JointIdx.R_ELBOW]     =  1.2858  # +73.68°

# ─────────────────────────────────────────────
#  PD 增益（实机值，比 PyBullet 训练值保守）
#  训练值：waist kp=200/kd=20, arms kp=60/kd=6
#  实机建议：先从 1/4 开始，逐步提高
# ─────────────────────────────────────────────
KP_REAL = np.zeros(G1JointIdx.NUM_MOTORS, dtype=np.float64)
KD_REAL = np.zeros(G1JointIdx.NUM_MOTORS, dtype=np.float64)

# 腰部（主平衡）
for idx in [G1JointIdx.WAIST_YAW, G1JointIdx.WAIST_ROLL, G1JointIdx.WAIST_PITCH]:
    KP_REAL[idx] = 50.0   # 训练值 200 的 25%，保守起步
    KD_REAL[idx] = 5.0

# 手臂
for idx in [G1JointIdx.L_SH_PITCH, G1JointIdx.L_SH_ROLL, G1JointIdx.L_SH_YAW,
            G1JointIdx.L_ELBOW,
            G1JointIdx.R_SH_PITCH, G1JointIdx.R_SH_ROLL, G1JointIdx.R_ELBOW]:
    KP_REAL[idx] = 15.0   # 训练值 60 的 25%
    KD_REAL[idx] = 2.0

# 腿部（骑行时固定坐姿，给阻尼）
for idx in range(12):
    KP_REAL[idx] = 0.0    # 不控制位置
    KD_REAL[idx] = 2.0    # 只加阻尼，防止被动晃动

# 手腕（锁定，保持车把姿势）
for idx in [G1JointIdx.L_WRIST_ROLL, G1JointIdx.L_WRIST_PITCH, G1JointIdx.L_WRIST_YAW,
            G1JointIdx.R_WRIST_ROLL, G1JointIdx.R_WRIST_PITCH, G1JointIdx.R_WRIST_YAW]:
    KP_REAL[idx] = 30.0
    KD_REAL[idx] = 3.0

# 动作限幅（与训练环境一致）
ACT_LIMIT = np.array([
    0.30, 0.30, 0.20,           # waist yaw/roll/pitch
    0.50, 0.30, 0.50, 0.50,     # left shoulder pitch/roll/yaw, elbow
    0.50, 0.30, 0.50,           # right shoulder pitch/roll, elbow
], dtype=np.float64)

# 控制频率
CTRL_HZ = 50
CTRL_DT = 1.0 / CTRL_HZ

# 热身步数（从 Damping 到策略控制的过渡帧数）
WARMUP_STEPS = 100   # 100帧 × 20ms = 2s


# ═══════════════════════════════════════════════════
#  观测向量构建（与 pybullet_riding_env._get_obs 对应）
# ═══════════════════════════════════════════════════
def build_obs(low_state, prev_action: np.ndarray, target_speed: float = 3.0) -> np.ndarray:
    """
    从 G1 LowState 构建 38 维观测向量

    obs 结构（与训练环境完全一致）：
      [0:3]   ang_vel        — 骨盆 IMU 角速度 (rad/s)
      [3:6]   proj_gravity   — 机体坐标系中的重力方向
      [6:16]  joint_pos      — 10个控制关节角度 (rad)
      [16:26] joint_vel      — 10个控制关节角速度 (rad/s)
      [26]    cmd_vel        — 速度指令（归一化，target/3.0）
      [27]    yaw_error      — 偏航误差 (rad)
      [28:38] prev_action    — 上一步动作
    """
    obs = np.zeros(38, dtype=np.float64)

    # ── IMU ──
    imu = low_state.imu_state
    obs[0:3] = imu.gyroscope          # 角速度

    # ── 投影重力（从 IMU 四元数计算）──
    # G1 IMU quaternion: [w, x, y, z]
    w, x, y, z = imu.quaternion
    # 旋转矩阵第三列（机体 z 轴在世界坐标系）
    rot_col3 = np.array([
        2*(x*z - w*y),
        2*(y*z + w*x),
        1 - 2*(x*x + y*y)
    ])
    # 世界重力 [0,0,-1] 在机体坐标系中的投影
    obs[3:6] = -rot_col3   # proj_gravity

    # ── 关节状态 ──
    for i, motor_idx in enumerate(POLICY_MOTOR_IDX):
        ms = low_state.motor_state[motor_idx]
        # 减去骑行参考姿态偏置，使参考姿态下观测为 0
        obs[6  + i] = ms.q  - RIDING_POSE_OFFSET[motor_idx]
        obs[16 + i] = ms.dq

    # ── 任务状态 ──
    obs[26] = target_speed / 3.0      # 速度指令（归一化）
    obs[27] = imu.rpy[2]              # yaw 偏航角

    # ── 上一步动作 ──
    obs[28:38] = prev_action

    return obs.astype(np.float32)


def quat_to_rot_matrix(w, x, y, z):
    """四元数转旋转矩阵"""
    return np.array([
        [1-2*(y*y+z*z),  2*(x*y-w*z),   2*(x*z+w*y)],
        [2*(x*y+w*z),    1-2*(x*x+z*z), 2*(y*z-w*x)],
        [2*(x*z-w*y),    2*(y*z+w*x),   1-2*(x*x+y*y)],
    ])


# ═══════════════════════════════════════════════════
#  归一化（VecNormalize 推理时手动应用）
# ═══════════════════════════════════════════════════
class ObsNormalizer:
    """从 VecNormalize pkl 或 npy 文件提取统计量，用于单步推理"""
    def __init__(self, norm_path: str):
        if norm_path.endswith('.json'):
            import json
            with open(norm_path, 'r') as f:
                d = json.load(f)
            self.mean     = np.array(d['obs_mean'], dtype=np.float32)
            self.var      = np.array(d['obs_var'],  dtype=np.float32)
            self.clip_obs = float(d['clip_obs'])
        elif norm_path.endswith('.npy'):
            d = np.load(norm_path, allow_pickle=True).item()
            self.mean     = np.array(d['obs_mean'], dtype=np.float32)
            self.var      = np.array(d['obs_var'],  dtype=np.float32)
            self.clip_obs = float(d['clip_obs'])
        else:
            with open(norm_path, 'rb') as f:
                vec_norm = pickle.load(f)
            self.mean     = vec_norm.obs_rms.mean.astype(np.float32)
            self.var      = vec_norm.obs_rms.var.astype(np.float32)
            self.clip_obs = float(vec_norm.clip_obs)
        print(f"[归一化] 均值范围: [{self.mean.min():.3f}, {self.mean.max():.3f}]")
        print(f"[归一化] 方差范围: [{self.var.min():.4f}, {self.var.max():.4f}]")

    def normalize(self, obs: np.ndarray) -> np.ndarray:
        # 对 std<0.01 的维度（训练中常数，如恒定速度指令）直接置 0，避免除以近零值
        std = np.sqrt(self.var + 1e-8)
        safe = std > 0.01
        obs_norm = np.where(safe, (obs - self.mean) / std, 0.0)
        return np.clip(obs_norm, -self.clip_obs, self.clip_obs).astype(np.float32)


# ═══════════════════════════════════════════════════
#  干跑模式（无需真实机器人，验证代码逻辑）
# ═══════════════════════════════════════════════════
class FakeLowState:
    """模拟 G1 LowState，用于 dry-run 测试"""
    class ImuState:
        def __init__(self):
            self.gyroscope  = [0.0, 0.0, 0.0]
            self.quaternion = [1.0, 0.0, 0.0, 0.0]  # w, x, y, z
            self.rpy        = [0.0, 0.0, 0.0]

    class MotorState:
        def __init__(self):
            self.q  = 0.0
            self.dq = 0.0

    def __init__(self):
        self.imu_state   = self.ImuState()
        self.motor_state = [self.MotorState() for _ in range(G1JointIdx.NUM_MOTORS)]


# ═══════════════════════════════════════════════════
#  主控制器
# ═══════════════════════════════════════════════════
class RidingController:
    def __init__(self, model_path: str, norm_path: str, dry_run: bool = False):
        self.dry_run = dry_run

        # ── 加载策略（支持 SB3 zip 或 TorchScript pt）──
        import torch as _torch
        print(f"[加载] 策略: {model_path}")
        if model_path.endswith('.pt'):
            self._ts_model = _torch.jit.load(model_path, map_location='cpu')
            self._ts_model.eval()
            self.model = None
        else:
            self._ts_model = None
            if not HAS_SB3:
                raise RuntimeError("SB3 未安装，请使用 .pt TorchScript 模型")
            self.model = PPO.load(model_path, device='cpu')

        print(f"[加载] 归一化: {norm_path}")
        self.normalizer = ObsNormalizer(norm_path)

        # ── 状态 ──
        self.prev_action = np.zeros(ACT_DIM, dtype=np.float32)
        self.current_low_state = FakeLowState()
        self._lock = threading.Lock()
        self._running = False
        self._estop   = False

        # ── 实机 SDK ──
        self._pub = None
        if not dry_run:
            if not HAS_SDK:
                raise RuntimeError(
                    "unitree_sdk2py 未安装，请先安装：\n"
                    "  pip3 install unitree-sdk2py"
                )

    def init_sdk(self, network_interface: str):
        """初始化 Unitree SDK，连接机器人"""
        print(f"[SDK] 初始化，网卡: {network_interface}")
        ChannelFactoryInitialize(0, network_interface)

        # 释放高层运动控制器，否则 LowCmd 会被高层覆盖
        msc = MotionSwitcherClient()
        msc.SetTimeout(5.0)
        msc.Init()
        status, result = msc.CheckMode()
        if result.get('name'):
            print(f"[SDK] 释放高层控制器: {result['name']}")
            msc.ReleaseMode()
            time.sleep(1.0)

        # 读取当前 mode_machine（必须与 LowCmd 一致）
        self._mode_machine = 0

        # 订阅机器人状态
        self._sub = ChannelSubscriber("rt/lowstate", LowState_)
        self._sub.Init(self._low_state_callback, 10)
        # 发布控制命令
        self._pub = ChannelPublisher("rt/lowcmd", LowCmd_)
        self._pub.Init()
        time.sleep(1.0)   # 等待 DDS 发现
        # 从第一帧状态中读取 mode_machine
        if self.current_low_state:
            self._mode_machine = self.current_low_state.mode_machine
        print(f"[SDK] 连接成功，mode_machine={self._mode_machine}")

    def _low_state_callback(self, msg):
        """接收机器人状态的回调函数"""
        with self._lock:
            self.current_low_state = msg

    def _build_cmd(self, action: np.ndarray) -> 'LowCmd_':
        """
        根据 RL 动作构建 G1 LowCmd

        action[i] = 关节位置目标（相对骑行参考姿态的偏移，rad）
        实际指令 = RIDING_POSE_OFFSET[motor_idx] + action[i]
        """
        cmd = unitree_hg_msg_dds__LowCmd_()
        cmd.mode_pr = 0
        cmd.mode_machine = self._mode_machine  # 与 LowState 保持一致

        for i in range(G1JointIdx.NUM_MOTORS):
            cmd.motor_cmd[i].mode = 1    # 1=Enable（必须设置，否则电机不响应）
            cmd.motor_cmd[i].q   = 0.0
            cmd.motor_cmd[i].dq  = 0.0
            cmd.motor_cmd[i].kp  = KP_REAL[i]
            cmd.motor_cmd[i].kd  = KD_REAL[i]
            cmd.motor_cmd[i].tau = 0.0

        # RL 控制的 10 个平衡关节：设置位置目标
        for i, motor_idx in enumerate(POLICY_MOTOR_IDX):
            target_q = RIDING_POSE_OFFSET[motor_idx] + float(action[i])
            cmd.motor_cmd[motor_idx].q = target_q

        cmd.crc = _crc.Crc(cmd)
        return cmd

    def _send_damping(self):
        """发送纯阻尼命令（安全状态）"""
        if self.dry_run:
            return
        cmd = unitree_hg_msg_dds__LowCmd_()
        cmd.mode_pr = 0
        cmd.mode_machine = self._mode_machine
        for i in range(G1JointIdx.NUM_MOTORS):
            cmd.motor_cmd[i].mode = 1    # 1=Enable
            cmd.motor_cmd[i].q   = 0.0
            cmd.motor_cmd[i].dq  = 0.0
            cmd.motor_cmd[i].kp  = 0.0
            cmd.motor_cmd[i].kd  = 2.0
            cmd.motor_cmd[i].tau = 0.0
        cmd.crc = _crc.Crc(cmd)
        self._pub.Write(cmd)

    def estop(self, *args):
        """紧急停止"""
        print("\n[急停] 接收到停止信号，切换到纯阻尼模式...")
        self._estop = True
        self._running = False

    def run(self, target_speed: float = 3.0, max_episodes: int = 5,
            max_steps_per_episode: int = 0):
        """主控制循环
        max_steps_per_episode: 每集最大步数，0 表示不限（dry-run 默认 50）
        """
        signal.signal(signal.SIGINT,  self.estop)
        signal.signal(signal.SIGTERM, self.estop)
        if max_steps_per_episode == 0 and self.dry_run:
            max_steps_per_episode = 50

        self._running = True

        print("\n" + "="*50)
        print(" G1 骑行平衡控制 启动")
        print(f" 控制频率: {CTRL_HZ} Hz")
        print(f" 目标速度: {target_speed} m/s")
        print(f" 模式: {'DRY-RUN（无机器人）' if self.dry_run else '实机'}")
        print("="*50)

        # ── 热身阶段：发送阻尼命令等待稳定 ──
        print(f"\n[热身] 发送阻尼命令 {WARMUP_STEPS} 帧 ({WARMUP_STEPS*CTRL_DT:.1f}s)...")
        for step in range(WARMUP_STEPS):
            if self._estop:
                break
            self._send_damping()
            time.sleep(CTRL_DT)
            if step % 25 == 0:
                print(f"  热身进度: {step}/{WARMUP_STEPS}")

        if self._estop:
            return

        print("\n[策略] 开始运行 RL 策略...")
        print("按 Ctrl+C 急停\n")

        # ── 主循环 ──
        episode = 0
        while self._running and episode < max_episodes:
            episode += 1
            print(f"\n── Episode {episode}/{max_episodes} ──")
            self.prev_action = np.zeros(ACT_DIM, dtype=np.float32)
            step_count = 0

            while self._running:
                t0 = time.time()

                # 1. 读取机器人状态
                with self._lock:
                    low_state = self.current_low_state

                # 2. 构建观测
                obs_raw = build_obs(low_state, self.prev_action, target_speed)

                # 3. 安全检查
                roll  = low_state.imu_state.rpy[0]
                pitch = low_state.imu_state.rpy[1]
                if abs(roll) > 0.6 or abs(pitch) > 0.6:
                    print(f"[安全] 倾斜过大 roll={np.degrees(roll):.1f}° pitch={np.degrees(pitch):.1f}°，急停")
                    self.estop()
                    break

                # 4. 归一化
                obs_norm = self.normalizer.normalize(obs_raw)

                # 5. 策略推理
                if self._ts_model is not None:
                    import torch as _torch
                    with _torch.no_grad():
                        t = _torch.from_numpy(obs_norm[None]).float()
                        action = self._ts_model(t).numpy()[0]
                else:
                    action, _ = self.model.predict(obs_norm, deterministic=True)
                action = np.clip(action, -ACT_LIMIT, ACT_LIMIT).astype(np.float32)

                # 6. 发送命令
                if not self.dry_run:
                    cmd = self._build_cmd(action)
                    self._pub.Write(cmd)

                self.prev_action = action
                step_count += 1

                # 7. 打印状态（每 50 步）
                if step_count % 50 == 0:
                    print(f"  步数={step_count:4d} | "
                          f"roll={np.degrees(roll):+6.2f}° | "
                          f"pitch={np.degrees(pitch):+6.2f}° | "
                          f"act_norm={np.linalg.norm(action):.3f}")

                # 8. 控制频率
                elapsed = time.time() - t0
                sleep_t = CTRL_DT - elapsed
                if sleep_t > 0:
                    time.sleep(sleep_t)
                elif step_count % 100 == 0:
                    print(f"  [警告] 控制延迟 {-sleep_t*1000:.1f}ms")

            if self._estop:
                break

        # ── 结束：切回阻尼 ──
        print("\n[结束] 切换到纯阻尼模式...")
        for _ in range(3):
            self._send_damping()
            time.sleep(0.1)
        print("[结束] 完成")


# ═══════════════════════════════════════════════════
#  入口
# ═══════════════════════════════════════════════════
def main():
    _script_dir = os.path.dirname(os.path.abspath(__file__))
    parser = argparse.ArgumentParser(description='G1 实机部署')
    parser.add_argument('--model',    default=os.path.join(_script_dir, 'models', 'ppo_riding_2000000_steps'),
                        help='PPO 模型路径（不含 .zip）')
    parser.add_argument('--norm',     default=os.path.join(_script_dir, 'models', 'vec_normalize_final.pkl'),
                        help='VecNormalize 统计文件路径')
    parser.add_argument('--network',  default='eth0',
                        help='连接 G1 的网卡名（仅实机模式）')
    parser.add_argument('--speed',    type=float, default=0.0,
                        help='目标速度 m/s（建议先用 0.0 静止测试）')
    parser.add_argument('--episodes', type=int,   default=3,
                        help='运行集数')
    parser.add_argument('--dry-run',  action='store_true',
                        help='干跑模式（不连接机器人，验证代码逻辑）')
    parser.add_argument('--yes', action='store_true',
                        help='跳过安全确认（SSH 非交互模式使用）')
    args = parser.parse_args()

    # 切换到脚本所在目录的上级（项目根目录）
    os.chdir(os.path.join(os.path.dirname(__file__), '..'))

    ctrl = RidingController(
        model_path=args.model,
        norm_path=args.norm,
        dry_run=args.dry_run,
    )

    if not args.dry_run:
        if args.yes:
            print("[安全确认] 已通过 --yes 参数跳过")
        else:
            print("\n[安全确认]")
            print("  1. 机器人已绑好安全吊绳")
            print("  2. 机器人已坐在电动车上")
            print("  3. 附近无障碍物")
            print("  4. 已确认急停设备就绪")
            confirm = input("  确认以上条件，输入 'yes' 继续：")
            if confirm.strip().lower() != 'yes':
                print("取消部署")
                sys.exit(0)

        ctrl.init_sdk(args.network)
    else:
        print("\n[DRY-RUN 模式] 将模拟 50 步控制循环，验证代码逻辑...")

    ctrl.run(target_speed=args.speed, max_episodes=args.episodes)


if __name__ == '__main__':
    main()
