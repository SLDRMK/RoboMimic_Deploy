from common.path_config import PROJECT_ROOT

from FSM.FSMState import FSMStateName, FSMState
from common.ctrlcomp import StateAndCmd, PolicyOutput, FSMCommand
import numpy as np
import yaml
import torch
import os
import json
import math

class DanceUpper(FSMState):
    def __init__(self, state_cmd:StateAndCmd, policy_output:PolicyOutput):
        super().__init__()
        self.state_cmd = state_cmd
        self.policy_output = policy_output
        self.name = FSMStateName.SKILL_KungFu2  # 使用原来的kungfu2状态名
        self.name_str = "dance_upper"
        self.alpha = 0.
        self.cur_step = 0
        self.control_dt = 0.02
        
        # 加载skill_cooldown配置
        current_dir = os.path.dirname(os.path.abspath(__file__))
        skill_cooldown_config_path = os.path.join(current_dir, "..", "skill_cooldown", "config", "SkillCooldown.yaml")
        with open(skill_cooldown_config_path, "r") as f:
            config = yaml.load(f, Loader=yaml.FullLoader)
            self.policy_path = os.path.join(current_dir, "..", "skill_cooldown", "model", config["policy_path"])
            self.kps = np.array(config["kps"], dtype=np.float32)
            self.kds = np.array(config["kds"], dtype=np.float32)
            self.default_angles = np.array(config["default_angles"], dtype=np.float32)
            self.upper_body_motor_idx = np.array(config["upper_body_motor_idx"], dtype=np.int32)
            self.lower_body_motor_idx = np.array(config["lower_body_motor_idx"], dtype=np.int32)
            self.tau_limit = np.array(config["tau_limit"], dtype=np.float32)
            self.num_actions = config["num_actions"]
            self.num_obs = config["num_obs"]
            self.ang_vel_scale = config["ang_vel_scale"]
            self.dof_pos_scale = config["dof_pos_scale"]
            self.dof_vel_scale = config["dof_vel_scale"]
            self.action_scale = config["action_scale"]
            self.total_time = config["total_time"]
            self.period = config["period"]
            
            self.qj_obs = np.zeros(self.num_actions, dtype=np.float32)
            self.dqj_obs = np.zeros(self.num_actions, dtype=np.float32)
            self.obs = np.zeros(self.num_obs)
            self.action = np.zeros(self.num_actions)
            
            # 加载skill_cooldown策略
            self.policy = torch.jit.load(self.policy_path)
            
            for _ in range(50):
                with torch.inference_mode():
                    obs_tensor = self.obs.reshape(1, -1)
                    obs_tensor = obs_tensor.astype(np.float32)
                    self.policy(torch.from_numpy(obs_tensor))
        
        # 加载舞蹈数据
        dance_data_path = os.path.join(PROJECT_ROOT, "舞蹈数据_小苹果_2025-07-31.json")
        with open(dance_data_path, "r", encoding="utf-8") as f:
            self.dance_data = json.load(f)
        
        # 初始化舞蹈相关变量
        self.dance_poses = self.dance_data["poses"]
        self.choreography = self.dance_data["choreography"]["sequence"]
        self.current_pose_idx = 0
        self.pose_start_time = 0.0
        self.total_dance_time = 0.0
        self.upper_init_pos = np.zeros(len(self.upper_body_motor_idx), dtype=np.float32)
        
        print("DanceUpper policy initializing ...")
    
    def s_curve_interpolation(self, t, t_start, t_end, start_pos, end_pos):
        """
        S型插值函数
        t: 当前时间
        t_start: 开始时间
        t_end: 结束时间
        start_pos: 起始位置
        end_pos: 结束位置
        """
        if t <= t_start:
            return start_pos
        elif t >= t_end:
            return end_pos
        
        # 计算插值进度 (0-1)
        progress = (t - t_start) / (t_end - t_start)
        
        # S型插值: 使用sigmoid函数
        # 将progress从[0,1]映射到[-6,6]，然后应用sigmoid
        x = (progress - 0.5) * 12  # 映射到[-6, 6]
        s_curve = 1.0 / (1.0 + math.exp(-x))
        
        # 线性插值
        return start_pos + s_curve * (end_pos - start_pos)
    
    def get_current_pose_target(self, current_time):
        """
        根据当前时间获取目标姿态
        """
        if self.current_pose_idx >= len(self.choreography):
            # 舞蹈结束，返回最后一个姿态
            last_pose = self.dance_poses[-1]["positions"]
            return np.array(last_pose[:14], dtype=np.float32)  # 只取前14个关节（上肢）
        
        current_sequence = self.choreography[self.current_pose_idx]
        pose_name = current_sequence["pose_name"]
        transition_time = current_sequence["transition_time"]
        hold_time = current_sequence["hold_time"]
        
        # 找到对应的姿态数据
        pose_data = None
        for pose in self.dance_poses:
            if pose["name"] == pose_name:
                pose_data = pose
                break
        
        if pose_data is None:
            return np.array(self.default_angles[self.upper_body_motor_idx], dtype=np.float32)
        
        # 计算当前姿态的时间范围
        pose_start = self.pose_start_time
        pose_transition_end = pose_start + transition_time
        pose_end = pose_transition_end + hold_time
        
        # 获取当前姿态的目标位置（前14个关节）
        dance_positions = np.array(pose_data["positions"][:14], dtype=np.float32)
        
        # 舞蹈数据中的关节顺序：左臂(0-6) + 右臂(7-13)
        # 需要映射到机器人的关节索引：左臂(15-21) + 右臂(22-28)
        target_pos = np.zeros(14, dtype=np.float32)
        # 左臂映射
        target_pos[0:7] = dance_positions[0:7]  # 左臂关节
        # 右臂映射  
        target_pos[7:14] = dance_positions[7:14]  # 右臂关节
        
        # 获取上一个姿态的位置（用于插值）
        if self.current_pose_idx > 0:
            prev_sequence = self.choreography[self.current_pose_idx - 1]
            prev_pose_name = prev_sequence["pose_name"]
            prev_pose_data = None
            for pose in self.dance_poses:
                if pose["name"] == prev_pose_name:
                    prev_pose_data = pose
                    break
            if prev_pose_data:
                prev_dance_positions = np.array(prev_pose_data["positions"][:14], dtype=np.float32)
                prev_pos = np.zeros(14, dtype=np.float32)
                prev_pos[0:7] = prev_dance_positions[0:7]  # 左臂关节
                prev_pos[7:14] = prev_dance_positions[7:14]  # 右臂关节
            else:
                prev_pos = self.upper_init_pos
        else:
            prev_pos = self.upper_init_pos
        
        # 使用S型插值
        if current_time <= pose_transition_end:
            # 过渡阶段
            interpolated_pos = self.s_curve_interpolation(
                current_time, pose_start, pose_transition_end, prev_pos, target_pos
            )
        else:
            # 保持阶段
            interpolated_pos = target_pos
        
        return interpolated_pos
    
    def enter(self):
        self.num_step = int(self.total_time / self.control_dt)
        self.upper_dof_size = len(self.upper_body_motor_idx)
        self.upper_init_pos = np.zeros(self.upper_dof_size, dtype=np.float32)
        self.alpha = 0.
        self.cur_step = 0
        self.current_pose_idx = 0
        self.pose_start_time = 0.0
        self.total_dance_time = 0.0
        
        # 记录当前上肢位置作为初始位置
        for i in range(self.upper_dof_size):
            self.upper_init_pos[i] = self.state_cmd.q[self.upper_body_motor_idx[i]]
        
        print("DanceUpper entered - 开始上肢舞蹈")
    
    def run(self):
        # 下肢控制（使用skill_cooldown逻辑）
        self.gravity_orientation = self.state_cmd.gravity_ori
        self.qj = self.state_cmd.q.copy()
        self.dqj = self.state_cmd.dq.copy()
        self.ang_vel = self.state_cmd.ang_vel.copy()
        self.cmd = np.zeros(3)
        
        self.qj_obs = (self.qj[self.lower_body_motor_idx] - self.default_angles[self.lower_body_motor_idx]) * self.dof_pos_scale
        self.dqj_obs = self.dqj[self.lower_body_motor_idx] * self.dof_vel_scale
        self.ang_vel = self.ang_vel * self.ang_vel_scale
        
        count = self.cur_step * self.control_dt
        phase = count % self.period / self.period
        sin_phase = np.sin(2 * np.pi * phase)
        cos_phase = np.cos(2 * np.pi * phase)
        
        self.obs[:3] = self.ang_vel.copy()
        self.obs[3:6] = self.gravity_orientation.copy()
        self.obs[6:9] = self.cmd.copy()
        self.obs[9: 9 + self.num_actions] = self.qj_obs.copy()
        self.obs[9 + self.num_actions: 9 + self.num_actions * 2] = self.dqj_obs.copy()
        self.obs[9 + self.num_actions * 2: 9 + self.num_actions * 3] = self.action.copy()
        self.obs[9 + 3 * self.num_actions : 9 + 3 * self.num_actions + 2] = np.array([sin_phase, cos_phase])
        
        obs_tensor = self.obs.reshape(1, -1)
        obs_tensor = obs_tensor.astype(np.float32)
        self.action = self.policy(torch.from_numpy(obs_tensor)).detach().numpy().squeeze()
        loco_action = self.action * self.action_scale + self.default_angles[self.lower_body_motor_idx]
        
        # 设置下肢动作
        self.policy_output.actions[self.lower_body_motor_idx] = loco_action[self.lower_body_motor_idx].copy()
        
        # 上肢舞蹈控制
        current_time = self.total_dance_time
        upper_target_pos = self.get_current_pose_target(current_time)
        
        # 设置上肢动作
        for i, motor_idx in enumerate(self.upper_body_motor_idx):
            self.policy_output.actions[motor_idx] = upper_target_pos[i]
        
        # 设置PD参数
        self.policy_output.kps = self.kps.copy()
        self.policy_output.kds = self.kds.copy()
        
        # 更新时间和姿态索引
        self.cur_step += 1
        self.total_dance_time += self.control_dt
        
        # 检查是否需要切换到下一个姿态
        if self.current_pose_idx < len(self.choreography):
            current_sequence = self.choreography[self.current_pose_idx]
            transition_time = current_sequence["transition_time"]
            hold_time = current_sequence["hold_time"]
            pose_duration = transition_time + hold_time
            
            if self.total_dance_time >= self.pose_start_time + pose_duration:
                self.current_pose_idx += 1
                if self.current_pose_idx < len(self.choreography):
                    self.pose_start_time = self.total_dance_time
                    print(f"切换到舞蹈姿态 {self.current_pose_idx + 1}/{len(self.choreography)}")
    
    def exit(self):
        print("DanceUpper exited")
    
    def checkChange(self):
        if self.state_cmd.skill_cmd == FSMCommand.LOCO:
            self.state_cmd.skill_cmd = FSMCommand.INVALID
            return FSMStateName.SKILL_COOLDOWN
        elif self.state_cmd.skill_cmd == FSMCommand.PASSIVE:
            self.state_cmd.skill_cmd = FSMCommand.INVALID
            return FSMStateName.PASSIVE
        elif self.state_cmd.skill_cmd == FSMCommand.POS_RESET:
            self.state_cmd.skill_cmd = FSMCommand.INVALID
            return FSMStateName.FIXEDPOSE
        else:
            self.state_cmd.skill_cmd = FSMCommand.INVALID
            return FSMStateName.SKILL_KungFu2 