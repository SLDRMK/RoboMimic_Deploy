from common.path_config import PROJECT_ROOT

from FSM.FSMState import FSMStateName, FSMState
from common.ctrlcomp import StateAndCmd, PolicyOutput
from common.utils import scale_values
import numpy as np
import yaml
from common.utils import FSMCommand, progress_bar
import onnx
import onnxruntime
import torch
import os

class KungFu(FSMState):
    def __init__(self, state_cmd:StateAndCmd, policy_output:PolicyOutput):
        super().__init__()
        self.state_cmd = state_cmd
        self.policy_output = policy_output
        self.name = FSMStateName.SKILL_KungFu
        self.name_str = "skill_kungfu"

        self.motion_phase = 0
        self.counter_step = 0
        self.ref_motion_phase = 0
        
        current_dir = os.path.dirname(os.path.abspath(__file__))
        config_path = os.path.join(current_dir, "config", "KungFu_dul_policy.yaml")
        with open(config_path, "r") as f:
            config = yaml.load(f, Loader=yaml.FullLoader)
            self.onnx_path = os.path.join(current_dir, "model", config["onnx_path"])
            self.kps = np.array(config["kps"], dtype=np.float32)
            self.kds = np.array(config["kds"], dtype=np.float32)
            self.default_angles =  np.array(config["default_angles"], dtype=np.float32)
            self.dof23_index =  np.array(config["dof23_index"], dtype=np.int32)
            self.tau_limit =  np.array(config["tau_limit"], dtype=np.float32)
            self.num_actions = config["num_actions"]
            self.num_obs = config["num_obs"]
            self.ang_vel_scale = config["ang_vel_scale"]
            self.dof_pos_scale = config["dof_pos_scale"]
            self.dof_vel_scale = config["dof_vel_scale"]
            self.action_scale = config["action_scale"]

            self.history_length = config["history_length"]
            self.motion_length = config["motion_length"]
            
            self.qj_obs = np.zeros(self.num_actions, dtype=np.float32)
            self.dqj_obs = np.zeros(self.num_actions, dtype=np.float32)
            self.obs = np.zeros(self.num_obs)
            self.action = np.zeros(self.num_actions)

            self.obs_history = np.zeros((self.history_length, self.num_obs), dtype=np.float32)
            
            self.ang_vel_buf = np.zeros(3 * self.history_length, dtype=np.float32)
            self.proj_g_buf = np.zeros(3 * self.history_length, dtype=np.float32)
            self.dof_pos_buf = np.zeros(23 * self.history_length, dtype=np.float32)
            self.dof_vel_buf = np.zeros(23 * self.history_length, dtype=np.float32)
            self.action_buf = np.zeros(23 * self.history_length, dtype=np.float32)
            self.ref_motion_phase_buf = np.zeros(1 * self.history_length, dtype=np.float32)
            
            # load policy
            self.onnx_model = onnx.load(self.onnx_path)
            self.ort_session = onnxruntime.InferenceSession(self.onnx_path)
            self.input_name = self.ort_session.get_inputs()[0].name
            for _ in range(50):
                obs_tensor = torch.from_numpy(self.obs).unsqueeze(0).cpu().numpy()
                obs_tensor = obs_tensor.astype(np.float32)
                self.ort_session.run(None, {self.input_name: obs_tensor})[0]
                    
            print("KungFu policy initializing ...")
            
            ##########################################################
            ################ Locomotion部分 ##########################
            ##########################################################
            self.policy_path_loco = os.path.join(current_dir, "model", config["policy_path_loco"])
            self.kps_loco = np.array(config["kps_loco"], dtype=np.float32)
            self.kds_loco = np.array(config["kds_loco"], dtype=np.float32)
            self.default_angles_loco =  np.array(config["default_angles_loco"], dtype=np.float32)
            self.joint2motor_idx =  np.array(config["joint2motor_idx"], dtype=np.int32)
            self.tau_limit_loco =  np.array(config["tau_limit_loco"], dtype=np.float32)
            self.num_actions_loco = config["num_actions_loco"]
            self.num_obs_loco = config["num_obs_loco"]
            self.ang_vel_scale_loco = config["ang_vel_scale"]
            self.dof_pos_scale_loco = config["dof_pos_scale_loco"]
            self.dof_vel_scale_loco = config["dof_vel_scale_loco"]
            self.action_scale_loco = config["action_scale_loco"]
            
            self.cmd_scale_loco = np.array(config["cmd_scale_loco"], dtype=np.float32)
            self.cmd_range_loco = config["cmd_range_loco"]
            self.range_velx_loco = np.array([self.cmd_range_loco["lin_vel_x"][0], self.cmd_range_loco["lin_vel_x"][1]], dtype=np.float32)
            self.range_vely_loco = np.array([self.cmd_range_loco["lin_vel_y"][0], self.cmd_range_loco["lin_vel_y"][1]], dtype=np.float32)
            self.range_velz_loco = np.array([self.cmd_range_loco["ang_vel_z"][0], self.cmd_range_loco["ang_vel_z"][1]], dtype=np.float32)
            
            self.qj_obs_loco = np.zeros(self.num_actions_loco, dtype=np.float32)
            self.dqj_obs_loco = np.zeros(self.num_actions_loco, dtype=np.float32)
            self.obs_loco = np.zeros(self.num_obs_loco)
            self.action_loco = np.zeros(self.num_actions_loco)
            
            self.cmd_loco = np.array(config["cmd_init_loco"], dtype=np.float32)

            self.policy_loco = torch.jit.load(self.policy_path_loco)

            for _ in range(50):
                with torch.inference_mode():
                    obs_tensor = self.obs_loco.reshape(1, -1)
                    obs_tensor = obs_tensor.astype(np.float32)
                    self.policy_loco(torch.from_numpy(obs_tensor))
            
            print("Locomotion policy initializing ...")
            
            
            
            
    
    def enter(self):
        self.motion_time = 0
        self.counter_step = 0  # 初始化计数器
        ##########################################################
        ################ Locomotion部分 ##########################
        ##########################################################
        self.kps_loco_reorder = np.zeros_like(self.kps_loco)
        self.kds_loco_reorder = np.zeros_like(self.kds_loco)
        self.default_angles_loco_reorder = np.zeros_like(self.default_angles_loco)
        for i in range(len(self.joint2motor_idx)):
            motor_idx = self.joint2motor_idx[i]
            self.kps_loco_reorder[motor_idx] = self.kps_loco[i]
            self.kds_loco_reorder[motor_idx] = self.kds_loco[i]
            self.default_angles_loco_reorder[motor_idx] = self.default_angles_loco[i]
            
        pass
        
        
    def run(self):
        gravity_orientation = self.state_cmd.gravity_ori.reshape(-1)
        qj = self.state_cmd.q.reshape(-1)
        dqj = self.state_cmd.dq.reshape(-1)
        ang_vel = self.state_cmd.ang_vel.reshape(-1)
        
        qj_23dof = qj[self.dof23_index].copy()
        dqj_23dof = dqj[self.dof23_index].copy()
        default_angles_23dof = self.default_angles[self.dof23_index].copy()
        qj_23dof = (qj_23dof - default_angles_23dof) * self.dof_pos_scale
        dqj_23dof = dqj_23dof * self.dof_vel_scale
        ang_vel = ang_vel * self.ang_vel_scale
        
        self.ang_vel_buf = np.concatenate((ang_vel, self.ang_vel_buf[:-3]), axis=-1, dtype=np.float32)
        self.proj_g_buf = np.concatenate((gravity_orientation, self.proj_g_buf[:-3] ), axis=-1, dtype=np.float32)
        self.dof_pos_buf = np.concatenate((qj_23dof, self.dof_pos_buf[:-23] ), axis=-1, dtype=np.float32)
        self.dof_vel_buf = np.concatenate((dqj_23dof, self.dof_vel_buf[:-23] ), axis=-1, dtype=np.float32)
        self.action_buf = np.concatenate((self.action, self.action_buf[:-23] ), axis=-1, dtype=np.float32)
        self.ref_motion_phase_buf = np.concatenate((np.array([min(self.ref_motion_phase,1.0)]), self.ref_motion_phase_buf[:-1] ), axis=-1, dtype=np.float32)
        
        mimic_history_obs_buf = np.concatenate((self.action_buf, 
                                                self.ang_vel_buf, 
                                                self.dof_pos_buf, 
                                                self.dof_vel_buf, 
                                                self.proj_g_buf, 
                                                self.ref_motion_phase_buf
                                                ), 
                                                axis=-1, dtype=np.float32)
        
        mimic_obs_buf = np.concatenate((self.action,
                                        ang_vel,
                                        qj_23dof,
                                        dqj_23dof,
                                        mimic_history_obs_buf,
                                        gravity_orientation,
                                        np.array([min(self.ref_motion_phase,1.0)])
                                        ),
                                        axis=-1, dtype=np.float32)
        
        mimic_obs_tensor = torch.from_numpy(mimic_obs_buf).unsqueeze(0).cpu().numpy()
        self.action = np.squeeze(self.ort_session.run(None, {self.input_name: mimic_obs_tensor})[0])
        self.action = np.clip(self.action, -10., 10.)
        
        
        target_dof_pos = np.zeros(29)
        target_dof_pos[:15] = self.action[:15] * self.action_scale + self.default_angles[:15]
        target_dof_pos[15:19] = self.action[15:19] * self.action_scale + self.default_angles[15:19]
        target_dof_pos[22:26] = self.action[19:] * self.action_scale + self.default_angles[22:26]
        
        target_dof_pos[19:22] = self.default_angles[19:22]
        target_dof_pos[26:29] = self.default_angles[26:29]
        
        ##########################################################
        ################ Locomotion部分 ##########################
        ##########################################################
        self.gravity_orientation_loco = self.state_cmd.gravity_ori
        self.qj_loco = self.state_cmd.q.copy()
        self.dqj_loco = self.state_cmd.dq.copy()
        self.ang_vel_loco = self.state_cmd.ang_vel.copy()
        joycmd = self.state_cmd.vel_cmd.copy()
        self.cmd_loco = scale_values(joycmd, [self.range_velx_loco, self.range_vely_loco, self.range_velz_loco])
        
        for i in range(len(self.joint2motor_idx)):
            self.qj_obs_loco[i] = self.qj_loco[self.joint2motor_idx[i]]
            self.dqj_obs_loco[i] = self.dqj_loco[self.joint2motor_idx[i]]
            
        self.qj_obs_loco = (self.qj_obs_loco - self.default_angles_loco) * self.dof_pos_scale_loco
        self.dqj_obs_loco = self.dqj_obs_loco * self.dof_vel_scale_loco
        self.ang_vel_loco = self.ang_vel_loco * self.ang_vel_scale_loco
        self.cmd_loco = self.cmd_loco * self.cmd_scale_loco
        
        self.obs_loco[:3] = self.ang_vel_loco.copy()
        self.obs_loco[3:6] = self.gravity_orientation_loco.copy()
        self.obs_loco[6:9] = self.cmd_loco.copy()
        self.obs_loco[9: 9 + self.num_actions_loco] = self.qj_obs_loco.copy()
        self.obs_loco[9 + self.num_actions_loco: 9 + self.num_actions_loco * 2] = self.dqj_obs_loco.copy()
        self.obs_loco[9 + self.num_actions_loco * 2: 9 + self.num_actions_loco * 3] = self.action_loco.copy()
        
        obs_tensor = self.obs_loco.reshape(1, -1)
        obs_tensor = obs_tensor.astype(np.float32)
        self.action_loco = self.policy_loco(torch.from_numpy(obs_tensor).clip(-100, 100)).clip(-100, 100).detach().numpy().squeeze()
        loco_action = self.action_loco * self.action_scale_loco + self.default_angles_loco
        action_reorder = loco_action.copy()
        for i in range(len(self.joint2motor_idx)):
            motor_idx = self.joint2motor_idx[i]
            action_reorder[motor_idx] = loco_action[i]

        # 计算alpha：从0到1的指数过渡，transition_time秒内完成
        transition_time = 0.8  # 过渡时间
        current_time = self.counter_step * 0.02  # 使用counter_step计算当前时间
        if current_time <= transition_time:
            # 指数过渡：从0到1，使用指数函数实现平滑过渡
            # alpha = 1.0 - np.exp(-3.0 * current_time / transition_time)  # 3.0是过渡速度参数
            # 线性过渡：从0到1，使用线性函数实现平滑过渡
            alpha = current_time / transition_time  # 线性插值
        else:
            alpha = 1.0  # 过渡完成后保持为1

        assert len(action_reorder) == len(target_dof_pos)
        
        self.policy_output.actions = (1 - alpha) * action_reorder.copy() + alpha * target_dof_pos
        self.policy_output.kps = (1 - alpha) * self.kps_loco_reorder.copy() + alpha * self.kps
        self.policy_output.kds = (1 - alpha) * self.kds_loco_reorder.copy() + alpha * self.kds
        
        
        # update motion phase
        self.counter_step += 1
        motion_time = self.counter_step * 0.02
        self.ref_motion_phase = motion_time / self.motion_length
        motion_time = min(motion_time, self.motion_length)
        print(progress_bar(motion_time, self.motion_length), end="", flush=True)
    
    def exit(self):
        self.action = np.zeros(23, dtype=np.float32)
        self.action_buf = np.zeros(23 * self.history_length, dtype=np.float32)
        self.ref_motion_phase = 0.
        self.ref_motion_phase_buf = np.zeros(1 * self.history_length, dtype=np.float32)
        self.motion_time = 0
        self.counter_step = 0
        print()

    
    def checkChange(self):
        if(self.state_cmd.skill_cmd == FSMCommand.LOCO):
            self.state_cmd.skill_cmd = FSMCommand.INVALID
            return FSMStateName.SKILL_COOLDOWN
        elif(self.state_cmd.skill_cmd == FSMCommand.PASSIVE):
            self.state_cmd.skill_cmd = FSMCommand.INVALID
            return FSMStateName.PASSIVE
        elif(self.state_cmd.skill_cmd == FSMCommand.POS_RESET):
            self.state_cmd.skill_cmd = FSMCommand.INVALID
            return FSMStateName.FIXEDPOSE
        else:
            self.state_cmd.skill_cmd = FSMCommand.INVALID
            return FSMStateName.SKILL_KungFu