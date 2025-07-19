import sys
from pathlib import Path
sys.path.append(str(Path(__file__).parent.parent.absolute()))

from common.path_config import PROJECT_ROOT

import time
import mujoco.viewer
import mujoco
import numpy as np
import yaml
import os
import threading
import queue
from common.ctrlcomp import *
from FSM.FSM import *
from common.utils import get_gravity_orientation, FSMCommand

class KeyboardController:
    def __init__(self):
        self.command_queue = queue.Queue()
        self.running = True
        self.current_cmd = "无命令"
        
    def start_keyboard_listener(self):
        """启动键盘监听线程"""
        def keyboard_listener():
            print("键盘控制说明:")
            print("数字键 1-6: 切换不同模式")
            print("  1: 阻尼保护模式 (PASSIVE)")
            print("  2: 位控模式 (POS_RESET)")
            print("  3: 行走模式 (LOCO)")
            print("  4: 舞蹈模式 (SKILL_1)")
            print("  5: 武术模式 (SKILL_2)")
            print("  6: 踢腿模式 (SKILL_3)")
            print("  7: 武术2模式 (SKILL_4)")
            print("  q: 退出程序")
            print("  w/s/a/d: 控制摇杆 (模拟)")
            print("按回车键开始...")
            input()
            
            while self.running:
                try:
                    # 使用input()获取键盘输入
                    key = input("请输入命令 (1-7, q, w/s/a/d): ").strip().lower()
                    
                    if key == 'q':
                        self.running = False
                        break
                    elif key == '1':
                        self.command_queue.put(FSMCommand.PASSIVE)
                        self.current_cmd = "PASSIVE (阻尼保护模式)"
                    elif key == '2':
                        self.command_queue.put(FSMCommand.POS_RESET)
                        self.current_cmd = "POS_RESET (位控模式)"
                    elif key == '3':
                        self.command_queue.put(FSMCommand.LOCO)
                        self.current_cmd = "LOCO (行走模式)"
                    elif key == '4':
                        self.command_queue.put(FSMCommand.SKILL_1)
                        self.current_cmd = "SKILL_1 (舞蹈模式)"
                    elif key == '5':
                        self.command_queue.put(FSMCommand.SKILL_2)
                        self.current_cmd = "SKILL_2 (武术模式)"
                    elif key == '6':
                        self.command_queue.put(FSMCommand.SKILL_3)
                        self.current_cmd = "SKILL_3 (踢腿模式)"
                    elif key == '7':
                        self.command_queue.put(FSMCommand.SKILL_4)
                        self.current_cmd = "SKILL_4 (武术2模式)"
                    elif key in ['w', 's', 'a', 'd']:
                        # 模拟摇杆输入
                        self.current_cmd = f"摇杆控制: {key}"
                    else:
                        print("无效命令，请重新输入")
                        
                except KeyboardInterrupt:
                    self.running = False
                    break
                except EOFError:
                    self.running = False
                    break
        
        # 启动键盘监听线程
        keyboard_thread = threading.Thread(target=keyboard_listener, daemon=True)
        keyboard_thread.start()
    
    def get_command(self):
        """获取命令队列中的命令"""
        try:
            return self.command_queue.get_nowait()
        except queue.Empty:
            return FSMCommand.INVALID
    
    def get_vel_cmd(self):
        """获取速度命令（模拟摇杆）"""
        # 这里可以扩展为从键盘获取速度命令
        return np.array([0.0, 0.0, 0.0], dtype=np.float32)

def print_status_table(keyboard_controller, FSM_controller):
    """打印状态表格"""
    os.system('clear' if os.name == 'posix' else 'cls')
    
    print("=" * 80)
    print("⌨️  键盘控制机器人仿真")
    print("=" * 80)
    print()
    
    print("🎯 当前状态:")
    print("-" * 50)
    print(f"最新命令: {keyboard_controller.current_cmd}")
    print(f"当前模式: {FSM_controller.cur_policy.name_str}")
    
    print()
    print("💡 操作说明:")
    print("-" * 50)
    print("1 → 阻尼保护模式 (PASSIVE)")
    print("2 → 位控模式 (POS_RESET)")
    print("3 → 行走模式 (LOCO)")
    print("4 → 舞蹈模式 (SKILL_1)")
    print("5 → 武术模式 (SKILL_2)")
    print("6 → 踢腿模式 (SKILL_3)")
    print("7 → 武术2模式 (SKILL_4)")
    print("q → 退出程序")
    print("w/s/a/d → 控制摇杆 (模拟)")
    
    print()
    print("=" * 80)
    print("按 Ctrl+C 退出程序")
    print("=" * 80)

def pd_control(target_q, q, kp, target_dq, dq, kd):
    """Calculates torques from position commands"""
    return (target_q - q) * kp + (target_dq - dq) * kd

if __name__ == "__main__":
    current_dir = os.path.dirname(os.path.abspath(__file__))
    mujoco_yaml_path = os.path.join(current_dir, "config", "mujoco.yaml")
    with open(mujoco_yaml_path, "r") as f:
        config = yaml.load(f, Loader=yaml.FullLoader)
        xml_path = os.path.join(PROJECT_ROOT, config["xml_path"])
        simulation_dt = config["simulation_dt"]
        control_decimation = config["control_decimation"]
        
    m = mujoco.MjModel.from_xml_path(xml_path)
    d = mujoco.MjData(m)
    m.opt.timestep = simulation_dt
    mj_per_step_duration = simulation_dt * control_decimation
    num_joints = m.nu
    policy_output_action = np.zeros(num_joints, dtype=np.float32)
    kps = np.zeros(num_joints, dtype=np.float32)
    kds = np.zeros(num_joints, dtype=np.float32)
    sim_counter = 0
    
    state_cmd = StateAndCmd(num_joints)
    policy_output = PolicyOutput(num_joints)
    FSM_controller = FSM(state_cmd, policy_output)
    
    keyboard_controller = KeyboardController()
    keyboard_controller.start_keyboard_listener()
    
    # 初始化显示
    print_status_table(keyboard_controller, FSM_controller)
    
    with mujoco.viewer.launch_passive(m, d) as viewer:
        sim_start_time = time.time()
        last_display_update = time.time()
        
        while viewer.is_running() and keyboard_controller.running:
            try:
                # 获取键盘命令
                cmd = keyboard_controller.get_command()
                if cmd != FSMCommand.INVALID:
                    state_cmd.skill_cmd = cmd
                    print(f"收到命令: {cmd} -> {keyboard_controller.current_cmd}")
                
                # 获取速度命令
                state_cmd.vel_cmd = keyboard_controller.get_vel_cmd()
                
                step_start = time.time()
                
                tau = pd_control(policy_output_action, d.qpos[7:], kps, np.zeros_like(kps), d.qvel[6:], kds)
                d.ctrl[:] = tau
                mujoco.mj_step(m, d)
                sim_counter += 1
                if sim_counter % control_decimation == 0:
                    
                    qj = d.qpos[7:]
                    dqj = d.qvel[6:]
                    quat = d.qpos[3:7]
                    
                    omega = d.qvel[3:6] 
                    gravity_orientation = get_gravity_orientation(quat)
                    
                    state_cmd.q = qj.copy()
                    state_cmd.dq = dqj.copy()
                    state_cmd.gravity_ori = gravity_orientation.copy()
                    state_cmd.ang_vel = omega.copy()
                    
                    FSM_controller.run()
                    policy_output_action = policy_output.actions.copy()
                    kps = policy_output.kps.copy()
                    kds = policy_output.kds.copy()
                
                # 每0.5秒更新一次显示
                if time.time() - last_display_update > 0.5:
                    print_status_table(keyboard_controller, FSM_controller)
                    last_display_update = time.time()
                    
            except ValueError as e:
                print(str(e))
            
            viewer.sync()
            time_until_next_step = m.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)
        
        print("程序退出") 