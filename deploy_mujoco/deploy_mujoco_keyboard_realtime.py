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
import select  # Linux/Mac
import tty
import termios
from common.ctrlcomp import *
from FSM.FSM import *
from common.utils import get_gravity_orientation, FSMCommand

class RealtimeKeyboardController:
    def __init__(self):
        self.command_queue = queue.Queue()
        self.running = True
        self.current_cmd = "无命令"
        self.vel_cmd = np.array([0.0, 0.0, 0.0], dtype=np.float32)
        self.key_states = {}
        
    def start_keyboard_listener(self):
        """启动实时键盘监听线程"""
        def keyboard_listener():
            print("实时键盘控制说明:")
            print("数字键 1-7: 切换不同模式")
            print("  1: 阻尼保护模式 (PASSIVE)")
            print("  2: 位控模式 (POS_RESET)")
            print("  3: 行走模式 (LOCO)")
            print("  4: 舞蹈模式 (SKILL_1)")
            print("  5: 武术模式 (SKILL_2)")
            print("  6: 踢腿模式 (SKILL_3)")
            print("  7: 上肢舞蹈模式 (SKILL_4)")
            print("  q: 退出程序")
            print("  w/s/a/d: 加速控制 (前进/后退/左移/右移) +0.2")
            print("  j/l: 转身加速 (左转/右转) +0.2")
            print("  z: 停止走路")
            print("按任意键开始...")
            
            # 设置非阻塞输入
            old_settings = termios.tcgetattr(sys.stdin)
            try:
                tty.setcbreak(sys.stdin.fileno())
                
                while self.running:
                    try:
                        # 检测按键输入
                        if select.select([sys.stdin], [], [], 0.01)[0]:
                            key = sys.stdin.read(1).lower()
                            self.process_key(key)
                        
                    except KeyboardInterrupt:
                        self.running = False
                        break
                    except Exception as e:
                        print(f"键盘监听错误: {e}")
                        break
            finally:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        
        # 启动键盘监听线程
        keyboard_thread = threading.Thread(target=keyboard_listener, daemon=True)
        keyboard_thread.start()
    
    def process_key(self, key):
        """处理按键输入"""
        if key == 'q':
            self.running = False
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
            self.current_cmd = "SKILL_4 (上肢舞蹈模式)"
        elif key == 'w':
            self.vel_cmd[0] += 0.2  # 前进加速
            self.vel_cmd[0] = min(self.vel_cmd[0], 2.0)  # 限制最大速度
            self.current_cmd = f"前进加速: {self.vel_cmd[0]:.1f}"
        elif key == 's':
            self.vel_cmd[0] -= 0.2  # 后退加速
            self.vel_cmd[0] = max(self.vel_cmd[0], -2.0)  # 限制最大速度
            self.current_cmd = f"后退加速: {self.vel_cmd[0]:.1f}"
        elif key == 'a':
            self.vel_cmd[1] += 0.2  # 左移加速
            self.vel_cmd[1] = min(self.vel_cmd[1], 2.0)  # 限制最大速度
            self.current_cmd = f"左移加速: {self.vel_cmd[1]:.1f}"
        elif key == 'd':
            self.vel_cmd[1] -= 0.2  # 右移加速
            self.vel_cmd[1] = max(self.vel_cmd[1], -2.0)  # 限制最大速度
            self.current_cmd = f"右移加速: {self.vel_cmd[1]:.1f}"
        elif key == 'j':
            self.vel_cmd[2] += 0.2  # 左转加速
            self.vel_cmd[2] = min(self.vel_cmd[2], 2.0)  # 限制最大角速度
            self.current_cmd = f"左转加速: {self.vel_cmd[2]:.1f}"
        elif key == 'l':
            self.vel_cmd[2] -= 0.2  # 右转加速
            self.vel_cmd[2] = max(self.vel_cmd[2], -2.0)  # 限制最大角速度
            self.current_cmd = f"右转加速: {self.vel_cmd[2]:.1f}"
        elif key == ' ':  # 空格键重置速度
            self.vel_cmd = np.array([0.0, 0.0, 0.0], dtype=np.float32)
            self.current_cmd = "摇杆重置"
        elif key == 'z':
            self.vel_cmd = np.array([0.0, 0.0, 0.0], dtype=np.float32)
            self.current_cmd = "停止走路"
    
    def get_command(self):
        """获取命令队列中的命令"""
        try:
            return self.command_queue.get_nowait()
        except queue.Empty:
            return FSMCommand.INVALID
    
    def get_vel_cmd(self):
        """获取速度命令"""
        return self.vel_cmd.copy()

def print_status_table(keyboard_controller, FSM_controller, qpos=None):
    """打印状态表格"""
    os.system('clear' if os.name == 'posix' else 'cls')
    
    print("=" * 80)
    print("⌨️  实时键盘控制机器人仿真")
    print("=" * 80)
    print()
    
    print("🎯 当前状态:")
    print("-" * 50)
    print(f"最新命令: {keyboard_controller.current_cmd}")
    print(f"当前模式: {FSM_controller.cur_policy.name_str}")
    print(f"速度命令: [{keyboard_controller.vel_cmd[0]:.2f}, {keyboard_controller.vel_cmd[1]:.2f}, {keyboard_controller.vel_cmd[2]:.2f}]")
    
    # 显示机器人关节位置状态
    if qpos is not None:
        print(f"机器人位置: [{qpos[0]:.3f}, {qpos[1]:.3f}, {qpos[2]:.3f}]")
        if len(qpos) > 7:
            quat = qpos[3:7]  # 根节点姿态四元数
            print(f"根节点姿态四元数: [{quat[0]:.3f}, {quat[1]:.3f}, {quat[2]:.3f}, {quat[3]:.3f}]")
            joint_pos = qpos[7:]  # 关节位置（跳过前7个位置和姿态）
            print(f"关节位置: [{', '.join([f'{pos:.3f}' for pos in joint_pos])}]")
    
    print()
    print("💡 操作说明:")
    print("-" * 50)
    print("1 → 阻尼保护模式 (PASSIVE)")
    print("2 → 位控模式 (POS_RESET)")
    print("3 → 行走模式 (LOCO)")
    print("4 → 舞蹈模式 (SKILL_1)")
    print("5 → 武术模式 (SKILL_2)")
    print("6 → 踢腿模式 (SKILL_3)")
    print("7 → 上肢舞蹈模式 (SKILL_4)")
    print("w/s/a/d → 加速控制 (前进/后退/左移/右移) +0.2")
    print("j/l → 转身加速 (左转/右转) +0.2")
    print("z → 停止走路")
    print("空格键 → 重置摇杆")
    print("q → 退出程序")
    
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
    
    keyboard_controller = RealtimeKeyboardController()
    keyboard_controller.start_keyboard_listener()
    
    # 初始化显示
    print_status_table(keyboard_controller, FSM_controller, d.qpos)
    
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
                
                # 每0.2秒更新一次显示
                if time.time() - last_display_update > 0.2:
                    print_status_table(keyboard_controller, FSM_controller, d.qpos)
                    last_display_update = time.time()
                    
            except ValueError as e:
                print(str(e))
            
            viewer.sync()
            time_until_next_step = m.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)
        
        print("程序退出") 