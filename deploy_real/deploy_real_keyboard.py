import sys
from pathlib import Path
sys.path.append(str(Path(__file__).parent.parent.absolute()))

from common.path_config import PROJECT_ROOT
from common.ctrlcomp import *
from FSM.FSM import *
from typing import Union
import numpy as np
import time
import os
import yaml
import threading
import queue
import select
import tty
import termios

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_, unitree_hg_msg_dds__LowState_
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_, unitree_go_msg_dds__LowState_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_ as LowCmdHG
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_ as LowCmdGo
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_ as LowStateHG
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_ as LowStateGo
from unitree_sdk2py.utils.crc import CRC

from common.command_helper import create_damping_cmd, create_zero_cmd, init_cmd_hg, init_cmd_go, MotorMode
from common.rotation_helper import get_gravity_orientation_real, transform_imu_data
from config import Config

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
            print("  1: PASSIVE  2: POS_RESET  3: LOCO  4: SKILL_1  5: SKILL_2  6: SKILL_3  7: SKILL_4")
            print("  q: 退出程序")
            print("  w/s/a/d: 加速控制 (前进/后退/左移/右移) +0.2")
            print("  j/l: 转身加速 (左转/右转) +0.2")
            print("  z: 停止走路")
            print("按任意键开始...")
            old_settings = termios.tcgetattr(sys.stdin)
            try:
                tty.setcbreak(sys.stdin.fileno())
                while self.running:
                    try:
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
        keyboard_thread = threading.Thread(target=keyboard_listener, daemon=True)
        keyboard_thread.start()
    def process_key(self, key):
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
            self.current_cmd = "SKILL_4 (武术2模式)"
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
        elif key == ' ':
            self.vel_cmd = np.array([0.0, 0.0, 0.0], dtype=np.float32)
            self.current_cmd = "摇杆重置"
        elif key == 'z':
            self.vel_cmd = np.array([0.0, 0.0, 0.0], dtype=np.float32)
            self.current_cmd = "停止走路"
    def get_command(self):
        try:
            return self.command_queue.get_nowait()
        except queue.Empty:
            return FSMCommand.INVALID
    def get_vel_cmd(self):
        return self.vel_cmd.copy()

def print_status_table(keyboard_controller, controller):
    os.system('clear' if os.name == 'posix' else 'cls')
    print("=" * 80)
    print("⌨️  实时键盘控制机器人")
    print("=" * 80)
    print()
    print("🎯 当前状态:")
    print("-" * 50)
    print(f"当前关节角:")
    for i in range(29):
        print(f"关节{i}: {controller.low_state.motor_state[i].q}")
    print(f"最新命令: {keyboard_controller.current_cmd}")
    print(f"当前模式: {controller.FSM_controller.cur_policy.name_str}")
    print(f"速度命令: [{keyboard_controller.vel_cmd[0]:.2f}, {keyboard_controller.vel_cmd[1]:.2f}, {keyboard_controller.vel_cmd[2]:.2f}]")
    print()
    print("💡 操作说明:")
    print("-" * 50)
    print("1 → PASSIVE 2 → POS_RESET 3 → LOCO 4 → SKILL_1 5 → SKILL_2 6 → SKILL_3 7 → SKILL_4")
    print("w/s/a/d → 加速控制 (前进/后退/左移/右移) +0.2")
    print("j/l → 转身加速 (左转/右转) +0.2")
    print("z → 停止走路")
    print("空格键 → 重置摇杆")
    print("q → 退出程序")
    print()
    print("=" * 80)
    print("按 Ctrl+C 退出程序")
    print("=" * 80)

class Controller:
    def __init__(self, config: Config):
        self.config = config
        self.num_joints = config.num_joints
        self.control_dt = config.control_dt
        self.low_cmd = unitree_hg_msg_dds__LowCmd_()
        self.low_state = unitree_hg_msg_dds__LowState_()
        self.mode_pr_ = MotorMode.PR
        self.mode_machine_ = 0
        self.lowcmd_publisher_ = ChannelPublisher(config.lowcmd_topic, LowCmdHG)
        self.lowcmd_publisher_.Init()
        self.lowstate_subscriber = ChannelSubscriber(config.lowstate_topic, LowStateHG)
        self.lowstate_subscriber.Init(self.LowStateHgHandler, 10)
        self.wait_for_low_state()
        init_cmd_hg(self.low_cmd, self.mode_machine_, self.mode_pr_)
        self.policy_output_action = np.zeros(self.num_joints, dtype=np.float32)
        self.kps = np.zeros(self.num_joints, dtype=np.float32)
        self.kds = np.zeros(self.num_joints, dtype=np.float32)
        self.qj = np.zeros(self.num_joints, dtype=np.float32)
        self.dqj = np.zeros(self.num_joints, dtype=np.float32)
        self.quat = np.zeros(4, dtype=np.float32)
        self.ang_vel = np.zeros(3, dtype=np.float32)
        self.gravity_orientation = np.array([0,0,-1], dtype=np.float32)
        self.state_cmd = StateAndCmd(self.num_joints)
        self.policy_output = PolicyOutput(self.num_joints)
        self.FSM_controller = FSM(self.state_cmd, self.policy_output)
        self.running = True
        self.counter_over_time = 0
    def LowStateHgHandler(self, msg: LowStateHG):
        self.low_state = msg
        self.mode_machine_ = self.low_state.mode_machine
    def send_cmd(self, cmd: Union[LowCmdGo, LowCmdHG]):
        cmd.crc = CRC().Crc(cmd)
        self.lowcmd_publisher_.Write(cmd)
    def wait_for_low_state(self):
        while self.low_state.tick == 0:
            time.sleep(self.config.control_dt)
        print("Successfully connected to the robot.")
    def zero_torque_state(self):
        print("Enter zero torque state.")
        print("Waiting for the start signal...")
        while True:
            create_zero_cmd(self.low_cmd)
            self.send_cmd(self.low_cmd)
            time.sleep(self.config.control_dt)
    def run(self, keyboard_controller):
        try:
            loop_start_time = time.time()
            # 键盘命令
            cmd = keyboard_controller.get_command()
            if cmd != FSMCommand.INVALID:
                self.state_cmd.skill_cmd = cmd
                print(f"收到命令: {cmd} -> {keyboard_controller.current_cmd}")
            self.state_cmd.vel_cmd = keyboard_controller.get_vel_cmd()
            for i in range(self.num_joints):
                self.qj[i] = self.low_state.motor_state[i].q
                self.dqj[i] = self.low_state.motor_state[i].dq
            quat = self.low_state.imu_state.quaternion
            ang_vel = np.array([self.low_state.imu_state.gyroscope], dtype=np.float32)
            gravity_orientation = get_gravity_orientation_real(quat)
            self.state_cmd.q = self.qj.copy()
            self.state_cmd.dq = self.dqj.copy()
            self.state_cmd.gravity_ori = gravity_orientation.copy()
            self.state_cmd.ang_vel = ang_vel.copy()
            self.FSM_controller.run()
            policy_output_action = self.policy_output.actions.copy()
            kps = self.policy_output.kps.copy()
            kds = self.policy_output.kds.copy()
            for i in range(self.num_joints):
                self.low_cmd.motor_cmd[i].q = policy_output_action[i]
                self.low_cmd.motor_cmd[i].qd = 0
                self.low_cmd.motor_cmd[i].kp = kps[i]
                self.low_cmd.motor_cmd[i].kd = kds[i]
                self.low_cmd.motor_cmd[i].tau = 0
            self.send_cmd(self.low_cmd)
            loop_end_time = time.time()
            delta_time = loop_end_time - loop_start_time
            if(delta_time < self.control_dt):
                time.sleep(self.control_dt - delta_time)
                self.counter_over_time = 0
            else:
                print("control loop over time.")
                self.counter_over_time += 1
        except ValueError as e:
            print(str(e))
            pass
if __name__ == "__main__":
    config = Config()
    ChannelFactoryInitialize(0, config.net)
    controller = Controller(config)
    keyboard_controller = RealtimeKeyboardController()
    keyboard_controller.start_keyboard_listener()
    print_status_table(keyboard_controller, controller)
    while keyboard_controller.running:
        try:
            controller.run(keyboard_controller)
            print_status_table(keyboard_controller, controller)
        except KeyboardInterrupt:
            break
    create_damping_cmd(controller.low_cmd)
    controller.send_cmd(controller.low_cmd)
    print("Exit") 