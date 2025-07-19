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
from common.ctrlcomp import *
from FSM.FSM import *
from common.utils import get_gravity_orientation
from common.joystick import JoyStick, JoystickButton

def print_button_status_table(joystick):
    """打印按键状态表格"""
    os.system('clear' if os.name == 'posix' else 'cls')
    
    # 获取摇杆值
    lx = joystick.get_axis_value(0)
    ly = joystick.get_axis_value(1)
    rx = joystick.get_axis_value(2)
    ry = joystick.get_axis_value(3)
    
    print("=" * 80)
    print("🎮 XBOX手柄实时按键状态监控")
    print("=" * 80)
    print()
    
    # 按钮状态表格
    print("📋 按钮状态:")
    print("-" * 50)
    buttons = [
        ("A", 0, "Cross(×)"),
        ("B", 1, "Circle(○)"),
        ("X", 2, "Square(□)"),
        ("Y", 3, "Triangle(△)"),
        ("L1", 4, "Left Bumper"),
        ("R1", 5, "Right Bumper"),
        ("SELECT", 6, "View/Back"),
        ("START", 7, "Start/Options"),
        ("L3", 8, "Left Stick Press"),
        ("R3", 9, "Right Stick Press"),
        ("HOME", 10, "Home/PS"),
        ("UP", 11, "D-pad Up"),
        ("DOWN", 12, "D-pad Down"),
        ("LEFT", 13, "D-pad Left"),
        ("RIGHT", 14, "D-pad Right")
    ]
    
    for name, button_id, description in buttons:
        status = "🔴 按下" if joystick.is_button_pressed(button_id) else "⚪ 释放"
        print(f"{name:8} | {status:8} | {description}")
    
    print()
    print("🎯 摇杆状态:")
    print("-" * 50)
    print(f"左摇杆 X: {lx:6.3f} | 左摇杆 Y: {ly:6.3f}")
    print(f"右摇杆 X: {rx:6.3f} | 右摇杆 Y: {ry:6.3f}")
    
    print()
    print("🎯 当前FSM命令:")
    print("-" * 50)
    # 这里会在主循环中更新
    print("等待命令...")
    
    print()
    print("💡 操作提示:")
    print("-" * 50)
    print("START     → 位控模式")
    print("SELECT    → 阻尼保护模式")
    print("L3        → 阻尼保护模式")
    print("R1 + A    → 行走模式")
    print("R1 + X    → 舞蹈模式")
    print("R1 + Y    → 武术模式")
    print("R1 + B    → 踢腿模式")
    print("L1 + Y    → 武术2模式")
    print("SELECT    → 退出程序")
    
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
    
    joystick = JoyStick()
    Running = True
    
    # 初始化显示
    print_button_status_table(joystick)
    
    with mujoco.viewer.launch_passive(m, d) as viewer:
        sim_start_time = time.time()
        last_display_update = time.time()
        
        while viewer.is_running() and Running:
            try:
                if(joystick.is_button_pressed(JoystickButton.SELECT)):
                    Running = False

                joystick.update()
                
                # 处理按键命令
                current_cmd = "无命令"
                if joystick.is_button_released(JoystickButton.L3):
                    state_cmd.skill_cmd = FSMCommand.PASSIVE
                    current_cmd = "PASSIVE (L3释放)"
                if joystick.is_button_released(JoystickButton.START):
                    state_cmd.skill_cmd = FSMCommand.POS_RESET
                    current_cmd = "POS_RESET (START释放)"
                if joystick.is_button_released(JoystickButton.A) and joystick.is_button_pressed(JoystickButton.R1):
                    state_cmd.skill_cmd = FSMCommand.LOCO
                    current_cmd = "LOCO (R1+A)"
                if joystick.is_button_released(JoystickButton.X) and joystick.is_button_pressed(JoystickButton.R1):
                    state_cmd.skill_cmd = FSMCommand.SKILL_1
                    current_cmd = "SKILL_1 (R1+X)"
                if joystick.is_button_released(JoystickButton.Y) and joystick.is_button_pressed(JoystickButton.R1):
                    state_cmd.skill_cmd = FSMCommand.SKILL_2
                    current_cmd = "SKILL_2 (R1+Y)"
                if joystick.is_button_released(JoystickButton.B) and joystick.is_button_pressed(JoystickButton.R1):
                    state_cmd.skill_cmd = FSMCommand.SKILL_3
                    current_cmd = "SKILL_3 (R1+B)"
                if joystick.is_button_released(JoystickButton.Y) and joystick.is_button_pressed(JoystickButton.L1):
                    state_cmd.skill_cmd = FSMCommand.SKILL_4
                    current_cmd = "SKILL_4 (L1+Y)"
                
                state_cmd.vel_cmd[0] = -joystick.get_axis_value(1)
                state_cmd.vel_cmd[1] = -joystick.get_axis_value(0)
                state_cmd.vel_cmd[2] = -joystick.get_axis_value(3)
                
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
                
                # 每0.1秒更新一次显示
                if time.time() - last_display_update > 0.1:
                    # 更新显示表格
                    os.system('clear' if os.name == 'posix' else 'cls')
                    
                    # 获取摇杆值
                    lx = joystick.get_axis_value(0)
                    ly = joystick.get_axis_value(1)
                    rx = joystick.get_axis_value(2)
                    ry = joystick.get_axis_value(3)
                    
                    print("=" * 80)
                    print("🎮 XBOX手柄实时按键状态监控")
                    print("=" * 80)
                    print()
                    
                    # 按钮状态表格
                    print("📋 按钮状态:")
                    print("-" * 50)
                    buttons = [
                        ("A", 0, "Cross(×)"),
                        ("B", 1, "Circle(○)"),
                        ("X", 2, "Square(□)"),
                        ("Y", 3, "Triangle(△)"),
                        ("L1", 4, "Left Bumper"),
                        ("R1", 5, "Right Bumper"),
                        ("SELECT", 6, "View/Back"),
                        ("START", 7, "Start/Options"),
                        ("L3", 8, "Left Stick Press"),
                        ("R3", 9, "Right Stick Press"),
                        ("HOME", 10, "Home/PS"),
                        ("UP", 11, "D-pad Up"),
                        ("DOWN", 12, "D-pad Down"),
                        ("LEFT", 13, "D-pad Left"),
                        ("RIGHT", 14, "D-pad Right")
                    ]
                    
                    for name, button_id, description in buttons:
                        status = "🔴 按下" if joystick.is_button_pressed(button_id) else "⚪ 释放"
                        print(f"{name:8} | {status:8} | {description}")
                    
                    print()
                    print("🎯 摇杆状态:")
                    print("-" * 50)
                    print(f"左摇杆 X: {lx:6.3f} | 左摇杆 Y: {ly:6.3f}")
                    print(f"右摇杆 X: {rx:6.3f} | 右摇杆 Y: {ry:6.3f}")
                    
                    print()
                    print("🎯 当前FSM命令:")
                    print("-" * 50)
                    print(f"最新命令: {current_cmd}")
                    print(f"当前状态: {FSM_controller.cur_policy.name_str}")
                    
                    print()
                    print("💡 操作提示:")
                    print("-" * 50)
                    print("START     → 位控模式")
                    print("SELECT    → 阻尼保护模式")
                    print("L3        → 阻尼保护模式")
                    print("R1 + A    → 行走模式")
                    print("R1 + X    → 舞蹈模式")
                    print("R1 + Y    → 武术模式")
                    print("R1 + B    → 踢腿模式")
                    print("L1 + Y    → 武术2模式")
                    print("SELECT    → 退出程序")
                    
                    print()
                    print("=" * 80)
                    print("按 Ctrl+C 退出程序")
                    print("=" * 80)
                    
                    last_display_update = time.time()
                    
            except ValueError as e:
                print(str(e))
            
            viewer.sync()
            time_until_next_step = m.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)
        