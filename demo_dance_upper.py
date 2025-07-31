#!/usr/bin/env python3
"""
演示上肢舞蹈功能
"""

import sys
from pathlib import Path
sys.path.append(str(Path(__file__).parent.absolute()))

import numpy as np
import time
from common.ctrlcomp import StateAndCmd, PolicyOutput
from policy.dance_upper.DanceUpper import DanceUpper

def demo_dance_upper():
    """演示上肢舞蹈功能"""
    print("=== 上肢舞蹈功能演示 ===")
    
    # 创建模拟的状态和输出对象
    num_joints = 29
    state_cmd = StateAndCmd(num_joints)
    policy_output = PolicyOutput(num_joints)
    
    # 初始化一些模拟的关节位置
    state_cmd.q = np.random.randn(num_joints) * 0.1  # 随机初始位置
    state_cmd.dq = np.zeros(num_joints)  # 零速度
    state_cmd.gravity_ori = np.array([0.0, 0.0, 1.0])  # 重力方向
    state_cmd.ang_vel = np.zeros(3)  # 零角速度
    
    # 创建DanceUpper策略
    print("初始化DanceUpper策略...")
    dance_upper = DanceUpper(state_cmd, policy_output)
    
    # 进入舞蹈模式
    print("进入舞蹈模式...")
    dance_upper.enter()
    
    # 模拟舞蹈执行
    print("开始舞蹈执行...")
    print("时间(s) | 当前姿态 | 上肢目标位置[0] | 下肢目标位置[0]")
    print("-" * 60)
    
    for step in range(100):  # 模拟100个控制步
        # 更新状态（模拟机器人状态变化）
        state_cmd.q += np.random.randn(num_joints) * 0.01  # 添加一些噪声
        
        # 运行舞蹈策略
        dance_upper.run()
        
        # 每10步打印一次状态
        if step % 10 == 0:
            current_time = step * 0.02  # 20ms控制周期
            current_pose_idx = dance_upper.current_pose_idx
            
            if current_pose_idx < len(dance_upper.choreography):
                pose_name = dance_upper.choreography[current_pose_idx]["pose_name"]
            else:
                pose_name = "END"
            
            upper_target = policy_output.actions[dance_upper.upper_body_motor_idx[0]]
            lower_target = policy_output.actions[dance_upper.lower_body_motor_idx[0]]
            
            print(f"{current_time:6.2f} | {pose_name:>8} | {upper_target:>14.3f} | {lower_target:>14.3f}")
    
    print("\n舞蹈演示完成！")
    print("\n功能特点:")
    print("1. 下肢由skill_cooldown策略控制，保持平衡")
    print("2. 上肢执行小苹果舞蹈动作")
    print("3. 使用S型插值实现平滑的姿态过渡")
    print("4. 支持89个舞蹈姿态的完整序列")

if __name__ == "__main__":
    demo_dance_upper() 