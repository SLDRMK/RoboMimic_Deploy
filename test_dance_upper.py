#!/usr/bin/env python3
"""
测试DanceUpper策略的舞蹈数据加载和S型插值功能
"""

import sys
from pathlib import Path
sys.path.append(str(Path(__file__).parent.absolute()))

import json
import numpy as np
import math
import os

def s_curve_interpolation(t, t_start, t_end, start_pos, end_pos):
    """
    S型插值函数
    """
    if t <= t_start:
        return start_pos
    elif t >= t_end:
        return end_pos
    
    # 计算插值进度 (0-1)
    progress = (t - t_start) / (t_end - t_start)
    
    # S型插值: 使用sigmoid函数
    x = (progress - 0.5) * 12  # 映射到[-6, 6]
    s_curve = 1.0 / (1.0 + math.exp(-x))
    
    # 线性插值
    return start_pos + s_curve * (end_pos - start_pos)

def test_dance_data_loading():
    """测试舞蹈数据加载"""
    print("=== 测试舞蹈数据加载 ===")
    
    # 加载舞蹈数据
    dance_data_path = "舞蹈数据_小苹果_2025-07-31.json"
    with open(dance_data_path, "r", encoding="utf-8") as f:
        dance_data = json.load(f)
    
    print(f"舞蹈名称: {dance_data['name']}")
    print(f"姿态数量: {len(dance_data['poses'])}")
    print(f"编舞序列长度: {len(dance_data['choreography']['sequence'])}")
    
    # 显示前几个姿态的信息
    poses = dance_data["poses"]
    choreography = dance_data["choreography"]["sequence"]
    
    print("\n前5个姿态信息:")
    for i in range(min(5, len(choreography))):
        seq = choreography[i]
        pose_name = seq["pose_name"]
        transition_time = seq["transition_time"]
        hold_time = seq["hold_time"]
        
        # 找到对应的姿态数据
        pose_data = None
        for pose in poses:
            if pose["name"] == pose_name:
                pose_data = pose
                break
        
        if pose_data:
            positions = pose_data["positions"]
            print(f"姿态 {pose_name}: 过渡时间={transition_time}s, 保持时间={hold_time}s")
            print(f"  关节数量: {len(positions)}")
            print(f"  前14个关节: {positions[:14]}")
            print()

def test_s_curve_interpolation():
    """测试S型插值"""
    print("=== 测试S型插值 ===")
    
    # 测试简单的线性插值
    t_start = 0.0
    t_end = 1.0
    start_pos = 0.0
    end_pos = 1.0
    
    print("S型插值测试 (0.0 -> 1.0):")
    for t in np.linspace(0, 1, 11):
        result = s_curve_interpolation(t, t_start, t_end, start_pos, end_pos)
        print(f"t={t:.1f}: {result:.4f}")
    
    # 测试向量插值
    print("\n向量插值测试:")
    start_vec = np.array([0.0, 0.0, 0.0])
    end_vec = np.array([1.0, 2.0, 3.0])
    
    for t in np.linspace(0, 1, 6):
        result = s_curve_interpolation(t, t_start, t_end, start_vec, end_vec)
        print(f"t={t:.2f}: {result}")

def test_pose_sequence():
    """测试姿态序列"""
    print("\n=== 测试姿态序列 ===")
    
    # 加载舞蹈数据
    dance_data_path = "舞蹈数据_小苹果_2025-07-31.json"
    with open(dance_data_path, "r", encoding="utf-8") as f:
        dance_data = json.load(f)
    
    poses = dance_data["poses"]
    choreography = dance_data["choreography"]["sequence"]
    
    # 模拟时间推进
    current_time = 0.0
    current_pose_idx = 0
    pose_start_time = 0.0
    
    print("模拟前3个姿态的时间序列:")
    
    for step in range(50):  # 模拟50个控制步
        current_time = step * 0.02  # 20ms控制周期
        
        if current_pose_idx < len(choreography):
            current_sequence = choreography[current_pose_idx]
            pose_name = current_sequence["pose_name"]
            transition_time = current_sequence["transition_time"]
            hold_time = current_sequence["hold_time"]
            pose_duration = transition_time + hold_time
            
            # 找到对应的姿态数据
            pose_data = None
            for pose in poses:
                if pose["name"] == pose_name:
                    pose_data = pose
                    break
            
            if pose_data:
                target_pos = np.array(pose_data["positions"][:14])
                
                if step % 10 == 0:  # 每10步打印一次
                    print(f"时间 {current_time:.2f}s: 姿态 {pose_name}, 目标位置[0]={target_pos[0]:.3f}")
            
            # 检查是否需要切换到下一个姿态
            if current_time >= pose_start_time + pose_duration:
                current_pose_idx += 1
                if current_pose_idx < len(choreography):
                    pose_start_time = current_time
                    print(f"切换到姿态 {current_pose_idx + 1}")

if __name__ == "__main__":
    test_dance_data_loading()
    test_s_curve_interpolation()
    test_pose_sequence()
    print("\n=== 测试完成 ===") 