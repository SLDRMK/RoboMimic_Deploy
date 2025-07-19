import sys
from pathlib import Path
sys.path.append(str(Path(__file__).parent.parent.absolute()))

import pygame
import time
import os

def test_joystick_mapping():
    """测试手柄按键映射"""
    pygame.init()
    pygame.joystick.init()
    
    joystick_count = pygame.joystick.get_count()
    if joystick_count == 0:
        print("❌ 没有检测到手柄！")
        return
    
    print(f"✅ 检测到 {joystick_count} 个手柄")
    
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    
    print(f"手柄名称: {joystick.get_name()}")
    print(f"按钮数量: {joystick.get_numbuttons()}")
    print(f"摇杆数量: {joystick.get_numaxes()}")
    print(f"D-pad数量: {joystick.get_numhats()}")
    print()
    
    print("🎮 手柄按键映射测试")
    print("=" * 60)
    print("请按下手柄上的各个按键，程序会显示对应的按钮ID")
    print("按 Ctrl+C 退出测试")
    print("=" * 60)
    
    # 记录按键状态
    button_states = [False] * joystick.get_numbuttons()
    axis_states = [0.0] * joystick.get_numaxes()
    hat_states = [(0, 0)] * joystick.get_numhats()
    
    try:
        while True:
            pygame.event.pump()
            
            # 检测按钮状态变化
            for i in range(joystick.get_numbuttons()):
                current_state = joystick.get_button(i) == 1
                if current_state and not button_states[i]:
                    print(f"🔴 按钮 {i} 被按下")
                elif not current_state and button_states[i]:
                    print(f"⚪ 按钮 {i} 被释放")
                button_states[i] = current_state
            
            # 检测摇杆状态变化
            for i in range(joystick.get_numaxes()):
                current_value = joystick.get_axis(i)
                if abs(current_value - axis_states[i]) > 0.1:  # 阈值避免噪声
                    print(f"🎯 摇杆 {i}: {current_value:.3f}")
                axis_states[i] = current_value
            
            # 检测D-pad状态变化
            for i in range(joystick.get_numhats()):
                current_hat = joystick.get_hat(i)
                if current_hat != hat_states[i]:
                    print(f"⬆️  D-pad {i}: {current_hat}")
                hat_states[i] = current_hat
            
            time.sleep(0.01)  # 10ms polling rate
            
    except KeyboardInterrupt:
        print("\n测试结束")
    
    pygame.quit()

def suggest_mapping():
    """根据测试结果建议映射"""
    print("\n📋 建议的按键映射:")
    print("=" * 60)
    print("基于常见的Xbox/PS手柄布局，建议使用以下映射:")
    print()
    print("@unique")
    print("class JoystickButton(IntEnum):")
    print("    # Xbox手柄映射")
    print("    A = 0        # A按钮 (确认)")
    print("    B = 1        # B按钮 (取消)")
    print("    X = 2        # X按钮 (方形)")
    print("    Y = 3        # Y按钮 (三角形)")
    print("    L1 = 4       # 左肩键")
    print("    R1 = 5       # 右肩键")
    print("    SELECT = 6   # Select/View按钮")
    print("    START = 7    # Start按钮")
    print("    L3 = 8       # 左摇杆按下")
    print("    R3 = 9       # 右摇杆按下")
    print("    HOME = 10    # Home/PS按钮")
    print("    UP = 11      # D-pad上")
    print("    DOWN = 12    # D-pad下")
    print("    LEFT = 13    # D-pad左")
    print("    RIGHT = 14   # D-pad右")
    print()
    print("摇杆轴映射:")
    print("    轴 0: 左摇杆 X")
    print("    轴 1: 左摇杆 Y")
    print("    轴 2: 右摇杆 X")
    print("    轴 3: 右摇杆 Y")
    print("    轴 4: 左扳机")
    print("    轴 5: 右扳机")

if __name__ == "__main__":
    print("🎮 手柄按键映射测试工具")
    print("=" * 60)
    
    # 先显示建议的映射
    suggest_mapping()
    print()
    
    # 询问是否进行测试
    response = input("是否要测试实际的手柄映射？(y/n): ").strip().lower()
    if response == 'y':
        test_joystick_mapping()
    else:
        print("测试已取消") 