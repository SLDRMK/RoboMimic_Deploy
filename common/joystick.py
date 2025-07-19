from common.path_config import PROJECT_ROOT

import pygame
from pygame.locals import *
from enum import IntEnum, unique

@unique
class JoystickButton(IntEnum):
    # 根据实际手柄映射调整
    A = 0      # 确认按钮
    B = 1      # 取消按钮
    X = 3      # 方形按钮
    Y = 4      # 三角形按钮
    L1 = 6     # 左肩键
    R1 = 7     # 右肩键
    SELECT = 15  # Select/View按钮
    START = 11 # Start按钮
    L3 = 8     # 左摇杆按下 (保持原值)
    R3 = 9     # 右摇杆按下 (保持原值)
    HOME = 10  # Home按钮 (保持原值)
    UP = 2     # D-pad上 (重新分配ID避免冲突)
    DOWN = 12  # D-pad下
    LEFT = 13  # D-pad左
    RIGHT = 14 # D-pad右

class JoyStick:
    def __init__(self):
        pygame.init()
        pygame.joystick.init()
        
        joystick_count = pygame.joystick.get_count()
        if joystick_count == 0:
            raise RuntimeError("No joystick connected!")
        
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        
        self.button_count = self.joystick.get_numbuttons()
        self.button_states = [False] * self.button_count  
        self.button_pressed = [False] * self.button_count  
        self.button_released = [False] * self.button_count 

        self.axis_count = self.joystick.get_numaxes()
        self.axis_states = [0.0] * self.axis_count
        
        self.hat_count = self.joystick.get_numhats()
        self.hat_states = [(0, 0)] * self.hat_count
        
        
    def update(self):
        """update joystick state"""
        pygame.event.pump()  
        
        self.button_released = [False] * self.button_count
        
        for i in range(self.button_count):
            current_state = self.joystick.get_button(i) == 1
            if self.button_states[i] and not current_state:
                self.button_released[i] = True
            self.button_states[i] = current_state

        for i in range(self.axis_count):
            self.axis_states[i] = self.joystick.get_axis(i)
        
        for i in range(self.hat_count):
            self.hat_states[i] = self.joystick.get_hat(i)

    def is_button_pressed(self, button_id):
        """detect button pressed"""
        if 0 <= button_id < self.button_count:
            return self.button_states[button_id]
        return False

    def is_button_released(self, button_id):
        """detect button released"""
        if 0 <= button_id < self.button_count:
            return self.button_released[button_id]
        return False

    def get_axis_value(self, axis_id):
        """get joystick axis value"""
        if 0 <= axis_id < self.axis_count:
            return self.axis_states[axis_id]
        return 0.0

    def get_hat_direction(self, hat_id=0):
        """get joystick hat direction"""
        if 0 <= hat_id < self.hat_count:
            return self.hat_states[hat_id]
        return (0, 0)