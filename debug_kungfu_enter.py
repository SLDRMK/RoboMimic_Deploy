#!/usr/bin/env python3
"""
调试KungFu的enter方法调用
验证从SkillCast切换到KungFu时数据缓冲区是否正确初始化
"""

def debug_kungfu_enter():
    """调试KungFu的enter方法调用"""
    print("=== 调试KungFu的enter方法调用 ===")
    
    print("\n--- FSM切换流程分析 ---")
    print("1. NORMAL模式: 当前策略运行checkChange()")
    print("2. 如果需要切换:")
    print("   - 设置FSMmode = CHANGE")
    print("   - 调用当前策略的exit()")
    print("   - 调用get_next_policy()获取新策略")
    print("   - 打印切换信息")
    print("3. CHANGE模式:")
    print("   - 调用新策略的enter() ← 这里是关键！")
    print("   - 设置FSMmode = NORMAL")
    print("   - 立即运行新策略的run()")
    
    print("\n--- KungFu.enter()方法内容 ---")
    print("def enter(self):")
    print("    self.action = np.zeros(23, dtype=np.float32)")
    print("    self.action_buf = np.zeros(23 * self.history_length, dtype=np.float32)")
    print("    self.ref_motion_phase = 0.")
    print("    self.ref_motion_phase_buf = np.zeros(1 * self.history_length, dtype=np.float32)")
    print("    self.motion_time = 0")
    print("    self.counter_step = 0")
    print("    # ... 其他缓冲区初始化")
    print("    self.saved_skill_cmd = self.state_cmd.skill_cmd")
    
    print("\n--- 关键数据缓冲区 ---")
    print("1. action_buf: 动作历史缓冲区")
    print("2. ang_vel_buf: 角速度历史缓冲区")
    print("3. proj_g_buf: 重力投影历史缓冲区")
    print("4. dof_pos_buf: 关节位置历史缓冲区")
    print("5. dof_vel_buf: 关节速度历史缓冲区")
    print("6. ref_motion_phase_buf: 运动相位历史缓冲区")
    
    print("\n--- 可能的问题 ---")
    print("1. 如果enter()没有被调用，这些缓冲区可能包含之前状态的数据")
    print("2. 特别是action_buf，如果包含SkillCast的动作数据，会影响KungFu的推理")
    print("3. ref_motion_phase应该从0开始，否则会影响运动相位计算")
    
    print("\n--- 验证方法 ---")
    print("1. 在KungFu.enter()开始处添加打印语句:")
    print("   print('KungFu.enter() called - initializing buffers')")
    print("2. 在KungFu.run()开始处检查缓冲区状态:")
    print("   print('action_buf[0:5]:', self.action_buf[0:5])")
    print("   print('ref_motion_phase:', self.ref_motion_phase)")
    
    print("\n=== 建议的调试步骤 ===")
    print("1. 在KungFu.enter()方法开始处添加调试打印")
    print("2. 在KungFu.run()方法开始处检查关键变量")
    print("3. 运行测试，观察从SkillCast切换到KungFu时的输出")
    print("4. 确认所有缓冲区都被正确初始化为零")

if __name__ == "__main__":
    debug_kungfu_enter() 