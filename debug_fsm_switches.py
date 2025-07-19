#!/usr/bin/env python3
"""
调试FSM状态切换过程
模拟从LOCOMODE到SKILL_CAST到SKILL_KungFu的完整流程
"""

def simulate_fsm_switches():
    """模拟FSM状态切换过程"""
    print("=== FSM状态切换调试 ===")
    
    # 模拟状态
    current_state = "LOCOMODE"
    skill_cmd = "INVALID"
    cur_step = 0
    num_step = 50  # 1秒 / 0.02秒 = 50步
    ref_motion_phase = 0.0
    motion_length = 18.267
    
    print(f"初始状态: {current_state}")
    
    # 步骤1: LOCOMODE收到SKILL_2命令
    print("\n--- 步骤1: LOCOMODE收到SKILL_2命令 ---")
    skill_cmd = "SKILL_2"
    print(f"当前状态: {current_state}")
    print(f"收到命令: {skill_cmd}")
    
    # 模拟LocoMode.checkChange()
    if skill_cmd == "SKILL_2":
        next_state = "SKILL_CAST"
        print(f"LocoMode.checkChange() 返回: {next_state}")
        current_state = next_state
        skill_cmd = "INVALID"
        cur_step = 0
        print(f"切换到: {current_state}")
    
    # 步骤2: SKILL_CAST运行1秒
    print("\n--- 步骤2: SKILL_CAST运行1秒 ---")
    print(f"当前状态: {current_state}")
    print(f"开始运行，cur_step: {cur_step}, num_step: {num_step}")
    
    for step in range(num_step):
        cur_step = step + 1
        if step % 10 == 0:  # 每0.2秒打印一次
            print(f"  步骤 {cur_step}: {current_state}")
        
        # 模拟SkillCast.checkChange()
        if cur_step >= num_step and skill_cmd == "SKILL_2":
            next_state = "SKILL_KungFu"
            print(f"  步骤 {cur_step}: SkillCast.checkChange() 返回: {next_state}")
            current_state = next_state
            skill_cmd = "INVALID"
            ref_motion_phase = 0.0
            print(f"  切换到: {current_state}")
            break
        elif skill_cmd == "PASSIVE":
            next_state = "PASSIVE"
            print(f"  步骤 {cur_step}: SkillCast.checkChange() 返回: {next_state}")
            current_state = next_state
            break
        else:
            # 继续在SKILL_CAST中
            pass
    
    # 步骤3: SKILL_KungFu运行
    print(f"\n--- 步骤3: SKILL_KungFu运行 ---")
    print(f"当前状态: {current_state}")
    print(f"开始运行，ref_motion_phase: {ref_motion_phase}")
    
    # 模拟运行一段时间
    for step in range(100):  # 模拟运行2秒
        motion_time = step * 0.02
        ref_motion_phase = motion_time / motion_length
        
        if step % 25 == 0:  # 每0.5秒打印一次
            print(f"  步骤 {step}: {current_state}, phase: {ref_motion_phase:.3f}")
        
        # 模拟收到LOCO命令
        if step == 50:  # 1秒后收到LOCO命令
            skill_cmd = "LOCO"
            print(f"  步骤 {step}: 收到命令: {skill_cmd}")
            
            # 模拟KungFu.checkChange()
            if skill_cmd == "LOCO":
                next_state = "SKILL_COOLDOWN"
                print(f"  步骤 {step}: KungFu.checkChange() 返回: {next_state}")
                current_state = next_state
                skill_cmd = "INVALID"
                cur_step = 0
                print(f"  切换到: {current_state}")
                break
    
    # 步骤4: SKILL_COOLDOWN运行1秒
    print(f"\n--- 步骤4: SKILL_COOLDOWN运行1秒 ---")
    print(f"当前状态: {current_state}")
    print(f"开始运行，cur_step: {cur_step}, num_step: {num_step}")
    
    for step in range(num_step):
        cur_step = step + 1
        if step % 10 == 0:  # 每0.2秒打印一次
            print(f"  步骤 {cur_step}: {current_state}")
        
        # 模拟SkillCooldown.checkChange()
        if cur_step >= num_step:
            next_state = "LOCOMODE"
            print(f"  步骤 {cur_step}: SkillCooldown.checkChange() 返回: {next_state}")
            current_state = next_state
            print(f"  切换到: {current_state}")
            break
        elif skill_cmd == "PASSIVE":
            next_state = "PASSIVE"
            print(f"  步骤 {cur_step}: SkillCooldown.checkChange() 返回: {next_state}")
            current_state = next_state
            break
        else:
            # 继续在SKILL_COOLDOWN中
            pass
    
    print(f"\n=== 最终状态: {current_state} ===")
    print("调试完成！")

if __name__ == "__main__":
    simulate_fsm_switches() 