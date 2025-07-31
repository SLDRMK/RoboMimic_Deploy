# 上肢舞蹈功能 (DanceUpper)

## 功能概述

上肢舞蹈功能是一个创新的机器人控制策略，将原来的 `kungfu2` 状态改造成上肢舞蹈模式。该功能结合了：

- **下肢控制**：使用 `skill_cooldown` 策略控制下肢和腰部（15个关节），保持机器人平衡
- **上肢舞蹈**：使用舞蹈数据控制上肢（14个关节），执行小苹果舞蹈动作
- **S型插值**：实现平滑的姿态过渡，避免突然的动作变化

## 技术特点

### 1. 混合控制架构
- **下肢（关节0-14）**：由 `skill_cooldown` 的 `policy_15dof.pt` 模型控制
- **上肢（关节15-28）**：由舞蹈数据控制，包括：
  - 左臂：关节15-21（7个关节）
  - 右臂：关节22-28（7个关节）

### 2. S型插值算法
使用sigmoid函数实现平滑的姿态过渡：
```python
def s_curve_interpolation(t, t_start, t_end, start_pos, end_pos):
    progress = (t - t_start) / (t_end - t_start)
    x = (progress - 0.5) * 12  # 映射到[-6, 6]
    s_curve = 1.0 / (1.0 + math.exp(-x))
    return start_pos + s_curve * (end_pos - start_pos)
```

### 3. 舞蹈数据支持
- 支持89个舞蹈姿态的完整序列
- 每个姿态包含过渡时间和保持时间
- 自动处理姿态切换和时序控制

## 使用方法

### 1. 仿真环境
```bash
# 启动键盘控制仿真
conda activate robomimic
python deploy_mujoco/deploy_mujoco_keyboard_realtime.py

# 按数字键 '7' 进入上肢舞蹈模式
```

### 2. 控制说明
- **数字键 7**：切换到上肢舞蹈模式
- **其他控制**：下肢仍可通过WASD进行移动控制
- **安全退出**：按数字键 '1' 返回阻尼保护模式

### 3. 演示脚本
```bash
# 运行演示脚本
python demo_dance_upper.py

# 运行测试脚本
python test_dance_upper.py
```

## 文件结构

```
policy/dance_upper/
├── DanceUpper.py          # 主要策略实现
├── __init__.py            # 模块初始化
└── README.md              # 说明文档

舞蹈数据_小苹果_2025-07-31.json  # 舞蹈数据文件
demo_dance_upper.py              # 演示脚本
test_dance_upper.py              # 测试脚本
```

## 舞蹈数据格式

舞蹈数据采用JSON格式，包含以下结构：
```json
{
  "name": "小苹果",
  "poses": [
    {
      "name": "1",
      "positions": [关节角度数组]
    }
  ],
  "choreography": {
    "sequence": [
      {
        "pose_name": "1",
        "transition_time": 0.8,
        "hold_time": 1.0
      }
    ]
  }
}
```

## 技术参数

- **控制周期**：20ms (50Hz)
- **关节数量**：29个（下肢15个 + 上肢14个）
- **舞蹈姿态**：89个
- **插值方式**：S型插值（sigmoid函数）
- **PD参数**：与skill_cooldown相同

## 安全特性

1. **平衡保持**：下肢始终由平衡策略控制
2. **平滑过渡**：S型插值避免突然动作
3. **紧急停止**：支持快速切换到阻尼模式
4. **状态监控**：实时显示当前姿态和进度

## 扩展性

该架构具有良好的扩展性：
- 可以轻松替换舞蹈数据文件
- 支持不同的插值算法
- 可以调整下肢控制策略
- 支持添加新的舞蹈动作

## 注意事项

1. **数据格式**：确保舞蹈数据中的关节顺序与机器人配置一致
2. **时序控制**：舞蹈数据的时序需要合理设置
3. **安全测试**：建议先在仿真环境中充分测试
4. **性能优化**：大量姿态数据可能影响实时性能 