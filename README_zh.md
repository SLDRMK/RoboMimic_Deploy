# RoboMimic Deploy - 键盘控制增强版

## 项目简介

这是 [Robomimic Deploy](https://github.com/ccrpRepo/RoboMimic_Deploy) 的一个增强版本，专门为Unitree G1机器人（29自由度）设计的多策略机器人部署框架。本版本在原有基础上添加了**键盘控制功能**和**实时状态显示**，让机器人控制更加便捷和可靠。

## 🌟 主要特性

### 新增功能
- **⌨️ 键盘控制**：支持实时键盘控制，无需手柄
- **📊 实时状态显示**：实时显示机器人状态和控制信息
- **🔄 可靠模式切换**：改进的状态机切换逻辑
- **🎯 精确控制**：支持数字摇杆控制，精确调节速度

### 支持的控制模式
| 模式名称 | 描述 | 适用场景 |
|---------|------|----------|
| **PassiveMode** | 阻尼保护模式 | 安全启动和紧急停止 |
| **FixedPose** | 位置控制重置 | 机器人姿态初始化 |
| **LocoMode** | 稳定行走控制 | 日常移动和导航 |
| **Dance** | 查尔斯顿舞蹈 | 表演和展示 |
| **KungFu** | 武术动作 | 仿真环境测试 |
| **KungFu2** | 武术训练动作 | 仿真环境测试 |
| **Kick** | 踢腿动作 | 仿真环境测试 |
| **SkillCast** | 上半身定位控制 | 技能执行前准备 |
| **SkillCooldown** | 下半身平衡控制 | 技能执行后恢复 |

## 🛠️ 环境配置

### 系统要求
- **操作系统**：Ubuntu 18.04+ / macOS 10.15+
- **Python版本**：Python 3.8
- **机器人硬件**：Unitree G1 (29自由度，带3自由度腰部)

### 1. 创建虚拟环境
```bash
# 使用Conda创建虚拟环境
conda create -n robomimic python=3.8
conda activate robomimic
```

### 2. 安装依赖
```bash
# 安装PyTorch
conda install pytorch==2.3.1 torchvision==0.18.1 torchaudio==2.3.1 pytorch-cuda=12.1 -c pytorch -c nvidia

# 安装基础依赖
pip install numpy==1.20.0
pip install onnx onnxruntime
pip install pyyaml

# 安装Unitree SDK
git clone https://github.com/unitreerobotics/unitree_sdk2_python.git
cd unitree_sdk2_python
pip install -e .
```

### 3. 项目配置
```bash
# 克隆项目
git clone https://github.com/SLDRMK/RoboMimic_Deploy.git
cd RoboMimic_Deploy

# 检查配置文件
ls deploy_mujoco/config/
ls deploy_real/config/
```

## 🚀 快速开始

### MuJoCo仿真环境

#### 1. 启动仿真
```bash
# 使用键盘控制（推荐）
python deploy_mujoco/deploy_mujoco_keyboard_realtime.py

# 或使用基础键盘控制
python deploy_mujoco/deploy_mujoco_keyboard.py
```

#### 2. 控制说明
- **数字键 1-7**：切换不同模式
- **WASD**：前进/后退/左移/右移
- **JL**：左转/右转
- **空格键**：重置摇杆
- **Q**：退出程序

### 真实机器人部署

#### 1. 安全准备
- 确保机器人处于悬挂状态
- 检查电源和网络连接
- 准备紧急停止按钮

#### 2. 启动控制
```bash
# 使用键盘控制（推荐）
python deploy_real/deploy_real_keyboard.py

# 或使用手柄控制
python deploy_real/deploy_real.py
```

#### 3. 操作流程
1. 启动程序，等待DDS连接成功
2. 按 `1` 进入阻尼保护模式
3. 按 `2` 进入位控模式进行初始化
4. 按 `3` 进入行走模式开始控制
5. 使用WASD进行移动控制

## 📁 项目结构

```
RoboMimic_Deploy/
├── deploy_mujoco/           # MuJoCo仿真环境
│   ├── deploy_mujoco.py              # 手柄控制版本
│   ├── deploy_mujoco_keyboard.py     # 基础键盘控制
│   ├── deploy_mujoco_keyboard_realtime.py  # 实时键盘控制
│   ├── README_keyboard.md            # 仿真环境说明
│   └── config/
│       └── mujoco.yaml               # 仿真配置
├── deploy_real/             # 真实机器人部署
│   ├── deploy_real.py               # 手柄控制版本
│   ├── deploy_real_keyboard.py      # 实时键盘控制
│   ├── README_keyboard.md           # 真机部署说明
│   ├── config.py                    # 配置加载器
│   └── config/
│       └── real.yaml                # 真机配置
├── policy/                  # 策略模型
│   ├── dance/              # 舞蹈策略
│   ├── kungfu/             # 武术策略
│   ├── loco_mode/          # 行走策略
│   ├── passive/            # 被动模式
│   └── ...                 # 其他策略
├── FSM/                    # 有限状态机
├── common/                 # 公共组件
├── g1_description/         # G1机器人描述文件
└── run.sh                  # 快速启动脚本
```

## 🎮 控制模式详解

### 仿真环境操作
1. **启动仿真**：运行键盘控制程序
2. **模式切换**：使用数字键1-7快速切换
3. **移动控制**：WASD控制前进后退左右移动
4. **旋转控制**：JL控制左右旋转
5. **安全退出**：按Q或Ctrl+C退出

### 真机操作流程
1. **安全检查**：确保机器人安全悬挂
2. **网络连接**：检查DDS通信状态
3. **初始化**：从阻尼模式开始
4. **模式切换**：逐步切换到目标模式
5. **实时控制**：使用键盘进行精确控制
6. **安全停止**：及时切换到阻尼模式

## ⚠️ 安全注意事项

### 仿真环境
- 无安全风险，可自由测试
- 建议熟悉所有模式后再进行真机操作

### 真实机器人
- **必须确保机器人安全悬挂**
- **准备紧急停止机制**
- **保持安全距离**
- **避免突然的模式切换**
- **建议先在仿真环境中充分测试**

## 🔧 故障排除

### 常见问题
1. **按键无响应**
   - 检查终端是否处于输入状态
   - 确保没有其他程序占用键盘

2. **模式切换失败**
   - 检查当前模式的checkChange()方法
   - 确保FSM状态机正常工作

3. **DDS连接失败**
   - 检查网络配置
   - 确保Unitree SDK正确安装

4. **控制延迟**
   - 降低控制频率
   - 检查网络延迟

### 调试工具
- `debug_fsm_switches.py`：FSM状态切换调试
- `debug_kungfu_enter.py`：武术模式进入调试
- `joystick_debug.py`：手柄调试工具

## 📚 详细文档

- [MuJoCo仿真环境说明](./deploy_mujoco/README_keyboard.md)
- [真实机器人部署说明](./deploy_real/README_keyboard.md)

## 🤝 贡献指南

欢迎提交Issue和Pull Request来改进这个项目！

## 📄 许可证

本项目基于原始 [Robomimic Deploy](https://github.com/ccrpRepo/RoboMimic_Deploy) 项目，遵循相同的许可证。

## 🙏 致谢

- 感谢 [ccrpRepo](https://github.com/ccrpRepo) 提供的原始RoboMimic Deploy项目
- 感谢Unitree Robotics提供的G1机器人平台和SDK