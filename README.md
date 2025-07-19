# RoboMimic Deploy - Enhanced Keyboard Control Version

## Project Overview

This is an enhanced version of [Robomimic Deploy](https://github.com/ccrpRepo/RoboMimic_Deploy), a multi-policy robot deployment framework specifically designed for the Unitree G1 robot (29-DoF). This version adds **keyboard control functionality** and **real-time status display** to make robot control more convenient and reliable.

## 🌟 Key Features

### New Features
- **⌨️ Keyboard Control**: Real-time keyboard control without requiring a gamepad
- **📊 Real-time Status Display**: Live display of robot status and control information
- **🔄 Reliable Mode Switching**: Improved state machine switching logic
- **🎯 Precise Control**: Digital joystick control with precise speed adjustment

### Supported Control Modes
| Mode Name | Description | Use Case |
|-----------|-------------|----------|
| **PassiveMode** | Damping protection mode | Safe startup and emergency stop |
| **FixedPose** | Position control reset | Robot pose initialization |
| **LocoMode** | Stable walking control | Daily movement and navigation |
| **Dance** | Charleston dance routine | Performance and demonstration |
| **KungFu** | Martial arts movements | Simulation environment testing |
| **KungFu2** | Martial arts training | Simulation environment testing |
| **Kick** | Kicking movements | Simulation environment testing |
| **SkillCast** | Upper body positioning control | Pre-skill execution preparation |
| **SkillCooldown** | Lower body balancing control | Post-skill execution recovery |

## 🛠️ Environment Setup

### System Requirements
- **Operating System**: Ubuntu 18.04+ / macOS 10.15+
- **Python Version**: Python 3.8
- **Robot Hardware**: Unitree G1 (29-DoF with 3-DoF waist)

### 1. Create Virtual Environment
```bash
# Create virtual environment using Conda
conda create -n robomimic python=3.8
conda activate robomimic
```

### 2. Install Dependencies
```bash
# Install PyTorch
conda install pytorch==2.3.1 torchvision==0.18.1 torchaudio==2.3.1 pytorch-cuda=12.1 -c pytorch -c nvidia

# Install basic dependencies
pip install numpy==1.20.0
pip install onnx onnxruntime
pip install pyyaml

# Install Unitree SDK
git clone https://github.com/unitreerobotics/unitree_sdk2_python.git
cd unitree_sdk2_python
pip install -e .
```

### 3. Project Configuration
```bash
# Clone the project
git clone https://github.com/SLDRMK/RoboMimic_Deploy.git
cd RoboMimic_Deploy

# Check configuration files
ls deploy_mujoco/config/
ls deploy_real/config/
```

## 🚀 Quick Start

### MuJoCo Simulation Environment

#### 1. Launch Simulation
```bash
# Use keyboard control (recommended)
python deploy_mujoco/deploy_mujoco_keyboard_realtime.py

# Or use basic keyboard control
python deploy_mujoco/deploy_mujoco_keyboard.py
```

#### 2. Control Instructions
- **Number keys 1-7**: Switch between different modes
- **WASD**: Forward/backward/left/right movement
- **JL**: Left/right rotation
- **Spacebar**: Reset joystick
- **Q**: Exit program

### Real Robot Deployment

#### 1. Safety Preparation
- Ensure the robot is suspended safely
- Check power and network connections
- Prepare emergency stop button

#### 2. Launch Control
```bash
# Use keyboard control (recommended)
python deploy_real/deploy_real_keyboard.py

# Or use gamepad control
python deploy_real/deploy_real.py
```

#### 3. Operation Procedure
1. Start the program and wait for DDS connection success
2. Press `1` to enter damping protection mode
3. Press `2` to enter position control mode for initialization
4. Press `3` to enter walking mode and start control
5. Use WASD for movement control

## 📁 Project Structure

```
RoboMimic_Deploy/
├── deploy_mujoco/           # MuJoCo simulation environment
│   ├── deploy_mujoco.py              # Gamepad control version
│   ├── deploy_mujoco_keyboard.py     # Basic keyboard control
│   ├── deploy_mujoco_keyboard_realtime.py  # Real-time keyboard control
│   ├── README_keyboard.md            # Simulation environment guide
│   └── config/
│       └── mujoco.yaml               # Simulation configuration
├── deploy_real/             # Real robot deployment
│   ├── deploy_real.py               # Gamepad control version
│   ├── deploy_real_keyboard.py      # Real-time keyboard control
│   ├── README_keyboard.md           # Real robot deployment guide
│   ├── config.py                    # Configuration loader
│   └── config/
│       └── real.yaml                # Real robot configuration
├── policy/                  # Policy models
│   ├── dance/              # Dance policy
│   ├── kungfu/             # Martial arts policy
│   ├── loco_mode/          # Walking policy
│   ├── passive/            # Passive mode
│   └── ...                 # Other policies
├── FSM/                    # Finite State Machine
├── common/                 # Common components
├── g1_description/         # G1 robot description files
└── run.sh                  # Quick start script
```

## 🎮 Control Mode Details

### Simulation Environment Operation
1. **Launch Simulation**: Run keyboard control program
2. **Mode Switching**: Use number keys 1-7 for quick switching
3. **Movement Control**: WASD for forward/backward/left/right movement
4. **Rotation Control**: JL for left/right rotation
5. **Safe Exit**: Press Q or Ctrl+C to exit

### Real Robot Operation Procedure
1. **Safety Check**: Ensure robot is safely suspended
2. **Network Connection**: Check DDS communication status
3. **Initialization**: Start from damping mode
4. **Mode Switching**: Gradually switch to target mode
5. **Real-time Control**: Use keyboard for precise control
6. **Safe Stop**: Switch to damping mode in time

## ⚠️ Safety Considerations

### Simulation Environment
- No safety risks, free to test
- Recommend familiarizing with all modes before real robot operation

### Real Robot
- **Must ensure robot is safely suspended**
- **Prepare emergency stop mechanism**
- **Maintain safe distance**
- **Avoid sudden mode switching**
- **Recommend thorough testing in simulation environment first**

## 🔧 Troubleshooting

### Common Issues
1. **No Response to Keys**
   - Check if terminal is in input state
   - Ensure no other program is occupying keyboard

2. **Mode Switching Failure**
   - Check checkChange() method of current mode
   - Ensure FSM state machine works properly

3. **DDS Connection Failure**
   - Check network configuration
   - Ensure Unitree SDK is properly installed

4. **Control Delay**
   - Reduce control frequency
   - Check network latency

### Debug Tools
- `debug_fsm_switches.py`: FSM state switching debug
- `debug_kungfu_enter.py`: Martial arts mode entry debug
- `joystick_debug.py`: Gamepad debug tool

## 📚 Detailed Documentation

- [MuJoCo Simulation Environment Guide](./deploy_mujoco/README_keyboard.md)
- [Real Robot Deployment Guide](./deploy_real/README_keyboard.md)

## 🤝 Contributing

Welcome to submit Issues and Pull Requests to improve this project!

## 📄 License

This project is based on the original [Robomimic Deploy](https://github.com/ccrpRepo/RoboMimic_Deploy) project and follows the same license.

## 🙏 Acknowledgments

- Thanks to [ccrpRepo](https://github.com/ccrpRepo) for providing the original RoboMimic Deploy project
- Thanks to Unitree Robotics for providing the G1 robot platform and SDK

## 🌍 Language Versions

- [中文版 (Chinese)](./README_zh.md)
- [English](./README.md) 