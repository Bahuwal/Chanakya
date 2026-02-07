# Humanoid Robot Control - Duke + CAN Implementation

This repository contains two implementations of bipedal robot control:

## 📁 Folders

### 1. `dukeHumanoidHardwareControl/` 
**Original Duke University Implementation** (EtherCAT-based)

- Uses C++ EtherCAT motor control
- Requires compilation (CMake, C++)
- Original RL policies and configurations
- Linux-only support
- **For reference and EtherCAT hardware**

### 2. `bipedCANController/`
**Pure Python CAN Implementation** (Motorevo-based)

- 100% Python - NO compilation needed!
- CAN bus motor control (Motorevo protocol)
- Same RL policies as Duke
- Cross-platform support
- Includes manual control scripts
- **Ready to use with CAN motors**

---

## 🚀 Quick Start

### For CAN Motors (Recommended):
```bash
cd bipedCANController
python3 biped_controller_can.py
```

See `bipedCANController/README.md` for full documentation.

### For EtherCAT Motors:
```bash
cd dukeHumanoidHardwareControl
# Compile C++ code first
mkdir build && cd build
cmake ..
make
cd ..
python3 biped_controller.py
```

---

## 🔍 Key Differences

| Feature | dukeHumanoidHardwareControl | bipedCANController |
|---------|----------------------------|-------------------|
| **Language** | Python + C++ | Pure Python |
| **Protocol** | EtherCAT | CAN bus |
| **Compilation** | Required | Not needed |
| **Platform** | Linux only | Cross-platform |
| **Manual Control** | Limited | Full keyboard/trajectory support |
| **Documentation** | Basic | Comprehensive |

---

## 📚 Documentation

- `bipedCANController/README.md` - Full CAN implementation guide
- `bipedCANController/QUICK_START.md` - Quick reference
- `dukeHumanoidHardwareControl/README.md` - Original Duke docs

---

## ✨ Features

### Both Implementations:
- ✅ RL policy-based autonomous walking
- ✅ IMU + force sensor integration
- ✅ UDP telemetry and command system
- ✅ Gamepad velocity control

### CAN Implementation Only:
- ✅ Manual trajectory testing (`can_trajectory_PD_test.py`)
- ✅ Keyboard control (`sshkeyboard_pd_test.py`)  
- ✅ Mode selection helper
- ✅ Enhanced safety (limit checking)
- ✅ No compilation required

---

## 🎯 Which One to Use?

**Use `bipedCANController` if:**
- You have Motorevo CAN motors
- You want quick setup (no compilation)
- You need manual control for testing
- You're developing on macOS/Windows

**Use `dukeHumanoidHardwareControl` if:**
- You have EtherCAT motors
- You need hard real-time control
- You're exactly replicating Duke's setup

---

## 🔧 Hardware Requirements

### CAN Version:
- Motorevo CAN motors (IDs 1-10)
- USB-to-CAN adapter (or serial port)
- Teensy IMU (optional for RL)
- Force sensors (optional for RL)

### EtherCAT Version:
- EtherCAT-compatible motors
- EtherCAT master interface
- Teensy IMU
- Force sensors

---

## 📖 Getting Started

1. **Clone this repository**
2. **Choose your implementation** (CAN or EtherCAT)
3. **Read the README** in your chosen folder
4. **Test motors manually** before running RL
5. **Run RL controller** for autonomous walking

---

## 🙏 Credits

- **Duke University**: Original RL implementation and policies
- **Motorevo**: CAN motor protocol adaptation
- Hardware integration and CAN implementation by this project

---

**Status**: ✅ Both implementations tested and ready!
