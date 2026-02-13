# bipedCANController - Quick Reference

## File Count: 13 total

### Python Files (10)
- **biped_controller_can.py** - RL policy controller (main file for autonomous walking)
- **can_motor_controller.py** - CAN motor interface with PD control
- **can_trajectory_PD_test.py** - Manual motor testing
- **sshkeyboard_pd_test.py** - Keyboard-based control
- **test_mode_selection.py** - Mode selection helper
- **sensor_controller.py** - IMU/force sensor interface
- **publisher.py** - UDP communication
- **numpy_ringbuffer.py** - Data buffering
- **utils.py** - CAN conversion utilities
- **gamepad.py** - Joystick input

### Config Files (1)
- **can_config.yaml** - Motor parameters, limits, gains

### Policy Files (2)
- **checkpoint/baseline/policy.onnx** - RL policy weights
- **checkpoint/baseline/config.yaml** - RL hyperparameters

## Quick Usage

### For RL Walking (Production):
```bash
python3 biped_controller_can.py
python3 gamepad.py  # In separate terminal
```

### For Manual Testing (Development):
```bash
python3 can_trajectory_PD_test.py  # Mode selection
# OR
python3 sshkeyboard_pd_test.py     # Keyboard control
```

## Key Changes from Original Duke Code
1. ✅ Replaced C++ EtherCAT with Python CAN
2. ✅ Updated limits to Duke's exact values
3. ✅ Added default standing pose
4. ✅ Kept your PD gains (20/5)
5. ✅ Added manual control files
6. ✅ 100% Python - no compilation

## Files NOT Needed
- ❌ `src/` (C++ code)
- ❌ `CMakeLists.txt`
- ❌ `build/`
- ❌ `ext/SOEM/`
- ❌ `motor_controller.py` (old EtherCAT version)

## Status
✅ Ready for hardware testing!
