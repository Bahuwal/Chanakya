# CAN-Based Biped RL Controller

This folder contains a **complete, standalone implementation** of Duke's RL bipedal controller adapted for CAN bus motors (Motorevo protocol).

## ✅ What's Included

All necessary files for running RL policies with CAN motors - **NO C++ compilation needed**!

### Core Files (Pure Python)
- `biped_controller_can.py` - Main RL controller (adapted from Duke's EtherCAT version)
- `can_motor_controller.py` - CAN motor interface with Duke-compatible API
- `sensor_controller.py` - IMU + force sensor reader
- `publisher.py` - UDP telemetry/command communication
- `numpy_ringbuffer.py` - Observation frame stacking
- `utils.py` - CAN data conversion utilities
- `gamepad.py` - Gamepad velocity command input
- `can_config.yaml` - Motor configuration (**updated to match Duke's exact limits**)

### Manual Control Files
- `can_trajectory_PD_test.py` - Manual motor testing with mode selection
- `sshkeyboard_pd_test.py` - Keyboard-based motor control for debugging
- `test_mode_selection.py` - Helper for control mode selection

### RL Policy Files
- `checkpoint/baseline/policy.onnx` - Trained walking policy weights
- `checkpoint/baseline/config.yaml` - Training hyperparameters (kp, kd, limits, etc.)

## 🔧 Configuration Changes from Original

### Updated to Match Duke Exactly:
1. **Joint Limits** - Now using Duke's exact values from `motor_controller.py`:
   ```yaml
   min_limit: [-0.7854, -1.2566, -0.3927, -0.3000, -0.6632, ...]
   max_limit: [0.7854, 1.2566, 0.6283, 1.0472, 0.6981, ...]
   ```

2. **Default Standing Pose** - Added from Duke's checkpoint config:
   ```yaml
   default_dof_pos: [0.0, 0.175, 0.1, 0.387, -0.213, ...]
   ```

3. **Motor IDs** - Using 1-10 instead of 0-9:
   ```yaml
   motor_ids: [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
   ```

### Kept Your Values:
- **PD Gains** - Still using your current kp/kd values (not Duke's 80/8)
  ```yaml
  default_kp: [20, 20, 20, 20, 20, ...]  # Your value
  default_kd: [5.0, 5.0, 5.0, 5.0, 5.0, ...]  # Your value
  ```

## 🆕 New Methods Added

### CANMotorController enhancements:
1. **`loop_counter`** - Tracks control loop iterations for RL timing
2. **`limit_check(positions)`** - Joint safety enforcement

## 🚀 How to Run

### Hardware Setup
1. Connect CAN motors to `/dev/ttyUSB0` (motor commands)
2. Connect param feedback to `/dev/ttyACM0` (or use same port)
3. Connect Teensy IMU to `/dev/ttyACM0`

### Run RL Controller
```bash
cd /path/to/bipedCANController
python3 biped_controller_can.py
```

### Send Velocity Commands (Separate Terminal)
```bash
python3 gamepad.py
# Or manually:
# {'cmd': [vel_x, vel_y, vel_yaw], 'reset': False}
```

### Monitor Telemetry (Optional)
```bash
plotjuggler --layout config.xml
# Listen on UDP port 9870 with MessagePack encoding
```

### Manual Motor Control (For Testing/Debugging)

#### Option 1: Trajectory PD Test (Recommended)
```bash
python3 can_trajectory_PD_test.py
# Select mode: 1 (servo) or 2 (ptm)
# Motors will hold zero position
# Send targets via UDP port 9871
```

#### Option 2: Keyboard Control
```bash
python3 sshkeyboard_pd_test.py
# Use keyboard to control individual joints
# Press keys to move motors (see script for key mappings)
# Good for debugging individual motor response
```

**Use Cases**:
- **Before RL**: Test motors work correctly
- **Calibration**: Verify joint ranges and limits
- **Debugging**: Isolate motor issues
- **Development**: Test control gains without full RL stack


## 📊 Control Flow

```
┌─────────────────────┐
│  policy.onnx        │─── RL Policy
│  (ONNX runtime)     │    Trained walking
└──────────┬──────────┘
           │ Actions (joint targets)
           ▼
┌─────────────────────┐
│ biped_controller_   │
│      can.py         │─── Main Controller
│                     │    - Loads policy
│  ┌──────────────┐   │    - Reads sensors
│  │ limit_check()│◄──┼─── - Enforces safety
│  └──────────────┘   │    - Controls motors
└─────┬───────────────┘
      │ Safe targets
      ▼
┌─────────────────────┐
│ can_motor_          │
│  controller.py      │─── Motor Interface
│                     │    - PD control @ 200Hz
│  loop_counter       │    - CAN communication
└─────┬───────────────┘
      │ CAN commands
      ▼
┌─────────────────────┐
│  Motorevo Motors    │─── Hardware
│  CAN Bus (1-10)     │
└─────────────────────┘
```

## 🔍 Key Differences from Original

| Feature | Original (EtherCAT) | CAN Version |
|---------|---------------------|-------------|
| Motor Interface | C++ `motor_controller.py` | Python `can_motor_controller.py` |
| Communication | EtherCAT protocol | CAN bus (serial) |
| Compilation | ✅ Required (CMake, C++) | ❌ Not needed |
| Platform | Linux only | Cross-platform |
| Control Loop | ~1000 Hz (hard real-time) | ~200 Hz (soft real-time) |
| Dependencies | SOEM library, pybind11 | pyserial, numpy |

## 📝 Notes

- **Limits Match Duke**: Joint limits now exactly match Duke's motor_controller.py
- **Standing Pose Added**: Default pose from checkpoint config for stable initialization
- **PD Gains**: Kept your current values (20/5) - can increase to Duke's (80/8) later
- **Debug Output**: Added loop_counter and limit_check for RL compatibility
- **Pure Python**: No compilation needed - runs on any platform!

## 🐛 Troubleshooting

### Motors don't respond:
- Check serial ports: `ls /dev/tty*`
- Verify motor IDs are 1-10 (not 0-9)
- Ensure param_port is reading feedback

### RL policy unstable:
- Try increasing kp/kd to Duke's values (80/8)
- Check joint limits match your robot's hardware
- Verify sensor data is correct (IMU, force sensors)

### Limit violations:
- Check `can_config.yaml` limits match your robot
- Enable logging in `limit_check()` method
- Verify standing pose is reasonable

## 📚 Related Documents

See brain artifacts for detailed analysis:
- `rl_ethercat_architecture.md` - RL system architecture
- `can_rl_requirements.md` - File dependency analysis
- `duke_parameters_reference.md` - Parameter comparison

---

**Status**: ✅ Ready to test on hardware!
