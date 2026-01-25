# Chanakaya - Biped Humanoid Control with CAN Protocol

## Quick Start

### Prerequisites

```bash
# Install dependencies
pip install numpy pyserial pyyaml

# For keyboard control (optional)
pip install sshkeyboard
```

### Configuration

Edit `can_config.yaml` to match your setup:

```yaml
serial_port: "/dev/ttyUSB0"  # Your serial port
motor_ids: [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]  # Your motor CAN IDs
```

---

## Running the Joint Trajectory Matching Test

Control your motors using keyboard commands while visualizing telemetry in PlotJuggler.

### Step 1: Start the Real Robot Controller

```bash
# Linux (with real-time priority)
sudo chrt -f 99 python3 -u can_trajectory_PD_test.py

# Or without real-time priority
python3 can_trajectory_PD_test.py
```

### Step 2: Start the Keyboard Controller

In a **separate terminal**:

```bash
sudo chrt -f 99 python3 -u sshkeyboard_pd_test.py
```

### Step 3: Visualize in PlotJuggler

#### Install PlotJuggler

```bash
# Ubuntu/Debian
sudo apt install plotjuggler

# Or download AppImage from:
# https://github.com/facontidavide/PlotJuggler/releases
```

#### Configure UDP Streaming

1. Open PlotJuggler
2. Click **Streaming** → **Start: UDP Server**
3. Set **Port**: `9870`
4. Set **Protocol**: `MessagePack`
5. Click **Start**

#### Available Data Channels

| Channel | Description |
|---------|-------------|
| `real/dof_pos` | Actual joint positions (10 values) |
| `real/dof_pos_target` | Target joint positions |
| `real/dof_vel` | Joint velocities |
| `real/dof_force` | Joint torques |
| `real/action_is_on` | Which joints are active |

---

## Keyboard Commands

| Key | Action |
|-----|--------|
| `0` | Test hip pitch joints (sweep 0.5-3.5 Hz) |
| `1` | Test hip roll joints (sweep 0.5-3.0 Hz) |
| `2` | Test hip yaw joints (sweep 0.5-2.5 Hz) |
| `3` | Test knee joints (sweep 0.5-3.0 Hz) |
| `4` | Test ankle joints (sweep 0.5-3.0 Hz) |
| `r` | Full sweep test (all 10 joints, 30 seconds) |
| `9` | Reset to zero position |

---

## File Structure

```
├── can_motor_controller.py   # CAN motor adapter (main controller)
├── can_trajectory_PD_test.py # Trajectory test for CAN motors
├── can_config.yaml           # Motor configuration
├── utils.py                  # Float/uint conversion utilities
├── sshkeyboard_pd_test.py    # Keyboard controller (sends via UDP)
├── publisher.py              # UDP communication
├── motor_controller.py       # Original EtherCAT controller (reference)
├── trajectory_PD_test.py     # Original EtherCAT test (reference)
└── biped_controller.py       # RL biped controller
```

---

## Configuration Reference

### can_config.yaml

| Parameter | Description | Default |
|-----------|-------------|---------|
| `serial_port` | Serial port for sending motor commands | `/dev/ttyUSB0` |
| `param_port` | **[NEW]** Separate port for reading motor parameters | `null` |
| `baudrate` | Serial baud rate | `921600` |
| `motor_ids` | CAN IDs for 10 motors | `[0-9]` |
| `default_kp` | Position P gain | `60.0` |
| `default_kd` | Position D gain | `5.0` |
| `default_ikp` | Inner loop P gain | `30.0` |
| `ankle_coupling` | Enable ankle-knee coupling | `false` |

---

## Motor Parameter Initialization (IMPORTANT!)

**Motorevo motors won't move until their parameters are read!**

There are two methods to initialize motor parameters:

### Method 1: Dual Serial Port (Recommended for Linux/macOS)

Configure `param_port` in `can_config.yaml`:

```yaml
serial_port: "/dev/ttyUSB0"  # For sending commands
param_port: "/dev/ttyUSB1"   # For reading parameters
```

The `CANMotorController` will automatically:
- Send commands via `serial_port`
- Read motor feedback via `param_port` in a background thread

### Method 2: MotorEvo Application (Windows Only)

1. Connect motors to your Windows PC
2. Open the MotorEvo application
3. Read motor parameters through the GUI
4. Then run your control script (with `param_port: null`)

### Disabling Separate Param Port

If feedback comes on the same port as commands, set:

```yaml
param_port: null  # or "" or omit entirely
```

---

## Architecture

```
┌─────────────────┐     UDP:9871     ┌──────────────────────┐
│   Keyboard      │ ───────────────► │  can_trajectory_     │
│   Controller    │                  │  PD_test.py          │
│                 │                  │                      │
│ sshkeyboard_    │                  │  ┌────────────────┐  │
│ pd_test.py      │                  │  │ CANMotor       │  │
└─────────────────┘                  │  │ Controller     │  │
                                     │  └───────┬────────┘  │
┌─────────────────┐     UDP:9871     │          │           │
│   Simulation    │ ◄─────────────── │       TX │ RX        │
│   (legged_env)  │                  │          ▼           │
└─────────────────┘                  │  ┌────────────────┐  │
                                     │  │   Motorevo     │  │
                                     │  │   Motors (CAN) │  │
                                     │  └────────────────┘  │
                                     │          │           │
                                     │  motor_port  param_port
                                     │  (commands)  (feedback)
                                     └──────────────────────┘
```

