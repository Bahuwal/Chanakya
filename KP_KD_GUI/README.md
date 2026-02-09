# PTM Motor Tuning GUI

Interactive GUI for tuning Kp and Kd gains in PTM (Position-Torque Mix) mode for Motorevo motors.

## Features

### Real-Time Plotting
- **Position Graph**: Target vs Actual position tracking
- **Velocity Graph**: Actual velocity from motor feedback
- **Torque Graph**: Target vs Actual torque

### Control Parameters
- Motor ID (change motor being controlled)
- Target Position (radians, -12.5 to +12.5)
- Target Velocity (rad/s, -10.0 to +10.0)  
- Target Torque (Nm, -50.0 to +50.0)
- Kp Gain (0 to 250)
- Kd Gain (0 to 50)

### Motor Commands
- **Reset**: Send reset command (0xFD)
- **Motor Mode**: Enable motor mode (0xFC)
- **Zero Position**: Set current position as zero (0xFE)
- **Send Command**: Start PTM control loop to drive motor to target
- **STOP**: Emergency stop

### Live Feedback
- Current position, velocity, torque
- Motor temperature
- Error codes

## Usage

```bash
cd KP_KD_GUI
python ptm_tuning_gui.py
```

## Requirements

- Python 3.7+
- tkinter (usually built-in)
- matplotlib
- python-can
- pyserial
- numpy

Install dependencies:
```bash
pip install matplotlib python-can pyserial numpy
```

## Hardware Setup

1. Connect CAN bus to `can0` at 1 Mbps
2. Configure `usb.json` with param port (for motor wake-up)
3. Ensure motor is powered and connected

## Workflow

1. **Initialize**: GUI auto-initializes CAN and serial
2. **Reset**: Click "Reset Motor" to reset motor
3. **Motor Mode**: Click "Motor Mode" to enable motor control
4. **Zero Position**: (Optional) Click "Zero Position" to set current pos as zero
5. **Set Parameters**: Enter target position, Kp, Kd in text boxes
6. **Send Command**: Click "Send Command" - motor will move to target
7. **Tune Gains**: Adjust Kp/Kd while watching real-time plots
8. **Stop**: Click "STOP" when done

## Control Formula

```
T_total = Kp × (P_target - P_actual) + Kd × (V_target - V_actual) + T_feedforward
```

## Tips

- Start with low Kp (5-10) and low Kd (0.5-1.0)
- Increase Kp to improve tracking (but may cause oscillations)
- Increase Kd to dampen oscillations
- Use Target Torque for gravity compensation
- Monitor temperature and error codes

## Troubleshooting

**GUI doesn't start:**
- Check CAN interface: `ip link show can0`
- Verify param port: `ls -l /dev/ttyACM0`

**Motor doesn't respond:**
- Click "Reset Motor" first
- Then "Motor Mode"
- Check error code in status display

**Plots not updating:**
- Check CAN feedback is arriving: `candump can0`
- Verify motor ID matches

## Safety

- Always start with low gains
- Monitor temperature (< 80°C recommended)
- Use STOP button if motor behaves unexpectedly
- Error codes: 0x06=overvoltage, 0x07=undervoltage, 0x08=overcurrent

---

Based on Final_PTM_CAN_v2.py with fixed-rate control (5ms) for stable operation.
