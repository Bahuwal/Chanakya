# Waveshare CAN Adapter - Testing Guide

##  Created Test File

✅ `waveshare_ptm_test.py` - Standalone test with Waveshare protocol

## What Changed

| Old (Jason) | New (Waveshare) |
|-------------|-----------------|
| 921,600 bps | **2,000,000 bps** |
| No init | **AA 55 12 01** (1Mbps CAN config) |
| AA 01 start | **AA C8** start |
| ID at end (4 bytes) | **ID at start** (2 bytes) |
| F4 tail | **55 tail** |
| 15 bytes | **13 bytes** |

## Testing Steps

### 1. Hardware Setup

```
Waveshare Adapter     Motors
───────────────       ──────
USB → Jetson
CAN_H (Yellow)   →    All motors CAN_H
CAN_L (Green)    →    All motors CAN_L
GND              →    Motor GND

Add 120Ω resistors between CAN_H and CAN_L at both ends!
```

### 2. Find Waveshare Port

```bash
# List USB devices
ls /dev/ttyUSB*

# Should see: /dev/ttyUSB0 (or similar)

# Check permissions
ls -l /dev/ttyUSB0

# Fix if needed
sudo chmod 666 /dev/ttyUSB0
```

### 3. Create USB Config

Create `usb.json` in the same directory:

```json
{
  "motor_port": "/dev/ttyUSB0",
  "param_port": "/dev/ttyUSB0",
  "baudrate": 2000000
}
```

### 4. Run Test

```bash
cd /path/to/Biped_CPP
python3 waveshare_ptm_test.py
```

## Expected Output

If working correctly, you should see:

```
✓ Serial port /dev/ttyUSB0 opened.
✓ Waveshare adapter configured to 1Mbps CAN

PTM MODE HARDWARE TEST - Position-Torque Mix Control
======================================================================
Target Position: 5.0 rad
Target Velocity: 0.0 rad/s
Target Torque: 2.0 Nm (feed-forward)
Kp: 20.0, Kd: 0.8
======================================================================

Time: 0.00s | Pos:   0.12 rad | Vel:   0.05 rad/s | Torque:   1.98 Nm | Temp: 25.0°C | Err: 0
...
```

## Troubleshooting

### No motor response
- Check CAN wiring (CANH/CANL)
- Verify 120Ω termination
- Ensure motors are powered ON
- Check motor ID (default is 7 in test)

### Serial port error
```bash
# Add user to dialout group
sudo usermod -a -G dialout $USER
# Logout and login again
```

### Wrong baudrate
The test file defaults to 2,000,000 bps. If Waveshare needs different:
- Check Waveshare documentation
- Modify `load_usb_config()` defaults

## Next Steps

Once test works:
1. ✅ Waveshare hardware confirmed working
2. ✅ Protocol implementation verified
3. → Apply same changes to `can_motor_controller.py`
4. → Test with `can_trajectory_PD_test.py`
5. → Test with full biped controller

## Reverting to Jason Protocol

If you need to switch back, use the original files:
- `Final PTM Mode With Reset and Read Param.py` (Jason protocol)
- Baudrate: 921,600 bps
