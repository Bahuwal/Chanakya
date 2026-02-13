# Waveshare Biped Controller Setup Guide

## Quick Start

### Windows Setup

#### 1. Find Your COM Ports

Open Device Manager and look for "USB Serial Port" devices. You should see two Waveshare USB-CAN-A modules:
- One for TX (motor control)
- One for RX (parameter feedback)

Example: `COM13` and `COM18`

#### 2. Edit `can_config.yaml`

Open `can_config.yaml` in the same folder as your script and set your COM ports:

```yaml
# Waveshare Serial Ports (Windows)
motor_port: "COM13"   # ← Change to YOUR TX port
param_port: "COM18"   # ← Change to YOUR RX port (optional)
```

**Important**: Keep the quotes around the port names!

#### 3. Run the Script

```powershell
cd bipedCANControllerWaveshare
python can_trajectory_PD_test.py
```

**Note**: The script MUST be run from the `bipedCANControllerWaveshare` folder so it can find `can_config.yaml`.

---

### Linux Setup

#### 1. Find Your USB Devices

Plug in your Waveshare modules and find the device names:

```bash
ls /dev/ttyUSB*
# Should show: /dev/ttyUSB0  /dev/ttyUSB1
```

Or use `dmesg` to see which ports were assigned:

```bash
dmesg | grep tty
```

#### 2. Set Permissions (Important!)

Give your user permission to access the serial ports:

```bash
sudo usermod -a -G dialout $USER
# Log out and back in for this to take effect
```

Or run with sudo (not recommended for production):

```bash
sudo python can_trajectory_PD_test.py
```

#### 3. Edit `can_config.yaml`

```yaml
# Waveshare Serial Ports (Linux)
motor_port: "/dev/ttyUSB0"   # ← Change to YOUR TX port
param_port: "/dev/ttyUSB1"   # ← Change to YOUR RX port (optional)
```

#### 4. Run the Script

```bash
cd bipedCANControllerWaveshare
python can_trajectory_PD_test.py
```

---

## Configuration Options

### Required Settings

```yaml
motor_port: "COM13"           # Windows
# OR
motor_port: "/dev/ttyUSB0"    # Linux
```

### Optional Settings

```yaml
# If you have a separate feedback port
param_port: "COM18"           # Windows
param_port: "/dev/ttyUSB1"    # Linux

# Leave empty or set to null to use single-port mode
param_port: null
```

### Motor Parameters

All other parameters (motor IDs, gains, limits) are already configured in `can_config.yaml`. You can adjust them if needed:

```yaml
# Motor gains (already configured)
default_kp: [8.0, 195.0, 200.0, ...]
default_kd: [0.8, 58.0, 45.0, ...]

# Motor IDs
motor_ids: [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
```

---

## Troubleshooting

### Error: "can_config.yaml not found"

**Solution**: Run the script from the `bipedCANControllerWaveshare` folder:

```bash
# ✅ CORRECT
cd bipedCANControllerWaveshare
python can_trajectory_PD_test.py

# ❌ WRONG
python bipedCANControllerWaveshare/can_trajectory_PD_test.py
```

### Error: "motor_port must be specified"

**Solution**: Make sure `can_config.yaml` has `motor_port` set:

```yaml
motor_port: "COM13"  # Windows
# OR
motor_port: "/dev/ttyUSB0"  # Linux
```

### Error: "Permission denied" (Linux)

**Solution**: Add yourself to the `dialout` group:

```bash
sudo usermod -a -G dialout $USER
# Log out and back in
```

### Error: "Port not found" or "Cannot open port"

**Solution**: 
1. Check Device Manager (Windows) or `ls /dev/ttyUSB*` (Linux)
2. Make sure drivers are installed
3. Try unplugging and replugging the Waveshare modules
4. On Windows, update the COM port numbers in `can_config.yaml`

### Motors not responding

**Solution**:
1. Check that both Waveshare modules are powered and connected
2. Verify the baud rate is 2000000 (this is set automatically)
3. Make sure motor IDs in `can_config.yaml` match your actual motor IDs

---

## Single Port vs Dual Port Mode

### Dual Port (Recommended)

Use two Waveshare modules:
- One for TX (sending commands)
- One for RX (receiving feedback)

```yaml
motor_port: "COM13"
param_port: "COM18"
```

### Single Port (Fallback)

Use one Waveshare module for both TX and RX:

```yaml
motor_port: "COM13"
param_port: null
```

---

## Testing Your Setup

### Quick Test

1. Edit `can_config.yaml` with your COM ports
2. Run the test script:

```bash
python can_trajectory_PD_test.py
```

3. You should see:
```
✓ Waveshare USB-CAN-A configured: COM13 @ 1 Mbps, STANDARD frames
✓ Waveshare USB-CAN-A configured: COM18 @ 1 Mbps, STANDARD frames
CANMotorController initialized with 10 motors
  └── Motor port (TX): COM13
  └── Param port (RX): COM18
```

If you see this, your setup is working! ✅

---

## Platform Differences

| Feature | Windows | Linux |
|---------|---------|-------|
| Port Names | `COM13`, `COM18` | `/dev/ttyUSB0`, `/dev/ttyUSB1` |
| Permissions | Not needed | Need `dialout` group |
| Working Directory | Must be in script folder | Must be in script folder |
| Driver | Auto-installed | Usually works out of box |
