# IMU Quick Start Guide

## ✅ Your IMU is Working!

Congratulations! Your LORD Microstrain 3DM-CV7 IMU is successfully communicating with the Teensy 4.0.

## 🎯 Two Ways to View Data

### Option 1: Serial Monitor (Human-Readable Debug)

**Use this for:** Quick debugging and verification

1. **Comment out** binary transmission in `teensy_comm_debug.ino` (lines 440-441):
   ```cpp
   // uint8_t* data = reinterpret_cast<uint8_t*>(&combinedData);
   // Packetizer::send(Serial, send_index, data, sizeof(combinedData));
   ```

2. Upload to Teensy

3. Open Serial Monitor (115200 baud)

4. See clean output:
   ```
   [Raw Accelerometer] (m/s²)
     X: 0.0169  Y: -0.0144  Z: 9.7948
   ```

### Option 2: Python GUI (Real-Time Visualization)

**Use this for:** Real-time monitoring and data visualization

1. **Uncomment** binary transmission in `teensy_comm_debug.ino` (lines 440-441):
   ```cpp
   uint8_t* data = reinterpret_cast<uint8_t*>(&combinedData);
   Packetizer::send(Serial, send_index, data, sizeof(combinedData));
   ```

2. Upload to Teensy

3. Run the GUI:
   ```bash
   python3 imu_gui.py
   ```

4. Enter port (e.g., `/dev/ttyACM0`) and click **Connect**

## 📊 What Data You're Getting

| Sensor | Description | Units |
|--------|-------------|-------|
| **Raw Accelerometer** | Direct sensor readings | m/s² |
| **Raw Gyroscope** | Direct sensor readings | rad/s |
| **Quaternion** | Orientation (x, y, z, w) | - |
| **Gravity Vector** | Estimated gravity direction | g |
| **Filtered Acceleration** | Gravity-compensated | m/s² |
| **Filtered Gyroscope** | Filter-compensated | rad/s |
| **Load Cells** | 4 force sensors | raw |
| **Filter Status** | IMU filter state | hex |

## 🔧 Hardware Setup Summary

```
IMU 3DM-CV7          Teensy 4.0
─────────────────    ──────────
Pin 3 (5V)    ────→  5V
Pin 4 (RxD)   ────→  Pin 1 (TX1)
Pin 5 (TxD)   ────→  Pin 0 (RX1)
Pin 8 (GND)   ────→  GND
```

**Serial Settings:**
- UART Baud Rate: **921600**
- USB Serial (PC): **115200** (for debug) or **115200** (for GUI)

##⚙️ Configuration Checklist

✅ IMU configured with SensorConnect:
  - Baud rate: **921600**
  - Sample on Startup: **DISABLED**
  - Settings saved to non-volatile memory

✅ Wiring:
  - TX/RX crossed correctly
  - 5V power connected
  - Ground connected

✅ Teensy Code:
  - `Serial1.begin(921600)` for IMU
  - Choose debug mode OR binary mode (see above)

## 📁 Files in This Directory

- `teensy_comm_debug.ino` - Teensy code with debug output
- `imu_gui.py` - Python GUI application  
- `README_GUI.md` - Detailed GUI setup instructions
- `QUICKSTART.md` - This file

## 🐛 Common Issues

| Issue | Fix |
|-------|-----|
| All zeros in data | IMU not configured - use SensorConnect |
| "Failed to set to idle" | IMU has "Sample on Startup" enabled |
| Garbage in Serial Monitor | Binary mode is on - comment out Packetizer lines |
| GUI shows zeros | Binary mode is off - uncomment Packetizer lines |
| Port busy error | Close Serial Monitor or other programs using the port |

## 🎉 Next Steps

Now that your IMU is working:
1. ✅ Calibrate the load cells if needed
2. ✅ Integrate with your biped control code
3. ✅ Log data for analysis
4. ✅ Tune filter parameters if required

**Questions?** Check the original Duke Biped documentation:
- Electronics: https://sleepy-yoke-a21.notion.site/Electronics-a7955bf0432e4bbea03c808f642c2290
- GitHub: https://github.com/generalroboticslab/dukeHumanoidHardwareControl
