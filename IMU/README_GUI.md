# IMU Data GUI - Setup Instructions

## Quick Start

### 1. Install Python Dependencies
```bash
pip install pyserial
```
(tkinter comes pre-installed with Python on most systems)

### 2. Enable Binary Data Mode in Teensy

Open `teensy_comm_debug.ino` and **uncomment** these lines in the `loop()` function (around line 440):

```cpp
// BEFORE (commented out):
// uint8_t* data = reinterpret_cast<uint8_t*>(&combinedData);
// Packetizer::send(Serial, send_index, data, sizeof(combinedData));

// AFTER (uncommented):
uint8_t* data = reinterpret_cast<uint8_t*>(&combinedData);
Packetizer::send(Serial, send_index, data, sizeof(combinedData));
```

**Re-upload the code** to your Teensy board.

### 3. Run the GUI

```bash
python3 imu_gui.py
```

### 4. Connect to Teensy

1. In the GUI, enter your serial port:
   - **Mac/Linux**: `/dev/ttyACM0` (or `/dev/ttyUSB0`)
   - **Windows**: `COM3` (check Device Manager for your port)

2. Click **Connect**

3. You should see real-time IMU data!

## Troubleshooting

| Problem | Solution |
|---------|----------|
| "No module named 'serial'" | Run `pip install pyserial` |
| "Port not found" | Check the port name in Device Manager (Windows) or `ls /dev/tty*` (Mac/Linux) |
| GUI shows all zeros | Make sure you uncommented the Packetizer lines and re-uploaded to Teensy |
| Connection fails | Close any other Serial Monitor that might be using the port |

## Data Display

The GUI shows:
- ✅ **Raw Accelerometer** (m/s²)
- ✅ **Raw Gyroscope** (rad/s)
- ✅ **Quaternion** (x, y, z, w)
- ✅ **Gravity Vector** (g)
- ✅ **Filtered Acceleration** (m/s²)
- ✅ **Filtered Gyroscope** (rad/s)
- ✅ **Load Cells** (4 sensors)
- ✅ **Filter Status** (hex values)

## Notes

- The GUI reads binary data at ~100 Hz
- Close the GUI properly to release the serial port
- You cannot have both Serial Monitor and GUI open at the same time
