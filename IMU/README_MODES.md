# 📁 Teensy Code - Two Versions

## 🎯 Which Version Should I Use?

You now have **TWO separate versions** of the Teensy code:

### 1️⃣ `teensy_comm_gui.ino` - FOR PYTHON GUI
**Use this when:** You want to use the Python GUI (`imu_gui.py`)

**What it does:**
- ✅ Sends **binary packets** to the GUI
- ❌ **NO text output** to Serial Monitor
- Perfect for real-time visualization

**How to use:**
1. Open `teensy_comm_gui/teensy_comm_gui.ino` in Arduino IDE
2. Upload to Teensy
3. Close Arduino IDE
4. Run `python3 imu_gui.py`
5. Connect to your COM port

---

### 2️⃣ `teensy_comm_serial_debug.ino` - FOR SERIAL MONITOR
**Use this when:** You want to debug with Arduino Serial Monitor

**What it does:**
- ✅ Prints **human-readable text** to Serial Monitor
- ❌ **NO binary packets** (GUI won't work)
- Perfect for debugging and verification

**How to use:**
1. Open `teensy_comm_serial_debug/teensy_comm_serial_debug.ino` in Arduino IDE
2. Upload to Teensy
3. Open Serial Monitor (115200 baud)
4. See clean, readable IMU data

---

## ⚠️ IMPORTANT

**You CANNOT use both at the same time!**

- If you upload the GUI version → Use Python GUI, NOT Serial Monitor
- If you upload the Debug version → Use Serial Monitor, NOT Python GUI

**Switching between modes:**
1. Close whatever is currently open (GUI or Serial Monitor)
2. Upload the other version
3. Reset Teensy (press the button)
4. Open the appropriate tool

---

## 📂 File Structure

```
IMU/
├── teensy_comm_gui/
│   └── teensy_comm_gui.ino          ← For Python GUI
│
├── teensy_comm_serial_debug/
│   └── teensy_comm_serial_debug.ino ← For Serial Monitor Debug
│
├── imu_gui.py                       ← Python GUI program
└── README_MODES.md                  ← This file
```

---

## 🐛 Troubleshooting

| Problem | Solution |
|---------|----------|
| GUI shows all zeros | Upload `teensy_comm_gui.ino` version |
| Serial Monitor shows garbage | Upload `teensy_comm_serial_debug.ino` version |
| "Port busy" error | Close Serial Monitor before running GUI |
| GUI not responding | Make sure you uploaded the GUI version, not debug |

---

## ✅ Quick Switch Guide

### Want to use GUI?
```bash
# 1. Open teensy_comm_gui.ino
# 2. Upload to Teensy
# 3. Close Arduino IDE
python3 imu_gui.py
```

### Want to debug with Serial Monitor?
```bash
# 1. Close GUI if running
# 2. Open teensy_comm_serial_debug.ino
# 3. Upload to Teensy
# 4. Open Serial Monitor (115200 baud)
```

That's it! 🎉
