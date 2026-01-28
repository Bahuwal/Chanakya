# Jetson Orin Nano Super - Native CAN Setup Guide

## Overview

The **Jetson Orin Nano Super** has **2 built-in CAN controllers** (CAN0 and CAN1). This guide shows you how to enable and use them - **NO STM32F7 needed!**

## Why This is Better

✅ **Simpler**: Only need CAN transceiver (no STM32F7)  
✅ **Faster**: Native hardware CAN controller  
✅ **More Reliable**: Direct kernel driver support  
✅ **Less Code**: Use standard Linux SocketCAN  

---

## Hardware Setup

### What You Need

1. ✅ Jetson Orin Nano Super (you have this!)
2. ✅ **MCP2551** or **SN65HVD230** CAN transceiver module ($2-5)
3. ✅ Jumper wires
4. ✅ Your 10 Motorevo motors
5. ✅ 120Ω resistors (2x) for CAN bus termination

### Jetson Orin Nano 40-Pin Header Pinout

The Jetson Orin Nano has CAN signals on the **40-pin expansion header**:

```
Pin Layout (Top View):
┌─────────────────────────────────┐
│ 3.3V  [1]  [2]  5V              │
│ SDA   [3]  [4]  5V              │
│ SCL   [5]  [6]  GND             │
│ ...                             │
│ CAN0_DIN [37] [38] CAN1_DIN     │  ← CAN RX pins
│ GND     [39] [40] CAN0_DOUT     │  ← CAN TX pin
└─────────────────────────────────┘

CAN1_DOUT is also available (check specific carrier board)
```

#### **CAN0 Pins:**
- **Pin 37**: CAN0_DIN (CAN0 RX) - Receive
- **Pin 40**: CAN0_DOUT (CAN0 TX) - Transmit
- **Pin 39**: GND

#### **CAN1 Pins:**
- **Pin 38**: CAN1_DIN (CAN1 RX) - Receive  
- **Pin ??**: CAN1_DOUT (CAN1 TX) - Check your carrier board docs

**We'll use CAN0 for this guide.**

### Wiring Diagram

```
Jetson Orin Nano (40-pin)    MCP2551 Module        Motors (CAN Bus)
─────────────────────────    ──────────────        ────────────────
Pin 40 (CAN0_DOUT/TX) ──→    TXD                   
Pin 37 (CAN0_DIN/RX)  ←──    RXD                   
Pin 1  (3.3V)         ──→    VCC (if 3.3V module)  
  or Pin 2 (5V)       ──→    VCC (if 5V module)    
Pin 6 or 39 (GND)     ──→    GND                   
                             CANH ─────────┬──→ Motor 1 CAN_H (Yellow)
                                           ├──→ Motor 2 CAN_H
                                           ├──→ ...
                                           └──→ Motor 10 CAN_H
                                           
                             CANL ─────────┬──→ Motor 1 CAN_L (Green)
                                           ├──→ Motor 2 CAN_L
                                           ├──→ ...
                                           └──→ Motor 10 CAN_L

⚠️  IMPORTANT: Add 120Ω resistor between CANH-CANL at BOTH ends of bus!
```

### Physical Connection Photos

**MCP2551 Module** typically has:
```
[TXD] [RXD] [VCC] [GND] [CANH] [CANL]
  ↑     ↑     ↑     ↑      ↑      ↑
  To    To   3.3V  GND    To     To
 Pin40 Pin37  or        Yellow  Green
              5V         wire    wire
```

---

## Software Setup

### Step 1: Enable CAN in Device Tree

The Jetson Orin Nano CAN controllers may need to be enabled in the device tree.

#### Option A: Using Jetson-IO Tool (Recommended)

```bash
# Run the configuration tool
sudo /opt/nvidia/jetson-io/jetson-io.py

# In the menu:
# 1. Select "Configure 40-pin expansion header"
# 2. Navigate to and enable "can0"
# 3. Save and exit
# 4. Reboot

sudo reboot
```

#### Option B: Manual Device Tree Edit

If jetson-io doesn't have CAN option, you'll need to modify the device tree overlay:

```bash
# Check if CAN device exists
ls /sys/class/net/can*

# If not found, you may need a custom device tree overlay
# This is board-specific - check NVIDIA Jetson Linux documentation
```

### Step 2: Install CAN Utilities

```bash
# Update package list
sudo apt-get update

# Install CAN utilities
sudo apt-get install -y can-utils

# Optional: Install Python CAN library
pip3 install python-can
```

### Step 3: Configure CAN Interface

Create a script to automatically configure CAN on boot:

```bash
# Create setup script
sudo nano /usr/local/bin/setup_can.sh
```

Add this content:

```bash
#!/bin/bash
# CAN0 Setup Script for Jetson Orin Nano

# Set CAN bitrate to 500 kbps (match Motorevo motors)
sudo ip link set can0 type can bitrate 500000

# Optional: Set sample point (default is fine)
# sudo ip link set can0 type can bitrate 500000 sample-point 0.875

# Bring up the interface
sudo ip link set can0 up

# Check status
ip -details link show can0

echo "CAN0 enabled at 500 kbps"
```

Make it executable:

```bash
sudo chmod +x /usr/local/bin/setup_can.sh
```

### Step 4: Run CAN Setup

```bash
# Run the setup script
sudo /usr/local/bin/setup_can.sh

# Verify CAN is up
ifconfig can0
# or
ip link show can0
```

You should see something like:
```
can0: <NOARP,UP,LOWER_UP,ECHO> mtu 16 qdisc pfifo_fast state UP
    link/can 
    can state ERROR-ACTIVE (berr-counter tx 0 rx 0) restart-ms 0 
    bitrate 500000 sample-point 0.875
```

### Step 5: Make CAN Auto-Start on Boot

```bash
# Create systemd service
sudo nano /etc/systemd/system/can-setup.service
```

Add this content:

```ini
[Unit]
Description=Setup CAN0 interface
After=network.target

[Service]
Type=oneshot
ExecStart=/usr/local/bin/setup_can.sh
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
```

Enable the service:

```bash
sudo systemctl daemon-reload
sudo systemctl enable can-setup.service
sudo systemctl start can-setup.service
sudo systemctl status can-setup.service
```

---

## Testing CAN Communication

### Test 1: CAN Utilities Test

```bash
# Terminal 1: Listen to CAN bus
candump can0

# Terminal 2: Send test message
cansend can0 123#DEADBEEF

# You should see the message in Terminal 1
```

### Test 2: Test with Single Motor

First, make sure one motor is powered and connected to the CAN bus.

```bash
# Send a position command to motor ID 1
# Format: [Motor ID (0x000-0x3FF)] # [8 bytes of data]

# Example: Send zero position command
cansend can0 001#0000000000000000
```

Check if motor responds:
```bash
candump can0
# You should see response from motor
```

### Test 3: Python CAN Test

Create a test script:

```python
#!/usr/bin/env python3
import can
import time

# Create CAN bus instance
bus = can.interface.Bus(channel='can0', bustype='socketcan')

print("Sending CAN test message...")

# Create a CAN message
msg = can.Message(
    arbitration_id=0x123,
    data=[0xDE, 0xAD, 0xBE, 0xEF, 0x00, 0x00, 0x00, 0x00],
    is_extended_id=False
)

# Send message
bus.send(msg)
print(f"Sent: {msg}")

# Listen for responses (2 seconds)
print("\nListening for responses...")
timeout = time.time() + 2.0

while time.time() < timeout:
    msg = bus.recv(timeout=0.1)
    if msg:
        print(f"Received: ID=0x{msg.arbitration_id:03X}, Data={msg.data.hex()}")

bus.shutdown()
```

Run it:
```bash
python3 can_test.py
```

---

## Update Your Biped Controller Code

### Modify `can_motor_controller.py`

#### Option 1: Add SocketCAN Support

Add this at the top of `can_motor_controller.py`:

```python
import can  # pip3 install python-can

class CANPort:
    """Native SocketCAN interface for Jetson"""
    def __init__(self, channel='can0', bitrate=500000):
        self.bus = can.interface.Bus(channel=channel, bustype='socketcan')
    
    def write(self, data):
        """Send data as CAN message"""
        # Your protocol-specific encoding here
        # This depends on how your serial protocol maps to CAN
        pass
    
    def read(self, size=1):
        """Receive CAN messages"""
        msg = self.bus.recv(timeout=0.001)
        if msg:
            return msg.data
        return b''
    
    def close(self):
        self.bus.shutdown()
```

#### Option 2: Keep Serial Interface (Temporary)

If you want to keep using your current code with minimal changes:

```bash
# Create virtual serial port that forwards to CAN
# This requires slcand (serial line CAN daemon)

sudo slcand -o -c -s6 can0 /dev/ttyVCAN0
sudo ifconfig can0 up
```

Then use `/dev/ttyVCAN0` as your serial port - but **native CAN is better!**

### Recommended: Native CAN Implementation

I can help you modify `can_motor_controller.py` to use native SocketCAN instead of serial. This will be:
- ✅ Faster
- ✅ More reliable  
- ✅ Better error handling
- ✅ Lower latency

Would you like me to create this modified version?

---

## Troubleshooting

### CAN interface doesn't appear

```bash
# Check if CAN driver is loaded
lsmod | grep can

# If not loaded, manually load
sudo modprobe can
sudo modprobe can_raw
sudo modprobe mttcan

# Check device tree
ls /proc/device-tree/ | grep can
```

### Permission denied

```bash
# Add user to dialout group (for serial) and netdev (for CAN)
sudo usermod -a -G dialout,netdev $USER

# Or run commands with sudo
sudo ip link set can0 up
```

### CAN bus errors

```bash
# Check error counters
ip -details -statistics link show can0

# Reset CAN interface
sudo ip link set can0 down
sudo ip link set can0 up

# Check for bus-off state
candump can0 -e  # -e shows errors
```

### Motors not responding

1. **Check wiring**: CANH to all yellow, CANL to all green
2. **Check termination**: 120Ω at both ends of bus
3. **Check motor power**: Motors must be powered ON
4. **Check bitrate**: Must be 500 kbps
5. **Check motor IDs**: Should be 1-10 (0x001-0x00A)

---

## Summary

✅ **Hardware**: Connect MCP2551 transceiver to Pins 37, 40, and GND  
✅ **Software**: Enable CAN0 with `setup_can.sh`  
✅ **Testing**: Use `candump` and `cansend` to verify  
✅ **Integration**: Modify Python code to use `python-can`  

**You don't need the STM32F7!** Your Jetson already has everything you need built-in.

---

## Next Steps

1. **Wire up the MCP2551** transceiver to your Jetson
2. **Run the setup script** to enable CAN0
3. **Test with `candump/cansend`** to verify hardware works
4. **Let me know** and I'll help you modify `can_motor_controller.py` to use native CAN instead of serial!

This will give you the **fastest and most reliable** motor control possible! 🚀
