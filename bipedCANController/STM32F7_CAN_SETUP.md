# STM32F7 CAN Communication Setup Guide

## Overview

This guide explains how to use an STM32F7 board as a USB-to-CAN bridge to communicate with Motorevo motors, replacing the damaged Jetson module.

## System Architecture

```
┌──────────────┐  USB Serial   ┌────────────┐   CAN Bus   ┌─────────────┐
│   Jetson/PC  │ ◄──────────► │  STM32F7   │ ◄─────────► │ Motorevo    │
│              │  /dev/ttyACM0 │  CAN Bridge│   500kbps   │ Motors x10  │
└──────────────┘               └────────────┘             └─────────────┘
```

## Hardware Setup

### 1. STM32F7 Board Connections

#### CAN Bus Pins (STM32F746):
- **CAN1_TX**: PB9 or PD1
- **CAN1_RX**: PB8 or PD0
- **GND**: Connect to CAN bus ground

#### Recommended Pin Configuration:
```
STM32F7 Pin      →  Function
────────────────────────────
PB8 (CAN1_RX)    →  CAN RX (to CAN transceiver)
PB9 (CAN1_TX)    →  CAN TX (to CAN transceiver)
5V               →  CAN transceiver VCC
GND              →  CAN transceiver & bus GND
```

### 2. CAN Transceiver (Required!)

You need a CAN transceiver chip (e.g., **MCP2551**, **TJA1050**, or **SN65HVD230**):

```
STM32F7          CAN Transceiver        CAN Bus
────────         ───────────────        ───────
PB9 (TX)    →    TXD (Pin 1)
PB8 (RX)    ←    RXD (Pin 4)
5V          →    VCC (Pin 3)
GND         →    GND (Pin 2)
                 CANH (Pin 7)      →    CAN_H (Yellow)
                 CANL (Pin 6)      →    CAN_L (Green)
```

### 3. Motor CAN Bus Wiring

Connect all 10 motors in parallel on the CAN bus:
```
CAN_H (Yellow) ──┬── Motor 1
                 ├── Motor 2
                 ├── ...
                 └── Motor 10

CAN_L (Green)  ──┬── Motor 1
                 ├── Motor 2
                 ├── ...
                 └── Motor 10

GND ─────────────┬── Motor 1
                 ├── Motor 2
                 ├── ...
                 └── Motor 10
```

**Important**: Add 120Ω termination resistors at both ends of the CAN bus!

## STM32F7 Firmware

### Required Features:
1. **USB CDC (Virtual COM Port)** - for serial communication with Jetson/PC
2. **CAN1 Interface** - configured at 500 kbps
3. **Message forwarding** - bidirectional between USB and CAN

### STM32CubeMX Configuration:

#### 1. Enable Peripherals:
- **USB_OTG_FS** → Mode: Device Only, Class: CDC (Virtual COM Port)
- **CAN1** → Mode: Master, Bitrate: 500 kbps
- **USART/UART** (optional) → for debugging

#### 2. CAN Configuration:
```c
// CAN bitrate settings for 500 kbps @ 216 MHz APB1
Prescaler: 27
Time Quanta in Bit Segment 1: 13
Time Quanta in Bit Segment 2: 2
Time Quantum: 125 ns
Bit Time: 2 μs (500 kbps)
```

#### 3. GPIO Configuration:
- **PB8**: CAN1_RX (Alternate Function)
- **PB9**: CAN1_TX (Alternate Function)

### Sample Firmware Code:

#### main.c (Simplified):
```c
#include "main.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

CAN_HandleTypeDef hcan1;
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
uint8_t TxData[8];
uint8_t RxData[8];
uint32_t TxMailbox;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_CAN1_Init();
    MX_USB_DEVICE_Init();
    
    // Start CAN
    HAL_CAN_Start(&hcan1);
    
    // Activate CAN RX notification
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    
    while (1) {
        // Main loop - USB CDC and CAN handled by interrupts
        HAL_Delay(10);
    }
}

// CAN1 Initialization
static void MX_CAN1_Init(void) {
    hcan1.Instance = CAN1;
    hcan1.Init.Prescaler = 27;
    hcan1.Init.Mode = CAN_MODE_NORMAL;
    hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
    hcan1.Init.TimeSeg1 = CAN_BS1_13TQ;
    hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
    hcan1.Init.TimeTriggeredMode = DISABLE;
    hcan1.Init.AutoBusOff = ENABLE;
    hcan1.Init.AutoWakeUp = DISABLE;
    hcan1.Init.AutoRetransmission = ENABLE;
    hcan1.Init.ReceiveFifoLocked = DISABLE;
    hcan1.Init.TransmitFifoPriority = DISABLE;
    
    if (HAL_CAN_Init(&hcan1) != HAL_OK) {
        Error_Handler();
    }
    
    // Configure CAN filter to accept all messages
    CAN_FilterTypeDef sFilterConfig;
    sFilterConfig.FilterBank = 0;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh = 0x0000;
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0x0000;
    sFilterConfig.FilterMaskIdLow = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;
    
    HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);
}

// USB CDC Receive Callback - Forward to CAN
void CDC_Receive_Callback(uint8_t* Buf, uint32_t Len) {
    // Parse USB data and send to CAN
    // Format: [ID (2 bytes)] [DLC (1 byte)] [Data (0-8 bytes)]
    
    if (Len >= 3) {
        uint16_t can_id = (Buf[0] << 8) | Buf[1];
        uint8_t dlc = Buf[2];
        
        TxHeader.StdId = can_id;
        TxHeader.ExtId = 0;
        TxHeader.IDE = CAN_ID_STD;
        TxHeader.RTR = CAN_RTR_DATA;
        TxHeader.DLC = dlc;
        
        // Copy data
        for (int i = 0; i < dlc && i < 8; i++) {
            TxData[i] = Buf[3 + i];
        }
        
        // Send CAN message
        HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
    }
}

// CAN RX Interrupt Callback - Forward to USB
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
        // Format CAN message for USB
        uint8_t usb_buf[11];  // Max: 2 (ID) + 1 (DLC) + 8 (Data)
        
        usb_buf[0] = (RxHeader.StdId >> 8) & 0xFF;
        usb_buf[1] = RxHeader.StdId & 0xFF;
        usb_buf[2] = RxHeader.DLC;
        
        for (int i = 0; i < RxHeader.DLC; i++) {
            usb_buf[3 + i] = RxData[i];
        }
        
        // Send to USB CDC
        CDC_Transmit_FS(usb_buf, 3 + RxHeader.DLC);
    }
}
```

## Python Code Changes (Minimal!)

### Good News: Almost No Changes Needed!

Your current `can_motor_controller.py` already uses serial communication, so you just need to:

1. **Change the serial port** in your config or command line:
   ```yaml
   # can_config.yaml
   motor_serial_port: "/dev/ttyACM0"  # STM32F7 appears as ACM device
   ```

2. **Or specify when running**:
   ```bash
   python3 biped_controller_can.py --motor-port /dev/ttyACM0
   ```

### Finding the STM32F7 Port:

```bash
# Before plugging in STM32F7
ls /dev/ttyACM* /dev/ttyUSB*

# Plug in STM32F7

# After plugging in
ls /dev/ttyACM* /dev/ttyUSB*
# New device will appear (usually /dev/ttyACM0)

# Check device info
udevadm info --name=/dev/ttyACM0 | grep ID_VENDOR
```

## Testing the Setup

### 1. Test USB Connection:
```bash
# Install minicom
sudo apt-get install minicom

# Connect to STM32F7
minicom -D /dev/ttyACM0 -b 921600

# You should see data when motors respond
```

### 2. Test with Single Motor:
```bash
cd ~/Chanakya/bipedCANController
python3 can_trajectory_PD_test.py --motor-port /dev/ttyACM0
```

### 3. Full System Test:
```bash
python3 biped_controller_can.py --motor-port /dev/ttyACM0
```

## Troubleshooting

### Issue: No /dev/ttyACM0 appears
**Solution**: 
- Check USB cable (must support data, not just power)
- Verify USB CDC is enabled in STM32 firmware
- Try different USB port on Jetson

### Issue: Permission denied
**Solution**:
```bash
sudo chmod 666 /dev/ttyACM0
# Or add user to dialout group
sudo usermod -a -G dialout $USER
# Then logout and login again
```

### Issue: CAN messages not received
**Solution**:
- Check CAN transceiver connections
- Verify 120Ω termination resistors at both ends
- Confirm CAN bitrate is 500 kbps
- Use oscilloscope to check CANH/CANL signals

### Issue: Motor commands work but no feedback
**Solution**:
- Check CAN RX interrupt is enabled
- Verify CAN filter accepts motor response IDs (0x200-0x209)
- Add debug LED toggle in CAN RX callback

## Alternative: Pre-built Firmware

If you don't want to write firmware, you can use existing USB-to-CAN adapters like:
- **PCAN-USB** (Peak Systems)
- **SLCAN** compatible devices
- **CANable** (open-source)

Then use Python `python-can` library instead of serial.

## Summary

✅ **Hardware**: STM32F7 + CAN transceiver + termination resistors  
✅ **Firmware**: USB CDC + CAN forwarding at 500 kbps  
✅ **Python**: Change port from `/dev/ttyUSB0` to `/dev/ttyACM0`  
✅ **Testing**: Use minicom and existing test scripts  

The STM32F7 acts as a transparent USB-to-CAN bridge - your Python code doesn't need to know the difference!
