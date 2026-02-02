#!/usr/bin/env python3
"""
CAN Transceiver Test - Monitor Motor Commands

This script sends motor commands via CAN and prints what's being transmitted.
Use with candump to verify transceiver output:

Terminal 1: candump can0
Terminal 2: python3 can_transceiver_test.py 3

You should see the same messages in both terminals!
"""

import can
import time
import sys

# Motor command definitions
CMD_MOTOR_MODE = 0xFC
CMD_RESET = 0xFD
CMD_ZERO = 0xFE


def print_can_frame(msg, description=""):
    """Print CAN frame in readable format"""
    print(f"\n{'='*70}")
    if description:
        print(f"Command: {description}")
    print(f"CAN ID: 0x{msg.arbitration_id:03X} (Motor {msg.arbitration_id})")
    print(f"Data:   {' '.join([f'{b:02X}' for b in msg.data])}")
    print(f"ASCII:  can0  {msg.arbitration_id:03X}   [{len(msg.data)}]  {' '.join([f'{b:02X}' for b in msg.data])}")
    print(f"{'='*70}")


def send_motor_mode(bus, motor_id):
    """Send Motor Mode Enable (0xFC)"""
    data = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, CMD_MOTOR_MODE]
    msg = can.Message(arbitration_id=motor_id, data=data, is_extended_id=False)
    
    print_can_frame(msg, f"MOTOR MODE ENABLE (0xFC) - Motor {motor_id}")
    bus.send(msg)
    return msg


def send_reset(bus, motor_id):
    """Send Reset Command (0xFD)"""
    data = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, CMD_RESET]
    msg = can.Message(arbitration_id=motor_id, data=data, is_extended_id=False)
    
    print_can_frame(msg, f"RESET (0xFD) - Motor {motor_id}")
    bus.send(msg)
    return msg


def send_zero_position(bus, motor_id):
    """Send Zero Position (0xFE)"""
    data = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, CMD_ZERO]
    msg = can.Message(arbitration_id=motor_id, data=data, is_extended_id=False)
    
    print_can_frame(msg, f"ZERO POSITION (0xFE) - Motor {motor_id}")
    bus.send(msg)
    return msg


def send_ptm_command(bus, motor_id, pos_uint, vel_uint, kp_uint, kd_uint):
    """Send PTM control command"""
    data = [
        pos_uint & 0xFF, (pos_uint >> 8) & 0xFF,
        vel_uint & 0xFF, (vel_uint >> 8) & 0xFF,
        kp_uint & 0xFF, (kp_uint >> 8) & 0xFF,
        kd_uint & 0xFF, (kd_uint >> 8) & 0xFF
    ]
    msg = can.Message(arbitration_id=motor_id, data=data, is_extended_id=False)
    
    print_can_frame(msg, f"PTM CONTROL - Motor {motor_id}")
    print(f"  Position: 0x{pos_uint:04X}")
    print(f"  Velocity: 0x{vel_uint:04X}")
    print(f"  Kp: 0x{kp_uint:04X}")
    print(f"  Kd: 0x{kd_uint:04X}")
    
    bus.send(msg)
    return msg


def main():
    motor_id = int(sys.argv[1]) if len(sys.argv) > 1 else 3
    
    print("="*70)
    print("CAN TRANSCEIVER TEST - Motor Command Monitor")
    print("="*70)
    print(f"Motor ID: {motor_id}")
    print(f"Interface: can0 @ 1Mbps")
    print()
    print("INSTRUCTIONS:")
    print("  1. In another terminal, run: candump can0")
    print("  2. Watch for matching messages in candump output")
    print("  3. Verify transceiver LEDs are blinking (if present)")
    print("="*70)
    
    # Open CAN bus
    try:
        bus = can.Bus(channel='can0', interface='socketcan', bitrate=1000000)
        print("\n✓ CAN interface opened successfully")
    except Exception as e:
        print(f"\n✗ Failed to open CAN interface: {e}")
        print("\nMake sure CAN is configured:")
        print("  sudo ip link set can0 type can bitrate 1000000")
        print("  sudo ip link set can0 up")
        return 1
    
    try:
        input("\nPress ENTER to start sending commands...")
        
        # Test 1: Reset
        print("\n\n" + "="*70)
        print("TEST 1: Sending RESET command")
        print("="*70)
        send_reset(bus, motor_id)
        time.sleep(1)
        
        # Test 2: Motor Mode
        print("\n\n" + "="*70)
        print("TEST 2: Sending MOTOR MODE command")
        print("="*70)
        send_motor_mode(bus, motor_id)
        time.sleep(1)
        
        # Test 3: Zero Position
        print("\n\n" + "="*70)
        print("TEST 3: Sending ZERO POSITION command")
        print("="*70)
        send_zero_position(bus, motor_id)
        time.sleep(1)
        
        # Test 4: PTM Commands
        print("\n\n" + "="*70)
        print("TEST 4: Sending PTM CONTROL commands (10 frames)")
        print("="*70)
        
        # Example PTM values (position = 0x8000 (neutral), kp = 0x1000, kd = 0x0100)
        for i in range(10):
            pos_uint = 0x8000  # Neutral position
            vel_uint = 0x8000  # Zero velocity
            kp_uint = 0x1000   # Low gain
            kd_uint = 0x0100   # Low damping
            
            send_ptm_command(bus, motor_id, pos_uint, vel_uint, kp_uint, kd_uint)
            time.sleep(0.1)  # 10Hz
        
        # Test 5: Final Reset
        print("\n\n" + "="*70)
        print("TEST 5: Sending final RESET command")
        print("="*70)
        send_reset(bus, motor_id)
        
        print("\n\n" + "="*70)
        print("✓ TEST COMPLETE!")
        print("="*70)
        print("\nIf you saw matching messages in candump, your transceiver is working!")
        print("\nYou should have seen:")
        print(f"  - can0  {motor_id:03X}   [8]  FF FF FF FF FF FF FF FD  (Reset)")
        print(f"  - can0  {motor_id:03X}   [8]  FF FF FF FF FF FF FF FC  (Motor Mode)")
        print(f"  - can0  {motor_id:03X}   [8]  FF FF FF FF FF FF FF FE  (Zero)")
        print(f"  - can0  {motor_id:03X}   [8]  00 80 00 80 00 10 00 01  (PTM)")
        print("="*70)
        
        return 0
        
    except KeyboardInterrupt:
        print("\n\n✗ Test interrupted")
        return 1
    
    finally:
        bus.shutdown()


if __name__ == "__main__":
    exit(main())
