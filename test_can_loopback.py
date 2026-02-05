#!/usr/bin/env python3
"""
Simple CAN loopback test
Sends a message and verifies it receives it back
"""
import can
import time

print("="*60)
print("CAN LOOPBACK TEST")
print("="*60)

try:
    # Open CAN bus
    bus = can.Bus(channel='can0', interface='socketcan', bitrate=1000000)
    print("✓ CAN bus opened")
    
    # Create test message
    test_msg = can.Message(
        arbitration_id=0x123,
        data=[0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88],
        is_extended_id=False
    )
    
    print(f"\nSending: ID=0x{test_msg.arbitration_id:03X}, Data={test_msg.data.hex()}")
    
    # Send message
    bus.send(test_msg)
    print("✓ Message sent")
    
    # Try to receive it back
    print("\nWaiting for loopback...")
    received_msg = bus.recv(timeout=1.0)
    
    if received_msg:
        print(f"✓ Received: ID=0x{received_msg.arbitration_id:03X}, Data={received_msg.data.hex()}")
        
        # Verify it matches
        if (received_msg.arbitration_id == test_msg.arbitration_id and 
            received_msg.data == test_msg.data):
            print("\n" + "="*60)
            print("✅ LOOPBACK TEST PASSED!")
            print("="*60)
        else:
            print("\n❌ Data mismatch!")
    else:
        print("\n❌ No message received (timeout)")
        print("Check: sudo ip link show can0")
        print("Should show LOOPBACK flag")
        
except Exception as e:
    print(f"\n❌ Error: {e}")
    
finally:
    try:
        bus.shutdown()
    except:
        pass

print("\nTo disable loopback:")
print("  sudo ip link set can0 down")
print("  sudo ip link set can0 type can bitrate 1000000")
print("  sudo ip link set can0 up")
