#!/usr/bin/env python3
"""
Raw CAN RX Monitor - Just prints everything received on CAN0
No processing, no TX, just raw RX data
"""
import can
import time

print("="*70)
print("RAW CAN RX MONITOR - CAN0")
print("="*70)
print("Listening for ALL messages on can0...")
print("Press Ctrl+C to stop")
print("="*70 + "\n")

try:
    # Open CAN bus
    bus = can.Bus(channel='can0', interface='socketcan', bitrate=1000000)
    
    msg_count = 0
    start_time = time.time()
    
    while True:
        # Receive message with 1 second timeout
        msg = bus.recv(timeout=1.0)
        
        if msg:
            msg_count += 1
            elapsed = time.time() - start_time
            
            # Print raw message
            print(f"[{elapsed:7.3f}s] #{msg_count:4d} | "
                  f"ID: 0x{msg.arbitration_id:03X} | "
                  f"DLC: {msg.dlc} | "
                  f"Data: {msg.data.hex()} | "
                  f"Raw bytes: {' '.join([f'{b:02X}' for b in msg.data])}")

except KeyboardInterrupt:
    print(f"\n\n{'='*70}")
    print(f"Stopped. Received {msg_count} messages in {time.time() - start_time:.1f} seconds")
    print("="*70)

except Exception as e:
    print(f"\nError: {e}")

finally:
    try:
        bus.shutdown()
    except:
        pass
