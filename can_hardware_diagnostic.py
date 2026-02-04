#!/usr/bin/env python3
"""
CAN Hardware Diagnostic Tool
Tests CAN interface at hardware and software levels to identify RX/TX issues
"""

import can
import time
import subprocess
import os

def check_can_interface_status():
    """Check the current status of CAN interface"""
    print("\n" + "="*60)
    print("1. CAN INTERFACE STATUS")
    print("="*60)
    
    try:
        result = subprocess.run(['ip', 'link', 'show', 'can0'], 
                              capture_output=True, text=True, timeout=5)
        print(result.stdout)
        
        # Check for LOOPBACK mode (this could cause echo issues)
        if 'LOOPBACK' in result.stdout:
            print("⚠️  WARNING: LOOPBACK mode is ENABLED - this causes TX echo!")
        else:
            print("✓ LOOPBACK mode is disabled")
            
    except Exception as e:
        print(f"❌ Error checking interface: {e}")

def check_can_statistics():
    """Check CAN interface error statistics"""
    print("\n" + "="*60)
    print("2. CAN INTERFACE STATISTICS")
    print("="*60)
    
    try:
        result = subprocess.run(['ip', '-details', '-statistics', 'link', 'show', 'can0'], 
                              capture_output=True, text=True, timeout=5)
        print(result.stdout)
    except Exception as e:
        print(f"❌ Error checking statistics: {e}")

def test_can_loopback():
    """Test CAN in loopback mode to verify TX/RX hardware"""
    print("\n" + "="*60)
    print("3. LOOPBACK TEST (verifies CAN controller RX works)")
    print("="*60)
    
    # Enable loopback mode temporarily
    print("Setting up loopback mode...")
    try:
        subprocess.run(['sudo', 'ip', 'link', 'set', 'can0', 'down'], timeout=5)
        subprocess.run(['sudo', 'ip', 'link', 'set', 'can0', 'type', 'can', 
                       'bitrate', '1000000', 'loopback', 'on'], timeout=5)
        subprocess.run(['sudo', 'ip', 'link', 'set', 'can0', 'up'], timeout=5)
        time.sleep(0.5)
        
        # Create CAN bus
        bus = can.interface.Bus(channel='can0', bustype='socketcan')
        
        # Send a test message
        test_msg = can.Message(arbitration_id=0x123, data=[0x11, 0x22, 0x33], is_extended_id=False)
        print(f"Sending test message: ID=0x{test_msg.arbitration_id:03X}, Data={test_msg.data.hex()}")
        bus.send(test_msg)
        
        # Try to receive it back
        print("Listening for loopback message...")
        msg = bus.recv(timeout=1.0)
        
        if msg:
            print(f"✓ RECEIVED: ID=0x{msg.arbitration_id:03X}, Data={msg.data.hex()}")
            print("✓ CAN controller RX capability confirmed!")
        else:
            print("❌ No loopback received - CAN controller RX may be broken")
        
        bus.shutdown()
        
        # Restore normal mode
        print("\nRestoring normal mode...")
        subprocess.run(['sudo', 'ip', 'link', 'set', 'can0', 'down'], timeout=5)
        subprocess.run(['sudo', 'ip', 'link', 'set', 'can0', 'type', 'can', 
                       'bitrate', '1000000', 'loopback', 'off'], timeout=5)
        subprocess.run(['sudo', 'ip', 'link', 'set', 'can0', 'up'], timeout=5)
        
    except Exception as e:
        print(f"❌ Loopback test failed: {e}")

def monitor_can_traffic(duration=10):
    """Monitor CAN bus and categorize TX vs RX traffic"""
    print("\n" + "="*60)
    print("4. LIVE CAN TRAFFIC MONITOR")
    print("="*60)
    print(f"Monitoring for {duration} seconds...")
    print("Send some motor commands while this runs!\n")
    
    try:
        bus = can.interface.Bus(channel='can0', bustype='socketcan')
        
        tx_count = 0
        rx_count = 0
        seen_ids = set()
        
        start_time = time.time()
        last_msg_time = {}
        
        while time.time() - start_time < duration:
            msg = bus.recv(timeout=0.1)
            if msg:
                arb_id = msg.arbitration_id
                seen_ids.add(arb_id)
                
                # Check if this looks like an echo (same ID repeated quickly)
                now = time.time()
                if arb_id in last_msg_time:
                    delta = now - last_msg_time[arb_id]
                    if delta < 0.01:  # Less than 10ms - likely echo
                        tx_count += 1
                        print(f"TX (echo): ID=0x{arb_id:03X}, Data={msg.data.hex()}, Δt={delta*1000:.1f}ms")
                    else:
                        rx_count += 1
                        print(f"RX (motor): ID=0x{arb_id:03X}, Data={msg.data.hex()}, Δt={delta*1000:.1f}ms")
                else:
                    tx_count += 1
                    print(f"TX (first): ID=0x{arb_id:03X}, Data={msg.data.hex()}")
                
                last_msg_time[arb_id] = now
        
        bus.shutdown()
        
        print("\n" + "-"*60)
        print(f"Total messages: {tx_count + rx_count}")
        print(f"TX (sent/echo): {tx_count}")
        print(f"RX (from motor): {rx_count}")
        print(f"Unique CAN IDs seen: {[hex(x) for x in sorted(seen_ids)]}")
        
        if rx_count == 0:
            print("\n❌ NO RX MESSAGES DETECTED!")
            print("   This confirms motor is NOT sending feedback on CAN bus")
            print("   OR there is a hardware RX problem")
        else:
            print(f"\n✓ Detected {rx_count} RX messages from motor")
            
    except Exception as e:
        print(f"❌ Monitoring failed: {e}")

def check_can_transceiver():
    """Check for common CAN transceiver issues"""
    print("\n" + "="*60)
    print("5. HARDWARE CHECKS")
    print("="*60)
    
    print("\nPlease verify the following physically:")
    print("□ CAN_H and CAN_L are properly connected to motor")
    print("□ 120Ω termination resistor is present (measure ~60Ω between CAN_H and CAN_L)")
    print("□ CAN transceiver power supply is connected and stable")
    print("□ Ground reference is shared between Jetson and motor")
    print("□ Jason USB adapter (if used) is properly configured")
    
    print("\nFor Jetson GPIO CAN:")
    print("□ Check /boot/extlinux/extlinux.conf for CAN overlay")
    print("□ Verify CAN TX/RX pins are not used by other peripherals")
    print("□ Measure voltage on CAN_H (~3.5V idle) and CAN_L (~1.5V idle)")

def main():
    print("\n" + "="*60)
    print("CAN HARDWARE DIAGNOSTIC TOOL")
    print("="*60)
    
    # Run all diagnostics
    check_can_interface_status()
    check_can_statistics()
    test_can_loopback()
    
    print("\n\nNow we'll monitor live traffic.")
    print("Please run your motor control script in another terminal...")
    input("Press Enter when ready to start monitoring...")
    
    monitor_can_traffic(duration=15)
    check_can_transceiver()
    
    print("\n" + "="*60)
    print("DIAGNOSTIC COMPLETE")
    print("="*60)
    print("\nNext steps based on results:")
    print("1. If loopback test FAILED → CAN controller hardware issue")
    print("2. If loopback OK but NO RX in monitor → Check physical wiring/transceiver")
    print("3. If only seeing echoes → Motor may not support CAN feedback in this mode")
    print("4. If seeing RX messages → Great! Check if they're on expected CAN ID")

if __name__ == "__main__":
    main()
