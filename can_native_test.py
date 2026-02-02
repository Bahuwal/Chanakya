#!/usr/bin/env python3
"""
Quick test script for native CAN control
Tests single motor with PTM mode using SocketCAN

Usage:
    python3 can_native_test.py [motor_id]

Example:
    python3 can_native_test.py 3
"""

import sys
import time

# Import native CAN controller
from bipedCANController.can_native_controller import CANMotorController, Motor

def main():
    # Get motor ID from command line
    motor_id = int(sys.argv[1]) if len(sys.argv) > 1 else 3
    
    print("="*70)
    print(f"Native CAN Test - Motor ID {motor_id}")
    print("="*70)
    print("Hardware: Jetson J17 + SN65HVD230 + Motors")
    print("Interface: can0 @ 500kbps")
    print("="*70)
    
    # Create controller
    try:
        controller = CANMotorController(channel='can0', bitrate=500000)
    except Exception as e:
        print(f"\n✗ Failed to initialize CAN!")
        print(f"\nTroubleshooting:")
        print(f"  1. Check if CAN is enabled:")
        print(f"     $ ls /sys/class/net/ | grep can")
        print(f"  2. Bring up CAN interface:")
        print(f"     $ sudo ip link set can0 type can bitrate 500000")
        print(f"     $ sudo ip link set can0 up")
        print(f"  3. Verify device tree:")
        print(f"     $ sudo /opt/nvidia/jetson-io/jetson-io.py")
        return 1
    
    # Create motor
    motor = Motor(f"motor_{motor_id}", motor_id=motor_id, type_name="REVO")
    controller.add_motor(motor)
    
    # Start receiver thread
    controller.start()
    time.sleep(0.2)
    
    try:
        # Enable motor mode
        print(f"\n✓ Enabling motor {motor_id}...")
        controller.motor_mode(motor)
        time.sleep(0.5)
        
        # Test parameters
        target_pos = 2.0  # rad
        kp = 50.0
        kd = 1.0
        
        print(f"\n✓ Sending PTM commands (target: {target_pos} rad, Kp: {kp}, Kd: {kd})...")
        print(f"{'Time (s)':>8} | {'Position':>10} | {'Velocity':>10} | {'Torque':>10} | {'Temp':>6}")
        print(f"{'-'*70}")
        
        start_time = time.time()
        
        for i in range(50):  # 5 seconds @ 100Hz
            # Send control command
            controller.PTM_control(motor, pos=target_pos, vel=0.0, 
                                  kp=kp, kd=kd, torque=0.0)
            
            # Print feedback every 10 iterations
            if i % 10 == 0:
                t = time.time() - start_time
                print(f"{t:>8.2f} | {motor.pos:>9.3f} rad | {motor.vel:>9.3f} rad/s | "
                      f"{motor.torque:>9.3f} Nm | {motor.temperature:>4d}°C")
            
            time.sleep(0.01)  # 100Hz control loop
        
        # Return to zero
        print(f"\n✓ Returning to zero...")
        for i in range(30):
            controller.PTM_control(motor, pos=0.0, vel=0.0, 
                                  kp=kp, kd=kd, torque=0.0)
            time.sleep(0.01)
        
        # Reset motor
        print(f"\n✓ Resetting motor {motor_id}...")
        controller.reset_mode(motor)
        time.sleep(0.2)
        
        print(f"\n{'='*70}")
        print(f"✓ Test complete!")
        print(f"  Final position: {motor.pos:.3f} rad")
        print(f"  Final velocity: {motor.vel:.3f} rad/s")
        print(f"  Temperature: {motor.temperature}°C")
        print(f"{'='*70}")
        
        return 0
        
    except KeyboardInterrupt:
        print(f"\n\n✗ Test interrupted by user")
        controller.reset_mode(motor)
        return 1
    
    finally:
        controller.stop()


if __name__ == "__main__":
    exit(main())
