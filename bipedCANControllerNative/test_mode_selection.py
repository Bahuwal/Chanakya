#!/usr/bin/env python3
"""
Quick test to verify mode selection works
"""

print("=" * 60)
print("CAN Motor Control - Select Control Mode")
print("=" * 60)
print("1. Servo Mode (with inner-loop PID)")
print("   - Uses: kp, kd, ikp, ikd, iki parameters")
print("   - Best for: Standard position control")
print("")
print("2. PTM Mode (Position-Torque Mix)")
print("   - Uses: kp, kd + feed-forward torque")
print("   - Best for: Position control with gravity compensation")
print("   - Formula: T = Kp*(Pref-Pact) + Kd*(Vref-Vact) + Tref")
print("=" * 60)

while True:
    choice = input("Enter mode (1 or 2): ").strip()
    if choice == "1":
        control_mode = "servo"
        break
    elif choice == "2":
        control_mode = "ptm"
        break
    else:
        print("Invalid choice. Please enter 1 or 2.")

print(f"\n✓ Selected control mode: {control_mode.upper()}\n")
print("Mode selection works correctly!")
