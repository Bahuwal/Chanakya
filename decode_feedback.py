#!/usr/bin/env python3
"""
Decode the motor feedback data to see what values we're actually getting
"""

def uint_to_float(x_int, x_min, x_max, bits):
    """Convert unsigned int to float"""
    span = x_max - x_min
    offset = x_min
    return ((float(x_int) * span) / float((1 << bits) - 1)) + offset

# ACTUAL feedback data from CAN diagnostics
print("="*70)
print("ANALYZING REAL MOTOR FEEDBACK DATA")
print("="*70)

# Test multiple samples
samples = [
    "0281eb7fd7ff0019",  # Motor at rest (0.187 rad)
    "02b2b2a1e77d0019",  # Motor at target (4.951 rad)
    "0280be7ff8150019",  # Motor stuck (0.073 rad)
]

for i, hex_data in enumerate(samples, 1):
    data = bytes.fromhex(hex_data)
    
    print(f"\n{'='*70}")
    print(f"Sample {i}: {hex_data}")
    print(f"{'='*70}")
    
    # According to Motorevo manual:
    # Byte 0: Motor ID
    # Byte 1-2: Position (16-bit)
    # Byte 3 + Byte4[7:4]: Velocity (12-bit)
    # Byte4[3:0] + Byte5: Torque (12-bit)
    # Byte 6: Temperature
    # Byte 7: Error code
    
    print("\nManual Format (Byte 0 = Motor ID):")
    motor_id = data[0]
    q_uint = (data[1] << 8) | data[2]
    dq_uint = (data[3] << 4) | (data[4] >> 4)
    tau_uint = ((data[4] & 0x0F) << 8) | data[5]
    temp = data[6]
    error = data[7]
    
    pos = uint_to_float(q_uint, -12.5, 12.5, 16)
    vel = uint_to_float(dq_uint, -10.0, 10.0, 12)
    tau = uint_to_float(tau_uint, -50.0, 50.0, 12)
    
    print(f"  Motor ID: {motor_id} (0x{motor_id:02X})")
    print(f"  Position: {q_uint:5d} → {pos:7.3f} rad")
    print(f"  Velocity: {dq_uint:4d} → {vel:7.3f} rad/s")
    print(f"  Torque:   {tau_uint:4d} → {tau:7.3f} Nm")
    print(f"  Temp:     {temp}°C")
    print(f"  Error:    0x{error:02X} ({error})")
    
    # Raw bytes
    print("\n  Raw bytes:")
    for j, byte in enumerate(data):
        print(f"    Byte {j}: 0x{byte:02X} ({byte:3d}) = {byte:08b}b")

print(f"\n{'='*70}")
print("KEY OBSERVATIONS:")
print("="*70)
print("1. Byte 0 is consistently 0x02 (Motor ID 2?) but we're controlling ID 3")
print("2. Byte 7 is 0x19 (25) - this error code doesn't exist in manual!")
print("3. Temperature is always 0°C - seems wrong")
print("4. The position values DO change (0.187 → 4.951 → 0.073)")
print("\nPOSSIBLE ISSUES:")
print("- Wrong CAN ID being received")
print("- Byte 7 might be checksum/packet type, not error code")
print("- Need to verify actual motor ID in feedback vs command")
print("="*70)
