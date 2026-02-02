"""
Native CAN Motor Controller using Linux SocketCAN

This module provides direct CAN bus communication for Motorevo motors
using the Jetson Orin Nano Super's built-in CAN controller.

Advantages over USB adapters:
- Lower latency (~0.5ms vs 2-5ms)
- No USB protocol overhead
- Direct hardware access
- Standard Linux SocketCAN interface

Hardware: J17 CAN header + SN65HVD230 transceiver
"""

import can
import time
import threading
import sys
import os

# Import conversion utilities
sys.path.insert(0, os.path.dirname(__file__))
from utils import float_to_uint, uint_to_float


class Motor:
    """Motor object for native CAN control"""
    
    def __init__(self, name, motor_id, type_name="REVO"):
        self.name = name
        self.id = motor_id  # CAN ID (1-10 for your motors)
        self.type_name = type_name
        
        # Motor state (updated by feedback)
        self.pos = 0.0
        self.vel = 0.0
        self.torque = 0.0
        self.temperature = 0
        self.status_words = 0
        
        # Limits (same as USB version)
        self.P_MIN = -12.5
        self.P_MAX = 12.5
        self.V_MIN = -30
        self.V_MAX = 30
        self.T_MIN = -12
        self.T_MAX = 12
        self.KP_MIN = 0
        self.KP_MAX = 500
        self.KD_MIN = 0
        self.KD_MAX = 5


class CANMotorController:
    """Motor controller using Linux SocketCAN interface"""
    
    def __init__(self, channel='can0', bitrate=500000):
        """
        Initialize CAN motor controller
        
        Args:
            channel: CAN interface name (e.g., 'can0')
            bitrate: CAN bus speed in bps (500000 for Motorevo)
        """
        self.channel = channel
        self.bitrate = bitrate
        self.motors = {}
        
        # Initialize CAN bus
        try:
            self.bus = can.Bus(channel=channel, interface='socketcan', 
                              bitrate=bitrate, receive_own_messages=False)
            print(f"✓ CAN interface '{channel}' opened at {bitrate} bps")
        except OSError as e:
            print(f"✗ Failed to open CAN interface '{channel}'")
            print(f"  Error: {e}")
            print(f"\n  Make sure:")
            print(f"  1. CAN is enabled in device tree (run: sudo /opt/nvidia/jetson-io/jetson-io.py)")
            print(f"  2. Interface is up (run: sudo ip link set {channel} type can bitrate {bitrate})")
            print(f"  3.                      sudo ip link set {channel} up)")
            raise
        except Exception as e:
            print(f"✗ Unexpected error opening CAN: {e}")
            raise
        
        # Background receiver thread
        self.running = False
        self.rx_thread = None
    
    def add_motor(self, motor):
        """Add motor to controller"""
        self.motors[motor.id] = motor
        print(f"  Added motor '{motor.name}' with CAN ID {motor.id}")
    
    def start(self):
        """Start background receiver thread"""
        if self.running:
            return
        
        self.running = True
        self.rx_thread = threading.Thread(target=self._receive_loop, daemon=True)
        self.rx_thread.start()
        print(f"✓ CAN receiver thread started")
    
    def stop(self):
        """Stop receiver thread and close bus"""
        self.running = False
        if self.rx_thread:
            self.rx_thread.join(timeout=1.0)
        self.bus.shutdown()
        print(f"✓ CAN bus closed")
    
    def _receive_loop(self):
        """Background thread to continuously receive CAN messages"""
        while self.running:
            try:
                msg = self.bus.recv(timeout=0.01)
                if msg and msg.arbitration_id in self.motors:
                    self._process_feedback(msg)
            except can.CanError as e:
                print(f"[CAN RX Error] {e}")
            except Exception as e:
                print(f"[RX Thread Error] {e}")
    
    def _process_feedback(self, msg):
        """
        Parse CAN feedback message and update motor state
        
        Frame format (8 bytes):
        [0]      Status byte
        [1-2]    Position (uint16, little-endian)
        [3-4]    Velocity (uint16, little-endian)
        [5-6]    Torque (uint16, little-endian)
        [7]      Temperature (uint8)
        """
        motor = self.motors.get(msg.arbitration_id)
        if not motor or len(msg.data) < 8:
            return
        
        data = msg.data
        
        # Parse feedback using same format as USB version
        motor.status_words = data[0]
        motor.pos = uint_to_float(
            int.from_bytes(data[1:3], 'little'), 
            motor.P_MIN, motor.P_MAX, 16
        )
        motor.vel = uint_to_float(
            int.from_bytes(data[3:5], 'little'), 
            motor.V_MIN, motor.V_MAX, 16
        )
        motor.torque = uint_to_float(
            int.from_bytes(data[5:7], 'little'), 
            motor.T_MIN, motor.T_MAX, 16
        )
        motor.temperature = data[7]
    
    def motor_mode(self, motor):
        """
        Enable motor mode (allows motor to move)
        Command: 0xFC
        """
        data = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC]
        msg = can.Message(
            arbitration_id=motor.id, 
            data=data, 
            is_extended_id=False
        )
        try:
            self.bus.send(msg)
        except can.CanError as e:
            print(f"[TX Error] motor_mode to ID {motor.id}: {e}")
    
    def reset_mode(self, motor):
        """
        Reset motor to idle mode
        Command: 0xFD
        """
        data = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD]
        msg = can.Message(
            arbitration_id=motor.id, 
            data=data, 
            is_extended_id=False
        )
        try:
            self.bus.send(msg)
        except can.CanError as e:
            print(f"[TX Error] reset_mode to ID {motor.id}: {e}")
    
    def zero_position(self, motor):
        """
        Set current position as zero
        Command: 0xFE
        """
        data = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE]
        msg = can.Message(
            arbitration_id=motor.id, 
            data=data, 
            is_extended_id=False
        )
        try:
            self.bus.send(msg)
        except can.CanError as e:
            print(f"[TX Error] zero_position to ID {motor.id}: {e}")
    
    def PTM_control(self, motor, pos, vel, kp, kd, torque):
        """
        Send PTM (Position-Torque Mix) control command
        
        Args:
            motor: Motor object
            pos: Target position (rad)
            vel: Target velocity (rad/s)
            kp: Position gain
            kd: Damping gain
            torque: Feedforward torque (Nm)
        
        Frame format (8 bytes):
        [0-1]    Position (uint16, little-endian)
        [2-3]    Velocity (uint16, little-endian)
        [4-5]    Kp (uint16, little-endian)
        [6-7]    Kd (uint16, little-endian)
        
        Note: Torque is sent separately in some implementations
        """
        # Clamp values to limits
        pos = max(motor.P_MIN, min(motor.P_MAX, pos))
        vel = max(motor.V_MIN, min(motor.V_MAX, vel))
        kp = max(motor.KP_MIN, min(motor.KP_MAX, kp))
        kd = max(motor.KD_MIN, min(motor.KD_MAX, kd))
        torque = max(motor.T_MIN, min(motor.T_MAX, torque))
        
        # Convert to uint16
        pos_uint = float_to_uint(pos, motor.P_MIN, motor.P_MAX, 16)
        vel_uint = float_to_uint(vel, motor.V_MIN, motor.V_MAX, 16)
        kp_uint = float_to_uint(kp, motor.KP_MIN, motor.KP_MAX, 16)
        kd_uint = float_to_uint(kd, motor.KD_MIN, motor.KD_MAX, 16)
        
        # Pack into 8 bytes (little-endian)
        data = [
            pos_uint & 0xFF, (pos_uint >> 8) & 0xFF,
            vel_uint & 0xFF, (vel_uint >> 8) & 0xFF,
            kp_uint & 0xFF, (kp_uint >> 8) & 0xFF,
            kd_uint & 0xFF, (kd_uint >> 8) & 0xFF
        ]
        
        msg = can.Message(
            arbitration_id=motor.id, 
            data=data, 
            is_extended_id=False
        )
        
        try:
            self.bus.send(msg)
        except can.CanError as e:
            print(f"[TX Error] PTM_control to ID {motor.id}: {e}")
    
    def poll(self):
        """
        Compatibility method for old polling-based code
        Does nothing - receiver thread handles this automatically
        """
        pass


# Example usage
if __name__ == "__main__":
    import time
    
    print("="*70)
    print("Native CAN Motor Controller Test")
    print("="*70)
    
    # Create controller
    controller = CANMotorController(channel='can0', bitrate=500000)
    
    # Create motor
    motor = Motor("test_motor", motor_id=3, type_name="REVO")
    controller.add_motor(motor)
    
    # Start receiver
    controller.start()
    
    try:
        # Enable motor
        print("\n1. Enabling motor mode...")
        controller.motor_mode(motor)
        time.sleep(0.5)
        
        # Send PTM command
        print("2. Sending PTM control (pos=0.5 rad, kp=50, kd=1.0)...")
        for i in range(10):
            controller.PTM_control(motor, pos=0.5, vel=0.0, kp=50, kd=1.0, torque=0.0)
            time.sleep(0.1)
            
            # Print feedback
            print(f"  t={i*0.1:.1f}s | Pos: {motor.pos:6.3f} rad | "
                  f"Vel: {motor.vel:6.3f} rad/s | Torque: {motor.torque:6.3f} Nm | "
                  f"Temp: {motor.temperature:3d}°C")
        
        # Reset motor
        print("\n3. Resetting motor...")
        controller.reset_mode(motor)
        time.sleep(0.2)
        
        print("\n✓ Test complete!")
        
    except KeyboardInterrupt:
        print("\n\n✗ Test interrupted by user")
    finally:
        controller.stop()
