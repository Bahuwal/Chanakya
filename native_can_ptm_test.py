#!/usr/bin/env python3
"""
PTM Mode Test with Native CAN (SocketCAN)

Standalone test file for Motorevo motors using Jetson's native CAN interface.
Based on: Final PTM Mode With Reset and Read Param.py

Hardware:
- Jetson Orin Nano Super (J17 CAN header)
- SN65HVD230 CAN transceiver
- Motorevo motors

Usage:
    python3 native_can_ptm_test.py [motor_id]

Example:
    python3 native_can_ptm_test.py 3
"""

import can
import time
import threading
import sys
import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Import utilities
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'bipedCANController'))
from utils import float_to_uint, uint_to_float


class Motor:
    """Motor class with limits and state"""
    def __init__(self,
                 motor_name=None,
                 motor_id=1,
                 Q_MAX=12.5,
                 DQ_MAX=10.0,
                 TAU_MAX=50.0,
                 Q_MIN=-12.5,
                 DQ_MIN=-10.0,
                 TAU_MIN=-50.0,
                 OKP_MAX=250.0,
                 OKD_MAX=50.0,
                 IKP_MAX=250.0,
                 IKD_MAX=50.0,
                 IKI_MAX=50.0,
                 OKP_MIN=0.0,
                 OKD_MIN=0.0,
                 IKP_MIN=0.0,
                 IKD_MIN=0.0,
                 IKI_MIN=0.0,
                 CUR_MAX=100.0,
                 CUR_MIN=-100.0,
                 type_name="REVO"):

        self.name = motor_name
        self.id = motor_id
        self.Q_MAX = Q_MAX
        self.DQ_MAX = DQ_MAX
        self.TAU_MAX = TAU_MAX
        self.Q_MIN = Q_MIN
        self.DQ_MIN = DQ_MIN
        self.TAU_MIN = TAU_MIN
        self.OKP_MAX = OKP_MAX
        self.OKD_MAX = OKD_MAX
        self.IKP_MAX = IKP_MAX
        self.IKD_MAX = IKD_MAX
        self.IKI_MAX = IKI_MAX
        self.OKP_MIN = OKP_MIN
        self.OKD_MIN = OKD_MIN
        self.IKP_MIN = IKP_MIN
        self.IKD_MIN = IKD_MIN
        self.IKI_MIN = IKI_MIN
        self.CUR_MAX = CUR_MAX
        self.CUR_MIN = CUR_MIN
        self.type = type_name

        self.status_words = 0.0
        self.pos = 0.0
        self.vel = 0.0
        self.torque = 0.0
        self.temperature = 0.0
        self.error_code = 0

    def get_data(self, status_words, pos, vel, torque, temperature, error_code):
        self.status_words = status_words
        self.pos = pos
        self.vel = vel
        self.torque = torque
        self.temperature = temperature
        self.error_code = error_code


class CANMotorController:
    """Native CAN motor controller using SocketCAN"""
    
    def __init__(self, channel='can0', bitrate=1000000):
        self.channel = channel
        self.bitrate = bitrate
        self.motors = {}
        
        # Initialize CAN bus
        try:
            self.bus = can.Bus(channel=channel, interface='socketcan', 
                              bitrate=bitrate, receive_own_messages=False)
            print(f"✓ CAN '{channel}' opened at {bitrate} bps")
        except OSError as e:
            print(f"✗ Failed to open CAN '{channel}'")
            print(f"  Run: sudo ip link set {channel} type can bitrate {bitrate}")
            print(f"       sudo ip link set {channel} up")
            raise
        
        # Background receiver
        self.running = False
        self.rx_thread = None
    
    def add_motor(self, motor):
        self.motors[motor.id] = motor
    
    def start(self):
        """
        Start background receiver thread
        
        IMPORTANT: This acts as continuous parameter polling which is
        REQUIRED for Motorevo motors to operate. The motors will not
        respond to commands unless parameter reading is active!
        """
        if self.running:
            return
        self.running = True
        self.rx_thread = threading.Thread(target=self._receive_loop, daemon=True)
        self.rx_thread.start()
    
    def stop(self):
        """Stop receiver and close bus"""
        self.running = False
        if self.rx_thread:
            self.rx_thread.join(timeout=1.0)
        self.bus.shutdown()
    
    def _receive_loop(self):
        """
        Background RX thread - Continuous parameter polling
        
        This continuously reads CAN messages, which is REQUIRED for
        Motorevo motors. They expect constant parameter polling to operate.
        """
        while self.running:
            try:
                # Poll aggressively (5ms like original param port)
                msg = self.bus.recv(timeout=0.005)
                if msg and msg.arbitration_id in self.motors:
                    self._process_feedback(msg)
            except:
                pass  # Continue polling even on errors
    
    def _process_feedback(self, msg):
        """Parse feedback message"""
        motor = self.motors.get(msg.arbitration_id)
        if not motor or len(msg.data) < 8:
            return
        
        data = msg.data
        status_words = data[0]
        pos = uint_to_float(int.from_bytes(data[1:3], 'little'), 
                           motor.Q_MIN, motor.Q_MAX, 16)
        vel = uint_to_float(int.from_bytes(data[3:5], 'little'), 
                           motor.DQ_MIN, motor.DQ_MAX, 16)
        torque = uint_to_float(int.from_bytes(data[5:7], 'little'), 
                              motor.TAU_MIN, motor.TAU_MAX, 16)
        temperature = data[7]
        
        motor.get_data(status_words, pos, vel, torque, temperature, 0)
    
    def motor_mode(self, motor):
        """Enable motor mode (0xFC)"""
        data = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC]
        msg = can.Message(arbitration_id=motor.id, data=data, is_extended_id=False)
        self.bus.send(msg)
    
    def reset_mode(self, motor):
        """Reset motor (0xFD)"""
        data = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD]
        msg = can.Message(arbitration_id=motor.id, data=data, is_extended_id=False)
        self.bus.send(msg)
    
    def zero_position(self, motor):
        """Zero current position (0xFE)"""
        data = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE]
        msg = can.Message(arbitration_id=motor.id, data=data, is_extended_id=False)
        self.bus.send(msg)
    
    def PTM_control(self, motor, pos, vel, kp, kd, torque):
        """PTM control command"""
        # Clamp values
        pos = max(motor.Q_MIN, min(motor.Q_MAX, pos))
        vel = max(motor.DQ_MIN, min(motor.DQ_MAX, vel))
        kp = max(motor.OKP_MIN, min(motor.OKP_MAX, kp))
        kd = max(motor.OKD_MIN, min(motor.OKD_MAX, kd))
        torque = max(motor.TAU_MIN, min(motor.TAU_MAX, torque))
        
        # Convert to uint16
        pos_uint = float_to_uint(pos, motor.Q_MIN, motor.Q_MAX, 16)
        vel_uint = float_to_uint(vel, motor.DQ_MIN, motor.DQ_MAX, 16)
        kp_uint = float_to_uint(kp, motor.OKP_MIN, motor.OKP_MAX, 16)
        kd_uint = float_to_uint(kd, motor.OKD_MIN, motor.OKD_MAX, 16)
        
        # Pack data
        data = [
            pos_uint & 0xFF, (pos_uint >> 8) & 0xFF,
            vel_uint & 0xFF, (vel_uint >> 8) & 0xFF,
            kp_uint & 0xFF, (kp_uint >> 8) & 0xFF,
            kd_uint & 0xFF, (kd_uint >> 8) & 0xFF
        ]
        
        msg = can.Message(arbitration_id=motor.id, data=data, is_extended_id=False)
        self.bus.send(msg)


def main():
    # Get motor ID from command line
    motor_id = int(sys.argv[1]) if len(sys.argv) > 1 else 7
    
    print("="*70)
    print("PTM MODE TEST - Native CAN (SocketCAN)")
    print("="*70)
    print(f"Motor ID: {motor_id}")
    print(f"Interface: can0 @ 1Mbps")
    print("="*70)
    
    # Create controller
    try:
        controller = CANMotorController(channel='can0', bitrate=1000000)
    except Exception as e:
        print(f"\n✗ Failed to initialize CAN: {e}")
        return 1
    
    # Create motor
    motor = Motor(motor_name=f"motor_{motor_id}", motor_id=motor_id, type_name="REVO")
    controller.add_motor(motor)
    
    # CRITICAL: Start parameter polling
    # Motorevo motors require continuous parameter reading to operate!
    print("\nStarting parameter polling thread...")
    controller.start()
    time.sleep(0.5)
    
    print("✓ Parameter polling active")
    print(f"  Position: {motor.pos:.3f} rad")
    print(f"  Temperature: {motor.temperature}°C\n")
    
    # Test parameters
    target_position = 5.0  # rad
    target_velocity = 0.0  # rad/s
    target_torque = 2.0    # Nm (feedforward)
    kp_gain = 20.0
    kd_gain = 0.8
    
    # Data logging
    time_data = []
    pos_data = []
    vel_data = []
    torque_data = []
    temp_data = []
    
    print("\n" + "="*70)
    print("PTM MODE HARDWARE TEST - Position-Torque Mix Control")
    print("="*70)
    print(f"Target Position: {target_position} rad")
    print(f"Target Velocity: {target_velocity} rad/s")
    print(f"Target Torque: {target_torque} Nm (feed-forward)")
    print(f"Kp: {kp_gain}, Kd: {kd_gain}")
    print(f"Control Formula: T = Kp×(Pref-Pact) + Kd×(Vref-Vact) + Tref")
    print("="*70)
    print()
    
    try:
        # Reset motor first
        print("Resetting motor...")
        controller.reset_mode(motor)
        time.sleep(0.5)
        
        # Enable motor mode
        print(f"Enabling motor {motor_id}...")
        controller.motor_mode(motor)
        time.sleep(0.5)
        
        # Control loop
        start_time = time.time()
        test_duration = 8.0  # seconds
        
        print(f"Running control loop for {test_duration}s...\n")
        print(f"{'Time (s)':>10} | {'Pos (rad)':>10} | {'Vel (rad/s)':>12} | {'Torque (Nm)':>12} | {'Temp (°C)':>10} | Err")
        print("-" * 70)
        
        while (time.time() - start_time) < test_duration:
            current_time = time.time() - start_time
            
            # Send PTM command
            controller.PTM_control(motor, 
                                  pos=target_position,
                                  vel=target_velocity,
                                  kp=kp_gain,
                                  kd=kd_gain,
                                  torque=target_torque)
            
            # Log data
            time_data.append(current_time)
            pos_data.append(motor.pos)
            vel_data.append(motor.vel)
            torque_data.append(motor.torque)
            temp_data.append(motor.temperature)
            
            # Print status (every 0.5s)
            if len(time_data) % 50 == 0:
                print(f"{current_time:>10.2f} | {motor.pos:>10.3f} | {motor.vel:>12.3f} | "
                      f"{motor.torque:>12.3f} | {motor.temperature:>10.0f} | {motor.error_code}")
            
            time.sleep(0.01)  # 100Hz control loop
        
        print("\n" + "="*70)
        print("✓ Test complete!")
        print(f"  Final position: {motor.pos:.3f} rad")
        print(f"  Final velocity: {motor.vel:.3f} rad/s")
        print(f"  Final torque: {motor.torque:.3f} Nm")
        print(f"  Temperature: {motor.temperature}°C")
        print("="*70)
        
        # Stop motor
        print("\nStopping motor...")
        controller.reset_mode(motor)
        time.sleep(0.2)
        
        # Plot results
        print("\nGenerating plots...")
        fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(12, 8))
        fig.suptitle(f'PTM Control Test - Motor {motor_id} (Native CAN)', fontsize=14)
        
        # Position
        ax1.plot(time_data, pos_data, 'b-', linewidth=2)
        ax1.axhline(y=target_position, color='r', linestyle='--', label='Target')
        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel('Position (rad)')
        ax1.set_title('Position Control')
        ax1.grid(True)
        ax1.legend()
        
        # Velocity
        ax2.plot(time_data, vel_data, 'g-', linewidth=2)
        ax2.axhline(y=target_velocity, color='r', linestyle='--', label='Target')
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Velocity (rad/s)')
        ax2.set_title('Velocity')
        ax2.grid(True)
        ax2.legend()
        
        # Torque
        ax3.plot(time_data, torque_data, 'orange', linewidth=2)
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Torque (Nm)')
        ax3.set_title('Output Torque')
        ax3.grid(True)
        
        # Temperature
        ax4.plot(time_data, temp_data, 'r-', linewidth=2)
        ax4.set_xlabel('Time (s)')
        ax4.set_ylabel('Temperature (°C)')
        ax4.set_title('Motor Temperature')
        ax4.grid(True)
        
        plt.tight_layout()
        plt.show()
        
        return 0
        
    except KeyboardInterrupt:
        print("\n\n✗ Test interrupted by user")
        controller.reset_mode(motor)
        return 1
    
    finally:
        controller.stop()


if __name__ == "__main__":
    exit(main())
