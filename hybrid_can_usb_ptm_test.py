#!/usr/bin/env python3
"""
Hybrid CAN + USB PTM Test

Uses:
- USB param port (continuous polling - REQUIRED for motors to work)
- Native CAN (for sending commands and reading feedback)

This combines the best of both worlds:
- USB param polling ensures motors respond
- CAN provides direct, low-latency control

Hardware:
- Jason USB module (param port only)
- Jetson CAN (J17 + SN65HVD230 transceiver)
"""

import can
import serial
import time
import threading
import sys
import os
import json
import numpy as np
import matplotlib.pyplot as plt
from collections import deque

# Import utilities
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'bipedCANController'))
from utils import float_to_uint, uint_to_float


class Motor:
    """Motor class"""
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
        self.OKP_MIN = 0.0
        self.OKD_MIN = 0.0
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


# USB PARAM PORT CONTROLLER (from original file)
class ParamController:
    """USB-based parameter reader (original Jason module behavior)"""
    
    def __init__(self, serial_device):
        self.serial_device = serial_device
        self.motors = {}
        self.rx_buffer = bytes()
        
        # Open serial port
        try:
            if hasattr(self.serial_device, "is_open") and self.serial_device.is_open:
                self.serial_device.close()
        except:
            pass
        
        try:
            self.serial_device.open()
            print(f"✓ Param port opened: {self.serial_device.port}")
        except Exception as e:
            print(f"✗ Failed to open param port: {e}")
    
    def add_motor(self, motor):
        self.motors[motor.id] = motor
    
    def poll(self):
        """Poll serial device for parameter feedback"""
        try:
            data_recv = b''.join([self.rx_buffer, self.serial_device.read_all()])
        except Exception:
            return
        
        packets = self._extract_packets(data_recv)
        
        # Handle remainder bytes
        frame_length = 12
        if len(data_recv) >= frame_length:
            remainder_pos = len(data_recv) % frame_length
            self.rx_buffer = data_recv[-remainder_pos:] if remainder_pos else b''
        else:
            self.rx_buffer = data_recv
        
        for packet in packets:
            data = packet[0:8]
            CANID = (packet[11] << 24) | (packet[10] << 16) | (packet[9] << 8) | packet[8]
            self._process_packet(data, CANID)
    
    def _extract_packets(self, data):
        """Extract 12-byte packets from serial data"""
        packets = []
        i = 0
        while i <= len(data) - 12:
            packet = data[i:i+12]
            packets.append(packet)
            i += 12
        return packets
    
    def _process_packet(self, data, motor_id):
        """Process feedback packet"""
        motor = self.motors.get(motor_id)
        if not motor or len(data) < 8:
            return
        
        status_words = data[0]
        pos = uint_to_float(int.from_bytes(data[1:3], 'little'), 
                           motor.Q_MIN, motor.Q_MAX, 16)
        vel = uint_to_float(int.from_bytes(data[3:5], 'little'), 
                           motor.DQ_MIN, motor.DQ_MAX, 16)
        torque = uint_to_float(int.from_bytes(data[5:7], 'little'), 
                              motor.TAU_MIN, motor.TAU_MAX, 16)
        temperature = data[7]
        
        motor.get_data(status_words, pos, vel, torque, temperature, 0)


# CAN COMMAND CONTROLLER
class CANCommandController:
    """Native CAN controller for sending commands"""
    
    def __init__(self, channel='can0', bitrate=1000000):
        self.channel = channel
        
        try:
            self.bus = can.Bus(channel=channel, interface='socketcan', 
                              bitrate=bitrate, receive_own_messages=False)
            print(f"✓ CAN '{channel}' opened at {bitrate} bps")
        except OSError as e:
            print(f"✗ Failed to open CAN '{channel}'")
            print(f"  Run: sudo ip link set {channel} type can bitrate {bitrate}")
            print(f"       sudo ip link set {channel} up")
            raise
    
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
    
    def close(self):
        self.bus.shutdown()


# PARAM POLLING THREAD (from original file)
def start_param_polling(param_controller, poll_interval=0.005, stop_event=None):
    """Start background thread to continuously poll parameter feedback"""
    def run():
        print("  [Param Thread] Started polling...")
        while not (stop_event and stop_event.is_set()):
            try:
                param_controller.poll()
            except Exception:
                pass
            time.sleep(poll_interval)
        print("  [Param Thread] Stopped")
    
    t = threading.Thread(target=run, daemon=True)
    t.start()
    return t


def load_usb_config(path="usb.json"):
    """Load USB config for param port"""
    default = {
        "param_port": "/dev/ttyUSB1",
        "baudrate": 921600
    }
    if os.path.exists(path):
        try:
            with open(path, "r") as f:
                cfg = json.load(f)
            param_port = cfg.get("param_port", default["param_port"])
            baudrate = int(cfg.get("baudrate", default["baudrate"]))
            return param_port, baudrate
        except Exception as e:
            print(f"[Config] Error: {e}. Using defaults.")
            return default["param_port"], default["baudrate"]
    else:
        print(f"[Config] {path} not found. Using defaults.")
        return default["param_port"], default["baudrate"]


def main():
    motor_id = int(sys.argv[1]) if len(sys.argv) > 1 else 7
    
    print("="*70)
    print("HYBRID CAN + USB PTM TEST")
    print("="*70)
    print(f"Motor ID: {motor_id}")
    print("Command: Native CAN (can0)")
    print("Param polling: USB (Jason module)")
    print("="*70)
    
    # Load USB config
    param_port, BAUDRATE = load_usb_config("usb.json")
    print(f"\nUSB param_port: {param_port} @ {BAUDRATE} bps")
    
    # Create param serial
    try:
        param_serial = serial.Serial(port=param_port, baudrate=BAUDRATE, timeout=0.05)
    except Exception as e:
        print(f"✗ Failed to open param port {param_port}: {e}")
  print(f"  Update usb.json or specify correct port")
        return 1
    
    # Create CAN controller
    try:
        can_ctrl = CANCommandController(channel='can0', bitrate=1000000)
    except Exception as e:
        print(f"✗ Failed to open CAN: {e}")
        return 1
    
    # Create motor
    motor = Motor(motor_name=f"motor_{motor_id}", motor_id=motor_id, type_name="REVO")
    
    # Create param controller
    param_ctrl = ParamController(param_serial)
    param_ctrl.add_motor(motor)
    
    # Start param polling thread
    stop_event = threading.Event()
    print("\n✓ Starting param polling thread...")
    start_param_polling(param_ctrl, poll_interval=0.005, stop_event=stop_event)
    time.sleep(0.5)
    
    print(f"✓ Param polling active")
    print(f"  Position: {motor.pos:.3f} rad")
    print(f"  Temperature: {motor.temperature}°C\n")
    
    # Test parameters
    target_pos = 5.0
    target_vel = 0.0
    target_torque = 2.0
    kp_gain = 20.0
    kd_gain = 0.8
    
    # Data logging
    time_data = deque(maxlen=1000)
    pos_data = deque(maxlen=1000)
    vel_data = deque(maxlen=1000)
    torque_data = deque(maxlen=1000)
    
    print("="*70)
    print("PTM MODE TEST")
    print("="*70)
    print(f"Target Position: {target_pos} rad")
    print(f"Kp: {kp_gain}, Kd: {kd_gain}")
    print("="*70)
    
    try:
        # Reset motor
        print("\nResetting motor...")
        can_ctrl.reset_mode(motor)
        time.sleep(0.5)
        
        # Enable motor mode
        print(f"Enabling motor {motor_id}...")
        can_ctrl.motor_mode(motor)
        time.sleep(0.5)
        
        # Control loop
        start_time = time.time()
        test_duration = 8.0
        
        print(f"\nRunning control loop for {test_duration}s...\n")
        print(f"{'Time':>8} | {'Pos':>8} | {'Vel':>8} | {'Torque':>8} | {'Temp':>6}")
        print("-" * 60)
        
        while (time.time() - start_time) < test_duration:
            current_time = time.time() - start_time
            
            # Send CAN command
            can_ctrl.PTM_control(motor, 
                                pos=target_pos,
                                vel=target_vel,
                                kp=kp_gain,
                                kd=kd_gain,
                                torque=target_torque)
            
            # Log data (feedback updated by param thread)
            time_data.append(current_time)
            pos_data.append(motor.pos)
            vel_data.append(motor.vel)
            torque_data.append(motor.torque)
            
            # Print status
            if len(time_data) % 50 == 0:
                print(f"{current_time:>8.2f} | {motor.pos:>8.3f} | {motor.vel:>8.3f} | "
                      f"{motor.torque:>8.3f} | {motor.temperature:>6.0f}")
            
            time.sleep(0.01)  # 100Hz
        
        print("\n" + "="*70)
        print("✓ Test complete!")
        print(f"  Final position: {motor.pos:.3f} rad")
        print(f"  Temperature: {motor.temperature}°C")
        print("="*70)
        
        # Stop motor
        can_ctrl.reset_mode(motor)
        
        # Plot
        fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(12, 8))
        fig.suptitle(f'Hybrid CAN+USB PTM Test - Motor {motor_id}', fontsize=14)
        
        ax1.plot(time_data, pos_data, 'b-', linewidth=2)
        ax1.axhline(y=target_pos, color='r', linestyle='--', label='Target')
        ax1.set_ylabel('Position (rad)')
        ax1.set_title('Position Control')
        ax1.grid(True)
        ax1.legend()
        
        ax2.plot(time_data, vel_data, 'g-', linewidth=2)
        ax2.set_ylabel('Velocity (rad/s)')
        ax2.set_title('Velocity')
        ax2.grid(True)
        
        ax3.plot(time_data, torque_data, 'orange', linewidth=2)
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Torque (Nm)')
        ax3.set_title('Output Torque')
        ax3.grid(True)
        
        plt.tight_layout()
        plt.show()
        
        return 0
        
    except KeyboardInterrupt:
        print("\n\n✗ Interrupted")
        can_ctrl.reset_mode(motor)
        return 1
    
    finally:
        stop_event.set()
        can_ctrl.close()
        param_serial.close()


if __name__ == "__main__":
    exit(main())
