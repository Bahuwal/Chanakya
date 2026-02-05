from time import sleep
import time
import json
import threading
import os
import numpy as np
import can
from utils import float_to_uint, uint_to_float

class Motor:
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

    def getPosition(self):
        return self.pos

    def getVelocity(self):
        return self.vel

    def getTorque(self):
        return self.torque

    def getTemperature(self):
        return self.temperature

    def getErrorCode(self):
        return self.error_code


class MotorController:
    def __init__(self, serial_device, can_bus=None):
        """Initialize with serial (for CAN wrapper) and optional CAN bus for feedback"""
        self.serial_device = serial_device
        self.can_bus = can_bus  # For reading CAN feedback
        self.motors = dict()
        self.rx_buffer = bytes()
        self.running = False
        self.rx_thread = None

        self.tx_buffer = bytearray([
            0xAA, 0x01,
            0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC,
            0x00, 0x00, 0x00, 0x00,
            0xF4
        ])

        # Close port if already open
        try:
            if hasattr(self.serial_device, "is_open") and self.serial_device.is_open:
                self.serial_device.close()
        except Exception:
            pass

        try:
            self.serial_device.open()
            print(f"Serial port {getattr(self.serial_device, 'port', '<unknown>')} opened successfully.")
        except Exception as e:
            print(f"Error opening serial port: {e}")

    def add_motor(self, motor: Motor):
        self.motors[motor.id] = motor
    
    def start_can_feedback(self):
        """Start CAN feedback reading thread"""
        if not self.can_bus or self.running:
            return
        self.running = True
        self.rx_thread = threading.Thread(target=self._can_receive_loop, daemon=True)
        self.rx_thread.start()
        print("✓ CAN feedback receiver started")
    
    def stop_can_feedback(self):
        """Stop CAN feedback reading"""
        self.running = False
        if self.rx_thread:
            self.rx_thread.join(timeout=1.0)
    
    def _can_receive_loop(self):
        """Background thread to read CAN feedback from motor"""
        print("[CAN RX] Listening for motor feedback...")
        
        while self.running:
            try:
                msg = self.can_bus.recv(timeout=0.01)
                if msg and msg.arbitration_id in self.motors:
                    self._process_can_feedback(msg)
            except Exception as e:
                if "timeout" not in str(e).lower():
                    print(f"[CAN RX ERROR] {e}")
    
    def _process_can_feedback(self, msg):
        """Process CAN feedback message from motor"""
        motor = self.motors.get(msg.arbitration_id)
        if not motor or len(msg.data) < 8:
            return
        
        data = msg.data
        
        # FILTER OUT COMMAND ECHOES!
        # Real motor feedback starts with 0x02 (motor ID byte)
        # Command echoes start with 0xE6, 0xFF, 0x7F, etc.
        if data[0] != 0x02:
            # This is a command echo, ignore it!
            return
        
        # Parse feedback according to OFFICIAL Motorevo code (Revo_CAN.py lines 206-222)
        # This is the CORRECT format verified from working code:
        status_words = data[0]
        q_uint = (data[1] << 8) | data[2]
        dq_uint = (data[3] << 4) | (data[4] >> 4)
        tau_uint = ((data[4] & 0x0F) << 8) | data[5]
        error_code = data[6]      # ← WAS SWAPPED! Error is byte 6
        temperature = data[7]     # ← WAS SWAPPED! Temp is byte 7
        
        # Convert to floats
        recv_q = uint_to_float(q_uint, motor.Q_MIN, motor.Q_MAX, 16)
        recv_dq = uint_to_float(dq_uint, motor.DQ_MIN, motor.DQ_MAX, 12)
        recv_tau = uint_to_float(tau_uint, motor.TAU_MIN, motor.TAU_MAX, 12)
        
        # Update motor state
        motor.get_data(status_words, recv_q, recv_dq, recv_tau, temperature, error_code)

    # Zero Position Command
    def set_zero_position(self, motor: Motor):
        data_buff = bytes([
            0xFF, 0xFF, 0xFF, 0xFF,
            0xFF, 0xFF, 0xFF, 0xFE
        ])
        self.__send_data(motor.id, data_buff)
        sleep(0.005)

    # Mode Commands
    def reset_mode(self, motor=None):
        """Reset motor(s). If motor provided, reset that motor; else reset all or broadcast."""
        if motor is not None:
            if isinstance(motor, Motor):
                mid = int(motor.id)
            else:
                try:
                    mid = int(motor)
                except Exception:
                    mid = 0xFFFF
            self.__control_cmd(mid, np.uint8(0xFD))
            sleep(0.01)
            return

        # No specific motor -> reset all registered motors (or broadcast)
        if not self.motors:
            self.__control_cmd(0xFFFF, np.uint8(0xFD))
        else:
            for mid in list(self.motors.keys()):
                self.__control_cmd(int(mid), np.uint8(0xFD))
        sleep(0.01)

    def motor_mode(self, motor: Motor):
        self.__control_cmd(motor, np.uint8(0xFC))
        sleep(0.01)

    # SERVO MODE (8 bytes)
    def Servo_control(self, motor: Motor, pos, vel, kp, kd, ikp, ikd, iki, type_name="REVO"):
        """
        Servo mode control:
        
        Args:
            motor: Motor object
            pos: Target position (radians)
            vel: Target velocity (rad/s)
            kp: Outer position proportional gain
            kd: Outer position derivative gain
            ikp: Inner loop proportional gain
            ikd: Inner loop derivative gain
            iki: Inner loop integral gain
        """
        pos_uint = float_to_uint(pos, motor.Q_MIN, motor.Q_MAX, 16)
        vel_uint = float_to_uint(vel, motor.DQ_MIN, motor.DQ_MAX, 8)
        kp_uint = float_to_uint(kp, motor.OKP_MIN, motor.OKP_MAX, 8)
        kd_uint = float_to_uint(kd, motor.OKD_MIN, motor.OKD_MAX, 8)
        ikp_uint = float_to_uint(ikp, motor.IKP_MIN, motor.IKP_MAX, 8)
        ikd_uint = float_to_uint(ikd, motor.IKD_MIN, motor.IKD_MAX, 8)
        iki_uint = float_to_uint(iki, motor.IKI_MIN, motor.IKI_MAX, 8)

        data_buff = bytes([
            (pos_uint >> 8) & 0xFF,
            pos_uint & 0xFF,
            vel_uint & 0xFF,
            kp_uint & 0xFF,
            kd_uint & 0xFF,
            ikp_uint & 0xFF,
            ikd_uint & 0xFF,
            iki_uint & 0xFF
        ])

        self.__send_data(motor.id, data_buff)
        sleep(0.001)

    # Low-level TX/RX
    def __control_cmd(self, motor, cmd):
        if isinstance(motor, Motor):
            motor_id = int(motor.id)
        elif isinstance(motor, int):
            motor_id = int(motor)
        else:
            motor_id = 0xFFFF
        data_buff = bytes([0xFF] * 7 + [cmd])
        self.__send_data(motor_id, data_buff)

    def __send_data(self, motor_id, data):
        self.tx_buffer[10] = motor_id & 0xFF
        self.tx_buffer[11] = (motor_id >> 8) & 0xFF
        self.tx_buffer[12] = (motor_id >> 16) & 0xFF
        self.tx_buffer[13] = (motor_id >> 24) & 0xFF

        self.tx_buffer[2:10] = data[:8] if len(data) >= 8 else (data + bytes(8 - len(data)))
        
        try:
            self.serial_device.write(self.tx_buffer)
        except Exception as e:
            print(f"[__send_data] write error: {e}")


# CAN wrapper - acts like serial.Serial for MotorController
class CANMotorSender:
    """Wrapper that makes CAN bus look like serial port for MotorController"""
    def __init__(self, can_bus):
        self.can_bus = can_bus
        self.is_open = True
        self.debug = True  # Enable debug output
    
    def open(self):
        """Dummy method to match serial interface"""
        self.is_open = True
    
    def write(self, data):
        """Extract CAN data from serial protocol wrapper and send via CAN"""
        if len(data) >= 14:
            can_data = data[2:10]  # Extract 8 CAN data bytes
            motor_id = data[10] | (data[11] << 8) | (data[12] << 16) | (data[13] << 24)
            
            # Create standard 11-bit CAN frame
            msg = can.Message(
                arbitration_id=motor_id,  # Frame ID = Motor ID
                data=can_data,             # 8 bytes of data
                is_extended_id=False       # Standard 11-bit ID
            )
            
            # Debug output
            if self.debug:
                print(f"[CAN TX] ID: 0x{motor_id:08X} ({motor_id}) | Data: {' '.join([f'{b:02X}' for b in can_data])}")
            
            try:
                self.can_bus.send(msg)
            except Exception as e:
                print(f"[CAN TX ERROR] {e}")
    
    def close(self):
        self.is_open = False


# MAIN TEST LOOP
if __name__ == "__main__":
    import matplotlib.pyplot as plt
    from collections import deque
    import time

    # Plotting Setup
    plt.ion()
    fig, ax = plt.subplots(figsize=(10, 6))

    time_data = deque(maxlen=200)
    pos_data = deque(maxlen=200)
    target_data = deque(maxlen=200)

    line_pos, = ax.plot([], [], 'r-', label='Actual Pos', linewidth=2)
    line_target, = ax.plot([], [], 'g--', label='Target Pos', linewidth=2)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Position (rad)')
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)
    ax.set_title('Servo Mode: Position Control via CAN')

    print(f"Using CAN: can0 @ 1Mbps")

    # Create CAN bus
    try:
        motor_can_bus = can.Bus(channel='can0', interface='socketcan', bitrate=1000000)
        motor_serial = CANMotorSender(motor_can_bus)
        print(f"✓ CAN opened for motor commands")
    except Exception as e:
        print(f"[main] Failed to open CAN: {e}")
        raise SystemExit("CAN not available.")

    # Initialize motor
    revo = Motor("revo_motor", motor_id=3, type_name="REVO")

    # Command controller (sends via CAN, reads feedback from CAN)
    ctrl = MotorController(motor_serial, can_bus=motor_can_bus)
    ctrl.add_motor(revo)
    ctrl.start_can_feedback()  # Start reading CAN feedback
    ctrl.motor_mode(revo)

    # Servo Control parameters
    target_pos = 10.0      # Target position in radians
    sent_vel = 2.0         # Target velocity (rad/s)
    kp_gain = 50.0         # Outer loop position stiffness
    kd_gain = 5.0          # Outer loop damping
    ikp_gain = 30.0        # Inner loop proportional
    ikd_gain = 0.01        # Inner loop derivative
    iki_gain = 0.0         # Inner loop integral
    
    CONTROL_DURATION = 8
    start = time.time()

    print("\n" + "="*70)
    print("SERVO MODE HARDWARE TEST - Position Control")
    print("="*70)
    print(f"Target Position: {target_pos} rad")
    print(f"Target Velocity: {sent_vel} rad/s")
    print(f"Outer Loop: Kp={kp_gain}, Kd={kd_gain}")
    print(f"Inner Loop: iKp={ikp_gain}, iKd={ikd_gain}, iKi={iki_gain}")
    print("="*70 + "\n")

    try:
        while time.time() - start < CONTROL_DURATION:
            now = time.time() - start

            # Send Servo command
            ctrl.Servo_control(
                revo, 
                pos=target_pos,
                vel=sent_vel,
                kp=kp_gain, 
                kd=kd_gain,
                ikp=ikp_gain,
                ikd=ikd_gain,
                iki=iki_gain
            )

            # AUTO ZERO POSITION FEATURE
            if abs(revo.pos - target_pos) < 0.05:
                print("Reached target → Zeroing position...")
                ctrl.set_zero_position(revo)
                target_pos = 0  # next command is relative

            # Print feedback
            print(f"Time: {now:.2f}s | "
                  f"Pos: {revo.pos:6.2f} rad | "
                  f"Vel: {revo.vel:6.2f} rad/s | "
                  f"Torque: {revo.torque:6.2f} Nm | "
                  f"Temp: {revo.temperature:4.1f}°C | "
                  f"Err: {revo.error_code}")

            # Plotting
            time_data.append(now)
            pos_data.append(revo.pos)
            target_data.append(target_pos)

            line_pos.set_data(time_data, pos_data)
            line_target.set_data(time_data, target_data)
            ax.relim()
            ax.autoscale_view()
            fig.canvas.draw()
            fig.canvas.flush_events()

            sleep(0.01)

    except KeyboardInterrupt:
        print("\n[INTERRUPTED] User stopped execution.")

    print("\nStopping motor...")
    ctrl.Servo_control(revo, pos=0, vel=0, kp=0, kd=0, ikp=0, ikd=0, iki=0)
    sleep(0.1)

    print("Resetting motor...")
    ctrl.reset_mode(revo)
    sleep(0.1)

    # Stop CAN feedback
    ctrl.stop_can_feedback()

    # Keep plot visible
    plt.ioff()
    plt.show()

    # Close CAN
    try:
        if motor_serial and motor_serial.is_open:
            motor_serial.close()
            print("CAN bus closed.")
    except Exception:
        pass

    print("Exiting.")
