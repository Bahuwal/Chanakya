from time import sleep
import time
import json
import threading
import os
import numpy as np
import serial
import matplotlib.pyplot as plt
import yaml
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


class WaveshareCANWrapper:
    """
    Wrapper to make Waveshare USB-CAN-A module compatible with MotorController interface.
    
    Waveshare Protocol:
    - TX: 0xAA, 0xC8 (standard frame), ID[0-1], Data[0-7], 0x55
    - RX: 0xAA, 0xCx (flags), ID bytes, Data bytes, 0x55
    """
    def __init__(self, port, baudrate=2000000):
        self.port = port
        self.baudrate = baudrate
        self.is_open = False
        self._serial = None
        self.open()

    def open(self):
        """Open serial port and configure Waveshare module"""
        try:
            self._serial = serial.Serial(self.port, self.baudrate, timeout=0.01)
            self.is_open = True
            
            # Configure Waveshare module: 1 Mbps, STANDARD frames, Variable protocol
            set_can_baudrate = [
                0xAA,     # Packet header
                0x55,     # Packet header
                0x12,     # Variable protocol
                0x01,     # CAN Baud Rate: 1 Mbps
                0x01,     # Frame Type: STANDARD Frame (NOT extended)
                0x00, 0x00, 0x00, 0x00,  # Filter ID
                0x00, 0x00, 0x00, 0x00,  # Mask ID
                0x00,     # CAN mode: normal
                0x00,     # automatic resend
                0x00, 0x00, 0x00, 0x00,  # Spare
            ]
            
            # Calculate checksum
            checksum = sum(set_can_baudrate[2:]) & 0xFF
            set_can_baudrate.append(checksum)
            
            self._serial.write(bytes(set_can_baudrate))
            sleep(0.1)  # Wait for configuration
            
            print(f"✓ Waveshare USB-CAN-A configured: {self.port} @ 1 Mbps, STANDARD frames")
            
        except Exception as e:
            print(f"Error opening serial port: {e}")
            self.is_open = False
            return
        
        print("✓ Serial port opened successfully.\n")

    def write(self, data):
        """
        Translate Jason's protocol to Waveshare format and send.
        
        Jason format: 0xAA 0x01 Data[8] ID[4]
        Waveshare TX:  0xAA 0xC8 ID[2] Data[8] 0x55 (for standard frames)
        """
        if not self.is_open or len(data) < 14:
            return
        
        # Extract from Jason's protocol
        can_data = data[2:10]  # 8 bytes of CAN data
        motor_id = data[10] | (data[11] << 8) | (data[12] << 16) | (data[13] << 24)
        
        # Build Waveshare packet for STANDARD frame with 8 data bytes
        # Byte 1: 0xC0 (base) | 0x00 (STANDARD frame) | 0x08 (8 data bytes) = 0xC8
        waveshare_packet = [
            0xAA,     # Header
            0xC8,     # Type: STANDARD frame (11-bit ID), 8 data bytes
            motor_id & 0xFF,              # ID byte 0 (LSB)
            (motor_id >> 8) & 0xFF,       # ID byte 1 (MSB)
        ]

        # Add 8 bytes of CAN data
        waveshare_packet.extend(can_data[:8])

        # Add end marker
        waveshare_packet.append(0x55)
        
        try:
            self._serial.write(bytes(waveshare_packet))
        except Exception as e:
            print(f"[Waveshare TX Error] {e}")

    def read_all(self):
        """
        Read and parse ALL available Waveshare CAN frames, convert to Jason's protocol format.
        Returns: bytes in Jason's protocol format (multiple 14-byte packets concatenated)
        """
        if not self.is_open:
            return b''
            
        all_packets = b''
        
        try:
            # Read all available data first
            available = self._serial.in_waiting
            if available < 2:
                return b''
            
            # Process frames continuously while data is available
            while self._serial.in_waiting >= 2:
                # Read header (2 bytes)
                header = self._serial.read(2)
                if len(header) < 2:
                    break
                    
                # Check for Waveshare RX frame: 0xAA 0xCx
                if header[0] != 0xAA or (header[1] & 0xC0) != 0xC0:
                    continue
                    
                # Parse frame info
                data_len = header[1] & 0x0F
                is_extended = (header[1] & 0x20) != 0
                
                # Calculate total bytes to read
                # For STANDARD frames: 2-byte ID
                # For EXTENDED frames: 4-byte ID
                if is_extended:
                    frame_bytes = 4 + data_len + 1  # ID(4) + Data(n) + End(1)
                else:
                    frame_bytes = 2 + data_len + 1  # ID(2) + Data(n) + End(1)
                    
                # Read rest of frame
                frame_data = self._serial.read(frame_bytes)
                if len(frame_data) < frame_bytes:
                    break
                
                # Check end marker
                if frame_data[-1] != 0x55:
                    continue
                    
                # Extract motor ID and CAN data
                if is_extended:
                    motor_id = (frame_data[0] | 
                               (frame_data[1] << 8) | 
                               (frame_data[2] << 16) | 
                               (frame_data[3] << 24))
                    can_data = frame_data[4:4+data_len]
                else:
                    # STANDARD frame: 2-byte ID (11-bit)
                    motor_id = frame_data[0] | (frame_data[1] << 8)
                    can_data = frame_data[2:2+data_len]
                    
                # Pad CAN data to 8 bytes
                can_data = can_data + bytes(8 - len(can_data))
                
                # Convert to Jason's protocol format
                jason_packet = bytearray([
                    0xAA, 0x01,  # Header
                ])
                jason_packet.extend(can_data[:8])
                jason_packet.extend([
                    motor_id & 0xFF,
                    (motor_id >> 8) & 0xFF,
                    (motor_id >> 16) & 0xFF,  # Will be 0 for standard frames
                    (motor_id >> 24) & 0xFF,  # Will be 0 for standard frames
                ])
                
                all_packets += bytes(jason_packet)
            
            return all_packets
            
        except Exception as e:
            print(f"[Waveshare RX Error] {e}")
            return b''

    def close(self):
        """Close serial port"""
        if self._serial and self._serial.is_open:
            self._serial.close()
            self.is_open = False
            print(f"{self.port} closed.")


class MotorController:
    def __init__(self, serial_device):
        self.serial_device = serial_device
        self.motors = dict()
        self.rx_buffer = bytes()

        self.tx_buffer = bytearray([
            0xAA, 0x01,
            0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC,
            0x00, 0x00, 0x00, 0x00,
            0xF4
        ])

    def add_motor(self, motor: Motor):
        self.motors[motor.id] = motor

    def set_zero_position(self, motor: Motor):
        data_buff = bytes([
            0xFF, 0xFF, 0xFF, 0xFF,
            0xFF, 0xFF, 0xFF, 0xFE
        ])
        self.__send_data(motor.id, data_buff)
        sleep(0.005)

    def reset_mode(self, motor=None):
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

        if not self.motors:
            self.__control_cmd(0xFFFF, np.uint8(0xFD))
        else:
            for mid in list(self.motors.keys()):
                self.__control_cmd(int(mid), np.uint8(0xFD))
        sleep(0.01)

    def motor_mode(self, motor: Motor):
        self.__control_cmd(motor, np.uint8(0xFC))
        sleep(0.01)

    def PTM_control(self, motor: Motor, pos, vel, kp, kd, torque, type_name="REVO"):
        """PTM (Position-Torque Mix) mode control"""
        pos_uint = float_to_uint(pos, motor.Q_MIN, motor.Q_MAX, 16)
        vel_uint = float_to_uint(vel, motor.DQ_MIN, motor.DQ_MAX, 12)
        kp_uint = float_to_uint(kp, motor.OKP_MIN, motor.OKP_MAX, 12)
        kd_uint = float_to_uint(kd, motor.OKD_MIN, motor.OKD_MAX, 12)
        tau_uint = float_to_uint(torque, motor.TAU_MIN, motor.TAU_MAX, 12)

        data_buff = bytes([
            (pos_uint >> 8) & 0xFF,
            pos_uint & 0xFF,
            (vel_uint >> 4) & 0xFF,
            ((vel_uint & 0x0F) << 4) | ((kp_uint >> 8) & 0x0F),
            kp_uint & 0xFF,
            (kd_uint >> 4) & 0xFF,
            ((kd_uint & 0x0F) << 4) | ((tau_uint >> 8) & 0x0F),
            tau_uint & 0xFF
        ])

        self.__send_data(motor.id, data_buff)

    def poll(self):
        try:
            self.__recv_data("REVO", 0)
        except Exception:
            pass

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

    def __recv_data(self, type, motor_id):
        try:
            data_recv = b''.join([self.rx_buffer, self.serial_device.read_all()])
        except Exception as e:
            return

        packets = self.__extract_packets(data_recv, type, motor_id)

        # Handle remainder bytes
        # Jason protocol: 2 (header) + 8 (data) + 4 (motor ID) = 14 bytes
        frame_length = 14
        if len(data_recv) >= frame_length:
            remainder_pos = len(data_recv) % frame_length
            self.rx_buffer = data_recv[-remainder_pos:] if remainder_pos else b''
        else:
            self.rx_buffer = data_recv

        for packet in packets:
            data = packet[2:10]  # Data is at bytes 2-9 (after 0xAA 0x01 header)
            # Extract CANID from Jason packet (bytes 10-13 for full 4-byte ID)
            CANID = packet[10] | (packet[11] << 8) | (packet[12] << 16) | (packet[13] << 24)
            self.__process_packet(data, CANID, type)

    def __extract_packets(self, data, type, motor_id):
        packets = []
        i = 0
        # Jason protocol packet size: 2 (header) + 8 (data) + 4 (motor ID) = 14 bytes
        while i < len(data) - 13:  # Need at least 14 bytes for a complete packet
            if data[i] == 0xAA and data[i + 1] == 0x01:
                packet = data[i:i + 14]  # Extract 14 bytes
                if len(packet) == 14:
                    packets.append(packet)
                i += 14
            else:
                i += 1
        return packets

    def __process_packet(self, data, CANID, type):
        if CANID not in self.motors:
            return

        motor = self.motors[CANID]

        # Parse feedback according to Motorevo protocol
        status_word = data[0]
        q_uint = (data[1] << 8) | data[2]
        dq_uint = (data[3] << 4) | (data[4] >> 4)
        tau_uint = ((data[4] & 0x0F) << 8) | data[5]
        error_code = data[6]
        temperature = data[7]

        # Convert to floats
        recv_q = uint_to_float(q_uint, motor.Q_MIN, motor.Q_MAX, 16)
        recv_dq = uint_to_float(dq_uint, motor.DQ_MIN, motor.DQ_MAX, 12)
        recv_tau = uint_to_float(tau_uint, motor.TAU_MIN, motor.TAU_MAX, 12)

        # Update motor state
        motor.get_data(status_word, recv_q, recv_dq, recv_tau, temperature, error_code)


def start_param_polling(controller, poll_interval=0.005, stop_event=None):
    """Start background thread to continuously poll parameter feedback."""
    print("[Param Polling] Thread started")
    def run():
        while not (stop_event and stop_event.is_set()):
            try:
                controller.poll()
            except Exception:
                pass
            sleep(poll_interval)
    t = threading.Thread(target=run, daemon=True)
    t.start()
    return t


# MAIN
if __name__ == "__main__":
    # Load parameters from YAML
    with open("parameters.yaml", "r") as f:
        params = yaml.safe_load(f)
    
    # Configuration from YAML
    motor_port = params['motor_port']
    param_port = params.get('param_port', None)
    motor_id = params['motor_id']
    target_pos = params['target_position']
    target_vel = params['target_velocity']
    target_torque = params['target_torque']
    kp_gain = params['kp']
    kd_gain = params['kd']
    CONTROL_DURATION = params['control_duration']

    # Initialize Waveshare wrappers
    motor_serial = WaveshareCANWrapper(motor_port)
    param_serial = WaveshareCANWrapper(param_port) if param_port else None

    # Create motor object
    revo = Motor(motor_name="Revo", motor_id=motor_id, type_name="REVO")

    # Create main controller (for TX)
    ctrl = MotorController(motor_serial)
    ctrl.add_motor(revo)

    # Setup plotting
    plt.ion()
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))
    
    time_data = []
    pos_data = []
    target_pos_data = []
    torque_data = []
    target_torque_data = []

    line_pos, = ax1.plot([], [], 'b-', label='Actual Position', linewidth=2)
    line_target_pos, = ax1.plot([], [], 'r--', label='Target Position', linewidth=2)
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Position (rad)')
    ax1.set_title('Motor Position Tracking')
    ax1.legend()
    ax1.grid(True)

    line_torque, = ax2.plot([], [], 'g-', label='Actual Torque', linewidth=2)
    line_target_torque, = ax2.plot([], [], 'm--', label='Target Torque', linewidth=2)
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Torque (Nm)')
    ax2.set_title('Motor Torque')
    ax2.legend()
    ax2.grid(True)

    plt.tight_layout()

    print("\n" + "="*70)
    print("Waveshare USB-CAN-A - PTM Mode with YAML Parameters")
    print("="*70)
    print(f"Motor Port: {motor_port}")
    print(f"Param Port: {param_port if param_port else 'None (using motor port)'}")
    print(f"Motor ID: {motor_id}")
    print("="*70 + "\n")

    # Reset motor
    print("Resetting motor...")
    ctrl.reset_mode(revo)
    sleep(1.0)

    # Enter motor mode
    print("Entering motor mode...")
    ctrl.motor_mode(revo)
    sleep(0.5)

    # Zero position
    print("Zeroing position...")
    ctrl.set_zero_position(revo)
    sleep(0.5)

    # Start parameter feedback polling (optional second port)
    stop_event = threading.Event()
    if param_serial is not None:
        try:
            param_controller = MotorController(param_serial)
            param_controller.add_motor(revo)
            start_param_polling(param_controller, poll_interval=0.005, stop_event=stop_event)
            print("✓ Parameter polling thread started.")
        except Exception as e:
            print(f"[main] Failed to start param controller: {e}\n")

    # PTM Control parameters
    print("\n" + "="*70)
    print("PTM MODE HARDWARE TEST - Position-Torque Mix Control")
    print("="*70)
    print(f"Target Position: {target_pos} rad")
    print(f"Target Velocity: {target_vel} rad/s")
    print(f"Target Torque: {target_torque} Nm (feed-forward)")
    print(f"Kp: {kp_gain}, Kd: {kd_gain}")
    print(f"Control Formula: T = Kp×(Pref-Pact) + Kd×(Vref-Vact) + Tref")
    print("="*70 + "\n")

    start = time.time()

    try:
        while time.time() - start < CONTROL_DURATION:
            now = time.time() - start

            # Send PTM command
            ctrl.PTM_control(
                revo, 
                pos=target_pos,
                vel=target_vel,
                kp=kp_gain, 
                kd=kd_gain,
                torque=target_torque
            )

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
            target_pos_data.append(target_pos)
            torque_data.append(revo.torque)
            target_torque_data.append(target_torque)

            # Update position plot
            line_pos.set_data(time_data, pos_data)
            line_target_pos.set_data(time_data, target_pos_data)
            ax1.relim()
            ax1.autoscale_view()

            # Update torque plot
            line_torque.set_data(time_data, torque_data)
            line_target_torque.set_data(time_data, target_torque_data)
            ax2.relim()
            ax2.autoscale_view()

            fig.canvas.draw()
            fig.canvas.flush_events()

            sleep(0.01)

    except KeyboardInterrupt:
        print("\n[INTERRUPTED] User stopped execution.")

    print("\nStopping motor...")
    ctrl.PTM_control(revo, pos=0, vel=0, kp=0, kd=0, torque=0)
    sleep(0.1)

    print("Resetting motor...")
    ctrl.reset_mode(revo)
    sleep(0.1)

    # Stop polling thread
    if stop_event:
        stop_event.set()
        sleep(0.05)

    # Keep plot visible
    plt.ioff()
    plt.show()

    # Close ports
    motor_serial.close()
    if param_serial:
        param_serial.close()

    print("Exiting.")
