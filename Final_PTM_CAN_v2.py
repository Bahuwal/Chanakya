from time import sleep
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

    # PTM MODE (8 bytes) - Position-Torque Mix Mode
    def PTM_control(self, motor: Motor, pos, vel, kp, kd, torque, type_name="REVO"):
        """
        PTM (Position-Torque Mix) mode control :
        
        Control Formula: T_total = Kp×(P_ref - P_actual) + Kd×(V_ref - V_actual) + T_ref
        
        Args:
            motor: Motor objects
            pos: Target position (radians), -12.5 to +12.5
            vel: Target velocity (rad/s), -10.0 to +10.0
            kp: Position proportional gain, 0.0 to 250.0
            kd: Position derivative gain, 0.0 to 50.0
            torque: Feed-forward torque (Nm), -50.0 to +50.0
            
        Byte Structure :
            Byte 0-1: Position (16-bit)
            Byte 2, 3[7:4]: Velocity (12-bit)
            Byte 3[3:0], 4: Kp (12-bit)
            Byte 5, 6[7:4]: Kd (12-bit)
            Byte 6[3:0], 7: Torque (12-bit)
        """
        # Scale parameters to integer ranges
        pos_uint = float_to_uint(pos, motor.Q_MIN, motor.Q_MAX, 16)      # 16-bit
        vel_uint = float_to_uint(vel, motor.DQ_MIN, motor.DQ_MAX, 12)    # 12-bit
        kp_uint = float_to_uint(kp, motor.OKP_MIN, motor.OKP_MAX, 12)    # 12-bit
        kd_uint = float_to_uint(kd, motor.OKD_MIN, motor.OKD_MAX, 12)    # 12-bit
        tau_uint = float_to_uint(torque, motor.TAU_MIN, motor.TAU_MAX, 12)  # 12-bit

        # Pack bytes according to manual specification
        data_buff = bytes([
            (pos_uint >> 8) & 0xFF,                          # Byte 0: Position high byte
            pos_uint & 0xFF,                                 # Byte 1: Position low byte
            (vel_uint >> 4) & 0xFF,                          # Byte 2: Velocity high 8 bits
            ((vel_uint & 0x0F) << 4) | ((kp_uint >> 8) & 0x0F),  # Byte 3: Vel[3:0] | Kp[11:8]
            kp_uint & 0xFF,                                  # Byte 4: Kp low 8 bits
            (kd_uint >> 4) & 0xFF,                           # Byte 5: Kd high 8 bits
            ((kd_uint & 0x0F) << 4) | ((tau_uint >> 8) & 0x0F),  # Byte 6: Kd[3:0] | Tau[11:8]
            tau_uint & 0xFF                                  # Byte 7: Torque low 8 bits
        ])

        self.__send_data(motor.id, data_buff)
        # Poll for immediate response
        self.poll()
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

    def __recv_data(self, type, motor_id):
        try:
            data_recv = b''.join([self.rx_buffer, self.serial_device.read_all()])
        except Exception as e:
            return

        # DEBUG: Print raw serial data ONCE
        if len(data_recv) > 0 and not hasattr(self, '_debug_printed'):
            print(f"\n[DEBUG] Port {getattr(self.serial_device, 'port', '?')} received {len(data_recv)} bytes:")
            print(f"  Hex: {data_recv.hex(' ')}")
            self._debug_printed = True

        packets = self.__extract_packets(data_recv, type, motor_id)

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
            self.__process_packet(data, CANID, type)

    def poll(self):
        """Public method to poll serial device and process incoming frames."""
        if not self.motors:
            try:
                self.__recv_data("REVO", 0)
            except Exception:
                pass
        else:
            for mid, motor in list(self.motors.items()):
                try:
                    self.__recv_data(motor.type, mid)
                except Exception:
                    pass

    def __extract_packets(self, data, type, CANID):
        frames = []
        tail1 = (CANID >> 24) & 0xFF
        tail2 = (CANID >> 16) & 0xFF
        tail3 = (CANID >> 8) & 0xFF
        tail4 = CANID & 0xFF
        tail = [tail4, tail3, tail2, tail1]

        frame_length = 12
        i = 0

        while i <= len(data) - frame_length:
            if list(data[i+8:i+12]) == tail:
                frames.append(data[i:i+frame_length])
                i += frame_length
            else:
                i += 1

        return frames

    def __process_packet(self, data, CANID, type):
        if CANID != 0x00 and CANID in self.motors:
            if type == "REVO":
                status_words = data[0]
                q_uint = (data[1] << 8) | data[2]
                dq_uint = (data[3] << 4) | (data[4] >> 4)
                tau_uint = ((data[4] & 0x0F) << 8) | data[5]

                motor = self.motors[CANID]
                recv_q = uint_to_float(q_uint, motor.Q_MIN, motor.Q_MAX, 16)
                recv_dq = uint_to_float(dq_uint, motor.DQ_MIN, motor.DQ_MAX, 12)
                recv_tau = uint_to_float(tau_uint, motor.TAU_MIN, motor.TAU_MAX, 12)

                error_code = data[6]
                temperature = data[7]
                motor.get_data(status_words, recv_q, recv_dq, recv_tau, temperature, error_code)


# CAN wrapper - acts like serial.Serial for MotorController
class CANMotorSender:
    """Wrapper that makes CAN bus look like serial port for MotorController"""
    def __init__(self, can_bus):
        self.can_bus = can_bus
        self.is_open = True
    
    def write(self, data):
        """Extract CAN data from serial protocol wrapper and send via CAN"""
        if len(data) >= 14:
            can_data = data[2:10]  # Extract 8 CAN data bytes
            motor_id = data[10] | (data[11] << 8) | (data[12] << 16) | (data[13] << 24)
            msg = can.Message(arbitration_id=motor_id, data=can_data, is_extended_id=False)
            try:
                self.can_bus.send(msg)
            except:
                pass
    
    def close(self):
        self.is_open = False


# Helper: USB config loader
def load_usb_config(path="usb.json"):
    default = {
        "param_port": "/dev/ttyACM0",
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
            print(f"[load_usb_config] failed to parse {path}: {e}. Using defaults.")
            return default["param_port"], default["baudrate"]
    else:
        print(f"[load_usb_config] {path} not found. Using defaults.")
        return default["param_port"], default["baudrate"]


# Param polling thread 
def start_param_polling(param_controller: MotorController, poll_interval=0.005, stop_event=None):
    """Start background thread to continuously poll parameter feedback."""
    def run():
        while not (stop_event and stop_event.is_set()):
            try:
                param_controller.poll()
            except Exception:
                pass
            sleep(poll_interval)
    t = threading.Thread(target=run, daemon=True)
    t.start()
    return t


# MAIN TEST LOOP
if __name__ == "__main__":
    import matplotlib.pyplot as plt
    from collections import deque
    import serial
    import time

    # Plotting Setup - 2 subplots for position and torque
    plt.ion()
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))

    time_data = deque(maxlen=200)
    pos_data = deque(maxlen=200)
    target_pos_data = deque(maxlen=200)
    torque_data = deque(maxlen=200)
    target_torque_data = deque(maxlen=200)

    # Position plot
    line_pos, = ax1.plot([], [], 'r-', label='Actual Pos', linewidth=2)
    line_target_pos, = ax1.plot([], [], 'g--', label='Target Pos', linewidth=2)
    ax1.set_ylabel('Position (rad)')
    ax1.legend(loc='upper right')
    ax1.grid(True, alpha=0.3)
    ax1.set_title('PTM Mode: Position-Torque Mix Control')

    # Torque plot
    line_torque, = ax2.plot([], [], 'b-', label='Actual Torque', linewidth=2)
    line_target_torque, = ax2.plot([], [], 'm--', label='Target Torque', linewidth=2)
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Torque (Nm)')
    ax2.legend(loc='upper right')
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()

    # Load USB Config
    param_port, BAUDRATE = load_usb_config("usb.json")
    print(f"Using param_port={param_port}, baudrate={BAUDRATE}")
    print(f"Using CAN: can0 @ 1Mbps")

    # Create CAN bus (replaces motor_serial)
    try:
        motor_can_bus = can.Bus(channel='can0', interface='socketcan', bitrate=1000000)
        motor_serial = CANMotorSender(motor_can_bus)
        print(f"✓ CAN opened for motor commands")
    except Exception as e:
        print(f"[main] Failed to open CAN: {e}")
        raise SystemExit("CAN not available.")

    try:
        param_serial = serial.Serial(port=param_port, baudrate=BAUDRATE, timeout=0.05)
    except Exception as e:
        print(f"[main] Failed to create param_serial on {param_port}: {e}")
        param_serial = None

    # Initialize motor
    revo = Motor("revo_motor", motor_id=7, type_name="REVO")

    # Command controller (sends via motor_port)
    ctrl = MotorController(motor_serial)
    ctrl.add_motor(revo)
    ctrl.motor_mode(revo)

    # Parameter reader controller (reads feedback)
    param_controller = None
    stop_event = threading.Event()
    
    if param_serial is not None:
        try:
            param_controller = MotorController(param_serial)
            param_controller.add_motor(revo)  # Same motor object
            start_param_polling(param_controller, poll_interval=0.005, stop_event=stop_event)
            print("✓ Parameter polling thread started.")
        except Exception as e:
            print(f"[main] Failed to start param controller: {e}")

    # PTM Control parameters
    target_pos = 5.0       # Target position in radians
    target_vel = 0.0       # Target velocity (usually 0 for position control)
    target_torque = 2.0    # Feed-forward torque in Nm (gravity compensation)
    kp_gain = 20.0         # Position control stiffness
    kd_gain = 0.8          # Position damping
    
    CONTROL_DURATION = 8
    start = time.time()

    print("\n" + "="*70)
    print("PTM MODE HARDWARE TEST - Position-Torque Mix Control")
    print("="*70)
    print(f"Target Position: {target_pos} rad")
    print(f"Target Velocity: {target_vel} rad/s")
    print(f"Target Torque: {target_torque} Nm (feed-forward)")
    print(f"Kp: {kp_gain}, Kd: {kd_gain}")
    print(f"Control Formula: T = Kp×(Pref-Pact) + Kd×(Vref-Vact) + Tref")
    print("="*70 + "\n")

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

            # AUTO ZERO POSITION FEATURE
            if abs(revo.pos - target_pos) < 0.05:
                print("Reached target → Zeroing position...")
                ctrl.set_zero_position(revo)
                target_pos = 0 

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

    # Close serial ports
    try:
        if motor_serial and motor_serial.is_open:
            motor_serial.close()
            print(f"{motor_port} closed.")
    except Exception:
        pass
    
    try:
        if param_serial and param_serial.is_open:
            param_serial.close()
            print(f"{param_port} closed.")
    except Exception:
        pass

    print("Exiting.")
