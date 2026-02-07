from time import sleep
import json
import threading
import os
import numpy as np
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

    # NEW: Zero Position Command
    def set_zero_position(self, motor: Motor):
        data_buff = bytes([
            0xFF, 0xFF, 0xFF, 0xFF,
            0xFF, 0xFF, 0xFF, 0xFE  # <-- Zero position
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
        pos = float_to_uint(pos, motor.Q_MIN, motor.Q_MAX, 16)
        vel = float_to_uint(vel, motor.DQ_MIN, motor.DQ_MAX, 8)
        kp = float_to_uint(kp, motor.OKP_MIN, motor.OKP_MAX, 8)
        kd = float_to_uint(kd, motor.OKD_MIN, motor.OKD_MAX, 8)
        ikp = float_to_uint(ikp, motor.IKP_MIN, motor.IKP_MAX, 8)
        ikd = float_to_uint(ikd, motor.IKD_MIN, motor.IKD_MAX, 8)
        iki = float_to_uint(iki, motor.IKI_MIN, motor.IKI_MAX, 8)

        data_buff = bytes([
            (pos >> 8) & 0xFF,
            pos & 0xFF,
            vel & 0xFF,
            kp & 0xFF,
            kd & 0xFF,
            ikp & 0xFF,
            ikd & 0xFF,
            iki & 0xFF
        ])

        self.__send_data(motor.id, data_buff)
        # Poll for immediate response (optional)
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

    def poll(self): #Read Parameters from Motors
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


# Helper: USB config loader
def load_usb_config(path="usb.json"):
    default = {
        "motor_port": "COM3",
        "param_port": "COM4",
        "baudrate": 921600
    }
    if os.path.exists(path):
        try:
            with open(path, "r") as f:
                cfg = json.load(f)
            motor_port = cfg.get("motor_port", default["motor_port"])
            param_port = cfg.get("param_port", default["param_port"])
            baudrate = int(cfg.get("baudrate", default["baudrate"]))
            return motor_port, param_port, baudrate
        except Exception as e:
            print(f"[load_usb_config] failed to parse {path}: {e}. Using defaults.")
            return default["motor_port"], default["param_port"], default["baudrate"]
    else:
        print(f"[load_usb_config] {path} not found. Using defaults.")
        return default["motor_port"], default["param_port"], default["baudrate"]


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


# TEST LOOP WITH AUTO-ZERO SERVO MODE
if __name__ == "__main__":
    import matplotlib.pyplot as plt
    from collections import deque
    import serial
    import time

    # Plotting Setup
    plt.ion()
    fig, ax = plt.subplots()

    time_data = deque(maxlen=200)
    pos_data = deque(maxlen=200)
    target_data = deque(maxlen=200)

    line_pos, = ax.plot([], [], 'r-', label='Actual Pos')
    line_target, = ax.plot([], [], 'g--', label='Target Pos')
    ax.legend()
    ax.grid()

    # Load USB Config
    motor_port, param_port, BAUDRATE = load_usb_config("usb.json")
    print(f"Using motor_port={motor_port}, param_port={param_port}, baudrate={BAUDRATE}")

    # Create serial devices
    try:
        motor_serial = serial.Serial(port=motor_port, baudrate=BAUDRATE, timeout=0.05)
    except Exception as e:
        print(f"[main] Failed to create motor_serial on {motor_port}: {e}")
        raise SystemExit("Motor serial not available.")

    try:
        param_serial = serial.Serial(port=param_port, baudrate=BAUDRATE, timeout=0.05)
    except Exception as e:
        print(f"[main] Failed to create param_serial on {param_port}: {e}")
        param_serial = None

    # Initialize motor
    revo = Motor("revo_motor", motor_id=3, type_name="REVO")

    # Command controller (sends via Jason)
    ctrl = MotorController(motor_serial)
    ctrl.add_motor(revo)
    ctrl.motor_mode(revo)

    # Parameter reader controller (reads feedback)
    param_controller = None
    stop_event = threading.Event()
    
    if param_serial is not None:
        try:
            param_controller = MotorController(param_serial)
            param_controller.add_motor(revo)  # Same motor object!
            start_param_polling(param_controller, poll_interval=0.005, stop_event=stop_event)
            print("✓ Parameter polling thread started.")
        except Exception as e:
            print(f"[main] Failed to start param controller: {e}")

    # Control parameters
    target_pos = 10
    sent_vel = 2
    CONTROL_DURATION = 5
    start = time.time()

    try:
        while time.time() - start < CONTROL_DURATION:
            now = time.time() - start

            ctrl.Servo_control(
                revo, pos=target_pos, vel=sent_vel,
                kp=50, kd=5, ikp=30, ikd=0.01, iki=0
            )

            # AUTO ZERO POSITION FEATURE
            if abs(revo.pos - target_pos) < 0.05:
                print("Reached target → Zeroing position...")
                ctrl.set_zero_position(revo)
                target_pos = 0   # next command is relative

            # Print feedback
            print(f"Time: {now:.2f}s, "
                  f"Pos: {revo.pos:.2f}, "
                  f"Vel: {revo.vel:.2f}, "
                  f"Temp: {revo.temperature:.1f}, "
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
        pass

    print("Stopping motor...")
    ctrl.Servo_control(revo, pos=0, vel=0, kp=0, kd=0, ikp=0, ikd=0, iki=0)
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