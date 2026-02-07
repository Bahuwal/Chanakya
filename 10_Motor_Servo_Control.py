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

        packets = self.__extract_packets(data_recv, type, motor_id)

        # Handle remainder bytes (12-byte CAN frames)
        frame_length = 12
        if len(data_recv) >= frame_length:
            remainder_pos = len(data_recv) % frame_length
            self.rx_buffer = data_recv[-remainder_pos:] if remainder_pos else b''
        else:
            self.rx_buffer = data_recv

        for packet in packets:
            # CAN frame: First 8 bytes are data, last 4 bytes are CAN ID
            data = packet[0:8]
            CANID = packet[8] | (packet[9] << 8) | (packet[10] << 16) | (packet[11] << 24)
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
        """Extract 12-byte CAN feedback frames.
        Each frame: [8 bytes data] + [4 bytes CAN ID (motor ID)]
        """
        frames = []
        frame_length = 12
        i = 0

        # Extract all complete 12-byte frames
        while i <= len(data) - frame_length:
            frame = data[i:i+frame_length]
            # Extract CAN ID from last 4 bytes (little-endian)
            frame_can_id = frame[8] | (frame[9] << 8) | (frame[10] << 16) | (frame[11] << 24)
            
            # Check if this CAN ID (motor ID) is one we're tracking
            if frame_can_id in self.motors:
                frames.append(frame)
                i += frame_length
            else:
                # Not a valid frame, advance by 1 byte
                i += 1

        return frames

    def __process_packet(self, data, CANID, type):
        """Process 12-byte CAN feedback frame.
        Data format (first 8 bytes): [Status, Pos_H, Pos_L, Vel/Torque_1, Vel/Torque_2, Torque_L, Error, Temp]
        CAN ID (last 4 bytes): Motor ID
        """
        if CANID != 0x00 and CANID in self.motors:
            if type == "REVO":
                status_words = data[0]
                # Position: bytes 1-2 (16-bit unsigned)
                q_uint = (data[1] << 8) | data[2]
                # Velocity: 12-bit packed in bytes 3-4
                dq_uint = (data[3] << 4) | (data[4] >> 4)
                # Torque: 12-bit packed in bytes 4-5
                tau_uint = ((data[4] & 0x0F) << 8) | data[5]
                # Error code: byte 6
                error_code = data[6]
                # Temperature: byte 7
                temperature = data[7]
                
                motor = self.motors[CANID]
                recv_q = uint_to_float(q_uint, motor.Q_MIN, motor.Q_MAX, 16)
                recv_dq = uint_to_float(dq_uint, motor.DQ_MIN, motor.DQ_MAX, 12)
                recv_tau = uint_to_float(tau_uint, motor.TAU_MIN, motor.TAU_MAX, 12)
                
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


# TEN MOTOR TEST LOOP WITH AUTO-ZERO SERVO MODE
if __name__ == "__main__":
    import matplotlib.pyplot as plt
    from collections import deque
    import serial
    import time

    # Plotting Setup (10 Motors in 5x2 grid)
    plt.ion()
    fig, axes = plt.subplots(5, 2, figsize=(14, 16))
    axes = axes.flatten()  # Convert to 1D array for easier indexing
    
    # Data storage for each motor
    time_data = [deque(maxlen=200) for _ in range(10)]
    pos_data = [deque(maxlen=200) for _ in range(10)]
    target_data = [deque(maxlen=200) for _ in range(10)]
    
    # Plot lines for each motor
    line_pos = []
    line_target = []
    
    colors = ['red', 'blue', 'green', 'orange', 'purple', 
              'brown', 'pink', 'gray', 'olive', 'cyan']
    
    for i in range(10):
        ax = axes[i]
        lp, = ax.plot([], [], color=colors[i], linewidth=2, label=f'Motor {i+1} Actual')
        lt, = ax.plot([], [], '--', color='black', linewidth=1, label=f'Motor {i+1} Target')
        line_pos.append(lp)
        line_target.append(lt)
        
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)
        ax.set_ylabel('Position', fontsize=9)
        ax.set_title(f'Motor {i+1}', fontsize=10, fontweight='bold')
        
        # Set xlabel only for bottom row
        if i >= 8:
            ax.set_xlabel('Time (s)', fontsize=9)
    
    plt.tight_layout()

    # Load USB Config
    motor_port, param_port, BAUDRATE = load_usb_config("usb.json")
    print(f"Using motor_port={motor_port}, baudrate={BAUDRATE}")

    # Create serial device
    try:
        motor_serial = serial.Serial(port=motor_port, baudrate=BAUDRATE, timeout=0.05)
    except Exception as e:
        print(f"[main] Failed to create motor_serial on {motor_port}: {e}")
        raise SystemExit("Motor serial not available.")

    # INITIALIZE 10 MOTORS
    motor_ids = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
    motors = []
    
    print("\n=== Initializing 10 Motors ===")
    for i, mid in enumerate(motor_ids):
        motor = Motor(f"motor_{mid}", motor_id=mid, type_name="REVO")
        motors.append(motor)
        print(f"  Motor {i+1}: ID={mid}")

    # Command controller (sends via motor_serial = COM3)
    ctrl = MotorController(motor_serial)
    for motor in motors:
        ctrl.add_motor(motor)
        ctrl.motor_mode(motor)

    # Feedback comes on COM3, poll the same controller
    stop_event = threading.Event()
    start_param_polling(ctrl, poll_interval=0.005, stop_event=stop_event)
    print("✓ Polling thread started on COM3 (motor_serial).")

    # MOTOR CONTROL PARAMETERS
    target_positions = [10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0]
    velocities = [2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0]
    
    # Track which motors have been zeroed
    motors_zeroed = [False] * 10

    CONTROL_DURATION = 5  # seconds
    start = time.time()

    print(f"\n=== Starting Control Loop ===")
    print(f"Duration: {CONTROL_DURATION}s")
    print(f"Target positions: {target_positions}")
    print(f"Velocities: {velocities}\n")
    sleep(0.5)  # Give time for initial feedback

    try:
        while time.time() - start < CONTROL_DURATION:
            now = time.time() - start

            # CONTROL ALL 10 MOTORS
            for i, motor in enumerate(motors):
                ctrl.Servo_control(
                    motor, 
                    pos=target_positions[i], 
                    vel=velocities[i],
                    kp=50, kd=5, ikp=30, ikd=0.01, iki=0
                )
                sleep(0.001)  # Small delay between motor commands

            # AUTO ZERO POSITION FOR EACH MOTOR
            for i, motor in enumerate(motors):
                if not motors_zeroed[i] and abs(motor.pos - target_positions[i]) < 0.05:
                    print(f"Motor {motor.id} reached target → Zeroing position...")
                    ctrl.set_zero_position(motor)
                    target_positions[i] = 0   # next command is relative
                    motors_zeroed[i] = True

            # UPDATE PLOTTING DATA FOR ALL MOTORS
            for i, motor in enumerate(motors):
                time_data[i].append(now)
                pos_data[i].append(motor.pos)
                target_data[i].append(target_positions[i])
                
                line_pos[i].set_data(time_data[i], pos_data[i])
                line_target[i].set_data(time_data[i], target_data[i])
                
                axes[i].relim()
                axes[i].autoscale_view()
            
            fig.canvas.draw()
            fig.canvas.flush_events()

            # Print feedback every 1 second
            if int(now * 10) % 10 == 0:
                print(f"\n--- Time: {now:.2f}s ---")
                for i, motor in enumerate(motors):
                    print(f"  Motor {motor.id}: Pos={motor.pos:.2f}, "
                          f"Vel={motor.vel:.2f}, Target={target_positions[i]:.2f}, "
                          f"Zeroed={motors_zeroed[i]}")

            sleep(0.01)

    except KeyboardInterrupt:
        print("\n[Interrupted by user]")

    print("\n=== Stopping all motors ===")
    for motor in motors:
        ctrl.Servo_control(motor, pos=0, vel=0, kp=0, kd=0, ikp=0, ikd=0, iki=0)
        sleep(0.001)
    sleep(0.1)

    print("=== Resetting all motors ===")
    for motor in motors:
        ctrl.reset_mode(motor)
        sleep(0.01)

    # Stop polling thread
    if stop_event:
        stop_event.set()
        sleep(0.05)

    # Keep plots visible
    plt.ioff()
    plt.show()

    # Close serial port
    try:
        if motor_serial and motor_serial.is_open:
            motor_serial.close()
            print(f"{motor_port} closed.")
    except Exception:
        pass

    print("Exiting.")
