from time import sleep
import numpy as np
import can
import threading
import time
from utils import float_to_uint, uint_to_float

class Motor:
    def __init__(self, 
                 motor_name = None, 
                 motor_id = 1,
                 Q_MAX = 12.5,
                 DQ_MAX = 10.0,
                 TAU_MAX = 50.0,
                 Q_MIN = -12.5,
                 DQ_MIN = -10.0,
                 TAU_MIN = -50.0,
                 OKP_MAX = 250.0,
                 OKD_MAX = 50.0,
                 IKP_MAX = 250.0,
                 IKI_MAX = 50.0,
                 OKP_MIN = 0.0,
                 OKD_MIN = 0.0,
                 IKP_MIN = 0.0,
                 IKI_MIN = 0.0,
                 CUR_MAX = 100.0,
                 CUR_MIN = -100.0,
                 type_name = "REVO"):
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
        self.IKI_MAX = IKI_MAX
        self.OKP_MIN = OKP_MIN
        self.OKD_MIN = OKD_MIN
        self.IKP_MIN = IKP_MIN
        self.IKI_MIN = IKI_MIN
        self.CUR_MAX = CUR_MAX
        self.CUR_MIN = CUR_MIN
        self.type = type_name  # "REVO" or "ENCOSE"

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
    def __init__(self, can_bus):
        """Initialize with CAN bus for direct SocketCAN communication"""
        self.can_bus = can_bus
        self.motors = dict()
        self.running = False
        self.rx_thread = None

    def add_motor(self, motor: Motor):
        self.motors[motor.id] = motor
        return True
    
    def start_can_feedback(self):
        """Start CAN feedback reading thread"""
        if self.running:
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
        
        # Parse feedback according to official Motorevo format (REVO type)
        status_words = data[0]
        q_uint = (data[1] << 8) | data[2]
        dq_uint = (data[3] << 4) | (data[4] >> 4)
        tau_uint = ((data[4] & 0x0F) << 8) | data[5]
        error_code = data[6]
        temperature = data[7]
        
        # Convert to floats
        recv_q = uint_to_float(q_uint, motor.Q_MIN, motor.Q_MAX, 16)
        recv_dq = uint_to_float(dq_uint, motor.DQ_MIN, motor.DQ_MAX, 12)
        recv_tau = uint_to_float(tau_uint, motor.TAU_MIN, motor.TAU_MAX, 12)
        
        motor.get_data(status_words, recv_q, recv_dq, recv_tau, temperature, error_code)
    
    def reset_mode(self, motor: Motor):
        """Reset motor mode"""
        self.__control_cmd(motor, np.uint8(0xFD))
        sleep(0.01)

    def motor_mode(self, motor: Motor):
        """Enter motor mode"""
        self.__control_cmd(motor, np.uint8(0xFC))
        sleep(0.01)

    def PTM_control(self, motor: Motor, pos, vel, torque, kp, kd, type_name="REVO"):
        """PTM (Position-Torque-Mix) control mode"""
        if type_name == "REVO":
            pos = float_to_uint(pos, motor.Q_MIN, motor.Q_MAX, 16)
            vel = float_to_uint(vel, motor.DQ_MIN, motor.DQ_MAX, 12)
            torque = float_to_uint(torque, motor.TAU_MIN, motor.TAU_MAX, 12)
            kp = float_to_uint(kp, motor.OKP_MIN, motor.OKP_MAX, 12)
            kd = float_to_uint(kd, motor.OKD_MIN, motor.OKD_MAX, 12)
            data_buff = bytes([
                (pos >> 8) & 0xFF,                                  # msg[0]
                pos & 0xFF,                                         # msg[1]
                (vel >> 4) & 0xFF,                                  # msg[2]
                ((vel & 0x0F) << 4) | ((kp >> 8) & 0x0F),          # msg[3]
                kp & 0xFF,                                          # msg[4]
                (kd >> 4) & 0xFF,                                   # msg[5]
                ((kd & 0x0F) << 4) | ((torque >> 8) & 0x0F),       # msg[6]
                torque & 0xFF                                       # msg[7]
            ])

        self.__send_data(motor.id, data_buff)
        sleep(0.001)

    def __control_cmd(self, motor: Motor, cmd):
        """Send control command (reset, motor_mode, etc.)"""
        data_buff = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, cmd])
        self.__send_data(motor.id, data_buff)

    def __send_data(self, motor_id, data):
        """Send CAN message directly"""
        msg = can.Message(
            arbitration_id=motor_id,
            data=data[:8],
            is_extended_id=False
        )
        try:
            self.can_bus.send(msg)
        except Exception as e:
            print(f"[CAN TX ERROR] {e}")

    
if __name__ == "__main__":
    import matplotlib.pyplot as plt
    from collections import deque

    # --- Plotting Setup ---
    plt.ion()  # Turn on interactive mode
    fig, ax = plt.subplots()
    max_points = 100  # Number of points to display on the plot
    time_data = deque(maxlen=max_points)
    revo_pos_data = deque(maxlen=max_points)
    revo_vel_data = deque(maxlen=max_points)
    target_vel_data = deque(maxlen=max_points)

    line_revo_p, = ax.plot([], [], 'r-', label='Revo Position')
    line_revo_v, = ax.plot([], [], 'b-', label='Revo Velocity')
    line_revo_target_v, = ax.plot([], [], 'g--', label='Target Velocity')
    
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("data")
    ax.set_title("Motor Data Real-time Plot")
    ax.legend()
    ax.grid(True)
    # --- End Plotting Setup ---

    # Create motor with ID 1 (change to match your motor's ID)
    revo_motor = Motor(motor_name="revo_motor", motor_id=1, type_name="REVO")

    # Open CAN bus (native SocketCAN on Jetson)
    print("Opening CAN bus: can0 @ 1Mbps")
    try:
        can_bus = can.Bus(channel='can0', interface='socketcan', bitrate=1000000)
        print("✓ CAN bus opened successfully")
    except Exception as e:
        print(f"❌ Failed to open CAN bus: {e}")
        print("Make sure to run:")
        print("  sudo ip link set can0 type can bitrate 1000000")
        print("  sudo ip link set can0 up")
        exit(1)

    # Create controller
    controller = MotorController(can_bus)
    controller.add_motor(revo_motor)
    controller.start_can_feedback()
    
    # Enter motor mode
    controller.motor_mode(revo_motor)
    sleep(0.1)

    start_time = time.time()

    try:
        while True:
            current_time = time.time() - start_time
            
            # Sinusoidal velocity target
            target_vel = 2.0 * np.sin(0.5 * np.pi * 1 * time.time())
            
            # Send PTM command (velocity control with damping)
            controller.PTM_control(revo_motor, pos=0, vel=target_vel, torque=0, kp=0.0, kd=5.0, type_name="REVO")
            
            # Print feedback
            print(f"Pos: {revo_motor.getPosition():.2f} | "
                  f"Vel: {revo_motor.getVelocity():.2f} | "
                  f"Torque: {revo_motor.getTorque():.2f} | "
                  f"Temp: {revo_motor.getTemperature():.0f}°C | "
                  f"Err: 0x{revo_motor.getErrorCode():02X}")

            # --- Update Plot Data ---
            time_data.append(current_time)
            revo_pos_data.append(revo_motor.pos)
            revo_vel_data.append(revo_motor.vel)    
            target_vel_data.append(target_vel)

            # Update the plot lines
            line_revo_p.set_data(time_data, revo_pos_data)
            line_revo_v.set_data(time_data, revo_vel_data)
            line_revo_target_v.set_data(time_data, target_vel_data)

            # Adjust plot limits
            ax.relim()
            ax.autoscale_view()
            
            # Redraw the plot
            fig.canvas.draw()
            fig.canvas.flush_events()

            sleep(0.01)

    except KeyboardInterrupt:
        print("\n[INTERRUPTED] Stopping motor...")
        controller.PTM_control(revo_motor, pos=0, vel=0, torque=0, kp=0.0, kd=0.0)
        sleep(0.1)
        controller.reset_mode(revo_motor)
        sleep(0.1)
        controller.stop_can_feedback()
        can_bus.shutdown()
        print("Exiting.")
