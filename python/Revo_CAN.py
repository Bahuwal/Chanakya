from time import sleep
import numpy as np
from utils import float_to_uint, uint_to_float, float_to_uint

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
    def __init__(self, serial_device):
        self.serial_device = serial_device
        self.motors = dict()
        self.rx_buffer = bytes()
        self.tx_buffer = bytearray([0xAA, 0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0XFF, 0XFF, 0XFC, 0X00, 0X00,  0X00, 0x00, 0XF4])
        if self.serial_device.is_open:
            self.serial_device.close()
        try:
            self.serial_device.open()
            print(f"Serial port {self.serial_device.port} opened successfully.")
        except Exception as e:
            print(f"Error opening serial port: {e}")

    def add_motor(self, motor: Motor):
        self.motors[motor.id] = motor
        return True
    
    def reset_mode(self):
        self.__control_cmd(Motor, np.uint8(0xFD))
        sleep(0.01)

    def motor_mode(self,motor: Motor):
        self.__control_cmd(motor, np.uint8(0xFC))
        sleep(0.01)

    def PTM_control(self, motor: Motor, pos, vel, torque, kp, kd, type_name="REVO"):
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
                ((vel & 0x0F) << 4) | ((kp >> 8) & 0x0F),            # msg[3]
                kp & 0xFF,                                          # msg[4]
                (kd >> 4) & 0xFF,                                   # msg[5]
                ((kd & 0x0F) << 4) | ((torque >> 8) & 0x0F),         # msg[6]
                torque & 0xFF                                       # msg[7]
            ])
        elif type_name == "ENCOS":
            pos = float_to_uint(pos, motor.Q_MIN, motor.Q_MAX, 16)
            vel = float_to_uint(vel, motor.DQ_MIN, motor.DQ_MAX, 12)
            torque = float_to_uint(torque, motor.TAU_MIN, motor.TAU_MAX, 12)
            kp = float_to_uint(kp, motor.OKP_MIN, motor.OKP_MAX, 12)
            kd = float_to_uint(kd, motor.OKD_MIN, motor.OKD_MAX, 9)
            switch_user_mode = 0
            data_buff = bytearray(8)
            data_buff[0] = ((switch_user_mode & 0x07) << 5) | ((kp >> 7) & 0x1F)
            data_buff[1] = ((kp & 0x7F) << 1) | ((kd >> 8) & 0x01)
            data_buff[2] = kd & 0xFF
            data_buff[3] = (pos >> 8) & 0xFF
            data_buff[4] = pos & 0xFF
            data_buff[5] = (vel >> 4) & 0xFF
            data_buff[6] = ((vel & 0x0F) << 4) | ((torque >> 8) & 0x0F)
            data_buff[7] = torque & 0xFF


        self.__send_data(motor.id, data_buff)
        self.__recv_data(motor.type, motor.id)
        sleep(0.001)


    def __control_cmd(self, motor: Motor, cmd):
        data_buff = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0XFF, 0XFF, cmd])
        self.__send_data(motor.id, data_buff)

    def __send_data(self, motor_id, data):
        self.tx_buffer[10] = motor_id & 0xff
        self.tx_buffer[11] = (motor_id >> 8)& 0xff
        self.tx_buffer[12] = (motor_id >> 16)& 0xff
        self.tx_buffer[13] = (motor_id >> 24)& 0xff
        self.tx_buffer[2:10] = data
        self.serial_device.write(self.tx_buffer)

    def __recv_data(self,type,motor_id):
        data_recv = b''.join([self.rx_buffer, self.serial_device.read_all()])
        packets = self.__extract_packets(data_recv,type,motor_id)
        # print("Extracted packets:", len(data_recv))
        for packet in packets:
            data = packet[0:8]
            CANID = (packet[11] << 24) | (packet[10] << 16) | (packet[9] << 8) | packet[8]
            self.__process_packet(data, CANID, type)

    def __extract_packets(self, data, type, CANID):
        frames = []
        header = 0x20

        tail1 = (CANID >> 24) & 0xFF
        tail2 = (CANID >> 16) & 0xFF
        tail3 = (CANID >> 8) & 0xFF
        tail4 = CANID & 0xFF
        tail = [tail4, tail3, tail2, tail1]
        
        if type == "REVO":
            header = CANID & 0xFF
        elif type == "ENCOS":
            header = 0x20
            
        frame_length = 12
        i = 0
        remainder_pos = 0

        # according to the tail to extract frames
        while i <= len(data) - frame_length:
            if list(data[i+8:i+12]) == tail:
                frame = data[i:i + frame_length]
                frames.append(frame)
                i += frame_length
                remainder_pos = i
            else:
                i += 1
        # while i <= len(data) - frame_length:
        #     if data[i] == header:
        #         frame = data[i:i + frame_length]
        #         frames.append(frame)
        #         i += frame_length
        #         remainder_pos = i
        #     else:
        #         i += 1
        # self.data_save = data[remainder_pos:]
        return frames
    
    def __process_packet(self, data, CANID, type):
        # print("Received data from CAN ID:", CANID)
        if CANID != 0x00:
            if CANID in self.motors:
                if type == "REVO":
                    status_words = data[0]
                    q_uint = (data[1] << 8) | data[2]
                    dq_uint = (data[3] << 4) | (data[4] >> 4)
                    tau_uint = ((data[4] & 0x0F) << 8) | data[5]
                    Q_MAX = self.motors[CANID].Q_MAX
                    DQ_MAX = self.motors[CANID].DQ_MAX
                    TAU_MAX = self.motors[CANID].TAU_MAX
                    Q_MIN = self.motors[CANID].Q_MIN
                    DQ_MIN = self.motors[CANID].DQ_MIN
                    TAU_MIN = self.motors[CANID].TAU_MIN
                    recv_q = uint_to_float(q_uint, Q_MIN, Q_MAX, 16)
                    recv_dq = uint_to_float(dq_uint, DQ_MIN, DQ_MAX, 12)
                    recv_tau = uint_to_float(tau_uint, TAU_MIN, TAU_MAX, 12)
                    error_code = data[6]
                    temperature = data[7]
                    self.motors[CANID].get_data(status_words, recv_q, recv_dq, recv_tau, temperature, error_code)
                elif type == "ENCOS":
                    q_uint = (data[1] << 8) | data[2]
                    dq_uint = (data[3] << 4) | (data[4] >> 4)
                    tau_uint = ((data[4] & 0x0F) << 8) | data[5]
                    
                    Q_MAX = self.motors[CANID].Q_MAX
                    DQ_MAX = self.motors[CANID].DQ_MAX
                    TAU_MAX = self.motors[CANID].TAU_MAX
                    Q_MIN = self.motors[CANID].Q_MIN
                    DQ_MIN = self.motors[CANID].DQ_MIN
                    TAU_MIN = self.motors[CANID].TAU_MIN
                    CUR_MAX = self.motors[CANID].CUR_MAX
                    CUR_MIN = self.motors[CANID].CUR_MIN

                    recv_q = uint_to_float(q_uint, Q_MIN, Q_MAX, 16)
                    recv_dq = uint_to_float(dq_uint, DQ_MIN, DQ_MAX, 12)
                    recv_tau = uint_to_float(tau_uint, CUR_MIN, CUR_MAX, 12)
                    # print(TAU_MIN, TAU_MAX,tau_uint)
                    
                    status_words = data[0]
                    error_code = data[0] & 0x1F
                    motor_temperature = data[6]
                    pcb_temperature = data[7]
                    
                    self.motors[CANID].get_data(status_words, recv_q, recv_dq, recv_tau, motor_temperature, error_code)

    
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

    revo_motor = Motor(motor_name="revo_motor", motor_id=1, type_name="REVO")
                        # Q_MAX=12.5, Q_MIN=-12.5, 
                        # DQ_MAX=18.0, DQ_MIN=-18.0, 
                        # TAU_MAX=150.0, TAU_MIN=-150.0, 
                        # OKP_MAX=500.0, OKP_MIN=0.0,
                        # OKD_MAX=5.0, OKD_MIN=0.0,
                        # CUR_MAX=150.0, CUR_MIN=-150.0)

    # open com14 serial port for encose motor
    import serial
    import time
    serial_device = serial.Serial('COM14', 921600, timeout=0.5)

    controlelr = MotorController(serial_device)

    controlelr.add_motor(revo_motor)

    controlelr.motor_mode(revo_motor)

    start_time = time.time()

    
    while True:
        current_time = time.time() - start_time
        # target_torq = 20.0 * np.sin(0.5 * np.pi * 1 * time.time())
        target_vel = 2.0 * np.sin(0.5 * np.pi * 1 * time.time())
        
        controlelr.PTM_control(revo_motor,  pos=0,  vel=target_vel, torque=0, kp=0.0, kd=5.0, type_name="REVO")
        # print(f"Revo Motor - Pos: {revo_motor.getPosition():.2f}, Vel: {revo_motor.getVelocity():.2f}, Torque: {revo_motor.getTorque():.2f}, Temp: {revo_motor.getTemperature():.2f}, Error: {revo_motor.getErrorCode()}")

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






