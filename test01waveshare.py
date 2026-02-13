import serial
import threading
import time

# =========================================================
# WAVESHARE USB-CAN (UART)
# =========================================================

class WaveshareUARTCAN:
    def __init__(self, port, baudrate=2_000_000, timeout=0.1):
        self.ser = serial.Serial(port, baudrate, timeout=timeout)
        self.running = False
        self.rx_cb = None

    def close(self):
        self.running = False
        self.ser.close()

    def send_frame(self, can_id, data):
        if len(data) != 8:
            raise ValueError("Motorevo requires 8-byte CAN frames")

        frame = bytearray([
            0xAA,
            0xC8,  # standard frame, DLC=8
            can_id & 0xFF,
            (can_id >> 8) & 0xFF,
            *data,
            0x55
        ])
        self.ser.write(frame)

    def start_rx(self, callback):
        self.rx_cb = callback
        self.running = True
        threading.Thread(target=self._rx_loop, daemon=True).start()

    def _rx_loop(self):
        while self.running:
            if self.ser.read(1) != b'\xAA':
                continue
            if self.ser.read(1) != b'\xC8':
                continue

            can_id = int.from_bytes(self.ser.read(2), "little")
            data = self.ser.read(8)
            self.ser.read(1)  # 0x55

            if self.rx_cb:
                self.rx_cb(can_id, data)


# =========================================================
# MOTOREVO PROTOCOL
# =========================================================

ENTER_MOTOR_STATE = bytes([0xFF]*7 + [0xFC])
ENTER_REST_STATE  = bytes([0xFF]*7 + [0xFD])
SET_ZERO_POSITION = bytes([0xFF]*7 + [0xFE])


def float_to_uint(x, xmin, xmax, bits):
    return int((x - xmin) * ((1 << bits) - 1) / (xmax - xmin))


def servo_cmd(pos, vel, kp, kd, vkp, vkd, vki):
    p = float_to_uint(pos, -12.5, 12.5, 16)
    v = float_to_uint(vel, -10, 10, 8)
    kp = float_to_uint(kp, 0, 250, 8)
    kd = float_to_uint(kd, 0, 50, 8)
    vkp = float_to_uint(vkp, 0, 250, 8)
    vkd = float_to_uint(vkd, 0, 50, 8)
    vki = float_to_uint(vki, 0, 0.05, 8)

    return bytes([
        (p >> 8) & 0xFF, p & 0xFF,
        v, kp, kd, vkp, vkd, vki
    ])


def decode_feedback(data):
    pos_raw = (data[1] << 8) | data[2]
    vel_raw = (data[3] << 4) | (data[4] >> 4)
    torque_raw = ((data[4] & 0x0F) << 8) | data[5]

    return {
        "state": data[0],
        "position": pos_raw / 65535 * 25 - 12.5,
        "velocity": vel_raw / 4095 * 20 - 10,
        "torque": torque_raw / 4095 * 100 - 50,
        "fault": data[6],
        "temperature": data[7]
    }


# =========================================================
# MOTOREVO HIGH-LEVEL DRIVER
# =========================================================

class MotorevoMotor:
    def __init__(self, port, motor_id, period=0.01):
        self.motor_id = motor_id
        self.period = period
        self.can = WaveshareUARTCAN(port)
        self.tx_data = ENTER_REST_STATE
        self.running = False
        self.feedback = None

        self.can.start_rx(self._on_rx)

    def _on_rx(self, can_id, data):
        if can_id == self.motor_id:
            self.feedback = decode_feedback(data)

    def enter_motor(self):
        self.tx_data = ENTER_MOTOR_STATE

    def enter_rest(self):
        self.tx_data = ENTER_REST_STATE

    def set_zero(self):
        self.tx_data = SET_ZERO_POSITION

    def servo(self, *args):
        self.tx_data = servo_cmd(*args)

    def start(self):
        self.running = True
        threading.Thread(target=self._tx_loop, daemon=True).start()

    def stop(self):
        self.enter_rest()
        time.sleep(0.05)
        self.running = False
        self.can.close()

    def _tx_loop(self):
        while self.running:
            self.can.send_frame(self.motor_id, self.tx_data)
            time.sleep(self.period)


# =========================================================
# EXAMPLE USAGE
# =========================================================

if __name__ == "__main__":
    motor = MotorevoMotor("COM4", motor_id=7)
    motor.start()
    motor.enter_motor()

    time.sleep(0.5)

    motor.servo(1.0, 2.0, 15.0, 0.8, 5.0, 0.0, 0.001)

    for _ in range(20):
        print(motor.feedback)
        time.sleep(0.1)

    motor.stop()
