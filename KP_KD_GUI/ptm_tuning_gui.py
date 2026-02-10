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
        self.last_tx_data = {}  # Track last sent data per motor ID to filter echo

        self.tx_buffer = bytearray([
            0xAA, 0x01,
            0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC,
            0x00, 0x00, 0x00, 0x00,
            0xF4
        ])

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
        last_rx_time = {}  # Track timing to distinguish echo from feedback
        
        while self.running:
            try:
                msg = self.can_bus.recv(timeout=0.01)
                if msg:
                    arb_id = msg.arbitration_id
                    now = time.time()
                    
                    # Check if this is a registered motor
                    if arb_id in self.motors:
                        # Calculate time since last RX on this ID
                        time_since_last = (now - last_rx_time.get(arb_id, 0)) * 1000  # ms
                        last_rx_time[arb_id] = now
                        
                        # Echo: arrives <10ms after TX (hardware loopback)
                        # Real feedback: arrives >100ms after TX (motor processing time)
                        if time_since_last < 10:
                            # Too fast - this is echo/loopback
                            print(f"[CAN ECHO] ID: 0x{arb_id:03X} | Data: {msg.data.hex()} (Δt={time_since_last:.1f}ms)")
                        else:
                            # Real motor feedback!
                            print(f"[CAN RX] ID: 0x{arb_id:03X} | Data: {msg.data.hex()} (Δt={time_since_last:.1f}ms)")
                            self._process_can_feedback(msg)
                    else:
                        print(f"[CAN RX] Unknown ID: 0x{arb_id:03X} | Data: {msg.data.hex()}")
                        
            except Exception as e:
                if "Receive timeout" not in str(e):
                    print(f"[CAN RX ERROR] {e}")
                pass
    
    def _process_can_feedback(self, msg):
        """Process CAN feedback message from motor"""
        motor = self.motors.get(msg.arbitration_id)
        if not motor or len(msg.data) < 8:
            return
        
        data = msg.data
        
        # Parse feedback according to OFFICIAL Motorevo code (Revo_CAN.py lines 206-222)
        # This is the CORRECT format verified from working code:
        status_word = data[0]
        q_uint = (data[1] << 8) | data[2]
        dq_uint = (data[3] << 4) | (data[4] >> 4)
        tau_uint = ((data[4] & 0x0F) << 8) | data[5]
        error_code = data[6]      # ← FIXED! Error is byte 6
        temperature = data[7]     # ← FIXED! Temp is byte 7
        
        # Convert to floats
        recv_q = uint_to_float(q_uint, motor.Q_MIN, motor.Q_MAX, 16)
        recv_dq = uint_to_float(dq_uint, motor.DQ_MIN, motor.DQ_MAX, 12)
        recv_tau = uint_to_float(tau_uint, motor.TAU_MIN, motor.TAU_MAX, 12)
        
        # Update motor state
        motor.get_data(status_word, recv_q, recv_dq, recv_tau, temperature, error_code)
        
        print(f"  └─ FEEDBACK: Pos={recv_q:6.3f} rad | Vel={recv_dq:6.3f} rad/s | Tau={recv_tau:6.2f} Nm | Temp={temperature}°C | Err=0x{error_code:02X}")

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
        PTM (Position-Torque Mix) mode control:
        
        Control Formula: T_total = Kp×(P_ref - P_actual) + Kd×(V_ref - V_actual) + T_ref
        
        Args:
            motor: Motor object
            pos: Target position (radians), -12.5 to +12.5
            vel: Target velocity (rad/s), -10.0 to +10.0
            kp: Position proportional gain, 0.0 to 250.0
            kd: Position derivative gain, 0.0 to 50.0
            torque: Feed-forward torque (Nm), -50.0 to +50.0
            
        Byte Structure:
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
        sleep(0.001)  # Fixed 1ms delay for command processing

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
        
        # Track last sent data to filter echo
        self.last_tx_data[motor_id] = bytes(self.tx_buffer[2:10])
        
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



# Helper: USB config loader
def load_usb_config(path="usb.json"):
    default = {
        "param_port": "/dev/ttyACM3",
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


# ================================================================
# GUI VERSION - tkinter interface with parameter controls
# ================================================================
if __name__ == "__main__":
    import tkinter as tk
    from tkinter import ttk
    import matplotlib
    matplotlib.use('TkAgg')
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
    from matplotlib.figure import Figure
    from collections import deque
    import serial
    import time
    
    class PTMGUI:
        def __init__(self, root):
            self.root = root
            self.root.title("PTM Motor Tuning GUI")
            self.root.geometry("1400x800")
            
            # Hardware
            self.ctrl = None
            self.motor = None
            self.motor_serial = None
            self.param_serial = None
            self.motor_can_bus = None
            self.control_running = False
            self.start_time = time.time()
            
            # Plot data
            self.time_data = deque(maxlen=2000)
            self.pos_data = deque(maxlen=2000)
            self.target_pos_data = deque(maxlen=2000)
            self.vel_data = deque(maxlen=2000)
            self.torque_data = deque(maxlen=2000)
            self.target_torque_data = deque(maxlen=2000)
            
            self.create_widgets()
            self.initialize_hardware()
            self.update_gui()
        
        def create_widgets(self):
            main = ttk.Frame(self.root)
            main.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
            
            # Left: plots
            plot_frame = ttk.Frame(main)
            plot_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
            
            # Right: controls
            ctrl_frame = ttk.LabelFrame(main, text="Controls", padding=10)
            ctrl_frame.pack(side=tk.RIGHT, fill=tk.Y, padx=5)
            
            # Plots
            self.fig = Figure(figsize=(10, 8))
            self.ax_pos = self.fig.add_subplot(311)
            self.line_pos, = self.ax_pos.plot([], [], 'b-', label='Actual', linewidth=2)
            self.line_target_pos, = self.ax_pos.plot([], [], 'r--', label='Target', linewidth=2)
            self.ax_pos.set_ylabel('Position (rad)')
            self.ax_pos.set_title('Position')
            self.ax_pos.legend()
            self.ax_pos.grid(True, alpha=0.3)
            
            self.ax_vel = self.fig.add_subplot(312)
            self.line_vel, = self.ax_vel.plot([], [], 'g-', linewidth=2)
            self.ax_vel.set_ylabel('Velocity (rad/s)')
            self.ax_vel.set_title('Velocity')
            self.ax_vel.grid(True, alpha=0.3)
            
            self.ax_torque = self.fig.add_subplot(313)
            self.line_torque, = self.ax_torque.plot([], [], 'b-', label='Actual', linewidth=2)
            self.line_target_torque, = self.ax_torque.plot([], [], 'm--', label='Target', linewidth=2)
            self.ax_torque.set_xlabel('Time (s)')
            self.ax_torque.set_ylabel('Torque (Nm)')
            self.ax_torque.set_title('Torque')
            self.ax_torque.legend()
            self.ax_torque.grid(True, alpha=0.3)
            
            self.fig.tight_layout()
            self.canvas = FigureCanvasTkAgg(self.fig, plot_frame)
            self.canvas.draw()
            self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
            
            # Controls
            row = 0
            ttk.Label(ctrl_frame, text="Motor ID:", font=('Arial', 10, 'bold')).grid(row=row, column=0, sticky=tk.W, pady=5)
            self.motor_id_var = tk.IntVar(value=3)
            ttk.Entry(ctrl_frame, textvariable=self.motor_id_var, width=15).grid(row=row, column=1)
            row += 1
            
            ttk.Separator(ctrl_frame, orient=tk.HORIZONTAL).grid(row=row, column=0, columnspan=2, sticky=tk.EW, pady=10)
            row += 1
            
            ttk.Label(ctrl_frame, text="Targets", font=('Arial', 10, 'bold')).grid(row=row, column=0, columnspan=2)
            row += 1
            
            ttk.Label(ctrl_frame, text="Position (rad):").grid(row=row, column=0, sticky=tk.W)
            self.target_pos_var = tk.DoubleVar(value=0.0)
            ttk.Entry(ctrl_frame, textvariable=self.target_pos_var, width=15).grid(row=row, column=1)
            row += 1
            
            ttk.Label(ctrl_frame, text="Velocity (rad/s):").grid(row=row, column=0, sticky=tk.W)
            self.target_vel_var = tk.DoubleVar(value=0.0)
            ttk.Entry(ctrl_frame, textvariable=self.target_vel_var, width=15).grid(row=row, column=1)
            row += 1
            
            ttk.Label(ctrl_frame, text="Torque (Nm):").grid(row=row, column=0, sticky=tk.W)
            self.target_torque_var = tk.DoubleVar(value=0.0)
            ttk.Entry(ctrl_frame, textvariable=self.target_torque_var, width=15).grid(row=row, column=1)
            row += 1
            
            ttk.Separator(ctrl_frame, orient=tk.HORIZONTAL).grid(row=row, column=0, columnspan=2, sticky=tk.EW, pady=10)
            row += 1
            
            ttk.Label(ctrl_frame, text="Gains", font=('Arial', 10, 'bold')).grid(row=row, column=0, columnspan=2)
            row += 1
            
            ttk.Label(ctrl_frame, text="Kp (0-250):").grid(row=row, column=0, sticky=tk.W)
            self.kp_var = tk.DoubleVar(value=9.0)
            ttk.Entry(ctrl_frame, textvariable=self.kp_var, width=15).grid(row=row, column=1)
            row += 1
            
            ttk.Label(ctrl_frame, text="Kd (0-50):").grid(row=row, column=0, sticky=tk.W)
            self.kd_var = tk.DoubleVar(value=1.0)
            ttk.Entry(ctrl_frame, textvariable=self.kd_var, width=15).grid(row=row, column=1)
            row += 1
            
            ttk.Separator(ctrl_frame, orient=tk.HORIZONTAL).grid(row=row, column=0, columnspan=2, sticky=tk.EW, pady=10)
            row += 1
            
            ttk.Button(ctrl_frame, text="Reset Motor", command=self.cmd_reset, width=20).grid(row=row, column=0, columnspan=2, pady=3)
            row += 1
            ttk.Button(ctrl_frame, text="Motor Mode", command=self.cmd_motor_mode, width=20).grid(row=row, column=0, columnspan=2, pady=3)
            row += 1
            ttk.Button(ctrl_frame, text="Zero Position", command=self.cmd_zero, width=20).grid(row=row, column=0, columnspan=2, pady= 3)
            row += 1
            ttk.Button(ctrl_frame, text="START Control", command=self.cmd_start, width=20).grid(row=row, column=0, columnspan=2, pady=10)
            row += 1
            ttk.Button(ctrl_frame, text="STOP", command=self.cmd_stop, width=20).grid(row=row, column=0, columnspan=2, pady=3)
            row += 1
            
            ttk.Separator(ctrl_frame, orient=tk.HORIZONTAL).grid(row=row, column=0, columnspan=2, sticky=tk.EW, pady=10)
            row += 1
            
            ttk.Label(ctrl_frame, text="Status", font=('Arial', 10, 'bold')).grid(row=row, column=0, columnspan=2)
            row += 1
            self.status_text = tk.Text(ctrl_frame, height=10, width=30, font=('Courier', 9))
            self.status_text.grid(row=row, column=0, columnspan=2)
        
        def initialize_hardware(self):
            try:
                motor_id = self.motor_id_var.get()
                
                # Create CAN bus for motor commands
                self.motor_can_bus = can.Bus(channel='can0', interface='socketcan', bitrate=1000000)
                self.motor_serial = CANMotorSender(self.motor_can_bus)
                print(f"✓ CAN opened for motor commands")
                
                # Initialize motor controller
                self.motor = Motor("revo_motor", motor_id=motor_id, type_name="REVO")
                self.ctrl = MotorController(self.motor_serial, can_bus=self.motor_can_bus)
                self.ctrl.add_motor(self.motor)
                self.ctrl.start_can_feedback()
                
                print(f"✓ Hardware ready: Motor ID={motor_id}")
            except Exception as e:
                print(f"✗ Hardware init failed: {e}")
        
        def cmd_reset(self):
            if self.ctrl and self.motor:
                self.ctrl.reset_mode(self.motor)
                print("[CMD] Reset")
        
        def cmd_motor_mode(self):
            if self.ctrl and self.motor:
                self.ctrl.motor_mode(self.motor)
                print("[CMD] Motor mode")
        
        def cmd_zero(self):
            if self.ctrl and self.motor:
                self.ctrl.set_zero_position(self.motor)
                print("[CMD] Zero position")
        
        def cmd_start(self):
            self.control_running = True
            self.start_time = time.time()
            print("[CMD] Control started")
        
        def cmd_stop(self):
            self.control_running = False
            if self.ctrl and self.motor:
                self.ctrl.PTM_control(self.motor, pos=0, vel=0, kp=0, kd=0, torque=0)
            print("[CMD] Control stopped")
        
        def control_step(self):
            if not self.control_running or not self.ctrl or not self.motor:
                return
            
            # Send PTM command at 200Hz
            self.ctrl.PTM_control(
                self.motor,
                pos=self.target_pos_var.get(),
                vel=self.target_vel_var.get(),
                kp=self.kp_var.get(),
                kd=self.kd_var.get(),
                torque=self.target_torque_var.get()
            )
            
            # Update plot data
            now = time.time() - self.start_time
            self.time_data.append(now)
            self.pos_data.append(self.motor.pos)
            self.target_pos_data.append(self.target_pos_var.get())
            self.vel_data.append(self.motor.vel)
            self.torque_data.append(self.motor.torque)
            self.target_torque_data.append(self.target_torque_var.get())
        
        def update_gui(self):
            # Control step
            self.control_step()
            
            # Update plots
            if len(self.time_data) > 0:
                self.line_pos.set_data(list(self.time_data), list(self.pos_data))
                self.line_target_pos.set_data(list(self.time_data), list(self.target_pos_data))
                self.line_vel.set_data(list(self.time_data), list(self.vel_data))
                self.line_torque.set_data(list(self.time_data), list(self.torque_data))
                self.line_target_torque.set_data(list(self.time_data), list(self.target_torque_data))
                
                for ax in [self.ax_pos, self.ax_vel, self.ax_torque]:
                    ax.relim()
                    ax.autoscale_view()
                
                self.canvas.draw_idle()
            
            # Update status
            if self.motor:
                status = f"Pos: {self.motor.pos:6.2f} rad\nVel: {self.motor.vel:6.2f} rad/s\nTorque: {self.motor.torque:6.2f} Nm\nTemp: {self.motor.temperature:4.1f}°C\nError: 0x{self.motor.error_code:02X}"
                self.status_text.delete('1.0', tk.END)
                self.status_text.insert('1.0', status)
            
            # Schedule next update (5ms = 200 Hz)
            self.root.after(5, self.update_gui)
        
        def on_close(self):
            self.control_running = False
            if self.ctrl:
                self.ctrl.stop_can_feedback()
            if self.motor_can_bus:
                self.motor_can_bus.shutdown()
            self.root.destroy()
    
    # Run GUI
    root = tk.Tk()
    gui = PTMGUI(root)
    root.protocol("WM_DELETE_WINDOW", gui.on_close)
    root.mainloop()

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
    revo = Motor("revo_motor", motor_id=3, type_name="REVO")

    # Command controller (sends via CAN, reads feedback from CAN)
    ctrl = MotorController(motor_serial, can_bus=motor_can_bus)
    ctrl.add_motor(revo)
    ctrl.start_can_feedback()  # Start reading CAN feedback
    
    # Enhanced reset sequence (matches working serial code)
    print("Resetting motor...")
    ctrl.reset_mode(revo)
    sleep(0.5)  # Longer wait for full reset
    
    # CRITICAL: Force encoder zero at startup to prevent position persistence
    print("Clearing encoder position...")
    ctrl.set_zero_position(revo)
    sleep(0.5)
    
    print("Entering motor mode...")
    ctrl.motor_mode(revo)
    sleep(0.2)  # Let mode settle
       
    # Wait for position to actually zero
    '''print("Waiting for encoder to zero...", end='', flush=True)
    timeout = time.time() + 2.0
    zeroed = False
    while time.time() < timeout:
        if abs(revo.pos) < 0.5:  # Position near zero
            print(f" ✓ Zeroed at {revo.pos:.3f} rad")
            zeroed = True
            break
        sleep(0.1)
    
    if not zeroed:
        print(f" ⚠️ Warning: Position is {revo.pos:.3f} rad (expected ~0)")'''


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
    target_pos = 10.0       # Target position in radians
    target_vel = 0.0       # Target velocity (usually 0 for position control)
    target_torque = 0.0 #2.0    # Feed-forward torque in Nm (gravity compensation)
    kp_gain = 9.0         # Position control stiffness
    kd_gain = 1.0 #0.8          # Position damping
    
    CONTROL_DURATION = 4
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

    CONTROL_DT = 0.005  # 5ms = 200 Hz (matches bipedCANControllerNative)

    try:
        while time.time() - start < CONTROL_DURATION:
            loop_start = time.time()  # Track loop start for fixed-rate control
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
            if abs(revo.pos - target_pos) < 0.10:
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

            # Fixed-rate control: sleep to maintain consistent 5ms loop
            elapsed = time.time() - loop_start
            if elapsed < CONTROL_DT:
                sleep(CONTROL_DT - elapsed)

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
