"""
PTM Motor Tuning GUI - Interactive Kp/Kd Parameter Tuning

Features:
- Real-time plotting of position, velocity, torque
- Manual parameter adjustment (Kp, Kd, target pos/vel/torque)
- Motor control buttons (Reset, Motor Mode, Zero Position, Send Command)
- Live feedback from CAN bus

Usage:
    python ptm_tuning_gui.py
"""

import tkinter as tk
from tkinter import ttk, messagebox
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import threading
import queue
import time
from collections import deque
import can
import serial
import numpy as np
from utils import float_to_uint, uint_to_float

# ============================================================
# CONFIGURATION - Edit this if your param port is different
# ============================================================
PARAM_PORT = "/dev/ttyACM0"  # USB serial port for param initialization
CAN_INTERFACE = "can0"       # CAN interface name
CAN_BITRATE = 1000000        # 1 Mbps
# ============================================================


class Motor:
    """Motor state container"""
    def __init__(self, motor_id=3):
        self.id = motor_id
        self.Q_MIN, self.Q_MAX = -12.5, 12.5
        self.DQ_MIN, self.DQ_MAX = -10.0, 10.0
        self.TAU_MIN, self.TAU_MAX = -50.0, 50.0
        self.OKP_MIN, self.OKP_MAX = 0.0, 250.0
        self.OKD_MIN, self.OKD_MAX = 0.0, 50.0
        
        self.pos = 0.0
        self.vel = 0.0
        self.torque = 0.0
        self.temperature = 0.0
        self.error_code = 0
        self.status_words = 0


class MotorController:
    """Motor control backend - Native CAN version"""
    def __init__(self, can_bus):
        self.can_bus = can_bus
        self.motors = {}
        self.running = False
        self.rx_thread = None
    
    def add_motor(self, motor):
        self.motors[motor.id] = motor
    
    def start_can_feedback(self):
        if not self.can_bus or self.running:
            return
        self.running = True
        self.rx_thread = threading.Thread(target=self._can_receive_loop, daemon=True)
        self.rx_thread.start()
    
    def stop_can_feedback(self):
        self.running = False
    
    def _can_receive_loop(self):
        while self.running:
            try:
                msg = self.can_bus.recv(timeout=0.01)
                if msg and msg.arbitration_id in self.motors:
                    self._process_can_feedback(msg)
            except Exception:
                pass
    
    def _process_can_feedback(self, msg):
        motor = self.motors.get(msg.arbitration_id)
        if not motor or len(msg.data) < 8:
            return
        
        data = msg.data
        status_word = data[0]
        q_uint = (data[1] << 8) | data[2]
        dq_uint = (data[3] << 4) | (data[4] >> 4)
        tau_uint = ((data[4] & 0x0F) << 8) | data[5]
        error_code = data[6]
        temperature = data[7]
        
        motor.pos = uint_to_float(q_uint, motor.Q_MIN, motor.Q_MAX, 16)
        motor.vel = uint_to_float(dq_uint, motor.DQ_MIN, motor.DQ_MAX, 12)
        motor.torque = uint_to_float(tau_uint, motor.TAU_MIN, motor.TAU_MAX, 12)
        motor.temperature = temperature
        motor.error_code = error_code
        motor.status_words = status_word
    
    def PTM_control(self, motor, pos, vel, kp, kd, torque):
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
        time.sleep(0.001)
    
    def reset_mode(self, motor):
        motor_id = motor.id if hasattr(motor, 'id') else motor
        self.__control_cmd(motor_id, np.uint8(0xFD))
        time.sleep(0.01)
    
    def motor_mode(self, motor):
        self.__control_cmd(motor, np.uint8(0xFC))
        time.sleep(0.01)
    
    def set_zero_position(self, motor):
        data_buff = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE])
        self.__send_data(motor.id, data_buff)
        time.sleep(0.005)
    
    def __control_cmd(self, motor, cmd):
        motor_id = motor.id if hasattr(motor, 'id') else motor
        data_buff = bytes([0xFF] * 7 + [cmd])
        self.__send_data(motor_id, data_buff)
    
    def __send_data(self, motor_id, data):
        """Send native CAN message directly to motor"""
        if not self.can_bus:
            print(f"ERROR: No CAN bus available")
            return
            
        try:
            # Native CAN: arbitration_id = motor ID, data = 8 bytes
            msg = can.Message(
                arbitration_id=motor_id,
                data=data[:8] if len(data) >= 8 else (data + bytes(8 - len(data))),
                is_extended_id=False
            )
            self.can_bus.send(msg)
        except Exception as e:
            print(f"[CAN TX ERROR] Motor {motor_id}: {e}")


class PTMTuningGUI:
    """Main GUI application"""
    def __init__(self, root):
        self.root = root
        self.root.title("PTM Motor Tuning GUI - Kp/Kd Parameter Tuning")
        self.root.geometry("1400x800")
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        
        # Motor and controller
        self.motor = None
        self.ctrl = None
        self.control_thread = None
        self.control_running = False
        self.data_queue = queue.Queue()
        
        # Plot data (last 10 seconds at 200Hz = 2000 points)
        self.max_points = 2000
        self.time_data = deque(maxlen=self.max_points)
        self.pos_actual = deque(maxlen=self.max_points)
        self.pos_target = deque(maxlen=self.max_points)
        self.vel_actual = deque(maxlen=self.max_points)
        self.torque_actual = deque(maxlen=self.max_points)
        self.torque_target = deque(maxlen=self.max_points)
        
        self.start_time = time.time()
        
        # Build GUI
        self.create_widgets()
        self.initialize_hardware()
        
        # Start GUI update loop
        self.update_plots()
    
    def create_widgets(self):
        # Main container
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Left: Plots
        plot_frame = ttk.Frame(main_frame)
        plot_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        
        # Right: Controls
        control_frame = ttk.LabelFrame(main_frame, text="Control Panel", padding=10)
        control_frame.pack(side=tk.RIGHT, fill=tk.BOTH, padx=5)
        
        # Create plots
        self.create_plots(plot_frame)
        
        # Create controls
        self.create_controls(control_frame)
    
    def create_plots(self, parent):
        self.fig = Figure(figsize=(10, 8), dpi=100)
        
        # Position plot
        self.ax_pos = self.fig.add_subplot(311)
        self.line_pos_actual, = self.ax_pos.plot([], [], 'b-', label='Actual', linewidth=2)
        self.line_pos_target, = self.ax_pos.plot([], [], 'r--', label='Target', linewidth=2)
        self.ax_pos.set_ylabel('Position (rad)')
        self.ax_pos.set_title('Position Tracking')
        self.ax_pos.legend(loc='upper right')
        self.ax_pos.grid(True, alpha=0.3)
        
        # Velocity plot
        self.ax_vel = self.fig.add_subplot(312)
        self.line_vel, = self.ax_vel.plot([], [], 'g-', label='Actual', linewidth=2)
        self.ax_vel.set_ylabel('Velocity (rad/s)')
        self.ax_vel.set_title('Velocity Feedback')
        self.ax_vel.legend(loc='upper right')
        self.ax_vel.grid(True, alpha=0.3)
        
        # Torque plot
        self.ax_torque = self.fig.add_subplot(313)
        self.line_torque_actual, = self.ax_torque.plot([], [], 'b-', label='Actual', linewidth=2)
        self.line_torque_target, = self.ax_torque.plot([], [], 'm--', label='Target', linewidth=2)
        self.ax_torque.set_xlabel('Time (s)')
        self.ax_torque.set_ylabel('Torque (Nm)')
        self.ax_torque.set_title('Torque Tracking')
        self.ax_torque.legend(loc='upper right')
        self.ax_torque.grid(True, alpha=0.3)
        
        self.fig.tight_layout()
        
        self.canvas = FigureCanvasTkAgg(self.fig, parent)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
    
    def create_controls(self, parent):
        row = 0
        
        # Motor ID
        ttk.Label(parent, text="Motor ID:", font=('Arial', 10, 'bold')).grid(row=row, column=0, sticky=tk.W, pady=5)
        self.motor_id_var = tk.IntVar(value=3)
        ttk.Entry(parent, textvariable=self.motor_id_var, width=15).grid(row=row, column=1, pady=5)
        row += 1
        
        ttk.Separator(parent, orient=tk.HORIZONTAL).grid(row=row, column=0, columnspan=2, sticky=tk.EW, pady=10)
        row += 1
        
        # Target Parameters
        ttk.Label(parent, text="Target Parameters", font=('Arial', 10, 'bold')).grid(row=row, column=0, columnspan=2, pady=5)
        row += 1
        
        ttk.Label(parent, text="Position (rad):").grid(row=row, column=0, sticky=tk.W)
        self.target_pos_var = tk.DoubleVar(value=0.0)
        ttk.Entry(parent, textvariable=self.target_pos_var, width=15).grid(row=row, column=1)
        row += 1
        
        ttk.Label(parent, text="Velocity (rad/s):").grid(row=row, column=0, sticky=tk.W)
        self.target_vel_var = tk.DoubleVar(value=0.0)
        ttk.Entry(parent, textvariable=self.target_vel_var, width=15).grid(row=row, column=1)
        row += 1
        
        ttk.Label(parent, text="Torque (Nm):").grid(row=row, column=0, sticky=tk.W)
        self.target_torque_var = tk.DoubleVar(value=0.0)
        ttk.Entry(parent, textvariable=self.target_torque_var, width=15).grid(row=row, column=1)
        row += 1
        
        ttk.Separator(parent, orient=tk.HORIZONTAL).grid(row=row, column=0, columnspan=2, sticky=tk.EW, pady=10)
        row += 1
        
        # Gains
        ttk.Label(parent, text="Control Gains", font=('Arial', 10, 'bold')).grid(row=row, column=0, columnspan=2, pady=5)
        row += 1
        
        ttk.Label(parent, text="Kp (0-250):").grid(row=row, column=0, sticky=tk.W)
        self.kp_var = tk.DoubleVar(value=9.0)
        ttk.Entry(parent, textvariable=self.kp_var, width=15).grid(row=row, column=1)
        row += 1
        
        ttk.Label(parent, text="Kd (0-50):").grid(row=row, column=0, sticky=tk.W)
        self.kd_var = tk.DoubleVar(value=1.0)
        ttk.Entry(parent, textvariable=self.kd_var, width=15).grid(row=row, column=1)
        row += 1
        
        ttk.Separator(parent, orient=tk.HORIZONTAL).grid(row=row, column=0, columnspan=2, sticky=tk.EW, pady=10)
        row += 1
        
        # Command Buttons
        ttk.Label(parent, text="Motor Commands", font=('Arial', 10, 'bold')).grid(row=row, column=0, columnspan=2, pady=5)
        row += 1
        
        btn_reset = ttk.Button(parent, text="Reset Motor", command=self.cmd_reset, width=20)
        btn_reset.grid(row=row, column=0, columnspan=2, pady=3)
        row += 1
        
        btn_motor_mode = ttk.Button(parent, text="Motor Mode", command=self.cmd_motor_mode, width=20)
        btn_motor_mode.grid(row=row, column=0, columnspan=2, pady=3)
        row += 1
        
        btn_zero = ttk.Button(parent, text="Zero Position", command=self.cmd_zero_position, width=20)
        btn_zero.grid(row=row, column=0, columnspan=2, pady=3)
        row += 1
        
        btn_send = ttk.Button(parent, text="Send Command", command=self.cmd_send_command, width=20, style='Accent.TButton')
        btn_send.grid(row=row, column=0, columnspan=2, pady=10)
        row += 1
        
        btn_stop = ttk.Button(parent, text="STOP", command=self.cmd_stop, width=20)
        btn_stop.grid(row=row, column=0, columnspan=2, pady=3)
        row += 1
        
        ttk.Separator(parent, orient=tk.HORIZONTAL).grid(row=row, column=0, columnspan=2, sticky=tk.EW, pady=10)
        row += 1
        
        # Status Display
        ttk.Label(parent, text="Motor Status", font=('Arial', 10, 'bold')).grid(row=row, column=0, columnspan=2, pady=5)
        row += 1
        
        self.status_text = tk.Text(parent, height=10, width=30, font=('Courier', 9))
        self.status_text.grid(row=row, column=0, columnspan=2, pady=5)
        row += 1
    
    def initialize_hardware(self):
        try:
            # Create native CAN bus
            can_bus = can.Bus(channel=CAN_INTERFACE, interface='socketcan', bitrate=CAN_BITRATE)
            
            # Create motor and controller (direct CAN, no wrapper)
            motor_id = self.motor_id_var.get()
            self.motor = Motor(motor_id=motor_id)
            self.ctrl = MotorController(can_bus)
            self.ctrl.add_motor(self.motor)
            self.ctrl.start_can_feedback()
            
            self.update_status(f"Hardware initialized\nCAN: {CAN_INTERFACE} @ {CAN_BITRATE} bps\nMotor ID: {motor_id}\nReady")
        except Exception as e:
            messagebox.showerror("Hardware Error", f"Failed to initialize hardware:\n{e}")
            self.update_status(f"ERROR: {e}")
    
    def cmd_reset(self):
        if self.ctrl and self.motor:
            self.ctrl.reset_mode(self.motor)
            self.update_status("Reset command sent")
    
    def cmd_motor_mode(self):
        if self.ctrl and self.motor:
            self.ctrl.motor_mode(self.motor)
            self.update_status("Motor mode enabled")
    
    def cmd_zero_position(self):
        if self.ctrl and self.motor:
            self.ctrl.set_zero_position(self.motor)
            self.update_status("Zero position command sent")
    
    def cmd_send_command(self):
        if self.control_running:
            messagebox.showwarning("Already Running", "Control loop is already active. Click STOP first.")
            return
        
        # Start control loop
        self.control_running = True
        self.start_time = time.time()
        self.control_thread = threading.Thread(target=self.control_loop, daemon=True)
        self.control_thread.start()
        self.update_status("PTM control started")
    
    def cmd_stop(self):
        self.control_running = False
        if self.ctrl and self.motor:
            # Send zero gains
            self.ctrl.PTM_control(self.motor, pos=0, vel=0, kp=0, kd=0, torque=0)
        self.update_status("Control stopped")
    
    def control_loop(self):
        CONTROL_DT = 0.005  # 5ms
        
        while self.control_running:
            loop_start = time.time()
            
            # Get parameters from GUI
            target_pos = self.target_pos_var.get()
            target_vel = self.target_vel_var.get()
            target_torque = self.target_torque_var.get()
            kp = self.kp_var.get()
            kd = self.kd_var.get()
            
            # Send PTM command
            if self.ctrl and self.motor:
                self.ctrl.PTM_control(self.motor, target_pos, target_vel, kp, kd, target_torque)
                
                # Queue data for plotting
                now = time.time() - self.start_time
                self.data_queue.put({
                    'time': now,
                    'pos_actual': self.motor.pos,
                    'pos_target': target_pos,
                    'vel_actual': self.motor.vel,
                    'torque_actual': self.motor.torque,
                    'torque_target': target_torque
                })
            
            # Fixed-rate control
            elapsed = time.time() - loop_start
            if elapsed < CONTROL_DT:
                time.sleep(CONTROL_DT - elapsed)
    
    def update_plots(self):
        # Process queued data
        while not self.data_queue.empty():
            try:
                data = self.data_queue.get_nowait()
                self.time_data.append(data['time'])
                self.pos_actual.append(data['pos_actual'])
                self.pos_target.append(data['pos_target'])
                self.vel_actual.append(data['vel_actual'])
                self.torque_actual.append(data['torque_actual'])
                self.torque_target.append(data['torque_target'])
            except queue.Empty:
                break
        
        # Update plots
        if len(self.time_data) > 0:
            self.line_pos_actual.set_data(list(self.time_data), list(self.pos_actual))
            self.line_pos_target.set_data(list(self.time_data), list(self.pos_target))
            self.line_vel.set_data(list(self.time_data), list(self.vel_actual))
            self.line_torque_actual.set_data(list(self.time_data), list(self.torque_actual))
            self.line_torque_target.set_data(list(self.time_data), list(self.torque_target))
            
            # Autoscale
            for ax in [self.ax_pos, self.ax_vel, self.ax_torque]:
                ax.relim()
                ax.autoscale_view()
            
            self.canvas.draw_idle()
        
        # Update status text
        if self.motor:
            status = f"Pos: {self.motor.pos:6.2f} rad\n"
            status += f"Vel: {self.motor.vel:6.2f} rad/s\n"
            status += f"Torque: {self.motor.torque:6.2f} Nm\n"
            status += f"Temp: {self.motor.temperature:4.1f}°C\n"
            status += f"Error: 0x{self.motor.error_code:02X}"
            self.status_text.delete('1.0', tk.END)
            self.status_text.insert('1.0', status)
        
        # Schedule next update (50ms = 20 Hz)
        self.root.after(50, self.update_plots)
    
    def update_status(self, message):
        print(f"[GUI] {message}")
    
    def on_close(self):
        self.control_running = False
        if self.ctrl:
            self.ctrl.stop_can_feedback()
        self.root.destroy()


def main():
    root = tk.Tk()
    app = PTMTuningGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
