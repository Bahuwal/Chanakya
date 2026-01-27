#!/usr/bin/env python3
"""
Biped Controller with GUI Monitor
Real-time visualization of all robot states and debug information
"""

import sys
import time
import signal
import onnxruntime as ort
import numpy as np
from omegaconf import OmegaConf
import os
import copy
import tkinter as tk
from tkinter import ttk
import threading

from operator import itemgetter
from sensor_controller import SensorController
from can_motor_controller import CANMotorController
from publisher import DataPublisher, DataReceiver
from numpy_ringbuffer import RingArrayBuffer

# Import the existing Model and BipedController
# We'll create a GUI wrapper

class RobotGUI:
    """GUI Monitor for Biped Controller"""
    
    def __init__(self, controller):
        self.controller = controller
        self.root = tk.Tk()
        self.root.title("🤖 Biped Robot Monitor")
        self.root.geometry("1200x800")
        self.root.configure(bg='#1e1e1e')
        
        # Create main container
        main_frame = tk.Frame(self.root, bg='#1e1e1e')
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # === TOP ROW: Status and Commands ===
        top_row = tk.Frame(main_frame, bg='#1e1e1e')
        top_row.pack(fill=tk.X, pady=(0, 10))
        
        # System Status Panel
        self._create_status_panel(top_row)
        
        # Commands Panel
        self._create_commands_panel(top_row)
        
        # === MIDDLE ROW: Sensors ===
        middle_row = tk.Frame(main_frame, bg='#1e1e1e')
        middle_row.pack(fill=tk.X, pady=(0, 10))
        
        # IMU Panel
        self._create_imu_panel(middle_row)
        
        # Load Cells Panel
        self._create_load_cells_panel(middle_row)
        
        # === BOTTOM ROW: Motors ===
        bottom_row = tk.Frame(main_frame, bg='#1e1e1e')
        bottom_row.pack(fill=tk.BOTH, expand=True)
        
        # Motors Panel
        self._create_motors_panel(bottom_row)
        
        # Phase/Contact Panel
        self._create_phase_panel(bottom_row)
        
        # Update loop
        self.running = True
        self.update_gui()
        
    def _create_panel(self, parent, title):
        """Helper to create a styled panel"""
        frame = tk.LabelFrame(parent, text=title, bg='#2d2d2d', fg='#00ff00',
                             font=('Courier', 12, 'bold'), bd=2, relief=tk.RIDGE)
        return frame
        
    def _create_status_panel(self, parent):
        """System status panel"""
        panel = self._create_panel(parent, "🟢 SYSTEM STATUS")
        panel.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 5))
        
        self.status_labels = {}
        status_items = [
            ("Motor Control", "motors_ok"),
            ("IMU Data", "imu_ok"),
            ("Load Cells", "loadcells_ok"),
            ("Loop Rate", "loop_rate"),
            ("Decimation", "decimation")
        ]
        
        for i, (name, key) in enumerate(status_items):
            row = tk.Frame(panel, bg='#2d2d2d')
            row.pack(fill=tk.X, padx=5, pady=2)
            
            tk.Label(row, text=f"{name}:", bg='#2d2d2d', fg='#aaaaaa',
                    font=('Courier', 10), width=15, anchor='w').pack(side=tk.LEFT)
            
            label = tk.Label(row, text="---", bg='#2d2d2d', fg='#00ff00',
                           font=('Courier', 10, 'bold'))
            label.pack(side=tk.LEFT)
            self.status_labels[key] = label
            
    def _create_commands_panel(self, parent):
        """Velocity commands panel"""
        panel = self._create_panel(parent, "🎮 COMMANDS")
        panel.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(5, 0))
        
        self.cmd_labels = {}
        commands = [
            ("Forward (m/s)", "vel_x", "#00ff00"),
            ("Lateral (m/s)", "vel_y", "#00aaff"),
            ("Rotation (rad/s)", "yaw", "#ffaa00")
        ]
        
        for name, key, color in commands:
            row = tk.Frame(panel, bg='#2d2d2d')
            row.pack(fill=tk.X, padx=5, pady=3)
            
            tk.Label(row, text=f"{name}:", bg='#2d2d2d', fg='#aaaaaa',
                    font=('Courier', 10), width=18, anchor='w').pack(side=tk.LEFT)
            
            label = tk.Label(row, text="0.00", bg='#2d2d2d', fg=color,
                           font=('Courier', 12, 'bold'))
            label.pack(side=tk.LEFT)
            self.cmd_labels[key] = label
            
    def _create_imu_panel(self, parent):
        """IMU sensor panel"""
        panel = self._create_panel(parent, "📊 IMU SENSOR")
        panel.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 5))
        
        self.imu_labels = {}
        imu_data = [
            ("ωx (rad/s)", "wx"),
            ("ωy (rad/s)", "wy"),
            ("ωz (rad/s)", "wz"),
            ("Gravity Z", "gz")
        ]
        
        for name, key in imu_data:
            row = tk.Frame(panel, bg='#2d2d2d')
            row.pack(fill=tk.X, padx=5, pady=2)
            
            tk.Label(row, text=f"{name}:", bg='#2d2d2d', fg='#aaaaaa',
                    font=('Courier', 9), width=12, anchor='w').pack(side=tk.LEFT)
            
            label = tk.Label(row, text="0.00", bg='#2d2d2d', fg='#00ff00',
                           font=('Courier', 10))
            label.pack(side=tk.LEFT)
            self.imu_labels[key] = label
            
    def _create_load_cells_panel(self, parent):
        """Load cells/contact panel"""
        panel = self._create_panel(parent, "🦶 LOAD CELLS")
        panel.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(5, 0))
        
        self.force_labels = {}
        
        # Left foot
        left_frame = tk.Frame(panel, bg='#2d2d2d')
        left_frame.pack(fill=tk.X, padx=5, pady=2)
        tk.Label(left_frame, text="LEFT:", bg='#2d2d2d', fg='#ffaa00',
                font=('Courier', 9, 'bold')).pack(side=tk.LEFT)
        self.force_labels['left'] = tk.Label(left_frame, text="0 N", bg='#2d2d2d',
                                             fg='#00ff00', font=('Courier', 10))
        self.force_labels['left'].pack(side=tk.LEFT, padx=5)
        self.force_labels['left_contact'] = tk.Label(left_frame, text="○", bg='#2d2d2d',
                                                     fg='#666666', font=('Courier', 16, 'bold'))
        self.force_labels['left_contact'].pack(side=tk.LEFT)
        
        # Right foot
        right_frame = tk.Frame(panel, bg='#2d2d2d')
        right_frame.pack(fill=tk.X, padx=5, pady=2)
        tk.Label(right_frame, text="RIGHT:", bg='#2d2d2d', fg='#ffaa00',
                font=('Courier', 9, 'bold')).pack(side=tk.LEFT)
        self.force_labels['right'] = tk.Label(right_frame, text="0 N", bg='#2d2d2d',
                                              fg='#00ff00', font=('Courier', 10))
        self.force_labels['right'].pack(side=tk.LEFT, padx=5)
        self.force_labels['right_contact'] = tk.Label(right_frame, text="○", bg='#2d2d2d',
                                                      fg='#666666', font=('Courier', 16, 'bold'))
        self.force_labels['right_contact'].pack(side=tk.LEFT)
        
    def _create_motors_panel(self, parent):
        """Motor positions panel"""
        panel = self._create_panel(parent, "⚙️  MOTOR POSITIONS (rad)")
        panel.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 5))
        
        # Create table
        table_frame = tk.Frame(panel, bg='#2d2d2d')
        table_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Headers
        headers = ["ID", "Position", "Target", "Error"]
        for col, header in enumerate(headers):
            tk.Label(table_frame, text=header, bg='#1a1a1a', fg='#ffaa00',
                    font=('Courier', 9, 'bold'), width=10).grid(row=0, column=col, sticky='ew')
        
        self.motor_labels = {}
        for i in range(10):
            # Motor ID
            tk.Label(table_frame, text=f"M{i+1}", bg='#2d2d2d', fg='#aaaaaa',
                    font=('Courier', 9), width=10).grid(row=i+1, column=0)
            
            # Position
            pos_label = tk.Label(table_frame, text="0.00", bg='#2d2d2d', fg='#00ff00',
                                font=('Courier', 9), width=10)
            pos_label.grid(row=i+1, column=1)
            
            # Target  
            target_label = tk.Label(table_frame, text="0.00", bg='#2d2d2d', fg='#00aaff',
                                   font=('Courier', 9), width=10)
            target_label.grid(row=i+1, column=2)
            
            # Error
            error_label = tk.Label(table_frame, text="0.00", bg='#2d2d2d', fg='#ff6666',
                                  font=('Courier', 9), width=10)
            error_label.grid(row=i+1, column=3)
            
            self.motor_labels[i] = {
                'pos': pos_label,
                'target': target_label,
                'error': error_label
            }
            
    def _create_phase_panel(self, parent):
        """Phase and gait info panel"""
        panel = self._create_panel(parent, "👣 GAIT PHASE")
        panel.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(5, 0))
        
        self.phase_labels = {}
        
        # Phase values
        for foot in ['Left', 'Right']:
            row = tk.Frame(panel, bg='#2d2d2d')
            row.pack(fill=tk.X, padx=5, pady=3)
            
            tk.Label(row, text=f"{foot} Phase:", bg='#2d2d2d', fg='#aaaaaa',
                    font=('Courier', 10), width=12, anchor='w').pack(side=tk.LEFT)
            
            label = tk.Label(row, text="0.00", bg='#2d2d2d', fg='#00ff00',
                           font=('Courier', 10, 'bold'))
            label.pack(side=tk.LEFT)
            self.phase_labels[foot.lower()] = label
        
        # Error log
        log_frame = tk.Frame(panel, bg='#2d2d2d')
        log_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        tk.Label(log_frame, text="Error Log:", bg='#2d2d2d', fg='#ff6666',
                font=('Courier', 9, 'bold')).pack(anchor='w')
        
        self.error_text = tk.Text(log_frame, height=8, bg='#1a1a1a', fg='#ff6666',
                                 font=('Courier', 8), state='disabled')
        self.error_text.pack(fill=tk.BOTH, expand=True)
        
    def log_error(self, message):
        """Add error message to log"""
        self.error_text.config(state='normal')
        self.error_text.insert('1.0', f"{time.strftime('%H:%M:%S')} {message}\n")
        self.error_text.config(state='disabled')
       
    def update_gui(self):
        """Update all GUI elements from controller state"""
        if not self.running:
            return
            
        try:
            c = self.controller
            
            # System Status
            self.status_labels['motors_ok'].config(text="✓ OK", fg='#00ff00')
            
            if hasattr(c, 'sensor_data') and c.sensor_data:
                imu_ok = c.sensor_data.get("ang_vel") is not None
                force_ok = c.sensor_data.get("force_measurement") is not None
                
                self.status_labels['imu_ok'].config(
                    text="✓ OK" if imu_ok else "✗ FAIL",
                    fg='#00ff00' if imu_ok else '#ff0000'
                )
                self.status_labels['loadcells_ok'].config(
                    text="✓ OK" if force_ok else "✗ FAIL",
                    fg='#00ff00' if force_ok else '#ff0000'
                )
                
                # IMU Data
                if imu_ok:
                    ang_vel = c.sensor_data["ang_vel"]
                    gravity = c.sensor_data["gravity_vec"]
                    self.imu_labels['wx'].config(text=f"{ang_vel[0]:6.2f}")
                    self.imu_labels['wy'].config(text=f"{ang_vel[1]:6.2f}")
                    self.imu_labels['wz'].config(text=f"{ang_vel[2]:6.2f}")
                    self.imu_labels['gz'].config(text=f"{gravity[2]:6.2f}")
                
                # Load Cells
                if force_ok:
                    force = c.sensor_data["force_measurement"]
                    contact = c.sensor_data.get("contact", [False, False])
                    
                    left_force = force[0] + force[1]
                    right_force = force[2] + force[3]
                    
                    self.force_labels['left'].config(text=f"{left_force:4.0f} N")
                    self.force_labels['right'].config(text=f"{right_force:4.0f} N")
                    
                    self.force_labels['left_contact'].config(
                        text="●" if contact[0] else "○",
                        fg='#00ff00' if contact[0] else '#666666'
                    )
                    self.force_labels['right_contact'].config(
                        text="●" if contact[1] else "○",
                        fg='#00ff00' if contact[1] else '#666666'
                    )
            
            # Commands
            if hasattr(c, 'commands'):
                self.cmd_labels['vel_x'].config(text=f"{c.commands[0]:+.2f}")
                self.cmd_labels['vel_y'].config(text=f"{c.commands[1]:+.2f}")
                self.cmd_labels['yaw'].config(text=f"{c.commands[2]:+.2f}")
            
            # Loop Rate
            if hasattr(c, 'decimation_actual'):
                self.status_labels['decimation'].config(text=f"{c.decimation_actual}")
                loop_hz = 1000.0 / c.decimation_actual if c.decimation_actual > 0 else 0
                self.status_labels['loop_rate'].config(text=f"{loop_hz:.1f} Hz")
            
            # Motors
            if hasattr(c, 'dof_pos') and hasattr(c, 'dof_pos_target'):
                for i in range(10):
                    pos = c.dof_pos[i]
                    target = c.dof_pos_target[i]
                    error = target - pos
                    
                    self.motor_labels[i]['pos'].config(text=f"{pos:+.3f}")
                    self.motor_labels[i]['target'].config(text=f"{target:+.3f}")
                    self.motor_labels[i]['error'].config(text=f"{error:+.3f}")
            
            # Phase
            if hasattr(c, 'phase'):
                self.phase_labels['left'].config(text=f"{c.phase[0]:.3f}")
                self.phase_labels['right'].config(text=f"{c.phase[1]:.3f}")
                
        except Exception as e:
            self.log_error(f"GUI Update Error: {str(e)}")
        
        # Schedule next update
        self.root.after(50, self.update_gui)  # 20 Hz update rate
        
    def run(self):
        """Start the GUI main loop"""
        self.root.mainloop()
        
    def shutdown(self):
        """Stop the GUI"""
        self.running = False
        self.root.quit()


# Import BipedController properly
import importlib.util
spec = importlib.util.spec_from_file_location("biped_controller", "biped_controller_can.py")
biped_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(biped_module)
BipedController = biped_module.BipedController

if __name__ == "__main__":
    # Check if display is available (for Linux)
    import os
    if 'DISPLAY' not in os.environ:
        print("❌ ERROR: No display available!")
        print("   If running via SSH, use: ssh -X user@host")
        print("   Or run the terminal version: python3 biped_controller_can.py")
        sys.exit(1)
    
    print("🚀 Starting Biped Controller with GUI...")
    
    # Create controller
    controller = BipedController()
    
    # Create GUI in main thread
    gui = RobotGUI(controller)
    
    # Run controller in separate thread
    def control_loop():
        controller.initialize_stance()
        np.set_printoptions(formatter={"float": "{: 3.2f}".format})
        while gui.running:
            try:
                controller.step()
            except Exception as e:
                gui.log_error(f"Control Error: {str(e)}")
                break
    
    control_thread = threading.Thread(target=control_loop, daemon=True)
    control_thread.start()
    
    # Run GUI (blocks)
    try:
        gui.run()
    except KeyboardInterrupt:
        print("\n👋 Shutting down...")
    finally:
        gui.shutdown()
        controller.shutdown()
        print("✅ Shutdown complete")
