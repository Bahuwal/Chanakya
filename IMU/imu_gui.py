#!/usr/bin/env python3
"""
Simple IMU Data Visualization GUI
Reads binary data from Teensy and displays it in a clean GUI
"""

import serial
import struct
import tkinter as tk
from tkinter import ttk, messagebox
import threading
import time
try:
    from cobs import cobs
except ImportError:
    print("COBS library not found. Installing...")
    import subprocess
    subprocess.check_call(['pip3', 'install', 'cobs'])
    from cobs import cobs

class IMUDataGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("IMU Data Monitor")
        self.root.geometry("600x700")
        
        # Serial connection
        self.serial_port = None
        self.running = False
        self.frozen = False  # Freeze display updates
        
        # Data storage
        self.imu_data = {
            'accel_raw': [0.0, 0.0, 0.0],
            'gyro_raw': [0.0, 0.0, 0.0],
            'quaternion': [0.0, 0.0, 0.0, 0.0],
            'gravity': [0.0, 0.0, 0.0],
            'accel_filtered': [0.0, 0.0, 0.0],
            'gyro_filtered': [0.0, 0.0, 0.0],
            'load_cells': [0, 0, 0, 0],
            'filter_status': 0,
            'dynamics_mode': 0,
            'status_flags': 0,
        }
        
        self.setup_gui()
        
    def setup_gui(self):
        # Connection Frame
        conn_frame = ttk.LabelFrame(self.root, text="Connection", padding=10)
        conn_frame.pack(fill="x", padx=10, pady=5)
        
        ttk.Label(conn_frame, text="Port:").grid(row=0, column=0, sticky="w")
        self.port_entry = ttk.Entry(conn_frame, width=20)
        self.port_entry.insert(0, "/dev/ttyACM0")  # Default for Linux/Mac
        # Windows users should use: COM3 or similar
        self.port_entry.grid(row=0, column=1, padx=5)
        
        self.connect_btn = ttk.Button(conn_frame, text="Connect", command=self.toggle_connection)
        self.connect_btn.grid(row=0, column=2, padx=5)
        
        self.status_label = ttk.Label(conn_frame, text="● Disconnected", foreground="red")
        self.status_label.grid(row=0, column=3, padx=10)
        
        self.freeze_btn = ttk.Button(conn_frame, text="⏸ Freeze", command=self.toggle_freeze, state="disabled")
        self.freeze_btn.grid(row=0, column=4, padx=5)
        
        # Create data display sections
        self.create_section("Raw Accelerometer (m/s²)", ['X', 'Y', 'Z'], 'accel_raw')
        self.create_section("Raw Gyroscope (rad/s)", ['X', 'Y', 'Z'], 'gyro_raw')
        self.create_section("Quaternion (x, y, z, w)", ['X', 'Y', 'Z', 'W'], 'quaternion')
        self.create_section("Gravity Vector (g)", ['X', 'Y', 'Z'], 'gravity')
        self.create_section("Filtered Acceleration (m/s²)", ['X', 'Y', 'Z'], 'accel_filtered')
        self.create_section("Filtered Gyroscope (rad/s)", ['X', 'Y', 'Z'], 'gyro_filtered')
        self.create_section("Load Cells", ['Cell 0', 'Cell 1', 'Cell 2', 'Cell 3'], 'load_cells', is_int=True)
        
        # Status frame
        status_frame = ttk.LabelFrame(self.root, text="Filter Status", padding=10)
        status_frame.pack(fill="x", padx=10, pady=5)
        
        self.status_text = tk.Text(status_frame, height=3, width=70)
        self.status_text.pack()
        
    def create_section(self, title, labels, data_key, is_int=False):
        frame = ttk.LabelFrame(self.root, text=title, padding=10)
        frame.pack(fill="x", padx=10, pady=5)
        
        # Store label references for updating
        if not hasattr(self, 'value_labels'):
            self.value_labels = {}
        self.value_labels[data_key] = []
        
        for i, label in enumerate(labels):
            ttk.Label(frame, text=f"{label}:", width=8).grid(row=0, column=i*2, sticky="e", padx=5)
            value_label = ttk.Label(frame, text="0.0000" if not is_int else "0", 
                                   font=("Courier", 12, "bold"), width=10)
            value_label.grid(row=0, column=i*2+1, sticky="w", padx=5)
            self.value_labels[data_key].append(value_label)
    
    def toggle_freeze(self):
        self.frozen = not self.frozen
        if self.frozen:
            self.freeze_btn.config(text="▶ Resume")
        else:
            self.freeze_btn.config(text="⏸ Freeze")
    
    def toggle_connection(self):
        if not self.running:
            self.connect()
        else:
            self.disconnect()
    
    def connect(self):
        port = self.port_entry.get()
        try:
            # Windows-friendly serial settings
            self.serial_port = serial.Serial(
                port=port,
                baudrate=115200,
                timeout=1,  # Longer timeout for Windows
                write_timeout=1,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            time.sleep(2)  # Wait for connection to stabilize
            
            self.running = True
            self.connect_btn.config(text="Disconnect")
            self.status_label.config(text="● Connected", foreground="green")
            self.freeze_btn.config(state="normal")
            
            # Start reading thread
            self.read_thread = threading.Thread(target=self.read_serial_data, daemon=True)
            self.read_thread.start()
            
            messagebox.showinfo("Success", f"Connected to {port}")
        except Exception as e:
            messagebox.showerror("Connection Error", f"Failed to connect: {e}")
    
    def disconnect(self):
        self.running = False
        self.frozen = False
        if self.serial_port:
            self.serial_port.close()
        self.connect_btn.config(text="Connect")
        self.status_label.config(text="● Disconnected", foreground="red")
        self.freeze_btn.config(state="disabled", text="⏸ Freeze")
    
    def read_serial_data(self):
        """
        Read binary packets from Teensy using Packetizer with COBS encoding
        Packet format: [INDEX_BYTE] [COBS_ENCODED_DATA] [0x00]
        """
        buffer = bytearray()
        PACKET_START = 0x34  # send_index from Teensy
        PACKET_END = 0x00    # COBS delimiter
        
        packet_count = 0
        
        while self.running:
            try:
                # Windows fix: in_waiting can cause ClearCommError
                # Use try-except and read directly if needed
                try:
                    bytes_available = self.serial_port.in_waiting
                    if bytes_available > 0:
                        data = self.serial_port.read(bytes_available)
                        buffer.extend(data)
                        print(f"[DEBUG] Read {len(data)} bytes, buffer size: {len(buffer)}")
                except (OSError, PermissionError):
                    # Windows workaround: read with timeout instead
                    data = self.serial_port.read(100)  # Read up to 100 bytes
                    if data:
                        buffer.extend(data)
                        print(f"[DEBUG] Read {len(data)} bytes (fallback), buffer size: {len(buffer)}")
                
                # Look for complete packets: START_BYTE ... 0x00
                while True:
                    # Find packet start
                    start_idx = buffer.find(bytes([PACKET_START]))
                    if start_idx == -1:
                        if len(buffer) > 0:
                            print(f"[DEBUG] No packet start found in buffer of {len(buffer)} bytes")
                        buffer = bytearray()  # No start found, clear buffer
                        break
                    
                    # Remove data before packet start
                    if start_idx > 0:
                        buffer = buffer[start_idx:]
                    
                    # Find packet end (0x00 delimiter)
                    end_idx = buffer.find(bytes([PACKET_END]), 1)  # Start from index 1
                    if end_idx == -1:
                        # Incomplete packet, wait for more data
                        print(f"[DEBUG] Packet start found, waiting for end delimiter...")
                        break
                    
                    # Extract packet (excluding start byte and end delimiter)
                    cobs_packet = buffer[1:end_idx]
                    
                    # Remove processed packet from buffer
                    buffer = buffer[end_idx+1:]
                    
                    print(f"[DEBUG] Found packet! COBS length: {len(cobs_packet)}")
                    
                    try:
                        # COBS decode
                        decoded_data = cobs.decode(bytes(cobs_packet))
                        
                        print(f"[DEBUG] COBS decoded successfully! Data length: {len(decoded_data)}")
                        
                        # Parse the decoded data
                        if len(decoded_data) == 92:  # Expected size
                            packet_count += 1
                            print(f"[DEBUG] ✓ Packet #{packet_count} parsed successfully!")
                            self.parse_packet(decoded_data)
                        else:
                            print(f"[WARNING] Packet size mismatch. Expected 92, got {len(decoded_data)}")
                    except Exception as e:
                        print(f"[ERROR] COBS decode error: {e}")
                        print(f"[ERROR] COBS packet (first 20 bytes): {cobs_packet[:20].hex()}")
                        continue
                
                time.sleep(0.01)  # 100 Hz read rate (we update display at 10 Hz)
            except Exception as e:
                if "ClearCommError" not in str(e):
                    print(f"Read error: {e}")
                time.sleep(0.1)
    
    def parse_packet(self, data):
        """Parse binary packet matching CombinedData struct"""
        try:
            offset = 0
            
            # Load cells (4 x int16_t)
            self.imu_data['load_cells'] = list(struct.unpack('<4h', data[offset:offset+8]))
            offset += 8
            
            # Raw accelerometer (3 x float)
            self.imu_data['accel_raw'] = list(struct.unpack('<3f', data[offset:offset+12]))
            offset += 12
            
            # Raw gyroscope (3 x float)
            self.imu_data['gyro_raw'] = list(struct.unpack('<3f', data[offset:offset+12]))
            offset += 12
            
            # Gravity vector (3 x float)
            self.imu_data['gravity'] = list(struct.unpack('<3f', data[offset:offset+12]))
            offset += 12
            
            # Quaternion (4 x float)
            self.imu_data['quaternion'] = list(struct.unpack('<4f', data[offset:offset+16]))
            offset += 16
            
            # Filtered acceleration (3 x float)
            self.imu_data['accel_filtered'] = list(struct.unpack('<3f', data[offset:offset+12]))
            offset += 12
            
            # Filtered gyroscope (3 x float)
            self.imu_data['gyro_filtered'] = list(struct.unpack('<3f', data[offset:offset+12]))
            offset += 12
            
            # Status fields (3 x uint16_t)
            status_data = struct.unpack('<3H', data[offset:offset+6])
            self.imu_data['filter_status'] = status_data[0]
            self.imu_data['dynamics_mode'] = status_data[1]
            self.imu_data['status_flags'] = status_data[2]
            
            # Update GUI (only if not frozen)
            if not self.frozen:
                self.root.after(0, self.update_display)
            
        except Exception as e:
            print(f"Parse error: {e}")
    
    def update_display(self):
        """Update all GUI labels with current data"""
        # Update numerical displays
        for key, labels in self.value_labels.items():
            data = self.imu_data[key]
            for i, label in enumerate(labels):
                if i < len(data):
                    if key == 'load_cells':
                        # Check if load cell is connected (valid range: typically 0-16383 for 14-bit)
                        # Values of 0 or very low values often indicate disconnected sensor
                        value = data[i]
                        if value == 0 or abs(value) < 10:
                            label.config(text="N/A", foreground="gray")
                        else:
                            label.config(text=f"{value:d}", foreground="black")
                    else:
                        label.config(text=f"{data[i]:8.4f}", foreground="black")
        
        # Update status text
        self.status_text.delete('1.0', tk.END)
        status_text = f"Filter Status: 0x{self.imu_data['filter_status']:04X}\n"
        status_text += f"Dynamics Mode: 0x{self.imu_data['dynamics_mode']:04X}\n"
        status_text += f"Status Flags: 0x{self.imu_data['status_flags']:04X}"
        self.status_text.insert('1.0', status_text)

def main():
    root = tk.Tk()
    app = IMUDataGUI(root)
    
    # Handle window close
    def on_closing():
        app.disconnect()
        root.destroy()
    
    root.protocol("WM_DELETE_WINDOW", on_closing)
    root.mainloop()

if __name__ == "__main__":
    main()
