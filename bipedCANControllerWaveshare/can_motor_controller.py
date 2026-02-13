"""
CAN Motor Controller for Waveshare USB-CAN-A Module

Adapted from bipedCANControllerNative for Waveshare USB-CAN-A compatibility.
Uses WaveshareCANWrapper instead of native SocketCAN.

Usage:
    from can_motor_controller import CANMotorController
    motor = CANMotorController(config_file="can_config.yaml")
    motor.run()
"""

from time import sleep
import threading
import signal
import numpy as np
import json
import serial

try:
    import yaml
    HAS_YAML = True
except ImportError:
    HAS_YAML = False
    print("Warning: yaml not installed, using JSON fallback")

from utils import float_to_uint, uint_to_float

pi = np.pi


class Motor:
    """Individual motor state and parameters for Motorevo CAN motors."""
    
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
        self.type = type_name

        self.status_words = 0
        self.pos = 0.0
        self.vel = 0.0
        self.torque = 0.0
        self.temperature = 0.0
        self.error_code = 0

    def update_state(self, status_words, pos, vel, torque, temperature, error_code):
        """Update motor state from feedback."""
        self.status_words = status_words
        self.pos = pos
        self.vel = vel
        self.torque = torque
        self.temperature = temperature
        self.error_code = error_code


class WaveshareCANWrapper:
    """
    Wrapper to make Waveshare USB-CAN-A module compatible with MotorController interface.
    
    Waveshare Protocol:
    - TX: 0xAA, 0xE8 (0xC0 | 0x20 extended | 0x08 data length), ID[0-3], Data[0-7], 0x55
    - RX: 0xAA, 0xCx (flags), ID bytes, Data bytes, 0x55
    """
    def __init__(self, port, baudrate=2000000):
        self.port = port
        self.baudrate = baudrate
        self.is_open = False
        self._serial = None
        
    def open(self):
        """Open serial port and configure for 1 Mbps CAN."""
        if self.is_open:
            return
            
        self._serial = serial.Serial(self.port, self.baudrate, timeout=0.01)
        self.is_open = True
        
        # Configure Waveshare module: 1 Mbps, STANDARD frames, Variable protocol
        set_can_baudrate = [
            0xAA,     # Packet header
            0x55,     # Packet header
            0x12,     # Variable protocol
            0x01,     # CAN Baud Rate: 1 Mbps
            0x01,     # Frame Type: STANDARD Frame (NOT extended)
            0x00, 0x00, 0x00, 0x00,  # Filter ID
            0x00, 0x00, 0x00, 0x00,  # Mask ID
            0x00,     # CAN mode: normal
            0x00,     # automatic resend
            0x00, 0x00, 0x00, 0x00,  # Spare
        ]
        
        checksum = sum(set_can_baudrate[2:]) & 0xFF
        set_can_baudrate.append(checksum)
        
        self._serial.write(bytes(set_can_baudrate))
        sleep(0.1)
        print(f"✓ Waveshare USB-CAN-A configured: {self.port} @ 1 Mbps, STANDARD frames")
        
    def close(self):
        """Close serial port."""
        if self._serial and self.is_open:
            self._serial.close()
            self.is_open = False
            
    def write(self, data):
        """
        Write data using Waveshare protocol with STANDARD CAN frames.
        Expects data in Jason's format: 15 bytes with motor ID at bytes 10-13.
        Converts to Waveshare format: 0xAA 0xC8 ID[0-1] Data[0-7] 0x55
        """
        if not self.is_open or len(data) < 14:
            return
            
        # Extract CAN data and motor ID from Jason's protocol
        can_data = data[2:10]  # 8 bytes of CAN data
        motor_id = data[10] | (data[11] << 8)  # Only use lower 16 bits for standard frame
        
        # Build Waveshare packet for STANDARD frame with 8 data bytes
        # Byte 1: 0xC0 (base) | 0x00 (STANDARD frame) | 0x08 (8 data bytes) = 0xC8
        waveshare_packet = [
            0xAA,     # Header
            0xC8,     # Type: STANDARD frame (11-bit ID), 8 data bytes
            motor_id & 0xFF,              # ID byte 0 (LSB)
            (motor_id >> 8) & 0xFF,       # ID byte 1 (MSB)
        ]
        
        # Add 8 bytes of CAN data
        waveshare_packet.extend(can_data[:8])
        
        # Add end marker
        waveshare_packet.append(0x55)
        
        # Send packet
        try:
            self._serial.write(bytes(waveshare_packet))
        except Exception as e:
            print(f"[Waveshare TX Error] {e}")
            
    def read_all(self):
        """
        Read and parse ALL available Waveshare CAN frames, convert to Jason's protocol format.
        Returns: bytes in Jason's protocol format (multiple 14-byte packets concatenated)
        """
        if not self.is_open:
            return b''
            
        all_packets = b''
        
        try:
            # Read all available data first
            available = self._serial.in_waiting
            if available < 2:
                return b''
            
            # Process frames continuously while data is available
            while self._serial.in_waiting >= 2:
                # Read header (2 bytes)
                header = self._serial.read(2)
                if len(header) < 2:
                    break
                    
                # Check for Waveshare RX frame: 0xAA 0xCx
                if header[0] != 0xAA or (header[1] & 0xC0) != 0xC0:
                    continue
                    
                # Parse frame info
                data_len = header[1] & 0x0F
                is_extended = (header[1] & 0x20) != 0
                
                # Calculate total bytes to read
                # For STANDARD frames: 2-byte ID
                # For EXTENDED frames: 4-byte ID
                if is_extended:
                    frame_bytes = 4 + data_len + 1  # ID(4) + Data(n) + End(1)
                else:
                    frame_bytes = 2 + data_len + 1  # ID(2) + Data(n) + End(1)
                    
                # Read rest of frame
                frame_data = self._serial.read(frame_bytes)
                if len(frame_data) < frame_bytes:
                    break
                
                # Check end marker
                if frame_data[-1] != 0x55:
                    continue
                    
                # Extract motor ID and CAN data
                if is_extended:
                    motor_id = (frame_data[0] | 
                               (frame_data[1] << 8) | 
                               (frame_data[2] << 16) | 
                               (frame_data[3] << 24))
                    can_data = frame_data[4:4+data_len]
                else:
                    # STANDARD frame: 2-byte ID (11-bit)
                    motor_id = frame_data[0] | (frame_data[1] << 8)
                    can_data = frame_data[2:2+data_len]
                    
                # Pad CAN data to 8 bytes
                can_data = can_data + bytes(8 - len(can_data))
                
                # Convert to Jason's protocol format
                jason_packet = bytearray([
                    0xAA, 0x01,  # Header
                ])
                jason_packet.extend(can_data[:8])
                jason_packet.extend([
                    motor_id & 0xFF,
                    (motor_id >> 8) & 0xFF,
                    (motor_id >> 16) & 0xFF,  # Will be 0 for standard frames
                    (motor_id >> 24) & 0xFF,  # Will be 0 for standard frames
                ])
                
                all_packets += bytes(jason_packet)
            
            return all_packets
            
        except Exception as e:
            print(f"[Waveshare RX Error] {e}")
            return b''






class CANMotorController:
    """
    High-level CAN Motor Controller compatible with Duke's trajectory test scripts.
    
    This class provides the same interface as Duke's EtherCAT MotorController,
    allowing drop-in replacement for trajectory matching tests.
    The controller uses a single Waveshare port for both TX and RX.
    """
    
    def __init__(self, motor_port=None, config_path="can_config.yaml", control_mode="servo"):
        """
        Initialize Waveshare CAN Motor Controller.
        
        Args:
            motor_port: Serial port for Waveshare communication (default from config)
            config_path: Path to configuration YAML file
            control_mode: Control mode - "servo" or "ptm" (default: "servo")
        """
        self.num_dof = 10
        self.running = False
        self._stop_event = threading.Event()
        
        # Load configuration
        self._load_config(config_path)
        
        # Get port from config if not provided
        motor_port = motor_port or self.config_motor_port
        
        if not motor_port:
            raise ValueError("motor_port must be specified in constructor or config file")
        
        # Create Waveshare wrapper
        print(f"Initializing Waveshare port: {motor_port}")
        self._motor_serial = WaveshareCANWrapper(motor_port)
        self._motor_serial.open()
        
        # Create controller wrapper
        self._ctrl = self._create_controller(self._motor_serial)
        
        # Create and register motors
        self._motors = []
        for i, motor_id in enumerate(self.motor_ids):
            motor = Motor(
                motor_name=f"motor_{motor_id}",
                motor_id=motor_id,
                Q_MAX=self.Q_MAX,
                Q_MIN=self.Q_MIN,
                DQ_MAX=self.DQ_MAX,
                DQ_MIN=self.DQ_MIN,
                TAU_MAX=self.TAU_MAX,
                TAU_MIN=self.TAU_MIN,
                OKP_MAX=self.OKP_MAX,
                OKP_MIN=self.OKP_MIN,
                OKD_MAX=self.OKD_MAX,
                OKD_MIN=self.OKD_MIN,
                IKP_MAX=self.IKP_MAX,
                IKP_MIN=self.IKP_MIN,
                IKD_MAX=self.IKD_MAX,
                IKD_MIN=self.IKD_MIN,
                IKI_MAX=self.IKI_MAX,
                IKI_MIN=self.IKI_MIN
            )
            self._motors.append(motor)
            self._ctrl.add_motor(motor)
        
        # Control state
        self._control_mode = control_mode  # "servo" or "ptm"
        self._target_dof_position = np.zeros(self.num_dof)
        self._kp = np.array(self.default_kp) if isinstance(self.default_kp, list) else np.ones(self.num_dof) * self.default_kp
        self._kd = np.array(self.default_kd) if isinstance(self.default_kd, list) else np.ones(self.num_dof) * self.default_kd
        self._ikp = np.array(self.default_ikp) if isinstance(self.default_ikp, list) else np.ones(self.num_dof) * self.default_ikp
        self._ikd = np.array(self.default_ikd) if isinstance(self.default_ikd, list) else np.ones(self.num_dof) * self.default_ikd
        self._iki = np.array(self.default_iki) if isinstance(self.default_iki, list) else np.ones(self.num_dof) * self.default_iki
        self._vel = np.ones(self.num_dof) * self.default_vel
        self._target_torque = np.array(self.default_torque)  # For PTM mode
        self._use_position_pd = False  # Will be enabled in main loop
        self._torque_multiplier = np.ones(self.num_dof)
        
        # Loop counter for timing/decimation tracking  
        self.loop_counter = 0
        
        # Position offset (for zero calibration)
        self._motor_pos_offset = np.array(self.motor_pos_offset)
        
        # Control thread
        self._control_thread = None
        
        # Signal handler for clean shutdown
        def signal_handler(sig, frame):
            print("Keyboard Interrupt!")
            self.stop()
            exit()
        signal.signal(signal.SIGINT, signal_handler)
        
        print(f"✓ CANMotorController initialized with {self.num_dof} motors on {motor_port}")
    
    def _create_controller(self, device):
        """Create a simple controller wrapper for Waveshare device."""
        class SimpleController:
            def __init__(self, device):
                self.device = device
                self.motors = dict()
                self.rx_buffer = bytes()
                self.tx_buffer = bytearray([
                    0xAA, 0x01,
                    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC,
                    0x00, 0x00, 0x00, 0x00,
                    0xF4
                ])
            
            def add_motor(self, motor):
                self.motors[motor.id] = motor
            
            def set_zero_position(self, motor):
                data_buff = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE])
                self._send_data(motor.id, data_buff)
                sleep(0.005)
            
            def reset_mode(self, motor):
                mid = int(motor.id) if hasattr(motor, 'id') else int(motor)
                data_buff = bytes([0xFF] * 7 + [0xFD])
                self._send_data(mid, data_buff)
                sleep(0.01)
            
            def motor_mode(self, motor):
                mid = int(motor.id) if hasattr(motor, 'id') else int(motor)
                data_buff = bytes([0xFF] * 7 + [0xFC])
                self._send_data(mid, data_buff)
                sleep(0.01)
            
            def servo_control(self, motor, pos, vel, kp, kd, ikp, ikd, iki):
                pos_uint = float_to_uint(pos, motor.Q_MIN, motor.Q_MAX, 16)
                vel_uint = float_to_uint(vel, motor.DQ_MIN, motor.DQ_MAX, 8)
                kp_uint = float_to_uint(kp, motor.OKP_MIN, motor.OKP_MAX, 8)
                kd_uint = float_to_uint(kd, motor.OKD_MIN, motor.OKD_MAX, 8)
                ikp_uint = float_to_uint(ikp, motor.IKP_MIN, motor.IKP_MAX, 8)
                ikd_uint = float_to_uint(ikd, motor.IKD_MIN, motor.IKD_MAX, 8)
                iki_uint = float_to_uint(iki, motor.IKI_MIN, motor.IKI_MAX, 8)
                
                data_buff = bytes([
                    (pos_uint >> 8) & 0xFF,
                    pos_uint & 0xFF,
                    vel_uint & 0xFF,
                    kp_uint & 0xFF,
                    kd_uint & 0xFF,
                    ikp_uint & 0xFF,
                    ikd_uint & 0xFF,
                    iki_uint & 0xFF
                ])
                
                self._send_data(motor.id, data_buff)
                sleep(0.001)
            
            def ptm_control(self, motor, pos, vel, kp, kd, torque):
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
                
                self._send_data(motor.id, data_buff)
                sleep(0.001)
            
            def poll(self):
                """Poll for feedback from motors."""
                try:
                    data_recv = b''.join([self.rx_buffer, self.device.read_all()])
                except Exception:
                    return
                
                frame_length = 14  # Jason protocol: 2 + 8 + 4 = 14
                packets = []
                i = 0
                
                while i < len(data_recv) - 13:
                    if data_recv[i] == 0xAA and data_recv[i + 1] == 0x01:
                        packet = data_recv[i:i + 14]
                        if len(packet) == 14:
                            packets.append(packet)
                        i += 14
                    else:
                        i += 1
                
                # Store remainder
                if len(data_recv) >= frame_length:
                    remainder_pos = len(data_recv) % frame_length
                    self.rx_buffer = data_recv[-remainder_pos:] if remainder_pos else b''
                else:
                    self.rx_buffer = data_recv
                
                # Process packets
                for packet in packets:
                    data = packet[2:10]
                    can_id = packet[10] | (packet[11] << 8) | (packet[12] << 16) | (packet[13] << 24)
                    
                    if can_id in self.motors:
                        motor = self.motors[can_id]
                        status_words = data[0]
                        q_uint = (data[1] << 8) | data[2]
                        dq_uint = (data[3] << 4) | (data[4] >> 4)
                        tau_uint = ((data[4] & 0x0F) << 8) | data[5]
                        error_code = data[6]
                        temperature = data[7]
                        
                        recv_q = uint_to_float(q_uint, motor.Q_MIN, motor.Q_MAX, 16)
                        recv_dq = uint_to_float(dq_uint, motor.DQ_MIN, motor.DQ_MAX, 12)
                        recv_tau = uint_to_float(tau_uint, motor.TAU_MIN, motor.TAU_MAX, 12)
                        
                        motor.update_state(status_words, recv_q, recv_dq, recv_tau, temperature, error_code)
            
            def _send_data(self, motor_id, data):
                self.tx_buffer[10] = motor_id & 0xFF
                self.tx_buffer[11] = (motor_id >> 8) & 0xFF
                self.tx_buffer[12] = (motor_id >> 16) & 0xFF
                self.tx_buffer[13] = (motor_id >> 24) & 0xFF
                self.tx_buffer[2:10] = data[:8] if len(data) >= 8 else (data + bytes(8 - len(data)))
                
                try:
                    self.device.write(self.tx_buffer)
                except Exception as e:
                    print(f"[_send_data] write error: {e}")
        
        return SimpleController(device)

    def _load_config(self, config_path):
        """Load configuration from YAML or JSON file."""
        config = {}
        
        try:
            with open(config_path, "r") as f:
                if HAS_YAML and config_path.endswith(('.yaml', '.yml')):
                    config = yaml.safe_load(f)
                else:
                    # Try JSON fallback
                    config = json.load(f)
        except FileNotFoundError:
            print(f"Warning: {config_path} not found, using defaults")
        except Exception as e:
            print(f"Warning: Failed to load config: {e}, using defaults")
        
        # Motor port (required)
        self.config_motor_port = config.get("motor_port", None)
        
        # Motor IDs
        self.motor_ids = config.get("motor_ids", [1, 2, 3, 4, 5, 6, 7, 8, 9, 10])
        
        # Limits
        self.min_limit = np.array(config.get("min_limit", [-0.785]*5 + [-0.785]*5))
        self.max_limit = np.array(config.get("max_limit", [0.785]*5 + [0.785]*5))
        
        # Motor parameter ranges
        self.Q_MIN = config.get("Q_MIN", -12.5)
        self.Q_MAX = config.get("Q_MAX", 12.5)
        self.DQ_MIN = config.get("DQ_MIN", -10.0)
        self.DQ_MAX = config.get("DQ_MAX", 10.0)
        self.TAU_MIN = config.get("TAU_MIN", -50.0)
        self.TAU_MAX = config.get("TAU_MAX", 50.0)
        self.OKP_MIN = config.get("OKP_MIN", 0.0)
        self.OKP_MAX = config.get("OKP_MAX", 250.0)
        self.OKD_MIN = config.get("OKD_MIN", 0.0)
        self.OKD_MAX = config.get("OKD_MAX", 50.0)
        self.IKP_MIN = config.get("IKP_MIN", 0.0)
        self.IKP_MAX = config.get("IKP_MAX", 250.0)
        self.IKD_MIN = config.get("IKD_MIN", 0.0)
        self.IKD_MAX = config.get("IKD_MAX", 50.0)
        self.IKI_MIN = config.get("IKI_MIN", 0.0)
        self.IKI_MAX = config.get("IKI_MAX", 50.0)
        
        # Default gains
        self.default_kp = config.get("default_kp", 20) #60.0
        self.default_kd = config.get("default_kd", 0.8) #5.0
        self.default_ikp = config.get("default_ikp", 50) #30.0
        self.default_ikd = config.get("default_ikd", 0.01)
        self.default_iki = config.get("default_iki", 0.0)
        self.default_vel = config.get("default_vel", 2.0)
        self.default_torque = config.get("default_torque", [0.0]*10)  # For PTM mode
        
        # Position offset
        self.motor_pos_offset = config.get("motor_pos_offset", [0.0]*10)
        
        # Ankle coupling
        self.ankle_coupling = config.get("ankle_coupling", False)
        
        # Control timing
        self.control_dt = config.get("control_dt", 0.005)

    # ========== Properties matching Duke's interface ==========
    
    @property
    def dof_pos(self) -> np.ndarray:
        """Returns current DOF positions (radians)."""
        positions = np.array([m.pos for m in self._motors])
        return positions - self._motor_pos_offset
    
    @property
    def dof_vel(self) -> np.ndarray:
        """Returns current DOF velocities (rad/s)."""
        return np.array([m.vel for m in self._motors])
    
    @property
    def dof_velocity_filtered(self) -> np.ndarray:
        """Returns filtered DOF velocities (rad/s)."""
        # Simple pass-through, could add filtering if needed
        return self.dof_vel
    
    @property
    def dof_current(self) -> np.ndarray:
        """Returns current motor currents."""
        return np.array([m.torque for m in self._motors])
    
    @property
    def dof_force(self) -> np.ndarray:
        """Returns current DOF torques (Nm)."""
        return np.array([m.torque for m in self._motors])
    
    @property
    def _dof_vel_raw(self) -> np.ndarray:
        """Returns raw DOF velocities."""
        return self.dof_vel
    
    @property
    def target_dof_position(self) -> np.ndarray:
        """Target DOF positions."""
        return self._target_dof_position
    
    @target_dof_position.setter
    def target_dof_position(self, value):
        self._target_dof_position = np.array(value)
    
    @property
    def kp(self) -> np.ndarray:
        return self._kp
    
    @kp.setter
    def kp(self, value):
        self._kp = np.array(value)
    
    @property
    def kd(self) -> np.ndarray:
        return self._kd
    
    @kd.setter
    def kd(self, value):
        self._kd = np.array(value)
    
    @property
    def use_position_pd(self) -> bool:
        return self._use_position_pd
    
    @use_position_pd.setter
    def use_position_pd(self, value: bool):
        self._use_position_pd = value
    
    @property
    def torque_multiplier(self) -> np.ndarray:
        return self._torque_multiplier
    
    @torque_multiplier.setter
    def torque_multiplier(self, value):
        self._torque_multiplier = np.array(value)
    
    @property
    def control_mode(self) -> str:
        """Current control mode: 'servo' or 'ptm'."""
        return self._control_mode
    
    @control_mode.setter
    def control_mode(self, value: str):
        if value not in ["servo", "ptm"]:
            raise ValueError("control_mode must be 'servo' or 'ptm'")
        self._control_mode = value
    
    @property
    def target_torque(self) -> np.ndarray:
        """Target feed-forward torques for PTM mode."""
        return self._target_torque
    
    @target_torque.setter
    def target_torque(self, value):
        self._target_torque = np.array(value)
    
    @property
    def target_dof_torque_Nm(self) -> np.ndarray:
        """Placeholder for target torques."""
        return np.zeros(self.num_dof)
    
    @property
    def target_dof_torque_A(self) -> np.ndarray:
        """Placeholder for target torques in amps."""
        return np.zeros(self.num_dof)
    
    @property
    def target_dof_torque_A_adjusted(self) -> np.ndarray:
        """Placeholder for adjusted target torques."""
        return np.zeros(self.num_dof)

    # ========== Control methods ==========
    
    def run(self):
        """Start motor control loop."""
        if self.running:
            return
            
        self.running = True
        self._stop_event.clear()
        
        # Enable motor mode for all motors
        for motor in self._motors:
            self._ctrl.motor_mode(motor)
        
        # Poll for initial motor feedback
        for _ in range(20):  # Poll for ~100ms to read initial parameters
            try:
                self._ctrl.poll()
            except Exception:
                pass
            sleep(0.005)
        
        # Debug: Print motor positions before setting offsets
        motor_positions = [motor.pos for motor in self._motors]
        print(f"DEBUG: Motor positions after polling: {[f'{p:.2f}' for p in motor_positions]}")
        print(f"DEBUG: Motor IDs: {[motor.id for motor in self._motors]}")
        
        # Set current motor positions as reference zero to prevent jump starts
        # motor_pos_offset makes the current physical position act as "zero" for control
        for i, motor in enumerate(self._motors):
            self._motor_pos_offset[i] = motor.pos
        print(f"✓ Current motor positions set as reference zero: {[f'{p:.2f}' for p in self._motor_pos_offset]}")
        
        # Start control thread (sends commands via Waveshare)
        self._control_thread = threading.Thread(target=self._control_loop, daemon=True)
        self._control_thread.start()
        
        # Start polling thread - continuously poll feedback from Waveshare port
        # This is CRITICAL: without continuous polling, motor positions don't update!
        self._poll_thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._poll_thread.start()
        
        sleep(0.1)
        print("✓ Motor control started")
    
    def set_zero_position(self, motor_indices=None):
        """
        Set current position as zero for specified motors.
        
        Args:
            motor_indices: List of motor indices (0-9) to zero. If None, zeros all motors.
        """
        if motor_indices is None:
            motor_indices = range(self.num_dof)
        
        print(f"Setting zero position for {len(list(motor_indices))} motors...")
        for i in motor_indices:
            if 0 <= i < len(self._motors):
                self._ctrl.set_zero_position(self._motors[i])
                print(f"  ✓ Motor {self._motors[i].id} zeroed")
        
        # Small delay to let commands process
        sleep(0.1)
        print("✓ All motors zeroed - current positions are now reference zero")
    
    def limit_check(self, dof_pos_target):
        """
        Check and enforce joint limits for safety.
        
        Prevents robot from damaging itself by moving beyond safe joint angles.
        Only clamps when BOTH current position AND target are beyond limits.
        
        Args:
            dof_pos_target: Target joint positions [10]
            
        Returns:
            Safe target positions with limits enforced
        """
        current_pos = self.dof_pos
        over_max_limit = (current_pos > self.max_limit) & (dof_pos_target > self.max_limit)
        under_min_limit = (current_pos < self.min_limit) & (dof_pos_target < self.min_limit)
        
        if np.any(over_max_limit) or np.any(under_min_limit):
            # Clamp targets to limits
            result = np.where(over_max_limit, self.max_limit, dof_pos_target)
            result = np.where(under_min_limit, self.min_limit, result)
            # Optionally log warnings (comment out for production)
            # if np.any(under_min_limit):
            #     print(f"⚠ Joints {np.flatnonzero(under_min_limit)} under min limit")
            # if np.any(over_max_limit):
            #     print(f"⚠ Joints {np.flatnonzero(over_max_limit)} over max limit")
            return result
        else:
            return dof_pos_target
    
    def stop(self):
        """Stop motor control loop."""
        self._stop_event.set()
        self.running = False
        
        # Stop CAN feedback receiver
        if self._ctrl:
            self._ctrl.running = False
        
        # Reset all motors
        for motor in self._motors:
            self._ctrl.reset_mode(motor)
        
        # Close Waveshare port
        try:
            if self._motor_serial:
                self._motor_serial.close()
                print("✓ Waveshare port closed")
        except Exception as e:
            print(f"Error closing port: {e}")
        
        print("Motor control stopped")
    
    def _control_loop(self):
        """Background thread for sending control commands.
        
        CRITICAL: Motorevo servos require continuous command stream to maintain position!
        If commands stop, motors idle and internal gears may oscillate then stop.
        """
        while not self._stop_event.is_set():
            # Apply ankle coupling if enabled
            target_motor_pos = self._target_dof_position.copy()
            if self.ankle_coupling:
                target_motor_pos[4] += target_motor_pos[3]  # Left ankle
                target_motor_pos[9] += target_motor_pos[8]  # Right ankle
            
            # Add position offset
            target_motor_pos = target_motor_pos + self._motor_pos_offset
            
            # Send commands to ALL motors CONTINUOUSLY (matches working code behavior)
            for i, motor in enumerate(self._motors):
                if self._use_position_pd and self._torque_multiplier[i] > 0.5:
                    # Active position control - use selected control mode
                    if self._control_mode == "servo":
                        # Servo mode: inner-loop PID
                        self._ctrl.servo_control(
                            motor,
                            pos=target_motor_pos[i],
                            vel=self._vel[i],
                            kp=self._kp[i],
                            kd=self._kd[i],
                            ikp=self._ikp[i],
                            ikd=self._ikd[i],
                            iki=self._iki[i]
                        )
                    elif self._control_mode == "ptm":
                        # PTM mode: feed-forward torque
                        self._ctrl.ptm_control(
                            motor,
                            pos=target_motor_pos[i],
                            vel=self._vel[i],
                            kp=self._kp[i],
                            kd=self._kd[i],
                            torque=self._target_torque[i]
                        )
                else:
                    # Motor disabled or not in PD mode - send target position with zero gains
                    # This prevents motors from holding random initial positions
                    if self._control_mode == "servo":
                        self._ctrl.servo_control(
                            motor,
                            pos=target_motor_pos[i],  # Use target position, not current
                            vel=0,
                            kp=0,
                            kd=0,
                            ikp=0,
                            ikd=0,
                            iki=0
                        )
                    elif self._control_mode == "ptm":
                        self._ctrl.ptm_control(
                            motor,
                            pos=target_motor_pos[i],  # Use target position, not current
                            vel=0,
                            kp=0,
                            kd=0,
                            torque=0
                        )
                sleep(0.001)  # Small delay between motors (matches working code)
            
            # Increment loop counter for timing/decimation tracking
            self.loop_counter += 1
            sleep(self.control_dt)
    
    def _poll_loop(self):
        """Background thread for polling motor feedback from motor port."""
        while not self._stop_event.is_set():
            self._ctrl.poll()
            sleep(0.005)
    

    
    def set_max_torque(self, max_torques):
        """Set maximum torque for each motor (placeholder for compatibility)."""
        # Motorevo motors handle torque limits internally
        pass
    
    def set_position_offset(self, offsets):
        """Set position offset for calibration."""
        self._motor_pos_offset = np.array(offsets)
    
    def initialize_motor_pos_offset(self):
        """Calibrate motor position offsets at current position as zero."""
        self.run()
        sleep(0.2)
        
        current_pos = np.array([m.pos for m in self._motors])
        desire_pos = np.zeros(self.num_dof)
        offset = current_pos - desire_pos
        
        self._motor_pos_offset = offset
        
        # Save to config
        try:
            with open("can_config.yaml", "r") as f:
                config = yaml.safe_load(f)
            config["motor_pos_offset"] = offset.tolist()
            with open("can_config.yaml", "w") as f:
                yaml.dump(config, f)
            print(f"Position offset saved: {offset}")
        except Exception as e:
            print(f"Failed to save offset: {e}")
        
        return offset


# ========== Test code ==========
if __name__ == "__main__":
    import time
    import argparse
    
    parser = argparse.ArgumentParser(description='Test CAN Motor Controller')
    parser.add_argument('--motor-port', dest='motor_port',
                        help='Serial port for motor control (e.g., COM13 or /dev/ttyUSB0)')
    parser.add_argument('--config', default='can_config.yaml',
                        help='Path to config file')
    args = parser.parse_args()
    
    print("Testing CANMotorController...")
    print(f"  Motor port: {args.motor_port or '(from config)'}")
    
    # Create controller
    motor = CANMotorController(
        motor_port=args.motor_port,
        config_path=args.config
    )
    
    # Start control
    motor.run()
    
    # Print control parameters for safety verification
    print(f"\n  Control Parameters (from config):")
    print(f"    kp  = {motor.kp[0]:.2f}")
    print(f"    kd  = {motor.kd[0]:.2f}")
    print(f"    ikp = {motor._ikp[0]:.2f}")
    print(f"    vel = {motor._vel[0]:.2f}")
    print(f"\n    Starting in 2 seconds... (Ctrl+C to abort)\n")
    time.sleep(2)
    
    # Enable position control
    motor.target_dof_position = np.zeros(10)
    motor.use_position_pd = True
    
    print("Running for 5 seconds...")
    start = time.time()
    while time.time() - start < 5:
        print(f"dof_pos: {motor.dof_pos}")
        print(f"dof_vel: {motor.dof_vel}")
        sleep(0.5)
    
    motor.stop()
    print("Done")
