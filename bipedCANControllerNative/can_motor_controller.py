"""
CAN Motor Controller Adapter for Duke Humanoid Trajectory Matching Test

Usage:
    from can_motor_controller import CANMotorController
    motor = CANMotorController(serial_port="/dev/ttyUSB0")
    motor.run()

"""

from time import sleep
import threading
import signal
import numpy as np
import json
import serial
import can  # Native CAN bus support
import time  # For timeout handling

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


class LowLevelCANController:
    """Dual-mode CAN controller supporting both native CAN bus and serial devices.
    
    Auto-detects device type:
    - can.Bus → Native SocketCAN with background RX thread
    - serial.Serial → Jason USB adapter protocol with polling
    """
    
    def __init__(self, device):
        """Initialize with either CAN bus or serial device.
        
        Args:
            device: Either can.Bus (for native CAN) or serial.Serial (for Jason USB adapter)
        """
        self.motors = dict()
        
        # Auto-detect device type using duck typing
        # CAN bus has 'recv' method, serial has 'read' method
        if hasattr(device, 'recv') and hasattr(device, 'send'):
            # NATIVE CAN MODE
            self.mode = 'can'
            self.can_bus = device
            self.running = False
            self.rx_thread = None
            print(f"  LowLevelCANController: CAN mode")
            
        else:  # serial.Serial
            # SERIAL MODE (for param port)
            self.mode = 'serial'
            self.serial_device = device
            self.rx_buffer = bytes()

            # TX buffer format for Motorevo protocol
            self.tx_buffer = bytearray([
                0xAA, 0x01,  # Header
                0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC,  # Data (8 bytes)
                0x00, 0x00, 0x00, 0x00,  # Motor ID (4 bytes)
                0xF4  # Tail
            ])

            # Open serial port
            try:
                if hasattr(self.serial_device, "is_open") and self.serial_device.is_open:
                    self.serial_device.close()
            except Exception:
                pass

            try:
                self.serial_device.open()
                print(f"  LowLevelCANController: Serial mode on {getattr(self.serial_device, 'port', '<unknown>')}")
            except Exception as e:
                print(f"  Error opening serial port: {e}")

    def add_motor(self, motor: Motor):
        self.motors[motor.id] = motor

    def set_zero_position(self, motor: Motor):
        """Send zero position command to motor."""
        data_buff = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE])
        self._send_data(motor.id, data_buff)
        sleep(0.005)

    def reset_mode(self, motor: Motor = None):
        """Reset motor(s) to idle mode."""
        if motor is not None:
            mid = int(motor.id) if isinstance(motor, Motor) else int(motor)
            self._control_cmd(mid, np.uint8(0xFD))
            sleep(0.01)
            return

        if not self.motors:
            self._control_cmd(0xFFFF, np.uint8(0xFD))
        else:
            for mid in list(self.motors.keys()):
                self._control_cmd(int(mid), np.uint8(0xFD))
        sleep(0.01)

    def motor_mode(self, motor: Motor):
        """Enable motor mode for servo control."""
        self._control_cmd(motor, np.uint8(0xFC))
        sleep(0.01)

    def servo_control(self, motor: Motor, pos, vel, kp, kd, ikp, ikd, iki):
        """Send servo control command to a single motor."""
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
        sleep(0.001)  # Small delay for command processing

    def ptm_control(self, motor: Motor, pos, vel, kp, kd, torque):
        """
        PTM (Position-Torque Mix) mode control.
        
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
        pos_uint = float_to_uint(pos, motor.Q_MIN, motor.Q_MAX, 16)      # 16-bit
        vel_uint = float_to_uint(vel, motor.DQ_MIN, motor.DQ_MAX, 12)    # 12-bit
        kp_uint = float_to_uint(kp, motor.OKP_MIN, motor.OKP_MAX, 12)    # 12-bit
        kd_uint = float_to_uint(kd, motor.OKD_MIN, motor.OKD_MAX, 12)    # 12-bit
        tau_uint = float_to_uint(torque, motor.TAU_MIN, motor.TAU_MAX, 12)  # 12-bit

        # Pack bytes according to PTM specification
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

        self._send_data(motor.id, data_buff)
        sleep(0.001)  # Small delay for command processing

    def _control_cmd(self, motor, cmd):
        if isinstance(motor, Motor):
            motor_id = int(motor.id)
        elif isinstance(motor, int):
            motor_id = int(motor)
        else:
            motor_id = 0xFFFF
        data_buff = bytes([0xFF] * 7 + [cmd])
        self._send_data(motor_id, data_buff)

    def _send_data(self, motor_id, data):
        """Send command - routes based on mode."""
        
        if self.mode == 'can':
            # NATIVE CAN MODE: Send as CAN message
            msg = can.Message(
                arbitration_id=motor_id,
                data=data[:8],
                is_extended_id=False
            )
            try:
                self.can_bus.send(msg)
            except Exception as e:
                print(f"CAN TX error: {e}")
        
        else:  # serial mode
            # SERIAL MODE: Use Jason USB protocol wrapper
            self.tx_buffer[10] = motor_id & 0xFF
            self.tx_buffer[11] = (motor_id >> 8) & 0xFF
            self.tx_buffer[12] = (motor_id >> 16) & 0xFF
            self.tx_buffer[13] = (motor_id >> 24) & 0xFF

            self.tx_buffer[2:10] = data[:8] if len(data) >= 8 else (data + bytes(8 - len(data)))
            try:
                self.serial_device.write(self.tx_buffer)
            except Exception as e:
                print(f"[_send_data] write error: {e}")
    
    def _recv_data(self, motor_id):
        """Receive and process data for a specific motor (serial mode only)."""
        if self.mode != 'serial':
            return
        data_recv = b''.join([self.rx_buffer, self.serial_device.read_all()])
        frame_length = 12
        i = 0
        
        while i <= len(data_recv) - frame_length:
            frame = data_recv[i:i+frame_length]
            frame_can_id = frame[8] | (frame[9] << 8) | (frame[10] << 16) | (frame[11] << 24)
            
            if frame_can_id == motor_id:
                self._process_packet(frame[0:8], frame_can_id)
                i += frame_length
            else:
                i += 1

    def poll(self):
        """Read and process feedback from all motors (serial mode only)."""
        if self.mode != 'serial':
            return
        try:
            data_recv = b''.join([self.rx_buffer, self.serial_device.read_all()])
        except Exception:
            return

        frame_length = 12
        i = 0
        
        while i <= len(data_recv) - frame_length:
            frame = data_recv[i:i+frame_length]
            frame_can_id = frame[8] | (frame[9] << 8) | (frame[10] << 16) | (frame[11] << 24)
            
            # DEBUG: Log CAN IDs being received
            if frame_can_id in [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]:  # Only log our motor IDs
                print(f"DEBUG poll: Received packet from CAN ID {frame_can_id}, in motors: {frame_can_id in self.motors}")
            
            if frame_can_id in self.motors:
                self._process_packet(frame[0:8], frame_can_id)
                i += frame_length
            else:
                i += 1

        # Store remainder for next poll
        remainder_pos = len(data_recv) % frame_length if len(data_recv) >= frame_length else len(data_recv)
        self.rx_buffer = data_recv[-remainder_pos:] if remainder_pos else b''

    def _process_packet(self, data, can_id):
        """Process feedback packet from motor."""
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
            
            # DEBUG: Log motor 1 processing
            if can_id == 1:
                print(f"DEBUG _process_packet: Motor 1 - Setting pos={recv_q:.2f}, old pos={motor.pos:.2f}")
            
            motor.update_state(status_words, recv_q, recv_dq, recv_tau, temperature, error_code)

    def start_can_feedback(self):
        """Start background CAN RX thread (CAN mode only)."""
        if self.mode != 'can':
            return
        if self.running:
            return
        self.running = True
        self.rx_thread = threading.Thread(target=self._can_receive_loop, daemon=True)
        self.rx_thread.start()
        print("✓ CAN feedback receiver started")

    def _can_receive_loop(self):
        """Background CAN receiver thread."""
        while self.running:
            try:
                msg = self.can_bus.recv(timeout=0.01)
                if msg and msg.arbitration_id in self.motors:
                    self._process_can_feedback(msg)
            except Exception as e:
                if "timeout" not in str(e).lower():
                    print(f"CAN RX error: {e}")

    def _process_can_feedback(self, msg):
        """Process incoming CAN message and update motor state."""
        motor = self.motors[msg.arbitration_id]
        data = msg.data
        
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


class CANMotorController:
    """
    High-level CAN Motor Controller compatible with Duke's trajectory test scripts.
    
    This class provides the same interface as Duke's EtherCAT MotorController,
    allowing drop-in replacement for trajectory matching tests.
    
    Supports dual-port setup:
    - motor_port (serial_port): For sending motor commands
    - param_port: For reading motor parameters/feedback
    """
    
    def __init__(self, can_interface="can0", bitrate=1000000, config_path="can_config.yaml",
                 param_port=None, control_mode="servo"):
        """
        Initialize CAN Motor Controller with native SocketCAN.
        
        Args:
            can_interface: CAN interface name for live control (default "can0")
            bitrate: CAN bitrate in bps (default 1000000 = 1 Mbps)
            config_path: Path to configuration YAML file
            param_port: Optional serial port for motor wake-up initialization.
                       If None, uses value from config. If config has None/empty,
                       skips param port initialization.
            control_mode: Control mode - "servo" or "ptm" (default: "servo")
        """
        self.num_dof = 10
        self.running = False
        self._stop_event = threading.Event()
        
        # Load configuration
        self._load_config(config_path)
        
        # Determine param port (constructor arg overrides config)
        self._param_port_path = param_port if param_port else self.config_param_port
        self._use_separate_param_port = bool(self._param_port_path)
        
        # Create native CAN bus for live motor commands and feedback
        try:
            self.can_bus = can.Bus(interface='socketcan', channel=can_interface, bitrate=bitrate)
            print(f"✓ CAN bus '{can_interface}' opened at {bitrate} bps")
        except Exception as e:
            print(f"ERROR: Failed to open CAN interface '{can_interface}': {e}")
            raise
        
        # Create low-level controller with native CAN (for live control)
        self._ctrl = LowLevelCANController(self.can_bus)
        self._ctrl.start_can_feedback()  # Start background CAN RX thread
        
        # Create param port controller if specified (for motor wake-up initialization)
        self._param_serial = None
        self._param_ctrl = None
        if self._use_separate_param_port:
            try:
                self._param_serial = serial.Serial(
                    port=self._param_port_path,
                    baudrate=921600,  # Standard baud rate
                    timeout=0.05
                )
                self._param_ctrl = LowLevelCANController(self._param_serial)
                print(f"✓ Param port {self._param_port_path} initialized for motor wake-up")
            except Exception as e:
                print(f"⚠ Warning: Failed to open param port {self._param_port_path}: {e}")
                print("  Skipping param port initialization - motors may need manual wake-up")
                self._use_separate_param_port = False
                self._param_ctrl = None
        
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
            # Also register with param controller (same motor object so state is shared)
            if self._param_ctrl:
                self._param_ctrl.add_motor(motor)
        
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
        
        if self._use_separate_param_port:
            print(f"CANMotorController initialized with {self.num_dof} motors")
            print(f"  └── CAN bus (TX+RX): {can_interface}")
            print(f"  └── Param port (init): {self._param_port_path}")
        else:
            print(f"CANMotorController initialized with {self.num_dof} motors on {can_interface}")

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
        
        # Param port for reading motor parameters (optional)
        # Set to None, empty string, or "null" to disable
        param_port_val = config.get("param_port", None)
        if param_port_val in (None, "", "null", "None"):
            self.config_param_port = None
        else:
            self.config_param_port = param_port_val
        
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
        
        # If param_port is configured, read parameters ONCE at startup
        # This is required to "wake up" Motorevo motors
        if self._use_separate_param_port and self._param_ctrl:
            print(f"Reading motor parameters from {self._param_port_path}...")
            for _ in range(20):  # Poll for ~100ms to read initial parameters
                try:
                    self._param_ctrl.poll()
                except Exception:
                    pass
                sleep(0.005)
            print("✓ Motor parameters read - motors initialized")
        
        # Do one final poll to ensure we have the latest motor positions
        # This prevents race conditions, especially for motor 0 when param port is connected to it
        if self._use_separate_param_port and self._param_ctrl:
            for _ in range(5):  # Extra 25ms of polling
                try:
                    self._param_ctrl.poll()
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
        
        # Start control thread (sends commands via CAN bus)
        self._control_thread = threading.Thread(target=self._control_loop, daemon=True)
        self._control_thread.start()
        
        # CAN feedback is handled by background RX thread started in __init__
        # No need for separate poll thread - feedback is async
        
        sleep(0.1)
        print("Motor control started")
    
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
        
        # Shutdown CAN bus
        try:
            if self.can_bus:
                self.can_bus.shutdown()
                print("CAN bus closed")
        except Exception as e:
            print(f"Error closing CAN bus: {e}")
        
        # Close param serial port if used
        if self._param_serial:
            try:
                if self._param_serial.is_open:
                    self._param_serial.close()
                    print("Param serial port closed")
            except Exception:
                pass
        
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
    
    def _param_poll_loop(self):
        """Background thread for polling motor feedback from param port.
        
        This is the critical loop that reads motor parameters.
        Motors won't move until their parameters are read!
        """
        while not self._stop_event.is_set():
            try:
                self._param_ctrl.poll()
            except Exception:
                pass
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
    parser.add_argument('--motor-port', default='/dev/ttyUSB0', 
                        help='Serial port for motor commands')
    parser.add_argument('--param-port', default=None,
                        help='Serial port for reading motor parameters (optional)')
    parser.add_argument('--config', default='can_config.yaml',
                        help='Path to config file')
    args = parser.parse_args()
    
    print("Testing CANMotorController...")
    print(f"  Motor port: {args.motor_port}")
    print(f"  Param port: {args.param_port or '(from config or disabled)'}")
    
    # Create controller
    motor = CANMotorController(
        serial_port=args.motor_port,
        param_port=args.param_port,
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

