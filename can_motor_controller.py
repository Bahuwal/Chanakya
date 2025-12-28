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
    """Low-level CAN motor controller for serial communication."""
    
    def __init__(self, serial_device):
        self.serial_device = serial_device
        self.motors = dict()
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
            print(f"Serial port {getattr(self.serial_device, 'port', '<unknown>')} opened.")
        except Exception as e:
            print(f"Error opening serial port: {e}")

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
        self.poll()  # Read feedback immediately after command (like working code does)

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
        self.tx_buffer[10] = motor_id & 0xFF
        self.tx_buffer[11] = (motor_id >> 8) & 0xFF
        self.tx_buffer[12] = (motor_id >> 16) & 0xFF
        self.tx_buffer[13] = (motor_id >> 24) & 0xFF

        self.tx_buffer[2:10] = data[:8] if len(data) >= 8 else (data + bytes(8 - len(data)))
        try:
            self.serial_device.write(self.tx_buffer)
        except Exception as e:
            print(f"[_send_data] write error: {e}")

    def poll(self):
        """Read and process feedback from all motors."""
        try:
            data_recv = b''.join([self.rx_buffer, self.serial_device.read_all()])
        except Exception:
            return

        frame_length = 12
        i = 0
        
        while i <= len(data_recv) - frame_length:
            frame = data_recv[i:i+frame_length]
            frame_can_id = frame[8] | (frame[9] << 8) | (frame[10] << 16) | (frame[11] << 24)
            
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
    
    def __init__(self, serial_port="/dev/ttyUSB0", baudrate=921600, config_path="can_config.yaml",
                 param_port=None, control_mode="servo"):
        """
        Initialize CAN Motor Controller.
        
        Args:
            serial_port: Serial port for sending motor commands
            baudrate: Baud rate (default 921600)
            config_path: Path to configuration YAML file
            param_port: Optional separate port for reading motor parameters.
                       If None, uses value from config. If config has None/empty,
                       feedback is read from serial_port.
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
        
        # Create motor serial device (for sending commands)
        self.serial_device = serial.Serial(
            port=serial_port,
            baudrate=baudrate,
            timeout=0.05
        )
        
        # Create low-level controller for motor commands
        self._ctrl = LowLevelCANController(self.serial_device)
        
        # Create param port controller if specified (for reading feedback)
        self._param_serial = None
        self._param_ctrl = None
        if self._use_separate_param_port:
            try:
                self._param_serial = serial.Serial(
                    port=self._param_port_path,
                    baudrate=baudrate,
                    timeout=0.05
                )
                self._param_ctrl = LowLevelCANController(self._param_serial)
                print(f"✓ Param port {self._param_port_path} initialized for reading motor parameters")
            except Exception as e:
                print(f"⚠ Warning: Failed to open param port {self._param_port_path}: {e}")
                print("  Feedback will be read from motor port instead")
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
        self._kp = np.ones(self.num_dof) * self.default_kp
        self._kd = np.ones(self.num_dof) * self.default_kd
        self._ikp = np.ones(self.num_dof) * self.default_ikp
        self._ikd = np.ones(self.num_dof) * self.default_ikd
        self._iki = np.ones(self.num_dof) * self.default_iki
        self._vel = np.ones(self.num_dof) * self.default_vel
        self._target_torque = np.array(self.default_torque)  # For PTM mode
        self._use_position_pd = False  # Will be enabled in main loop
        self._torque_multiplier = np.ones(self.num_dof)
        
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
            print(f"  └── Motor port (TX): {serial_port}")
            print(f"  └── Param port (RX): {self._param_port_path}")
        else:
            print(f"CANMotorController initialized with {self.num_dof} motors on {serial_port}")

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
        self.motor_ids = config.get("motor_ids", [0, 1, 2, 3, 4, 5, 6, 7, 8, 9])
        
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
        
        # Start control thread (sends commands via motor_port)
        self._control_thread = threading.Thread(target=self._control_loop, daemon=True)
        self._control_thread.start()
        
        # Start polling thread - ALWAYS poll feedback from motor_port (USB0)
        # Motor feedback (pos, vel, torque) comes from motor_port during operation
        self._poll_thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._poll_thread.start()
        
        sleep(0.1)
        print("Motor control started")
    
    def stop(self):
        """Stop motor control loop."""
        self._stop_event.set()
        self.running = False
        
        # Reset all motors
        for motor in self._motors:
            self._ctrl.reset_mode(motor)
        
        # Close motor serial port
        try:
            if self.serial_device.is_open:
                self.serial_device.close()
                print("Motor serial port closed")
        except Exception:
            pass
        
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

