"""
CAN Trajectory PD Test for Joint Trajectory Matching

This script replaces trajectory_PD_test.py for CAN-based motors.
It receives target positions via UDP and sends them to CAN motors,
publishing telemetry data for comparison with simulation.

Usage:
    sudo chrt -f 99 $(which python) -u can_trajectory_PD_test.py
    
In another terminal:
    sudo chrt -f 99 $(which python) -u sshkeyboard_pd_test.py
"""

from can_motor_controller import CANMotorController
import time
import numpy as np
from publisher import DataPublisher, DataReceiver

pi = np.pi


def main():
    # Configure CAN interface - change this to match your system
    # Linux: can0, can1, etc.
    CAN_INTERFACE = "can0"
    
    print("=" * 60)
    print("CAN Motor Control - PTM Mode (Native SocketCAN)")
    print("=" * 60)
    print("PTM Mode (Position-Torque Mix)")
    print("  - Uses: kp, kd + feed-forward torque")
    print("  - Best for: Position control with gravity compensation")
    print("  - Formula: T = Kp*(Pref-Pact) + Kd*(Vref-Vact) + Tref")
    print("=" * 60)
    
    # Initialize CAN motor controller with native CAN
    # IMPORTANT: Disable background thread to prevent CAN buffer overflow!
    # This test script manually sends commands, so background thread would create
    # duplicate TX traffic causing Error 105 (ENOBUFS - no buffer space).
    # param_port is read from can_config.yaml (defaults to /dev/ttyACM0)
    motor = CANMotorController(
        can_interface=CAN_INTERFACE,
        bitrate=1000000,  # 1 Mbps
        config_path="can_config.yaml",
        control_mode="ptm",
        enable_background_control=False  # ← Disable background thread for manual control
    )
    motor.run()
    
    # Set max torque (placeholder - handled internally by motors)
    motor.set_max_torque(np.ones(10) * 1000) #1000 milliamps (mA)
    
    # Print control parameters for safety verification
    print(f"\n  Control Parameters (from config):") 
    print(f"    Mode: PTM")
    print(f"    kp  = {motor.kp}")
    print(f"    kd  = {motor.kd}")
    print(f"    torque = {motor.target_torque} Nm")
    print(f"    vel = {motor._vel}")
    
    # Zero all motors - set current position as reference zero
    print(f"\n  Zeroing all motors (current position → 0.0)...")
    motor.set_zero_position()
    
    print(f"\n    Starting in 2 seconds... (Ctrl+C to abort)\n")
    time.sleep(2)
    
    time.sleep(0.1)
    
    dt = 1/200  # 200 Hz control loop
    decimation = 4  # Publish every 4 cycles
    
    # Target positions
    dof_pos_target = np.zeros(10)
    
    np.set_printoptions(formatter={'float': '{: 3.2f}'.format})
    
    startTime = time.time()
    
    # Setup UDP publisher for telemetry
    data_publisher = DataPublisher()
    print(f"\n📊 Telemetry Publisher configured:")
    print(f"   Target: udp://localhost:9870")
    print(f"   Encoding: MessagePack")
    print(f"   PlotJuggler should listen on UDP port 9870 with MessagePack protocol\n")
    
    # Setup UDP receiver for commands
    receiver = DataReceiver(port=9871, decoding="msgpack", broadcast=True)
    receiver.receive_continuously()
    received_id = 0
    
    t = time.time()
    
    action_is_on = np.ones(10)
    should_publish = False
    publish_count = 0
    
    print("Starting control loop... Waiting for commands on UDP:9871")
    print("Press Ctrl+C to stop\n")
    
    try:
        for i in range(1000000):
            # Check for new commands from UDP
            if receiver.data is not None:
                if receiver.data_id != received_id:
                    received_id = receiver.data_id
                    
                    if "dof_pos_target" in receiver.data:
                        dof_pos_target = np.array(receiver.data["dof_pos_target"])
                        print(f"[CMD] Target position: {dof_pos_target}")
                        
                    if "action_is_on" in receiver.data:
                        action_is_on = np.array(receiver.data["action_is_on"], dtype=np.float64)
                        
                    if "should_publish" in receiver.data:
                        should_publish = receiver.data["should_publish"]
                        
                    if "kp" in receiver.data:
                        motor.kp = np.array(receiver.data["kp"], dtype=np.float64)
                        
                    if "kd" in receiver.data:
                        motor.kd = np.array(receiver.data["kd"], dtype=np.float64)
            
            # Control loop with decimation
            for _ in range(decimation):
                # CRITICAL: Manually send CAN commands (background thread is disabled)
                # Without this, motors receive no commands and will turn off!
                
                # Apply ankle coupling if enabled
                target_motor_pos = dof_pos_target.copy()
                if motor.ankle_coupling:
                    target_motor_pos[4] += target_motor_pos[3]  # Left ankle
                    target_motor_pos[9] += target_motor_pos[8]  # Right ankle
                
                # Add position offset (current pos was zeroed at startup)
                target_motor_pos = target_motor_pos + motor._motor_pos_offset
                
                # Send PTM commands to all motors continuously
                for i, m in enumerate(motor._motors):
                    if action_is_on[i] > 0.5:
                        # Active control - send PTM command
                        motor._ctrl.ptm_control(
                            m,
                            pos=target_motor_pos[i],
                            vel=motor._vel[i],
                            kp=motor._kp[i],
                            kd=motor._kd[i],
                            torque=motor._target_torque[i]
                        )
                    else:
                        # Motor disabled - send zero gains
                        motor._ctrl.ptm_control(
                            m,
                            pos=target_motor_pos[i],
                            vel=0,
                            kp=0,
                            kd=0,
                            torque=0
                        )
                    time.sleep(0.001)  # 1ms between motors (10ms total for 10 motors)
                
                # Prepare telemetry data
                data = {
                    "dof_pos": motor.dof_pos.tolist(),
                    "dof_vel": motor.dof_vel.tolist(),
                    "dof_vel_raw": motor._dof_vel_raw.tolist(),
                    "dof_current": motor.dof_current.tolist(),
                    "dof_force": motor.dof_force.tolist(),
                    "dof_force_target": motor.target_dof_torque_Nm.tolist(),
                    "dof_pos_target": dof_pos_target.tolist(),
                    "target_dof_torque_A": motor.target_dof_torque_A.tolist(),
                    "target_dof_torque_A_adjusted": motor.target_dof_torque_A_adjusted.tolist(),
                    "action_is_on": action_is_on.tolist(),
                    "t_ns": time.perf_counter_ns()
                }
                
                # Publish telemetry
                data_publisher.enable = True
                data_publisher.publish({"real": data})
                publish_count += 1
                
                # Maintain timing
                elapsed = time.time() - t
                if elapsed < dt:
                    time.sleep(dt - elapsed)
                t = time.time()
            
            # Print status every second
            if i % 200 == 0:
                print(f"[{time.time()-startTime:.1f}s] pos: {motor.dof_pos} | Published: {publish_count} packets")
                
    except KeyboardInterrupt:
        print("\n[Interrupted]")
    
    # Cleanup
    print("Stopping motors...")
    motor.target_dof_position = np.zeros(10)
    motor.use_position_pd = True
    time.sleep(0.2)
    
    motor.stop()
    receiver.stop()
    
    print("Done.")


if __name__ == "__main__":
    main()
