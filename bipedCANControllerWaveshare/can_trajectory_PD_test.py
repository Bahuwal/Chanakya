"""
CAN Trajectory PD Test for Joint Trajectory Matching (Waveshare USB-CAN-A)

This script replaces trajectory_PD_test.py for Waveshare CAN motors.
It receives target positions via UDP and sends them to motors via Waveshare USB-CAN-A,
publishing telemetry data for comparison with simulation.

Usage:
    python can_trajectory_PD_test.py
    
In another terminal:
    python sshkeyboard_pd_test.py
"""

from can_motor_controller import CANMotorController
import time
import numpy as np
from publisher import DataPublisher, DataReceiver

pi = np.pi

def main():
    # Waveshare ports are configured in can_config.yaml
    # Windows: COM13, COM18
    # Linux: /dev/ttyUSB0, /dev/ttyUSB1
    
    print("=" * 60)
    print("CAN Motor Control - PTM Mode (Waveshare USB-CAN-A)")
    print("=" * 60)
    print("PTM Mode (Position-Torque Mix)")
    print("  - Uses: kp, kd + feed-forward torque")
    print("  - Best for: Position control with gravity compensation")
    print("  - Formula: T = Kp*(Pref-Pact) + Kd*(Vref-Vact) + Tref")
    print("=" * 60)
    
    # Initialize Waveshare motor controller
    # motor_port is read from can_config.yaml
    motor = CANMotorController(
        config_path="can_config.yaml",
        control_mode="ptm"
    )
    
    # CRITICAL: Enable position control BEFORE motor.run()
    # Motors have 100ms watchdog - if they receive zero-gain commands, they timeout!
    # Background thread needs this flag to send actual control commands
    motor.use_position_pd = True
    
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
                # Send position commands
                motor.use_position_pd = True
                motor.target_dof_position = dof_pos_target
                motor.torque_multiplier = action_is_on
                
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