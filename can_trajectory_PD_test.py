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
    # Configure serial port - change this to match your system
    # Linux: /dev/ttyUSB0
    # macOS: /dev/cu.usbserial-*
    # Windows: COM3
    SERIAL_PORT = "/dev/ttyUSB0"
    
    print("=" * 60)
    print("CAN Trajectory PD Test - Joint Trajectory Matching")
    print("=" * 60)
    
    # Initialize CAN motor controller
    motor = CANMotorController(
        serial_port=SERIAL_PORT,
        config_path="can_config.yaml"
    )
    motor.run()
    
    # Set max torque (placeholder - handled internally by motors)
    motor.set_max_torque(np.ones(10) * 1000) #1000 milliamps (mA)
    
    # Set default PD gains
    motor.kp = np.ones(10) * 60
    motor.kd = np.ones(10) * 5
    
    time.sleep(0.1)
    
    dt = 1/200  # 200 Hz control loop
    decimation = 4  # Publish every 4 cycles
    
    # Target positions
    dof_pos_target = np.zeros(10)
    
    np.set_printoptions(formatter={'float': '{: 3.2f}'.format})
    
    startTime = time.time()
    
    # Setup UDP publisher for telemetry
    data_publisher = DataPublisher()
    
    # Setup UDP receiver for commands
    receiver = DataReceiver(port=9871, decoding="msgpack", broadcast=True)
    receiver.receive_continuously()
    received_id = 0
    
    t = time.time()
    
    action_is_on = np.ones(10)
    should_publish = False
    
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
                
                # Maintain timing
                elapsed = time.time() - t
                if elapsed < dt:
                    time.sleep(dt - elapsed)
                t = time.time()
            
            # Print status every second
            if i % 200 == 0:
                print(f"[{time.time()-startTime:.1f}s] pos: {motor.dof_pos}")
                
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
