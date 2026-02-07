import numpy as np
from publisher import DataPublisher
import pygame
import sys

# Maximum velocities
lin_vel_x_max = 0.3
lin_vel_y_max = 0.25
ang_vel_z_max = 0.2

# Keyboard velocity step (for smoother control)
vel_step = 0.05
yaw_step = 0.05

def print_controls(mode):
    """Print control instructions based on mode"""
    if mode == 1:
        print("\n🎮 GAMEPAD CONTROLS:")
        print("  Left Stick Y   : Forward/Backward")
        print("  Left Stick X   : Rotate Left/Right")
        print("  Right Stick Y  : Strafe Left/Right")
        print("  L1 Button (4)  : Reset Robot")
        print("  Press Ctrl+C to quit\n")
    else:
        print("\n⌨️  KEYBOARD CONTROLS:")
        print("  W/S     : Forward/Backward")
        print("  A/D     : Rotate Left/Right")
        print("  Q/E     : Strafe Left/Right")
        print("  R       : Reset Robot")
        print("  SPACE   : Stop (zero all velocities)")
        print("  ESC     : Quit")
        print("  Press Ctrl+C to quit\n")

def select_mode():
    """Ask user to select control mode"""
    print("\n" + "="*50)
    print("  BIPED ROBOT CONTROL - MODE SELECTION")
    print("="*50)
    print("\nSelect control mode:")
    print("  1 - Gamepad Control (Xbox/PS controller)")
    print("  2 - Keyboard Control (WASD)")
    print("="*50)
    
    while True:
        try:
            choice = input("\nEnter choice (1 or 2): ").strip()
            if choice in ['1', '2']:
                return int(choice)
            else:
                print("❌ Invalid choice! Please enter 1 or 2.")
        except KeyboardInterrupt:
            print("\n\nExiting...")
            sys.exit(0)

def gamepad_mode(data_publisher):
    """Run gamepad control mode"""
    print_controls(1)
    
    keyboard_operator_cmd = np.zeros(3)  # vel_x, vel_y, yaw_orientation
    reset = False
    
    pygame.init()
    pygame.joystick.init()
    
    joysticks = [pygame.joystick.Joystick(x) for x in range(pygame.joystick.get_count())]
    
    if len(joysticks) == 0:
        print("❌ No gamepad detected! Please connect a controller or use keyboard mode.")
        return
    
    for joystick in joysticks:
        joystick.init()
        print(f"✅ Connected: {joystick.get_name()}")
    
    running = True
    while running:
        updated = False
        
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.JOYBUTTONDOWN:
                if event.button == 4:  # L1/LB button
                    updated = True
                    reset = True
                    print("🔄 RESET command sent!")
            elif event.type == pygame.JOYAXISMOTION:
                if event.axis == 1:  # Left stick Y - Forward/Backward
                    keyboard_operator_cmd[0] = np.round(-event.value * lin_vel_x_max, decimals=2)
                    updated = True
                elif event.axis == 0:  # Left stick X - Rotation
                    keyboard_operator_cmd[2] = np.round(-event.value * ang_vel_z_max, decimals=2)
                    updated = True
                elif event.axis == 3:  # Right stick Y - Strafe
                    keyboard_operator_cmd[1] = np.round(-event.value * lin_vel_y_max, decimals=2)
                    updated = True
        
        if updated:
            data = {
                "cmd": keyboard_operator_cmd,
                "reset": reset,
            }
            print(f"📡 Cmd: vel_x={keyboard_operator_cmd[0]:.2f} vel_y={keyboard_operator_cmd[1]:.2f} yaw={keyboard_operator_cmd[2]:.2f}")
            data_publisher.publish(data)
            reset = False
    
    pygame.quit()

def keyboard_mode(data_publisher):
    """Run keyboard control mode"""
    print_controls(2)
    
    keyboard_operator_cmd = np.zeros(3)  # vel_x, vel_y, yaw_orientation
    reset = False
    
    pygame.init()
    screen = pygame.display.set_mode((400, 300))
    pygame.display.set_caption("Biped Control - WASD Mode")
    clock = pygame.time.Clock()
    
    # Font for on-screen display
    font = pygame.font.Font(None, 24)
    
    # Key states
    keys_pressed = set()
    
    running = True
    while running:
        updated = False
        
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                keys_pressed.add(event.key)
                
                # Special commands
                if event.key == pygame.K_r:
                    reset = True
                    updated = True
                    print("🔄 RESET command sent!")
                elif event.key == pygame.K_SPACE:
                    keyboard_operator_cmd[:] = 0
                    updated = True
                    print("⏹️  STOP - All velocities zeroed")
                elif event.key == pygame.K_ESCAPE:
                    running = False
                    
            elif event.type == pygame.KEYUP:
                keys_pressed.discard(event.key)
        
        # Update velocities based on pressed keys
        old_cmd = keyboard_operator_cmd.copy()
        
        # Forward/Backward (W/S)
        if pygame.K_w in keys_pressed:
            keyboard_operator_cmd[0] = min(keyboard_operator_cmd[0] + vel_step, lin_vel_x_max)
        elif pygame.K_s in keys_pressed:
            keyboard_operator_cmd[0] = max(keyboard_operator_cmd[0] - vel_step, -lin_vel_x_max)
        else:
            # Decay to zero when not pressed
            if abs(keyboard_operator_cmd[0]) > 0.01:
                keyboard_operator_cmd[0] *= 0.9
            else:
                keyboard_operator_cmd[0] = 0
        
        # Rotation (A/D)
        if pygame.K_a in keys_pressed:
            keyboard_operator_cmd[2] = min(keyboard_operator_cmd[2] + yaw_step, ang_vel_z_max)
        elif pygame.K_d in keys_pressed:
            keyboard_operator_cmd[2] = max(keyboard_operator_cmd[2] - yaw_step, -ang_vel_z_max)
        else:
            # Decay to zero
            if abs(keyboard_operator_cmd[2]) > 0.01:
                keyboard_operator_cmd[2] *= 0.9
            else:
                keyboard_operator_cmd[2] = 0
        
        # Strafe (Q/E)
        if pygame.K_q in keys_pressed:
            keyboard_operator_cmd[1] = min(keyboard_operator_cmd[1] + vel_step, lin_vel_y_max)
        elif pygame.K_e in keys_pressed:
            keyboard_operator_cmd[1] = max(keyboard_operator_cmd[1] - vel_step, -lin_vel_y_max)
        else:
            # Decay to zero
            if abs(keyboard_operator_cmd[1]) > 0.01:
                keyboard_operator_cmd[1] *= 0.9
            else:
                keyboard_operator_cmd[1] = 0
        
        # Round values
        keyboard_operator_cmd = np.round(keyboard_operator_cmd, decimals=2)
        
        # Check if command changed
        if not np.array_equal(old_cmd, keyboard_operator_cmd) or reset:
            updated = True
        
        # Send command
        if updated:
            data = {
                "cmd": keyboard_operator_cmd,
                "reset": reset,
            }
            print(f"📡 Cmd: vel_x={keyboard_operator_cmd[0]:.2f} vel_y={keyboard_operator_cmd[1]:.2f} yaw={keyboard_operator_cmd[2]:.2f}")
            data_publisher.publish(data)
            reset = False
        
        # Draw on-screen display
        screen.fill((30, 30, 30))
        
        texts = [
            f"Forward: {keyboard_operator_cmd[0]:.2f} m/s",
            f"Strafe:  {keyboard_operator_cmd[1]:.2f} m/s",
            f"Rotate:  {keyboard_operator_cmd[2]:.2f} rad/s",
            "",
            "W/S: Forward/Back",
            "A/D: Rotate",
            "Q/E: Strafe",
            "R: Reset | SPACE: Stop"
        ]
        
        for i, text in enumerate(texts):
            color = (0, 255, 0) if i < 3 else (200, 200, 200)
            surface = font.render(text, True, color)
            screen.blit(surface, (20, 20 + i * 30))
        
        pygame.display.flip()
        clock.tick(30)  # 30 FPS
    
    pygame.quit()

def main():
    """Main function"""
    # Select mode
    mode = select_mode()
    
    # Initialize publisher
    data_publisher = DataPublisher(
        'udp://localhost:9871', encoding="msgpack", broadcast=True
    )
    
    # Run selected mode
    try:
        if mode == 1:
            gamepad_mode(data_publisher)
        else:
            keyboard_mode(data_publisher)
    except KeyboardInterrupt:
        print("\n\n👋 Exiting control interface...")
    finally:
        pygame.quit()
        print("✅ Shutdown complete")

if __name__ == "__main__":
    main()