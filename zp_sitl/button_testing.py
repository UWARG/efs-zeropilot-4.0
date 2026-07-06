import pygame

# Initialize Pygame and the joystick system
pygame.init()
pygame.joystick.init()

# Check if the RC controller is detected
if pygame.joystick.get_count() == 0:
    print("No controller found! Check your USB/dongle connection.")
else:
    # Connect to the first available controller
    rc_controller = pygame.joystick.Joystick(0)
    rc_controller.init()
    print(f"Connected to: {rc_controller.get_name()}")
    print("Press controller buttons or flip switches to see their numbers...")

running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
            
        # Detect when a button or toggle switch is activated
        elif event.type == pygame.JOYBUTTONDOWN:
            print(f"Button {event.button} status: PRESSED")
            
        # Detect when a button or toggle switch is deactivated
        elif event.type == pygame.JOYBUTTONUP:
            print(f"Button {event.button} status: RELEASED")
            
        # RC switches are sometimes treated as "Hats" (D-pads) or Analog Axes
        elif event.type == pygame.JOYHATMOTION:
            print(f"Hat {event.hat} moved to position: {event.value}")
            
        elif event.type == pygame.JOYAXISMOTION:
            # Only print significant movements to avoid analog stick "jitter"
            if abs(event.value) > 0.1:
                print(f"Axis {event.axis} value: {event.value:.2f}")

pygame.quit()
