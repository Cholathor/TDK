import pygame
import sys
import serial


ser = serial.Serial('COM4', 115200)
enableStim = False



# Initialize pygame
pygame.init()

# Set up the display
width, height = 800, 600
screen = pygame.display.set_mode((width, height))
pygame.display.set_caption("Red Ball Game")

# Set up colors
red = (255, 0, 0)

# Set up the red ball
ball_radius = 20
ball_x, ball_y = width // 2, height // 2
ball_speed = 5

clock = pygame.time.Clock()

running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Get the mouse position
    mouse_x, mouse_y = pygame.mouse.get_pos()

    if mouse_x > 600 and not enableStim:
        ser.write('ENTER'.encode())
        enableStim = True
    elif mouse_x < 600 and enableStim:
        ser.write('ENTER'.encode())
        enableStim = False

    # Clear the screen
    screen.fill((0, 0, 0))

    # Draw the red ball
    pygame.draw.circle(screen, red, (int(mouse_x), int(mouse_y)), ball_radius)

    # Update the display
    pygame.display.flip()

    # Limit the frame rate
    clock.tick(60)

# Quit pygame
pygame.quit()
sys.exit()
