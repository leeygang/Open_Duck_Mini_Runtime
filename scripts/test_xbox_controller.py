import pygame

pygame.init()
_p1 = pygame.joystick.Joystick(0)
_p1.init()
print(f"Loaded joystick with {_p1.get_numaxes()} axes.")


while True:
    for event in pygame.event.get():
        print(_p1.get_axis(0), _p1.get_axis(1), _p1.get_axis(2), _p1.get_axis(3))

    pygame.event.pump()  # process event queue
