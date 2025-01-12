import pygame

pygame.init()
_p1 = pygame.joystick.Joystick(0)
_p1.init()
print(f"Loaded joystick with {_p1.get_numaxes()} axes.")


while True:
    for event in pygame.event.get():
        for i in range(_p1.get_numaxes()):
            print(i, _p1.get_axis(i))
        print("===")

    pygame.event.pump()  # process event queue
