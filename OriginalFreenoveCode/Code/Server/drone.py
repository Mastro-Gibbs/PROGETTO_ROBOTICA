"""
    Programma DRONE : un umano pilota il robot tramite comandi WASD da tastiera remota (via SSH)
"""
from time import sleep
from MyRobot import MyRobot
from pynput import keyboard

SPEED = 2500
FORWARD = False
BACKWaRD = False

def on_press(key):
    global R
    global SPEED
    global FORWARD
    print('\n', R.sensors_reading())
    try:
        if key.char == 'w':
            FORWARD = True
            R.move_forward(SPEED)
        elif key.char == 'a':
            R.turn_left(SPEED)
        elif key.char == 'd':
            R.turn_right(SPEED)
        elif key.char == 's':
            R.move_backward(SPEED)
        elif key.char == '+':
            SPEED += 500
        elif key.char == '-':
            SPEED -= 500
    except AttributeError:
        print(f'tasto speciale: {key}')


def on_release(key):
    print('rel')
    global R
    global FORWARD
    if key.char == 'w':
       FORWARD = False
    if FORWARD:
       key.char = 'w'
       on_press(key)
       sleep(1)
    R.stop_motors()
    print('\n', R.sensors_reading())
    if key == keyboard.Key.esc:
        # Stop listener
        return False


if __name__ == '__main__':
    R = MyRobot()
    R.beep()

    # Colleziona gli eventi di pressione dei tasti
    with keyboard.Listener(
            on_press=on_press,
            on_release=on_release) as listener:
        listener.join()

    # ...or, in a non-blocking fashion:
    listener = keyboard.Listener(
        on_press=on_press,
        on_release=on_release)
    listener.start()
