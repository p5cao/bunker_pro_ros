#!/usr/bin/env python3

import pygame
from websocket import create_connection

def rescale(oldValue, newMin=0, newMax=255):
    oldMax = 1
    oldMin = -1

    oldRange = (oldMax - oldMin)
    newRange = (newMax - newMin)
    newValue = (((oldValue - oldMin) * newRange) / oldRange) + newMin
    return int(newValue)

#helper function to change joystick range of [0,1] to [0,255]
def convertServo(axis):
    data = joystick.get_axis(abs(axis))
    if axis < 0: data = -data
    return rescale(data, 30, 150 )

def convertThrust(data):
    return abs(rescale(data, -255, 255))

#helper function to assign the proper hbridge values
def convertHBridge(data):
    if data > 0:
        return [1,0]
    else:
        return [0,1]

def convertAxis(axis):
    data = joystick.get_axis(axis)*-1
    return convertHBridge(data) + [ convertThrust(data) ]


if __name__ == "__main__":
    target = "ws://192.168.4.1:81"
    #Set up connection to Blimp Board over websocket
    ws = create_connection(target)

    # Set up joystick
    pygame.init()
    pygame.joystick.init()
    done = False

    # Joystick axes -- windows
    axes_win = dict(
        forward = 1,
        vertical = 3,
        turn = -2,
        camera_r = 5,
        camera_l = 4,
    )

    # Joystick axes -- linux
    axes_lin = dict(
        forward = 1,
        vertical = 4,
        turn = -3,
        camera_r = 5,
        camera_l = 2,
    )

    allaxes = [axes_win, axes_lin]
    arch = 0
    axes = allaxes[arch]

    debounce = 0

    while not done:
        try:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    done = True

            joystick = pygame.joystick.Joystick(0)
            joystick.init()

            # Swap axes 3 and 4 on 'A' button push [ windowns vs linux ]
            if joystick.get_button(0) and not debounce:
                arch = 1-arch
                axes = allaxes[arch]
                debounce = 10
            if debounce: debounce -= 1

            #Create a new binary code to send to blimp
            cmd = [126] + convertAxis(axes["forward"]) + \
                          [ convertServo(axes["turn"]) ] + \
                          convertAxis(axes["vertical"]) + \
                          [ (180 - convertServo(axes["camera_l"]) + convertServo(axes["camera_r"])) // 2 ]

            print(cmd)
            ws.send_binary(bytes(cmd))

            pygame.time.wait(100)

        except KeyboardInterrupt:
            done = True

    ws.close()
    pygame.quit()
