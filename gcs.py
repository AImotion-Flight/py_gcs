import math

from terminal import terminal
from mavlink_handler import MAVLinkHandler

import time

DEFAULT_YAW = (1 / 2) * math.pi
DEFAULT_ALTITUDE = 1.0

command_help = {
    'exit': 'exit # exit this application',
    'kill': 'kill # force disarming of the UAV',
    'arm': 'arm # arm the UAV',
    'takeoff': 'takeoff # switch into \'Takeoff\' mode',
    'land': 'land # switch into \'Land\' mode',
    'hold': 'hold # switch into \'Hold\' mode',
    'offboard': 'offboard # switch into \'Offboard\' mode',
    'sp': 'sp <x> <y> (<z> <yaw>) # set a setpoint',
    'waypoints': 'waypoints # move to waypoints'
}

waypoints = [
    (0, 9),
    (2, 9),
    (2, 6),
    (6, 6),
    (6, 0),
    (9, 0),
    (9, 9)
]

class GCS:
    def __init__(self, args):
        self.args = args

    def argument_error(self, command, expected, given):
        terminal.log(f'{command} command expects {expected} arguments, {given} were given')
        terminal.log(command_help[command])

    def run(self):
        px4 = MAVLinkHandler(self.args.url)

        try:
            while True:
                input = terminal.input().split()
                command = input[0]
                args = input[1:]

                if command == 'commands':
                    for help in command_help:
                        terminal.log(command_help[help])
                elif command == 'exit':
                    raise KeyboardInterrupt
                elif command == 'kill':
                    px4.kill()
                elif command == 'arm':
                    px4.arm()
                elif command == 'disarm':
                    px4.disarm()
                elif command == 'takeoff':
                    px4.takeoff()
                elif command == 'land':
                    px4.land()
                elif command == 'hold':
                    px4.hold()
                elif command == 'offboard':
                    px4.offboard()
                elif command == 'sp':
                    if len(args) < 2:
                        self.argument_error(command, 2, len(args))
                        continue
                    
                    x = float(args[0])
                    y = float(args[1])
                    z = DEFAULT_ALTITUDE
                    yaw = DEFAULT_YAW

                    if len(args) == 3:
                        z = float(args[2])
                    elif len(args) == 4:
                        yaw = float(args[3])
                  
                    px4.set_setpoint(
                        x=x,
                        y=y,
                        z=z,
                        yaw=yaw
                    )
                elif command == 'waypoints':
                    px4.set_setpoint(
                            x=0.0,
                            y=0.0,
                            z=DEFAULT_ALTITUDE,
                            yaw=DEFAULT_YAW
                        )

                    for w in waypoints:
                        px4.set_setpoint(
                            x=w[0],
                            y=w[1],
                            z=DEFAULT_ALTITUDE,
                            yaw=DEFAULT_YAW
                        )
                        time.sleep(3)

                else:
                    terminal.log('unknown command')

        except KeyboardInterrupt:
            px4.destroy()
            print('')