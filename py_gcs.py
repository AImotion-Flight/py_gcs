import math
import argparse
import numpy as np
import matplotlib.pyplot as plt
import os

from terminal import terminal
from mavlink_handler import MAVLinkHandler
from qlearning import QLearning
from environment import GridEnvironment
from agent import DynamicalSystem
from util import generate_actions_vector, generate_states_vector

DEFAULT_YAW = (1 / 2) * math.pi
DEFAULT_ALTITUDE = 1.0

command_help = {
    'exit': 'exit # exit this application',
    'info': 'info # show MAV state',
    'kill': 'kill # force disarming of the UAV',
    'get': 'get <param> # get value of Parameter',
    'arm': 'arm # arm the UAV',
    'takeoff': 'takeoff # switch into \'Takeoff\' mode',
    'land': 'land # switch into \'Land\' mode',
    'hold': 'hold # switch into \'Hold\' mode',
    'manual': 'hold # switch into \'Manual\' mode',
    'offboard': 'offboard # switch into \'Offboard\' mode',
    'sp': 'sp <x> <y> (<z> <yaw>) # set a setpoint',
    'waypoints': 'waypoints # move to waypoints'
}

class GCS:
    def __init__(self, args):
        self.args = args
        self.grid = np.ones((10, 10))
        self.grid[0:8, 1] = 0
        self.grid[7:, 4] = 0
        self.grid[0:5, 4] = 0
        self.grid[2:, 7] = 0
        self.waypoints = np.array([
            (0, 0),
            (0, 9),
            (2, 9),
            (2, 6),
            (6, 6),
            (6, 0),
            (9, 0),
            (9, 9)
        ])

    def check_arguments(self, command, args, condition):
        if condition(len(args)):
            terminal.log(f'wrong arguments for command {command}')
            terminal.log(command_help[command])
            return False

        return True
    
    def waypoint_reached(self, waypoint, pos):
        dist = waypoint - pos
        return np.linalg.norm(dist) < self.args.waypoint_thresh

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
                elif command == 'info':
                    terminal.log(f'Autopilot: {px4.get_autopilot()}')
                    terminal.log(f'Mode: {px4.get_mode()}')
                    terminal.log(f'Local Position: {px4.get_local_position_ned()}')
                elif command == 'exit':
                    raise KeyboardInterrupt
                elif command == 'kill':
                    px4.kill()
                elif command == 'get':
                    if not self.check_arguments(command, args, lambda x: x != 1):
                        continue
                    try:
                        terminal.log(px4.get_param(args[0]))
                    except KeyError:
                        terminal.log(f'parameter {args[0]} not found')
                elif command == 'set':
                    if not self.check_arguments(command, args, lambda x: x != 2):
                        continue
                    px4.set_param(args[0], float(args[1]))
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
                elif command == 'manual':
                    px4.manual()
                elif command == 'offboard':
                    px4.offboard()
                elif command == 'sp':
                    if not self.check_arguments(command, args, lambda x: x < 2):
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
                    for w in self.waypoints:
                        px4.set_setpoint(
                            x=w[0],
                            y=w[1],
                            z=DEFAULT_ALTITUDE,
                            yaw=DEFAULT_YAW
                        )
                        while not self.waypoint_reached(px4.get_setpoint(), px4.get_local_position_ned()):
                            continue
                elif command == 'plan':
                    env = GridEnvironment(self.grid)
                    size = np.shape(self.grid)
                    vstates = generate_states_vector(size[1], size[0], range(self.args.lower_vel, self.args.upper_vel + 1))
                    vactions = generate_actions_vector(range(self.args.lower_accel, self.args.upper_accel + 1))
                    agent = DynamicalSystem(vstates, vactions, self.args.lower_vel, self.args.upper_vel)
                    qlearning = QLearning((0, 0, 0, 0), (9, 9, 0, 0), agent, env, 50000, 0.9, 0.9, 0.1, False)

                    if os.path.exists('Q.npy'):
                        qlearning.load('Q.npy')
                    else:
                        qlearning.learn(terminal)
                        qlearning.save('Q.npy')

                    policy = qlearning.get_policy()
                    self.waypoints = policy[0][:, 0:3]
                elif command == 'map':
                    size = np.shape(self.grid)
                    fig, ax = plt.subplots()
                    fig.canvas.manager.set_window_title('Occupancy Grid Map')
                    fig.set_size_inches((10, 10))
        
                    ax.set_xlabel('x [m]')
                    ax.set_ylabel('y [m]')
        
                    ax.set_xticks(np.arange(0, size[0], 1))
                    ax.set_yticks(np.arange(0, size[1], 1))
        
                    ax.set_xticks(np.arange(-0.5, size[0], 1), minor=True)
                    ax.set_yticks(np.arange(-0.5, size[1], 1), minor=True)

                    ax.tick_params(which='minor', bottom=False, left=False)

                    ax.grid(which='minor')
                    ax.imshow(self.grid, cmap='gray', origin='lower')
                    ax.plot(0, 0, 'go')
                    ax.plot(9, 9, 'r*')
                    ax.plot(self.waypoints[:, 0], self.waypoints[:, 1], 'b--')
                    ax.plot(self.waypoints[-1, 0], self.waypoints[-1, 1], 'rx')
                    fig.show()
                else:
                    terminal.log('unknown command')

        except KeyboardInterrupt:
            px4.destroy()
            print('')

parser = argparse.ArgumentParser(
    prog='py_gcs',
    description='Python MAVLink GCS for sending Position Setpoints in Offboard-Mode'
)
parser.add_argument('url', type=str, help='udpin:<ip>:<port> or /<path>/<to>/<serial>')
parser.add_argument('-wt', '--waypoint_thresh', type=float, help='Distance from waypoint to consider it reached.', default=0.5)
parser.add_argument('-lv', '--lower_vel', type=int, help='Lower bound for velocity.', default=-2)
parser.add_argument('-uv', '--upper_vel', type=int, help='Upper bound for velocity.', default=2)
parser.add_argument('-la', '--lower_accel', type=int, help='Lower bound for acceleration .', default=-1)
parser.add_argument('-ua', '--upper_accel', type=int, help='Upper bound for acceleration .', default=1)
args = parser.parse_args()

gcs = GCS(args)

if __name__ == '__main__':
    gcs.run()