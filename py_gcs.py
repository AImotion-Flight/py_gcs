import argparse
import numpy as np
import matplotlib.pyplot as plt
import os
import time

from terminal import terminal
from mavlink_handler import MAVLinkHandler
from itg.qlearning import QLearning
from itg.environment import GridEnvironment
from itg.agent import DynamicalSystem
from itg.util import generate_actions_vector, generate_states_vector
import params

command_help = {
    'exit': 'exit # exit this application',
    'info': 'info (<frame>) # show MAV state',
    'kill': 'kill # force disarming of the UAV',
    'get': 'get <param> # get value of Parameter',
    'arm': 'arm # arm the UAV',
    'takeoff': 'takeoff # switch into \'Takeoff\' mode',
    'land': 'land # switch into \'Land\' mode',
    'hold': 'hold # switch into \'Hold\' mode',
    'manual': 'hold # switch into \'Manual\' mode',
    'offboard': 'offboard # switch into \'Offboard\' mode',
    'sp': 'sp <x> <y> (<z> (<yaw>)) # set a setpoint',
    'waypoints': 'waypoints # move to waypoints',
    'plan': 'plan # trajectory with ITG',
    'traj': 'traj # print waypoints of trajectory',
    'map': 'map # show map with trajectory'

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
            (0, -0),
            (0, -5),
            (5, -5),
            (5, -0),
            (0, -0)
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

                if len(input) == 0:
                    continue

                command = input[0]
                args = input[1:]

                if command == 'commands':
                    for help in command_help:
                        terminal.log(command_help[help])
                elif command == 'info':
                    frame = 'frd'

                    if len(args) == 1:
                        frame = args[0]

                    terminal.log(f'Autopilot: {px4.get_autopilot()}')
                    terminal.log(f'Mode: {px4.get_mode()}')

                    if frame == 'frd':
                        terminal.log(f'Local Position (FRD): {px4.get_local_position_frd()}')
                        terminal.log(f'Attitude (FRD): {px4.get_attitude_frd()}')
                        terminal.log(f'Current Setpoint (FRD): {px4.get_setpoint_frd()}')
                    elif frame == 'ned':
                        terminal.log(f'Local Position (NED): {px4.get_local_position_ned()}')
                        terminal.log(f'Attitude (NED): {px4.get_attitude_ned()}')
                        terminal.log(f'Current Setpoint (NED): {px4.get_setpoint_ned()}')
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
                    z = params.DEFAULT_ALTITUDE
                    yaw = params.DEFAULT_YAW

                    if len(args) == 3:
                        z = float(args[2])
                    elif len(args) == 4:
                        z = float(args[2])
                        yaw = float(args[3])
                  
                    px4.set_setpoint(
                        pos=np.array((x, y, z)),
                        yaw=yaw
                    )
                elif command == 'waypoints':
                    for w in self.waypoints:
                        px4.set_setpoint(
                            pos=np.array((w[0], w[1], params.DEFAULT_ALTITUDE)),
                            yaw=params.DEFAULT_YAW
                        )
                        time.sleep(2)
                    px4.land()                        
                elif command == 'plan':
                    plot = False

                    if len(args) == 1:
                        plot == True if args[0] == 't' else False

                    env = GridEnvironment(self.grid)
                    size = np.shape(self.grid)
                    vstates = generate_states_vector(size[1], size[0], range(self.args.lower_vel, self.args.upper_vel + 1))
                    vactions = generate_actions_vector(range(self.args.lower_accel, self.args.upper_accel + 1))
                    agent = DynamicalSystem(vstates, vactions, self.args.lower_vel, self.args.upper_vel)
                    qlearning = QLearning((0, 0, 0, 0), (9, 9, 0, 0), agent, env, 50000, 0.9, 0.9, 0.1, plot)

                    if os.path.exists('Q.npy'):
                        qlearning.load('Q.npy')
                    else:
                        qlearning.learn(terminal)
                        qlearning.save('Q.npy')

                    policy = qlearning.get_policy()
                    self.waypoints = policy[0][:, 0:2]
                    self.waypoints[:, 1] = -self.waypoints[:, 1]
                elif command == 'traj':
                    terminal.log(self.waypoints)
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
                    ax.plot(self.waypoints[:, 0], -self.waypoints[:, 1], 'b--')
                    ax.plot(self.waypoints[-1, 0], -self.waypoints[-1, 1], 'rx')
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
parser.add_argument('-wt', '--waypoint_thresh', type=float, help='Distance from waypoint to consider it reached.', default=2.0)
parser.add_argument('-lv', '--lower_vel', type=int, help='Lower bound for velocity.', default=-2)
parser.add_argument('-uv', '--upper_vel', type=int, help='Upper bound for velocity.', default=2)
parser.add_argument('-la', '--lower_accel', type=int, help='Lower bound for acceleration .', default=-1)
parser.add_argument('-ua', '--upper_accel', type=int, help='Upper bound for acceleration .', default=1)
args = parser.parse_args()

gcs = GCS(args)

if __name__ == '__main__':
    gcs.run()