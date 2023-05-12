import argparse

from terminal import Terminal
from gcs import GCS

parser = argparse.ArgumentParser(
    prog='gcs',
    description='MAVLink GCS for sending Position Setpoints in Offboard-Mode'
)
parser.add_argument('url', type=str, help='udpin:<ip>:<port> or /<path>/<to>/<serial>')
args = parser.parse_args()

gcs = GCS(args)

if __name__ == '__main__':
    gcs.run()