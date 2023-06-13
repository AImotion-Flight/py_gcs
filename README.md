# MAVLink Ground Control Station
## Prerequisites
### Pymavlink
Follow the steps of the [MAVLink documentation](https://mavlink.io/en/mavgen_python/).
## py_gcs
Connect to PX4 SITL:
```bash
python py_gcs.py udpin:127.0.0.1:14540
```
Connect to PX4 over serial:
```bash
python py_gcs.py /path/to/serial
```
## Commands
Enter the command ```commands``` to show all available commands:
```
- MAVLink version: 2.0
- system: 2
- component: 0
- waiting for initial position
- got initial position
> commands
- exit # exit this application
- info # show MAV state
- kill # force disarming of the UAV
- get <param> # get value of Parameter
- arm # arm the UAV
- takeoff # switch into 'Takeoff' mode
- land # switch into 'Land' mode
- hold # switch into 'Hold' mode
- hold # switch into 'Manual' mode
- offboard # switch into 'Offboard' mode
- sp <x> <y> (<z> <yaw>) # set a setpoint
- waypoints # move to waypoints
```