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
- system: 1
- component: 0
- waiting for initial position and attitude
- got initial position and attitude
> commands
- exit # exit this application
- info (<frame>) # show MAV state
- kill # force disarming of the UAV
- get <param> # get value of Parameter
- arm # arm the UAV
- takeoff # switch into 'Takeoff' mode
- land # switch into 'Land' mode
- hold # switch into 'Hold' mode
- hold # switch into 'Manual' mode
- offboard # switch into 'Offboard' mode
- sp <x> <y> (<z> (<yaw>)) # set a setpoint
- waypoints # move to waypoints
- plan # trajectory with ITG
- traj # print waypoints of trajectory
- map # show map with trajectory
> 
```
## Real UAV
1. Power on the UAV.
2. Start py_gcs. Setpoint gets initialized:
   https://github.com/AImotion-Flight/py_gcs/blob/5a2bddf78d49e462ceaa1717395fe13f09700e3e/mavlink_handler.py#L49
3. Use Remote Control to Takeoff
4. Set a setpoint with ```sp <x> <y> (<z> (<yaw>))```
5. Switch to offboard mode with ```offboard```
6. Plan a trajectory with ```plan```
7. Check the waypoints of the planned trajectory with ```traj```
8. Execute planned trajectory with ```waypoints```
