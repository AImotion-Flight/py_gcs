import time
from pymavlink import mavutil
import numpy as np

from terminal import terminal
from stoppable_thread import StoppableThread

POSITION_ONLY_TYPEMASK = (
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
)

class MAVLinkHandler:
    def __init__(self, url):
        self.connection = mavutil.mavlink_connection(url, baud=57600, dialect='common', source_system=254)
        self.connection.wait_heartbeat()
        
        terminal.log(f'MAVLink version: {self.connection.WIRE_PROTOCOL_VERSION}')
        terminal.log(f'system: {self.connection.target_system}')
        terminal.log(f'component: {self.connection.target_component}')

        #self.set_setpoint(0, 0, 0, 0)
        
        terminal.log('waiting for initial position')
        self.sp = None
        while self.sp is None:
            msg = self.connection.recv_msg()
            if msg is not None:
                if msg.get_type() == 'LOCAL_POSITION_NED':
                    self.sp = msg
        terminal.log('got initial position')

        self.read_thread = StoppableThread(target=self.read_message)
        self.write_thread = StoppableThread(target=self.write_setpoint)
        self.read_thread.start()
        self.write_thread.start()

        self.connection.param_fetch_all()

    def get_autopilot(self):
        heartbeat = self.connection.messages['HEARTBEAT']
        if heartbeat.autopilot == mavutil.mavlink.MAV_AUTOPILOT_PX4:
            return 'MAV_AUTOPILOT_PX4'

        return 'MAV_AUTOPILOT_GENERIC'

    def get_param(self, param):
        return self.connection.param_state[(self.connection.target_system, 1)].params[param]

    def set_param(self, param, value):
        self.connection.param_set_send(param, value)

    def read_message(self):
        msg = self.connection.recv_msg()
        if msg is not None:
            if msg.get_type() == 'HEARTBEAT':
                pass

    def write_setpoint(self):
        self.connection.mav.send(self.sp)
        time.sleep(0.1)

    def get_local_position_ned(self):
        msg = self.connection.messages['LOCAL_POSITION_NED']
        return np.array((msg.x, msg.y, msg.z))
    
    def get_setpoint(self):
        return np.array((self.sp.x, self.sp.y, self.sp.z))

    def set_setpoint(self, x, y, z, yaw):
        self.sp = self.connection.mav.set_position_target_local_ned_encode(
            time_boot_ms=int(self.connection.timestamp),
            target_system=self.connection.target_system,
            target_component=self.connection.target_component,
            coordinate_frame=mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            type_mask=POSITION_ONLY_TYPEMASK,
            x=y,
            y=x,
            z=-z,
            vx=0.0,
            vy=0.0,
            vz=0.0,
            afx=0.0,
            afy=0.0,
            afz=0.0,
            yaw=yaw,
            yaw_rate=0.0
        )

    def command_long(self, command, param1, param2, param3, param4, param5, param6, param7):
        self.connection.mav.command_long_send(
            target_system=self.connection.target_system,
            target_component=self.connection.target_component,
            command=command,
            confirmation=0, 
            param1=param1,
            param2=param2,
            param3=param3,
            param4=param4,
            param5=param5,
            param6=param6,
            param7=param7
        )

    def get_mode(self):
        heartbeat = self.connection.messages['HEARTBEAT']
        return mavutil.interpret_px4_mode(heartbeat.base_mode, heartbeat.custom_mode)

    def set_mode(self, mode):
        self.connection.set_mode(mode)

    def arm(self):
        self.command_long(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            param1=1,
            param2=0,
            param3=0,
            param4=0,
            param5=0,
            param6=0,
            param7=0
        )

    def disarm(self):
        self.command_long(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            param1=0,
            param2=0,
            param3=0,
            param4=0,
            param5=0,
            param6=0,
            param7=0
        )

    def kill(self):
        self.command_long(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            param1=0,
            param2=21196,
            param3=0,
            param4=0,
            param5=0,
            param6=0,
            param7=0
        )

    def takeoff(self):
        self.set_mode('TAKEOFF')

    def land(self):
        self.set_mode('LAND')

    def hold(self):
        self.connection.set_mode_loiter()

    def manual(self):
        self.connection.set_mode_manual()

    def offboard(self):
        self.set_mode('OFFBOARD')

    def destroy(self):
        self.read_thread.stop()
        self.write_thread.stop()
        self.read_thread.join()
        self.write_thread.join()