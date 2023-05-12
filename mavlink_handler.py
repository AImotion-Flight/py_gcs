import time
from pymavlink import mavutil

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
        self.connection = mavutil.mavlink_connection(url, dialect='common')
        self.connection.wait_heartbeat()
        self.time_boot = int(time.time() * 100)
        
        terminal.log(f'MAVLink version: {self.connection.WIRE_PROTOCOL_VERSION}')
        terminal.log(f'system: {self.connection.target_system}')
        terminal.log(f'component: {self.connection.target_component}')

        self.sp = None
        while self.sp is None:
            msg = self.connection.recv_msg()
            if msg is not None:
                if msg.get_type() == 'LOCAL_POSITION_NED':
                    self.sp = msg

        self.read_thread = StoppableThread(target=self.read_message)
        self.write_thread = StoppableThread(target=self.write_setpoint)
        self.read_thread.start()
        self.write_thread.start()

    def read_message(self):
        msg = self.connection.recv_msg()

    def write_setpoint(self):
        self.connection.mav.send(self.sp)
        time.sleep(0.25)

    def set_setpoint(self, x, y, z, yaw):
        self.sp = self.connection.mav.set_position_target_local_ned_encode(
            time_boot_ms=int(time.time() * 100) - self.time_boot,
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

    def offboard(self):
        self.set_mode('OFFBOARD')

    def destroy(self):
        self.read_thread.stop()
        self.write_thread.stop()
        self.read_thread.join()
        self.write_thread.join()