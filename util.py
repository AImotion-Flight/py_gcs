import math

def transform_yaw(yaw, rot):
    yaw = yaw + math.pi
    yaw_trans = (yaw + rot) % (2 * math.pi)
    return yaw_trans - math.pi