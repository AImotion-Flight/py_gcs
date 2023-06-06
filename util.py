import numpy as np

def generate_states_vector(map_width, map_height, vels):
    states = np.empty((0, 4), dtype='int')
    for i in range(map_width):
        for h in range(map_height):
            for _, j in enumerate(vels):
                for _, k in enumerate(vels):
                    states = np.vstack((states, (i, h, j, k)))
    return states

def generate_actions_vector(actions):
    return np.array(np.meshgrid(actions, actions)).T.reshape(-1, 2)

def liang_barsky(left, right, top, bottom, x0, y0, x1, y1):
    p = np.array([-(x1 - x0), x1 - x0, -(y1 - y0), y1- y0])
    q = np.array([x0 - left, right - x0, y0 - bottom, top - y0])
    qp = q / p

    if (p[0] == 0 and q[0] < 0) or (p[1] == 0 and q[1] < 0) or (p[2] == 0 and q[2] < 0) or (p[3] == 0 and q[3] < 0):
        return False

    tmin = 0
    tmax = 1

    for k in range(4):
        if p[k] < 0:
            if qp[k] > tmin:
                tmin = qp[k]
        elif p[k] > 0:
            if qp[k] < tmax:
                tmax = qp[k]

    return tmax > tmin