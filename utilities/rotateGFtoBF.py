import numpy as np

def rotateGFtoBF(X, Y, Z, phi, theta, psi):

    R_roll = np.array([[1, 0, 0],
               [0, np.cos(phi), -np.sin(phi)],
                 [0, np.sin(phi), np.cos(phi)]])

    R_pitch = np.array([[np.cos(theta), 0, np.sin(theta)],
                [ 0, 1, 0],
                [- np.sin(theta), 0, np.cos(theta)]])
    R_yaw = [[np.cos(psi), -np.sin(psi), 0],
            [np.sin(psi), np.cos(psi), 0],
            [ 0, 0, 1]]

    R = np.linalg.multi_dot([R_yaw , R_pitch , R_roll])
    temp = np.array([X, Y, Z])
    pts = np.dot(temp, R)

    X = pts[0]
    Y = pts[1]
    Z = pts[2]

    return np.array([X, Y, Z])
