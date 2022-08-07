from numpy import random
from numpy import mod


def sensor_meas(Quad):

    if mod(Quad.counter, Quad.GPS_freq) == 0:
        Quad.X = Quad.X + random.normal(0, 1) * Quad.X_error
        Quad.Y = Quad.Y + random.normal(0, 1) * Quad.Y_error
        Quad.Z = Quad.Z + random.normal(0, 1) * Quad.Z_error

    Quad.X_ddot = Quad.X_ddot + Quad.x_acc_bias + Quad.x_acc_sd * random.normal(0,1)
    Quad.Y_ddot = Quad.Y_ddot + Quad.y_acc_bias + Quad.y_acc_sd * random.normal(0,1)
    Quad.Z_ddot = Quad.Z_ddot + Quad.z_acc_bias + Quad.z_acc_sd * random.normal(0,1)

    Quad.p = Quad.p + Quad.x_gyro_bias + Quad.x_gyro_sd * random.normal(0,1)
    Quad.q = Quad.q + Quad.y_gyro_bias + Quad.y_gyro_sd * random.normal(0,1)
    Quad.r = Quad.r + Quad.z_gyro_bias + Quad.z_gyro_sd * random.normal(0,1)

    # Quad.X_ddot_meas_plot[Quad.counter] = Quad.X_ddot
    # Quad.Y_ddot_meas_plot[Quad.counter] = Quad.Y_ddot
    # Quad.Z_ddot_meas_plot[Quad.counter] = Quad.Z_ddot

    return Quad
