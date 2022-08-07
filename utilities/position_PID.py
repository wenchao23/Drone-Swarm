from utilities.rotateGFtoBF import rotateGFtoBF

def position_PID(Quad):

    if Quad.init == 0:
        Quad.x_error_sum = 0
        Quad.y_error_sum = 0

    # # High Level Position Controller

    # Measurement Model
    # if (Quad.ground_truth)
        # x = Quad.X
    # y = Quad.Y
    # z = Quad.Z
    # phi = Quad.phi
    # theta = Quad.theta
    # psi = Quad.psi
    # end
    #
    # if (Quad.sensor_unfiltered)
        # x = Quad.X_meas
    # y = Quad.Y_meas
    # z = Quad.Z_meas
    # phi = Quad.phi_meas
    # theta = Quad.theta_meas
    # psi = Quad.psi_meas
    # end
    #
    # if (Quad.sensor_kf)
        # x = Quad.X
    # y = Quad.Y
    # z = Quad.Z
    # phi = Quad.phi
    # theta = Quad.theta
    # psi = Quad.psi
    # end

    x = Quad.X
    y = Quad.Y
    z = Quad.Z
    phi = Quad.phi
    theta = Quad.theta
    psi = Quad.psi

    # Rotate Desired Position from GF toBF(Zaxis rotation only)

    temp = rotateGFtoBF(Quad.X_des_GF, Quad.Y_des_GF, Quad.Z_des_GF, 0 * phi, 0 * theta,psi)
    Quad.X_des = temp[0]
    Quad.Y_des = temp[1]
    Quad.Z_des = temp[2]
    # Rotate Current Position from GF to BF
    temp = rotateGFtoBF(x, y, z, phi, theta, psi)
    Quad.X_BF = temp[0]
    Quad.Y_BF = temp[1]
    Quad.Z_BF = temp[2]
    # Rotate Current Velocity from GF to BF
    temp = rotateGFtoBF(Quad.X_dot, Quad.Y_dot, Quad.Z_dot, phi, theta, psi)
    Quad.X_BF_dot = temp[0]
    Quad.Y_BF_dot = temp[1]
    Quad.Z_BF_dot = temp[2]
    # X Position PID controller
    x_error = Quad.X_des - Quad.X_BF
    if abs(x_error) < Quad.X_KI_lim:
        Quad.x_error_sum = Quad.x_error_sum + x_error

    cp = Quad.X_KP * x_error # Proportional term
    ci = Quad.X_KI * Quad.Ts * Quad.x_error_sum
    ci = min(Quad.theta_max, max(-Quad.theta_max, ci)) # Saturate ci
    cd = Quad.X_KD * Quad.X_BF_dot # Derivative  term
    Quad.theta_des = - (cp + ci + cd) # Theta and X inversely related
    Quad.theta_des = min(Quad.theta_max, max(-Quad.theta_max, Quad.theta_des))

    # Y Position PID controller
    y_error = Quad.Y_des - Quad.Y_BF
    if abs(y_error) < Quad.Y_KI_lim:
        Quad.y_error_sum = Quad.y_error_sum + y_error

    cp = Quad.Y_KP * y_error # Proportional term
    ci = Quad.Y_KI * Quad.Ts * Quad.y_error_sum
    ci = min(Quad.phi_max, max(-Quad.phi_max, ci)) # Saturate ci
    cd = Quad.Y_KD * Quad.Y_BF_dot # Derivative term
    Quad.phi_des = cp + ci + cd
    Quad.phi_des = min(Quad.phi_max, max(-Quad.phi_max, Quad.phi_des))

    return Quad
