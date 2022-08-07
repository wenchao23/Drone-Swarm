from numpy import sin,cos

def attitude_PID(Quad):

    # initialize persistent variables at beginning of simulation
    if Quad.init == 0:
        Quad.z_error_sum = 0
        Quad.phi_error_sum = 0
        Quad.theta_error_sum = 0
        Quad.psi_error_sum = 0


    # # Measurement Model
    # if(Quad.ground_truth)
    #     phi = Quad.phi
    #     theta = Quad.theta
    #     psi = Quad.psi
    # end
    #
    # if(Quad.sensor_unfiltered)
    #     phi = Quad.phi_meas
    #     theta = Quad.theta_meas
    #     psi = Quad.psi_meas
    # end
    #
    # if(Quad.sensor_kf)
    #     phi = Quad.phi
    #     theta = Quad.theta
    #     psi = Quad.psi
    # end

    phi = Quad.phi
    theta = Quad.theta
    psi = Quad.psi


    ## Z Position PID Controller/Altitude Controller
    z_error = Quad.Z_des_GF-Quad.Z
    if abs(z_error) < Quad.Z_KI_lim:
        Quad.z_error_sum = Quad.z_error_sum + z_error

    cp = Quad.Z_KP*z_error         #Proportional term
    ci = Quad.Z_KI*Quad.Ts*Quad.z_error_sum #Integral term
    ci = min(Quad.U1_max, max(Quad.U1_min, ci))    #Saturate ci
    cd = Quad.Z_KD*Quad.Z_dot                  #Derivative term
    Quad.U1 = -(cp + ci + cd)/(cos(theta)*cos(phi)) + (Quad.m * Quad.g)/(cos(theta)*cos(phi))   #Negative since Thurst and Z inversely related
    Quad.U1 = min(Quad.U1_max, max(Quad.U1_min, Quad.U1))


    ## Attitude Controller

    # Roll PID Controller
    phi_error = Quad.phi_des - phi
    if abs(phi_error) < Quad.phi_KI_lim:
        Quad.phi_error_sum = Quad.phi_error_sum + phi_error

    cp = Quad.phi_KP*phi_error
    ci = Quad.phi_KI*Quad.Ts*Quad.phi_error_sum
    ci = min(Quad.p_max, max(-Quad.p_max, ci))
    cd = Quad.phi_KD*Quad.p
    Quad.p_des = cp + ci + cd
    Quad.p_des = min(Quad.p_max, max(-Quad.p_max, Quad.p_des))

    # Pitch PID Controller
    theta_error = Quad.theta_des - theta
    if abs(theta_error) < Quad.theta_KI_lim:
        Quad.theta_error_sum = Quad.theta_error_sum + theta_error

    cp = Quad.theta_KP*theta_error
    ci = Quad.theta_KI*Quad.Ts*Quad.theta_error_sum
    ci = min(Quad.q_max, max(-Quad.q_max, ci))
    cd = Quad.theta_KD*Quad.q
    Quad.q_des = cp + ci + cd
    Quad.q_des = min(Quad.q_max, max(-Quad.q_max, Quad.q_des))


    # Yaw PID Controller
    psi_error = Quad.psi_des - psi
    if abs(psi_error) < Quad.psi_KI_lim:
        Quad.psi_error_sum = Quad.psi_error_sum + psi_error

    cp = Quad.psi_KP*psi_error
    ci = Quad.psi_KI*Quad.Ts*Quad.psi_error_sum
    ci = min(Quad.r_max, max(-Quad.r_max, ci))
    cd = Quad.psi_KD*Quad.r
    Quad.r_des = cp + ci + cd
    Quad.r_des = min(Quad.r_max, max(-Quad.r_max, Quad.r_des))

    return Quad
