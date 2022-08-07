def rate_PID(Quad):
    if Quad.init == 0:
        Quad.p_error_sum = 0
        Quad.q_error_sum = 0
        Quad.r_error_sum = 0
        Quad.init = 1
    p = Quad.p
    q = Quad.q
    r = Quad.r
    
    p_error = Quad.p_des - p
    if abs(p_error) < Quad.p_KI_lim:
        Quad.p_error_sum = Quad.p_error_sum + p_error

    cp = Quad.p_KP*p_error
    ci = Quad.p_KI*Quad.Ts*Quad.p_error_sum
    ci = min(Quad.U2_max, max(Quad.U2_min, ci))
    cd = Quad.p_KD*Quad.p_dot
    Quad.U2 = cp + ci + cd
    Quad.U2 = min(Quad.U2_max, max(Quad.U2_min, Quad.U2))

    # Pitch PID Controller
    q_error = Quad.q_des - q
    if abs(q_error) < Quad.q_KI_lim:
        Quad.q_error_sum = Quad.q_error_sum + q_error

    cp = Quad.q_KP*q_error
    ci = Quad.q_KI*Quad.Ts*Quad.q_error_sum
    ci = min(Quad.U3_max, max(Quad.U3_min, ci))
    cd = Quad.q_KD*Quad.q_dot
    Quad.U3 = cp + ci + cd
    Quad.U3 = min(Quad.U3_max, max(Quad.U3_min, Quad.U3))

    # Yaw PID Controller
    r_error = Quad.r_des - r
    if abs(r_error) < Quad.r_KI_lim:
        Quad.r_error_sum = Quad.r_error_sum + r_error

    cp = Quad.r_KP*r_error
    ci = Quad.r_KI*Quad.Ts*Quad.r_error_sum
    ci = min(Quad.U4_max, max(Quad.U4_min, ci))
    cd = Quad.r_KD*Quad.r_dot
    Quad.U4 = cp + ci + cd
    Quad.U4 = min(Quad.U4_max, max(Quad.U4_min, Quad.U4))
    return Quad