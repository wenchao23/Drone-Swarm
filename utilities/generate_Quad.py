from utilities import quad_variables,quad_dynamics_nonlinear

class Quad_class:
    pass
def generate_Quad(position_desire,position_init):
    Quad = Quad_class()
    Quad.X_des_GF = position_desire[0]
    Quad.Y_des_GF = position_desire[1]
    Quad.Z_des_GF = position_desire[2]

    Quad = quad_variables.quad_variables(Quad, position_desire, position_init)
    Quad = quad_dynamics_nonlinear.quad_dynamics_nonlinear(Quad)
    Quad.X = position_init[0]
    Quad.Y = position_init[1]
    Quad.Z = position_init[2]
    Quad.X_history = [Quad.X]
    Quad.Y_history = [Quad.Y]
    Quad.Z_history = [Quad.Z]
    Quad.CovX = [.1]
    Quad.CovY = [.1]
    Quad.CovZ = [.1]
    Quad.Cov = [.1]
    Quad.X_State = [.1]
    Quad.Y_State = [.1]
    Quad.Z_State = [.1]    
    Quad.Stuck = 0
    Quad.Stuck_flag = 0;
    return Quad