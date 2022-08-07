# This is the main function for the simulation of drone swam and collision avoidance.
# Verion 1.0

import matplotlib; matplotlib.use("TkAgg")
import numpy as np
import matplotlib .pyplot as plt
import os
from utilities import plot_quad, quad_dynamics_nonlinear
from utilities import sensor_meas,position_PID,attitude_PID,rate_PID
from utilities import update_plot_Quad,generate_Quad
from utilities import calculate_dis_k,calculate_speed_formation
import imageio

np.random.seed(2)

def init_plot(Quad,n):    
    fig = plt.figure(figsize = (5,5),dpi=200)
    fig.canvas.mpl_connect('close_event', handle_close)

    ax = fig.add_subplot(111, projection = '3d')
    plt.gca().spines['top'].set_visible(True)
    plt.gca().spines['right'].set_visible(True)
    lines = [ax.plot([],[],[],linewidth = .5)[0] for i in np.arange(start=0,stop=n,step=1)]
    heading = [ax.plot([],[],[],linewidth = .5, c='black')[0] for i in np.arange(start=0,stop=n,step=1)]
    heading_vel = [ax.plot([],[],[],linewidth = .5,c='red')[0] for i in np.arange(start=0,stop=n,step=1)]
    handles = lines + heading + heading_vel
    return ax, handles

def handle_close(evt):
    os._exit(0)

def init_pos(n):
    height = 20
    randint = np.random.randint(1, 10, size = (n*2, 2))
    pos_init = np.zeros((n,3))
    position_desire  = np.zeros((n,3))
    for i in np.arange(start=0,stop=n,step=1):
        pos_init[i,:] = np.append(randint[i+n,:],height)
        position_desire[i,:] = np.append(randint[i+n,:],height)
    position_desire = np.multiply(np.array([[-5, -5, height],
        [-10, -10, height],
        [5, 5, height],
        [10, 10, height],
        [-5, 5, height],
        [-10, 10, height],
        [5, -5, height],
        [10, -10, height]]),np.array([.3,.3,1]))  
    return pos_init,position_desire


def updatelines_ion(Quad, handles,j,n):
    line = handles[:n]
    heading = handles[n:2*n]
    heading_vel = handles[2*n:3*n]
    x, y, z,x_vel,y_vel,z_vel= cmo_heading(Quad,n)            
    for i in np.arange(start=0, stop=n, step=1):
        line[i].set_data(Quad[i].X_history,Quad[i].Y_history)
        line[i].set_3d_properties(Quad[i].Z_history)
        heading[i].set_data(x[i,:],y[i,:])
        if i == 0:
            heading[i].set_3d_properties(z[i,:])
            heading_vel[i].set_data(x_vel[i,:],y_vel[i,:])
            heading_vel[i].set_3d_properties(z_vel[i,:])
    plt.pause(.01)
    update_plot_Quad.update_plot_Quad(ax, Quad)

def cmo_heading(Quad,n):
    x = np.zeros((n,2))
    y = np.zeros((n,2))
    z = np.zeros((n,2))
    x_vel = np.zeros((n,2))
    y_vel = np.zeros((n,2))
    z_vel = np.zeros((n,2))
    for i in  np.arange(start=0,stop=n,step=1):
        x1 = np.cos(Quad[i].psi) * np.cos(Quad[i].theta)
        y1 = np.sin(Quad[i].psi) * np.cos(Quad[i].theta)
        z1 = np.sin(Quad[i].theta)
        normlizer = np.sqrt(x1**2+y1**2+z1**2)        
        x[i,:] = np.array([Quad[i].X,Quad[i].X+x1/normlizer*3])
        y[i,:] = np.array([Quad[i].Y,Quad[i].Y+y1/normlizer*3])
        z[i,:] = np.array([Quad[i].Z,Quad[i].Z+z1/normlizer*3])
        normlizer = np.sqrt(Quad[i].X_dot**2+Quad[i].Y_dot**2+Quad[i].Z_dot**2)
        x_vel[i,:] = np.array([Quad[i].X, Quad[i].X+Quad[i].X_dot/normlizer*normlizer])
        y_vel[i,:] = np.array([Quad[i].Y, Quad[i].Y+Quad[i].Y_dot/normlizer*normlizer])
        z_vel[i,:] = np.array([Quad[i].Z, Quad[i].Z+Quad[i].Z_dot/normlizer*normlizer])
    return x,y,z,x_vel,y_vel,z_vel

#########################
n= 8
Quad = []
delta = .01
########################
pos_init,position_desire = init_pos(n)
for i in np.arange(start=0,stop=n,step=1):
    Quad.append(generate_Quad.generate_Quad(position_desire[i,:],pos_init[i,:]))
    Quad[i].X_des_GF = 5+i
    Quad[i].Y_des_GF = 5+i
    Quad[i].Z_des_GF = 5+i
ax, handles = init_plot(Quad,n)
plot_quad.plot_quad(ax,Quad)
plt.ion()
# targets = [ax.plot([], [], [],'ro')[0] for i in np.arange(start=0,stop=n,step=1)]
#################################

p = np.zeros((n,3))
for i in np.arange(start=0,stop=n,step=1):
    p[i,:] = np.array([Quad[i].X, Quad[i].Y, Quad[i].Z])
dis_matrix = calculate_dis_k.dis_matrix_func(position_desire)
K, alpha = calculate_dis_k.calculate_k_func(dis_matrix,delta)


j = 0
filenames = []
my_path = os.getcwd()
images = []
k = 1
while True:

    for i in np.arange(start=0,stop=n,step=1):
        p[i,:] = np.array([Quad[i].X, Quad[i].Y, Quad[i].Z])
        
    v = calculate_speed_formation.calculate_speed_formation(p, K, alpha, delta, 0, 0)    
    for i in np.arange(start=0, stop=n, step=1):
        Quad[i] = quad_dynamics_nonlinear.quad_dynamics_nonlinear_speed(Quad[i],v[i,:])
        Quad[i].X_history.append(Quad[i].X)
        Quad[i].Y_history.append(Quad[i].Y)
        Quad[i].Z_history.append(Quad[i].Z)
        Quad[i] = sensor_meas.sensor_meas(Quad[i])
        Quad[i] = position_PID.position_PID(Quad[i])
        Quad[i] = attitude_PID.attitude_PID(Quad[i])
        Quad[i] = rate_PID.rate_PID(Quad[i])
    if np.mod(j,1) == 0:
        updatelines_ion(Quad, handles,j,n)
        ax.set_title(str(j))
        # filename = f'fig{k}.png'
        
        # a = os.path.join(my_path, 'Figures/')
        # a = a+filename
        # filegitnames.append(a)
        # plt.savefig(a)    
        # k = k+1  
    j = j+1
    

    if j > 150:
        break
# with imageio.get_writer('mygif.gif', mode='I') as writer:
#     for filename in filenames:
#         image = imageio.imread(filename)
#         writer.append_data(image)
    
