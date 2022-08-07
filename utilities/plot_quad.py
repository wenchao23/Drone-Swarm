import matplotlib; matplotlib.use("TkAgg")
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt

length_arm = .8
w = .05
radius = .25
   

def cuboid_data(p, phi,size=(1,1,1)):
    # code taken from
    # https://stackoverflow.com/a/35978146/4124317
    # suppose axis direction: x: to left; y: to inside; z: to upper
    # get the length, width, and height
    o = (0,0,0)
    phix = phi[0]
    phiy = phi[1]
    phiz = phi[2]
    Rx = np.array([[1, 0, 0],[0, np.cos(phix), -np.sin(phix)],[0, np.sin(phix), np.cos(phix)]])
    Ry = np.array([[np.cos(phiy), 0, np.sin(phiy)], [0, 1 ,0], [-np.sin(phiy), 0, np.cos(phiy)]])
    Rz = np.array([[np.cos(phiz), -np.sin(phiz), 0], [np.sin(phiz), np.cos(phiz), 0], [0, 0, 1]])
    l, w, h = size
    x = [[o[0]-.5*l, o[0] + l-.5*l, o[0] + l-.5*l, o[0]-.5*l, o[0]-.5*l],
         [o[0]-.5*l, o[0] + l-.5*l, o[0] + l-.5*l, o[0]-.5*l, o[0]-.5*l],
         [o[0]-.5*l, o[0] + l-.5*l, o[0] + l-.5*l, o[0]-.5*l, o[0]-.5*l],
         [o[0]-.5*l, o[0] + l-.5*l, o[0] + l-.5*l, o[0]-.5*l, o[0]-.5*l]]
    y = [[o[1]-.5*w,     o[1]-.5*w,     o[1] + w-.5*w, o[1] + w-.5*w, o[1]-.5*w],
         [o[1]-.5*w,     o[1]-.5*w,     o[1] + w-.5*w, o[1] + w-.5*w, o[1]-.5*w],
         [o[1]-.5*w,     o[1]-.5*w,     o[1]-.5*w,     o[1]-.5*w,     o[1]-.5*w],
         [o[1] + w-.5*w, o[1] + w-.5*w, o[1] + w-.5*w, o[1] + w-.5*w, o[1] + w-.5*w]]
    z = [[o[2]-.5*h, o[2]-.5*h, o[2]-.5*h, o[2]-.5*h, o[2]-.5*h],
         [o[2] + h-.5*h, o[2] + h-.5*h, o[2] + h-.5*h, o[2] + h-.5*h, o[2] + h-.5*h],
         [o[2]-.5*h, o[2]-.5*h, o[2] + h-.5*h, o[2] + h-.5*h, o[2]-.5*h],
         [o[2]-.5*h, o[2]-.5*h, o[2] + h-.5*h, o[2] + h-.5*h, o[2]-.5*h]]
    x = np.array(x)
    y = np.array(y)
    z = np.array(z)
    for i in np.arange(start=0,stop=4,step=1):
        data = np.transpose([x[i,:],y[i,:],z[i,:]])
        #np.dot(np.dot(ydata, Rx))
        data = np.transpose(data.dot(Rx).dot(Ry).dot(Rz))
        #ydata = np.transpose(np.dot(ydata,Rx))
        x[i,:] = data[0,:] + p[0]
        y[i, :] = data[1, :]+ p[1]
        z[i, :] = data[2, :]+ p[2]
    return x, y, z

def motor_data(pos,phi,index):
    phix = phi[0]
    phiy = phi[1]
    phiz = phi[2]
    p1 = index
    Rx = np.array([[1, 0, 0],[0, np.cos(phix), -np.sin(phix)],[0, np.sin(phix), np.cos(phix)]])
    Ry = np.array([[np.cos(phiy), 0, np.sin(phiy)], [0, 1 ,0], [-np.sin(phiy), 0, np.cos(phiy)]])
    Rz = np.array([[np.cos(phiz), -np.sin(phiz), 0], [np.sin(phiz), np.cos(phiz), 0], [0, 0, 1]])
    p = np.array(p1)
    p = np.transpose(p.dot(Rx).dot(Ry).dot(Rz))
    theta = np.linspace(0, 2 * np.pi, 20)
    x = radius * np.sin(theta)
    y = radius * np.cos(theta)
    n = np.array(([1.0]*np.size(y)))
    z = w*.05 * n
    data = np.transpose([x, y, z])
    data = np.transpose(data.dot(Rx).dot(Ry).dot(Rz))
    
    x = data[0, :]+p[0]+pos[0]
    y = data[1, :]+p[1]+pos[1]
    z = data[2, :] +p[2]+pos[2]
    return x, y, z

def plotCubeAt(phi=(0,0,0),pos=(0,0,0), size=(1,1,1), ax=None,**kwargs):
    # Plotting a cube element at position pos
    if ax !=None:
        X, Y, Z = cuboid_data(pos, phi, size)
        ax_arm = ax.plot_surface(X, Y, Z, rstride=1, cstride=1, **kwargs)

    return ax_arm


def plotMotor(phi=(0,0,0),pos=(0,0,0), size=(1,1,1), index=1, ax=None,**kwargs):
    # Plotting a cube element at position pos
    if ax !=None:

        X, Y, Z = motor_data(pos,phi, index)
        ax_motor1 = ax.plot(X, Y, Z,**kwargs)
    return ax_motor1


def plot_quad(ax,Quad):
    
    sizes = [(length_arm,w,w), (w,length_arm,w)]

    colors = ["crimson","black"]
    for j in  np.arange(start=0,stop=len(Quad),step=1):
        i = 0
        phi1 = [(Quad[j].phi,Quad[j].theta,Quad[j].psi),(Quad[j].phi,Quad[j].theta,Quad[j].psi)] #roll,pitch,yaw
        #phi1 = [(0,0,0),(0,0,0)]
        positions = (Quad[j].X, Quad[j].Y, Quad[j].Z)

        for ph, s,c in zip(phi1,sizes,colors):
            #ph = (ph[0],ph[1]+np.pi/4,ph[2]+np.pi/4)
            arm_temp = plotCubeAt(phi=ph, pos=positions, size=s, ax=ax, color=c)

            
            p_temp = (-(i==0)*length_arm*.5,0 -(i==1)*length_arm*.5, 0)
            motor1_temp = plotMotor(phi=ph, pos=positions, size=s, index=p_temp,ax=ax, color=c, linewidth = 1)


            p_temp = ((i==0)*length_arm*.5 , (i==1)*length_arm*.5, 0)
            motor2_temp = plotMotor(phi=ph, pos=positions, size=s, index=p_temp, ax=ax, color=c, linewidth = 1)
            if i==0:
                Quad[j].armX = arm_temp
                Quad[j].motorX1 = motor1_temp
                Quad[j].motorX2 = motor2_temp
            else:
                Quad[j].armY = arm_temp
                Quad[j].motorY1 = motor1_temp
                Quad[j].motorY2 = motor2_temp
            i = i + 1
    return Quad