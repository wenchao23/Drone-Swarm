import numpy as np
import utilities.quad_dynamics_nonlinear

def calculate_v_combine(Quad,v,TargetLocation,obstacle):
    max_speed = 5
    vvv = np.zeros(v.shape)
    n = Quad.__len__()
    for i in np.arange(start=0,stop=n,step=1):
        if i == 0:

            # if np.size(Quad[i].X_history)>200:
            #     temp = (Quad[i].X_history[-20]-Quad[i].X_history[-1])**2  + (Quad[i].Y_history[-20]-Quad[i].Y_history[-1])**2
            #     if np.sqrt(temp)<1 and Quad[i].Stuck is 0:
            #         Quad[i].Stuck = 1
            


            # if Quad[i].Stuck is not 0:
            #     TargetLocation = [Quad[i].X,Quad[i].Y,TargetLocation[-1]+30]
            #     Quad[i].Stuck = Quad[i].Stuck + 1
            #     if Quad[i].Stuck == 100:
            #         Quad[i].Stuck = 0
            p1 = np.array([Quad[i].X,Quad[i].Y,Quad[i].Z])-np.array(TargetLocation)
            # temp = np.sum(obstacle - np.array([Quad[i].X,Quad[i].Y,Quad[i].Z]),axis=1)**2
            # distance_ob_UAV = np.sqrt(temp)

            dis = (np.array([Quad[i].X,Quad[i].Y,Quad[i].Z]) - obstacle)/2;
            dis_temp1 = (np.array([Quad[i].X,Quad[i].Y,Quad[i].Z])-np.array([Quad[i].X,Quad[i].Y,110]))/5
            dis_temp2 = np.array([Quad[i].X,Quad[i].Y,Quad[i].Z])-np.array([Quad[i].X,Quad[i].Y,180])/1
            dis = np.vstack([dis,dis_temp1])
            dis = np.vstack([dis,dis_temp2])
            temp = np.sqrt(np.sum(dis**2,axis=1))**4
            temp = dis / temp[:, None]
            p2 = np.sum(10*temp,axis=0)
            vv = -.7*(1/(1+np.sum(np.abs(p2))))*p1+p2;
            vv = vv+.4*v[i,:]
            I = np.argwhere(np.abs(vv)>max_speed)
            if np.isnan(vv).any() or np.isinf(vv).any():
                vv


            
            if I.size:
                max_v = np.max(np.abs(vv))
                ratio = max_v/max_speed
                vv = vv/ratio
        else:
            # temp = np.sum(obstacle - np.array([Quad[i].X,Quad[i].Y,Quad[i].Z]),axis=1)**2
            # distance_ob_UAV = np.sqrt(temp)

            dis = (np.array([Quad[i].X,Quad[i].Y,Quad[i].Z]) - obstacle)/2;
            dis_temp1 = (np.array([Quad[i].X,Quad[i].Y,Quad[i].Z])-np.array([Quad[i].X,Quad[i].Y,110]))/5
            dis_temp2 = np.array([Quad[i].X,Quad[i].Y,Quad[i].Z])-np.array([Quad[i].X,Quad[i].Y,180])/1
            dis = np.vstack([dis,dis_temp1])
            dis = np.vstack([dis,dis_temp2])
            temp = np.sqrt(np.sum(dis**2,axis=1))**4
            temp = dis / temp[:, None]
            p2 = np.sum(12*temp,axis=0)
            vv = -.7*(1/(.00001+np.sum(np.abs(p2))))*p1+p2;
            vv = vv+5*v[i,:]
            I = np.argwhere(np.abs(vv)>max_speed)

            if I.size:
                max_v = np.max(np.abs(vv))
                ratio = max_v/max_speed
                vv = vv/ratio
        vvv[i,:] = vv
    return vvv