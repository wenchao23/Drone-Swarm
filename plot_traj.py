import matplotlib; matplotlib.use("TkAgg")
import numpy as np
import matplotlib .pyplot as plt
from utilities import obstacle_gen
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection, Line3DCollection
import pickle
def redraw_figure():
    plt.draw()
    plt.pause(0.00001)

fig = plt.figure(figsize = (5,5),dpi=200)
ax = Axes3D(fig)
points = np.load('points100.npy')
Quad = np.load('Traj31_MaxSpeed5_WithCov.npy',allow_pickle=1)
# m= int(np.round(np.size(Quad[4].X_history)))
# mm = int(np.round(m*1/2*1.3))
# for i in np.arange(start = 4, stop =Quad.__len__(), step = 1):
#     for j in np.arange(start = 0, stop =m-mm, step = 1):
#         Quad[i].X_history[mm+j] =  Quad[i].X_history[mm+j-1] - 2*0.02
#         Quad[i].Y_history[mm+j] =  Quad[i].Y_history[mm+j-1] 
#         Quad[i].Z_history[mm+j] =  Quad[i].Z_history[mm+j-1]

pp =  [list(zip(quad.X_history,quad.Y_history,quad.Z_history,quad.Cov)) for quad in Quad]

np.save('Traj31_MaxSpeed5_WithCov_New_slippoint_middle2.npy',pp)
#pp =  [list(zip(quad.X_history,quad.Y_history,quad.Z_history)) for quad in Quad]
# np.save('Traj17_MaxSpeed3_new_marlab1.npy',Quad[0])
# np.save('Traj17_MaxSpeed3_new_marlab2.npy',Quad[1])
# np.save('Traj17_MaxSpeed3_new_marlab3.npy',Quad[2])
# np.save('Traj17_MaxSpeed3_new_marlab4.npy',Quad[3])
# np.save('Traj17_MaxSpeed3_new_marlab5.npy',Quad[4])
# np.save('Traj17_MaxSpeed3_new_marlab6.npy',Quad[5])
# np.save('Traj17_MaxSpeed3_new_marlab7.npy',Quad[6])
# np.save('Traj17_MaxSpeed3_new_marlab8.npy',Quad[7])
# np.savetxt('test.out', zip(Quad[0].X_history,Quad[0].Y_history,Quad[0].Z_history))

# data = np.loadtxt('test.out')
# with open('ff.pickle', 'wb') as f:
#     pickle.dump(Quad, f)
#
#A = [list(zip(Quad.X_history,Quad.Y_history,Quad.Z_history)) for Quad in data]

row = points.__len__()
for i in np.arange(start = 0, stop =row, step = 1):   
    if  (points[i,-1]>100):# and (points[i,0]>320000 and  points[i,0]<320250) and (points[i,1]>5812700
     #and  points[i,1]<5812900):
        poistion = [points[i,0:3].tolist()]
        size = [points[i,-3:].tolist()]
        obstacle_gen.plotCubeAt2(ax,poistion,size,color = 'darkgrey', linewidths=1)


ax.set_xlim((318750-500, 321750+500))
ax.set_ylim((5811750-500, 5813250+500))
ax.set_zlim((0, 150))

# ax.set_xlim((320000, 320250))
# ax.set_ylim((5812700, 5812900))
# ax.set_zlim((0, 150))

for i in np.arange(start = 0, stop =Quad.__len__(), step = 1):   
    ax.plot(Quad[i].X_history,Quad[i].Y_history,Quad[i].Z_history)

#f.savefig("foo.pdf", bbox_inches='tight')

#ax.view_init(elev=-45, azim=65)


    #    ax.plot(Quad[i].X_State[301:],Quad[i].Y_State[301:],Quad[i].Z_history)
    #ax.plot(Quad[i,:,0],Quad[i,:,1],Quad[i,:,2])
#ax.plot(Quad[2].X_State[1:],Quad[2].Y_State[1:],Quad[2].Z_State[1:])
# lines = [ax.plot([],[],[],linewidth = 1)[0] for i in np.arange(start=0,stop=8,step=1)]

# for j in np.arange(start = 0, stop =Quad[0].X_history.__len__(), step = 100):  
    
#     for i in np.arange(start = 0, stop =Quad.__len__(), step = 1):
#         x = Quad[i].X_history[0:j]
#         y  = Quad[i].Y_history[0:j]
#         z = Quad[i].Z_history[0:j]
#         lines[i].set_data(x,y)
#         lines[i].set_3d_properties(z)
#         redraw_figure()

#print(Quad[2].Z_State)

plt.show()