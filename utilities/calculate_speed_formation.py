import numpy as np


def calculate_speed_formation(p, K,alpha,delta,obstacle,o, v0_flag = 0):
    max_speed = 5
    n = p[:,0].size
    v = np.zeros((n,3))
    d = np.zeros((n,n))
    w = np.zeros((n,n))
    for i in np.arange(start=0,stop=n,step=1):
        for j in np.arange(start=0,stop=n,step=1):
            if i != j:
                d[i,j] = np.linalg.norm(p[i,:]-p[j,:])
                w[i,j] = alpha[i,j] * (-1/(K[i,j] * d[i,j] ) * csch((d[i,j] -delta)/K[i,j] )**2+1)
                if j == i:
                    w[i,j] = w[j,i]
                if i == np.inf:
                    v[i,:] = np.array([0,0,0])
                else:
                    v[i,:] = v[i,:] - w[i,j] * (p[i,:]-p[j,:])
        if v0_flag == 1 and  i == 0:
            v[i,:] = np.array([0,0,0])


        if obstacle ==1:
            lpv = (np.linalg.norm(p[i,:]-o)-2)/5
            vpv = 1*(coth(lpv)+1/2*((np.linalg.norm(p[i,:]-o))**2)-np.array([5, 5, 5]))
            v[i,:] = v[i,:] +vpv

        I = np.argwhere(np.abs(v[i,:])>max_speed)

        if I.size:
            max_v = np.max(np.abs(v[i,:]))
            ratio = max_v/max_speed
            v[i,:] = v[i,:]/ratio
    return v


def csch(x):
    return 2/(np.exp(x)-np.exp(-x))

def coth(x):
    return (np.exp(2*x)+1)/(np.exp(2*x)-1)