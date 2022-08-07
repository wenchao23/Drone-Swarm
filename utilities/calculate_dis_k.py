from scipy.optimize import fsolve


import numpy as np
def calculate_k_func(d_matrix,delta):
    n = d_matrix[:,1].size
    K = np.zeros((n,n))
    alpha = np.ones((n,n))-np.eye(n)
    #x = symbols('x')
    for i in np.arange(start=0,stop=n,step=1):
        for j in np.arange(start=i+1,stop=n,step=1):
            if i != j:
                data =(delta, d_matrix[i,j])
                K[i,j] = fsolve(func, x0 = 1,args=data)
                K[j,i] = K[i,j]
                
                #K[i,j] = solve(np.sqrt(dis_matrix[i,j])-delta-x*acsch(np.sqrt( x * np.sqrt(dis_matrix[i,j]))),x)                
    return K,alpha

def dis_matrix_func(u,err = 0):
    n = u[:,0].size
    dis_matrix = np.zeros((n,n))
    for i in np.arange(start=0,stop=n,step=1):
        for j in np.arange(start=i+1,stop=n,step=1):
            if i != j:
                noise = np.random.randn(1)*err
                dis_matrix[i,j] = ( np.linalg.norm(u[i,:]-u[j,:])+noise)**2
                dis_matrix[j,i]  = dis_matrix[i,j]
                
    return dis_matrix

def func(x,*data):
    delta, d = data
    return np.sqrt(d)-delta-x*acsch(np.sqrt( x * np.sqrt(d)))

def acsch(x):
    return np.log(np.sqrt(1+1/x**2)+1/x)