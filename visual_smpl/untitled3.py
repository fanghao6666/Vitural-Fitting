from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt


def plot_line3D(v,f):
    fig=plt.figure()
    ax=fig.add_subplot(111,projection='3d')
    
    for i in range(f.shape[0]):
        x=np.array([v[f[i,0],0],v[f[i,1],0]])
        y=np.array([v[f[i,0],1],v[f[i,1],1]])
        z=np.array([v[f[i,0],2],v[f[i,1],2]])
        ax.plot(x,y,z,c='r')
    ax.legend()
    plt.show()
    
        