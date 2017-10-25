import scipy.io as sio
from lbs import *
import numpy as np


ii=sio.loadmat('training_result/result.mat')
W=np.array(ii['W'])
T=np.array(ii['T'])
JJ=np.array(ii['JJ'])
P=np.array(ii['P'])

i=sio.loadmat('training_result/U_and_mu.mat')
U=np.array(i['U']).T
mu=np.array(i['mu'])

def gen_pose_Model(theta,f_output):
    J=JJ.dot(T)
    v,n=pose_blend_skin_func(W,T,theta,J,P)
    if f_output:
        name=raw_input("Please input file name:")
        output(v,template_f+1,str(name)+'.obj')
        return v,n
    else:
        return v,n
    
def gen_shape_Model(theta,beta,f_output):
    Tb=U.dot(beta)+mu
    Tb=Tb.reshape((-1,3))
    J=JJ.dot(T+Tb)
    v,n=pose_blend_skin_func(W,T+Tb,theta,J,P)
    if f_output:
        name=raw_input("Please input file name:")
        output(v,template_f+1,str(name)+'.obj')
        return v,n
    else:
        return v,n