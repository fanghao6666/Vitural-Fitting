import numpy as np
from math import acos,sin
import scipy.io as sio


 
J_num=20
part_list=[[0, 1],
 [1, 2, 3, 10],
 [2, 4],
 [3, 5],
 [4, 6],
 [5, 7],
 [6, 8],
 [7, 9],
 [10, 11],
 [11, 12, 13],
 [12, 14],
 [13, 15],
 [14, 16],
 [15, 17],
 [16, 18],
 [17, 19]]

kin=[-1,0,  1,  1,  2,  3,  4,  5,  1,  8,  9,  9, 10, 11, 12, 13]
kin_v=[(0,0),(0,1),(1,2),(1,3),(2,4),(3,5),(4,6),(5,7),(6,8),(7,9),(1,10),(10,11),(11,12),(11,13),(12,14),(13,15),(14,16),(15,17),(16,18),(17,19)]
ax=sio.loadmat('training_result/temp_J1.mat')['ax']

J_t=[]

def trans_R(M):
    tr=np.trace(M)
    if tr>3.0:
        tr=3.0
    if tr<-1.0:
        tr=-1.0
    theta=acos((tr-1.0)/2)
    pp=[]
    pp.append(M[2,1]-M[1,2])
    pp.append(M[0,2]-M[2,0])
    pp.append(M[1,0]-M[0,1])
   
    t=abs(theta)/(2*sin(abs(theta))+0.00000000001)*np.array(pp)
    return t
    
    
def gen_Jt(J,ax):
    global J_t
    J_t.append(np.zeros(3))
    J_len=[]
    J_len.append(0)
    for i in range(1,len(kin_v)):
        J_len.append(np.linalg.norm((J[kin_v[i][1]]-J[kin_v[i][0]])))
        ori=ax[kin_v[i][1]]-ax[kin_v[i][0]]
        ori=ori/np.linalg.norm(ori)
        J_t.append(J_t[kin_v[i][0]]+J_len[i]*ori)
    return np.array(J_t)
    
    
def Exp(w):

    num=np.linalg.norm(w)
    if num>0.0:
        ww=w/num
        x1=ww[0].reshape((1,-1))
        x2=ww[1].reshape((1,-1))
        x3=ww[2].reshape((1,-1))
        x0=np.zeros(1).reshape((1,-1))
        v1=np.hstack((x0,-x3,x2))
        v2=np.hstack((x3,x0,-x1))
        v3=np.hstack((-x2,x1,x0))
        www=np.vstack((v1,v2,v3))

        return np.eye(3)+np.sin(num)*www+(1.0-np.cos(num))*(www.dot(www))
    else:
        return np.eye(3)  

def compute_n(j1,j2,j3):
    J1=j2-j1
    J2=j3-j1
    return np.cross(J1,J2)

def compute_t(x_t,x):
    axis=np.cross(x_t,x)
    axis=axis/(np.linalg.norm(axis)+0.000000000001)
    th=acos(np.dot(x,x_t)/(np.linalg.norm(x)*np.linalg.norm(x_t)+0.00000000001))
    axis=th*axis
    return axis

def first_J_to_pose(J):
    global J_t
    J_t=gen_Jt(J,ax)
    R=[]
    for i in range(len(part_list)):
#        if i==1 or i==9:
#            j=J[part_list[i]]
#            j_t=J_t[part_list[i]]
#            j-=np.mean(j,0)
#            j_t-=np.mean(j_t,0)
#            U,S,V=np.linalg.svd(j.T.dot(j_t))
#            R.append(U.dot(V))
        if i==1:
            j=J[part_list[i]]
            j_t=J_t[part_list[i]]
            n1=compute_n(j[0],j[1],j[3])
            n_t1=compute_n(j_t[0],j_t[1],j_t[3])
            n2=compute_n(j[0],j[2],j[3])
            n_t2=compute_n(j_t[0],j_t[2],j_t[3])
            n1[1]=-n1[1]
            n_t1[1]=-n_t1[1]
            n2[1]=-n2[1]
            n_t2[1]=-n_t2[1]
            axis1=compute_t(n_t1,n1)
            axis2=compute_t(n_t2,n2)
            axis=(axis1+axis2)/2.0
            R.append(Exp(axis))
        elif i==9:
            j=J[part_list[i]]
            j_t=J_t[part_list[i]]
            n=compute_n(j[0],j[1],j[2])
            n_t=compute_n(j_t[0],j_t[1],j_t[2])
            n[1]=-n[1]
            n_t[1]=-n_t[1]
            axis=compute_t(n_t,n)
            R.append(Exp(axis))
            
        else:
#            j=J[part_list[i]]
#            j_t=J_t[part_list[i]]
#            j-=np.mean(j,0)
#            j_t-=np.mean(j_t,0)
#            if np.linalg.norm(j-j_t)<0.05:
#                R.append(np.eye(3))
#            else:
#                U,S,V=np.linalg.svd(j.T.dot(j_t))
#                R.append(U.dot(V))
            j=J[part_list[i]]
            j_t=J_t[part_list[i]]
            j=j[1]-j[0]
            j_t=j_t[1]-j_t[0]
            j[1]=-j[1]
            j_t[1]=-j_t[1]
            axis=-np.cross(j,j_t)
            axis=axis/(np.linalg.norm(axis)+0.000000000001)
            th=acos(np.dot(j,j_t)/(np.linalg.norm(j)*np.linalg.norm(j_t)+0.00000000001))
            axis=th*axis
            R.append(Exp(axis))    
#    R[1]=np.eye(3)
#    R[9]=np.eye(3)
    rel_R=[]
    rel_R.append(R[0])
    for i in range(1,len(part_list)):
        rel_R.append(R[kin[i]].T.dot(R[i]))
    pose=[]
    for i in range(len(part_list)):
        pose.append(trans_R(rel_R[i]))
    pose=np.array(pose)

    return pose

def J_to_pose(J):
    global J_t
    J_t=np.array(J_t)
    R=[]
    for i in range(len(part_list)):
#        if i==1 or i==9:
#            j=J[part_list[i]]
#            j_t=J_t[part_list[i]]
#            j-=np.mean(j,0)
#            j_t-=np.mean(j_t,0)
#            U,S,V=np.linalg.svd(j.T.dot(j_t))
#            R.append(U.dot(V))
        if i==1:
            j=J[part_list[i]]
            j_t=J_t[part_list[i]]
            n1=compute_n(j[0],j[1],j[3])
            n_t1=compute_n(j_t[0],j_t[1],j_t[3])
            n2=compute_n(j[0],j[2],j[3])
            n_t2=compute_n(j_t[0],j_t[2],j_t[3])
            n1[1]=-n1[1]
            n_t1[1]=-n_t1[1]
            n2[1]=-n2[1]
            n_t2[1]=-n_t2[1]
            axis1=compute_t(n_t1,n1)
            axis2=compute_t(n_t2,n2)
            axis=(axis1+axis2)/2.0
            R.append(Exp(axis))
        elif i==9:
            j=J[part_list[i]]
            j_t=J_t[part_list[i]]
            n=compute_n(j[0],j[1],j[2])
            n_t=compute_n(j_t[0],j_t[1],j_t[2])
            n[1]=-n[1]
            n_t[1]=-n_t[1]
            axis=compute_t(n_t,n)
            R.append(Exp(axis))
            
        else:
#            j=J[part_list[i]]
#            j_t=J_t[part_list[i]]
#            j-=np.mean(j,0)
#            j_t-=np.mean(j_t,0)
#            if np.linalg.norm(j-j_t)<0.05:
#                R.append(np.eye(3))
#            else:
#                U,S,V=np.linalg.svd(j.T.dot(j_t))
#                R.append(U.dot(V))
            j=J[part_list[i]]
            j_t=J_t[part_list[i]]
            j=j[1]-j[0]
            j_t=j_t[1]-j_t[0]
            j[1]=-j[1]
            j_t[1]=-j_t[1]
            axis=-np.cross(j,j_t)
            axis=axis/(np.linalg.norm(axis)+0.000000000001)
            th=acos(np.dot(j,j_t)/(np.linalg.norm(j)*np.linalg.norm(j_t)+0.00000000001))
            axis=th*axis
            R.append(Exp(axis))    
#    R[1]=np.eye(3)
#    R[9]=np.eye(3)
    rel_R=[]
    rel_R.append(R[0])
    for i in range(1,len(part_list)):
        rel_R.append(R[kin[i]].T.dot(R[i]))
    pose=[]
    for i in range(len(part_list)):
        pose.append(trans_R(rel_R[i]))
    pose=np.array(pose)

    return pose
    
    
    
