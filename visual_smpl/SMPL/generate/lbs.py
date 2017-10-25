
import numpy as np

from load_and_output import *


template_v,template_color,template_f=load_template_obj()

template_v=np.array(template_v)
template_v-=np.mean(template_v,0)

theta_num=16
kintree_table=np.array([[-1,0,0,1,2,3,4,0,7,8,8,9,10,11,12],range(15)])   


def comput_norm(v):
    v_num=v.shape[0]
    f_num=template_f.shape[0]
    v_norm={}
    for i in range(v_num):
        v_norm[i]=[]
        
    for i in range(f_num):
        v0=v[template_f[i,1]]-v[template_f[i,0]]
        v1=v[template_f[i,2]]-v[template_f[i,0]]
        n=np.cross(v0,v1)
        n=n/(np.linalg.norm(n)+0.0000000000000000000000001)
        v_norm[template_f[i,0]].append(n)
        v_norm[template_f[i,1]].append(n)
        v_norm[template_f[i,2]].append(n)
        
    v_n=[]
    for i in range(v_num):
        summ=0
        l=len(v_norm[i])
        for j in range(l):
            summ+=v_norm[i][j]
        v_n.append(summ/(float(l)+0.0000000000000000000000001))
            
    return np.array(v_n)


template_norm=comput_norm(template_v)

def Exp(w):

    num=np.linalg.norm(w)
    if num>0.000000000000000000000001:
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

def G(pose,J):
    with_zeros=lambda x: np.vstack((x,np.array([[0,0,0,1]])))
    parent=np.array([[i,kintree_table[0,i]] for i in range(0,kintree_table.shape[1])])
    first=with_zeros(np.hstack((Exp(pose[parent[0,0],:]),np.zeros(3).reshape((-1,1)))))
    second=first.dot(with_zeros(np.hstack((Exp(pose[parent[1,0],:]),J[parent[0,0],:].reshape((-1,1))))))    
    result={}
    result[0]=first
    result[1]=second
    for i in range(1,parent.shape[0]):
        mm=with_zeros(np.hstack((Exp(pose[parent[i,0]+1,:]),(J[parent[i,0],:]-J[parent[i,1],:]).reshape((-1,1)))))
        mm=result[parent[i,1]+1].dot(mm)
        result[i+1]=mm

    return result


def global_rigid_transformation(pose,J):
    pack = lambda x : np.hstack([np.zeros((4, 3)), x.reshape((4,1))])
    results=G(pose,J)
    results2=[]
    results2.append(results[0] - (pack(results[0].dot(np.hstack( ( (np.zeros(3).reshape((1,-1))), np.zeros(1).reshape((1,-1))) ).reshape((-1,1))))))
    for i in range(len(results)-1):
        results2.append(results[i+1] - (pack(results[i+1].dot(np.hstack( ( (J[i,:].reshape((1,-1))), np.zeros(1).reshape((1,-1))) ).reshape((-1,1))))))
            
    return np.dstack(results2)


def verts_core(pose,J,W,t):
    A=global_rigid_transformation(pose,J)
    T=A.dot(W)
    rest_shape_h = np.vstack((t.T, np.ones((1, t.shape[0]))))
        
    v =(T[:,0,:] * rest_shape_h[0, :].reshape((1, -1)) + 
        T[:,1,:] * rest_shape_h[1, :].reshape((1, -1)) + 
        T[:,2,:] * rest_shape_h[2, :].reshape((1, -1)) + 
        T[:,3,:] * rest_shape_h[3, :].reshape((1, -1))).T
        
    n_ref=np.vstack((template_norm.T,np.zeros((1,template_norm.shape[0]))))
    
    n=(T[:,0,:] * n_ref[0, :].reshape((1, -1)) + 
        T[:,1,:] * n_ref[1, :].reshape((1, -1)) + 
        T[:,2,:] * n_ref[2, :].reshape((1, -1)) + 
        T[:,3,:] * n_ref[3, :].reshape((1, -1))).T
    n=n[:,:3]
    n=n/(np.linalg.norm(n,axis=1).reshape((-1,1)))
    return v[:,:3]/v[:,3].reshape((-1,1)),n[:,:3]

  
    
R0=np.eye(3).reshape((3,3))
for i in range(theta_num-1):
    R0=np.vstack((R0,np.eye(3).reshape((3,3))))
R0=R0.reshape(-1)



def basic_blend_skin_func(W,t,theta,J):
    return verts_core(theta, J,W,t)



def pose_blend_skin_func(W,t,theta,J,P):
    K=theta.shape[0]
    R=np.array([]).reshape((-1,3))
    for i in range(K):
        R=np.vstack((R,Exp(theta[i]).reshape((3,-1))))
    R=R.reshape(-1)
    
    B=(R-R0).dot(P.T).reshape((-1,3))

    return basic_blend_skin_func(W, t+B, theta, J)

def full_blend_skin_func(W,t,theta,J,P,beta,S):
    
    Bs=beta.dot(S)

    return pose_blend_skin_func(W, t+Bs, theta, J, P)




    
