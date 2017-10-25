import os

import scipy.io as sio
import numpy as np

class loader(object):
    
    def __init__(self,filetype):
        self.filetype=filetype
        
    def load_ply_v(self,path):
        file=open(path,'r')
        verts=[]
        txtline=file.readline()
        while txtline[0]>='a' and txtline[0]<='z':
            txtline=file.readline()
        
        while txtline:
            txtline1=txtline.split()
            if txtline1[0]=='3':
                break
            else:
                vv=[]
                for i in range(3):
                    vv.append(float(txtline1[i]))
                vv=np.array(vv)
                verts.append(vv)
            txtline=file.readline()
        file.close()
        return verts
    
    def load_ply_p(self,path):
        file=open(path,'r')
        p=[]
        txtline=file.readline()
        while txtline:
            if txtline[0]=='3':
                txtline1=txtline.split()
                txtline1.remove('3')
                pp=[]
                for elem in txtline1:
                    pp.append(int(elem))
                p.append(pp)
            else:
                pass
            txtline=file.readline()
        file.close()
        return p
    
    def load_template_obj(self,path):
        file=open(path,"r")
        txtline=file.readline()
        v=[]
        p=[]
        while txtline:
            if txtline[0]=='v':
                txtline1=txtline.split()
                txtline1.remove('v')
                vv=[]
                for i in range(3):
                    vv.append(float(txtline1[i]))
                v.append(vv)
            elif txtline[0]=='f':
                txtline1=txtline.split()
                txtline1.remove('f')
                pp=[]
                for i in range(3):
                    pp.append(int(txtline1[i]))
                p.append(pp)
            txtline=file.readline()
        file.close()
        return np.array(v),np.array(p)
    
    def load_obj_v(self,path):
        file=open(path,"r")
        txtline=file.readline()
        v=[]
        while txtline:
            if txtline[0]=='v' and txtline[1]!='n':
                txtline1=txtline.split()
                txtline1.remove('v')
                vv=[]
                for i in range(3):
                    vv.append(float(txtline1[i]))
                vv=np.array(vv)
                v.append(vv)
            else:
                break
            txtline=file.readline()
        file.close()
        return v

    def load_obj_p(self,path):
        file=open(path,"r")
        txtline=file.readline()
        p=[]
        while txtline:
            if txtline[0]=='f':
                txtline1=txtline.split()
                txtline1.remove('f')
                pp=[]
                for i in range(3):
                    pp.append(int(txtline1[i].split('//')[0]))
                p.append(pp)
            else:
                pass
            txtline=file.readline()
        file.close()
        return p
    
    def load(self,path):
        if self.filetype=='template_obj':
            return np.array(self.load_template_obj(path))
        elif self.filetype=='instance_ply':
            y_v=[]
            y_p=[]
            for elem in os.listdir(path):
                y_v.append(self.load_ply_v(path+'\\'+elem))
            y_p=np.array(self.load_ply_p(path+'\\'+os.listdir(path)[0]))
            y_v=np.array(y_v)
            return y_v,y_p
        elif self.filetype=='instance_obj' or self.filetype=='test_obj':
            y_v=[]
            y_p=[]
            for elem in os.listdir(path):
                y_v.append(self.load_obj_v(path+'\\'+elem))
            y_p=np.array(self.load_obj_p(path+'\\'+os.listdir(path)[0]))
            y_v=np.array(y_v)
            return y_v,y_p
        else:
            print "Error!"
    

def load_template_obj(path='dataset/template.obj'):
    v=[]
    face=[]
    f=open(path,'r')
    for line in f.readlines():
        words=line.split()
        if words[0]=='v':
            words.remove('v')
            vv=[]
            for word in words:
                vv.append(float(word))
            v.append(vv)
        if words[0]=='f':
            words.remove('f')
            ff=[]
            for word in words:
                ff.append(int(word))
            face.append(ff)
    v=np.array(v)
       
    color=9*2*v[:,3]+3*2*v[:,4]+2*v[:,5]
    for elem in color:
        elem=int(elem)
    return v[:,:3],np.array(color),np.array(face)-1




def output(v,face,path):
    f=open('output/'+path,'w')
    for i in range(v.shape[0]):
        f.write('v ')
        for j in range(v.shape[1]):
            f.write(str(v[i,j])+' ')
        f.write('\n')
    
    for i in range(face.shape[0]):
        f.write('f ')
        for j in range(face.shape[1]):
            f.write(str(face[i,j])+' ')
        f.write('\n')
        
    return 'output/'+path