from pyglet.gl import *


import numpy as np
from load_and_output import *
from ctypes import *

template_v,template_color,template_f=load_template_obj()
f=template_f
v=template_v
indices=[]

for i in range(f.shape[0]):
    indices.append(GLuint(f[i,0]))
    indices.append(GLuint(f[i,1]))
    indices.append(GLuint(f[i,2]))
  
gl_i=(GLuint*len(indices))(*(indices))
    
    

image_path="./texture.jpg"

def draw_texture():
    image=pyglet.resource.image(image_path).texture
    glEnable(image.target)
    glBindTexture(image.target, image.id)
    gl.glTexParameterf(image.target,
                       gl.GL_TEXTURE_WRAP_S, gl.GL_CLAMP)
    gl.glTexParameterf(image.target,
                       gl.GL_TEXTURE_WRAP_T, gl.GL_CLAMP)      

def gl_light(lighting):
    return (GLfloat * 4)(*(lighting))

def draw_material(vertices):
    global gl_i
    diffuse = [.8, .8, .8, 1.]
    ambient = [.2, .2, .2, 1.]
    specular = [0., 0., 0., 1.]
    emissive = [0., 0., 0., 1.]
    shininess = 0.
   # draw_texture()
    face=GL_FRONT_AND_BACK
    glMaterialfv(face, GL_DIFFUSE, gl_light(diffuse))
    glMaterialfv(face, GL_AMBIENT, gl_light(ambient))
    glMaterialfv(face, GL_SPECULAR, gl_light(specular))
    glMaterialfv(face, GL_EMISSION, gl_light(emissive))
    glMaterialf(face, GL_SHININESS, shininess)

    gl_v=vertices

    glInterleavedArrays(GL_N3F_V3F,0,gl_v)
    glDrawElements(GL_TRIANGLES,len(indices),GL_UNSIGNED_INT,gl_i)


              
    
def draw_mesh(vertices):
    glPushClientAttrib(GL_CLIENT_VERTEX_ARRAY_BIT)
    glPushAttrib(GL_CURRENT_BIT | GL_ENABLE_BIT | GL_LIGHTING_BIT)
    glEnable(GL_CULL_FACE)
    glEnable(GL_DEPTH_TEST)
    glEnable(GL_NORMALIZE)
    glCullFace(GL_BACK)
    draw_material(vertices)
    glPopAttrib()
    glPopClientAttrib()
    



def Bind(v,n):
    myarr=GLfloat*75000
    myar=myarr()
    k=np.hstack((n,v)).reshape(-1)
    for i in range(k.shape[0]):
        myar[i]=k[i]
    return myar

def draw(v,n):
    vv=np.hstack((v[:,0].reshape((-1,1)),v[:,1].reshape((-1,1)),v[:,2].reshape((-1,1))))
    nn=np.hstack((n[:,0].reshape((-1,1)),n[:,1].reshape((-1,1)),n[:,2].reshape((-1,1))))
    vertices=Bind(vv,nn)
    draw_mesh(vertices)
    

        