import ctypes

import pyglet
from pyglet.gl import *
import draw_model


from pykinect2 import PyKinectV2
from pykinect2.PyKinectV2 import *
from pykinect2 import PyKinectRuntime

from gen_kinect_data import *
from lbs import *
from generate_Model import *
from J_to_pose import *

import lbs
import _ctypes

from pyglet.window import key
import numpy as np
import sys

if sys.hexversion >= 0x03000000:
    import _thread as thread
else:
    import thread

rotationx = 0

v=None
n=None
count=0
cnt=0
pose_list = []
MAXNUM = 600

#
#tol=0.0001*np.ones((16,1))
#axx=[6,9,10,11,12,13,14,15]
#tol[axx]=1.0
      

window = pyglet.window.Window()

lightfv = ctypes.c_float * 4

my_kinect=PyKinectRuntime.PyKinectRuntime(PyKinectV2.FrameSourceTypes_Body)


@window.event
def on_resize(width, height):
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(80., float(width)/height, 1., 100.00)
    glMatrixMode(GL_MODELVIEW)
   
    return True

@window.event
def on_draw():
    global count
            
    
    window.clear()
    glLoadIdentity()

    glLightfv(GL_LIGHT0, GL_POSITION, lightfv(-1.0, 1.0, 1.0, 0.0))
    glEnable(GL_LIGHT0)

    glTranslated(0, 0, -3)
    glRotatef(rotationx, 0, 1, 0)
    glRotatef(90, 0, 0, 1)

    glScalef(2.0,2.0,2.0)
    glEnable(GL_LIGHTING)
#    print v,n
    if v is not None and n is not None:
        draw_model.draw(v,n)
        
        
@window.event
def on_key_press(symbol,modifiers):
    
        
    global my_kinect
    if symbol==key.Q:
        my_kinect.close()
    if symbol==key.Z:
        my_kinect=PyKinectRuntime.PyKinectRuntime(PyKinectV2.FrameSourceTypes_Body)
  
from pyglet.window import mouse
@window.event

def on_mouse_drag(x, y, dx, dy, buttons, modifiers):
    global rotationx
    
    if buttons & mouse.LEFT:
        rotationx+=dx

 

def update(dt):
    global v,n,count,cnt
    J=gen_joints_data(my_kinect)
    if J is not None:
        if count==0:
            pose=first_J_to_pose(J)
            count+=1			
        else:
            pose=J_to_pose(J)
            if cnt%3 == 0:
                pose_list.append(pose)
            print("Get No. " + str(cnt) +"  pose!")
            cnt += 1
 #       v,n=gen_pose_Model(pose,1,cnt)
 #   print ("Output No. " + str(cnt) + "  model successfully!")
    if cnt == MAXNUM:
        for i in range(MAXNUM):
            v,n=gen_pose_Model(pose_list[i],1,i)
        print("Output Successfully!")
        exit()
    


   

#    global rotation
#    rotation += 90*dt
#    if rotation > 720: rotation = 0
#    print dt

pyglet.clock.schedule(update)


pyglet.app.run()
