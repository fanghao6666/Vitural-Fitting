from pykinect2 import PyKinectV2
from pykinect2.PyKinectV2 import *
from pykinect2 import PyKinectRuntime

import numpy as np



def gen_joints_data(my_kinect):
 #   a=np.array([0,1,3,2,5,4,7,6,9,8,10,11,13,12,15,14,17,16,19,18])
    joint_points=[]
    my_bodies=None
    if my_kinect.has_new_body_frame():
        my_bodies=my_kinect.get_last_body_frame()
        
    if my_bodies is not None: 
        for i in range(0, my_kinect.max_body_count):
            body = my_bodies.bodies[i]
            if body.is_tracked:
                jp= body.joints 
                joint_points.append([jp[PyKinectV2.JointType_Head].Position.x,jp[PyKinectV2.JointType_Head].Position.y,jp[PyKinectV2.JointType_Head].Position.z])
                joint_points.append([jp[PyKinectV2.JointType_SpineShoulder].Position.x,jp[PyKinectV2.JointType_SpineShoulder].Position.y,jp[PyKinectV2.JointType_SpineShoulder].Position.z])
                joint_points.append([jp[PyKinectV2.JointType_ShoulderRight].Position.x,jp[PyKinectV2.JointType_ShoulderRight].Position.y,jp[PyKinectV2.JointType_ShoulderRight].Position.z])
                joint_points.append([jp[PyKinectV2.JointType_ShoulderLeft].Position.x,jp[PyKinectV2.JointType_ShoulderLeft].Position.y,jp[PyKinectV2.JointType_ShoulderLeft].Position.z])
                joint_points.append([jp[PyKinectV2.JointType_ElbowRight].Position.x,jp[PyKinectV2.JointType_ElbowRight].Position.y,jp[PyKinectV2.JointType_ElbowRight].Position.z])
                joint_points.append([jp[PyKinectV2.JointType_ElbowLeft].Position.x,jp[PyKinectV2.JointType_ElbowLeft].Position.y,jp[PyKinectV2.JointType_ElbowLeft].Position.z])
                joint_points.append([jp[PyKinectV2.JointType_WristRight].Position.x,jp[PyKinectV2.JointType_WristRight].Position.y,jp[PyKinectV2.JointType_WristRight].Position.z])
                joint_points.append([jp[PyKinectV2.JointType_WristLeft].Position.x,jp[PyKinectV2.JointType_WristLeft].Position.y,jp[PyKinectV2.JointType_WristLeft].Position.z])
                joint_points.append([jp[PyKinectV2.JointType_HandRight].Position.x,jp[PyKinectV2.JointType_HandRight].Position.y,jp[PyKinectV2.JointType_HandRight].Position.z])
                joint_points.append([jp[PyKinectV2.JointType_HandLeft].Position.x,jp[PyKinectV2.JointType_HandLeft].Position.y,jp[PyKinectV2.JointType_HandLeft].Position.z])

                
                joint_points.append([jp[PyKinectV2.JointType_SpineMid].Position.x,jp[PyKinectV2.JointType_SpineMid].Position.y,jp[PyKinectV2.JointType_SpineMid].Position.z])
                joint_points.append([jp[PyKinectV2.JointType_SpineBase].Position.x,jp[PyKinectV2.JointType_SpineBase].Position.y,jp[PyKinectV2.JointType_SpineBase].Position.z])
                joint_points.append([jp[PyKinectV2.JointType_HipRight].Position.x,jp[PyKinectV2.JointType_HipRight].Position.y,jp[PyKinectV2.JointType_HipRight].Position.z])
                joint_points.append([jp[PyKinectV2.JointType_HipLeft].Position.x,jp[PyKinectV2.JointType_HipLeft].Position.y,jp[PyKinectV2.JointType_HipLeft].Position.z])
                joint_points.append([jp[PyKinectV2.JointType_KneeRight].Position.x,jp[PyKinectV2.JointType_KneeRight].Position.y,jp[PyKinectV2.JointType_KneeRight].Position.z])
                joint_points.append([jp[PyKinectV2.JointType_KneeLeft].Position.x,jp[PyKinectV2.JointType_KneeLeft].Position.y,jp[PyKinectV2.JointType_KneeLeft].Position.z])
                joint_points.append([jp[PyKinectV2.JointType_AnkleRight].Position.x,jp[PyKinectV2.JointType_AnkleRight].Position.y,jp[PyKinectV2.JointType_AnkleRight].Position.z])
                joint_points.append([jp[PyKinectV2.JointType_AnkleLeft].Position.x,jp[PyKinectV2.JointType_AnkleLeft].Position.y,jp[PyKinectV2.JointType_AnkleLeft].Position.z])
                joint_points.append([jp[PyKinectV2.JointType_FootRight].Position.x,jp[PyKinectV2.JointType_FootRight].Position.y,jp[PyKinectV2.JointType_FootRight].Position.z])
                joint_points.append([jp[PyKinectV2.JointType_FootLeft].Position.x,jp[PyKinectV2.JointType_FootLeft].Position.y,jp[PyKinectV2.JointType_FootLeft].Position.z])
                joint_points=np.array(joint_points)
           #     joint_points=joint_points[a]
				
                jo=np.hstack((joint_points[:,1].reshape((-1,1)),-joint_points[:,2].reshape((-1,1)),joint_points[:,0].reshape((-1,1))))
                return jo
    
    return None

#my_kinect=PyKinectRuntime.PyKinectRuntime(PyKinectV2.FrameSourceTypes_Body)
#jp=[]
#while True:
#    jp.append(gen_joints_data(my_kinect))