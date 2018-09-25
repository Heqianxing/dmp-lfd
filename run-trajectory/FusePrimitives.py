#!/usr/bin/env python

#import libraries
import os
import glob
import numpy as np
import math
import rospy
import roslib
import tf
import geometry_msgs.msg
import intera_interface
import cv2
import os 
import subprocess 

import tensorflow
import os
from keras import Model
from keras.models import Sequential
from keras.layers import Activation
from keras.layers import Dense
from keras import optimizers
from keras.models import load_model

from sympy import lambdify
from SawyerClass import Sawyer

# Color values in HSV
BLUELOWER = np.array([110, 100, 100])
BLUEUPPER = np.array([120, 255, 255])

# Determines noise clear for morph
KERNELOPEN = np.ones((5,5))
KERNELCLOSE = np.ones((5,5))

# Font details for display windows
FONTFACE = cv2.FONT_HERSHEY_SIMPLEX
FONTSCALE = 1
FONTCOLOR = (255, 255, 255)

def transform(x_p,y_p,x_robot,y_robot,x_image,y_image):

    a_y=(y_robot[0]-y_robot[1])/(y_image[1]-y_image[0])
    b_y=y_robot[1]-a_y*y_image[0]
    y_r=a_y*y_p+b_y
    

    a_x=(x_robot[0]-x_robot[1])/(x_image[1]-x_image[0])
    b_x=x_robot[1]-a_x*x_image[0]
    x_r=a_x*x_p+b_x

    return [x_r,y_r]

def detection():

    cam = cv2.VideoCapture(-1)

    print(cam.isOpened())

    cameraMatrix=np.array([[808.615274, 0.000000, 618.694898],[0.000000,803.883580,356.546277],[0.000000,0.000000,1.000000]])
    distCoeffs=np.array([0.070456,-0.128921,-0.000695,-0.003474,0.000000])

    y_robot=[-0.4,-0.8]
    y_image=[204,389]

    x_robot=[-0.3,0.3]
    x_image=[184,456]

    for i in range(3):

    	ret_val,img = cam.read()
    	height, width, channels = img.shape

        und_img=cv2.undistort(img,cameraMatrix,distCoeffs)

        cv2.line(und_img,(x_image[1],y_image[0]),(x_image[0],y_image[0]),(0,0,255),1)
        cv2.line(und_img,(x_image[0],y_image[0]),(x_image[0],y_image[1]),(0,0,255),1)
        cv2.line(und_img,(x_image[0],y_image[1]),(x_image[1],y_image[1]),(0,0,255),1)
        cv2.line(und_img,(x_image[1],y_image[1]),(x_image[1],y_image[0]),(0,0,255),1)

        # Convert image to HSV
        imHSV = cv2.cvtColor(und_img, cv2.COLOR_BGR2HSV)

        # Threshold the colors  
        mask_blue = cv2.inRange(imHSV, BLUELOWER, BLUEUPPER)
        mask_blue_open = cv2.morphologyEx(mask_blue, cv2.MORPH_OPEN, KERNELOPEN)
        mask_blue_close = cv2.morphologyEx(mask_blue_open, cv2.MORPH_CLOSE, KERNELCLOSE)

        #cv2.imshow('Camera', mask_blue_close)

        conts, hierarchy = cv2.findContours(mask_blue_close,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

        # Hold the centers of the detected objects
        location=[]
        positions=[]

        # loop over the contours
        for c in conts:

            # compute the center of the contour
            M = cv2.moments(c)
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            
            #cv2.drawContours(mask_blue_open, conts, -1, (0, 0, 255), 2)
            cv2.circle(und_img, (cX, cY), 1, (0, 0, 255), -1)

            location.append([cX, cY])
            
        #cv2.imshow('Camera2', und_img)

        #print location

        for c in location:

            dummy=transform(c[0],c[1],x_robot,y_robot,x_image,y_image)
            positions.append(dummy)

     	print positions
        if cv2.waitKey(1) == 27: 
            break  # esc to quit

    #cv2.destroyAllWindows()

    return positions

def gaussian(x, mu, sig):
    return np.exp(-np.power(x - mu, 2.) / (2 * np.power(sig, 2.)))

def gaussianMixture(position,cube1,cube2,cube3,cube4):

	# Standard Deviation in the x and y domain
	s_x=0.13
	s_y=0.07

	# Mean of every primitive motion
	m1=np.array([cube1[0][0],cube1[0][1]])
	m2=np.array([cube2[0][0],cube2[0][1]])
	m3=np.array([cube3[0][0],cube3[0][1]])
	m4=np.array([cube4[0][0],cube4[0][1]])

	# Cube 1 distribution
	s_1_x=0.12
	s_1_y=0.065

	x1=gaussian(position[0][0],m1[0],s_1_x)
	y1=gaussian(position[0][1],m1[1],s_1_y)

	# Cube 2 distribution
	s_2_x=0.13
	s_2_y=0.07

	x2=gaussian(position[0][0],m2[0],s_2_x)
	y2=gaussian(position[0][1],m2[1],s_2_y)

	# Cube 3 distribution
	s_3_x=0.12
	s_3_y=0.05

	x3=gaussian(position[0][0],m3[0],s_3_x)
	y3=gaussian(position[0][1],m3[1],s_3_y)

	# Cube 4 distribution
	s_4_x=0.13
	s_4_y=0.07

	x4=gaussian(position[0][0],m4[0],s_4_x)
	y4=gaussian(position[0][1],m4[1],s_4_y)

	# Calculate the x and y probabilities
	p1_x=x1/(x1+x2+x3+x4)
	p2_x=x2/(x1+x2+x3+x4)
	p3_x=x3/(x1+x2+x3+x4)
	p4_x=x4/(x1+x2+x3+x4)

	p1_y=y1/(y1+y2+y3+y4)
	p2_y=y2/(y1+y2+y3+y4)
	p3_y=y3/(y1+y2+y3+y4)
	p4_y=y4/(y1+y2+y3+y4)

	# Calculate the total probabilities
	prob1=p1_x*p1_y
	prob2=p2_x*p2_y
	prob3=p3_x*p3_y
	prob4=p4_x*p4_y

	# Normalize the probabilities
	vector=[prob1,prob2,prob3,prob4]

	prob1_n=prob1/max(vector)
	prob2_n=prob2/max(vector)
	prob3_n=prob3/max(vector)
	prob4_n=prob4/max(vector)

	# Calculate the final weights
	total=prob1_n+prob2_n+prob3_n+prob4_n

	w1=prob1_n/total 
	w2=prob2_n/total
	w3=prob3_n/total
	w4=prob4_n/total

	weights=[w1,w2,w3,w4]

	return weights
		
# Define  new node
rospy.init_node("Sawyer_Fuse")

# Create an object to interface with the arm
limb=intera_interface.Limb('right')

# Create an object to interface with the gripper
gripper = intera_interface.Gripper('right')

# Call the Sawyer Class
robot=Sawyer()

#Open the gripper
gripper.open()

# Move the Robot to the Starting Position
angles=limb.joint_angles()
angles['right_j0']=math.radians(0)
angles['right_j1']=math.radians(-50)
angles['right_j2']=math.radians(0)
angles['right_j3']=math.radians(120)
angles['right_j4']=math.radians(0)
angles['right_j5']=math.radians(0)
angles['right_j6']=math.radians(90)
limb.move_to_joint_positions(angles)


# Get the position of the cube
positions=detection()
box_x=-0.6
box_y=0
U=0.3

# Check if the positions are passed
print positions

# Positions of cubes
p_cube1=np.array([[0.1874999999999999, -0.46270270270270264]]);
p_cube2=np.array([[-0.18308823529411766, -0.48]]);
p_cube3=np.array([[0.1941176470588235, -0.7048648648648649]]);
p_cube4=np.array([[-0.17867647058823533, -0.7048648648648649]]);

# Grab the files of the primitives
primitive_1=np.loadtxt('/home/michail/ros_ws/src/intera_sdk/intera_examples/scripts/MyScripts/InverseModelData/Demo10/primitive.txt',delimiter=',')
primitive_2=np.loadtxt('/home/michail/ros_ws/src/intera_sdk/intera_examples/scripts/MyScripts/InverseModelData/Demo11/primitive.txt',delimiter=',')
primitive_3=np.loadtxt('/home/michail/ros_ws/src/intera_sdk/intera_examples/scripts/MyScripts/InverseModelData/Demo9/primitive.txt',delimiter=',')
primitive_4=np.loadtxt('/home/michail/ros_ws/src/intera_sdk/intera_examples/scripts/MyScripts/InverseModelData/Demo12/primitive.txt',delimiter=',')

# Assign a weight to each primitive motion
weights=gaussianMixture(positions,p_cube1,p_cube2,p_cube3,p_cube4)
print weights

# Compute the new trajectories
traj1=np.concatenate((primitive_1[:,[0]],primitive_1[:,[1]],np.zeros((1000,1)),primitive_1[:,[2]], np.zeros((1000,1)),primitive_1[:,[3]],np.multiply(np.ones((1000,1)),1.5708)),axis=1)
traj2=np.concatenate((primitive_2[:,[0]],primitive_2[:,[1]],np.zeros((1000,1)),primitive_2[:,[2]], np.zeros((1000,1)),primitive_2[:,[3]],np.multiply(np.ones((1000,1)),1.5708)),axis=1)
traj3=np.concatenate((primitive_3[:,[0]],primitive_3[:,[1]],np.zeros((1000,1)),primitive_3[:,[2]], np.zeros((1000,1)),primitive_3[:,[3]],np.multiply(np.ones((1000,1)),1.5708)),axis=1)
traj4=np.concatenate((primitive_4[:,[0]],primitive_4[:,[1]],np.zeros((1000,1)),primitive_4[:,[2]], np.zeros((1000,1)),primitive_4[:,[3]],np.multiply(np.ones((1000,1)),1.5708)),axis=1)

# Aply the weights to the MPs
traj_final_1=np.add(np.multiply(traj1,weights[0]),np.multiply(traj2,weights[1]))
traj_final_2=np.add(np.multiply(traj3,weights[2]),np.multiply(traj4,weights[3]))
traj_final=np.add(traj_final_1,traj_final_2)

# Add the gripper information
traj_final=np.concatenate((traj_final,np.multiply(np.ones((1000,1)),0.0402075604203)),axis=1)

# Add time information
max_time=20
time=np.linspace(0, max_time,1000).reshape((1000,1))
traj_final=np.concatenate((time,traj_final),axis=1)

# Save trajectory
np.savetxt('traj_final.txt', traj_final, delimiter=',',header='time,right_j0,right_j1,right_j2,right_j3,right_j4,right_j5,right_j6,right_gripper',comments='',fmt="%1.12f")

# Send the trajectory to the controller
rospy.sleep(1)
subprocess.Popen(['rosrun', 'intera_examples', 'joint_trajectory_file_playback.py','-f','traj_final.txt'], stdout=subprocess.PIPE)
rospy.sleep(1)

#Close the gripper
gripper.close()



