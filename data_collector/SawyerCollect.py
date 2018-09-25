#!/usr/bin/env python2

# Script for Kinesthetic Teaching with the Sawyer Robot

#import libraries

import numpy as np
import math
import rospy
import roslib
import intera_interface
from intera_interface import CHECK_VERSION
from intera_examples import JointRecorder
import os 
import subprocess

import importlib

from std_msgs.msg import Empty

from intera_interface import (
    Gripper,
    Cuff,
)

class GripperConnect(object):

    def __init__(self, arm):

        self._arm = arm
        # inputs
        self._cuff = Cuff(limb=arm)
        self._gripper_position=[]
        self._max_gripper_position=0.01
        self._toogle=-1;
        self._recorder=[]
        self._data_number=1
        self._path='/home/cloud/code/learn-dmp/data_collector/'

        try:
            self._gripper = Gripper(arm)
            if not (self._gripper.is_calibrated() or
                    self._gripper.calibrate() == True):
                rospy.logerr("({0}_gripper) calibration failed.".format(self._gripper.name))
                raise
            self._cuff.register_callback(self._record_action,'{0}_button_upper'.format(arm))
            self._cuff.register_callback(self._open_action,'{0}_button_lower'.format(arm))
            rospy.loginfo("{0} Cuff Control initialized...".format(self._gripper.name))
        except:
            self._gripper = None
            msg = ("{0} Gripper is not connected to the robot").format(arm.capitalize())
            rospy.logwarn(msg)

    def _open_action(self, value):
    	self._position=self._gripper.get_position()

        if value and self._gripper.is_ready():

			if self._position>self._max_gripper_position:
				rospy.logdebug("gripper open triggered")
				self._gripper.close()
				print 'gripper is closed'
				print self._position
			else:
				rospy.logdebug("gripper close triggered")
				self._gripper.open()
				print 'gripper is opened'
				print self._position

    def _record_action(self, value):

    	if value and self._gripper.is_ready():
    		self._toogle=(-1)*self._toogle

    		if self._toogle==1:
    			print 'Opening a new file named demo_'+str(self._data_number)
    			self._recorder=subprocess.Popen(['rosrun', 'intera_examples', 'joint_recorder.py','-f',self._path+'demo'+str(self._data_number)+'.txt'], stdout=subprocess.PIPE)
    		if self._toogle==-1:
    			print self._toogle
    			print 'Close the file'
    			self._data_number=self._data_number+1
    			self._recorder.kill()
    			
class JointSprings(object):
    def __init__(self,limb = "right"):

        # control parameters
        self._rate = 1000.0  # Hz
        self._missed_cmds = 20.0  # Missed cycles before triggering timeout

        # create our limb instance
        self._limb = intera_interface.Limb(limb)

        # initialize parameters
        #self._springs = {'right_j6': 50, 'right_j5': 4, 'right_j4': 100.0, 'right_j3': 0, 'right_j2': 1000.0, 'right_j1': 0, 'right_j0': 0}
        #self._damping = {'right_j6': 0.9, 'right_j5': 0.09, 'right_j4': 0.9, 'right_j3': 0, 'right_j2': 0.9, 'right_j1': 0, 'right_j0': 0}
        self._springs = {'right_j6': 0, 'right_j5': 0, 'right_j4': 100, 'right_j3': 0, 'right_j2': 100, 'right_j1': 0, 'right_j0': 0}
        self._damping = {'right_j6': 0, 'right_j5': 0, 'right_j4': 0.5, 'right_j3': 0, 'right_j2': 0.5, 'right_j1': 0, 'right_j0': 0}
        self._start_angles = dict()
        self._angles = dict()

        # create cuff disable publisher
        cuff_ns = 'robot/limb/' + limb + '/suppress_cuff_interaction'
        self._pub_cuff_disable = rospy.Publisher(cuff_ns, Empty, queue_size=1)

        # verify robot is enabled
        print("Getting robot state... ")
        self._rs = intera_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()
        print("Running. Ctrl-c to quit")

    def _update_forces(self):

        # disable cuff interaction
        self._pub_cuff_disable.publish()

        # create our command dict
        cmd = dict()
        # record current angles/velocities
        cur_pos = self._limb.joint_angles()
        cur_vel = self._limb.joint_velocities()
        # calculate current forces
        for joint in self._start_angles.keys():
            # spring portion
            cmd[joint] = self._springs[joint] * (self._start_angles[joint] -cur_pos[joint])
            # damping portion
            cmd[joint] -= self._damping[joint] * cur_vel[joint]
        # command new joint torques
        #print cmd
        self._limb.set_joint_torques(cmd)

    def move_to_neutral(self):
    	self._angles['right_j0']=math.radians(0)
    	self._angles['right_j1']=math.radians(-50)
    	self._angles['right_j2']=math.radians(0)
    	self._angles['right_j3']=math.radians(120)
    	self._angles['right_j4']=math.radians(0)
    	self._angles['right_j5']=math.radians(0)
    	self._angles['right_j6']=math.radians(0)
    	self._limb.move_to_joint_positions(self._angles)

    def attach_springs(self):

        # record initial joint angles
        self._start_angles = self._limb.joint_angles()

        # set control rate
        control_rate = rospy.Rate(self._rate)

        # for safety purposes, set the control rate command timeout.
        # if the specified number of command cycles are missed, the robot
        # will timeout and return to Position Control Mode
        self._limb.set_command_timeout((1.0 / self._rate) * self._missed_cmds)

        # loop at specified rate commanding new joint torques
        while not rospy.is_shutdown():
            if not self._rs.state().enabled:
                rospy.logerr("Joint torque example failed to meet "
                             "specified control rate timeout.")
                break
            self._update_forces()
            control_rate.sleep()

    def clean_shutdown(self):

        print("\nExiting example...")
        self._limb.exit_control_mode()

print("Initializing node... ")
rospy.init_node("sdk_joint_torque_springs")

# Initialize the Spring Class
js = JointSprings(limb='right')

# Initialize the Cuff Control Class
GripperConnect(arm='right')

# Shutdown callback
rospy.on_shutdown(js.clean_shutdown)

# Move the Robot to a neutral position
js.move_to_neutral()

# Attach the Springs
js.attach_springs()


    
