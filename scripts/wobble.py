#!/usr/bin/env python

# Copyright (c) 2015-2018, Rethink Robotics Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Sawyer SDK Inverse Kinematics
"""
import argparse
import struct
import sys
import copy
import os
import math
import time
import random
import rospy
import rospkg
import subprocess, signal

from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
)
from std_msgs.msg import (
    UInt16,
)

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

import intera_interface

from intera_interface import CHECK_VERSION

class Wobbler(object):
     
     def __init__(self):
        """
        'Wobbles' both arms by commanding joint velocities sinusoidally.
        """
        self._pub_rate = rospy.Publisher('robot/joint_state_publish_rate',
                                         UInt16, queue_size=10)
        self._right_arm = intera_interface.limb.Limb("right")
        self._gripper = intera_interface.Gripper()
        self._tip_name = "right_gripper_tip"
        self._right_joint_names = self._right_arm.joint_names()
        
        # control parameters
        self._rate = 500.0  # Hz
        
        print("Getting robot state... ")
        self._rs = intera_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()
       
        # set joint state publishing to 500Hz
        self._pub_rate.publish(self._rate)
            
     def move_to_start(self, start_angles=None):
		 print("Moving the {0} arm to start pose...right limb")
		 if not start_angles:
			 start_angles = dict(zip(self._joint_names, [0]*7))
		 self._guarded_move_to_joint_position(start_angles)
		 self.gripper_open()

     def _guarded_move_to_joint_position(self, joint_angles, timeout=5.0):
		if rospy.is_shutdown():
			return
		if joint_angles:
			self._right_arm.move_to_joint_positions(joint_angles,timeout=timeout)
		else:
			rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")
			
     def gripper_open(self):
		self._gripper.open()
		rospy.sleep(1.0)

     def gripper_close(self):
		self._gripper.close()
		rospy.sleep(1.0)

     def _approach(self, pose):
		approach = copy.deepcopy(pose)
		# approach with a pose the hover-distance above the requested pose
		approach.position.z = approach.position.z + 0.07
		joint_angles = self._right_arm.ik_request(approach, self._tip_name)
		self._right_arm.set_joint_position_speed(0.001)
		self._guarded_move_to_joint_position(joint_angles)
		self._right_arm.set_joint_position_speed(0.1)

   
     def _servo_to_pose(self, pose, time=4.0, steps=400.0):
		''' An *incredibly simple* linearly-interpolated Cartesian move '''
		r = rospy.Rate(1/(time/steps)) # Defaults to 100Hz command rate
		current_pose = self._right_arm.endpoint_pose()
		ik_delta = Pose()
		ik_delta.position.x = (current_pose['position'].x - pose.position.x) / steps
		ik_delta.position.y = (current_pose['position'].y - pose.position.y) / steps
		ik_delta.position.z = (current_pose['position'].z - pose.position.z) / steps
		ik_delta.orientation.x = (current_pose['orientation'].x - pose.orientation.x) / steps
		ik_delta.orientation.y = (current_pose['orientation'].y - pose.orientation.y) / steps
		ik_delta.orientation.z = (current_pose['orientation'].z - pose.orientation.z) / steps
		ik_delta.orientation.w = (current_pose['orientation'].w - pose.orientation.w) / steps
		
		for d in range(int(steps), -1, -1):
			if rospy.is_shutdown():
				return
			ik_step = Pose()
			ik_step.position.x = d*ik_delta.position.x + pose.position.x
			ik_step.position.y = d*ik_delta.position.y + pose.position.y
			ik_step.position.z = d*ik_delta.position.z + pose.position.z
			ik_step.orientation.x = d*ik_delta.orientation.x + pose.orientation.x
			ik_step.orientation.y = d*ik_delta.orientation.y + pose.orientation.y
			ik_step.orientation.z = d*ik_delta.orientation.z + pose.orientation.z
			ik_step.orientation.w = d*ik_delta.orientation.w + pose.orientation.w
			joint_angles = self._right_arm.ik_request(ik_step, self._tip_name)
			if joint_angles:
				self._right_arm.set_joint_positions(joint_angles)
			else:
				rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")
			r.sleep()
		rospy.sleep(1.0)

     def pick(self, pose):
		# open the gripper
		self.gripper_open()
		# servo above pose
		self._approach(pose)
		# servo to pose
		self._servo_to_pose(pose)
		# close gripper
		self.gripper_close()
		#start to record
		#rosbag_process = start_rosbag_recording(filename)
		self._approach(pose)
		#return rosbag_process
    
     def place(self, pose):
        # servo above pose
		self._approach(pose)
		# servo to pose
		self._servo_to_pose(pose)
		# open the gripper
		self.gripper_open()
		# stop rosbag recording
		#stop_rosbag_recording(rosbag_process)
		self._approach(pose)
    
     def _reset_control_modes(self):
		rate = rospy.Rate(self._rate)
		for _ in xrange(100):
			if rospy.is_shutdown():
				return False
			self._right_arm.exit_control_mode()
			self._pub_rate.publish(100)  # 100Hz default joint state rate
			rate.sleep()
		return True

     def set_neutral(self):
		"""
		Sets both arms back into a neutral pose.
		"""
		print("Moving to neutral pose...")
		self._right_arm.move_to_neutral()

     def clean_shutdown(self):
		print("\nExiting example...")
		#return to normal
		self._reset_control_modes()
		self.set_neutral()
		if not self._init_state:
			print("Disabling robot...")
			self._rs.disable()
		return True
    
     def wobble(self,nm):
        
        fn2 = "sawyer_hold_model_" + str(nm)
        self.set_neutral()
        rp = start_rosbag_recording(fn2)
        time.sleep(5.0)
        stop_rosbag_recording(rp)
        
        """
        Performs the wobbling of both arms.
        """
        rate = rospy.Rate(self._rate)
        start = rospy.Time.now()

        def make_v_func():
            """
            returns a randomly parameterized cosine function to control a
            specific joint.
            """
            period_factor = random.uniform(0.3, 0.5)
            amplitude_factor = 0.1#random.uniform(0.1, 0.2)

            def v_func(elapsed):
                w = period_factor * elapsed.to_sec()
                return amplitude_factor * math.cos(w * 2 * math.pi)
            return v_func

        v_funcs = [make_v_func() for _ in self._right_joint_names]

        def make_cmd(joint_names, elapsed):
            return dict([(joint, v_funcs[i](elapsed))
                         for i, joint in enumerate(joint_names)])

        print("started shaking block....." + str(nm))
        n = 5000
        fn1 = "sawyer_shake_model_" + str(nm)
        rosbag_process = start_rosbag_recording(fn1)
        while n>0:
            self._pub_rate.publish(self._rate)
            elapsed = rospy.Time.now() - start
            cmd = make_cmd(self._right_joint_names, elapsed)
            self._right_arm.set_joint_velocities(cmd)
            n = n-1
            rate.sleep()
            
        print("done shaking block....." + str(nm))
        stop_rosbag_recording(rosbag_process)	
		

def load_gazebo_models(box_no = 4, table_pose=Pose(position=Point(x=0.75, y=0.0, z=0.0)),
                       table_reference_frame="world",
                       block_pose=Pose(position=Point(x=0.4225, y=0.1265, z=0.7725)),
                       block_reference_frame="world"):
    
    # Get Models' Path
    script_path = os.path.dirname(os.path.abspath(__file__)) 
    model_path = script_path[:-8]+"/models/"
    
    # Load Table SDF
    table_xml = ''
    with open (model_path + "cafe_table/model.sdf", "r") as table_file:
        table_xml=table_file.read().replace('\n', '')
    # Load Block URDF
    block_xml = ''
    block_path = "block/model"+ str(box_no) + ".urdf"
    with open (model_path + block_path, "r") as block_file:
        block_xml=block_file.read().replace('\n', '')
    # Spawn Table SDF
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp_sdf = spawn_sdf("cafe_table", table_xml, "/",
                             table_pose, table_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))
    # Spawn Block URDF
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        resp_urdf = spawn_urdf("block", block_xml, "/",
                               block_pose, block_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn URDF service call failed: {0}".format(e))

def delete_gazebo_models():
    # This will be called on ROS Exit, deleting Gazebo models
    # Do not wait for the Gazebo Delete Model service, since
    # Gazebo should already be running. If the service is not
    # available since Gazebo has been killed, it is fine to error out
    try:
        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        resp_delete = delete_model("cafe_table")
        resp_delete = delete_model("block")
    except rospy.ServiceException, e:
        print("Delete Model service call failed: {0}".format(e))

def load_gazebo_block(box_no = 4, block_pose=Pose(position=Point(x=0.4225, y=0.1265, z=0.7725)),
                       block_reference_frame="world"):
    # Get Models' Path
    script_path = os.path.dirname(os.path.abspath(__file__)) 
    model_path = script_path[:-8]+"/models/"
     
    # Load Blocks URDF
    block_xml = ''
    block_path = "block/model"+ str(box_no) + ".urdf"
    with open (model_path + block_path, "r") as block_file:
        block_xml=block_file.read().replace('\n', '')
  
    # Spawn Block URDF
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        resp_urdf = spawn_urdf("block", block_xml, "/",
                               block_pose, block_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn URDF service call failed: {0}".format(e))

def delete_gazebo_block():
    # This will be called on ROS Exit, deleting Gazebo models
    # Do not wait for the Gazebo Delete Model service, since
    # Gazebo should already be running. If the service is not
    # available since Gazebo has been killed, it is fine to error out
    try:
        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        resp_delete = delete_model("block")
    except rospy.ServiceException, e:
        rospy.loginfo("Delete Model service call failed: {0}".format(e))


def start_rosbag_recording(filename):
    # find the directory to save to
    rospy.loginfo(rospy.get_name() + ' start')
    script_path = os.path.dirname(os.path.abspath(__file__))
    rosbagfile_dir = script_path[:-8]+"/rosbagfiles/"
    
    #  modify the rosbag process with prof Jivko
    rosbag_process = subprocess.Popen('rosbag record -o {} /robot/joint_states'.format(filename), stdin=subprocess.PIPE, shell=True, cwd= rosbagfile_dir)
    return rosbag_process
    
def stop_rosbag_recording(p):
    rospy.loginfo(rospy.get_name() + ' stop recording.')
    rospy.loginfo(p.pid)
    
    import psutil
    process = psutil.Process(p.pid)
    for sub_process in process.children(recursive=True):
        sub_process.send_signal(signal.SIGINT)
    p.wait()  # we wait for children to terminate
    
    rospy.loginfo("I'm done")
    
def main():
  
    rospy.init_node("rsdk_joint_velocity_wobbler")
    
    #parse argument
    myargv = rospy.myargv(argv=sys.argv)
    filename = "sawyer_wobbler__model"+ str(myargv[1])+"_"
    num_of_run = int(myargv[1])
    
    # Load Gazebo Models via Spawning Services
    # Note that the models reference is the /world frame
    # and the IK operates with respect to the /base frame
    
    load_gazebo_models(7)
    # Remove models from the scene on shutdown
    rospy.on_shutdown(delete_gazebo_models)
    
    limb = 'right'
    hover_distance = 0.15 # meters
    # Starting Joint angles for right arm
    starting_joint_angles = {'right_j0': -0.041662954890248294,
                             'right_j1': -1.0258291091425074,
                             'right_j2': 0.0293680414401436,
                             'right_j3': 2.17518162913313,
                             'right_j4':  -0.06703022873354225,
                             'right_j5': 0.3968371433926965,
                             'right_j6': 1.7659649178699421}
    
    # An orientation for gripper fingers to be overhead and parallel to the obj
    overhead_orientation = Quaternion(
                             x=-0.00142460053167,
                             y=0.999994209902,
                             z=-0.00177030764765,
                             w=0.00253311793936)
  
    block_pose = Pose(position= Point(x=0.45, y=0.155, z=-0.145), orientation=overhead_orientation)
  
    wobbler = Wobbler()
    rospy.on_shutdown(wobbler.clean_shutdown)
    
    for y in range(7,19):
		for x in range(0,num_of_run):
			if(not rospy.is_shutdown()):
				wobbler.move_to_start(starting_joint_angles)
				wobbler.pick(block_pose)
				rospy.on_shutdown(wobbler.clean_shutdown)
				wobbler.wobble(y)
				wobbler.place(block_pose)
				wobbler.move_to_start(starting_joint_angles)
				delete_gazebo_block()
				load_gazebo_block(y)
			else:
			   break
            
    print("Done.")

    return 0

if __name__ == '__main__':
    sys.exit(main())
