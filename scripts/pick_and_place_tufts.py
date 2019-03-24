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
Sawyer SDK Inverse Kinematics Pick and Place Demo
"""
import argparse
import struct
import sys
import copy
import os
import rospy
import random
import rospkg
import time
import subprocess, signal

from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
)
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

import intera_interface

class PickAndPlace(object):
    def __init__(self, limb="right", hover_distance = 0.15, tip_name="right_gripper_tip"):
        self._limb_name = limb # string
        self._tip_name = tip_name # string
        self._hover_distance = hover_distance # in meters
        self._limb = intera_interface.Limb(limb)
        self._gripper = intera_interface.Gripper()
        # verify robot is enabled
        print("Getting robot state... ")
        self._rs = intera_interface.RobotEnable(intera_interface.CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()
    
    def move_to_start(self, start_angles=None):
		print("Moving the {0} arm to start pose...".format(self._limb_name))
		if not start_angles:
			start_angles = dict(zip(self._joint_names, [0]*7))
		self._guarded_move_to_joint_position(start_angles)
		self.gripper_open()

    def _guarded_move_to_joint_position(self, joint_angles, timeout=5.0):
        if rospy.is_shutdown():
            return
        if joint_angles:
            self._limb.move_to_joint_positions(joint_angles,timeout=timeout)
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
        approach.position.z = approach.position.z + self._hover_distance
        joint_angles = self._limb.ik_request(approach, self._tip_name)
        self._limb.set_joint_position_speed(0.001)
        self._guarded_move_to_joint_position(joint_angles)
        self._limb.set_joint_position_speed(0.1)

    def _retract(self):
        # retrieve current pose from endpoint
        current_pose = self._limb.endpoint_pose()
        ik_pose = Pose()
        ik_pose.position.x = current_pose['position'].x
        ik_pose.position.y = current_pose['position'].y
        ik_pose.position.z = current_pose['position'].z + self._hover_distance
        ik_pose.orientation.x = current_pose['orientation'].x
        ik_pose.orientation.y = current_pose['orientation'].y
        ik_pose.orientation.z = current_pose['orientation'].z
        ik_pose.orientation.w = current_pose['orientation'].w
        self._servo_to_pose(ik_pose)

    def _servo_to_pose(self, pose, time=4.0, steps=400.0):
        ''' An *incredibly simple* linearly-interpolated Cartesian move '''
        r = rospy.Rate(1/(time/steps)) # Defaults to 100Hz command rate
        current_pose = self._limb.endpoint_pose()
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
            joint_angles = self._limb.ik_request(ik_step, self._tip_name)
            if joint_angles:
                self._limb.set_joint_positions(joint_angles)
            else:
                rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")
            r.sleep()
        rospy.sleep(1.0)

    def pick(self, pose, filename):
		fn = "sawyer_grasp_model_" + filename
		filename = "sawyer_pick_model_" + filename
		rps = start_rosbag_recording(fn)
		time.sleep(0.5)
		self.gripper_open()
		self._approach(pose)
		self._servo_to_pose(pose)
		self.gripper_close()
		stop_rosbag_recording(rps)
		rosbag_process = start_rosbag_recording(filename)
		time.sleep(0.5)
		self._approach(pose)
		stop_rosbag_recording(rosbag_process)
 
    
    def place(self, pose, filename):
		filename = "sawyer_place_model_" + filename
		rosbag_process = start_rosbag_recording(filename)
		self._approach(pose)
		self._servo_to_pose(pose)
		self.gripper_open()
		stop_rosbag_recording(rosbag_process)
		self._approach(pose)

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
def addnoise_pose(overhead_orientation):
	pose = Pose(position= Point(x=0.45, y=0.155, z=-0.129), orientation=overhead_orientation)
	x = random.uniform(-0.09, 0.09)
	y = random.uniform(-0.09, 0.09)
	pose.position.x = pose.position.x + x
	pose.position.y = pose.position.y + y
	return pose
	   
def main():
    """SDK Inverse Kinematics Pick and Place Example

    A Pick and Place example using the Rethink Inverse Kinematics
    Service which returns the joint angles a requested Cartesian Pose.
    This ROS Service client is used to request both pick and place
    poses in the /base frame of the robot.

    Note: This is a highly scripted and tuned demo. The object location
    is "known" and movement is done completely open loop. It is expected
    behavior that Sawyer will eventually mis-pick or drop the block. You
    can improve on this demo by adding perception and feedback to close
    the loop.
    """
    rospy.init_node("pick_and_place_tufts")
    
    #parse argument
    myargv = rospy.myargv(argv=sys.argv)
    num_of_run = int(myargv[1])
    
    # Load Gazebo Models via Spawning Services
    # Note that the models reference is the /world frame
    # and the IK operates with respect to the /base frame
    
    limb = 'right'
    hover_distance = 0.07 # meters
    # Starting Joint angles for right arm
    starting_joint_angles = {'right_j0': -0.041662954890248294,
                             'right_j1': -1.0258291091425074,
                             'right_j2': 0.0293680414401436,
                             'right_j3': 2.17518162913313,
                             'right_j4':  -0.06703022873354225,
                             'right_j5': 0.3968371433926965,
                             'right_j6': 1.7659649178699421}
    
    pnp = PickAndPlace(limb, hover_distance)
    # An orientation for gripper fingers to be overhead and parallel to the obj
    overhead_orientation = Quaternion(
                             x=-0.00142460053167,
                             y=0.999994209902,
                             z=-0.00177030764765,
                             w=0.00253311793936)
  
    block_pose = Pose(position= Point(x=0.45, y=0.155, z=-0.145), orientation=overhead_orientation)
  
    pnp.move_to_start(starting_joint_angles)
    filename = str(0)
    load_gazebo_models(filename)
    # Remove models from the scene on shutdown
    rospy.on_shutdown(delete_gazebo_models)
    
    for x in range(0,12):
		filename = str(x)
		for y in range(0,num_of_run):
			if(not rospy.is_shutdown()):
				pnp.pick(block_pose, filename)
				tmp = addnoise_pose(overhead_orientation)
				pnp.place(tmp, filename)
				pnp.gripper_open()
				pnp.move_to_start(starting_joint_angles)
				delete_gazebo_block()
				load_gazebo_block(filename)
			else:
				break
   
    return 0

if __name__ == '__main__':
    sys.exit(main())
