#!/usr/bin/env python

import numpy
import random
import sys

import geometry_msgs.msg
import moveit_msgs.msg
import moveit_msgs.srv
import rospy
import tf
import moveit_commander
from urdf_parser_py.urdf import URDF
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Transform
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def convert_to_message(T):
    t = geometry_msgs.msg.Pose()
    position = tf.transformations.translation_from_matrix(T)
    orientation = tf.transformations.quaternion_from_matrix(T)
    t.position.x = position[0]
    t.position.y = position[1]
    t.position.z = position[2]
    t.orientation.x = orientation[0]
    t.orientation.y = orientation[1]
    t.orientation.z = orientation[2]
    t.orientation.w = orientation[3]        
    return t

class MoveArm(object):

    def __init__(self):

        #Loads the robot model, which contains the robot's kinematics information
	self.num_joints = 0
        self.joint_names = []
        self.joint_axes = []
        self.robot = URDF.from_parameter_server()
        self.base = self.robot.get_root()
        self.get_joint_info()

        # Wait for moveit IK service
        rospy.wait_for_service("compute_ik")
        self.ik_service = rospy.ServiceProxy('compute_ik',  moveit_msgs.srv.GetPositionIK)
        print "IK service ready"

        # Wait for validity check service
        rospy.wait_for_service("check_state_validity")
        self.state_valid_service = rospy.ServiceProxy('check_state_validity',  
                                                      moveit_msgs.srv.GetStateValidity)
        print "State validity service ready"

        # MoveIt parameter
        robot_moveit = moveit_commander.RobotCommander()
        self.group_name = robot_moveit.get_group_names()[0]

	#Subscribe to topics
	rospy.Subscriber('/joint_states', JointState, self.get_joint_state)
	rospy.Subscriber('/motion_planning_goal', Transform, self.motion_planning)
        self.current_obstacle = "None"
        rospy.Subscriber('/obstacle', String, self.get_obstacle)

	#Set up publisher
	self.pub = rospy.Publisher('/joint_trajectory', JointTrajectory, queue_size=10)
	self.joint_trajectory_msg = JointTrajectory()

    '''This callback provides you with the current joint positions of the robot 
     in member variable q_current.'''
    def get_joint_state(self, msg):
        self.q_current = []
        for name in self.joint_names:
            self.q_current.append(msg.position[msg.name.index(name)])

    '''This callback provides you with the name of the current obstacle which
    exists in the RVIZ environment. Options are "None", "Simple", "Hard",
    or "Super". '''
    def get_obstacle(self, msg):
        self.current_obstacle = msg.data

    '''This is the callback which will implement your RRT motion planning.
    You are welcome to add other functions to this class (i.e. an
    "is_segment_valid" function will likely come in handy multiple times 
    in the motion planning process and it will be easiest to make this a 
    seperate function and then call it from motion planning). You may also
    create trajectory shortcut and trajectory sample functions if you wish, 
    which will also be called from the motion planning function.'''    
    def motion_planning(self, ee_goal):
        print "Starting motion planning"
	########INSERT YOUR RRT MOTION PLANNING HERE##########
	trans = numpy.empty(3)
	trans[0] = ee_goal.translation.x
	trans[1] = ee_goal.translation.y
	trans[2] = ee_goal.translation.z
	rot = numpy.empty(4)
	rot[0] = ee_goal.rotation.x
	rot[1] = ee_goal.rotation.y
	rot[2] = ee_goal.rotation.z
	rot[3] = ee_goal.rotation.w
	T_trans = tf.transformations.translation_matrix(trans)
	T_rot = tf.transformations.quaternion_matrix(rot)
	T_goal = numpy.dot(T_trans, T_rot)
	q_goal = numpy.array(self.IK(T_goal))
	q_c = numpy.array(self.q_current)
	tree = []
	tree.append(q_c)
	treelist = []

	while self.is_segment_valid(q_c, q_goal) is False:
	    # get a ramdom point
	    print("yes")
	    p = numpy.empty(self.num_joints)
	    for i in range(self.num_joints):
		p[i] = numpy.random.uniform(-numpy.pi, numpy.pi)
	    
	    # find the nearest point in the tree
	    l_0 = 999999999
	    for i in range(len(tree)):
		v = p - tree[i]
		l = numpy.linalg.norm(v)
		if l <= l_0:
		    l_0 = l
		    p_near = tree[i]

	    # ensure no obstacle between p_nearest and p
	    #if self.is_segment_valid(p_near, p) is False:
		#continue

	    # find the next node on the tree
	    vector = p - p_near
	    p_next = p_near + 0.2 * vector / numpy.linalg.norm(vector)
	    if self.is_segment_valid(p_near, p_next) == False:
		continue
		#p_next = p_next - 0.1 * vector / numpy.linalg.norm(vector)
	    tree.append(p_next)
	    treelist.append(RRTBranch(p_near, p_next))
	    q_c = p_next
	tree.append(q_goal)
	treelist.append(RRTBranch(q_c, q_goal))

	# find path to the goal
	path_i = []
	p_q = q_goal
	path_i.append(p_q)
	i = 1
	while True: 
	    if numpy.linalg.norm(p_q) == numpy.linalg.norm(self.q_current):
		break
	    while numpy.linalg.norm(treelist[-i].q) != numpy.linalg.norm(p_q):
		i += 1
	    p_q = treelist[-i].parent
	    path_i.append(p_q)
	path_i.append(numpy.array(self.q_current))
	path = path_i[::-1]
	print(path)
	
	# shortcut
	#i = 1
	#q_n = numpy.array(self.q_current)
	#print(q_n)
	#path_s = []
	#path_s.append(numpy.array(self.q_current))
	#while True:
	 #   while self.is_segment_valid(q_n, path[i]):
	#	print("ok")
	 #       if numpy.linalg.norm(path[i]) == numpy.linalg.norm(q_goal):
	#	    break
	 #       i += 1
	  #  if numpy.linalg.norm(path[i]) == numpy.linalg.norm(q_goal):
	#	break
	 #   q_n = path[i-1]
	  #  print(i)
	   # print("good")
	    #path_s.append(path[i-1])
	#path_s.append(q_goal)
	#print(path_s)

	# resample nodes
	path_n = []
	#path_n.append(numpy.array(self.q_current))
	for i in range(1,len(path)):
	    path_n.append(path[i-1])
	    vector = path[i] - path[i-1]
	    length = numpy.linalg.norm(vector)
	    unit_vector = vector / length
	    step = length / 0.1
	    step = int(step)
	    for j in range(step+1):
		p_n = path[i-1] + j * 0.1 * unit_vector
		path_n.append(p_n)
	path_n.append(q_goal)
	#print(path_n)

	# publish
        joint_trajectory_msg = JointTrajectory()
        for j in range(len(path_n)):
            point = JointTrajectoryPoint()
            point.positions = path_n[j]
            joint_trajectory_msg.points.append(point)
        joint_trajectory_msg.joint_names = self.joint_names
        self.pub.publish(joint_trajectory_msg)

        ######################################################

    def is_segment_valid(self, q_c, q_g):
	vector = q_g - q_c
	d = numpy.linalg.norm(vector)
	n = d / 0.1
	if d == 0:
	    return True
	#n = round(n)
	n = int(n)
	for i in range(1, n+2):
	    qq = q_c + numpy.dot(0.1 * i, vector / d)
	    if self.is_state_valid(qq) == False:
		return False
		break
	if self.is_state_valid(qq) == True:
	    return True
     ######################################################
    """ This function will perform IK for a given transform T of the end-effector.
    It returns a list q[] of values, which are the result positions for the 
    joints of the robot arm, ordered from proximal to distal. If no IK solution 
    is found, it returns an empy list.
    """
    def IK(self, T_goal):
        req = moveit_msgs.srv.GetPositionIKRequest()
        req.ik_request.group_name = self.group_name
        req.ik_request.robot_state = moveit_msgs.msg.RobotState()
        req.ik_request.robot_state.joint_state.name = self.joint_names
        req.ik_request.robot_state.joint_state.position = numpy.zeros(self.num_joints)
        req.ik_request.robot_state.joint_state.velocity = numpy.zeros(self.num_joints)
        req.ik_request.robot_state.joint_state.effort = numpy.zeros(self.num_joints)
        req.ik_request.robot_state.joint_state.header.stamp = rospy.get_rostime()
        req.ik_request.avoid_collisions = True
        req.ik_request.pose_stamped = geometry_msgs.msg.PoseStamped()
        req.ik_request.pose_stamped.header.frame_id = self.base
        req.ik_request.pose_stamped.header.stamp = rospy.get_rostime()
        req.ik_request.pose_stamped.pose = convert_to_message(T_goal)
        req.ik_request.timeout = rospy.Duration(3.0)
        res = self.ik_service(req)
        q = []
        if res.error_code.val == res.error_code.SUCCESS:
            q = res.solution.joint_state.position
        return q

    '''This is a function which will collect information about the robot which
       has been loaded from the parameter server. It will populate the variables
       self.num_joints (the number of joints), self.joint_names and
       self.joint_axes (the axes around which the joints rotate)'''
    def get_joint_info(self):
        link = self.robot.get_root()
        while True:
            if link not in self.robot.child_map: break
            (joint_name, next_link) = self.robot.child_map[link][0]
            current_joint = self.robot.joint_map[joint_name]
            if current_joint.type != 'fixed':
                self.num_joints = self.num_joints + 1
                self.joint_names.append(current_joint.name)
                self.joint_axes.append(current_joint.axis)
            link = next_link


    """ This function checks if a set of joint angles q[] creates a valid state,
    or one that is free of collisions. The values in q[] are assumed to be values
    for the joints of the KUKA arm, ordered from proximal to distal. 
    """
    def is_state_valid(self, q):
        req = moveit_msgs.srv.GetStateValidityRequest()
        req.group_name = self.group_name
        req.robot_state = moveit_msgs.msg.RobotState()
        req.robot_state.joint_state.name = self.joint_names
        req.robot_state.joint_state.position = q
        req.robot_state.joint_state.velocity = numpy.zeros(self.num_joints)
        req.robot_state.joint_state.effort = numpy.zeros(self.num_joints)
        req.robot_state.joint_state.header.stamp = rospy.get_rostime()
        res = self.state_valid_service(req)
        return res.valid


'''This is a class which you can use to keep track of your tree branches.
It is easiest to do this by appending instances of this class to a list 
(your 'tree'). The class has a parent field and a joint position field (q). 
You can initialize a new branch like this:
RRTBranch(parent, q)
Feel free to keep track of your branches in whatever way you want - this
is just one of many options available to you.'''
class RRTBranch(object):
    def __init__(self, parent, q):
	self.parent = parent
	self.q = q


if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_arm', anonymous=True)
    ma = MoveArm()
    rospy.spin()

