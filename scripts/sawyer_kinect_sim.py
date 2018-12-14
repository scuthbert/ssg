#!/usr/bin/env python

import roslib
roslib.load_manifest('ssg')
import sys
import copy
import tf
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg as gm
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('sawyer_moveit_from_topic_sim',
                anonymous=True)

listener = tf.TransformListener()

robot = moveit_commander.RobotCommander()

scene = moveit_commander.PlanningSceneInterface()

group_name = "right_arm"
group = moveit_commander.MoveGroupCommander(group_name)
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)

group.set_planning_time(5)

BASE_X = 0.5
BASE_Y = 0.3
BASE_Z = 0.2
OR_X = 0.0
OR_Y = 0.0
OR_Z = -0.0
OR_W = 0.0
SCALING = 0.3

pose_goal = gm.Pose()
pose_goal.orientation.w = OR_W
pose_goal.orientation.x = OR_X
pose_goal.orientation.y = OR_Y
pose_goal.orientation.z = OR_Z
pose_goal.position.x = BASE_X
pose_goal.position.y = BASE_Y
pose_goal.position.z = BASE_Z
group.set_pose_target(pose_goal)

group.stop()
plan = group.go(wait=True)

group.set_planning_time(1)

rate = rospy.Rate(5.0)
while not rospy.is_shutdown():
    try:
        (trans, rot) = listener.lookupTransform('kinect/user_1/torso', 'kinect/user_1/right_hand', rospy.Time(0))
    except  (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        print 'excepted exception ' + str(type(e))
        continue

    print "setting goal state: " + str(trans)

    pose_goal = gm.Pose()
    pose_goal.orientation.w = OR_W
    pose_goal.orientation.x = OR_X
    pose_goal.orientation.y = OR_Y
    pose_goal.orientation.z = OR_Z
    trans = [SCALING*trans[0] + BASE_X, SCALING*trans[2] + BASE_Y, SCALING*trans[1] + BASE_Z]
    pose_goal.position.x = trans[0]
    pose_goal.position.y = trans[1]
    pose_goal.position.z = trans[2]
    group.set_pose_target(pose_goal)

    group.stop()
    plan = group.go(wait=False)
    rate.sleep()