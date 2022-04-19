#!/usr/bin/env python

# Author: 	Tony Willett
# Date:		25th March 2022
# Description: 	A stand alone ROS node to turn Twist messages into cartesian control of franka arm end effector.
# 		Based on 'move_group_python_interface_tutorial.py'.

## To use the Python MoveIt interfaces, we will import the `moveit_commander`_ namespace.
## This namespace provides us with a `MoveGroupCommander`_ class, a `PlanningSceneInterface`_ class,
## and a `RobotCommander`_ class.
##
## We also import `rospy`_ and some messages that we will use:
##

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import Twist
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

# The following function was copied directly from original.
def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True


class FrankaArmKeyboardControl(object):
  """FrankaArmKeyboardControl"""
  def __init__(self):
    super(FrankaArmKeyboardControl, self).__init__()

    ## BEGIN_SUB_TUTORIAL setup
    ##
    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_panda_keyboard_controller',
                    anonymous=True)

    ## Instantiate a `RobotCommander`_ object. This object is the outer-level interface to
    ## the robot:
    robot = moveit_commander.RobotCommander()

    ## Instantiate a `PlanningSceneInterface`_ object.  This object is an interface
    ## to the world surrounding the robot:
    scene = moveit_commander.PlanningSceneInterface()

    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    ## to one group of joints.  In this case the group is the joints in the Panda
    ## arm so we set ``group_name = panda_arm``. If you are using a different robot,
    ## you should change this value to the name of your robot arm planning group.
    ## This interface can be used to plan and execute motions on the Panda:
    group_name = "panda_arm"
    group = moveit_commander.MoveGroupCommander(group_name)

    ## We create a `DisplayTrajectory`_ publisher which is used later to publish
    ## trajectories for RViz to visualize:
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)
    ## END_SUB_TUTORIAL

    rospy.Subscriber("cmd_vel", Twist, self.cartesianControl) 
	
	## BEGIN_SUB_TUTORIAL basic_info
    ##
    ## Getting Basic Information
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^
    # We can get the name of the reference frame for this robot:
    planning_frame = group.get_planning_frame()
    print "============ Reference frame: %s" % planning_frame

    # We can also print the name of the end-effector link for this group:
    eef_link = group.get_end_effector_link()
    print "============ End effector: %s" % eef_link

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print"============ Robot Groups:", robot.get_group_names()

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    # print "============ Printing robot state"
    # print robot.get_current_state()
    # print ""
    ## END_SUB_TUTORIAL

    # Misc variables
    self.robot = robot
    self.scene = scene
    self.group = group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names


  def cartesianControl(self,message):
    # callback function to turn Twist messages into cartesian commands.

    # We combine angular and linear messages for richer control.
    moveX = message.linear.x + message.angular.x
    moveY = message.linear.y + message.angular.y
    moveZ = message.linear.z + message.angular.z
    # lets start by simply printing the recieved data to console.
    rospy.loginfo("x = %s", moveX)
    rospy.loginfo("y = %s", moveY)
    rospy.loginfo("z = %s", moveZ)
    
    
    group = self.group
    pose = group.get_current_pose().pose
    pose.position.x += moveX
    pose.position.y += moveY
    pose.position.z += moveZ
    group.set_pose_target(pose)

    plan = group.go(wait=True)
    group.stop


def main():
  try:
	rospy.loginfo("Starting Franka Arm Keyboard Controller")

	keyCon = FrankaArmKeyboardControl()
	
	#keyCon.

	# Keep ros running, looking for messages.
	rospy.spin()

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return


if __name__ == '__main__':
  main()
