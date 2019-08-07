import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

def get_basic_info(move_group, robot):
	
	# We can get the name of the reference frame for this robot:
	planning_frame = move_group.get_planning_frame()
	print ("========== Planning frame: %s" % planning_frame)
	
	# We can also print the name of the end-effector link for this group:
	eef_link = move_group.get_end_effector_link()
	print ("========== End effector link: %s" % eef_link)
	
	# We can get a list of all the groups in the robot:
	group_names = robot.get_group_names()
	print ("========== Available Planning Groups:", group_names)
	
	# Sometimes for debugging it is useful to print the entire state of 
	# the robot:
	
	print ("========== Printing robot state")
	print (robot.get_current_state())
	print ("")

def main():
	
	# Initialize the moveit_commander and rospy nodes.
	moveit_commander.roscpp_initialize(sys.argv)
	rospy.init_node('move_group_python_interface', anonymous=True)
	
	# Instantiate a Robot Commander object provide information such as 
	# the robot's kinematic model and the robot's current joint states.
	robot = moveit_commander.RobotCommander()
	
	# Instantiate a Planning Scene Interface object provides a remote 
	# interface for getting setting and updating the robot's internal 
	# understanding of the surrounding world.
	scene = moveit_commander.PlanningSceneInterface()
	
	# Instantiate a Move Group Commander object which is an interface
	# to a planning group.
	group_name = "manipulator"
	move_group = moveit_commander.MoveGroupCommander(group_name)
	
	# Create a Display Tragectory ROS publisher which is used to 
	# display trajectories in Rviz.
	display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path/', 
	  moveit_msgs.msg.DisplayTrajectory, queue_size=20)
	
	# Getting some basic information
	# get_basic_info(move_group, robot)
	
	# We can get the end-effector's pose and set new pose for it
	pose_goal = geometry_msgs.msg.Pose()
	pose_goal.orientation.x = 0.0
	pose_goal.position.x = 0.3
	pose_goal.position.y = 0.25
	pose_goal.position.z = 0.973985
	
	move_group.set_pose_target(pose_goal)
	
	# The go command can be called with joint values, poses, or without 
	# any parameters if you have already set the pose or joint target for 
	# the group
	plan = move_group.go(wait=True)
	
	# Calling ``stop()`` ensures that there is no residual movement
	move_group.stop()
	
	# It is always good to clear your targets after planning with poses.
	# Note: there is no equivalent function for clear_joint_value_targets()
	move_group.clear_pose_targets()


if __name__ == "__main__":
	main()
