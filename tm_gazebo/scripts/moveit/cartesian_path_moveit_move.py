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

def plan_cartesian_path(move_group):
	
	waypoints = []
	scale = 1.0
	
	wpose = move_group.get_current_pose().pose
	wpose.position.z -= scale * 0.1 # First move up (z)
	wpose.position.y -= scale * 0.2 # and sideways (y)
	waypoints.append(copy.deepcopy(wpose))
	
	wpose.position.x += scale * 0.1 # Second move forward/backwards in (x)
	waypoints.append(copy.deepcopy(wpose))
	
	wpose.position.y -= scale * 0.1 # Third move sideways (y)
	waypoints.append(copy.deepcopy(wpose))

	# We want the Cartesian path to be interpolated at a resolution of
	# 1 cm which is why we will specify 0.01 as the eef_step in Cartesian
	# translation. We will disable the jump threshold by setting it to 0.0,
	# ignoring the check for infeasible jumps in joint space, which is 
	# sufficient for us to use.
	(plan, fraction) = move_group.compute_cartesian_path(
							waypoints, # waypoints to follow 
							0.01, # eef_step
							0.0)  # jump_threshold
	return plan, fraction

def display_trajectory(robot, display_trajectory_publisher, plan):
	
	# A `DisplayTrajectory`_msg has two primary fields, trajectory_start 
	# and trajectory. We populate the trajectory_start with our current 
	# robot state to copy over any AttachedCollisionObjects and add our 
	# plan to the trajectory.
	display_trajectory = moveit_msgs.msg.DisplayTrajectory()
	display_trajectory.trajectory_start = robot.get_current_state()
	display_trajectory.trajectory.append(plan)
	
	# Publish
	display_trajectory_publisher.publish(display_trajectory)

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
	
	# You can plan a Cartesian path directly by specifying a list of 
	# waypoints for the end-effector to go through. If executing 
	# interactively in a Python shell, set scale = 1.0
	cartesian_plan, fraction = plan_cartesian_path(move_group)
	
	# You can ask RViz to visualize a plan (aka trajectory) for you. 
	# But the group.plan() method does this automatically so this is 
	# not that useful here. (It just displays the same trajectory 
	# again)
	display_trajectory(robot, display_trajectory_publisher, cartesian_plan)
	
	move_group.execute(cartesian_plan, wait=True)


if __name__ == "__main__":
	main()
