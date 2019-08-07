import sys
import rospy
from moveit_msgs.msg import MoveItErrorCodes
from moveit_python import MoveGroupInterface, PlanningSceneInterface
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from argparse import ArgumentParser

parser = ArgumentParser()
parser.add_argument("--reset", "-r", action="store_true", help="Reset the robot to home pose.", default=False)

def get_pose(reset):

	if reset:
		poses = [[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]
		pose_name = "Reset"	
	else:
		poses = [Pose(Point(0.3, 0.0, 1.0), 
				  Quaternion(0.0, 0.0, 0.0, 0.0))]
		pose_name = "Wave"

	return poses, pose_name

def main(args):

	rospy.init_node('simple_move')

	move_group = MoveGroupInterface('manipulator', "base_link") 

	planning_scene = PlanningSceneInterface("base_link")

	joint_names = ['shoulder_1_joint', 'shoulder_2_joint', 'elbow_1_joint',
		'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

	
	# This is the wrist link not the gripper itself
	gripper_frame = "wrist_3_link"

	poses, pose_name = get_pose(args.reset)

	# Construct a "pose_stamped" message as required by moveToPose
	gripper_pose_stamped = PoseStamped()

	gripper_pose_stamped.header.frame_id = "base_link"

	
	while not rospy.is_shutdown():

		for pose in poses:

			# Finish building the Pose_stamped message
			# If the message stampe is not current it could be ignored
			gripper_pose_stamped.header.stamp = rospy.Time.now()
			
			# Set the message pose
			gripper_pose_stamped.pose = pose

			move_group.moveToPose(gripper_pose_stamped, gripper_frame)
			
			result = move_group.get_move_action().get_result()

			if result:
				# Checking the MoveItErrorCode
				if result.error_code.val == MoveItErrorCodes.SUCCESS:
					log_output = pose_name + " Done!"
					rospy.loginfo(log_output)
				else:
					# If you get to this point please search for:
					# moveit_msgs/MoveItErrorCodes.msg
					rospy.logerr("Arm goal in state: %s", 
					  move_group.get_move_action().get_state())
			else:
				rospy.logerr("MoveIt! failure no result returned.")

	# This stops all arm movement goals
	# It should be called when a program is exiting so movement stops
	move_group.get_move_action().cancel_all_goals()

if __name__ == '__main__':
	
	args = parser.parse_args()
	print ("Reset:", args.reset)
	main(args)


