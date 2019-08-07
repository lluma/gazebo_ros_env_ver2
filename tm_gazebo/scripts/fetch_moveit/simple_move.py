import sys
import rospy
from moveit_msgs.msg import MoveItErrorCodes
from moveit_python import MoveGroupInterface, PlanningSceneInterface
from argparse import ArgumentParser
from math import pi

parser = ArgumentParser()
parser.add_argument("--case", "-c", help="Specify the move case fro robotic arm.", default="move_base")
parser.add_argument("--reset", "-r", action="store_true", help="Reset the robot to home pose.", default=False)

def get_pose(pose_case, reset):

	if reset:
		poses = [[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]
		pose_name = "Reset"
	else:
		if pose_case == "move_base":

			poses = [[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
			         [pi/2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]
			pose_name = "Base turn 90 degree counterclockwise"

		elif pose_case == "move_shoulder_1":

			poses = [[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
			         [0.0, pi/2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]
			pose_name = "Shoulder 1 turn 90 degree counterclockwise"

		elif pose_case == "move_shoulder_2":
			
			poses = [[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
			         [0.0, 0.0, pi/2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]
			pose_name = "Shoulder 2 turn 90 degree counterclockwise"
			
	return poses, pose_name

def main(args):

	rospy.init_node('simple_move')

	move_group = MoveGroupInterface('manipulator', "base_link") 

	planning_scene = PlanningSceneInterface("base_link")

	joint_names = ['shoulder_1_joint', 'shoulder_2_joint', 'elbow_1_joint',
		'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

	
	poses, pose_name = get_pose(args.case, args.reset)


	for pose in poses:
		
		if rospy.is_shutdown():
			break

		move_group.moveToJointPosition(joint_names, pose, wait=False)

	    # Since we passed in wait=False above we need to wait here
        move_group.get_move_action().wait_for_result()
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


