#include<tm_control/tm_gazebo_control.h>

bool isClose(const double a, const double b)
{
    if (abs(a-b) > 0.005) return false;
    return true;
}


GazeboInterface::GazeboInterface(): node_handle_("") {

//    joint_1_pub = node_handle_.advertise < std_msgs::Float64 > ("/joint_1_cntrl/command", 10);
//    joint_2_pub = node_handle_.advertise < std_msgs::Float64 > ("/joint_2_cntrl/command", 10);
//    joint_3_pub = node_handle_.advertise < std_msgs::Float64 > ("/joint_3_cntrl/command", 10);
//    joint_4_pub = node_handle_.advertise < std_msgs::Float64 > ("/joint_4_cntrl/command", 10);
//    joint_5_pub = node_handle_.advertise < std_msgs::Float64 > ("/joint_5_cntrl/command", 10);
    finger_l_joint_pub = node_handle_.advertise < std_msgs::Float64 > ("/finger_l_controller/command", 10);
    finger_r_joint_pub = node_handle_.advertise < std_msgs::Float64 > ("/finger_r_controller/command", 10);
//    head_pan_pub = node_handle_.advertise < std_msgs::Float64 > ("/pan/command", 10);
//    head_tilt_pub = node_handle_.advertise < std_msgs::Float64 > ("/tilt/command", 10);

//    node_handle_.getParam("torque_control", torque_control_);

//    smooth_joint_trajectory_server_
//        = new actionlib::SimpleActionServer < control_msgs::FollowJointTrajectoryAction > (
//            node_handle_,
//            "locobot_arm/joint_controller/trajectory",
//            boost::bind( & GazeboInteface::executeJointTrajectory, this, _1),
//            false);

//    joint_command_sub_ = node_handle_.subscribe("goal_dynamixel_position", 10, &
//        GazeboInteface::goalJointPositionCallback,
//        this);

    gripper_open_sub_ = node_handle_.subscribe("gripper/open", 10, & GazeboInterface::gripperOpen, this);
    gripper_close_sub_ = node_handle_.subscribe("gripper/close", 10, & GazeboInterface::gripperClose, this);

//    stop_joints_sub_ = node_handle_.subscribe("stop_execution", 10, & GazeboInterface::stopExecution, this);

    joint_state_sub_ = node_handle_.subscribe("joint_states", 10, & GazeboInterface::recordArmJoints, this);

    gripper_state_pub = node_handle_.advertise<std_msgs::Int8>("gripper/state", 10);


//    pub_arr[0] = joint_1_pub;
//    pub_arr[1] = joint_2_pub;
//    pub_arr[2] = joint_3_pub;
//    pub_arr[3] = joint_4_pub;
//    pub_arr[4] = joint_5_pub;
    pub_arr[5] = finger_l_joint_pub;
    pub_arr[6] = finger_r_joint_pub;
//    pub_arr[7] = head_pan_pub;
//    pub_arr[8] = head_tilt_pub;

//    smooth_joint_trajectory_server_->start();

//    //individual joint command service!!

//    joint_command_server_ = node_handle_.advertiseService("joint_command", &
//        GazeboInterface::jointCommandMsgCallback, this);

//    joint_torque_server_ = node_handle_.advertiseService("torque_command", 
//                            &GazeboInterface::jointTorqueCommandMsgCallback, this);

//    torque_command_client_ =  node_handle_.serviceClient<gazebo_msgs::ApplyJointEffort>(
//                                                                    "/gazebo/apply_joint_effort");


//    //set intial positions to 0 for all the joints
//    ros::Duration(5).sleep();
//    if (torque_control_)
//    {
//        // command 2 positon for joints 5,6,7, pan and tilt
//        for (int index = 4; index < 9; index++) {
//            std_msgs::Float64 msg;
//            msg.data = 0;
//            pub_arr[index].publish(msg);
//        }

//        // command zero torques for the first four joints
//        for (int i = 1; i <5; ++i)
//            SetJointTorque(i, 0);

//        // command toruqes to hold th initial position
//        double zero_postion_torques[4] = {-0.41155806170420917, -0.8730301010793617, 
//                                            -0.8806836788762595, -0.1379062563443867};
//        for (int i = 1; i <5; ++i)
//            SetJointTorque(i, zero_postion_torques[i]);

//    }
//    else
//    {
//        for (int index = 0; index < 9; index++) {
//            std_msgs::Float64 msg;
//            msg.data = 0;
//            pub_arr[index].publish(msg);
//        }        
//    }

    gripper_state = -1; // unknown
    gripper_timer = node_handle_.createTimer(ros::Duration(0.1),
                                                     &GazeboInterface::gripperStateCallback, this);
}

void GazeboInterface::recordArmJoints(const sensor_msgs::JointState::ConstPtr & msg) {
    for (int index = 0; index < 7; ++index)
        arm_state[index] = msg->position.at(index); //the index of arm joints starts at 2
    //ROS_INFO("Recording joint Angles!");
}

void GazeboInterface::gripperStateCallback(const ros::TimerEvent &)
{
    std_msgs::Int8 gripper_state_msg;
    gripper_state_msg.data = gripper_state;
    gripper_state_pub.publish(gripper_state_msg);
}

void GazeboInterface::gripperOpen(const std_msgs::Empty & mg) {
    gripper_state = -1; // unknown
    std_msgs::Float64 msg;
    msg.data = -1*GRIPPER_OPEN_POS;
    finger_r_joint_pub.publish(msg);
    msg.data = GRIPPER_OPEN_POS;
    finger_l_joint_pub.publish(msg);

    ros::Duration(1).sleep();
    
    if (!isClose(arm_state[5], -1*GRIPPER_OPEN_POS) || 
        !isClose(arm_state[6], GRIPPER_OPEN_POS))
        gripper_state = -1;  // unknown
    else
        gripper_state = 0; // open
}

void GazeboInterface::gripperClose(const std_msgs::Empty & mg) {
    gripper_state = 1; // closing

    std_msgs::Float64 msg;
    msg.data = GRIPPER_CLOSE_POS;
    finger_l_joint_pub.publish(msg);
    finger_r_joint_pub.publish(msg);

    ros::Duration(1).sleep();

    if (!isClose(arm_state[5], GRIPPER_CLOSE_POS) || 
        !isClose(arm_state[6], GRIPPER_CLOSE_POS))
        gripper_state = 2;    //object in hand
    else
        gripper_state = 3; //Fully closed
}

int main(int argc, char ** argv) {
    // Init ROS node
    ros::init(argc, argv, "tm_gazebo_control");
    GazeboInterface gzi;
    ros::spin();

    return 0;
}

