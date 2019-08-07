#ifndef TM_GAZEBO_H
#define TM_GAZEBO_H

#define GRIPPER_OPEN_POS 0.03
#define GRIPPER_CLOSE_POS 0.0

#include <ros/ros.h>

#include <sensor_msgs/JointState.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int8.h>
//#include <trajectory_msgs/JointTrajectory.h>

// Below includes are for SimpleActionCilent
//#include <actionlib/client/simple_action_client.h>
//#include <actionlib/client/simple_action_server.h>
//#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/GripperCommandAction.h>
#include <boost/thread/recursive_mutex.hpp>
//#include <gazebo_msgs/ApplyJointEffort.h>

#define NUMBER_OF_ARM_JOINTS 6

class GazeboInterface {
  private:
    
    ros::NodeHandle node_handle_;
    
    // [shoulder_1_joint, shoulder_2_joint, elbow_1_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint]
//    ros::Publisher shoulder_1_joint_pub;
//    ros::Publisher shoulder_2_joint_pub;
//    ros::Publisher elbow_1_joint_pub;
//    ros::Publisher wrist_1_joint_pub;
//    ros::Publisher wrist_2_joint_pub;
//    ros::Publisher wrist_3_joint_pub;
    ros::Publisher finger_r_joint_pub;
    ros::Publisher finger_l_joint_pub;
    ros::Publisher gripper_state_pub;
    ros::Publisher pub_arr[9];
    
//    ros::Subscriber joint_command_sub_;
    ros::Subscriber gripper_open_sub_;
    ros::Subscriber gripper_close_sub_;
//    ros::Subscriber stop_joints_sub_;
    ros::Subscriber joint_state_sub_;
    
//    ros::ServiceServer joint_command_server_;
//    ros::ServiceServer joint_torque_server_;
//    ros::ServiceServer torque_command_client_;
    
    ros::Timer gripper_timer;
    
//    actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> *smooth_joint_trajectory_server_;
    
//    bool torque_control_;
    double arm_state[7];
    int gripper_state;
    
  public:
    
    GazeboInterface();
    

//    bool jointCommandMsgCallback(locobot_control::JointCommand::Request &req,
//                                              locobot_control::JointCommand::Response &res);

//    bool jointTorqueCommandMsgCallback(locobot_control::JointCommand::Request &req,
//                                              locobot_control::JointCommand::Response &res);

//    void goalJointPositionCallback(const sensor_msgs::JointState::ConstPtr &msg);
    void recordArmJoints(const sensor_msgs::JointState::ConstPtr &msg);
//    void stopExecution(const std_msgs::Empty &msg);
    void gripperOpen(const std_msgs::Empty &mg);
    void gripperClose(const std_msgs::Empty &mg);
//    void executeJointTrajectory(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal);
//    void SetJointTorque(const int id, const double value);
    void gripperStateCallback(const ros::TimerEvent &);
    
};

#endif
