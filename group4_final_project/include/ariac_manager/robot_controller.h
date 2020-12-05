//
// Created by zeid on 2/27/20.
//

#ifndef SRC_ROBOT_CONTROLLER_H
#define SRC_ROBOT_CONTROLLER_H


#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <ros/ros.h>
#include <stdarg.h>
#include <tf/transform_listener.h>
#include <iostream>
#include <string>
#include <initializer_list>
#include <osrf_gear/LogicalCameraImage.h>

#include <osrf_gear/VacuumGripperControl.h>
#include <osrf_gear/VacuumGripperState.h>

class RobotController {
public:
    RobotController(std::string arm_id);
    
    ~RobotController();
    bool Planner();
    void Execute();
    void GoToTarget(std::initializer_list<geometry_msgs::Pose> list);
    void GoToTarget(const geometry_msgs::Pose& pose);
    void GoToTarget_with_orientation(std::initializer_list<geometry_msgs::Pose> list);
    void GoToTarget_with_orientation(const geometry_msgs::Pose& pose);
    void SendRobotHome();
    void SendRobotConveyer();
    void SendRobotFlipping();
    void SendRobotJointPosition(std::vector<double> joint_states);
    int DropPart(std::string,geometry_msgs::Pose pose);
    void GripperToggle(const bool& state);
    void GripperCallback(const osrf_gear::VacuumGripperState::ConstPtr& grip);
    void KitTrayCameraCallback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg);
    void QualityControlSensor1Callback(const osrf_gear::LogicalCameraImage::ConstPtr &);
    bool GetQualityControl1Status();
    void JointStatesCallback(const sensor_msgs::JointState::ConstPtr& joint_states);
    //make this for arm1_ and another for arm2
    std::vector<double> GetJointStates();
    void JointStatesCallback_2(const sensor_msgs::JointState::ConstPtr& joint_states);
    bool GetGripperStatus(){ros::spinOnce();ros::Duration(0.05).sleep();   return gripper_state_;}
    void GripperStateCheck(geometry_msgs::Pose pose);
    bool PickPart(geometry_msgs::Pose& part_pose);
    geometry_msgs::Pose GetEEPose();
    std::vector<std::pair<std::string,geometry_msgs::Pose>> GetKitTrayList();
    osrf_gear::LogicalCameraImage quality_control_sensor_image_;
    void ThrowFaultyProducts();
private:
    ros::NodeHandle robot_controller_nh_;
    moveit::planning_interface::MoveGroupInterface::Options robot_controller_options;
    ros::ServiceClient gripper_client_;
    ros::NodeHandle gripper_nh_;
    ros::Subscriber gripper_subscriber_;
    ros::Subscriber quality_control_sensor_1_subscriber_;
    ros::Subscriber joint_states_subscriber_;
    ros::Subscriber joint_states_subscriber_2_;
    ros::Subscriber kit_tray_camera_subscriber_;

   

    tf::TransformListener robot_tf_listener_;
    tf::StampedTransform robot_tf_transform_;
    tf::TransformListener agv_tf_listener_;
    tf::StampedTransform agv_tf_transform_;

    geometry_msgs::Pose target_pose_;

    moveit::planning_interface::MoveGroupInterface robot_move_group_;
    moveit::planning_interface::MoveGroupInterface::Plan robot_planner_;

    osrf_gear::VacuumGripperControl gripper_service_;
    osrf_gear::VacuumGripperState gripper_status_;

    std::string object;
    bool plan_success_;
    std::vector<double> home_joint_pose_;
    std::vector<double> agv_joint_pose_;
    std::vector<double> conveyer_joint_pose_;
    std::vector<double> flipping_joint_pose_;


    geometry_msgs::Pose home_cart_pose_;
    geometry_msgs::Quaternion fixed_orientation_;
    geometry_msgs::Pose agv_position_;
    std::vector<double> end_position_;
    double offset_;
    double roll_def_,pitch_def_,yaw_def_;
    tf::Quaternion q;
    int counter_;
    bool gripper_state_, drop_flag_,quality_control_sensor_1_flag_=false;
    std::string defected_part_;
    std::string kit_tray_;
    std::vector <double> joint_states_;
    std::vector <double> joint_states_2_;
    std::string arm_id_;
    int kit_tray_camera_count_=0,init_before_=0,init_after_=0;
    std::vector<std::pair<std::string,geometry_msgs::Pose>> kit_tray_list_before_;
    std::vector<std::pair<std::string,geometry_msgs::Pose>> kit_tray_list_;
    std::vector<std::pair<std::string,geometry_msgs::Pose>> kit_tray_list_without_transform_;     


};
#endif //SRC_ROBOT_CONTROLLER_H
