#pragma once

#include <list>
#include <map>
#include <string>
#include <iostream>
#include <algorithm>
#include <vector>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <osrf_gear/LogicalCameraImage.h>
#include <osrf_gear/Order.h>
#include <osrf_gear/Shipment.h>
#include <osrf_gear/Proximity.h>

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include "sensor.h"
#include "robot_controller.h"
#include <sensor_msgs/JointState.h>
#include <osrf_gear/Product.h>

class AriacOrderManager {
public:
    AriacOrderManager();
    ~AriacOrderManager();
    void OrderCallback(const osrf_gear::Order::ConstPtr& order_msg);
    // void break_beam_callback(const osrf_gear::Proximity::ConstPtr& msg);
    void LogicalCamera4Callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg);
    //for conveyer pickup
    void LogicalCamera5Callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg);
    //for kit_tray1

    void LogicalCamera5_2Callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg);
    //for kit_tray2

    void LogicalCamera1Callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg);
    //for flipping on bin1 for arm1_
    
     void LogicalCameraRepickCallback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg);
    //for flipping on bin1 for arm1_

    void LogicalCameraConveyerCallback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg);

    void JointStatesCallback(const sensor_msgs::JointState::ConstPtr& joint_states);
    

    void JointStatesCallback_2(const sensor_msgs::JointState::ConstPtr& joint_states);
    void ExecuteOrder();
    std::string GetProductFrame(std::string product_type);
    std::map<std::string, std::list<std::pair<std::string,geometry_msgs::Pose>>> GetOrder();
    bool PickAndPlace(std::pair<std::string,geometry_msgs::Pose> object_prop,int agvnum,int empty_flag);

    int BigPickAndPlace(std::pair<std::string,geometry_msgs::Pose> object_prop,std::string agvnum);
    void ContinueDuringBlackout();


    //for arm1_
    void SubmitAGV(int num);
    void PickFromConveyer(std::string product_type,int agv_id);
    int PickFromConveyerBefore(std::vector<std::pair<float,float> >drop_pose,std::string product_type,std::string empty_bin);

    //for arm1_
    int pulleyFlipper(int agv_id);
    //for arm1_
    void CompleteOrderUpdate(osrf_gear::Order order);
    void CompletethisOrder(osrf_gear::Order order);
    
    void removeElement(osrf_gear::Order ord );
    void ThrowComponents(std::pair<std::string,geometry_msgs::Pose>,int);
    int RepositionComponents(std::pair<std::string,std::vector<geometry_msgs::Pose>>,int );
    std::pair<std::string,geometry_msgs::Pose> PartThere(geometry_msgs::Pose ,int );
    float euler_distance(geometry_msgs::Pose p1,geometry_msgs::Pose p2);
    int GetFromArm2(std::string product_type,geometry_msgs::Pose part_pose,std::string product_frame,int agv_id);
    std::vector <std::pair<std::string,std::vector<geometry_msgs::Pose> > > UpdateLists(osrf_gear::Shipment shipment);
    std::vector<std::pair<float,float> >empty_bin_coordinates_;
    void InitializeEmptyBinCoord(std::string product_type);
    void InitializeMiddleVector(float);
    int GiveArm2(std::string,geometry_msgs::Pose);
    bool GetSensorBlackout();
    void CheckOrder(osrf_gear::Shipment shipment);
    void UpdateShipmentVector(osrf_gear::Shipment shipment);
    void RemovefromVector(osrf_gear::Product product);
    void ThrowFaultyComponents(osrf_gear::LogicalCameraImage quality_image,int agv_id);
private:
    double start_time_=ros::Time::now().toSec();
    ros::NodeHandle order_manager_nh_;
    ros::Subscriber order_subscriber_;
    ros::Subscriber break_beam_subscriber_;
    ros::Subscriber camera_4_subscriber_;
    ros::Subscriber camera_1_subscriber_;
    ros::Subscriber camera_5_subscriber_;
    ros::Subscriber camera_5_2_subscriber_;

    ros::Subscriber joint_states_subscriber_;
    ros::Subscriber joint_states_subscriber_2_;
    ros::Subscriber camera_repick_subscriber_;
    ros::Subscriber camera_conveyer_indices_subscriber_;

    std::vector<osrf_gear::Order> received_orders_;
    osrf_gear::Order current_order_;
    osrf_gear::Shipment current_shipment_;
    std::vector<std::pair <std::string, geometry_msgs::Pose> >current_shipment_vector_;
    AriacSensorManager camera_;

    RobotController arm1_{"arm1"};
    RobotController arm2_{"arm2"};

    //RobotController *arm;
    //RobotController *arm_dash;
    
    tf::TransformListener part_tf_listener_;
    std::pair<std::string,geometry_msgs::Pose> product_type_pose_;
    std::string object;
    std::map<std::string, std::vector<std::string>> product_frame_list_;
    osrf_gear::Order order_;
    osrf_gear::Proximity beam_;

    tf::TransformListener camera4_tf_listener_;
    tf::StampedTransform camera4_tf_transform_;
    
    int init_=0;
    int flag_ = 0,order_update_flag_=0;
    int flag_break_beam_=0;
    std::string camera_4_product_;
    geometry_msgs::Pose camera4_op_pose_;
    std::vector <double> joint_states_;
    std::vector <double> joint_states_2_;
    std::vector <double> home_joint_pose_={0.0,3.1,-1.1, 1.9, 3.9, 4.7, 0};
    
    std::vector <double> end_position_={1.18,3.1,-1.1, 1.9, 3.9, 4.7, 0};
    //end_position_[0] = 2.2;
    
    std::vector<std::pair<std::string,geometry_msgs::Pose>> cam5_list_;
    std::vector<std::pair<std::string,geometry_msgs::Pose>> cam5_list_dash_;
   
    std::vector<std::string> cameraParts;
    ros::Time break_beam_time;
    double camera_4_detection_time_;
    std::map<std::string,int > camera4_product_type_passed_;
    int init_cam1_=0;
    int init_cam5_=0,init_cam5_2_=0,repose_flag_=0,init_cam5_dash_=0,repose_flag_dash_=0,init_cam5_2_dash_=0;
    int init_repick_=0;
    std::string repick_part_;
    geometry_msgs::Pose repick_pose_,repick_pose_dash_;
    bool pulley_on_bin_;
    geometry_msgs::Pose flipping_pulley_pose_;
    std::vector<std::pair< std::string, std::vector<geometry_msgs::Pose > > > MultipleRepositionVector_; 
    std::vector<geometry_msgs::Pose> middle_poses_vector_;
    std::vector<char> logical_camera_indices_={'1','2','3','5','6','7'};
    
    // camera4_product_type_passed_["pulley_part"]=0;
    // camera4_product_type_passed_["gasket_part"]=0;
    // camera4_product_type_passed_["piston_rod"]=0;
    // camera4_product_type_passed_["gear_part"]=0;


    tf::TransformListener robot_tf_listener_;
    tf::StampedTransform robot_tf_transform_;
    geometry_msgs::Quaternion fixed_orientation_;
    
    geometry_msgs::Pose reposition_part_pose_,reposition_part_pose_dash_;
    geometry_msgs::Pose reposition_part_pose_back_,reposition_part_pose_back_dash_;
    std::string reposition_part_type_,reposition_part_type_dash_;
    std::string product_type_conveyer_bin_;
    int init_conveyer_bin_=0,part_present_=0;
    std::string indices_conveyer_, indices_conveyer_callback_;
    geometry_msgs::Pose conveyer_pick_up_pose_;
};
