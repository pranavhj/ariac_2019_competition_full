//
// Created by zeid on 2/27/20.
//

#pragma once


#include <list>
#include <map>
#include <string>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <osrf_gear/LogicalCameraImage.h>

#include "ariac_part_manager.h"

class AriacSensorManager {
public:
    AriacSensorManager();
    ~AriacSensorManager();
    void LogicalCamera1Callback(const osrf_gear::LogicalCameraImage::ConstPtr &);
    void LogicalCamera2Callback(const osrf_gear::LogicalCameraImage::ConstPtr &);
    void LogicalCamera3Callback(const osrf_gear::LogicalCameraImage::ConstPtr &);
    void LogicalCamera5Callback(const osrf_gear::LogicalCameraImage::ConstPtr &);
    void LogicalCamera6Callback(const osrf_gear::LogicalCameraImage::ConstPtr &);
    void LogicalCamera7Callback(const osrf_gear::LogicalCameraImage::ConstPtr &);
    
    std::vector <std::string> getEmptyBins(){
        return emptyBins;
    } ;

    std::vector <std::string> getbinPartCount();
    ros::Time callback_time;

    bool sensorBlackout(); 
    void updateProductList();

    geometry_msgs::Pose GetPartPose(const std::string& src_frame,
                                    const std::string& target_frame);
    std::map<std::string, std::vector<std::string>> get_product_frame_list(){
        return product_frame_list_;
    }
    //void ScanParts(int cam_number);
    void BuildProductFrames(int);

private:
    ros::NodeHandle sensor_nh_;
    ros::Subscriber camera_1_subscriber_;
    ros::Subscriber camera_2_subscriber_;
    ros::Subscriber camera_3_subscriber_;
    ros::Subscriber camera_5_subscriber_;
    ros::Subscriber camera_6_subscriber_;
    ros::Subscriber camera_7_subscriber_;


    tf::TransformListener camera_tf_listener_;
    tf::StampedTransform camera_tf_transform_;

    osrf_gear::LogicalCameraImage current_parts_1_;
    osrf_gear::LogicalCameraImage current_parts_2_;
    osrf_gear::LogicalCameraImage current_parts_3_;
    osrf_gear::LogicalCameraImage current_parts_5_;
    osrf_gear::LogicalCameraImage current_parts_6_;
    osrf_gear::LogicalCameraImage current_parts_7_;
    std::map<std::string, std::vector<geometry_msgs::Pose>> part_list_;
    std::vector<AriacPartManager> camera1_part_list,camera2_part_list,camera3_part_list,camera5_part_list,camera6_part_list,camera7_part_list;

    //std::map<std::string, std::list<std::string>> parts_list_;
    std::map<std::string, std::vector<std::string>> product_frame_list_;

    std::vector <std::string> emptyBins ;
    // std::vector <int> fullBins ;

    std::vector <std::string> binParts = {"gasket_part", "piston_rod_part", "gear_part", "pulley_part", "disk_part"};
    std::map<std::string,int> binPartCount;

    bool init_, cam_1_, cam_2_,cam_3_ ,cam_5_ , cam_6_,cam_7_;
    int camera1_frame_counter_, camera2_frame_counter_, camera3_frame_counter_, camera5_frame_counter_, camera6_frame_counter_, camera7_frame_counter_;
};

