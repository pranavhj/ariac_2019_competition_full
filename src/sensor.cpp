//
// Created by zeid on 2/27/20.
//
#include "sensor.h"

AriacSensorManager::AriacSensorManager():
camera1_part_list{},
camera2_part_list{},
camera3_part_list{},
camera5_part_list{},
camera6_part_list{},
camera7_part_list{}{
    ROS_INFO_STREAM(">>>>> Subscribing to logical sensors");
    camera_1_subscriber_ = sensor_nh_.subscribe("/ariac/logical_camera_1", 10,
                                                &AriacSensorManager::LogicalCamera1Callback, this);
    camera_2_subscriber_ = sensor_nh_.subscribe("/ariac/logical_camera_2", 10,
                                                &AriacSensorManager::LogicalCamera2Callback, this);
    camera_3_subscriber_ = sensor_nh_.subscribe("/ariac/logical_camera_3", 10,
                                                &AriacSensorManager::LogicalCamera3Callback, this);
    camera_5_subscriber_ = sensor_nh_.subscribe("/ariac/logical_camera_5", 10,
                                                &AriacSensorManager::LogicalCamera5Callback, this);
    camera_6_subscriber_ = sensor_nh_.subscribe("/ariac/logical_camera_6", 10,
                                                &AriacSensorManager::LogicalCamera6Callback, this);
    camera_7_subscriber_ = sensor_nh_.subscribe("/ariac/logical_camera_7", 10,
                                                &AriacSensorManager::LogicalCamera7Callback, this);
    
    camera1_frame_counter_ = 1;
    camera2_frame_counter_ = 1;
    camera3_frame_counter_ = 1;
    camera5_frame_counter_ = 1;
    camera6_frame_counter_ = 1;
    camera7_frame_counter_ = 1;

    init_ = false;
    cam_1_ = false;
    cam_2_ = false;
    cam_3_ = false;
    cam_5_ = false;
    cam_6_ = false;
    cam_7_ = false;

}

AriacSensorManager::~AriacSensorManager() {}




void AriacSensorManager::LogicalCamera1Callback(const osrf_gear::LogicalCameraImage::ConstPtr& image_msg){
    callback_time = ros::Time::now();
    //if (sensorBlackout()) ROS_INFO_STREAM (" SENSOR BLACKOUT INITIATED"<< sensorBlackout() );
    if (init_) return;
    ROS_INFO_STREAM_THROTTLE(10,
                             "Logical camera 1: '" << image_msg->models.size() << "' objects.");

    if (image_msg->models.size() == 0) {
        emptyBins.push_back("/bin1_frame");
        ROS_ERROR_STREAM("Logical Camera 1 does not see anything");
    }
    // else fullBins.push_back(1);
    current_parts_1_ = *image_msg;
    this->BuildProductFrames(1);
//    //--Object which represents a part
//    AriacPartManager part_manager;
//    //--Logical camera always represents the first product seen in frame 1
//
//    int part_frame{1};
//
//    for (auto& msg : image_msg->models) {
//        geometry_msgs::Pose part_pose;
//
//        //--position
//        part_pose.position.x = msg.pose.position.x;
//        part_pose.position.y = msg.pose.position.y;
//        part_pose.position.z = msg.pose.position.z;
//        //--orientation
//        part_pose.orientation.x = msg.pose.orientation.x;
//        part_pose.orientation.y = msg.pose.orientation.y;
//        part_pose.orientation.z = msg.pose.orientation.z;
//        part_pose.orientation.w = msg.pose.orientation.w;
//
//        part_manager.set_part_frame(part_frame);
//        part_manager.set_part_type(msg.type);
//        part_manager.set_part_pose(part_pose);
//            camera1_part_list.push_back(part_manager);
//        //--next frame id is incremented by 1
//        part_frame++;
//    }
//
//    for (auto &part: camera1_part_list)
//    {
//        ROS_INFO_STREAM(">>>>> Part type:" << part.get_part_type());
//        ROS_INFO_STREAM(">>>>> Part Pose x:" << part.get_part_pose().position.x);
//        ROS_INFO_STREAM(">>>>> Part frame:" << part.get_part_frame());
//    }
}


void AriacSensorManager::LogicalCamera2Callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg){
    if (init_) return;
    ROS_INFO_STREAM_THROTTLE(10,
                             "Logical camera 2: '" << image_msg->models.size() << "' objects.");
    if (image_msg->models.size() == 0) {
        emptyBins.push_back("/bin2_frame");
        ROS_ERROR_STREAM("Logical Camera 2 does not see anything");
    }
    // else fullBins.push_back(2);
    current_parts_2_ = *image_msg;
    this->BuildProductFrames(2);
}

void AriacSensorManager::LogicalCamera3Callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg){
    if (init_) return;
    ROS_INFO_STREAM_THROTTLE(10,
                             "Logical camera 3: '" << image_msg->models.size() << "' objects.");
    if (image_msg->models.size() == 0) {
        emptyBins.push_back("/bin3_frame");
        ROS_ERROR_STREAM("Logical Camera 3 does not see anything");
    }
    // else fullBins.push_back(3);

    current_parts_3_ = *image_msg;
    this->BuildProductFrames(3);
}

void AriacSensorManager::LogicalCamera5Callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg){
    if (init_) return;
    ROS_INFO_STREAM_THROTTLE(10,
                             "Logical camera 5: '" << image_msg->models.size() << "' objects.");
    if (image_msg->models.size() == 0) {
        emptyBins.push_back("/bin4_frame");
        ROS_ERROR_STREAM("Logical Camera 5 does not see anything");
    }
    // else fullBins.push_back(4);

    current_parts_5_ = *image_msg;
    this->BuildProductFrames(5);
}

void AriacSensorManager::LogicalCamera6Callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg){
    if (init_) return;
    ROS_INFO_STREAM_THROTTLE(10,
                             "Logical camera 6: '" << image_msg->models.size() << "' objects.");
    if (image_msg->models.size() == 0) {
        emptyBins.push_back("/bin5_frame");
        ROS_ERROR_STREAM("Logical Camera 6 does not see anything");
    }
    // else fullBins.push_back(5);

    current_parts_6_ = *image_msg;
    this->BuildProductFrames(6);
}

void AriacSensorManager::LogicalCamera7Callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg){
    if (init_) return;
    ROS_INFO_STREAM_THROTTLE(10,
                             "Logical camera 7: '" << image_msg->models.size() << "' objects.");
    if (image_msg->models.size() == 0) {
        emptyBins.push_back("/bin6_frame");
        ROS_ERROR_STREAM("Logical Camera 7 does not see anything");
    }
    // else fullBins.push_back(6);

    current_parts_7_ = *image_msg;
    this->BuildProductFrames(7);
}

void AriacSensorManager::BuildProductFrames(int camera_id){
    if (camera_id == 1) {
        for (auto& msg : current_parts_1_.models) {
            //--build the frame for each product
            std::string product_frame = "logical_camera_1_" + msg.type + "_" +
                                        std::to_string(camera1_frame_counter_) + "_frame";

            product_frame_list_[msg.type].emplace_back(product_frame);
            binPartCount[msg.type] += 1;
            camera1_frame_counter_++;
        }
        cam_1_ = true;
    }
    else if (camera_id == 2) {
        for (auto& msg : current_parts_2_.models) {
            //--build the frame for each product
            std::string product_frame = "logical_camera_2_" + msg.type + "_" +
                                        std::to_string(camera2_frame_counter_) + "_frame";

            product_frame_list_[msg.type].emplace_back(product_frame);
            binPartCount[msg.type] += 1;
            camera2_frame_counter_++;
        }
        cam_2_ = true;
    }
    else if (camera_id == 3) {
        for (auto& msg : current_parts_3_.models) {
            //--build the frame for each product
            std::string product_frame = "logical_camera_3_" + msg.type + "_" +
                                        std::to_string(camera3_frame_counter_) + "_frame";

            product_frame_list_[msg.type].emplace_back(product_frame);
            binPartCount[msg.type] += 1;
            camera3_frame_counter_++;       
        }
        cam_3_ = true;
    }
    else if (camera_id == 5) {
        for (auto& msg : current_parts_5_.models) {
            //--build the frame for each product
            std::string product_frame = "logical_camera_5_" + msg.type + "_" +
                                        std::to_string(camera5_frame_counter_) + "_frame";

            product_frame_list_[msg.type].emplace_back(product_frame);
            binPartCount[msg.type] += 1;
            camera5_frame_counter_++;
        }
        cam_5_ = true;
    }
    else if (camera_id == 6) {
        for (auto& msg : current_parts_6_.models) {
            //--build the frame for each product
            std::string product_frame = "logical_camera_6_" + msg.type + "_" +
                                        std::to_string(camera6_frame_counter_) + "_frame";

            product_frame_list_[msg.type].emplace_back(product_frame);
            binPartCount[msg.type] += 1;
            camera6_frame_counter_++;
        }
        cam_6_ = true;
    }
    else if (camera_id == 7) {
        for (auto& msg : current_parts_7_.models) {
            //--build the frame for each product
            std::string product_frame = "logical_camera_7_" + msg.type + "_" +
                                        std::to_string(camera7_frame_counter_) + "_frame";

            product_frame_list_[msg.type].emplace_back(product_frame);
            binPartCount[msg.type] += 1;
            camera7_frame_counter_++;
        }
        cam_7_ = true;
    }
    if (cam_1_ && cam_2_ && cam_3_ && cam_5_ && cam_6_ && cam_7_) {
        init_ = true;
    }
}

geometry_msgs::Pose AriacSensorManager::GetPartPose(const std::string& src_frame,
                                        const std::string& target_frame) {
    geometry_msgs::Pose part_pose;
    std::vector<char> cameraList = {'1','2','3','5','6','7'};
    std::string target_frame_ = target_frame;
    int counter = -1;

    ROS_INFO_STREAM("Getting part pose...");

    if (init_) {
        retryAgain:
        try {
            camera_tf_listener_.waitForTransform(src_frame, target_frame_, ros::Time(0),
                                             ros::Duration(3));

            camera_tf_listener_.lookupTransform(src_frame, target_frame_, ros::Time(0),
                                            camera_tf_transform_);
        }
        catch (tf2::LookupException& e) {
            //ROS_ERROR_STREAM("In the catch part for the tf2 LookupException");
            counter++;
            ROS_ERROR_STREAM("New frame is"<<target_frame_);
            target_frame_[15] = cameraList.at(counter);
            goto retryAgain;
        }

        part_pose.position.x = camera_tf_transform_.getOrigin().x();
        part_pose.position.y = camera_tf_transform_.getOrigin().y();
        part_pose.position.z = camera_tf_transform_.getOrigin().z();

    } else {
        ros::spinOnce();
        ros::Duration(0.2).sleep();
        this->BuildProductFrames(1);
        ros::spinOnce();
        ros::Duration(0.2).sleep();
        this->BuildProductFrames(2);

        part_pose = this->GetPartPose(src_frame, target_frame);
    }

    return part_pose;
}



std::vector <std::string> AriacSensorManager::getbinPartCount() {
    std::vector <std::string> temp;
    for(auto& x: binParts) {
        if (binPartCount.find(x) == binPartCount.end()) {
            temp.push_back(x);
        }
    }

    if (temp.size()==0) {
        for (auto &x : binPartCount) {
            if(x.second < 3) {
                temp.push_back(x.first);
            }
        }
    }
    return temp;
}

bool AriacSensorManager::sensorBlackout() {
    bool blackout = false;
    //ros::spinOnce();
    // if (product_frame_list_.size()==0) blackout = true;
    if ((ros::Time::now() - callback_time) > ros::Duration(3.0)) { 
        blackout = true;
        //ROS_INFO_STREAM("SENSOR BLACKOUT DETECTED.");

        // ROS_INFO_STREAM("callback_time: " << callback_time << " ros time now:" << ros::Time::now());
        ros::spinOnce();
        //ros::Duration(1).sleep();
    }
    else {
        blackout = false;
        //ROS_INFO_STREAM("Blacout over");
    }     
    return blackout;
}

void AriacSensorManager::updateProductList() {
    camera1_frame_counter_ = 1;
    camera2_frame_counter_ = 1;
    camera3_frame_counter_ = 1;
    camera5_frame_counter_ = 1;
    camera6_frame_counter_ = 1;
    camera7_frame_counter_ = 1;
    init_ = false;
    cam_1_ = false;
    cam_2_ = false;
    cam_3_ = false;
    cam_5_ = false;
    cam_6_ = false;
    cam_7_ = false;

    product_frame_list_.clear();
    ros::spinOnce();
    ros::Duration(0.1).sleep();
    
}