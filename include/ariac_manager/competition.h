//
// Created by zeid on 2/28/20.
//

#ifndef ARIAC_MANAGER_COMPETITION_H
#define ARIAC_MANAGER_COMPETITION_H

#include <algorithm>
#include <vector>

#include <ros/ros.h>

#include <osrf_gear/LogicalCameraImage.h>
#include <osrf_gear/Order.h>
#include <osrf_gear/Proximity.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <trajectory_msgs/JointTrajectory.h>

class AriacOrderManager;

class Competition {
    public:
        explicit Competition(ros::NodeHandle & node)
                : current_score_(0), arm_1_has_been_zeroed_(false), arm_2_has_been_zeroed_(false)
        {
            // %Tag(ADV_CMD)%
            arm_1_joint_trajectory_publisher_ = node.advertise<trajectory_msgs::JointTrajectory>(
                    "/ariac/arm1/arm/command", 10);

            arm_2_joint_trajectory_publisher_ = node.advertise<trajectory_msgs::JointTrajectory>(
                    "/ariac/arm2/arm/command", 10);
            // %EndTag(ADV_CMD)%
        }

        /// Called when a new message is received.
        void current_score_callback(const std_msgs::Float32::ConstPtr & msg) {
            if (msg->data != current_score_)
            {
                ROS_INFO_STREAM("Score: " << msg->data);
            }
            current_score_ = msg->data;
        }

        /// Called when a new message is received.
        void competition_state_callback(const std_msgs::String::ConstPtr & msg) {
            if (msg->data == "done" && competition_state_ != "done")
            {
                ROS_INFO("Competition ended.");
            }
            competition_state_ = msg->data;
        }

        /// Called when a new Order message is received.
//        void order_callback(const osrf_gear::Order::ConstPtr & order_msg) {
//            //ROS_INFO_STREAM("Received order:\n" << *order_msg);
//            ROS_INFO_STREAM("Pushing order\n");
//            received_orders_.push_back(*order_msg);
//
//            //order_ = *order_msg;
//            //--count for the number of orders in the trial config file
//
//            //-- number of orders the trial file has
//            //auto shipments = order1_.shipments;
//
//            //auto product_type = shipments[0].products[0].type;
//        }


        /// Called when a new JointState message is received.
        void arm_1_joint_state_callback(
                const sensor_msgs::JointState::ConstPtr & joint_state_msg)
        {
            ROS_INFO_STREAM_THROTTLE(10,
                                     "Joint States arm 1 (throttled to 0.1 Hz):\n" << *joint_state_msg);
            // ROS_INFO_STREAM("Joint States:\n" << *joint_state_msg);
            arm_1_current_joint_states_ = *joint_state_msg;
            if (!arm_1_has_been_zeroed_) {
                arm_1_has_been_zeroed_ = true;
                ROS_INFO("Sending arm to zero joint positions...");
                send_arm_to_zero_state(arm_1_joint_trajectory_publisher_);
            }
        }

        void arm_2_joint_state_callback(
                const sensor_msgs::JointState::ConstPtr & joint_state_msg)
        {
            ROS_INFO_STREAM_THROTTLE(10,
                                     "Joint States arm 2 (throttled to 0.1 Hz):\n" << *joint_state_msg);
            // ROS_INFO_STREAM("Joint States:\n" << *joint_state_msg);
            arm_2_current_joint_states_ = *joint_state_msg;
            if (!arm_2_has_been_zeroed_) {
                arm_2_has_been_zeroed_ = true;
                ROS_INFO("Sending arm 2 to zero joint positions...");
                send_arm_to_zero_state(arm_2_joint_trajectory_publisher_);
            }
        }


        /// Create a JointTrajectory with all positions set to zero, and command the arm.
        void send_arm_to_zero_state(ros::Publisher & joint_trajectory_publisher) {
            // Create a message to send.
            trajectory_msgs::JointTrajectory msg;

            // Fill the names of the joints to be controlled.
            // Note that the vacuum_gripper_joint is not controllable.
            msg.joint_names.clear();
            msg.joint_names.push_back("shoulder_pan_joint");
            msg.joint_names.push_back("shoulder_lift_joint");
            msg.joint_names.push_back("elbow_joint");
            msg.joint_names.push_back("wrist_1_joint");
            msg.joint_names.push_back("wrist_2_joint");
            msg.joint_names.push_back("wrist_3_joint");
            msg.joint_names.push_back("linear_arm_actuator_joint");
            // Create one point in the trajectory.
            msg.points.resize(1);
            // Resize the vector to the same length as the joint names.
            // Values are initialized to 0.
            msg.points[0].positions.resize(msg.joint_names.size(), 0.0);
            // How long to take getting to the point (floating point seconds).
            msg.points[0].time_from_start = ros::Duration(0.001);
            ROS_INFO_STREAM("Sending command:\n" << msg);
            joint_trajectory_publisher.publish(msg);
        }
        // %EndTag(ARM_ZERO)%

        /// Called when a new LogicalCameraImage message is received.
        void logical_camera_callback(
                const osrf_gear::LogicalCameraImage::ConstPtr & image_msg)
        {
            ROS_INFO_STREAM_THROTTLE(10,
                    "Logical camera: '" << image_msg->models.size() << "' objects.");
        }

        /// Called when a new Proximity message is received.
        void break_beam_callback(const osrf_gear::Proximity::ConstPtr & msg) {
            if (msg->object_detected) {  // If there is an object in proximity.
                ROS_INFO("Break beam triggered.");
            }
        }

    private:
        std::string competition_state_;
        double current_score_;
        ros::Publisher arm_1_joint_trajectory_publisher_;
        ros::Publisher arm_2_joint_trajectory_publisher_;

        sensor_msgs::JointState arm_1_current_joint_states_;
        sensor_msgs::JointState arm_2_current_joint_states_;
        bool arm_1_has_been_zeroed_;
        bool arm_2_has_been_zeroed_;
        osrf_gear::Order order_;

};


#endif //ARIAC_MANAGER_COMPETITION_H
