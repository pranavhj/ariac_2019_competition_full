#include <osrf_gear/AGVControl.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <tf/tf.h>
#include <iostream>
#include "order_manager.h"
#include "competition.h"


void StartCompetition(ros::NodeHandle & node) {
    ROS_INFO("Competition function.");
    // Create a Service client for the correct service, i.e. '/ariac/start_competition'.
    ros::ServiceClient start_client =
            node.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
    // If it's not already ready, wait for it to be ready.
    // Calling the Service using the client before the server is ready would fail.
    if (!start_client.exists()) {
        ROS_INFO("Waiting for the competition to be ready...");
        start_client.waitForExistence();
        ROS_INFO("Competition is now ready.");
    }
    ROS_INFO("Requesting competition start...");
    std_srvs::Trigger srv;  // Combination of the "request" and the "response".
    start_client.call(srv);  // Call the start Service.
    if (!srv.response.success) {  // If not successful, print out why.
        ROS_ERROR_STREAM("Failed to start the competition: " << srv.response.message);
    } else {
        ROS_INFO("Competition started!");
    }
}

void EndCompetition(ros::NodeHandle &node) {
    ros::ServiceClient start_client =
            node.serviceClient<std_srvs::Trigger>("/ariac/end_competition");

    if (!start_client.exists()) {
        ROS_INFO("Waiting for the competition to be End...");
        start_client.waitForExistence();
        ROS_INFO("Competition  now Ended.");
    }
    ROS_INFO("Requesting competition End...");
    std_srvs::Trigger srv;
    start_client.call(srv);
    if (!srv.response.success) {
        ROS_ERROR_STREAM(
                "Failed to End the competition: " << srv.response.message);
    } else {
        ROS_INFO("Competition Ended!");
    }
}

int main(int argc, char **argv) {
    ROS_INFO("Starting main function");
    ros::init(argc, argv, "ariac_manager_node");
    ros::NodeHandle node;
    AriacOrderManager manager;
    Competition comp(node);

    ros::Subscriber current_score_subscriber = node.subscribe(
            "/ariac/current_score", 10,
            &Competition::current_score_callback, &comp);

    // Subscribe to the '/ariac/competition_state' topic.
    ros::Subscriber competition_state_subscriber = node.subscribe(
            "/ariac/competition_state", 10,
            &Competition::competition_state_callback, &comp);



    ROS_INFO("Setup complete.");


    StartCompetition(node);
    ros::Duration(2.0).sleep();
    //manager.SetScannedProducts();
    manager.ExecuteOrder();

    ros::spin();  // This executes callbacks on new data until ctrl-c.

    //manager.ExecuteOrder();
    //EndCompetition(node);

    // ROS_WARN_STREAM("Killing the node....");

    return 0;
}