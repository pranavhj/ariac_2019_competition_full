//
// Created by zeid on 2/27/20.
//

#include "order_manager.h"
#include <osrf_gear/AGVControl.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>



//AriacOrderManager::AriacOrderManager(): arm1_{"arm1"}, arm2_{"arm2"}
AriacOrderManager::AriacOrderManager()//: //arm1_("arm1")
{   InitializeEmptyBinCoord("pulley_part");
    order_subscriber_ = order_manager_nh_.subscribe(
            "/ariac/orders", 10,
            &AriacOrderManager::OrderCallback, this);


    // break_beam_subscriber_ = order_manager_nh_.subscribe(
    //         "/ariac/break_beam_1_change", 10,
    //         &AriacOrderManager::break_beam_callback, this);

    camera_4_subscriber_ = order_manager_nh_.subscribe("/ariac/logical_camera_4", 10,
                                                &AriacOrderManager::LogicalCamera4Callback, this);

    camera_1_subscriber_ = order_manager_nh_.subscribe("/ariac/logical_camera_1", 10,
                                                &AriacOrderManager::LogicalCamera1Callback, this);

    camera_5_subscriber_ = order_manager_nh_.subscribe("/ariac/logical_camera_8", 10,
                                                &AriacOrderManager::LogicalCamera5Callback, this);

    camera_5_2_subscriber_ = order_manager_nh_.subscribe("/ariac/logical_camera_9", 10,
                                                &AriacOrderManager::LogicalCamera5_2Callback, this);

    camera_repick_subscriber_ = order_manager_nh_.subscribe("/ariac/logical_camera_5", 10,
                                                &AriacOrderManager::LogicalCameraRepickCallback, this);


    joint_states_subscriber_ = order_manager_nh_.subscribe("/ariac/arm1/joint_states", 10,
                                                &AriacOrderManager::JointStatesCallback, this);

    joint_states_subscriber_2_ = order_manager_nh_.subscribe("/ariac/arm2/joint_states", 10,
                                                &AriacOrderManager::JointStatesCallback_2, this);

	indices_conveyer_="/ariac/logical_camera_x";
	indices_conveyer_callback_="/logical_camera_x_frame";
	//std::string  empty_bin= camera_.GetEmptyBin();
	std::string empty_bin=camera_.getEmptyBins()[0];

	indices_conveyer_[22]=logical_camera_indices_.at((empty_bin[4]-'0')-1);
	indices_conveyer_callback_[16]=logical_camera_indices_.at((empty_bin[4]-'0')-1);
	ROS_INFO_STREAM("Conveyer Pickup Bin Logical Camera is     "<<indices_conveyer_);

    camera_conveyer_indices_subscriber_=order_manager_nh_.subscribe(indices_conveyer_, 10,
                                                &AriacOrderManager::LogicalCameraConveyerCallback, this);

}


AriacOrderManager::~AriacOrderManager(){}

bool AriacOrderManager::GetSensorBlackout(){
	auto temp_time=ros::Time::now().toSec();
	if (temp_time-start_time_>200     &&   temp_time-start_time_<250 ){
		return true;
	}

return false;
}

void AriacOrderManager::InitializeMiddleVector(float sign){
	middle_poses_vector_.clear();
	geometry_msgs::Pose temp;
	temp.position.x=0.305923;
	temp.position.y=-0.445270;
	temp.position.z=0.952811+0.2;
	for (int i=0;i<4;i++){
		temp.position.y+=(sign*0.3);
		middle_poses_vector_.push_back(temp);
	}
}


void AriacOrderManager::InitializeEmptyBinCoord(std::string pulley_part){
	std::pair<float, float> temp;
	
	for(int i=0;i<2;i++){
		for(int j=0;j<2;j++){
			temp.first=0.15-(float(i)*0.3);
			temp.second=0.15-(float(j)*0.3);
			ROS_INFO_STREAM("Poses are "<<std::to_string(temp.first)+"  "+std::to_string(temp.second));
			empty_bin_coordinates_.push_back(temp);
		}
}
//temp.first=0.30;
//temp.second=0.30;
empty_bin_coordinates_.push_back(temp);


}

void AriacOrderManager::OrderCallback(const osrf_gear::Order::ConstPtr& order_msg) {
    ROS_WARN(">>>>> OrderCallback");
    received_orders_.push_back(*order_msg);
    if ((*order_msg).order_id.size()>13){
    	ROS_WARN("ORDER UPDATE RECEIVED.....");
    	order_update_flag_=1;
    }
}


// void AriacOrderManager::break_beam_callback(const osrf_gear::Proximity::ConstPtr& msg) {
    

//     double temp=ros::Time::now().toSec();
//     if (flag_==1)
//       {//ROS_INFO_STREAM("Time Delay observed"<<temp-camera_4_detection_time_);
// }
//     if (flag_==1     &&    temp-camera_4_detection_time_>4.50)
//     {
//     if(msg->object_detected) {
//         //ROS_INFO("Part Required Detected by Break beam triggered..");
//         flag_break_beam_ = 1;
//         break_beam_time = ros::Time::now();
//     }
//   }
// }
void AriacOrderManager::LogicalCamera4Callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg){
    //For conveyer pickup
    

    if (init_==0)
      return;
  	//ROS_INFO_STREAM("Cam4");
    for(int i=0; i<image_msg->models.size();i++){//ROS_INFO_STREAM("Part detected");
       if (image_msg->models[i].type==camera_4_product_){
        //ROS_INFO_STREAM("Part Required Detected");
        flag_=1;
        camera_4_detection_time_=ros::Time::now().toSec();
        //ROS_INFO_STREAM("Camera Detected at "<<camera_4_detection_time_);
        geometry_msgs::PoseStamped StampedPose_in,StampedPose_out;          
        StampedPose_in.header.frame_id = "/logical_camera_4_frame";
        StampedPose_in.pose = image_msg->models[i].pose;
        //ROS_INFO_STREAM("StampedPose_int (" << StampedPose_in.pose.position.x <<","<< StampedPose_in.pose.position.y << "," << StampedPose_in.pose.position.z<<")");
        part_tf_listener_.transformPose("/world",StampedPose_in,StampedPose_out);
        camera4_op_pose_=StampedPose_out.pose;
        //ROS_INFO_STREAM("part is at   "<<camera4_op_pose_);
       }

     }
     if (image_msg->models.size()==0)
      {flag_=0;}
    init_=1;}



void AriacOrderManager::LogicalCameraConveyerCallback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg){
	//ROS_INFO_STREAM("hello");

	if(init_conveyer_bin_==1){
		//return;
	part_present_=0;
	for(int i=0; i<image_msg->models.size();i++){
		//ROS_INFO_STREAM("Callback working");
       if (image_msg->models[i].type==product_type_conveyer_bin_){
       		ROS_INFO_STREAM("Part Conveyer pickup detected");
       		geometry_msgs::PoseStamped StampedPose_in,StampedPose_out;          
	        StampedPose_in.header.frame_id = indices_conveyer_callback_;          //logical camera frame
	        StampedPose_in.pose = image_msg->models[i].pose;
	        //ROS_INFO_STREAM("StampedPose_int (" << StampedPose_in.pose.position.x <<","<< StampedPose_in.pose.position.y << "," << StampedPose_in.pose.position.z<<")");
	        part_tf_listener_.transformPose("/world",StampedPose_in,StampedPose_out);
	        conveyer_pick_up_pose_=StampedPose_out.pose;
	    	ROS_INFO_STREAM("Pose of the part is  "<<conveyer_pick_up_pose_);
	    	part_present_=1;
	    }


	}
}
}


void AriacOrderManager::LogicalCamera1Callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg){
//For Flipping


if (init_cam1_==1){
	//ROS_INFO_STREAM("Pulley detected for flipping");
	//init_cam1_=0;
	pulley_on_bin_=false;
	 geometry_msgs::PoseStamped StampedPose_in,StampedPose_out;          
	for(int i=0;i<image_msg->models.size();i++){
		if (image_msg->models[i].type=="pulley_part"   )// &&   abs(image_msg->models[i].pose.position.z-0.15)>0.1){  //to see if pulley is still standing
			//geometry_msgs::PoseStamped StampedPose_in,StampedPose_out;          
        	{StampedPose_in.header.frame_id = "/logical_camera_1_frame";
       		StampedPose_in.pose = image_msg->models[i].pose;
        	//ROS_INFO_STREAM("StampedPose_int (" << StampedPose_in.pose.position.x <<","<< StampedPose_in.pose.position.y << "," << StampedPose_in.pose.position.z<<")");
        	part_tf_listener_.transformPose("/world",StampedPose_in,StampedPose_out);
        	flipping_pulley_pose_=StampedPose_out.pose;
        	pulley_on_bin_=true;

			//flipping_pulley_pose_
        	}
		}
	}
}


void AriacOrderManager::LogicalCameraRepickCallback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg){
//For repick

//ROS_INFO_STREAM("HELLO  "<<init_repick_);
if (init_repick_ != 0){
	
	//ROS_INFO_STREAM("Camera loop");
	int flag=0;
	//ROS_INFO_STREAM("Pulley detected for flipping");
	//init_cam1_=0;
	//pulley_on_bin_=false;
	geometry_msgs::PoseStamped StampedPose_in,StampedPose_out;          
	for(int i=0;i<image_msg->models.size();i++){
		if (image_msg->models[i].type==repick_part_   )// &&   abs(image_msg->models[i].pose.position.z-0.15)>0.1){  //to see if pulley is still standing
			//geometry_msgs::PoseStamped StampedPose_in,StampedPose_out;          
        	{StampedPose_in.header.frame_id = "/logical_camera_5_frame";
       		StampedPose_in.pose = image_msg->models[i].pose;
        	//ROS_INFO_STREAM("StampedPose_int (" << StampedPose_in.pose.position.x <<","<< StampedPose_in.pose.position.y << "," << StampedPose_in.pose.position.z<<")");
        	part_tf_listener_.transformPose("/world",StampedPose_in,StampedPose_out);
        	repick_pose_=StampedPose_out.pose;
        	//pulley_on_bin_=true;
        	flag=1;
        	//ROS_INFO_STREAM("PArt to Repicking found");
			//flipping_pulley_pose_
        	}
		}
	if (flag!=1)
		init_repick_=-1;
}
}



void AriacOrderManager::LogicalCamera5Callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg){
	//for kit tray1
	if (init_cam5_==1)
		{
	cam5_list_.clear();
	
	for(int i=0;i<image_msg->models.size();i++){
		std::pair<std::string,geometry_msgs::Pose> product_type_pose_cam5;
		product_type_pose_cam5.first=image_msg->models[i].type;
		product_type_pose_cam5.second=image_msg->models[i].pose;
		//ROS_INFO_STREAM("  "<<product_type_pose_cam5.first);
		cam5_list_.push_back(product_type_pose_cam5);

		}
		ROS_INFO_STREAM("Making List of parts on kit tray 1 with these many components   "<<cam5_list_.size()  );
	}
	init_cam5_=0;
	repose_flag_=0;
	geometry_msgs::PoseStamped StampedPose_in,StampedPose_out; 
	if(init_cam5_2_==1){
		//ROS_INFO_STREAM("Went into repose_flag_loop");
		for(int i=0;i<image_msg->models.size();i++){
			StampedPose_in.header.frame_id = "/logical_camera_8_frame";
       		StampedPose_in.pose = image_msg->models[i].pose;
        	//ROS_INFO_STREAM("StampedPose_int (" << StampedPose_in.pose.position.x <<","<< StampedPose_in.pose.position.y << "," << StampedPose_in.pose.position.z<<")");
        	part_tf_listener_.transformPose("/world",StampedPose_in,StampedPose_out);
        	repick_pose_=StampedPose_out.pose;
        	//ROS_INFO_STREAM("///////////////");
        	if (euler_distance(repick_pose_,reposition_part_pose_)<0.01   &&   reposition_part_type_==image_msg->models[i].type){
        		//ROS_INFO_STREAM("Part at right position detected sending back pose");
        		reposition_part_pose_back_=StampedPose_out.pose;
        		repose_flag_=1;

        	}
        }

	//init_cam5_2_=0;
	}
	

	}



void AriacOrderManager::LogicalCamera5_2Callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg){
	//for kit tray2
	if (init_cam5_dash_==1)
		{
	cam5_list_dash_.clear();
	
	for(int i=0;i<image_msg->models.size();i++){
		std::pair<std::string,geometry_msgs::Pose> product_type_pose_cam5;
		product_type_pose_cam5.first=image_msg->models[i].type;
		product_type_pose_cam5.second=image_msg->models[i].pose;
		//ROS_INFO_STREAM("  "<<product_type_pose_cam5.first);
		cam5_list_dash_.push_back(product_type_pose_cam5);

		}
		ROS_INFO_STREAM("Making List of parts on kit tray 2 with these many components   "<<cam5_list_dash_.size()  );
	}
	init_cam5_dash_=0;
	repose_flag_dash_=0;
	geometry_msgs::PoseStamped StampedPose_in,StampedPose_out; 
	if(init_cam5_2_dash_==1){
		for(int i=0;i<image_msg->models.size();i++){
			StampedPose_in.header.frame_id = "/logical_camera_9_frame";
       		StampedPose_in.pose = image_msg->models[i].pose;
        	//ROS_INFO_STREAM("StampedPose_int (" << StampedPose_in.pose.position.x <<","<< StampedPose_in.pose.position.y << "," << StampedPose_in.pose.position.z<<")");
        	part_tf_listener_.transformPose("/world",StampedPose_in,StampedPose_out);
        	repick_pose_dash_=StampedPose_out.pose;
        	//ROS_INFO_STREAM("///////////////");
        	if (euler_distance(repick_pose_dash_,reposition_part_pose_dash_)<0.005   &&   reposition_part_type_dash_==image_msg->models[i].type){
        		//ROS_INFO_STREAM("Part at right position detected sending back pose");
        		reposition_part_pose_back_dash_=StampedPose_out.pose;
        		repose_flag_dash_=1;

        	}
        }

	//init_cam5_2_dash_=0;
	}
	

	}


void AriacOrderManager::ContinueDuringBlackout(){
	int agv_id=0;
	//agv_id=0;
    ROS_INFO_STREAM("Blackout loop entered.");
	for(int i=0;i<current_shipment_vector_.size();i++){
		ROS_INFO_STREAM("Continuing to work during Blackout");
		if (product_frame_list_.find(current_shipment_vector_[i].first)!=product_frame_list_.end()){  //Found in list
		    if (current_shipment_.agv_id=="any"   ||current_shipment_.agv_id=="agv1"){
		      PickAndPlace(current_shipment_vector_[i], 1 ,0);
		  	}
		  	else{
	  		  PickAndPlace(current_shipment_vector_[i],2 ,0);
		  	}
	  	}
	  	
	}

while(1){
	if(camera_.sensorBlackout()==false){
		break;

	}

	}
}


	

	


void AriacOrderManager::JointStatesCallback(const sensor_msgs::JointState::ConstPtr& joint_states){
    //ROS_WARN(">>>>> OrderCallback");
    if (camera_.sensorBlackout()) {
    	// while(1) {
    	// 	if(!camera_.sensorBlackout()){
    	// 		ROS_WARN("Complete");
    	// 		break;
    	// 	}
    	// }
    	//ROS_INFO_STREAM("Blackout detected..............");
    }
    		//ros::Duration(1.0).sleep();
    		// ContinueDuringBlackout();
    	

    auto positions=joint_states->position;
    std::vector <int>indices={1,3,2,0,4,5,6};
    joint_states_.clear();
    for(int i=0;i<indices.size();i++){
    	joint_states_.push_back(positions[indices[i]]);
    }
}


void AriacOrderManager::JointStatesCallback_2(const sensor_msgs::JointState::ConstPtr& joint_states){
    //ROS_WARN(">>>>> OrderCallback");
    // if (camera_.sensorBlackout()) {
    // 	while(1) {
    // 		if(!camera_.sensorBlackout()) return;
    // 	}
    // } //ros::Duration(1.0).sleep();
    auto positions=joint_states->position;
    std::vector <int>indices={1,3,2,0,4,5,6};
    joint_states_2_.clear();
    for(int i=0;i<indices.size();i++){
    	joint_states_2_.push_back(positions[indices[i]]);
    }
}

/**
 * @brief Get the product frame for a product type
 * @param product_type
 * @return
 */
std::string AriacOrderManager::GetProductFrame(std::string product_type) {
    //--Grab the last one from the list then remove it
    std::string frame;
    if (!product_frame_list_.empty()) {
    	try{
        	frame = product_frame_list_[product_type].back();
        	ROS_INFO_STREAM("Frame >>>> " << frame);
        	if (frame.size() == 0){
        		throw std::exception();
        	}
        	product_frame_list_[product_type].pop_back();
        }
        catch(std::exception& e){
        	ROS_ERROR_STREAM("In exception handlling for no product found");
        	return "empty";
        }
        // catch(...){
        // 	ROS_ERROR_STREAM("any other exception apart from standard");
        // 	return "empty";
        // }
        return frame;
    } else {

        return "empty";
    }
}



int AriacOrderManager::GetFromArm2(std::string product_type,geometry_msgs::Pose part_pose,std::string product_frame,int agv_id){
	int arm_id;
	std::string arm1,arm_dash1;

	RobotController *arm;
	RobotController *arm_dash;


	if (agv_id==1){
		arm1={"arm1"};
		arm_dash1={"arm2"};
		arm_id=2;
		arm= &arm1_;
		arm_dash= &arm2_;

	}

	if (agv_id==2){
		arm1={"arm2"};
		arm_dash1={"arm1"};
		arm_id=2;
		arm= &arm2_;
		arm_dash= &arm1_;
	}
	
	bool failed_pick = arm_dash->PickPart(part_pose);
	ROS_WARN_STREAM("Picking up state arm2" << failed_pick);
	ros::Duration(0.2).sleep();


    	// while(!failed_pick){
     //    	auto part_pose = camera_.GetPartPose("/world",product_frame);
     //    	failed_pick = arm_dash->PickPart(part_pose);
     //    	//n=n+1;
     //    	ROS_INFO_STREAM("Attempt no to pick up part");
        	

    	// }
	//Part has been picked up by arm2
    	if(arm_dash->GetGripperStatus()==false){
    		ROS_INFO_STREAM("Arm2 has dropped part");
    		return -1;
    	}

    	arm->SendRobotHome();
	    auto place_pose=part_pose;
	    place_pose.position.x=0.31292-0.04;
	    place_pose.position.y=-0.5954;
	    place_pose.position.z=0.9499;
	    place_pose.position.z+=0.1;
	    if(product_type == "pulley_part")
        	place_pose.position.z += 0.08;
	   	
	   	if(arm_id==2){ 
	    	home_joint_pose_[1]=1.55;
	    	home_joint_pose_[0]-=0.4;
		}
		if(arm_id==1){ 
	    	home_joint_pose_[1]=3.1+1.55;
	    	home_joint_pose_[0]-=0.4;
		}
	    while(1){         
		arm_dash->SendRobotJointPosition(home_joint_pose_);//ros::spinOnce();
		ROS_INFO_STREAM("Diff is "<<arm_dash->GetJointStates()[1]-home_joint_pose_[1]);
		if (arm_dash->GetJointStates()[1]-home_joint_pose_[1]<0.1 )
			break;
		}
		 
		home_joint_pose_[1]=3.1;
    	home_joint_pose_[0]+=0.4;
		

		arm_dash->GoToTarget(place_pose);
		while(1){
			ROS_INFO_STREAM("Send arm2 to drop part "<<abs(arm_dash->GetEEPose().position.x-0.31292+0.04));
			arm_dash->GoToTarget(place_pose);
			//ros::spinOnce();
			if (abs(arm_dash->GetEEPose().position.x-0.31292+0.04)<0.05){
				ROS_INFO_STREAM("State achieved");
				break;
			}
		}
		if(arm_dash->GetGripperStatus()==false){
			ROS_INFO_STREAM("Part droppped somewhere go again");
			return -1;
		}

		arm_dash->GripperToggle(false);
		
		while(1){
			ROS_INFO_STREAM("Send arm2 home");
			arm_dash->SendRobotHome();
			ros::spinOnce();
			if (arm_dash->GetJointStates()[0]-home_joint_pose_[0]<0.05)
				break;
		}
		//home_joint_pose_[1]=3.1+1.5;
		if(arm_id==2){ 
	    	home_joint_pose_[1]=3.1+1.55;
	    	
		}
		if(arm_id==1){ 
	    	home_joint_pose_[1]=1.55;
	    	
		}
		while(1){
			ROS_INFO_STREAM("Send arm1 home twisted");
			arm->SendRobotJointPosition(home_joint_pose_);
			//ros::spinOnce();
			if (abs(arm->GetJointStates()[1]-(home_joint_pose_[1]))<0.1)
				break;
		}
		home_joint_pose_[1]=3.1;


		place_pose.position.z-=0.1;
		place_pose.position.z+=0.025;
		//if(product_type == "pulley_part")
        	//place_pose.position.z += 0.08;
		auto up_pose=place_pose;
		up_pose.position.z+=0.2;
		int n=0;
		while(1){//Pick up part by arm1
			arm->GripperToggle(true);
			arm->GoToTarget({up_pose,place_pose});
			if (arm->GetGripperStatus()==true){
				ROS_INFO_STREAM("Product attached");
				break;
			}
			place_pose.position.z-=0.005;
			if (n>5)
				return -1;
			n++;
		}
		//Product picked up by arm1
		


		while(1){//Send arm1 home
			ROS_INFO_STREAM("Send arm1 home ");
			arm->SendRobotHome();
			//ros::spinOnce();
			if (arm->GetJointStates()[0]-home_joint_pose_[0]<0.05)
				break;
		}
		return 0;
}







int AriacOrderManager::GiveArm2(std::string product_type,geometry_msgs::Pose part_pose){
	int agv_id=2;       // Dont change this while checking
	int arm_id;
	std::string arm1,arm_dash1;

	RobotController *arm;
	RobotController *arm_dash;


	if (agv_id==1){
		arm1={"arm1"};
		arm_dash1={"arm2"};
		arm_id=2;
		arm= &arm1_;
		arm_dash= &arm2_;

	}

	if (agv_id==2){
		arm1={"arm2"};
		arm_dash1={"arm1"};
		arm_id=2;
		arm= &arm2_;
		arm_dash= &arm1_;
	}


	arm_dash->SendRobotHome();
	arm->SendRobotHome();
	
	
	//Part has been picked up by arm2
	ros::spinOnce();
	if(arm_dash->GetGripperStatus()==false){
		ROS_INFO_STREAM("Arm1 has dropped part");
		return -1;
    	}

    	
	    auto place_pose=part_pose;
	    place_pose.position.x=0.31292-0.08;
	    place_pose.position.y=-0.5954;
	    place_pose.position.z=0.9499;
	    place_pose.position.z+=0.1;
	    if(product_type == "pulley_part")
        	place_pose.position.z += 0.08;
	   	
	   	if(arm_id==2){ 
	    	home_joint_pose_[1]=1.55;
	    	home_joint_pose_[0]-=0.4;
		}
		if(arm_id==1){ 
	    	home_joint_pose_[1]=3.1+1.55;
	    	home_joint_pose_[0]-=0.4;
		}
	    while(1){         
		arm_dash->SendRobotJointPosition(home_joint_pose_);ros::spinOnce();
		ROS_INFO_STREAM("Diff is "<<arm_dash->GetJointStates()[0]-home_joint_pose_[0]);
		if (arm_dash->GetJointStates()[0]-home_joint_pose_[0]<0.1 )
			break;
		}
		 
		home_joint_pose_[1]=3.1;
    	home_joint_pose_[0]+=0.4;
		

		arm_dash->GoToTarget(place_pose);
		while(1){
			ROS_INFO_STREAM("Send arm1 to drop part ");
			arm_dash->GoToTarget(place_pose);
			ros::spinOnce();
			if (abs(arm_dash->GetEEPose().position.x-0.31292+0.08)<0.05){
				ROS_INFO_STREAM("State achieved");
				break;
			}
		}
		ros::spinOnce();
		if(arm_dash->GetGripperStatus()==false){
			ROS_INFO_STREAM("Part droppped somewhere go again");
			return -1;
		}

		arm_dash->GripperToggle(false);
		
		while(1){
			ROS_INFO_STREAM("Send arm2 home");
			arm_dash->SendRobotHome();
			ros::spinOnce();
			if (arm_dash->GetJointStates()[0]-home_joint_pose_[0]<0.05)
				break;
		}
		//home_joint_pose_[1]=3.1+1.5;
		if(arm_id==2){ 
	    	home_joint_pose_[1]=3.1+1.55;
	    	
		}
		if(arm_id==1){ 
	    	home_joint_pose_[1]=1.55;
	    	
		}
		while(1){
			ROS_INFO_STREAM("Send arm2 home twisted");
			arm->SendRobotJointPosition(home_joint_pose_);
			ros::spinOnce();
			if (abs(arm->GetJointStates()[1]-(home_joint_pose_[1]))<0.1)
				break;
		}
		home_joint_pose_[1]=3.1;


		place_pose.position.z-=0.1;
		place_pose.position.z+=0.02;
		if(product_type == "pulley_part")
        	place_pose.position.z += 0.08;
		auto up_pose=place_pose;
		up_pose.position.z+=0.2;
		int n=0;
		while(1){//Pick up part by arm1
			arm->GripperToggle(true);
			arm->GoToTarget({up_pose,place_pose});
			if (arm->GetGripperStatus()==true){
				ROS_INFO_STREAM("Product attached by arm2");
				break;
			}
			place_pose.position.z-=0.005;
			if (n>5)
				return -1;
			n++;
		}
		//Product picked up by arm2
		


		while(1){//Send arm1 home
			ROS_INFO_STREAM("Send arm2 home ");
			arm->SendRobotHome();
			ros::spinOnce();
			if (arm->GetJointStates()[0]-home_joint_pose_[0]<0.05)
				break;
		}

		while(1){
			arm->GoToTarget(part_pose);
			if ( abs( arm->GetEEPose().position.x- part_pose.position.x)<0.01){
				ROS_INFO_STREAM("Reached bin keeping pose");
				break;
			}
			ros::spinOnce();
			if (arm->GetGripperStatus()==false){
				continue;
			}
		}


	ROS_INFO_STREAM("Dropping part on empty bin");
	arm->GripperToggle(false);


		return 0;
}













bool AriacOrderManager::PickAndPlace(const std::pair<std::string,geometry_msgs::Pose> product_type_pose, int agv_id,int empty_flag) {
    //agv_id=2;
    ROS_INFO_STREAM("Pick and Place started.");
    while(1){
	int arm_id;
	std::string arm1,arm_dash1;

	RobotController *arm;
	RobotController *arm_dash;


	if (agv_id==1){
		arm1={"arm1"};
		arm_dash1={"arm2"};
		arm_id=2;
		arm= &arm1_;
		arm_dash= &arm2_;

	}

	if (agv_id==2){
		arm1={"arm2"};
		arm_dash1={"arm1"};
		arm_id=2;
		arm= &arm2_;
		arm_dash= &arm1_;
	}

	arm->SendRobotHome();

    if(agv_id==2){
    	while(1){
    		arm_dash->SendRobotJointPosition(end_position_);
    		if ((arm_dash->GetJointStates()[0]-1.18)<0.01){
    			break;
    		}
    	}
    }

    std::string product_type,product_frame;
    geometry_msgs::Pose part_pose;
    if(empty_flag==0){
    	ROS_INFO_STREAM("PArt already on bin");
	    product_type = product_type_pose.first;
	    ROS_WARN_STREAM("Product type >>>> " << product_type);
	    product_frame = this->GetProductFrame(product_type);
	    if (product_frame == "empty")
	    	return false;
	    ROS_WARN_STREAM("Product frame >>>> " << product_frame);
	    part_pose = camera_.GetPartPose("/world",product_frame);
	}

	else{
		product_type = product_type_pose.first;
	    ROS_WARN_STREAM("Product was not on bin " << product_type);
	    init_conveyer_bin_=1;
	    product_type_conveyer_bin_=product_type;
	    ros::Duration(0.1).sleep();
		ros::spinOnce();
		ros::Duration(0.1).sleep();
		part_pose=conveyer_pick_up_pose_;
		init_conveyer_bin_=0;
		ROS_INFO_STREAM("Conveyer Bin part Pose is   "<<part_pose);
		if(part_present_==0){
			ROS_INFO_STREAM("Part not on conveyer on empty bin");
			return false;
		}
	}



    // if(product_type == "pulley_part")
    //     part_pose.position.z += 0.06;
    //--task the robot to pick up this part
    int repick_flag=0;
    
    
    if (agv_id==1   &&    part_pose.position.y<-0.65*2){  //arm1 cannot reach this position
    	repick_flag=1;
    	
    	int result=GetFromArm2(product_type,part_pose,product_frame,1);
    	if (result==-1)
    		continue;


    }
    else if(agv_id==2   &&    part_pose.position.y>0.65*2){
    	repick_flag=1;
    	
    	int result=GetFromArm2(product_type,part_pose,product_frame,2);
    	if (result==-1)
    		continue;

    }
    
    
    else{//Sadha Pick up
    	
	    ROS_INFO_STREAM("Sadha Pick up");
	    int n=0;
	    part_pose.position.z+=0.015;
	    if(product_type=="pulley_part"){
	    	part_pose.position.z+=0.055;
	    }
	    if (product_type=="disk_part"){
	    	// part_pose.position.z+=0.005;
	    }
	    if (product_type=="piston_rod_part"){
	    	part_pose.position.z-=0.015;
	    }

	    if (product_type=="gear_part"){
	    	part_pose.position.z+=0.01;
	    }
	    bool failed_pick = arm->PickPart(part_pose);
	    ROS_WARN_STREAM("Picking up state " << failed_pick);
	    
	    ros::Duration(0.1).sleep();

	    // while(!failed_pick){
	    //     auto part_pose = camera_.GetPartPose("/world",product_frame);
	    //     failed_pick = arm->PickPart(part_pose);
	    //     n=n+1;
	    //     ROS_INFO_STREAM("Attempt no to pick up part"  <<n);
	    // }
	    ROS_INFO_STREAM("Successfully picked up");
	}
    

    
    if (product_type=="pulley_part"){
                      
    	//do something for second arm
		auto temp = pulleyFlipper(agv_id);

	}	



    //--get the pose of the object in the tray from the order
    geometry_msgs::Pose drop_pose = product_type_pose.second;

    geometry_msgs::PoseStamped StampedPose_in,StampedPose_out;

    


    if(agv_id==1){
        StampedPose_in.header.frame_id = "/kit_tray_1";
        StampedPose_in.pose = drop_pose;
        ROS_INFO_STREAM("StampedPose_int (" << StampedPose_in.pose.position.x <<","<< StampedPose_in.pose.position.y << "," << StampedPose_in.pose.position.z<<")");
        part_tf_listener_.transformPose("/world",StampedPose_in,StampedPose_out);
        StampedPose_out.pose.position.z += 0.1;
        StampedPose_out.pose.position.y -= 0.2;
        ROS_INFO_STREAM("StampedPose_out (" << StampedPose_out.pose.position.x <<","<< StampedPose_out.pose.position.y << "," << StampedPose_out.pose.position.z<<")");

    }
    else{
        StampedPose_in.header.frame_id = "/kit_tray_2";
        StampedPose_in.pose = drop_pose;
        //ROS_INFO_STREAM("StampedPose_in " << StampedPose_in.pose.position.x);
        part_tf_listener_.transformPose("/world",StampedPose_in,StampedPose_out);
        StampedPose_out.pose.position.z += 0.1;
        StampedPose_out.pose.position.y += 0.2;
        StampedPose_out.pose.position.y-=0.06;

        //ROS_INFO_STREAM("StampedPose_out " << StampedPose_out.pose.position.x);
    }

    ros::spinOnce();
    
    if (arm->GetGripperStatus()==false){
    	ROS_WARN("Part has been dropped somewhere");
    	
    	arm->SendRobotHome();
    	continue;
    }


    ros::spinOnce();
	if (repick_flag==1    &&   agv_id==1){             //Because while repickig its perfectly picked up 
		StampedPose_out.pose.position.y+=0.06;
	}
	if (repick_flag==1    &&   agv_id==2){             //Because while repickig its perfectly picked up 
		StampedPose_out.pose.position.y-=0.06;
	}
	//end_position_[1]=1.55;
	
	
	if(agv_id==1){
	while(1){         
		arm->SendRobotJointPosition(end_position_);
		ROS_INFO_STREAM("Diff is "<<arm->GetJointStates()[0]-end_position_[0]);
		ros::spinOnce();
		if (arm->GetJointStates()[0]-1.18<0.1 )
			break;
		}
	}
	if(agv_id==2){
	while(1){         
		arm->SendRobotJointPosition(home_joint_pose_);
		ROS_INFO_STREAM("Diff is for arm2 "<<arm->GetJointStates()[0]-home_joint_pose_[0]);
		ros::spinOnce();
		if (arm->GetJointStates()[0]-(home_joint_pose_[0])<0.1 )
			break;
		}
	}


	//end_position_[1]=3.1;
	ros::Duration(0.1).sleep();
    auto result = arm->DropPart(product_type,StampedPose_out.pose);

    bool result1;
    if (result==-1){
      ROS_INFO_STREAM("Retrying due to defected part or dropped part");
      // std::pair<std::string,geometry_msgs::Pose> retry_pair;
      // retry_pair.first=product_type;
      // retry_pair.second=drop_pose;
      // //Make it go to home first
      arm->SendRobotHome();
      ros::Duration(0.1).sleep();
      arm->GripperToggle(false);   //if the part is defected it will drop it
      continue;
      //result1=PickAndPlace(retry_pair, int(1)); }
    }


    if (result==0)
      result1=false;
    else
      result1=true;
    return result1;
	
	


	
}
}


void printVector(std::vector<auto> vect)
{for (int i=0;i<vect.size();i++)
	{ROS_INFO_STREAM(vect[i]);}
	}




void AriacOrderManager::PickFromConveyer(std::string product_type,int agv_id)
{  


camera_4_product_=product_type;   //Let camera detect product_type parts
    //agv_id=2;
	int arm_id;
	std::string arm1,arm_dash1;
	
	
	arm1={"arm1"};
	arm_dash1={"arm2"};
	arm_id=2;


	RobotController *arm= &arm1_;
	arm->SendRobotHome();
	RobotController *arm_dash= &arm2_;
	arm_dash->SendRobotHome();

   ROS_INFO_STREAM("Picking up from conveyer");
   
   //std::vector<double> conveyer_joint_pose_ = {-1.1800,0.0,-1.1, 1.9, 3.9, 4.7, 0};
   arm->SendRobotConveyer();
 
   auto temp_pose=camera4_op_pose_;
   temp_pose.position.z=temp_pose.position.z-0.1;

   geometry_msgs::Pose temporaryPose;
   temporaryPose.position.x = 1.193;
   temporaryPose.position.y = 0.407;
   temporaryPose.position.z = 1.175+0.1 ;
   
   auto temp_pose_1 = temporaryPose;
   temp_pose_1.position.z -= (0.120);
   if (product_type=="pulley_part")
    {
     temp_pose_1.position.z += (0.120+0.08-0.04-0.1); 
    }                                       //Take robot to conveyer position
   arm->GoToTarget({ temporaryPose,temp_pose_1,});       
   int n=0;
   
   while(1){   //for picking up from pulley
   
    init_=1;
    ros::spinOnce();
    if (arm->GetGripperStatus()==true){
            ROS_INFO_STREAM("Product is attached");
            break;}
    //auto time1=ros::Time::now();
    auto up_pose=camera4_op_pose_;
    up_pose.position.z=up_pose.position.z+0.1;
    up_pose.position.y=up_pose.position.y+0.10;
    if (flag_==1){
	    ROS_INFO_STREAM("Moving down...");
	    camera4_op_pose_.position.y=camera4_op_pose_.position.y-0.25+0.008;
	    arm->GripperToggle(true);
	    camera4_op_pose_.position.z=camera4_op_pose_.position.z+0.02;
	    if (product_type=="pulley_part"){
    		if ( camera4_op_pose_.position.y<0.425){
    			ROS_INFO_STREAM("1st if");
    			camera4_op_pose_.position.z=camera4_op_pose_.position.z+0.08-0.0110;
    			camera4_op_pose_.position.y=camera4_op_pose_.position.y+0.00005-0.03;    			
    		}
	    	if ( camera4_op_pose_.position.y>0.425){
	    		ROS_INFO_STREAM("2nd if");
    			camera4_op_pose_.position.z=camera4_op_pose_.position.z+0.08-0.0140;
    			camera4_op_pose_.position.y=camera4_op_pose_.position.y+0.00005-0.027;    			
    		}


	    	}
	    ROS_INFO_STREAM("Moving to grip  "<<product_type);
	    arm->GoToTarget(camera4_op_pose_);
	    ros::Duration(0.3).sleep();
	    ros::spinOnce();
        if (arm->GetGripperStatus()==true){
            ROS_INFO_STREAM("Product is attached");
            break;}
        else{
        	ros::spinOnce();
        	arm->GoToTarget(temp_pose_1);}
      }
    
    
    //ros::Duration(0.25).sleep();
	n++;
	}
   
init_=0;
//arm1_.SendRobotConveyer();
arm->GoToTarget(temporaryPose);//GOup
ROS_INFO_STREAM("Going up");

if (product_type=="pulley_part"){//waste
	
	ros::Duration(1).sleep();

}


	
arm->SendRobotHome();



if (product_type=="pulley_part"){
                      

		pulleyFlipper(agv_id);

}

ros::spinOnce();
if (arm->GetGripperStatus()==false){
	PickFromConveyer(product_type,agv_id);
}


if (agv_id==2){
	arm->SendRobotHome();
    auto place_pose=temp_pose;
    place_pose.position.x=0.31292;
    place_pose.position.y=-0.5954;
    place_pose.position.z=0.9499;
    place_pose.position.z+=0.1;
    if(product_type == "pulley_part")
    	place_pose.position.z += 0.08;
   	
   	if(agv_id==1){ 
    	home_joint_pose_[1]=1.55;
    	home_joint_pose_[0]-=0.4;
	}
	if(agv_id==2){ 
    	home_joint_pose_[1]=3.1+1.55;
    	home_joint_pose_[0]-=0.4;
	}
    while(1){         
	arm->SendRobotJointPosition(home_joint_pose_);ros::spinOnce();
	ROS_INFO_STREAM("Diff is "<<arm->GetJointStates()[0]-home_joint_pose_[0]);
	if (arm->GetJointStates()[0]-home_joint_pose_[0]<0.1 )
		break;
	}
	 
	home_joint_pose_[1]=3.1;
	home_joint_pose_[0]+=0.4;
	

	arm->GoToTarget(place_pose);
	while(1){
		ROS_INFO_STREAM("Send arm1 to drop part ");
		arm->GoToTarget(place_pose);
		ros::spinOnce();
		if (abs(arm->GetEEPose().position.x-0.31292)<0.05){
			ROS_INFO_STREAM("State achieved");
			break;
		}
	}
	if (arm->GetGripperStatus()==false){
	PickFromConveyer(product_type,agv_id);
}

	arm->GripperToggle(false);
	arm->SendRobotHome();
	while(1){
		ROS_INFO_STREAM("Send arm2 home");
		arm_dash->SendRobotHome();
		ros::spinOnce();
		if (arm_dash->GetJointStates()[0]-home_joint_pose_[0]<0.05)
			break;
	}
	//home_joint_pose_[1]=3.1+1.5;
	if(agv_id==1){ 
    	home_joint_pose_[1]=3.1+1.55;
    	
	}
	if(agv_id==2){ 
    	home_joint_pose_[1]=1.55;
    	
	}
	while(1){
		ROS_INFO_STREAM("Send arm2 home twisted");
		arm_dash->SendRobotJointPosition(home_joint_pose_);
		ros::spinOnce();
		if (abs(arm_dash->GetJointStates()[1]-(home_joint_pose_[1]))<0.1)
			break;
	}
	home_joint_pose_[1]=3.1;


	place_pose.position.z-=0.1;
	place_pose.position.z+=0.02;
	if(product_type == "pulley_part")
    	place_pose.position.z += 0.08;
	auto up_pose=place_pose;
	up_pose.position.z+=0.2;
	int n=0;
	int pickflag=0;
	while(1){//Pick up part by arm2
		arm_dash->GripperToggle(true);
		arm_dash->GoToTarget({up_pose,place_pose});
		if (arm_dash->GetGripperStatus()==true ){
			ROS_INFO_STREAM("Product attached arm2");
			break;
		}
		place_pose.position.z-=0.005;
		if (n>5){
			PickFromConveyer(product_type,agv_id);pickflag=1;
		}
		if (pickflag==1)
			break;
		n++;
	}


}

}





int AriacOrderManager::PickFromConveyerBefore
(std::vector<std::pair<float,float>> drop_poses,std::string product_type,std::string empty_bin)
{	std::string direction;
	ROS_INFO_STREAM("character is  "<<empty_bin[4]);
	if(empty_bin[4]=='2'  ||  empty_bin[4]=='3'   ||   empty_bin[4]=='4'   ||  empty_bin[4]=='5'||  empty_bin[4]=='6'){
		InitializeMiddleVector(-1.0);   //keep on rhs
		direction="right";
		ROS_WARN("Vector on RHS");
	}
	
	else{
		InitializeMiddleVector(-1.0);
		direction="left";
		ROS_WARN("vector on LHS");
	}



	int arm_id;
	std::string arm1,arm_dash1;
	arm1={"arm1"};
	arm_dash1={"arm2"};
	arm_id=2;
	RobotController *arm= &arm1_;
	
	RobotController *arm_dash = &arm2_;
	


	double temp=ros::Time::now().toSec();
	ROS_INFO_STREAM("Timer Started...........");
	int n=0;
	for(int i=0;i<2;i++)
		{auto drop_pose_pair=drop_poses[i];
		while(1){
		camera_4_product_=product_type;   //Let camera detect product_type parts
	    //agv_id=2;
		
		//arm_dash->SendRobotHome();

	   ROS_INFO_STREAM("Picking up from conveyer");

	   //std::vector<double> conveyer_joint_pose_ = {-1.1800,0.0,-1.1, 1.9, 3.9, 4.7, 0};
	   arm->SendRobotConveyer();
	 
	   auto temp_pose=camera4_op_pose_;
	   temp_pose.position.z=temp_pose.position.z-0.1;

	   geometry_msgs::Pose temporaryPose;
	   temporaryPose.position.x = 1.193;
	   temporaryPose.position.y = 0.407;
	   temporaryPose.position.z = 1.175+0.1 ;
	   
	   auto temp_pose_1 = temporaryPose;
	   temp_pose_1.position.z -= (0.120);


	   if (product_type=="pulley_part")
	    {
	     temp_pose_1.position.z += (0.120+0.08-0.04-0.1); 
	    }                                       //Take robot to conveyer position
	   //arm->GoToTarget(temp_pose_1);       
	   int n=0;
	   //double temp=ros::Time::now().toSec();
	   

	   while(1){   //for picking up from pulley
		   	
		   
			if (ros::Time::now().toSec()-temp>60){
	   			ROS_INFO_STREAM("60 secs since parts are being picked up");
	   			goto STOP;
	   			}

		    init_=1;
		    //ros::Duration(0.1).sleep();
		    ros::spinOnce();
		    if (arm->GetGripperStatus()==true){
		            ROS_INFO_STREAM("Product is attached");
		            break;}
		    //auto time1=ros::Time::now();
		    auto up_pose=camera4_op_pose_;
		    up_pose.position.z=up_pose.position.z+0.1;
		    up_pose.position.y=up_pose.position.y+0.10;
		    if (flag_==1){
			    ROS_INFO_STREAM("Moving down...");
			    camera4_op_pose_.position.y=camera4_op_pose_.position.y-0.25;
			    arm->GripperToggle(true);
			    camera4_op_pose_.position.z=camera4_op_pose_.position.z+0.02;
			    if (product_type=="pulley_part"){
		    		if ( camera4_op_pose_.position.y<0.425){
		    			ROS_INFO_STREAM("1st if");
		    			camera4_op_pose_.position.z=camera4_op_pose_.position.z+0.08-0.0110-0.004;
		    			camera4_op_pose_.position.y=camera4_op_pose_.position.y+0.00005-0.03;    			
		    		}
			    	if ( camera4_op_pose_.position.y>0.425){
			    		ROS_INFO_STREAM("2nd if");
		    			camera4_op_pose_.position.z=camera4_op_pose_.position.z+0.08-0.0140-0.004;
		    			camera4_op_pose_.position.y=camera4_op_pose_.position.y+0.00005-0.027;    			
		    		}


			    	}

		    	if(product_type=="disk_part"){
		    		camera4_op_pose_.position.z+=0.015;
		    	}
			    ROS_INFO_STREAM("Moving to grip  "<<product_type);
			    arm->GoToTarget(camera4_op_pose_);
			    //ros::Duration(0.3).sleep();
			    ros::spinOnce();
		        if (arm->GetGripperStatus()==true){
		            ROS_INFO_STREAM("Product is attached");
		            break;}
		        else{
		        	ros::spinOnce();
		        	arm->GoToTarget(temp_pose_1);}
		      }
		    
		    
		    //ros::Duration(0.25).sleep();
			n++;
		}
	   
	init_=0;
	//arm1_.SendRobotConveyer();
	while(1){
		arm->GoToTarget(temporaryPose);//GOup
		if(abs(arm->GetEEPose().position.z- temporaryPose.position.z)<0.01){
			ROS_INFO_STREAM("Upstate achieved");
			break;
		}
	}
	
	ROS_INFO_STREAM("Going up");

	if (product_type=="pulley_part"){//waste
		
		//ros::Duration(1).sleep();

	}



	ros::spinOnce();
	if (arm->GetGripperStatus()==false){
		ROS_INFO_STREAM("Product fell down somewhere");
		continue;
	}

		
	//arm->SendRobotHome();


	ros::spinOnce();
	if (arm->GetGripperStatus()==false){
		ROS_INFO_STREAM("Product fell down somewhere");
		continue;
	}

	if(arm_id==1){ 
	    	home_joint_pose_[1]=1.55;
	    	//home_joint_pose_[0]-=0.4;
		}
	if(arm_id==2){ 
    	home_joint_pose_[1]=3.1+1.55;
    	//home_joint_pose_[0]-=0.4;
	}
    while(1){         
	arm->SendRobotJointPosition(home_joint_pose_);ros::spinOnce();
	ROS_INFO_STREAM("Diff is "<<arm->GetJointStates()[0]-home_joint_pose_[0]);
	if (arm->GetJointStates()[0]-home_joint_pose_[0]<0.1 )
		break;
	}
	 
	home_joint_pose_[1]=3.1;
	//home_joint_pose_[0]+=0.4;


	ros::spinOnce();
	if (arm->GetGripperStatus()==false){
		ROS_INFO_STREAM("Product fell down somewhere");
		continue;
	}

	while(1){
		arm->GoToTarget(middle_poses_vector_[i]);
		if (abs(arm->GetEEPose().position.x- middle_poses_vector_[i].position.x)<0.01){
			ROS_INFO_STREAM("State Achieved");
			break;

	}
		}

	arm->GripperToggle(false);

	break;
	}

 n++;}
 
 STOP:
 ROS_WARN_STREAM("These many products were placed successfully"<<n);

for(int i=0;i<n;i++){
	ROS_INFO_STREAM("GOing to keep part from ladder");
	geometry_msgs::Pose drop_pose;
	geometry_msgs::PoseStamped StampedPose_in,StampedPose_out;  
	drop_pose.position.x=drop_poses[i].first;
	drop_pose.position.y=drop_poses[i].second;


	drop_pose.orientation.w=1;

	StampedPose_in.header.frame_id = empty_bin;
	StampedPose_in.pose = drop_pose;
	//ROS_INFO_STREAM("agv1StampedPose_int (" << StampedPose_in.pose.position.x <<","<< StampedPose_in.pose.position.y << "," << StampedPose_in.pose.position.z<<")");
	part_tf_listener_.transformPose("/world",StampedPose_in,StampedPose_out);
	StampedPose_out.pose.position.z += 0.25;
	//StampedPose_out.pose.position.y -= 0.2;
	drop_pose=StampedPose_out.pose;
	drop_pose.orientation.w=1;
	if(empty_bin[4]=='3'   ||   empty_bin[4]=='4'   ||  empty_bin[4]=='5'||  empty_bin[4]=='6'){
		ROS_INFO_STREAM("Arm1 will pick and place parts");
		if(arm_id==1){ 
	    	home_joint_pose_[1]=1.55;
	    	//home_joint_pose_[0]-=0.4;
		}
		if(arm_id==2){ 
	    	home_joint_pose_[1]=3.1+1.55;
	    	//home_joint_pose_[0]-=0.4;
		}
	    while(1){         
		arm->SendRobotJointPosition(home_joint_pose_);ros::spinOnce();
		ROS_INFO_STREAM("Diff is "<<arm->GetJointStates()[0]-home_joint_pose_[0]);
		if (arm->GetJointStates()[0]-home_joint_pose_[0]<0.1 )
			break;
		}
		 
		home_joint_pose_[1]=3.1;
		//home_joint_pose_[0]+=0.4;
		middle_poses_vector_[i].position.z-=0.2;
		if(product_type=="pulley_part")
			middle_poses_vector_[i].position.z+=0.06;

		else if(product_type=="disk_part")
			middle_poses_vector_[i].position.z+=0.02;

		else
			middle_poses_vector_[i].position.z+=0.01;
		middle_poses_vector_[i].orientation.w=1;
		ROS_INFO_STREAM("pose ise   "<<middle_poses_vector_[i]);
		arm->PickPart(middle_poses_vector_[i]);
		arm->SendRobotHome();
		//drop_pose.position.z+=0.2;
		while(1){
			arm->GoToTarget(drop_pose);
			if(abs(arm->GetEEPose().position.x-drop_pose.position.x)<0.01){
				ROS_INFO_STREAM("State Achieved");
				break;
			}
		}

		arm->GripperToggle(false);
		}
	else{
		ROS_INFO_STREAM("Arm2 will pick and place parts");
		arm->SendRobotJointPosition(end_position_);
		if(arm_id==2){ 
	    	home_joint_pose_[1]=1.55;
	    	home_joint_pose_[0]=-1.18;
		}
		if(arm_id==1){ 
	    	home_joint_pose_[1]=3.1+1.55;
	    	home_joint_pose_[0]-=1.18;
		}
	    while(1){         
		arm_dash->SendRobotJointPosition(home_joint_pose_);ros::spinOnce();
		ROS_INFO_STREAM("Diff is "<<arm_dash->GetJointStates()[0]-home_joint_pose_[0]);
		if (arm_dash->GetJointStates()[0]-home_joint_pose_[0]<0.1 )
			break;
		}
		 
		home_joint_pose_[1]=3.1;
		home_joint_pose_[0]=0;
		middle_poses_vector_[i].position.z-=0.2;
		if(product_type=="pulley_part")
			middle_poses_vector_[i].position.z+=0.06;

		else if(product_type=="disk_part")
			middle_poses_vector_[i].position.z+=0.02;

		else
			middle_poses_vector_[i].position.z+=0.01;
		middle_poses_vector_[i].orientation.w=1;
		ROS_INFO_STREAM("pose ise   "<<middle_poses_vector_[i]);
		//arm->PickPart(middle_poses_vector_[i]);
		//arm->SendRobotHome();
		//home_joint_pose_[0]+=0.4;
		arm_dash->PickPart(middle_poses_vector_[i]);
		arm_dash->SendRobotHome();
		while(1){
			arm_dash->GoToTarget(drop_pose);
			if(abs(arm_dash->GetEEPose().position.x-drop_pose.position.x)<0.01){
				ROS_INFO_STREAM("State Achieved");
				break;
			}
		}

		arm_dash->GripperToggle(false);
	}
	

	}

	
}

// 	geometry_msgs::Pose drop_pose;
// 	geometry_msgs::PoseStamped StampedPose_in,StampedPose_out;  
// 	drop_pose.position.x=drop_pose_pair.first;
// 	drop_pose.position.y=drop_pose_pair.second;

// 	drop_pose.orientation.w=1;

// 	StampedPose_in.header.frame_id = empty_bin;
// 	StampedPose_in.pose = drop_pose;
// 	//ROS_INFO_STREAM("agv1StampedPose_int (" << StampedPose_in.pose.position.x <<","<< StampedPose_in.pose.position.y << "," << StampedPose_in.pose.position.z<<")");
// 	part_tf_listener_.transformPose("/world",StampedPose_in,StampedPose_out);
// 	StampedPose_out.pose.position.z += 0.25;
// 	//StampedPose_out.pose.position.y -= 0.2;
// 	//ROS_INFO_STREAM("agv1StampedPose_out (" << StampedPose_out.pose.position.x <<","<< StampedPose_out.pose.position.y << "," << StampedPose_out.pose.position.z<<")");

	
// 	if (empty_bin=="/bin1_frame"){
// 		ROS_INFO_STREAM("Part has to be kept in bin 1 frame");
// 		auto flag=GiveArm2(product_type,StampedPose_out.pose);
// 		if (flag==-1)
// 			continue;
// 	}
// 	else{
// 		while(1){
// 			ROS_INFO_STREAM("Normal Keeping activated");
// 			arm->GoToTarget(StampedPose_out.pose);
// 			if ( abs( arm->GetEEPose().position.x- StampedPose_out.pose.position.x)<0.01){
// 				ROS_INFO_STREAM("Reached bin keeping pose");
// 				break;
// 			}
// 			ros::spinOnce();
// 			if (arm->GetGripperStatus()==false){
// 				continue;
// 			}
// 		}
// 	}

// 	ROS_INFO_STREAM("Dropping part on empty bin");
// 	arm->GripperToggle(false);



// 	break;


// 	}

// }
// return 0;
// }




















//will send to bin5
//flip and pick up 
//ends with arm near bin5
int AriacOrderManager::pulleyFlipper(int agv_id){
	int arm_id;
	std::string arm1,arm_dash1;

	RobotController *arm;
	RobotController *arm_dash;


	if (agv_id==1){
		arm1={"arm1"};
		arm_dash1={"arm2"};
		arm_id=2;
		arm= &arm1_;
		arm_dash= &arm2_;

	}

	if (agv_id==2){
		arm1={"arm2"};
		arm_dash1={"arm1"};
		arm_id=2;
		arm= &arm2_;
		arm_dash= &arm1_;
	}
	
	geometry_msgs::Pose drop_pose;
    drop_pose.position.x=0.31292;
    drop_pose.position.y=0.057890;
    drop_pose.position.z=1.717943;
 	drop_pose.position.z -= 0.75;

	std::vector<double> flipping_joint_pose_arm1 = {-0.50, 3.79, -2.00, 2.14, 3.01, 5.71, 0.0};
	std::vector<double> flipping_joint_pose_arm1_temp = {0.5, 3.79, -2.00, 2.14, 3.01, 5.68, 0.0};
	std::vector<double> flipping_joint_pose_arm2 = {0.42, 1.54, -2.00, 2.14, 3.01, 4.74, 0.0};
	std::vector<double> flipping_joint_pose_arm2_temp = {0.0, 1.54, -2.00, 2.14, 3.01, 4.74, 0.0};
	std::vector<double> flipping_drop_pose_arm1 = {0.0, 3.79, -2.00, 2.14, 4.58, 5.71, 0.0};
	std::vector<double> flipping_drop_pose_arm2 = {0.0, 1.54, -2.0, 2.14, 4.58, -1.54, 0.0};

	ROS_INFO_STREAM("Send arm to  flipping position");
	if (agv_id == 1) arm->SendRobotJointPosition(flipping_joint_pose_arm1);
	else arm->SendRobotJointPosition(flipping_joint_pose_arm2);

	ROS_INFO_STREAM("Send arm_dash to  flipping position");
    arm_dash->GripperToggle(true);
    int n=0;
    if (agv_id == 1) {
    	while(1) {
	    	arm_dash->SendRobotJointPosition(flipping_joint_pose_arm2);
	    	flipping_joint_pose_arm2[0] -= 0.0007;
	    	ros::spinOnce();
	    	if (arm_dash->GetGripperStatus() == true) {
	    		break;
	    	}
	    n++;
	    if(n>15){
	    	flipping_joint_pose_arm2[0] += 0.005;	
	    }
	    
	    }	
    }
    else {
    	while(1) {
    	arm_dash->SendRobotJointPosition(flipping_joint_pose_arm1);
    	flipping_joint_pose_arm1[0] += 0.0007;
    	ros::spinOnce();
    	if (arm_dash->GetGripperStatus() == true) {
    		break;
    	}
    n++;
    if(n>15){
	    	flipping_joint_pose_arm1[0] -= 0.005;	
	    }
    }
    }

    ROS_INFO_STREAM("Sending arm to intermediate position. ");
    arm->GripperToggle(false);
    if (agv_id==1) arm->SendRobotJointPosition(flipping_joint_pose_arm1_temp);
    else arm->SendRobotJointPosition(flipping_joint_pose_arm2_temp);


    auto tempPose1 = drop_pose;
    tempPose1.position.z += 0.15;
    ROS_INFO_STREAM("Dropping pulley");
    if (agv_id==1) {
    	arm_dash->GoToTarget(tempPose1);
    	arm_dash->GripperToggle(false);
    	ROS_INFO_STREAM("Send arm_dash home");
    	arm_dash->SendRobotHome();
	}

	else {
		flipping_drop_pose_arm1[1]=3.1+1.55;
		while(1){         
			arm_dash->SendRobotJointPosition(flipping_drop_pose_arm1);ros::spinOnce();
			ROS_INFO_STREAM("Diff is "<<arm_dash->GetJointStates()[0]-flipping_drop_pose_arm1[0]);
			if (arm_dash->GetJointStates()[0]-flipping_drop_pose_arm1[0]<0.1 )
			break;
		}
		flipping_drop_pose_arm1[1]=3.1;
		auto temp_arm1_pose = arm_dash->GetEEPose();
   		arm_dash->GoToTarget(temp_arm1_pose);
   		arm_dash->GoToTarget(tempPose1);
    	arm_dash->GripperToggle(false);
    	ROS_INFO_STREAM("Send arm2 home");
    	arm_dash->SendRobotHome();
	}
    
  
    ROS_INFO_STREAM("Picking up pulley");   
	if(agv_id==1){ 
    	flipping_drop_pose_arm1[1]=3.1+1.55;
    	while(1){         
		arm->SendRobotJointPosition(flipping_drop_pose_arm1);ros::spinOnce();
		ROS_INFO_STREAM("Diff is "<<arm->GetJointStates()[0]-flipping_drop_pose_arm1[0]);
		if (arm->GetJointStates()[0]-flipping_drop_pose_arm1[0]<0.1 )
			break;
		}
		 
		flipping_drop_pose_arm1[1]=3.1;

	    auto temp_arm1_pose = arm->GetEEPose();
	    arm->GoToTarget(temp_arm1_pose);
	    auto pick_pose = drop_pose;
	    pick_pose.position.z += 0.047 ;
	    arm->GripperToggle(true);
	    arm->PickPart(pick_pose);

	    ROS_INFO_STREAM("Send arm home");
	    while(1) {
	    arm->SendRobotHome();
	    if (arm->GetJointStates()[0]-home_joint_pose_[0]<0.1 )
			break;
		}
	}
    else {
    	auto pick_pose = drop_pose;
	    pick_pose.position.z += 0.047 ;
	    arm->GripperToggle(true);
	    arm->PickPart(pick_pose);

	    ROS_INFO_STREAM("Send  home");
	    while(1) {
	    arm->SendRobotHome();
	    if (arm->GetJointStates()[0]-home_joint_pose_[0]<0.1 )
			break;
		}
    }
	
	return 0;
}





void AriacOrderManager::CompletethisOrder(osrf_gear::Order order){
	ROS_INFO_STREAM("Complete this order");
	//arm1_.SendRobotHome();
	//arm2_.SendRobotHome();
	auto shipments = order.shipments;
	for (const auto &shipment: shipments){
	 auto shipment_type = shipment.shipment_type;
	 current_shipment_=shipment;
	 UpdateShipmentVector(current_shipment_);
	 int agv_id=0;
	  for (const auto &product: shipment.products){
	    ROS_INFO_STREAM("Product to place is  "    <<product.type);
	    //arm2_.SendRobotJointPosition(end_position_);
	    std::pair<std::string,geometry_msgs::Pose> product_type_pose;
	    product_type_pose.first=product.type;
	    product_type_pose.second=product.pose;
	    //BigPickAndPlace(product_type_pose,shipment.agv_id);
	    agv_id=0;
	    if (shipment.agv_id=="any"   ||shipment.agv_id=="agv1"){
	    	agv_id=1;
	    }
	    else{
	    	agv_id=2;
	    }
	    
	    
	    //ROS_INFO_STREAM("Size of product frame list    "<<product_frame_list_.size());
	    if (product_frame_list_.find(product.type)!=product_frame_list_.end()){  //Found in list
		    if (shipment.agv_id=="any"   ||shipment.agv_id=="agv1"){
		      PickAndPlace(product_type_pose, 1 ,0);
		  	}
		  	else{
	  		  PickAndPlace(product_type_pose,2 ,0);
		  	}
	  	}
	  	else
	  		{PickAndPlace(product_type_pose,agv_id,1);
	  		}
  		RemovefromVector(product);
  		ros::spinOnce();   //for detecting blacout
	  	
	 //    else{//conveyer part
	 //      ROS_INFO_STREAM("Please pick up from conveyer"    <<product.type);
	 //      int agv_id=0;
	 //      if (shipment.agv_id=="any"   ||shipment.agv_id=="agv1"){
	 //      	agv_id=1;
	 //      }
	 //      else{
	 //      	agv_id=2;
	 //      }
	 //      //agv_id=2;
	 //      PickFromConveyer(product_type_pose.first,agv_id);
	 //      bool result,result1;
	 //      geometry_msgs::Pose drop_pose = product_type_pose.second;

	 //      geometry_msgs::PoseStamped StampedPose_in,StampedPose_out;

	 //     //shipment.agv_id="agv2" ;
	 //    if(agv_id==1){
	 //        StampedPose_in.header.frame_id = "/kit_tray_1";
	 //        StampedPose_in.pose = drop_pose;
	 //        ROS_INFO_STREAM("agv1StampedPose_int (" << StampedPose_in.pose.position.x <<","<< StampedPose_in.pose.position.y << "," << StampedPose_in.pose.position.z<<")");
	 //        part_tf_listener_.transformPose("/world",StampedPose_in,StampedPose_out);
	 //        StampedPose_out.pose.position.z += 0.1;
	 //        StampedPose_out.pose.position.y -= 0.2;
	 //        ROS_INFO_STREAM("agv1StampedPose_out (" << StampedPose_out.pose.position.x <<","<< StampedPose_out.pose.position.y << "," << StampedPose_out.pose.position.z<<")");

	 //    }
	 //    else{
	 //        StampedPose_in.header.frame_id = "/kit_tray_2";
	 //        StampedPose_in.pose = drop_pose;
	 //        //ROS_INFO_STREAM("StampedPose_in " << StampedPose_in.pose.position.x);
	 //        part_tf_listener_.transformPose("/world",StampedPose_in,StampedPose_out);
	 //        StampedPose_out.pose.position.z += 0.1;
	 //        StampedPose_out.pose.position.y += 0.2;
	 //        //ROS_INFO_STREAM("StampedPose_out " << StampedPose_out.pose.position.x);
	 //    }
	 //    int arm_id;
		//   std::string arm1,arm_dash1;

		//   if (agv_id==1){
		// 		arm1={"arm1"};
		// 		arm_dash1={"arm2"};
		// 		arm_id=2;

		// 	}

		//   if (agv_id==2){
		// 		arm1={"arm2"};
		// 		arm_dash1={"arm1"};
		// 		arm_id=2;
		// 		//arm1_.SendRobotJointPosition(end_position_);
		// 	}


		// RobotController arm(arm1);
		// arm->SendRobotHome();
		// RobotController arm_dash(arm_dash1);
		// arm_dash->SendRobotHome();
	      

	 //      ros::Duration(0.5).sleep();
	 //      while(1) {//Conveyer part

	 //          ros::spinOnce();
	 //          if (arm->GetGripperStatus()==false){
	 //          	//arm1_.SendRobotConveyer();
	 //          	PickFromConveyer(product_type_pose.first,agv_id);
	 //          	continue;
	 //          	}

	 //          result = arm->DropPart(StampedPose_out.pose);
	 //          ROS_INFO_STREAM("result from DropPart: "<< result);

		//     // bool result1;
		//     if (result==-1){
		//       ROS_INFO_STREAM("Retrying due to defected or dropped part");
		//       arm->SendRobotConveyer();
		//       arm->GripperToggle(false);
		//       continue;
		//       }
		//     if (result==0) {
		//   	  ROS_INFO_STREAM("Breaking out......");
		//   	  break;
		//   		}

		// 	}
		// }//conveyer part

	

	if (order_update_flag_==1){
		ROS_WARN("Order being updated.....");
		return;
		}
	}//product
  // if(camera_.sensorBlackout()) {
  // 	while(1) {
  // 		if (camera_.sensorBlackout() == false) break;
  // 	}
  // }
  CheckOrder(shipment);
  SubmitAGV(agv_id);}//shipment
}//func





void AriacOrderManager::RemovefromVector(osrf_gear::Product product){
	//current_shipment_vector_
	std::pair<std::string, geometry_msgs::Pose> temp;
	temp.first=product.type;
	temp.second=product.pose;
	for(int i=0;i<current_shipment_vector_.size();i++){
		if(temp.first== current_shipment_vector_[i].first     &&     euler_distance(temp.second,current_shipment_vector_[i].second)<0.01){
			current_shipment_vector_.erase(current_shipment_vector_.begin()+i);
			ROS_INFO_STREAM("Part removed from shipment Vector        "<<temp.first);
			return; 		
		}
	}

}





std::vector <std::pair<std::string,std::vector<geometry_msgs::Pose> > > AriacOrderManager::UpdateLists(osrf_gear::Shipment shipment){
	ROS_INFO_STREAM("Updating Lists");
	int agv_id=0;
	auto agv_id_string=shipment.agv_id;
	
	//agv_id_string="agv2";
	std::vector <std::pair<std::string,geometry_msgs::Pose> > cam5_list_right;
	if (agv_id_string=="any"    ||    agv_id_string=="agv1"){
		init_cam5_=1;
		ros::Duration(0.5).sleep();
		ros::spinOnce();     //to update part list of kit_tray i.e. cam5_list_	
		init_cam5_=0;
		cam5_list_right=cam5_list_;
		cam5_list_right=arm1_.GetKitTrayList();
		agv_id=1;
	}

	else{
		init_cam5_dash_=1;
		ros::Duration(0.5).sleep();
		ros::spinOnce();     //to update part list of kit_tray 2 i.e. cam5_list_dash	
		init_cam5_dash_=0;
		cam5_list_right=cam5_list_dash_;
		cam5_list_right=arm2_.GetKitTrayList();
		agv_id=2;
	}

	//agv_id=1;
	//cam5_list_right=
	std::vector <std::pair<std::string,geometry_msgs::Pose> > order_list;
	//auto shipments = order.shipments;
	//for (const auto &shipment: shipments){
	auto shipment_type = shipment.shipment_type;
	  for (const auto &product: shipment.products){
	  	std::pair<std::string,geometry_msgs::Pose> temp;
	  	temp.first=product.type;
	  	temp.second=product.pose;
	  	order_list.push_back(temp);
	  }
	std::vector <std::pair<std::string,std::vector<geometry_msgs::Pose> > > repos_list;
	std::vector <std::pair<std::string,geometry_msgs::Pose> > order_repos_list(order_list);
	std::vector <std::pair<std::string,geometry_msgs::Pose> > cam5_repos_list(cam5_list_right);
	std::vector <std::pair<std::string,geometry_msgs::Pose> > diff_list;
	std::vector <std::pair<std::string,geometry_msgs::Pose> > get_list;
	ROS_INFO_STREAM("Camera 5 can see these many components Updated List  "<<cam5_list_right.size());
	for(int i=0;i<cam5_list_right.size();i++)
		{ROS_INFO_STREAM("  " <<cam5_list_right[i].first);}

	for(int i=0;i<order_repos_list.size();i++){
		for (int j=0;j<cam5_repos_list.size();j++){
			if (cam5_repos_list[j].first==order_repos_list[i].first){
				std::vector<geometry_msgs::Pose> poses;
				poses.push_back(cam5_repos_list[j].second);
				poses.push_back(order_repos_list[i].second);
				std::pair<std::string,std::vector<geometry_msgs::Pose>> temp;
				temp.first=cam5_repos_list[j].first;
				temp.second=poses;
				repos_list.push_back(temp);
				cam5_repos_list.erase(cam5_repos_list.begin()+j);
				order_repos_list.erase(order_repos_list.begin()+i);
				i--;
				continue;
			}
		}
	}

	diff_list=cam5_repos_list;
	get_list=order_repos_list;
	return repos_list;
//}

}


void AriacOrderManager::UpdateShipmentVector(osrf_gear::Shipment shipment){
	for(int i=0;i<shipment.products.size();i++){
		std::pair<std::string,geometry_msgs::Pose> temp;
		temp.first=shipment.products[i].type;
		temp.second=shipment.products[i].pose;
		current_shipment_vector_.push_back(temp);
	}
}


void AriacOrderManager::CompleteOrderUpdate(osrf_gear::Order order){
	ROS_WARN("Order being Updated");
	//look at components on tray
		//take diff of order components and on tray components
		//make vector of components on tray that are not needed
		//make vector of components on tray that need repositioning
		//make vector of components on tray that are new
	//remove waste components
	//reposition elements present
	//pick up and place new elements needed
	// int agv_id=0;
	// auto agv_id_string=order.shipments[0].agv_id;
	
	// std::vector <std::pair<std::string,geometry_msgs::Pose> > cam5_list_right;
	// if (agv_id_string=="any"    ||    agv_id_string=="agv1"){
	// 	init_cam5_=1;
	// 	ros::Duration(0.5).sleep();
	// 	ros::spinOnce();     //to update part list of kit_tray i.e. cam5_list_	
	// 	init_cam5_=0;
	// 	cam5_list_right=cam5_list_;
	// 	agv_id=1;
	// }

	// else{
	// 	init_cam5_dash_=1;
	// 	ros::Duration(0.5).sleep();
	// 	ros::spinOnce();     //to update part list of kit_tray i.e. cam5_list_	
	// 	init_cam5_dash_=0;
	// 	cam5_list_right=cam5_list_dash_;
	// 	agv_id=2;
	// }

	//agv_id=2;
	if(camera_.sensorBlackout()) {
  	while(1) {
  		if (camera_.sensorBlackout() == false) break;
  	}
   }

	
	auto shipments = order.shipments;
	for (const auto &shipment: shipments){
	//current_shipment_=shipment;
	//UpdateShipmentVector(current_shipment_);
	std::vector <std::pair<std::string,geometry_msgs::Pose> > order_list;
	auto shipment_type = shipment.shipment_type;
	int agv_id=0;
	auto agv_id_string=shipment.agv_id;
	//agv_id_string="agv2";
	
	std::vector <std::pair<std::string,geometry_msgs::Pose> > cam5_list_right;
	if (agv_id_string=="any"    ||    agv_id_string=="agv1"){
		init_cam5_=1;
		ros::Duration(0.1).sleep();
		ros::spinOnce();     //to update part list of kit_tray i.e. cam5_list_	
		init_cam5_=0;
		cam5_list_right=cam5_list_;
		cam5_list_right=arm1_.GetKitTrayList();
		agv_id=1;
	}

	else{
		init_cam5_dash_=1;
		ros::Duration(0.1).sleep();
		ros::spinOnce();     //to update part list of kit_tray i.e. cam5_list_	
		init_cam5_dash_=0;
		cam5_list_right=cam5_list_dash_;
		cam5_list_right=arm2_.GetKitTrayList();
		agv_id=2;
	}
	  for (const auto &product: shipment.products){
	  	std::pair<std::string,geometry_msgs::Pose> temp;
	  	temp.first=product.type;
	  	temp.second=product.pose;
	  	order_list.push_back(temp);
	  }
	std::vector <std::pair<std::string,std::vector<geometry_msgs::Pose> > > repos_list;
	std::vector <std::pair<std::string,geometry_msgs::Pose> > order_repos_list(order_list);
	std::vector <std::pair<std::string,geometry_msgs::Pose> > cam5_repos_list(cam5_list_right);
	std::vector <std::pair<std::string,geometry_msgs::Pose> > diff_list;
	std::vector <std::pair<std::string,geometry_msgs::Pose> > get_list;
	ROS_INFO_STREAM("Camera 5 can see these many components  "<<cam5_list_right.size());
	for(int i=0;i<cam5_list_right.size();i++)
		{ROS_INFO_STREAM("  " <<cam5_list_right[i].first);}

	for(int i=0;i<order_repos_list.size();i++){
		for (int j=0;j<cam5_repos_list.size();j++){
			if (cam5_repos_list[j].first==order_repos_list[i].first){
				std::vector<geometry_msgs::Pose> poses;
				poses.push_back(cam5_repos_list[j].second);
				poses.push_back(order_repos_list[i].second);
				std::pair<std::string,std::vector<geometry_msgs::Pose>> temp;
				temp.first=cam5_repos_list[j].first;
				temp.second=poses;
				repos_list.push_back(temp);
				cam5_repos_list.erase(cam5_repos_list.begin()+j);
				order_repos_list.erase(order_repos_list.begin()+i);
				i--;
				continue;
			}
		}
	}

	diff_list=cam5_repos_list;
	get_list=order_repos_list;

	if(camera_.sensorBlackout()) {
  	while(1) {
  		if (camera_.sensorBlackout() == false) break;
  		}
    }
	
	for(int i=0;i<diff_list.size();i++){
		ROS_INFO_STREAM("Throwing    "<<diff_list[i].first);
		ThrowComponents(diff_list[i],agv_id);
	}

	for(int j=0;j<repos_list.size();j++){
		int r=-1;
		while(r==-1){
			ROS_INFO_STREAM("Reposition    "<<repos_list[j].first);
			r=RepositionComponents(repos_list[j],agv_id);
			if(r==-1)
				repos_list=UpdateLists(shipment);
		}
	}

	for(int k=0;k<get_list.size();k++){
		if (product_frame_list_.find(get_list[k].first)!=product_frame_list_.end()){  //Found in list
	      ROS_INFO_STREAM("Get    "<<get_list[k].first);
	      PickAndPlace(get_list[k], agv_id,0);     //add conveyer type code
	  	}
	  	else
	  		PickAndPlace(get_list[k], agv_id,1);

	}
	SubmitAGV(agv_id);
}

}













void AriacOrderManager::ThrowComponents(std::pair<std::string,geometry_msgs::Pose>diff_list,int agv_id){
	

	int arm_id;
	std::string arm1,arm_dash1;

	RobotController *arm;
	RobotController *arm_dash;


	if (agv_id==1){
		arm1={"arm1"};
		arm_dash1={"arm2"};
		arm_id=2;
		arm= &arm1_;
		arm_dash= &arm2_;

	}

	if (agv_id==2){
		arm1={"arm2"};
		arm_dash1={"arm1"};
		arm_id=2;
		arm= &arm2_;
		arm_dash= &arm1_;
	}
	geometry_msgs::Pose part_pose;
	
	ros::spinOnce();
	ROS_INFO_STREAM("ThrowComponents");
	ros::spinOnce();
	while(1){         //so that we know the arm is at agv1
		arm->SendRobotJointPosition(end_position_);
		ros::spinOnce();
		ROS_INFO_STREAM(arm->GetJointStates()[0]-end_position_[0]);
		if (arm->GetJointStates()[0]-end_position_[0]<0.1)
			break;
	}
	geometry_msgs::PoseStamped StampedPose_in,StampedPose_out;     
	if (agv_id==1){
	part_pose=diff_list.second;
	     
    StampedPose_in.header.frame_id = "/logical_camera_8_frame";
    StampedPose_in.pose = part_pose;
    part_tf_listener_.transformPose("/world",StampedPose_in,StampedPose_out);
    part_pose=StampedPose_out.pose;
	}

	if (agv_id==2){
	part_pose=diff_list.second;
	//geometry_msgs::PoseStamped StampedPose_in,StampedPose_out;          
    StampedPose_in.header.frame_id = "/logical_camera_9_frame";
    StampedPose_in.pose = part_pose;
    part_tf_listener_.transformPose("/world",StampedPose_in,StampedPose_out);
    part_pose=StampedPose_out.pose;
	}
	

	//if (diff_list.first=="pulley_part")
		//part_pose.position.z+=0.08;
	

	part_pose.position.z+=0.02;
	auto up_pose=part_pose;
	up_pose.position.z+=0.2;
	int n=0;
	arm->GripperToggle(true);
	ROS_INFO_STREAM("GOing to pick   "<<diff_list.first);
	

	while(1){
		if (agv_id==1)
		{init_cam5_2_=1;
		reposition_part_type_=diff_list.first;
		reposition_part_pose_=part_pose;
		ros::Duration(0.25).sleep();
		ros::spinOnce();
		reposition_part_pose_back_.position.z+=0.02;
		//if (diff_list.first=="pulley_part")
			//reposition_part_pose_back_.position.z+=0.06;

		}

		if (agv_id==2)
		{init_cam5_2_dash_=1;
		reposition_part_type_dash_=diff_list.first;
		reposition_part_pose_dash_=part_pose;
		ros::Duration(0.25).sleep();
		ros::spinOnce();
		reposition_part_pose_back_dash_.position.z+=0.02;
		//if (diff_list.first=="pulley_part")
			//reposition_part_pose_back_dash_.position.z+=0.06;
	}
		
		if(agv_id==1){
		for (int i=0;i<n;i++)
			reposition_part_pose_back_.position.z-=0.005;}


		if(agv_id==2){
		for (int i=0;i<n;i++)
			reposition_part_pose_back_dash_.position.z-=0.005;}


		ros::spinOnce();
		if(agv_id==1){
		if (repose_flag_==1){
			arm->GoToTarget({up_pose,reposition_part_pose_back_});
			part_pose=reposition_part_pose_back_;
		}
		else{
			ROS_INFO_STREAM("Reposition flag unsuccessfull");
			arm->GoToTarget({up_pose,part_pose});
			part_pose.position.z-=0.005;

		}
		}


		if(agv_id==2){
		if (repose_flag_dash_==1){
			arm->GoToTarget({up_pose,reposition_part_pose_back_dash_});
			part_pose=reposition_part_pose_back_dash_;
		}
		else{
			ROS_INFO_STREAM("Reposition flag unsuccessfull");
			arm->GoToTarget({up_pose,part_pose});
			part_pose.position.z-=0.005;

		}
		}



		ros::Duration(0.1).sleep();
		arm->GoToTarget(up_pose);
		//other_part_pose.position.z-=0.005;
		
		if (arm->GetGripperStatus()==true){
			ROS_INFO_STREAM("Product attached");
			break;
		}

		n++;
	}
	init_cam5_2_=0;
	init_cam5_2_dash_=0;

	ros::spinOnce();
	while(1){         //so that we know the arm is at agv1 throwing pos
		arm->SendRobotJointPosition(end_position_);
		ros::spinOnce();
		ROS_INFO_STREAM(joint_states_[0]-2.2);
		if (arm->GetJointStates()[0]-2.2<0.1)
			break;
	}
	arm->GripperToggle(false);
	ROS_INFO_STREAM("Part dropped");

}

float AriacOrderManager::euler_distance(geometry_msgs::Pose p1,geometry_msgs::Pose p2){
	float x1=p1.position.x;
	float y1=p1.position.y;
	float z1=p1.position.z;
	float x2=p2.position.x;
	float y2=p2.position.y;
	float z2=p2.position.z;

	float dist=((x2-x1)*(x2-x1))+((y2-y1)*(y2-y1));
	return dist;



}




std::pair<std::string,geometry_msgs::Pose> AriacOrderManager:: PartThere(geometry_msgs::Pose drop_pose,int agv_id){
	
	if (agv_id==1){
	init_cam5_=1;
	ros::Duration(0.5).sleep();
	ros::spinOnce();
	init_cam5_=0;
	//cam5_list_ updated
	std::pair<std::string,geometry_msgs::Pose> temp;
	temp.first="None";
	auto kit_tray_list_=arm1_.GetKitTrayList();
	for(int i=0;i<kit_tray_list_.size();i++){
		//ROS_INFO_STREAM("Checking for  "<<cam5_list_[i].first);
		auto part_pose=kit_tray_list_[i].second;
		geometry_msgs::PoseStamped StampedPose_in,StampedPose_out;          
	    StampedPose_in.header.frame_id = "/logical_camera_8_frame";
	    StampedPose_in.pose = part_pose;
	    part_tf_listener_.transformPose("/world",StampedPose_in,StampedPose_out);
	    part_pose=StampedPose_out.pose;  //in world frame now

	    if( euler_distance(part_pose,drop_pose)<0.005){
	    	ROS_INFO_STREAM("Checking for  "<<kit_tray_list_[i].first);
	    	ROS_INFO_STREAM("There is this part where we have to keep part");
	    	temp.first=kit_tray_list_[i].first;
	    	temp.second=part_pose;
	    	return temp;
	    }

	}
return temp;}

	else{
		init_cam5_dash_=1;
		ros::Duration(0.5).sleep();
		ros::spinOnce();
		init_cam5_dash_=0;
		//cam5_list_ updated
		std::pair<std::string,geometry_msgs::Pose> temp;
		temp.first="None";
		auto kit_tray_list_=arm2_.GetKitTrayList();
		for(int i=0;i<kit_tray_list_.size();i++){
			//ROS_INFO_STREAM("Checking for  "<<cam5_list_[i].first);
			auto part_pose=kit_tray_list_[i].second;
			geometry_msgs::PoseStamped StampedPose_in,StampedPose_out;          
		    StampedPose_in.header.frame_id = "/logical_camera_9_frame";
		    StampedPose_in.pose = part_pose;
		    part_tf_listener_.transformPose("/world",StampedPose_in,StampedPose_out);
		    part_pose=StampedPose_out.pose;  //in world frame now

		    if( euler_distance(part_pose,drop_pose)<0.005){
		    	ROS_INFO_STREAM("Checking for  "<<kit_tray_list_[i].first);
		    	ROS_INFO_STREAM("There is this part where we have to keep part");
		    	temp.first=kit_tray_list_[i].first;
		    	temp.second=part_pose;
		    	return temp;
		    	}

			}
		return temp;
	}

	}


int AriacOrderManager::RepositionComponents(std::pair<std::string,std::vector<geometry_msgs::Pose> >diff_list,int agv_id){
	ROS_INFO_STREAM("Reposition Components");
	
	int arm_id;
	std::string arm1,arm_dash1;

	RobotController *arm;
	RobotController *arm_dash;


	if (agv_id==1){
		arm1={"arm1"};
		arm_dash1={"arm2"};
		arm_id=2;
		arm= &arm1_;
		arm_dash= &arm2_;

	}

	if (agv_id==2){
		arm1={"arm2"};
		arm_dash1={"arm1"};
		arm_id=2;
		arm= &arm2_;
		arm_dash= &arm1_;
	}
	
	ros::spinOnce();
	
	if (agv_id==1)
		end_position_[1]=1.55;
	else{
		end_position_[1]=3.1+1.5;
		end_position_[0]=-0.5;
	}
	
	while(1){         //so that we know the arm is at agv1
		arm->SendRobotJointPosition(end_position_);
		ros::spinOnce();
		//ROS_INFO_STREAM(joint_states_[0]-2.2);
		if (arm->GetJointStates()[0]-end_position_[0]<0.1)
			break;
	}
	end_position_[1]=3.1;
	end_position_[0]=1.18;
	

	//Transform Poses

	auto part_pose=diff_list.second.at(0);
	

	geometry_msgs::PoseStamped StampedPose_in,StampedPose_out; 
	if (agv_id==1){
		         
	    StampedPose_in.header.frame_id = "/logical_camera_8_frame";
	    StampedPose_in.pose = part_pose;
	    part_tf_listener_.transformPose("/world",StampedPose_in,StampedPose_out);
	    part_pose=StampedPose_out.pose;
	}
    //ROS_INFO_STREAM(part_pose); 
	if (agv_id==2){
		//geometry_msgs::PoseStamped StampedPose_in,StampedPose_out;          
	    StampedPose_in.header.frame_id = "/logical_camera_9_frame";
	    StampedPose_in.pose = part_pose;
	    part_tf_listener_.transformPose("/world",StampedPose_in,StampedPose_out);
	    part_pose=StampedPose_out.pose;
	}


    ros::spinOnce();
	auto drop_pose=diff_list.second.at(1);
    //int agv_id=1;           //change this to have as input
	if(agv_id==1){
        StampedPose_in.header.frame_id = "/kit_tray_1";
        StampedPose_in.pose = drop_pose;
        //ROS_INFO_STREAM("StampedPose_int (" << StampedPose_in.pose.position.x <<","<< StampedPose_in.pose.position.y << "," << StampedPose_in.pose.position.z<<")");
        part_tf_listener_.transformPose("/world",StampedPose_in,StampedPose_out);
        StampedPose_out.pose.position.z += 0.1;
        StampedPose_out.pose.position.y -= 0.2;
        StampedPose_out.pose.position.y+=0.06;
        //ROS_INFO_STREAM("StampedPose_out (" << StampedPose_out.pose.position.x <<","<< StampedPose_out.pose.position.y << "," << StampedPose_out.pose.position.z<<")");

    }
    else{
        StampedPose_in.header.frame_id = "/kit_tray_2";
        StampedPose_in.pose = drop_pose;
        //ROS_INFO_STREAM("StampedPose_in " << StampedPose_in.pose.position.x);
        part_tf_listener_.transformPose("/world",StampedPose_in,StampedPose_out);
        StampedPose_out.pose.position.z += 0.1;
        StampedPose_out.pose.position.y += 0.2;
        StampedPose_out.pose.position.y-=0.06;
        //ROS_INFO_STREAM("StampedPose_out " << StampedPose_out.pose.position.x);
    }
	
	

	//to check wheter part already there
	ROS_INFO_STREAM("Distance between 2 parts is   "<<euler_distance(part_pose,StampedPose_out.pose));
    if (euler_distance(part_pose,StampedPose_out.pose)<0.03){
    	ROS_INFO_STREAM("Reposition not required as part already at right position");
    	ros::Duration(1.0).sleep();
    	return 0;
    }

    

    for(int i=0;i<MultipleRepositionVector_.size();i++){
    	if (diff_list.first==MultipleRepositionVector_[i].first    &&    euler_distance(MultipleRepositionVector_[i].second.at(0), part_pose)<0.005){
    		part_pose=MultipleRepositionVector_[i].second.at(1);
    	}
    }
    geometry_msgs::Pose up_pose;
    //There is some part there
    auto other_part_type_pose=PartThere(StampedPose_out.pose,agv_id);
    auto other_part_pose=other_part_type_pose.second;
    if(other_part_type_pose.first!="None"   ){// &&   other_part_type_pose.first!=diff_list.first   ){
    	//there is a part there or there is not a part with same type
    	ROS_INFO_STREAM("Multiple repositioning Activated....");
    	
    	if(other_part_type_pose.first==diff_list.first){
    		ROS_INFO_STREAM("Both parts are the same   ");
    		return 0;
    	}

    	std::vector<geometry_msgs::Pose> tempposes;
    	tempposes.push_back(other_part_pose);
    	tempposes.push_back(part_pose);
    	std::pair< std::string, std::vector<geometry_msgs::Pose > > temppair;
    	temppair.first=other_part_type_pose.first;
    	temppair.second=tempposes;
    	MultipleRepositionVector_.push_back(temppair);
    	ros::Duration(1.0).sleep();
    	geometry_msgs::Pose temp_pose;
    	temp_pose.position.x=0.488342-0.03;
    	if (agv_id==1)
    		temp_pose.position.y=3.353200;
    	else
    		temp_pose.position.y=-3.353200;
    	temp_pose.position.z=0.755067;
    	temp_pose.position.z+=0.1;
    	temp_pose.orientation=other_part_type_pose.second.orientation;
    	ROS_INFO_STREAM("Poses to check are    "<<StampedPose_out.pose<<other_part_type_pose.second);
    	up_pose=other_part_pose;

    	while(1){
    	auto other_part_pose=other_part_type_pose.second;  //this is the same as stamped_pose_out
    	//other_part_pose.position.z+=0.02;
		up_pose=other_part_pose;
		other_part_pose.position.z+=0.02;
		//if (other_part_type_pose.first=="pulley_part")
			//other_part_pose.position.z+=0.06;

		up_pose.position.z+=0.25;
		int n=0;
		arm->GripperToggle(true);
		ROS_INFO_STREAM("GOing to pick other part   "<<other_part_type_pose.first);

		while(1){
			
			if(agv_id==1){
			init_cam5_2_=1;
			reposition_part_type_=other_part_type_pose.first;
			reposition_part_pose_=other_part_pose;
			ros::Duration(0.25).sleep();
			ros::spinOnce();
			reposition_part_pose_back_.position.z+=0.02;
			//if (other_part_type_pose.first=="pulley_part")
				//reposition_part_pose_back_.position.z+=0.06;
			}

			if(agv_id==2){
			init_cam5_2_dash_=1;
			reposition_part_type_dash_=other_part_type_pose.first;
			reposition_part_pose_dash_=other_part_pose;
			ros::Duration(0.25).sleep();
			ros::spinOnce();
			reposition_part_pose_back_dash_.position.z+=0.02;
			//if (other_part_type_pose.first=="pulley_part")
				//reposition_part_pose_back_dash_.position.z+=0.06;
			}



			
			if(agv_id==1){		
				for (int i=0;i<n;i++)
					reposition_part_pose_back_.position.z-=0.005;
				}

			if(agv_id==2){		
				for (int i=0;i<n;i++)
					reposition_part_pose_back_dash_.position.z-=0.005;
				}

			
			ros::spinOnce();
			if(agv_id==1){
			if (repose_flag_==1){
				arm->GoToTarget({up_pose,reposition_part_pose_back_});
				other_part_pose=reposition_part_pose_back_;
			}
			else{
				ROS_INFO_STREAM("Reposition flag unsuccessfull");
				arm->GoToTarget({up_pose,other_part_pose});
				other_part_pose.position.z-=0.005;

			}
			}

			ros::spinOnce();
			if(agv_id==2){
			if (repose_flag_dash_==1){
				arm->GoToTarget({up_pose,reposition_part_pose_back_dash_});
				other_part_pose=reposition_part_pose_back_dash_;
			}
			else{
				ROS_INFO_STREAM("Reposition flag unsuccessfull");
				arm->GoToTarget({up_pose,other_part_pose});
				other_part_pose.position.z-=0.005;

			}
			}

			ros::Duration(0.1).sleep();
			arm->GoToTarget(up_pose);
			//other_part_pose.position.z-=0.005;
			
			if (arm->GetGripperStatus()==true){
				ROS_INFO_STREAM("Product attached");
				break;
			}

			n++;
		}
		init_cam5_2_=0;
		init_cam5_2_dash_=0;
		//Other part picked up

		init_cam5_=1;
		init_cam5_dash_=1;
		ros::Duration(0.2).sleep();             //Update cam5 list
		ros::spinOnce();
		init_cam5_=0;
		init_cam5_dash_=0;

		cam5_list_=arm->GetKitTrayList();

		while(1){
		    arm->GoToTarget(temp_pose);
		    ros::spinOnce();
		    if (abs(arm->GetEEPose().position.x-temp_pose.position.x)<0.1     &&   abs(arm->GetEEPose().position.y-temp_pose.position.y)<0.1 ){
		    	ROS_INFO_STREAM("State achieved");
		    	break;
		    }
		}

		
		ros::Duration(0.5).sleep();
		ros::spinOnce();               
		
		if(arm->GetGripperStatus()==false){
			ROS_INFO_STREAM("Part was dropped before keeping");
			if (agv_id==1)
				end_position_[1]=1.55;
			else{
				end_position_[1]=3.1+1.5;
				end_position_[0]=-0.5;
			}
			
			while(1){         //so that we know the arm is at agv1
				arm->SendRobotJointPosition(end_position_);
				ros::spinOnce();
				//ROS_INFO_STREAM(joint_states_[0]-2.2);
				if (arm->GetJointStates()[0]-end_position_[0]<0.1)
					break;
			}
			end_position_[1]=3.1;
			end_position_[0]=1.18;
			return -1;
			continue;
		}
		ROS_INFO_STREAM("Other Part being dropped at temp position");
		arm->GripperToggle(false);
		//Other part droppped at temp pose


		


		//arm->GoToTarget(up_pose);
		//ros::Duration(0.25).sleep();
		ros::spinOnce();
		if(agv_id==1){
			end_position_[1]=1.55;
		while(1){         //so that we know the arm is at agv1
			arm->SendRobotJointPosition(end_position_);
			ros::spinOnce();
			//ROS_INFO_STREAM(joint_states_[0]-2.2);
			if (arm->GetJointStates()[0]-end_position_[0]<0.1)
				break;
		}
		end_position_[1]=3.1;
		}

		if(agv_id==2){
		end_position_[1]=3.1+1.55;
		end_position_[0]=-0.5;
		while(1){         //so that we know the arm is at agv1
			arm->SendRobotJointPosition(end_position_);
			ros::spinOnce();
			//ROS_INFO_STREAM(joint_states_[0]-2.2);
			if (arm->GetJointStates()[0]-end_position_[0]<0.1)
				break;
		}
		end_position_[1]=3.1;
		end_position_[0]=1.18;
		}

		break;

	}

		
		while(1){
		//if (diff_list.first=="pulley_part")
			//part_pose.position.z+=0.06;
		
		part_pose.position.z+=0.02;
		up_pose=part_pose;
		up_pose.position.z+=0.25;
		int n=0;
		arm->GripperToggle(true);
		ROS_INFO_STREAM("GOing to pick   "<<diff_list.first);
		while(1){
			if(agv_id==1){
			init_cam5_2_=1;
			reposition_part_type_=diff_list.first;
			reposition_part_pose_=part_pose;
			ros::Duration(0.25).sleep();
			ros::spinOnce();
			reposition_part_pose_back_.position.z+=0.02;
			//if (diff_list.first=="pulley_part")
				//reposition_part_pose_back_.position.z+=0.06;
			}

			if(agv_id==2){
			init_cam5_2_dash_=1;
			reposition_part_type_dash_=diff_list.first;
			reposition_part_pose_dash_=part_pose;
			ros::Duration(0.25).sleep();
			ros::spinOnce();
			reposition_part_pose_back_dash_.position.z+=0.02;
			//if (diff_list.first=="pulley_part")
				//reposition_part_pose_back_dash_.position.z+=0.06;

			}
			

			if(agv_id==1){
			for (int i=0;i<n;i++)
				reposition_part_pose_back_.position.z-=0.005;
			}


			if(agv_id==2){
			for (int i=0;i<n;i++)
				reposition_part_pose_back_dash_.position.z-=0.005;
			}

			ros::spinOnce();
			if(agv_id==1){
			if (repose_flag_==1){
				arm->GoToTarget({up_pose,reposition_part_pose_back_});
				part_pose=reposition_part_pose_back_;
			}
			else{
				ROS_INFO_STREAM("Reposition flag unsuccessfull");
				arm->GoToTarget({up_pose,part_pose});
				part_pose.position.z-=0.005;

			}
			}
			ros::spinOnce();
			if(agv_id==2){
			if (repose_flag_dash_==1){
				arm->GoToTarget({up_pose,reposition_part_pose_back_dash_});
				part_pose=reposition_part_pose_back_dash_;
			}
			else{
				ROS_INFO_STREAM("Reposition flag unsuccessfull");
				arm->GoToTarget({up_pose,part_pose});
				part_pose.position.z-=0.005;

			}			
			}




			
			ros::Duration(0.1).sleep();
			arm->GoToTarget(up_pose);
			
			if (arm->GetGripperStatus()==true){
				ROS_INFO_STREAM("Product attached");
				break;
			}

			n++;
		}
		arm->GoToTarget(up_pose);
		init_cam5_2_=0;
		init_cam5_2_dash_=0;
		//part to be repositioned picked up
	    
	    
	    


		while(1){
		    arm->GoToTarget(StampedPose_out.pose);
		    ros::spinOnce();
		    if (abs(arm->GetEEPose().position.x-StampedPose_out.pose.position.x)<0.1   &&    abs(arm->GetEEPose().position.y-StampedPose_out.pose.position.y)<0.1 ){
		    	ROS_INFO_STREAM("State achieved");
		    	break;
		    }
		}



	    ros::Duration(1.0).sleep();
	    ROS_INFO_STREAM("Gone to keep part ");
	    ros::spinOnce();
	    if(arm->GetGripperStatus()==false){
			ROS_INFO_STREAM("Part was dropped before keeping");
			if (agv_id==1)
				end_position_[1]=1.55;
			else{
				end_position_[1]=3.1+1.5;
				end_position_[0]=-0.5;
			}

			
			while(1){         //so that we know the arm is at agv1
				arm->SendRobotJointPosition(end_position_);
				ros::spinOnce();
				//ROS_INFO_STREAM(joint_states_[0]-2.2);
				if (arm->GetJointStates()[0]-end_position_[0]<0.1)
					break;
			}
			end_position_[1]=3.1;
			end_position_[0]=1.18;
			return -1;
			continue;
		}

	    // if (arm->GetGripperStatus()==false){
	    // 	ROS_INFO_STREAM("Part was dropped somewhere while reposition before dropping");
	    // 	std::pair<std::string,geometry_msgs::Pose> temp;
	    // 	temp.first=diff_list.first;
	    // 	temp.second=diff_list.second.at(1);//in kit tray frame
	    // 	PickAndPlace(temp,int(1));
	    // }
	    
	    arm->GripperToggle(false);
	    //Dropping part at specified posn
	    break;}
	    



	    while(1){
	    //arm->GoToTarget(temp_pose);
	    temp_pose.position.z-=0.1;
	    temp_pose.position.z+=0.025;
	    //if(other_part_type_pose.first=="pulley_part")
	    	//temp_pose.position.z+=0.06;

		up_pose=temp_pose;
		up_pose.position.z+=0.25;
		

		int n=0;
		arm->GripperToggle(true);
		ROS_INFO_STREAM("GOing to pick other part      "<<other_part_type_pose.first);
		while(1){
			if(agv_id==1){
			init_cam5_2_=1;
			reposition_part_type_=other_part_type_pose.first;
			reposition_part_pose_=temp_pose;
			ros::Duration(0.25).sleep();
			ros::spinOnce();
			reposition_part_pose_back_.position.z+=0.02;
			//if(other_part_type_pose.first=="pulley_part")
	    		//reposition_part_pose_back_.position.z+=0.06;
			}


			if(agv_id==2){
			init_cam5_2_dash_=1;
			reposition_part_type_dash_=other_part_type_pose.first;
			reposition_part_pose_dash_=temp_pose;
			ros::Duration(0.25).sleep();
			ros::spinOnce();
			reposition_part_pose_back_dash_.position.z+=0.02;
			//if(other_part_type_pose.first=="pulley_part")
	    		//reposition_part_pose_back_dash_.position.z+=0.06;
			}



			if(agv_id==1){
			for (int i=0;i<n;i++)
				reposition_part_pose_back_.position.z-=0.005;
			}

			if(agv_id==2){
			for (int i=0;i<n;i++)
				reposition_part_pose_back_.position.z-=0.005;
			}
			ros::spinOnce();
			if(agv_id==1){
			if (repose_flag_==1){
				arm->GoToTarget({up_pose,reposition_part_pose_back_});
				temp_pose=reposition_part_pose_back_;
			}
			else{
				ROS_INFO_STREAM("Reposition flag unsuccessfull");
				arm->GoToTarget({up_pose,temp_pose});
				temp_pose.position.z-=0.005;

			}
			}

			if(agv_id==2){
			if (repose_flag_dash_==1){
				arm->GoToTarget({up_pose,reposition_part_pose_back_dash_});
				temp_pose=reposition_part_pose_back_dash_;
			}
			else{
				ROS_INFO_STREAM("Reposition flag unsuccessfull");
				arm->GoToTarget({up_pose,temp_pose});
				temp_pose.position.z-=0.005;

			}
			}


			ros::Duration(0.1).sleep();
			arm->GoToTarget(up_pose);
			//other_part_pose.position.z-=0.005;
			//other_part_pose=reposition_part_pose_back_;
			if (arm->GetGripperStatus()==true){
				ROS_INFO_STREAM("Product attached");
				break;
			}

			n++;
		}
		init_cam5_2_=0;
		init_cam5_2_dash_=0;
		//picking up other part from temp position
		




		
		ros::Duration(1.0).sleep();
		part_pose.position.z+=0.15;
		while(1){
		    arm->GoToTarget(part_pose);
		    ros::spinOnce();
		    if (abs(arm->GetEEPose().position.x-part_pose.position.x)<0.1    and   abs(arm->GetEEPose().position.y-part_pose.position.y)<0.1     ){
		    	ROS_INFO_STREAM("State achieved");
		    	break;
		    }
		}

		ros::Duration(0.5).sleep();
		ros::spinOnce();
		if(arm->GetGripperStatus()==false){
			ROS_INFO_STREAM("Part was dropped before keeping");
			if (agv_id==1)
				end_position_[1]=1.55;
			else{
				end_position_[1]=3.1+1.5;
				end_position_[0]=-0.5;
			}
			
			while(1){         //so that we know the arm is at agv1
				arm->SendRobotJointPosition(end_position_);
				ros::spinOnce();
				//ROS_INFO_STREAM(joint_states_[0]-2.2);
				if (arm->GetJointStates()[0]-end_position_[0]<0.1)
					break;
			}
			end_position_[1]=3.1;
			end_position_[0]=1.18;
			return -1;
			continue;
		}
		arm->GripperToggle(false);
		ROS_INFO_STREAM("Dropping other part to app location");
		//Dropping part at proper position

		ros::spinOnce();
		break;}
	}
    

    




    
    else{
    	ros::spinOnce();				
		//if (diff_list.first=="pulley_part")
			//part_pose.position.z+=0.06;
		
		ROS_INFO_STREAM("Normal reposition activated....");
		part_pose.position.z+=0.02;
		// if (agv_id==1)
		// 	//part_pose.position.y+=0.04;
		// else                                       //prevent fall off
		// 	//part_pose.position.y-=0.04;
		auto up_pose=part_pose;
		

		ROS_INFO_STREAM("y pos of part is   "<<abs(abs(part_pose.position.y)-3.353200));
		if(abs(abs(part_pose.position.y)-3.353200)<0.05){   //2 abs for both sides    Checking if pick up is at end of the tray
			up_pose.position.z+=0.25;
			ROS_INFO_STREAM("Normal positioning part is at temp position");

		}
		else{
			up_pose.position.z+=0.25;
		}

		int n=0;

		arm->GripperToggle(true);
		ROS_INFO_STREAM("GOing to pick   "<<diff_list.first);
		ROS_INFO_STREAM("pose"<<part_pose);
		while(1){
			arm->GoToTarget(part_pose);
			ros::spinOnce();
			arm->GoToTarget(up_pose);
			part_pose.position.z-=0.005;
			if (arm->GetGripperStatus()==true){
				ROS_INFO_STREAM("Product attached");
				break;
			}

		}

		

		


	    //Dropping part at specified posn
	    if (agv_id==1)
			StampedPose_out.pose.position.y+=0.04;
		else                                       //prevent fall off
			StampedPose_out.pose.position.y-=0.04;
	    
		while(1){
	    	arm->GoToTarget(StampedPose_out.pose);
	    	ros::spinOnce();
	    	if((StampedPose_out.pose.position.y-arm->GetEEPose().position.y)<0.01){
	    		ROS_INFO_STREAM("State achieved");
	    		break;
	    	}

		}
	    

	    ros::Duration(0.2).sleep();

	    if (arm->GetGripperStatus()==false){
	    	ROS_INFO_STREAM("Part was dropped somewhere while reposition before dropping");
	    	
			ROS_INFO_STREAM("Part was dropped before keeping");
			if (agv_id==1)
				end_position_[1]=1.55;
			else{
				end_position_[1]=3.1+1.5;
				end_position_[0]=-0.5;
			}
			
			while(1){         //so that we know the arm is at agv1
				arm->SendRobotJointPosition(end_position_);
				ros::spinOnce();
				//ROS_INFO_STREAM(joint_states_[0]-2.2);
				if (arm->GetJointStates()[0]-end_position_[0]<0.1)
					break;
			}
			end_position_[1]=3.1;
			end_position_[0]=1.18;
			
			return -1;
	    	// std::pair<std::string,geometry_msgs::Pose> temp;
	    	// temp.first=diff_list.first;
	    	// temp.second=diff_list.second.at(1);//in kit tray frame
	    	// PickAndPlace(temp,agv_id);
	    }

	    
    arm->GripperToggle(false);
		}
    

    ros::spinOnce();
	if (agv_id==1)
		end_position_[1]=1.55;
	else{
		end_position_[1]=3.1+1.5;
		end_position_[0]=-0.5;
	}

	while(1){         //so that we know the arm is at agv1
		arm->SendRobotJointPosition(end_position_);
		ros::spinOnce();
		ROS_INFO_STREAM(arm->GetJointStates()[1]-end_position_[1]);
		if (arm->GetJointStates()[1]-end_position_[1]<0.1)
			break;
	}
	end_position_[1]=3.1;
	end_position_[0]=1.18;
    //arm1_.SendRobotJointPosition(end_position_);

    //arm1_.DropPart(StampedPose_out.pose);
    //arm1_.GripperToggle(false);



return 0;}

void AriacOrderManager::removeElement(osrf_gear::Order ord ){
	for(int i=0;i<received_orders_.size();i++)
		{if (ord.order_id==received_orders_[i].order_id){
			received_orders_.erase(received_orders_.begin()+i);
			return;}
		}
}

void AriacOrderManager::ExecuteOrder() {
    ROS_WARN(">>>>>> Executing order...");
    //scanned_objects_ = camera_.GetParts();

    //-- used to check if pick and place was successful
    

    
    //Check empty bins
    std::string product_type=camera_.getbinPartCount().back();
    //double temp_time=ros::Time::now().toSec();
    std::string empty_bin=camera_.getEmptyBins()[0];;
    

    //arm1_.SendRobotHome();
    //arm2_.SendRobotHome();
    //RobotController *arm= &arm1_;
    //RobotController *arm_dash= &arm2_;
    //RobotController *arm=&arm1_;
    //RobotController *arm_dash=&arm2_;
    // geometry_msgs::Pose part_pose_1;part_pose_1.position.x=-0.200015;part_pose_1.position.y=1.766002;part_pose_1.position.z=0.7241+0.02;
    // arm->SendRobotHome();
    // arm->GripperToggle(true);
    // arm->GoToTarget(part_pose_1);
    // ROS_INFO_STREAM("Picked up");
    // part_pose_1.position.z+=0.2;

    // arm->GoToTarget(part_pose_1);
    // part_pose_1.position.z-=0.2;
    // arm->GripperToggle(false);
    // arm->PickPart(part_pose_1);



    // for(int i=0;i<empty_bin_coordinates_.size();i++){
    // 	auto temp_pair=empty_bin_coordinates_[i];
    // 	ROS_INFO_STREAM("Trying to keep part from conveyer into empty bin");
    // 	auto flag=PickFromConveyerBefore(temp_pair,product_type,empty_bin);
    // 	while(flag==-1){
    // 		flag=PickFromConveyerBefore(temp_pair,product_type,empty_bin);
    // 		ROS_INFO_STREAM("Part fell down before keeping so trying again");

    // 	}
    // 	if (flag==-2){
    // 		ROS_INFO_STREAM("Time elaped greater than 15");
    // 		break;
    // 	}
    		
    // }
    product_frame_list_=camera_.get_product_frame_list();
    //ROS_INFO_STREAM("These many type of products available   "<<product_frame_list_.size());
    auto temp=PickFromConveyerBefore(empty_bin_coordinates_,product_type,empty_bin);


    while (1)//Wait to receive 1st order
        {ros::spinOnce();
            if (received_orders_.size()!=0)
                break;
        }

    //ROS_INFO_STREAM("ARM2");
   
    ros::spinOnce();
    

    current_order_=received_orders_.at(0);
    removeElement(current_order_);
    
    bool pick_n_place_success{false};

    std::list<std::pair<std::string,geometry_msgs::Pose>> failed_parts;

    //camera_.UpdateLists();
    //product_frame_list_ = camera_.get_product_frame_list();
    //ros::spinOnce();
    //ROS_INFO_STREAM("These many type of products available   "<<product_frame_list_.size());

    //ros::Duration(1).sleep();
    //ros::Duration(5).sleep();
    
    while (1){


	    CompletethisOrder(current_order_);
	    ROS_INFO_STREAM("Congragulations Order Completed");
	    osrf_gear::Order order_update;
	    order_update.order_id="None";
	    ros::Duration(3.0).sleep();    //Waiting to see if there is a order update
	    ros::spinOnce();
	    if (order_update_flag_==1){
	    	
	    	for(const auto &ord:received_orders_){
	    		if(current_order_.order_id==ord.order_id.substr(0,current_order_.order_id.size())) {
	    			order_update=ord;
	    			removeElement(ord);

    				
	    			//removing order update from received orders
	    		}
	    	}
	    	if (order_update.order_id!="None"){
	    		CompleteOrderUpdate(order_update);
	    		order_update_flag_=0;
	    	}
	    }

	    ros::spinOnce();
	    int no=0;
	    if(received_orders_.size()==0){   //Waiting for new order to arrive if none has
	    	while(received_orders_.size()==0){
	    		ros::Duration(1.0).sleep();
	    		ros::spinOnce();
	    		ROS_INFO_STREAM("Waiting for new Order    "<<product_frame_list_.size());
	    		no++;
	    	}
    		
	    }
	    if (received_orders_.size()==0){
	    	ROS_INFO_STREAM("No new order received Finished the code.....");
	    	break;
	    }
	    
	    current_order_=received_orders_.at(0);
		received_orders_.erase(received_orders_.begin());
		ROS_INFO_STREAM("Order Finished going for next order....  "<<current_order_.order_id);
   
		}		
   }
   

   // for (const auto &order:received_orders_){
   //       //auto order_id = order.order_id;
   //       auto shipments = order.shipments;
   //       for (const auto &shipment: shipments){
   //           auto shipment_type = shipment.shipment_type;
   //            for (const auto &product: shipment.products){
   //              ROS_INFO_STREAM("Product to place is  "    <<product.type);
   //              std::pair<std::string,geometry_msgs::Pose> product_type_pose;
   //              product_type_pose.first=product.type;
   //              product_type_pose.second=product.pose;
   //              if (product_frame_list_.find(product.type)!=product_frame_list_.end())  //Found in list
   //                PickAndPlace(product_type_pose, int(1));
   //              else{
   //                ROS_INFO_STREAM("Please pick up from conveyer"    <<product.type);
   //                PickFromConveyer(product_type_pose.first,agv_id);
   //                bool result,result1;
   //                geometry_msgs::Pose drop_pose = product_type_pose.second;

			//       geometry_msgs::PoseStamped StampedPose_in,StampedPose_out;

			//       // if(agv_id==1){
			//       StampedPose_in.header.frame_id = "/kit_tray_1";
			//       StampedPose_in.pose = drop_pose;
			//       //ROS_INFO_STREAM("StampedPose_int (" << StampedPose_in.pose.position.x <<","<< StampedPose_in.pose.position.y << "," << StampedPose_in.pose.position.z<<")");
			//       part_tf_listener_.transformPose("/world",StampedPose_in,StampedPose_out);
			//       StampedPose_out.pose.position.z += 0.1;
			//       StampedPose_out.pose.position.y -= 0.2;
			//       //ROS_INFO_STREAM("StampedPose_out (" << StampedPose_out.pose.position.x <<","<< StampedPose_out.pose.position.y << "," << StampedPose_out.pose.position.z<<")");

			//     // }
			//       ros::Duration(0.5).sleep();
   //                while(1) {

	  //                 ros::spinOnce();
	  //                 if (arm1_.GetGripperStatus()==false){
	  //                 	//arm1_.SendRobotConveyer();
	  //                 	PickFromConveyer(product_type_pose.first,agv_id);
	  //                 	continue;
	  //                 }

	  //                 result = arm1_.DropPart(StampedPose_out.pose);
	  //                 ROS_INFO_STREAM("result from DropPart: "<< result);

			// 	    // bool result1;
			// 	    if (result==-1){
			// 	      ROS_INFO_STREAM("Retrying due to defected part");
			// 	      arm1_.SendRobotConveyer();
			// 	      arm1_.GripperToggle(false);
			// 	      continue;
				      

			// 	  }
			// 	  if (result==0) {
			// 	  	ROS_INFO_STREAM("Breaking out......");
			// 	  	break;
			// 	  }
			// }
			    

   //                  }
                
   //              }
   //              //Write function for pick and place from conveyer
   //              ROS_INFO_STREAM("Shipment completed Please send back AGV");
   //              SubmitAGV(1);
   //              ros::spinOnce();
   //              //SubmitAGV(1);
   //              }
   //            ROS_INFO_STREAM("Congragulations Order Completed");
   //            ros::spinOnce();
   //            }
             
//}


   


   
void AriacOrderManager::ThrowFaultyComponents(osrf_gear::LogicalCameraImage quality_image,int agv_id){
	int arm_id;
	std::string arm1,arm_dash1;

	RobotController *arm;
	RobotController *arm_dash;


	if (agv_id==1){
		arm1={"arm1"};
		arm_dash1={"arm2"};
		arm_id=2;
		arm= &arm1_;
		arm_dash= &arm2_;

	}

	if (agv_id==2){
		arm1={"arm2"};
		arm_dash1={"arm1"};
		arm_id=2;
		arm= &arm2_;
		arm_dash= &arm1_;
	}
	ROS_INFO_STREAM("Throwing ThrowFaultyComponents");
	
	// RobotController arm(arm1);
	// RobotController arm_dash(arm_dash1);


	if(agv_id==1)
	{end_position_[1]-=1.55;
	arm->SendRobotJointPosition(end_position_);
	end_position_[1]+=1.55;

	//end_position_[1]+=1.55;
	//arm_dash->SendRobotJointPosition(end_position_);
	//end_position_[1]-=1.55;
	}



	if(agv_id==2)
	{home_joint_pose_[1]+=1.55;
	arm->SendRobotJointPosition(home_joint_pose_);
	home_joint_pose_[1]-=1.55;

	// end_position_[1]+=1.55;
	// arm_dash->SendRobotJointPosition(end_position_);
	// end_position_[1]-=1.55;
	}



	

	for(int i=0;i<quality_image.models.size();i++){
		std::string product_type;
		if (abs(quality_image.models[i].pose.position.z-0.4235)<0.01)
			product_type="pulley_part";

		geometry_msgs::PoseStamped StampedPose_in,StampedPose_out;          
        StampedPose_in.header.frame_id = "/quality_control_sensor_1_frame";
        if(agv_id==2)StampedPose_in.header.frame_id = "/quality_control_sensor_2_frame";
        StampedPose_in.pose = quality_image.models[i].pose;
        //ROS_INFO_STREAM("StampedPose_int (" << StampedPose_in.pose.position.x <<","<< StampedPose_in.pose.position.y << "," << StampedPose_in.pose.position.z<<")");
        part_tf_listener_.transformPose("/world",StampedPose_in,StampedPose_out);
        auto part_pose = StampedPose_out.pose;
        part_pose.position.z+=0.02;
        if (product_type=="pulley_part"){
        	part_pose.position.z+=0.07;
        }
        while(1){
        	arm->PickPart(part_pose);
        	if(arm->GetGripperStatus()==true)
        		break;
        }

        if (agv_id==1)
        	part_pose.position.y=2.5;
        else
            part_pose.position.y=-2.5;

        arm->GoToTarget(part_pose);
        arm->GripperToggle(false);






	}

}

   
void AriacOrderManager::CheckOrder(osrf_gear::Shipment shipment){
	
	ROS_INFO_STREAM("Checking shipment before departure");
	std::vector <std::pair<std::string,geometry_msgs::Pose> > order_list;
	auto shipment_type = shipment.shipment_type;
	int agv_id=0;
	auto agv_id_string=shipment.agv_id;
	//agv_id_string="agv2";
	int agv=0;
	ros::spinOnce();
	osrf_gear::LogicalCameraImage quality_image;
	if (agv_id_string=="agv1"   ||   agv_id_string=="any"){
		quality_image=arm1_.quality_control_sensor_image_;
		agv=1;
	}
	else{
		quality_image=arm2_.quality_control_sensor_image_;
		agv=2;
	}
	ROS_INFO_STREAM(quality_image);
	if(quality_image.models.size()!=0){
		ThrowFaultyComponents(quality_image,agv);
	}

	
	std::vector <std::pair<std::string,geometry_msgs::Pose> > cam5_list_right;
	if (agv_id_string=="any"    ||    agv_id_string=="agv1"){
		init_cam5_=1;
		ros::Duration(0.2).sleep();
		ros::spinOnce();     //to update part list of kit_tray i.e. cam5_list_	
		init_cam5_=0;
		cam5_list_right=cam5_list_;
		cam5_list_right=arm1_.GetKitTrayList();
		agv_id=1;
	}

	else{
		init_cam5_dash_=1;
		ros::Duration(0.2).sleep();
		ros::spinOnce();     //to update part list of kit_tray i.e. cam5_list_	
		init_cam5_dash_=0;
		cam5_list_right=cam5_list_dash_;
		cam5_list_right=arm2_.GetKitTrayList();
		agv_id=2;
	}



	  for (const auto &product: shipment.products){
	  	std::pair<std::string,geometry_msgs::Pose> temp;
	  	temp.first=product.type;
	  	temp.second=product.pose;
	  	order_list.push_back(temp);
	  }
	std::vector <std::pair<std::string,std::vector<geometry_msgs::Pose> > > repos_list;
	std::vector <std::pair<std::string,geometry_msgs::Pose> > order_repos_list(order_list);
	std::vector <std::pair<std::string,geometry_msgs::Pose> > cam5_repos_list(cam5_list_right);
	std::vector <std::pair<std::string,geometry_msgs::Pose> > diff_list;
	std::vector <std::pair<std::string,geometry_msgs::Pose> > get_list;
	ROS_INFO_STREAM("Camera 5 can see these many components  "<<cam5_list_right.size());
	for(int i=0;i<cam5_list_right.size();i++)
		{ROS_INFO_STREAM("  " <<cam5_list_right[i].first);}

	for(int i=0;i<order_repos_list.size();i++){
		for (int j=0;j<cam5_repos_list.size();j++){
			if (cam5_repos_list[j].first==order_repos_list[i].first){
				std::vector<geometry_msgs::Pose> poses;
				poses.push_back(cam5_repos_list[j].second);
				poses.push_back(order_repos_list[i].second);
				std::pair<std::string,std::vector<geometry_msgs::Pose>> temp;
				temp.first=cam5_repos_list[j].first;
				temp.second=poses;
				repos_list.push_back(temp);
				cam5_repos_list.erase(cam5_repos_list.begin()+j);
				order_repos_list.erase(order_repos_list.begin()+i);
				i--;
				continue;
			}
		}
	}

	diff_list=cam5_repos_list;
	get_list=order_repos_list;
	
	for(int i=0;i<diff_list.size();i++){
		ROS_INFO_STREAM("Throwing    "<<diff_list[i].first);
		ThrowComponents(diff_list[i],agv_id);
	}

	for(int j=0;j<repos_list.size();j++){
		int r=-1;
		while(r==-1){
			ROS_INFO_STREAM("Reposition    "<<repos_list[j].first);
			r=RepositionComponents(repos_list[j],agv_id);
			//if(r==-1)
				//repos_list=UpdateLists(order);
		}
	}



	for(int k=0;k<get_list.size();k++){
		if (product_frame_list_.find(get_list[k].first)!=product_frame_list_.end()){  //Found in list
	      ROS_INFO_STREAM("Get    "<<get_list[k].first);
	      PickAndPlace(get_list[k], agv_id,0);     //add conveyer type code
	  	}
	  	else
  		  

  		  PickAndPlace(get_list[k], agv_id,1);

	}
}



void AriacOrderManager::SubmitAGV(int num) {
    std::string s = std::to_string(num);
    ros::ServiceClient start_client =
            order_manager_nh_.serviceClient<osrf_gear::AGVControl>("/ariac/agv"+s);
    if (!start_client.exists()) {
        ROS_INFO("Waiting for the client to be ready...");
        start_client.waitForExistence();
        ROS_INFO("Service started.");
    }

    osrf_gear::AGVControl srv;
    // srv.request.kit_type = "order_0_kit_0";
    start_client.call(srv);

    if (!srv.response.success) {
        ROS_ERROR_STREAM("Service failed!");
    } else
        ROS_INFO("Service succeeded.");
}