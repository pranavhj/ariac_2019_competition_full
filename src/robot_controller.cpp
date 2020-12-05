//
// Created by zeid on 2/27/20.
//
#include "robot_controller.h"

/**
 * Constructor for the robot
 * Class attributes are initialized in the constructor init list
 * You can instantiate another robot by passing the correct parameter to the constructor
 */




RobotController::RobotController(std::string arm_id) :
robot_controller_nh_("/ariac/"+arm_id),
robot_controller_options("manipulator",
        "/ariac/"+arm_id+"/robot_description",
        robot_controller_nh_),
robot_move_group_(robot_controller_options) 



{   arm_id_=arm_id;
    //ROS_WARN(">>>>> RobotController");
    if (arm_id=="arm1")
        {kit_tray_="kit_tray_1";}
    if (arm_id=="arm2")
        {kit_tray_="kit_tray_2";}

    robot_move_group_.setPlanningTime(20);
    robot_move_group_.setNumPlanningAttempts(10);
    robot_move_group_.setPlannerId("RRTConnectkConfigDefault");
    robot_move_group_.setMaxVelocityScalingFactor(0.9);
    robot_move_group_.setMaxAccelerationScalingFactor(0.9);
    // robot_move_group_.setEndEffector("moveit_ee");
    robot_move_group_.allowReplanning(true);


    //--These are joint positions used for the home position
    // home_joint_pose_ = {0.0, 3.1, -1.1, 1.9, 3.9, 4.7, 0};
    //home_joint_pose_ = {0.5753406842498247, 5.933530060266838, -0.6150311670595325, 1.6234181541925308, 3.5494995694291624, 4.692497629212904, 2.819532850928649};
    home_joint_pose_ = {0.0,3.1,-1.1, 1.9, 3.9, 4.7, 0};
    //conveyer_joint_pose_ = {-1.2800,0.0,-1.1, 1.9, 3.9, 4.7, 0};
    conveyer_joint_pose_={1.2347303272802614, -1.18000996636373, -0.7160215018955451, 0.4537723748583051, 4.201092772034111, 4.7268835205218185, -2.645671088827692, 0.0};
    auto cvjp=conveyer_joint_pose_;
    auto positions=cvjp;
    std::vector <int>indices={1,3,2,0,4,5,6};
    conveyer_joint_pose_.clear();
    for(int i=0;i<indices.size();i++){
        conveyer_joint_pose_.push_back(positions[indices[i]]);
    }
    // conveyer_joint_pose_[4]=0;
    // conveyer_joint_pose_[5]=0;
    // conveyer_joint_pose_[6]=0;
    
    
    if (arm_id_=="arm2")
        home_joint_pose_[0]-=0.7;
    
    flipping_joint_pose_ = {0.507,3.1,-1.1, 1.9, 3.9, 4.7, 0};

    //-- offset used for picking up parts
    //-- For the pulley_part, the offset is different since the pulley is thicker
    offset_ = 0.02;

    //--topic used to get the status of the gripper
    gripper_subscriber_ = gripper_nh_.subscribe(
            "/ariac/"+arm_id+"/gripper/state", 10, &RobotController::GripperCallback, this);

    if (arm_id_=="arm1"){
        quality_control_sensor_1_subscriber_ = gripper_nh_.subscribe(
            "/ariac/quality_control_sensor_1", 10, &RobotController::QualityControlSensor1Callback, this);
    }
    else{
        quality_control_sensor_1_subscriber_ = gripper_nh_.subscribe(
            "/ariac/quality_control_sensor_2", 10, &RobotController::QualityControlSensor1Callback, this);
    }


    //add arm_id over here
    joint_states_subscriber_ = gripper_nh_.subscribe("/ariac/arm1/joint_states", 10,
                                                &RobotController::JointStatesCallback, this);

    joint_states_subscriber_2_ = gripper_nh_.subscribe("/ariac/arm2/joint_states", 10,
                                                &RobotController::JointStatesCallback_2, this);


    if (arm_id_=="arm1")
        kit_tray_camera_subscriber_ = gripper_nh_.subscribe("/ariac/logical_camera_8", 10,
                                                &RobotController::KitTrayCameraCallback, this);
    else
        kit_tray_camera_subscriber_ = gripper_nh_.subscribe("/ariac/logical_camera_9", 10,
                                                &RobotController::KitTrayCameraCallback, this);

    SendRobotHome();
    ros::Duration(0.25).sleep();

    robot_tf_listener_.waitForTransform(arm_id+"_linear_arm_actuator", arm_id+"_ee_link",
                                            ros::Time(0), ros::Duration(10));
    robot_tf_listener_.lookupTransform("/"+arm_id+"_linear_arm_actuator", "/"+arm_id+"_ee_link",
                                           ros::Time(0), robot_tf_transform_);


    fixed_orientation_.x = robot_tf_transform_.getRotation().x();
    fixed_orientation_.y = robot_tf_transform_.getRotation().y();
    fixed_orientation_.z = robot_tf_transform_.getRotation().z();
    fixed_orientation_.w = robot_tf_transform_.getRotation().w();

    tf::quaternionMsgToTF(fixed_orientation_,q);
    tf::Matrix3x3(q).getRPY(roll_def_,pitch_def_,yaw_def_);


    end_position_ = home_joint_pose_;
    if(arm_id_=="arm1"){
        end_position_[0] = 2.2;             //This is for agv1
    }
    else
        end_position_[0]=-2.2;
    //end_position_[0]=1.18;

    //end_position_[1] = 1.55;             //This is for agv1
//    end_position_[1] = 4.5;
//    end_position_[2] = 1.2;


    robot_tf_listener_.waitForTransform("world", arm_id+"_ee_link", ros::Time(0),
                                            ros::Duration(10));
    robot_tf_listener_.lookupTransform("/world", "/"+arm_id+"_ee_link", ros::Time(0),
                                           robot_tf_transform_);

    home_cart_pose_.position.x = robot_tf_transform_.getOrigin().x();
    home_cart_pose_.position.y = robot_tf_transform_.getOrigin().y();
    home_cart_pose_.position.z = robot_tf_transform_.getOrigin().z();
    home_cart_pose_.orientation.x = robot_tf_transform_.getRotation().x();
    home_cart_pose_.orientation.y = robot_tf_transform_.getRotation().y();
    home_cart_pose_.orientation.z = robot_tf_transform_.getRotation().z();
    home_cart_pose_.orientation.w = robot_tf_transform_.getRotation().w();

    agv_tf_listener_.waitForTransform("world", kit_tray_,
                                      ros::Time(0), ros::Duration(10));
    agv_tf_listener_.lookupTransform("/world", "/"+kit_tray_,
                                     ros::Time(0), agv_tf_transform_);
    agv_position_.position.x = agv_tf_transform_.getOrigin().x();
    agv_position_.position.y = agv_tf_transform_.getOrigin().y();
    agv_position_.position.z = agv_tf_transform_.getOrigin().z() + 4 * offset_;

    gripper_client_ = robot_controller_nh_.serviceClient<osrf_gear::VacuumGripperControl>(
            "/ariac/"+arm_id+"/gripper/control");
    counter_ = 0;
    drop_flag_ = false;
    //SendRobotHome();
}

RobotController::~RobotController() {}

/**
 *
 * @return
 */


void RobotController::KitTrayCameraCallback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg){
    kit_tray_camera_count_=image_msg->models.size();

    //repose_flag_=0;
    geometry_msgs::PoseStamped StampedPose_in,StampedPose_out; 
    if(init_before_==1){
        //if (init_after_==0)
        //    kit_tray_list_before_.clear();
        //else
            kit_tray_list_.clear();
            kit_tray_list_without_transform_.clear();
        ROS_INFO_STREAM("Making List");
        for(int i=0;i<image_msg->models.size();i++){
            if (arm_id_=="arm1")
                StampedPose_in.header.frame_id = "/logical_camera_8_frame";
            else
                StampedPose_in.header.frame_id = "/logical_camera_9_frame";
            StampedPose_in.pose = image_msg->models[i].pose;
            
            robot_tf_listener_.transformPose("/world",StampedPose_in,StampedPose_out);
            std::pair<std::string,geometry_msgs::Pose> temp;
            temp.first=image_msg->models[i].type;
            temp.second=image_msg->models[i].pose;
            kit_tray_list_without_transform_.push_back(temp);
            temp.second=StampedPose_out.pose;
            //if (init_after_==0)
            //    kit_tray_list_before_.push_back(temp);
            //else
            kit_tray_list_.push_back(temp);
        }

    //init_cam5_2_=0;
    }

}


std::vector<std::pair<std::string,geometry_msgs::Pose>> RobotController::GetKitTrayList(){
    init_before_=1;
    ros::Duration(0.2).sleep();
    ros::spinOnce();
    //auto list_before=kit_tray_list_;
    ROS_INFO_STREAM("Kit tray list has these many components  "<<kit_tray_list_without_transform_.size()); 
    init_before_=0;
    return kit_tray_list_without_transform_;

}


void printVector(std::vector<auto> vect)
{for (int i=0;i<vect.size();i++)
    {std::cout<<vect[i]<<"  ";}
std::cout<<std::endl;}


void RobotController::JointStatesCallback(const sensor_msgs::JointState::ConstPtr& joint_states){
    //printVector(joint_states_);
    auto positions=joint_states->position;
    std::vector <int>indices={1,3,2,0,4,5,6};
    joint_states_.clear();
    for(int i=0;i<indices.size();i++){
        joint_states_.push_back(positions[indices[i]]);
    }
}

geometry_msgs::Pose RobotController::GetEEPose(){
    ros::spinOnce();
    geometry_msgs::Pose p;
    robot_tf_listener_.waitForTransform("world", arm_id_+"_ee_link",
                                            ros::Time(0), ros::Duration(10));
    robot_tf_listener_.lookupTransform("/world", "/"+arm_id_+"_ee_link",
                                           ros::Time(0), robot_tf_transform_);


    p.position.x= robot_tf_transform_.getOrigin().x();
    p.position.y= robot_tf_transform_.getOrigin().y();
    p.position.z= robot_tf_transform_.getOrigin().z();
    
    return p;

}

std::vector<double> RobotController::GetJointStates(){
    //ros::Duration(0.05).sleep();
    ros::spinOnce();
    ros::Duration(0.05).sleep();
    if (arm_id_=="arm1"){
        return joint_states_;

    }
    if (arm_id_=="arm2"){
        return joint_states_2_;
    }

}


void RobotController::JointStatesCallback_2(const sensor_msgs::JointState::ConstPtr& joint_states){
    //ROS_WARN(">>>>> OrderCallback");
    auto positions=joint_states->position;
    std::vector <int>indices={1,3,2,0,4,5,6};
    joint_states_2_.clear();
    for(int i=0;i<indices.size();i++){
        joint_states_2_.push_back(positions[indices[i]]);
    }
}


void RobotController::QualityControlSensor1Callback(const osrf_gear::LogicalCameraImage::ConstPtr & msg){

osrf_gear::LogicalCameraImage temp;
quality_control_sensor_image_=temp;

if((*msg).models.size()!=0){
    quality_control_sensor_1_flag_=true;
    defected_part_=(*msg).models[0].type;
    quality_control_sensor_image_=*msg;
    //ROS_INFO_STREAM("Quality image updated");
    }    //Bad Quality Detected



}

bool RobotController::GetQualityControl1Status(){
return quality_control_sensor_1_flag_;

}

bool RobotController::Planner() {
    //ROS_INFO_STREAM("Planning started...");
    if (robot_move_group_.plan(robot_planner_) ==
        moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        plan_success_ = true;
        //ROS_INFO_STREAM("Planner succeeded!");
    } else {
        plan_success_ = false;
        ROS_WARN_STREAM("Planner failed!");
    }

    return plan_success_;
}


void RobotController::Execute() {
    ros::AsyncSpinner spinner(4);
    spinner.start();
    if (this->Planner()) {
        robot_move_group_.move();
        //ros::Duration(0.5).sleep();
    }
}

void RobotController::GoToTarget(const geometry_msgs::Pose& pose) {
    target_pose_.orientation = fixed_orientation_;
    target_pose_.position = pose.position;
    ros::AsyncSpinner spinner(4);
    robot_move_group_.setPoseTarget(target_pose_);
    spinner.start();
    if (this->Planner()) {
        robot_move_group_.move();
        //ros::Duration(0.5).sleep();
    }
    //ROS_INFO_STREAM("Point reached...");
}

void RobotController::GoToTarget(
        std::initializer_list<geometry_msgs::Pose> list) {
    ros::AsyncSpinner spinner(4);
    spinner.start();

    std::vector<geometry_msgs::Pose> waypoints;
    for (auto i : list) {
        i.orientation.x = fixed_orientation_.x;
        i.orientation.y = fixed_orientation_.y;
        i.orientation.z = fixed_orientation_.z;
        i.orientation.w = fixed_orientation_.w;
        waypoints.emplace_back(i);
    }

    moveit_msgs::RobotTrajectory traj;
    auto fraction =
            robot_move_group_.computeCartesianPath(waypoints, 0.01, 0.0, traj, true);

    ROS_WARN_STREAM("Fraction: " << fraction * 100);
    //ros::Duration(5.0).sleep();
    //ros::Duration(0.5).sleep();

    robot_planner_.trajectory_ = traj;

    //if (fraction >= 0.3) {
        robot_move_group_.execute(robot_planner_);
        //ros::Duration(5.0).sleep();
        //ros::Duration(0.5).sleep();
//    } else {
//        ROS_ERROR_STREAM("Safe Trajectory not found!");
//    }
}


void RobotController::GoToTarget_with_orientation(const geometry_msgs::Pose& pose) {
    target_pose_.orientation = pose.orientation;
    target_pose_.position = pose.position;
    ros::AsyncSpinner spinner(4);
    robot_move_group_.setPoseTarget(target_pose_);
    spinner.start();
    if (this->Planner()) {
        robot_move_group_.move();
        //ros::Duration(0.5).sleep();
    }
    //ROS_INFO_STREAM("Point reached...");
}


void RobotController::GoToTarget_with_orientation(
        std::initializer_list<geometry_msgs::Pose> list) {
    ros::AsyncSpinner spinner(4);
    spinner.start();

    std::vector<geometry_msgs::Pose> waypoints;
    for (auto i : list) {
        i.orientation.x = i.orientation.x;
        i.orientation.y = i.orientation.y;
        i.orientation.z = i.orientation.z;
        i.orientation.w = i.orientation.w;
        waypoints.emplace_back(i);
    }

    moveit_msgs::RobotTrajectory traj;
    auto fraction =
            robot_move_group_.computeCartesianPath(waypoints, 0.01, 0.0, traj, true);

    ROS_WARN_STREAM("Fraction: " << fraction * 100);
    //ros::Duration(5.0).sleep();
    //ros::Duration(0.5).sleep();

    robot_planner_.trajectory_ = traj;

    //if (fraction >= 0.3) {
        robot_move_group_.execute(robot_planner_);
        //ros::Duration(5.0).sleep();
        //ros::Duration(0.5).sleep();
//    } else {
//        ROS_ERROR_STREAM("Safe Trajectory not found!");
//    }
}

void RobotController::SendRobotHome() {
    // ros::Duration(2.0).sleep();
    ROS_INFO_STREAM("Sending robot home");
    robot_move_group_.setJointValueTarget(home_joint_pose_);
    // this->execute();
    ros::AsyncSpinner spinner(4);
    spinner.start();
    if (this->Planner()) {
        robot_move_group_.move();
        ros::Duration(0.1).sleep();
    }

     //ros::Duration(2.0).sleep();
}


void RobotController::SendRobotJointPosition(std::vector<double> joint_states) {
    // ros::Duration(2.0).sleep();
    ROS_INFO_STREAM("Sending robot to specified joint_position");
    robot_move_group_.setJointValueTarget(joint_states);
    // this->execute();
    ros::AsyncSpinner spinner(4);
    spinner.start();
    if (this->Planner()) {
        robot_move_group_.move();
        ros::Duration(0.1).sleep();
    }

     ros::Duration(0.3).sleep();
}


void RobotController::SendRobotConveyer() {
    // ros::Duration(2.0).sleep();
    ROS_INFO_STREAM("Sending robot Conveyer");
    robot_move_group_.setJointValueTarget(conveyer_joint_pose_);
    // this->execute();
    ros::AsyncSpinner spinner(4);
    spinner.start();
    if (this->Planner()) {
        robot_move_group_.move();
        //ros::Duration(0.5).sleep();
    }

     //ros::Duration(2.0).sleep();
}


void RobotController::SendRobotFlipping() {
    // ros::Duration(2.0).sleep();
    ROS_INFO_STREAM("Sending robot Flipping");
    robot_move_group_.setJointValueTarget(flipping_joint_pose_);
    // this->execute();
    ros::AsyncSpinner spinner(4);
    spinner.start();
    if (this->Planner()) {
        robot_move_group_.move();
        ros::Duration(0.5).sleep();
    }

     //ros::Duration(2.0).sleep();
}

void RobotController::GripperToggle(const bool& state) {
    ROS_INFO_STREAM("Actuating the gripper :"<<state);
    gripper_service_.request.enable = state;
    gripper_client_.call(gripper_service_);
    ros::Duration(0.5).sleep();
    // if (gripper_client_.call(gripper_service_)) {
    if (gripper_service_.response.success) {
        //ROS_INFO_STREAM("Gripper activated!");
    } else {
        //ROS_WARN_STREAM("Gripper activation failed!");
    }
}

// bool RobotController::dropPart(geometry_msgs::Pose part_pose) {
//   counter_++;
//
//   pick = false;
//   drop = true;
//
//   ROS_WARN_STREAM("Dropping the part number: " << counter_);
//
//   // ROS_INFO_STREAM("Moving to end of conveyor...");
//   // robot_move_group_.setJointValueTarget(part_pose);
//   // this->execute();
//   // ros::Duration(1.0).sleep();
//   // this->gripper_state_check(part_pose);
//
//   if (drop == false) {
//     // ROS_INFO_STREAM("I am stuck here..." << object);
//     ros::Duration(2.0).sleep();
//     return drop;
//   }
//   ROS_INFO_STREAM("Dropping on AGV...");
//
//   // agv_position_.position.x -= 0.1;
//   // if (counter_ == 1) {
//   //   agv_position_.position.y -= 0.1;
//   // }
//   // if (counter_ >= 2) {
//   //   agv_position_.position.y += 0.1;
//   //   // agv_position_.position.x +=0.1;
//   // }
//
//   auto temp_pose = part_pose;
//   // auto temp_pose = agv_position_;
//   temp_pose.position.z += 0.35;
//   // temp_pose.position.y += 0.5;
//
//   // this->setTarget(part_pose);
//   // this->execute();
//   // ros::Duration(1.0).sleep();
//   this->goToTarget({temp_pose, part_pose});
//   ros::Duration(1).sleep();
//   ROS_INFO_STREAM("Actuating the gripper...");
//   this->gripperToggle(false);
//
//   // ROS_INFO_STREAM("Moving to end of conveyor...");
//   // robot_move_group_.setJointValueTarget(end_position_);
//   // this->execute();
//   // ros::Duration(1.0).sleep();
//
//   ROS_INFO_STREAM("Going to home...");
//   // this->sendRobotHome();
//   // temp_pose = home_cart_pose_;
//   // temp_pose.position.z -= 0.05;
//   this->goToTarget({temp_pose, home_cart_pose_});
//   return drop;
// }
float euler_distance(geometry_msgs::Pose p1,geometry_msgs::Pose p2){
    float x1=p1.position.x;
    float y1=p1.position.y;
    float z1=p1.position.z;
    float x2=p2.position.x;
    float y2=p2.position.y;
    float z2=p2.position.z;

    float dist=((x2-x1)*(x2-x1))+((y2-y1)*(y2-y1));
    return dist;



}


void RobotController::ThrowFaultyProducts(){
    for(int i=0;i<quality_control_sensor_image_.models.size();i++){
        ROS_INFO_STREAM("ThrowFaultyProducts.........");
        auto part_pose=quality_control_sensor_image_.models[i].pose;

        std::string product_type;
        if (abs(quality_control_sensor_image_.models[i].pose.position.z-0.4235)<0.01)
            product_type="pulley_part";

        geometry_msgs::PoseStamped StampedPose_in,StampedPose_out;          
        StampedPose_in.header.frame_id = "/quality_control_sensor_1_frame";
        if(arm_id_=="arm2")StampedPose_in.header.frame_id = "/quality_control_sensor_2_frame";
        StampedPose_in.pose = quality_control_sensor_image_.models[i].pose;
        //ROS_INFO_STREAM("StampedPose_int (" << StampedPose_in.pose.position.x <<","<< StampedPose_in.pose.position.y << "," << StampedPose_in.pose.position.z<<")");
        robot_tf_listener_.transformPose("/world",StampedPose_in,StampedPose_out);
        part_pose = StampedPose_out.pose;
        part_pose.position.z+=0.02;
        if (product_type=="pulley_part"){
            part_pose.position.z+=0.07;
        }
        while(1){
            this->PickPart(part_pose);
            if(this->GetGripperStatus()==true)
                break;
            ros::spinOnce();
            //quality_control_sensor_image_.models[i].pose;
            StampedPose_in.header.frame_id = "/quality_control_sensor_1_frame";
            if(arm_id_=="arm2")StampedPose_in.header.frame_id = "/quality_control_sensor_2_frame";
            StampedPose_in.pose = quality_control_sensor_image_.models[i].pose;
            //ROS_INFO_STREAM("StampedPose_int (" << StampedPose_in.pose.position.x <<","<< StampedPose_in.pose.position.y << "," << StampedPose_in.pose.position.z<<")");
            robot_tf_listener_.transformPose("/world",StampedPose_in,StampedPose_out);
            part_pose = StampedPose_out.pose;
            part_pose.position.z+=0.02;
            if (product_type=="pulley_part"){
                part_pose.position.z+=0.07;
            }
        }

        if (arm_id_=="arm1")
            part_pose.position.y=2.6;
        else
            part_pose.position.y=-2.6;

        this->GoToTarget(part_pose);
        this->GripperToggle(false);

    }

}

int RobotController::DropPart(std::string product_type,geometry_msgs::Pose part_pose) {
    // counter_++;

    drop_flag_ = true;
    ros::Duration(0.1).sleep();
    ros::spinOnce();
    ROS_INFO_STREAM("Placing phase activated...");

    if (gripper_state_==false){
        ROS_INFO_STREAM("Part was dropped somewhere");
        return -1;
    }

    init_before_=1;
    ros::Duration(0.2).sleep();
    ros::spinOnce();
    auto list_before=kit_tray_list_;
    ROS_INFO_STREAM("Before list has these many components  "<<list_before.size()); 
    init_before_=0;


    if(gripper_state_==false){ROS_INFO_STREAM("returning before moving towards agv");return -1;}
    if (gripper_state_){//--while the part is still attached to the gripper
        //--move the robot to the end of the rail
        
        if(arm_id_=="arm1"){
            end_position_[1]-=1.55;
            while(1)
            {    ROS_INFO_STREAM("Moving towards AGV1...");
                 robot_move_group_.setJointValueTarget(end_position_);
                 this->Execute();
                 ros::Duration(0.05).sleep();
                 ros::spinOnce();
                 ros::Duration(0.05).sleep();
                 ROS_INFO_STREAM("diff  "<<abs(joint_states_[1]- 1.55));
                 //if (gripper_state_==false) return -1;
                 if (abs(joint_states_[1]-1.55)<0.1)
                    break;
            }
         end_position_[1]+=1.55;
        }

         else{
            SendRobotHome();
            home_joint_pose_[1]+=1.55;
            while(1)
            {    ROS_INFO_STREAM("Moving towards AGV2...");
                 robot_move_group_.setJointValueTarget(home_joint_pose_);
                 this->Execute();
                 ros::Duration(0.05).sleep();
                 ros::spinOnce();
                 ros::Duration(0.05).sleep();
                 ROS_INFO_STREAM("diff  "<<abs(joint_states_2_[1]-home_joint_pose_[1]));
                 //if (gripper_state_==false) return -1;
                 if (abs(joint_states_2_[1]-home_joint_pose_[1])<0.01)
                    break;
            }
            home_joint_pose_[1]-=1.55;
         }
        //ros::Duration(1).sleep();
         //ROS_INFO_STREAM("Actuating the gripper...");
         //this->GripperToggle(false);
        ros::spinOnce(); 
        //ros::Duration(0.05).sleep();
        ros::Duration(0.1).sleep();
        int kit_tray_count_before=list_before.size();
        ros::spinOnce();
        //if (gripper_state_==false) return -1; 
        auto temp_pose = part_pose;
        temp_pose.position.z += 0.2;    //uppose
        ROS_INFO_STREAM("Going to keep part"<<list_before.size());
        
        
        //ros::Duration(1).sleep();
        part_pose.position.z+=0.2;         //so that arm remains above before keeping so that we can check where part has fallen
        GoToTarget({temp_pose, part_pose});
        while(1){
            GoToTarget(part_pose);
            if(abs(GetEEPose().position.x-part_pose.position.x)<0.02){            //achieve keeping pose
                ROS_INFO_STREAM("State achieved");
                break;
            }
            ROS_INFO_STREAM("Trying to go to keeping pose");
            ros::spinOnce();ros::Duration(0.05).sleep();




            ros::Duration(0.1).sleep();
        }

        ros::Duration(0.1).sleep();
        
        ros::spinOnce();

        if (gripper_state_==false) {
            init_before_=1;
            ros::Duration(0.2).sleep();
            ros::spinOnce();
            init_before_=0;


            ROS_INFO_STREAM("Before and After  "<<std::to_string(list_before.size())+"  "+std::to_string(kit_tray_list_.size()));
            if(kit_tray_list_.size()!=list_before.size()){
                ROS_INFO_STREAM("Part has fallen down on kit tray");
                //RepositionDropppedPart(part_pose);
                init_before_=1;
                ros::Duration(0.2).sleep();    //making after list
                ros::spinOnce();
                init_before_=0;
                
                //auto list_before=kit_tray_list_before_;       //seprating to ensure no changes due to rosspin
                auto list_after=kit_tray_list_;
                ROS_INFO_STREAM("No of part before and after are are"<<std::to_string(list_before.size())+"  "+std::to_string(list_after.size()));
                for(int i=0;i<list_before.size();i++){
                    for(int j=0;j<list_after.size();j++){
                        ROS_INFO_STREAM("parts are"<<list_before[i].first+list_after[j].first);
                        ROS_INFO_STREAM("Distance is  "<<euler_distance(list_before[i].second,list_after[j].second));
                        if(list_before[i].first==list_after[j].first    &&     euler_distance(list_before[i].second,list_after[j].second)<0.001) {
                            //ROS_INFO_STREAM("Part removed       "<<list_before[i].first);
                            list_after.erase(list_after.begin()+j);
                            list_before.erase(list_before.begin()+i);
                            i--;
                            break;


                        }
                    }


                
                }

                ROS_INFO_STREAM("Reposition part is"<<list_after[0].first+" "+std::to_string(list_after.size()));
                //ROS_INFO_STREAM("And it is at"<<list_after[0].second);
                auto picking_part_pose=list_after[0].second;
                if (list_after[0].first=="pulley_part")
                    picking_part_pose.position.z+=0.06;
                picking_part_pose.position.z+=0.02;
                
                auto up_pose=picking_part_pose;
                up_pose.position.z+=0.2;
                GripperToggle(true);
                while(1){
                    GoToTarget(picking_part_pose);
                    GoToTarget(up_pose);
                    ros::spinOnce();ros::Duration(0.05).sleep();
                    if (gripper_state_==true){
                        ROS_INFO_STREAM("Product attached");
                        break;
                    }
                    
                    picking_part_pose.position.z-=0.004;

                }
                up_pose.position.z+=0.2;
                if(part_pose.position.y<2.83   &&   arm_id_=="arm1")
                    part_pose.position.y=2.83 ;

                if(part_pose.position.y>-2.83   &&   arm_id_=="arm2")
                    part_pose.position.y=-2.83 ;
                
                GoToTarget(up_pose);
                ros::spinOnce();
                while(abs(GetEEPose().position.x- part_pose.position.x)>0.01   &&    abs(GetEEPose().position.y- part_pose.position.y)>0.01   ){
                    GoToTarget(part_pose);
                    ROS_INFO_STREAM("Trying to go to drop part pose");
                    ros::spinOnce();ros::Duration(0.05).sleep();
                }
                ROS_INFO_STREAM("state achieved");
                ros::spinOnce();ros::Duration(0.05).sleep();
                ros::Duration(0.1).sleep();
                
                GripperToggle(false);

                return -2;
                }
            

            return -1;
        }

        if(part_pose.position.y<2.83   &&   arm_id_=="arm1")
                    part_pose.position.y=2.83 ;
        if(part_pose.position.y>-2.83   &&   arm_id_=="arm2")
                    part_pose.position.y=-2.83 ;

        part_pose.position.z-=0.18;
        this->GoToTarget(part_pose);

        //Quality Control Sensor
        ros::spinOnce();ros::Duration(0.05).sleep();
        ros::Duration(0.1).sleep();
        // if (GetQualityControl1Status()==true)      //If true bad quality
        //     {return -1;

        //     }

        //ROS_INFO_STREAM("Actuating the gripper...");
        this->GripperToggle(false);
        this->GoToTarget(temp_pose);//So that arm goes up 
        ros::spinOnce();ros::Duration(0.05).sleep();


        if(GetQualityControl1Status()==true){
            ThrowFaultyProducts();
        }


        init_before_=1;
        ros::Duration(0.2).sleep();
        ros::spinOnce();
        //auto list_before=kit_tray_list_;
        ROS_INFO_STREAM("After list has these many components  "<<kit_tray_list_.size()); 
        init_before_=0;
        
        if (!gripper_state_) {
            ROS_INFO_STREAM("Going to home position...");
            this->SendRobotHome();
            //ros::Duration(1.0).sleep();
        }
            
    }
    // robot_move_group_.setJointValueTarget(home_joint_pose_);
    // this->Execute();

    ros::Duration(0.2).sleep();
    drop_flag_ = false;
    int temp=0;
    if (gripper_state_==1)
        {temp=1;}
    return temp;
}

void RobotController::GripperCallback(
        const osrf_gear::VacuumGripperState::ConstPtr& grip) {
    gripper_state_ = grip->attached;
}


bool RobotController::PickPart(geometry_msgs::Pose& part_pose) {
    // gripper_state = false;
    // pick = true;
    //ROS_INFO_STREAM("fixed_orientation_" << part_pose.orientation = fixed_orientation_);
    //ROS_WARN_STREAM("Picking the part...");

    ros::Duration(0.05).sleep();
    ros::spinOnce();
    ros::Duration(0.05).sleep();   

        
    ROS_INFO_STREAM("Moving to part...");
    part_pose.position.z = part_pose.position.z + offset_;
    auto temp_pose_1 = part_pose;
    temp_pose_1.position.z += 0.2;

    // this->GoToTarget({temp_pose_1, part_pose});

    //ROS_INFO_STREAM("Actuating the gripper..." << part_pose.position.z);
    this->GoToTarget(temp_pose_1);
    this->GripperToggle(true);

    ros::spinOnce();ros::Duration(0.05).sleep();
    int n=0;
    while (GetGripperStatus()==false) {
        part_pose.position.z -= 0.005;
        this->GripperToggle(true);
        this->GoToTarget(part_pose);
        ROS_INFO_STREAM("attempts     "<<n);
        n++;
        ros::spinOnce();
        this->GoToTarget(temp_pose_1);
        if(GetGripperStatus()==true){
            ROS_INFO_STREAM("Picked up: ");
            break;
        }
        ros::spinOnce();ros::Duration(0.05).sleep();
        if (n>5){
            ROS_WARN("To many attempts");
            break;
        }
    }

    //ROS_INFO_STREAM("Going to waypoint...");
    this->GoToTarget(temp_pose_1);
    return gripper_state_;
}
