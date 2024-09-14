#pragma once
#include "ros/ros.h"
#include <std_msgs/Bool.h>
#include <vector>
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Float64.h"
#include <cmath>
#include <ctime>
#include <iostream>
#include <stdlib.h>
#include <random>
#include <iostream>
#include <algorithm>
#include "mir_custom_msgs/distances_mir.h"
#include <actionlib/server/simple_action_server.h>
#include <task_manager/parkdockAction.h>

class PrecisonTaks
{
    public:
        //Constructors
        PrecisonTaks(std::string name);

        //Destructors
        ~PrecisonTaks();
        
        // Methods
        int CorrectOrientation();
        void VelocityCommand();
        void Target_DynamicBack();
        void Target_DynamicFront();
        int  Dynamic_Result();
        int GotoPark(float final_x,float final_y);
        int ReturnFromPark();
        int GoDock();
        int ReturnFromDock();
        int CorrectOrientationStructure();
        void executeCB(const task_manager::parkdockGoalConstPtr &goal);
 
        //Subscribers Callbacks
        void Sector_Callback(const sensor_msgs::LaserScan::ConstPtr& msg_sectors);
        void MiRPosition_Callback(const geometry_msgs::Vector3::ConstPtr& msg_position);
        void MirOrientation_Callback(const geometry_msgs::Vector3::ConstPtr& msg_orientation);
        void Target_Callback(const geometry_msgs::Vector3::ConstPtr& msg_target);
        void Distances_Callback(const mir_custom_msgs::distances_mir::ConstPtr& msg_distances);
        void Structure_Callback(const geometry_msgs::Vector3::ConstPtr& msg_structure);
        void LiftState_Callback(const std_msgs::Float64::ConstPtr& msg_lift_state);

        double v_x;
        double w;
        bool lift;
        bool clear_to_start;
        std_msgs::Bool msg;

        ros::Publisher pub_cmd_vel;
        ros::Publisher pub_lift;

    private:

        // Robot Position in worlds
        float robot_x;
        float robot_y;
        float robot_phi;

        float target_x;
        float target_y;

        float Orides;

        float structure_x;
        float structure_y;

        float lift_state;

        //SDNL variables
        float ftar;
        int StopFront;
        int StopRear;
        int dist_min_front;
        int dist_min_rear;

        //Sectors Variables
        std::vector<float> sector_ranges;
        std::vector<float> sector_intensities;

        //Distances Variavels
        std::vector<float> dist_front_obs;
        std::vector<float> dist_rear_obs;
        std::vector<float> dist_right_obs;
        std::vector<float> dist_left_obs;

        //Subscribers and Pubishers
        ros::Subscriber PositionTopic; 
        ros::Subscriber OrientationTopic; 
        ros::Subscriber SectorTopic;   
        ros::Subscriber DistancesTopic;   
        ros::Subscriber StructureTopic; 
        ros::Subscriber LiftTopic; 

        actionlib::SimpleActionServer<task_manager::parkdockAction> as_;
        std::string action_name_;
        task_manager::parkdockFeedback feedback_;
        task_manager::parkdockResult result_;

        ros::NodeHandle nh;   

};

