#pragma once
#include "ros/ros.h"
#include "std_msgs/Bool.h"
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
#include <actionlib/server/simple_action_server.h>
#include <task_manager/navigationstructureAction.h>


class Navigation_Structure
{
    public:
        //Constructors
        Navigation_Structure(std::string name);

        //Destructors
        ~Navigation_Structure();
        
        // Methods
        void Target_Dynamic();
        void Obs_Dynamic();
        int Dynamic_Result();
        int getSectorSize();

        //Subscribers Callbacks
        void Sector_Callback(const sensor_msgs::LaserScan::ConstPtr& msg_sectors);
        void MiRPosition_Callback(const geometry_msgs::Vector3::ConstPtr& msg_position);
        void MirOrientation_Callback(const geometry_msgs::Vector3::ConstPtr& msg_orientation);
        void executeCB(const task_manager::navigationstructureGoalConstPtr &goal);

        double v_x;
        double w;
        double phi_tar;
        double ftar;
        double Fobs;
        std::vector<double> psi_obs;
        std::vector<double> lambda_obs;
        std::vector<double> sigma;
        std::vector<double> fobs;
        ros::Publisher pub_cmd_vel;


    private:

        // Robot Position in worlds
        float robot_x;
        float robot_y;
        float robot_phi;

        //Target Position
        float target_x;
        float target_y;
        float target_x_final;
        float target_y_final;

        //Sectors Variables
        std::vector<float> sector_ranges;
        std::vector<float> sector_intensities;

        // SNDL Variables
        float P;
        float f_total;
        float fstoch;
        float dist_min;
        float dist_minVel;
        float Max_dist_alvo;


        //Subscribers and Pubishers
        ros::Subscriber PositionTopic; 
        ros::Subscriber OrientationTopic; 
        ros::Subscriber SectorTopic;     

        ros::NodeHandle nh;

         actionlib::SimpleActionServer<task_manager::navigationstructureAction> as_;
        std::string action_name_;
        task_manager::navigationstructureFeedback feedback_;
        task_manager::navigationstructureResult result_;
};

