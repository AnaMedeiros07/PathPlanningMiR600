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
#include "matplotlibcpp.h"
#include <geometry_msgs/Vector3.h>
#include <actionlib/server/simple_action_server.h>
#include <task_manager/navigationAction.h>

namespace plt = matplotlibcpp;

class Navigation
{
    public:
        //Constructors
        Navigation(std::string name);

        //Destructors
        ~Navigation();
        
        // Methods
        void Target_Dynamic();
        void Obs_Dynamic();
        int Dynamic_Result();
        int getSectorSize();

        //Subscribers Callbacks
        void Sector_Callback(const sensor_msgs::LaserScan::ConstPtr& msg_sectors);
        void MiRPosition_Callback(const geometry_msgs::Vector3::ConstPtr& msg_position);
        void MirOrientation_Callback(const geometry_msgs::Vector3::ConstPtr& msg_orientation);
        void PlotTar();
        void executeCB(const task_manager::navigationGoalConstPtr &goal);

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
        ros::Subscriber TargetTopic; 
        ros::Subscriber SectorTopic;     

        actionlib::SimpleActionServer<task_manager::navigationAction> as_;
        std::string action_name_;
        task_manager::navigationFeedback feedback_;
        task_manager::navigationResult result_;

        ros::NodeHandle nh;

};

