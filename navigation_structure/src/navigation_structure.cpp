#include "navigation_structure.h"


#define Q 0.01
#define TIMESTEP 0.05 // CoppeliaSim dt
#define TAU_TAR 50.0*TIMESTEP
#define TAU_OBS 5*TIMESTEP
#define BETA1 4
#define BETA2 160
#define N_SECTORS 21
#define TOO_FAR 200
#define C 10
#define CvALVO 4
#define CvOBS 35 // x10 beta 1
#define lambda_tar 0.4


Navigation_Structure::Navigation_Structure(std::string name) :
        as_(nh, name, boost::bind(&Navigation_Structure::executeCB, this, _1), false),
        action_name_(name)
{
    as_.start();

    //Init Subscribers
    PositionTopic = nh.subscribe("/position_topic",100,&Navigation_Structure::MiRPosition_Callback,this); 
    SectorTopic = nh.subscribe("/sector_dist_obs",100,&Navigation_Structure::Sector_Callback,this); 
    OrientationTopic = nh.subscribe("/orientation_topic",100,&Navigation_Structure::MirOrientation_Callback,this);  

    //Init Publishers
    pub_cmd_vel = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    // inicialização de vetores e variáveis auxiliares
    sector_intensities.resize(N_SECTORS, 0.0);
    sector_ranges.resize(N_SECTORS, 0.0);
    psi_obs.resize(N_SECTORS, 0.0);
    lambda_obs.resize(N_SECTORS, 0.0);
    sigma.resize(N_SECTORS, 0.0);
    fobs.resize(N_SECTORS, 0.0);

    Max_dist_alvo =1.0;
}
Navigation_Structure::~Navigation_Structure()
{

}

void Navigation_Structure::Target_Dynamic()
{

    phi_tar=atan2(target_y-robot_y,target_x-robot_x);
    ftar = -(lambda_tar)*sin(robot_phi-phi_tar);

    // Create a random number generator and a uniform distribution
    std::random_device rd; // Seed generator
    std::mt19937 gen(rd()); // Mersenne Twister generator
    std::uniform_real_distribution<> dis(0.0, 1.0); // Uniform distribution between -1 and 1

    double random_int = dis(gen);// Generate random number in the range [-1, 1]
    fstoch = std::sqrt(Q) * random_int;

}
void Navigation_Structure::Obs_Dynamic()
{

    int N = Navigation_Structure::getSectorSize();
    dist_min =1000.0;
    dist_minVel=1000.0;
    float too_far =200.0;
    Fobs=0.0;
    P=0.0;
    float beta1 =BETA1;
    float beta2 =BETA2;

    float Dtheta = sector_intensities[2]-sector_intensities[1];
    
    for (int i =0; i<N;i++)
    {   
        if (sector_ranges[i]<dist_min &&  sector_ranges[i]>10.0)
        {
            dist_min = sector_ranges[i];
            if ( i>6 && i<16) // só se considera os 
            {
                dist_minVel=sector_ranges[i];
            }
            
        }
        if (sector_ranges[i]>=too_far &&  sector_ranges[i]>10.0)
            lambda_obs.at(i)=0;
        else 
            lambda_obs.at(i)=beta1*exp(-(sector_ranges[i]/beta2));
        
        //compute repeller value
        psi_obs.at(i)=robot_phi+sector_intensities[i];
        
        //compute range of repulsion
        sigma.at(i)= atan(tan(Dtheta/2))+((115.0/2)/((140.0/2)+sector_ranges[i]));

        //compute fobsi
        fobs.at(i)=lambda_obs[i]*(robot_phi-psi_obs[i])*exp(-(pow(robot_phi-psi_obs[i],2))/(2*pow(sigma[i],2)));
        //std::cout<<"fobs" <<i <<" = "<<fobs[i]<<std::endl;

        Fobs = Fobs + fobs[i];
      
        P = P + (lambda_obs[i])*pow(sigma[i],2)*exp((-pow((robot_phi-psi_obs[i]),2))/(2*pow(sigma[i],2))) -(lambda_obs[i]*pow(sigma[i],2))/sqrt(exp(1));
        
    }
    

}
int Navigation_Structure::Dynamic_Result()
{
    int goal_reached=0;
    double Valvo;
    double distance = sqrt(pow(target_y_final-robot_y,2)+pow(target_x_final-robot_x,2));
    double distance_to_waypoint = sqrt(pow(target_y-robot_y,2)+pow(target_x-robot_x,2));
    std::cout<<"distance to final target"<<distance<<std::endl;
    
    if ((distance > Max_dist_alvo) )
    {
        Max_dist_alvo=distance;
        //Valvo = (distance/Max_dist_alvo)*0.6;
        //std::cout<<"Valvo1="<<Valvo<<std::endl;
    }
    if(distance<200.0)
    {
        Valvo = (distance/Max_dist_alvo)*0.8+0.1;
    }
    else{
        Valvo=0.8;
    }
    double alpha = atan(C*P)/M_PI; 
    double Cobs = CvOBS*(1/2.0 + alpha);
    double Calvo = CvALVO*(1/2.0 - alpha);
    double Vobs = ((dist_minVel-100.0)/1000.0)*0.8;
    //double Valvo = (distance/Max_dist_alvo)*0.8;
    double acc=0.0;

    std::cout<<"dist_min="<<dist_min<<std::endl;
   
    
    if (alpha <=0 && dist_min>40)
    {
        f_total = ftar+fstoch;
        acc= -Calvo*(v_x-Valvo);
        v_x = v_x + acc*0.005;

    }
    else
    {
        f_total =ftar +fstoch+Fobs;
        acc=-Cobs*(v_x-Vobs);
        v_x = v_x + acc*0.005;
    }
    
    w = f_total;
    std::cout<<"distance to waypoint"<<distance_to_waypoint<<std::endl;
    if ((distance_to_waypoint<50 && distance_to_waypoint !=distance) || (distance_to_waypoint ==distance && distance <30))
    {
        //v_x=0.0;
       // w=0.0;
        goal_reached=1;
        //Max_dist_alvo=1.0;
    }
    else
    {
        goal_reached=0;
    }
    //std::cout<< "Valvo = "<<Valvo<<"  Vobs = "<<Vobs<<"Cobs = "<<Cobs<<" Calvo= "<<Calvo<<"v_x ="<<v_x<<std::endl;
    return goal_reached;
}

int Navigation_Structure::getSectorSize()
{
    return sector_ranges.size();
}
// Subscribers Callbacks
void Navigation_Structure::Sector_Callback(const sensor_msgs::LaserScan::ConstPtr& msg_sectors){
    
    for (int i=0;i<N_SECTORS;i++)
    {
        sector_ranges.at(i)=(msg_sectors->ranges[i]*100);
        sector_intensities.at(i)=(msg_sectors->intensities[i]);
        //std::cout<<"Sector ranges "<<i<<" = "<<msg_sectors->ranges[i]*100<<", Sector Intensities ="<<msg_sectors->intensities[i]<<std::endl;
    }
}
void Navigation_Structure::MiRPosition_Callback(const geometry_msgs::Vector3::ConstPtr& msg_position)
{
    robot_x = msg_position->x*100;
    robot_y = msg_position->y*100;
    //std::cout << "Robot x="<< robot_x<<"Robot_y"<<robot_y<<std::endl;

}
void Navigation_Structure::MirOrientation_Callback(const geometry_msgs::Vector3::ConstPtr& msg_orientation)
{
     robot_phi = msg_orientation->z;
     //std::cout << "Robot_phi= "<< robot_phi<<std::endl;
}
void Navigation_Structure::executeCB(const task_manager::navigationstructureGoalConstPtr &goal)
{   
    int goal_reached =0;
    geometry_msgs::Twist cmd_vel;
    std::vector<float> position={robot_x,robot_y};
    ros::Rate r(1);
    std::vector<float> target = goal->target;
    int number_of_waypoins = goal->target.size(),count =1;
    double distance;
    target_x_final = target[number_of_waypoins-2]*100.0;
    target_y_final = target[number_of_waypoins-1]*100.0;
    v_x=0.2;
    if (Navigation_Structure::getSectorSize()>0)
    {
        while(count<=number_of_waypoins){
            target_x =target[count-1]*100.0;
            target_y =target[count]*100.0;
            distance = sqrt(pow(target_y-robot_y,2)+pow(target_x-robot_x,2));
           
            Navigation_Structure::Target_Dynamic();
            
            Navigation_Structure::Obs_Dynamic();
            goal_reached = Navigation_Structure::Dynamic_Result();

            cmd_vel.linear.x =v_x;
            cmd_vel.angular.z = w;
            pub_cmd_vel.publish(cmd_vel);
            feedback_.vx=v_x;
            feedback_.w=w;
            position[0]= robot_x/100.0;
            position[1]= robot_y/100.0;
            feedback_.position=position;
            feedback_.state_message ="Excetuting waypoint number";
            as_.publishFeedback(feedback_);
            
          
            r.sleep();
            if (goal_reached==1 ){
                count=count+2;
                v_x =0.2;
            }
        }
        Max_dist_alvo=1.0;
        v_x=0.0;
        w=0.0;
        cmd_vel.linear.x =v_x;
        cmd_vel.angular.z = w;
        pub_cmd_vel.publish(cmd_vel);
    }
    
    result_.success = true;
    result_.result_message = "Tarefa concluída com sucesso!";
    ROS_INFO("%s: Succeeded", action_name_.c_str());
    as_.setSucceeded(result_);
}
int main(int argc, char **argv)
{

    ros::init(argc, argv, "navigation_structure_node");
    ros::NodeHandle n_priv("~");

    int goal =0;
    geometry_msgs::Twist cmd_vel;


    Navigation_Structure *localNavigation;
    localNavigation = new Navigation_Structure("navigationstructure"); //cria objeto

    ros::Rate rate(200); //20Hz (50ms)      //80Hz (12,5ms)

    while(ros::ok())
    { 

        ros::spinOnce();
	    rate.sleep();
    }
  
    

    return 0;
}