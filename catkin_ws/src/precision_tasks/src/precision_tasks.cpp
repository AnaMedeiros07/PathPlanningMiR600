#include "precision_tasks.h"
#include "define_positions.h"

#define N_SECTORS 21
#define longitudinal_points 21
#define KparkOri 1.0
#define KparkVel 0.2
#define lambda_tar 0.4

enum class State {
    STATE_IDLE,
    STATE_GOTOPARK,
    STATE_RETURNFROMPARK,
    STATE_GOTODOCK,
    STATE_RETURNFROMDOCK,

};
enum class StateGoPark {

    STATE_CORRECT_ORIENTATION1,
    STATE_CORRECT_ORIENTATION2,
    STATE_CORRECT_ORIENTATION3,
    STATE_CORRECT_POSITION,
    STATE_GO_POSE,

};
enum class StateGoDock {

    STATE_CORRECT_ORIENTATION,
    STATE_CORRECT_POSITION,
    STATE_GO_POSE,

};

State current_state,last_state;
StateGoPark current_state_park;
StateGoDock current_state_dock;

PrecisonTaks::PrecisonTaks(std::string name) :
        as_(nh, name, boost::bind(&PrecisonTaks::executeCB, this, _1), false),
        action_name_(name)
{
    as_.start();

    //Init Subscribers
    PositionTopic = nh.subscribe("/position_topic",100,&PrecisonTaks::MiRPosition_Callback,this); 
    SectorTopic = nh.subscribe("/sector_dist_obs",100,&PrecisonTaks::Sector_Callback,this); 
    OrientationTopic = nh.subscribe("/orientation_topic",100,&PrecisonTaks::MirOrientation_Callback,this); 
    DistancesTopic = nh.subscribe("/distances_obs",100,&PrecisonTaks::Distances_Callback,this); 
    StructureTopic =nh.subscribe("/Structure_pose",100,&PrecisonTaks::Structure_Callback,this); 
    LiftTopic =nh.subscribe("/lift_state",100,&PrecisonTaks::LiftState_Callback,this); 

    //Init Publishers
    pub_cmd_vel = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    pub_lift = nh.advertise<std_msgs::Bool>("/lift", 1);

    sector_intensities.resize(N_SECTORS, 0.0);
    sector_ranges.resize(N_SECTORS, 0.0);
    dist_front_obs.resize(longitudinal_points, 5.0);
    dist_rear_obs.resize(longitudinal_points, 5.0);


    v_x =0.0;
    w=0.0;
    lift =false;
    StopFront=1;
    StopRear=1;
    clear_to_start=false;
    current_state_park = StateGoPark::STATE_GO_POSE;
    current_state_dock = StateGoDock::STATE_CORRECT_ORIENTATION;
}
PrecisonTaks::~PrecisonTaks()
{

}
int PrecisonTaks::CorrectOrientation()
{
    // Calcular o erro angular
    double phi_erro = Orides - robot_phi;

    // Normalizar o erro para o intervalo [-M_PI, M_PI]
    while (phi_erro > M_PI)
        phi_erro -= 2 * M_PI;
    while (phi_erro < -M_PI)
        phi_erro += 2 * M_PI;

    // Aplicar a correção de velocidade angular
    w = KparkOri * sin(phi_erro);

    // Checar se o erro angular está dentro da tolerância
    if (fabs(phi_erro) < 0.01)
        return 1;

    return 0;
}

void PrecisonTaks::VelocityCommand()
{
    
    float dist_erro =sqrt(pow((target_y-robot_y),2)+pow((target_x-robot_x),2)); //distância euclediana
    float KparkVelR =-0.4/pow(dist_min_front,2);

    if (current_state == State::STATE_GOTOPARK)
        v_x= -KparkVel*(dist_erro/100.0)*StopRear;
    else if (current_state==State::STATE_RETURNFROMPARK)
    {
        v_x= 0.2*StopFront;
    }

}

void PrecisonTaks::Target_DynamicBack()
{
    //phi_tar=atan2(target_y-robot_y,target_x-robot_x);
    float phi_tar=atan2(target_y-robot_y,target_x-robot_x);
    ftar = -(lambda_tar)*sin(robot_phi-phi_tar+M_PI);

}

void PrecisonTaks::Target_DynamicFront()
{
    //phi_tar=atan2(target_y-robot_y,target_x-robot_x);
    float phi_tar=atan2(target_y-robot_y,target_x-robot_x);
    ftar = -(lambda_tar)*sin(robot_phi-phi_tar);

}
int PrecisonTaks::Dynamic_Result()
{
    int goal_reached=0;
    double distance = sqrt(pow(target_y-robot_y,2)+pow(target_x-robot_x,2));
    std::cout<<"distance ="<<distance<<std::endl;
  
    w = ftar;

    if(( (distance) <4 &&  current_state_park == StateGoPark::STATE_CORRECT_POSITION) || (current_state == State::STATE_GOTODOCK && distance <2 )|| (current_state == State::STATE_RETURNFROMPARK && distance <20 ))
    {
        goal_reached=1;
        w=0.0;
    }

    return goal_reached;
}

int PrecisonTaks::GotoPark(float final_x,float final_y){

   int goal,orientation_result,state;
   double distance;
   target_y=final_y;
   switch (current_state_park)
    {
        case StateGoPark::STATE_GO_POSE:
            if (final_x <0) {target_x = -4.8*100.0;v_x= 0.3;}else{target_x = 4.8*100.0;v_x= 0.2; }
            distance = sqrt(pow(target_y-robot_y,2)+pow(target_x-robot_x,2));
            std::cout<<"distance ="<<distance<<std::endl;
            PrecisonTaks::Target_DynamicFront();
            w = ftar;
            if (distance<3){
                v_x=0.0;
                w=0.0;
                current_state_park = StateGoPark::STATE_CORRECT_ORIENTATION1;
            }
        break;
        case StateGoPark::STATE_CORRECT_ORIENTATION1:
            orientation_result =PrecisonTaks::CorrectOrientation();
            if (orientation_result==1){
                current_state_park = StateGoPark::STATE_CORRECT_POSITION;
                w=0.0;
            }
        break;
        case StateGoPark::STATE_CORRECT_POSITION:
            target_x = final_x+0.085*100.0;
            PrecisonTaks::Target_DynamicBack();
            distance = sqrt(pow(target_y-robot_y,2)+pow(target_x-robot_x,2));
            std::cout<<"target_x ="<<target_x<<"target_y"<<target_y<<std::endl;
            if(final_x <0 ){ v_x=-0.3;}else{ v_x=-0.10;}
            w = ftar;
            std::cout<<"distance="<<distance <<std::endl;
            if (distance <2){
                v_x=0.0;
                w=0.0;
                current_state_park = StateGoPark::STATE_CORRECT_ORIENTATION2;
               
            }
        break;
           case StateGoPark::STATE_CORRECT_ORIENTATION2:
            orientation_result =PrecisonTaks::CorrectOrientation();
            if (orientation_result==1){
                current_state_park = StateGoPark::STATE_GO_POSE;
                w=0.0;
                return 1;
            }
        break;
        default:break;
    }
    return 0;

}
int PrecisonTaks::ReturnFromPark()
{
    int goal;

    goal = PrecisonTaks::Dynamic_Result();
    v_x =0.3;
    if (goal ==1 ) 
    {
        std::cout<<"Goal Return from Park"<<std::endl;
        v_x=0.0;
        w=0.0;
        return 1;
        
    }
    w=0.0;
    
    return 0;
}
int PrecisonTaks::GoDock()
{
   int goal,orientation_result;

   switch (current_state_dock)
    {
        case StateGoDock::STATE_CORRECT_ORIENTATION:
            orientation_result =PrecisonTaks::CorrectOrientation();
            if (orientation_result==1)
                current_state_dock = StateGoDock::STATE_GO_POSE;
        break;
        case StateGoDock::STATE_GO_POSE:
            //PrecisonTaks::Target_Dynamic();
            //goal = PrecisonTaks::Dynamic_Result();
            goal=1;
            if (goal ==1){
                v_x=0.0;
                w=0.0;
                current_state_dock = StateGoDock::STATE_CORRECT_ORIENTATION;
                return 1;
            }
        break;
        default:break;
    }


    return 0;
}
int PrecisonTaks::ReturnFromDock()
{
    int orientation_result =PrecisonTaks::CorrectOrientation();
    
    return orientation_result;
}
void PrecisonTaks::Sector_Callback(const sensor_msgs::LaserScan::ConstPtr& msg_sectors)
{
    for (int i=0;i<N_SECTORS;i++)
    {
        sector_ranges.at(i)=(msg_sectors->ranges[i]*100);
        sector_intensities.at(i)=(msg_sectors->intensities[i]);
        //std::cout<<"Sector ranges "<<i<<" = "<<msg_sectors->ranges[i]*100<<", Sector Intensities ="<<msg_sectors->intensities[i]<<std::endl;
    }
}
void PrecisonTaks::MiRPosition_Callback(const geometry_msgs::Vector3::ConstPtr& msg_position)
{
    robot_x = msg_position->x*100;
    robot_y = msg_position->y*100;
    clear_to_start = true;
    //std::cout << "Robot x="<< robot_x<<"Robot_y"<<robot_y<<std::endl;
}

void PrecisonTaks::Structure_Callback(const geometry_msgs::Vector3::ConstPtr& msg_structure)
{
    structure_x = msg_structure->x*100;
    structure_y = msg_structure->y*100;
   
}

void PrecisonTaks::MirOrientation_Callback(const geometry_msgs::Vector3::ConstPtr& msg_orientation)
{
    robot_phi = msg_orientation->z;
     //std::cout << "Robot_phi= "<< robot_phi<<std::endl;
}
void PrecisonTaks::LiftState_Callback(const std_msgs::Float64::ConstPtr& msg_lift_state)
{
    lift_state = msg_lift_state->data;
     //std::cout << "Robot_phi= "<< robot_phi<<std::endl;
}
void PrecisonTaks::Distances_Callback(const mir_custom_msgs::distances_mir::ConstPtr& msg_distances)
{
    dist_min_rear =100.0;
    dist_min_front =100.0;
    for(int i =3; i<longitudinal_points-3; i++)
    {
        dist_front_obs.at(i) = msg_distances->dist_front[i]*100.0;
        dist_rear_obs.at(i) = msg_distances->dist_rear[i]*100.0;
        if (dist_front_obs[i]<dist_min_front )
            dist_min_front=dist_front_obs[i];
        if (dist_rear_obs[i]<dist_min_rear )
            dist_min_rear=dist_rear_obs[i];

    }
    
    StopFront = (dist_min_front<=10.0)? 0:1;
    StopRear = (dist_min_rear<=10.0)? 0:1;


}
void PrecisonTaks::executeCB(const task_manager::parkdockGoalConstPtr &goal)
{   
    int goal_reached =0;
    geometry_msgs::Twist cmd_vel;
    ros::Rate r(1);
    std::vector<float> target = goal->target;
    Orides = goal->orientation;
    if (PrecisonTaks::clear_to_start)
    {
        if (goal->lift==3 && goal->type==5 ){while(lift_state<0.058){msg.data =true;pub_lift.publish(msg);}}
        else if(goal->lift==0 && goal->type ==3){while(lift_state>0.0001){msg.data =false;pub_lift.publish(msg); }}
       
        while(goal_reached==0)
        {
            switch(goal->type)
            {
                case 0:
                    feedback_.state_message="Go to Park Park 1 executing!";
                    current_state = State::STATE_GOTOPARK;
                    goal_reached=PrecisonTaks::GotoPark(PARK1_POSITION[0]*100.0-2.6*100.0,PARK1_POSITION[1]*100.0);
                    break;
                case 1:
                    target_x = PARK1_POSITION[0]*100.0-50.0;
                    target_y = PARK1_POSITION[1]*100.0;
                    current_state = State::STATE_RETURNFROMPARK;
                    feedback_.state_message="Return From Park 1 executing!";
                    goal_reached=PrecisonTaks::ReturnFromPark();
                    break;
                case 2:
                    if(goal->lift==1){
                        goal_reached=PrecisonTaks::GotoPark(structure_x,structure_y);
                    }
                    else{
                        goal_reached=PrecisonTaks::GotoPark(PARK2_POSITION[0]*100.0+2.4*100.0,PARK2_POSITION[1]*100.0);
                    }
                    feedback_.state_message="Go to Park Park 2 executing!";
                    current_state = State::STATE_GOTOPARK;
                    if(goal_reached==1){if(goal->lift==0){msg.data=false;}else if(goal->lift==1){msg.data=true;}}
                    break;
                case 3:
                    target_x = PARK2_POSITION[0]*100.0+50.0;
                    target_y = PARK2_POSITION[1]*100.0;
                    current_state = State::STATE_RETURNFROMPARK;
                    feedback_.state_message="Return From Park 2 executing!";
                    goal_reached=PrecisonTaks::ReturnFromPark();
                    break;
                case 4:
                    if(Orides==0.0){target_x = target[target.size()-2]*100.0+20.0;}else{target_x = target[target.size()-2]*100.0-20.0;}
                    target_y = target[target.size()-1]*100.0;
                    current_state = State::STATE_GOTODOCK;
                    feedback_.state_message="Go to dock executing!";
                    goal_reached=PrecisonTaks::GoDock();
                    if(goal_reached==1){if(goal->lift==2){msg.data=false;}else if(goal->lift==3){msg.data=true;}}
                    break;
                case 5: 
                    Orides = (target[1]*100.0>robot_y)? M_PI/2.0 : -M_PI/2.0;
                    std::cout<<"target[1] = "<<target[1]<<std::endl;
                    std::cout<<"robot_y = "<<robot_y<<std::endl;
                    std::cout<<"Orides = "<<Orides<<std::endl;
                    current_state = State::STATE_RETURNFROMDOCK;
                    feedback_.state_message="Return from dock executing!";
                    goal_reached =PrecisonTaks::ReturnFromDock();
                    break;
                default: break;
                
            }
            ROS_INFO("Valor de msg.data: %s", msg.data ? "true" : "false");
            cmd_vel.linear.x =v_x;
            cmd_vel.angular.z = w;
            pub_cmd_vel.publish(cmd_vel);
            feedback_.orientation =robot_phi;
            feedback_.vx =v_x;
            feedback_.w =w;
            as_.publishFeedback(feedback_);
            r.sleep();
        }
    }
    pub_lift.publish(msg);
    result_.success = true;
    result_.result_message = "Tarefa concluída com sucesso!";
    ROS_INFO("%s: Succeeded", action_name_.c_str());
    as_.setSucceeded(result_);
}
int main(int argc, char **argv)
{

    ros::init(argc, argv, "precision_node");
    ros::NodeHandle n_priv("~");

    geometry_msgs::Twist cmd_vel;

    int start=0;
    PrecisonTaks *localPrecision;
    localPrecision = new PrecisonTaks("parkdock"); //cria objeto

    ros::Rate rate(200); //20Hz (50ms)      //80Hz (12,5ms)

    while(ros::ok())
    {
       

        ros::spinOnce();
	    rate.sleep();
    }
  
    

    return 0;
}