#include <ros/ros.h>
#include <iostream>
#include <cstring>  // Para usar strcmp
#include <geometry_msgs/Vector3.h>
#include <std_msgs/String.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <task_manager/operationAction.h>
#include <task_manager/navigationAction.h>
#include <task_manager/parkdockAction.h>
#include <task_manager/navigationstructureAction.h>
#include "define_positions.h"


#define comprNav 1350.00/1000.00
#define comprNavStructure 1350.00/1000.00
#define largNav 910.00/1000.00
#define largNavStructure 1145.00/1000.00

#define NO_ERROR 0
#define ERROR_ALREADY_IN_PLACE 1
#define ERROR_NEEDS_TO_PARK_STRUCTURE_FIRST 2
#define ERROR_ALREADY_IN_PARK2 4
#define ERROR_NEEDS_TO_GO_TO_PARK2_FIRST 3


enum class State {
    IDLE,
    GO_DOCK,
    GO_PARK1,
    GO_PARK2,
    RETURN_DOCK,
    RETURN_PARK1,
    RETURN_PARK2,
    NAVIGATION,
    NAVIGATION_STRUCTURE,
    REPLISHMENT,
    ERROR
};

State current_state,next_state;
std::string last_goal;
class TaskManager
{
protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<task_manager::operationAction> as_;
    actionlib::SimpleActionClient<task_manager::navigationAction> ac_;  // Action Client
    actionlib::SimpleActionClient<task_manager::parkdockAction> ac_dp;  // Action Client
    actionlib::SimpleActionClient<task_manager::navigationstructureAction> ac_st;  // Action Client
    std::string action_name_;
    task_manager::operationFeedback feedback_;
    task_manager::operationResult result_;
    bool task_done_;
    bool success_;
    int status;
    int error_type;
    ros::Publisher pub_dimensions;
    geometry_msgs::Vector3 msg;

public:
    TaskManager(std::string name) :
    as_(nh_, name, boost::bind(&TaskManager::executeCB, this, _1), false),
    ac_("navigation", true),  // Initialize the Action Client
    ac_dp("parkdock", true),  // Initialize the Action Client
    ac_st("navigationstructure", true),  // Initialize the Action Client
    action_name_(name)
    {
        as_.start();
        last_goal = "Go Park1";
        ROS_INFO("Waiting for action navigation server to start.");
        ac_.waitForServer(); // Will wait for infinite time
        ROS_INFO("Action server navigation started.");
        ROS_INFO("Waiting for action parkdock server to start.");
        ac_dp.waitForServer(); // Will wait for infinite time
        ROS_INFO("Action server parkdock started.");
        ROS_INFO("Waiting for action navigation structure server to start.");
        ac_st.waitForServer(); // Will wait for infinite time
        ROS_INFO("Action server  navigation structure started.");
        pub_dimensions = nh_.advertise<geometry_msgs::Vector3>("/dimensions", 1);
        msg.x =comprNav;
        msg.y = largNav;
        task_done_ = true;
    }

~TaskManager(void)
{
}
void sendNavigationTask( std::vector<float> target)
{
    task_manager::navigationGoal goal;
    goal.target = target;

    ROS_INFO("Enviando objetivo ao action server.");
    task_done_=false;
    ac_.sendGoal(goal,
                    boost::bind(&TaskManager::doneCbNavigation, this, _1, _2),
                    actionlib::SimpleActionClient<task_manager::navigationAction>::SimpleActiveCallback(),
                    boost::bind(&TaskManager::feedbackCbNavigation, this, _1));
}
void sendParkDockTask( std::vector<float> target,float orientation, int type,int lift)
{
    task_manager::parkdockGoal goal;
    goal.target = target;
    goal.orientation = orientation;
    goal.type = type;
    goal.lift = lift;

    ROS_INFO("Enviando objetivo ao action server.");
    task_done_=false;
    std::cout<<"send"<<task_done_<<std::endl;
    ac_dp.sendGoal(goal,
                    boost::bind(&TaskManager::doneCbParkDock, this, _1, _2),
                    actionlib::SimpleActionClient<task_manager::parkdockAction>::SimpleActiveCallback(),
                    boost::bind(&TaskManager::feedbackCbParkDock, this, _1));
}

void sendNavigationStructureTask( std::vector<float> target)
{
    task_manager::navigationstructureGoal goal;
    goal.target = target;

    ROS_INFO("Enviando objetivo ao action server.");
    task_done_=false;
    ac_st.sendGoal(goal,
                    boost::bind(&TaskManager::doneCbNavigationStructure, this, _1, _2),
                    actionlib::SimpleActionClient<task_manager::navigationstructureAction>::SimpleActiveCallback(),
                    boost::bind(&TaskManager::feedbackCbNavigationStructure, this, _1));
}

void doneCbNavigation(const actionlib::SimpleClientGoalState &state, const task_manager::navigationResultConstPtr &result)
{
    //ROS_INFO("Finalizado em estado [%s]", state.toString().c_str());
    ROS_INFO("Resultado: %s", result->result_message.c_str());

    task_done_ = true;
    success_ = result->success;
}

void doneCbNavigationStructure(const actionlib::SimpleClientGoalState &state, const task_manager::navigationstructureResultConstPtr &result)
{
    //ROS_INFO("Finalizado em estado [%s]", state.toString().c_str());
    ROS_INFO("Resultado: %s", result->result_message.c_str());

    task_done_ = true;
    success_ = result->success;
}

void doneCbParkDock(const actionlib::SimpleClientGoalState &state, const task_manager::parkdockResultConstPtr &result)
{
    //ROS_INFO("Finalizado em estado [%s]", state.toString().c_str());
    ROS_INFO("Resultado: %s", result->result_message.c_str());

    task_done_ = true;
    success_ = result->success;
}
void feedbackCbNavigation(const task_manager::navigationFeedbackConstPtr &feedback)
{
    ROS_INFO("Feedback: vx %f, w %f , robot_x %f, robot_y %f e %s ", feedback->vx, feedback->w,feedback->position[0],feedback->position[1],feedback->state_message.c_str());
}

void feedbackCbNavigationStructure(const task_manager::navigationstructureFeedbackConstPtr &feedback)
{
    ROS_INFO("Feedback: vx %f, w %f e %s ", feedback->vx, feedback->w,feedback->state_message.c_str());
}

void feedbackCbParkDock(const task_manager::parkdockFeedbackConstPtr &feedback)
{
    ROS_INFO("Feedback: vx %f, w %f ,orientation %f e %s ", feedback->vx, feedback->w, feedback->orientation,feedback->state_message.c_str());
}
bool isTaskDone() const
{
    return task_done_;
}

void FindNextState(const task_manager::operationGoalConstPtr &goal)
{
    if (last_goal == "Go Park1" && strcmp(goal->task_name.c_str(),"Go Park1")!=0) {
        next_state = State::RETURN_PARK1;
    } else if (last_goal == "Go Park2" && strcmp(goal->task_name.c_str(),"Go Park2")!=0) {
        next_state = State::RETURN_PARK2;
    } else if (last_goal == "Go Dock" ) {
        next_state = State::RETURN_DOCK;
    } else {
        next_state = State::ERROR;
        error_type =ERROR_ALREADY_IN_PLACE;
    }
}
int ExecuteStateMachine(const task_manager::operationGoalConstPtr &goal)
{
    if (isTaskDone() && status ==0){ 
        switch (next_state)
        {
            case State::RETURN_DOCK:
                    current_state =State::RETURN_DOCK;
                    if(goal->lift==2 || goal->lift==3){
                        feedback_.state_message = "Em progresso estado return dock";
                        sendParkDockTask(goal->target,goal->orientation,5,goal->lift);
                        next_state = State::NAVIGATION_STRUCTURE;
                        msg.x =comprNavStructure;
                        msg.y = largNavStructure;
                    }
                    else {
                        error_type=ERROR_NEEDS_TO_PARK_STRUCTURE_FIRST; 
                        next_state = State::ERROR;
                    }                
                    break;
            case State::RETURN_PARK1:
                    current_state =State::RETURN_PARK1;
                    if(goal->lift==1 ){
                        sendParkDockTask(PARK1_POSITION,goal->orientation,1,goal->lift);
                        feedback_.state_message = "Em progresso estado return park1";
                        next_state = State::NAVIGATION;
                        msg.x =comprNav;
                        msg.y = largNav;
                    }
                    else{
                        error_type=ERROR_NEEDS_TO_GO_TO_PARK2_FIRST;
                        next_state = State::ERROR;
                    }
                    break;
            case State::RETURN_PARK2:
                    current_state =State::RETURN_PARK2;
                    sendParkDockTask(PARK2_POSITION,goal->orientation,3,goal->lift);
                    feedback_.state_message = "Em progresso estado return park2";
                    if (goal->lift ==0){
                        next_state = State::NAVIGATION;
                        msg.x =comprNav;
                        msg.y = largNav;
                    }
                    else if(goal->lift ==2) {
                        next_state= State::NAVIGATION_STRUCTURE;
                        msg.x =comprNavStructure;
                        msg.y = largNavStructure;
                    }
                    else{
                        error_type=ERROR_ALREADY_IN_PARK2;
                        next_state = State::ERROR;
                    }
                    break;
            case State::GO_PARK1:
                    current_state =State::GO_PARK1;
                    sendParkDockTask(PARK1_POSITION,goal->orientation,0,goal->lift);
                    feedback_.state_message = "Em progresso estado go park1";
                    status =1;
                    break;
            case State::GO_PARK2:
                    current_state =State::GO_PARK2;
                    sendParkDockTask(PARK2_POSITION,goal->orientation,2,goal->lift);
                    feedback_.state_message = "Em progresso estado go park2";
                    status =1;
                    break;
            case State::GO_DOCK:
                    current_state =State::GO_DOCK;
                    sendParkDockTask(goal->target,goal->orientation,4,goal->lift);
                    feedback_.state_message = "Em progresso estado go dock";
                    status =1;
                    break;
            case State::NAVIGATION:
                    current_state =State::NAVIGATION;
                    feedback_.state_message = "Em progresso estado navigation";
                    sendNavigationTask(goal->target);
                    if (strcmp(goal->task_name.c_str(),"Go Park1")==0)
                        next_state = State::GO_PARK1;
                    else if (strcmp(goal->task_name.c_str(),"Go Park2")==0)
                        next_state = State::GO_PARK2;
                    else{
                        error_type=ERROR_NEEDS_TO_PARK_STRUCTURE_FIRST;
                        next_state = State::ERROR;
                    }
                    break;
            case State::NAVIGATION_STRUCTURE:
                    current_state = State::NAVIGATION_STRUCTURE;
                    feedback_.state_message = "Em progresso estado navigation structure";
                    sendNavigationStructureTask(goal->target);
                    if (strcmp(goal->task_name.c_str(),"Go Park1")==0){
                        error_type=ERROR_NEEDS_TO_GO_TO_PARK2_FIRST;
                        next_state = State::ERROR;
                    }
                    else if (strcmp(goal->task_name.c_str(),"Go Park2")==0)
                        next_state = State::GO_PARK2;
                    else
                        next_state = State::GO_DOCK;
                    break;
            case State::ERROR:
                    current_state =State::ERROR;
                    feedback_.state_message = "Em progresso estado error";
                    task_done_ = true;
                    status=1;
                    break;
            case State::IDLE:
                    current_state = State::IDLE;
                    FindNextState(goal);
                    task_done_ = true;
                    feedback_.state_message = "Em progresso estado idle";
                    break;
            default: break;
        }
    }
    if (status ==1 && isTaskDone()==true){
        status=0;
        return 1;
    }
    return 0;
}
void PrintResultMessage(const task_manager::operationGoalConstPtr &goal)
{
    if (error_type ==NO_ERROR){
        result_.success = true;
        result_.result_message = "Tarefa executada pelo task_manager concluída com sucesso!";
        ROS_INFO("%s: Succeeded", action_name_.c_str());
        as_.setSucceeded(result_);

        last_goal = goal->task_name;
    }
    else if(error_type ==ERROR_NEEDS_TO_PARK_STRUCTURE_FIRST){
        result_.success = false;
        result_.result_message = "Não foi possível concluir a tarefa, o MiR precisa de primeiro ir buscar a estrutura antes de ir para a zona de reposição ou já estar numa zona de reposição antes de executar esta tarefa!";
        ROS_INFO("%s: Failed", action_name_.c_str());
        as_.setSucceeded(result_);
    }
    else if(error_type ==ERROR_NEEDS_TO_GO_TO_PARK2_FIRST){
        result_.success = false;
        result_.result_message = "Não foi possível concluir a tarefa, o MiR precisa de primeiro ir buscar a estrutura!";
        ROS_INFO("%s: Failed", action_name_.c_str());
        as_.setSucceeded(result_);
    }
    else if(error_type ==ERROR_ALREADY_IN_PARK2){
        result_.success = false;
        result_.result_message = "Não foi possível concluir a tarefa, o MiR  já se encontra no parque na estrutura!";
        ROS_INFO("%s: Failed", action_name_.c_str());
        as_.setSucceeded(result_);
    }
     else if(error_type ==ERROR_ALREADY_IN_PLACE){
        result_.success = false;
        result_.result_message = "Não foi possível concluir a tarefa, o MiR  já se encontra na posição indicada!";
        ROS_INFO("%s: Failed", action_name_.c_str());
        as_.setSucceeded(result_);
    }
}
void executeCB(const task_manager::operationGoalConstPtr &goal)
{
    ros::Rate r(1);
    bool success = true;
    //ROS_INFO("%s: Executando tarefa %s com target %f e lift %d ", action_name_.c_str(), goal->task_name.c_str(), goal->target, goal->lift);
    next_state =State::IDLE;
    error_type=NO_ERROR;
    status=0;
    while(ExecuteStateMachine(goal)==0)
    {
        as_.publishFeedback(feedback_);
        pub_dimensions.publish(msg);
        ExecuteStateMachine(goal);
        as_.publishFeedback(feedback_);
        pub_dimensions.publish(msg);
        r.sleep();
    }

    PrintResultMessage(goal);
}
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "task_manager_node");

    TaskManager task_manager("operation");
    ros::spin();

    return 0;
}
