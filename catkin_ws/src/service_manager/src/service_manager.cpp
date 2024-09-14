#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/String.h>
#include <service_manager/operationAction.h>
#include "define_positions.h"
#include <cmath>

/*--- Nota ----
    O service manager é cliente pois realiza pedidos, neste caso ao task manager que é servidor pois executa os pedidos
*/ 

// Vetores que definem as rotas a ser executadas
std::vector<float> route_dd; // rota de uma estante para outra
std::vector<float> route_dp2; // rota de uma estante para o park2 ( parque da estrutura)
std::vector<float> route_pd; // rota do park2 até a uma estante indicada pelo utilizador

class ServiceManager
{
public:
    ServiceManager() : ac_("operation", true) ,task_done_(true),success_(true)// Conecte ao action server do Task Manager
    {
        ROS_INFO("Esperando o action server iniciar.");
        ac_.waitForServer(); // Espera que o servidor seja inicializado 
        ROS_INFO("Action server iniciado.");
    }

    void sendTask(const std::string &task_name, std::vector<float> target, int lift,float orientation)
    {
        service_manager::operationGoal goal;
        goal.task_name = task_name;
        goal.target = target;
        goal.lift = lift;
        goal.orientation = orientation;

        ROS_INFO("Enviando objetivo ao action server.");
        task_done_=false;
        ac_.sendGoal(goal,
                     boost::bind(&ServiceManager::doneCb, this, _1, _2),
                     actionlib::SimpleActionClient<service_manager::operationAction>::SimpleActiveCallback(),
                     boost::bind(&ServiceManager::feedbackCb, this, _1));
    }

    void doneCb(const actionlib::SimpleClientGoalState &state, const service_manager::operationResultConstPtr &result)
    {
        //ROS_INFO("Finalizado em estado [%s]", state.toString().c_str());
        ROS_INFO("Resultado: %s", result->result_message.c_str());

        task_done_ = true;
        success_ = result->success;
    }

    void feedbackCb(const service_manager::operationFeedbackConstPtr &feedback)
    {
        ROS_INFO("Feedback: progresso %d, status %s", feedback->state, feedback->state_message.c_str());
    }

    // função que realiza o desenho das estantes no terminal para o utilizador escolher
    void desenhaCorredoresComEstantes() 
    {
         int contador = 1;

        for (int i = 0; i < 5; ++i) {
            for (int corredorIndex = 0; corredorIndex < 2; ++corredorIndex) {
                // Imprime a numeração antes do "|-----|"
                std::cout << std::setw(2) << contador++ << "|-----|" << std::setw(2) << contador++ << "   ";
            }
            std::cout << std::endl;
        }
    }
    bool isTaskDone() const
    {
        return task_done_;
    }
    /*
        As próximas três funções têm como objetivo definir rotas :

            1. Rota do parque da estrutura até á estante escolhida
            2. Rota de uma estante para outra
            3. Rota de uma estante até ao parque da estrutura
    */
    void DefineRouteParktoDock(float docking_number)
    {
        if (docking_number<=12 &&(docking_number ==1 || docking_number ==2 || docking_number ==5 || docking_number ==6 || docking_number ==9 || docking_number==10))
            route_pd = {MIDDLE_CORRIDOR_FRONT[0],MIDDLE_CORRIDOR_FRONT[1],LEFT_CORRIDOR_FRONT[0],LEFT_CORRIDOR_FRONT[1],
            POSITIONS[docking_number-1][0],POSITIONS[docking_number-1][1]};
        else if (docking_number<=12 )
            route_pd ={MIDDLE_CORRIDOR_FRONT[0],MIDDLE_CORRIDOR_FRONT[1],RIGHT_CORRIDOR_FRONT[0],RIGHT_CORRIDOR_FRONT[1],
            POSITIONS[docking_number-1][0],POSITIONS[docking_number-1][1]};
        else if (docking_number>12 && (docking_number ==13 || docking_number ==14 || docking_number ==17 || docking_number ==18 ))
            route_pd ={MIDDLE_CORRIDOR_FRONT[0],MIDDLE_CORRIDOR_FRONT[1],MIDDLE_CORRIDOR_BACK[0],MIDDLE_CORRIDOR_BACK[1],LEFT_CORRIDOR_BACK[0],LEFT_CORRIDOR_BACK[1],
            POSITIONS[docking_number-1][0],POSITIONS[docking_number-1][1]};
        else if(docking_number>12 )
            route_pd ={MIDDLE_CORRIDOR_FRONT[0],MIDDLE_CORRIDOR_FRONT[1],MIDDLE_CORRIDOR_BACK[0],MIDDLE_CORRIDOR_BACK[1],RIGHT_CORRIDOR_BACK[0],RIGHT_CORRIDOR_BACK[1],
            POSITIONS[docking_number-1][0],POSITIONS[docking_number-1][1]};
    }
    void DefineRouteDocktoDock(float docking_number, float last_docking_number)
    {
        if (POSITIONS[docking_number-1][0] == POSITIONS[last_docking_number-1][0])
            route_dd = {POSITIONS[docking_number-1][0],POSITIONS[docking_number-1][1]};
        else if ( POSITIONS[last_docking_number-1][1]<-2 && POSITIONS[docking_number-1][0]>0)
            route_dd = {MIDDLE_CORRIDOR_BACK[0],MIDDLE_CORRIDOR_BACK[1],RIGHT_CORRIDOR_BACK[0],RIGHT_CORRIDOR_BACK[1],POSITIONS[docking_number-1][0],POSITIONS[docking_number-1][1]};
        else if ( POSITIONS[last_docking_number-1][1]<-2 &&POSITIONS[docking_number-1][0]<0)
            route_dd = {MIDDLE_CORRIDOR_BACK[0],MIDDLE_CORRIDOR_BACK[1],LEFT_CORRIDOR_BACK[0],LEFT_CORRIDOR_BACK[1],POSITIONS[docking_number-1][0],POSITIONS[docking_number-1][1]};
        else if ( POSITIONS[last_docking_number-1][1]>-2 &&POSITIONS[docking_number-1][0]>0)
            route_dd = {MIDDLE_CORRIDOR_FRONT_DD[0],MIDDLE_CORRIDOR_FRONT_DD[1],RIGHT_CORRIDOR_FRONT_DD[0],RIGHT_CORRIDOR_FRONT_DD[1],POSITIONS[docking_number-1][0],POSITIONS[docking_number-1][1]};
        else if ( POSITIONS[last_docking_number-1][1]>-2 &&POSITIONS[docking_number-1][0]<0)
            route_dd = {MIDDLE_CORRIDOR_FRONT_DD[0],MIDDLE_CORRIDOR_FRONT_DD[1],LEFT_CORRIDOR_FRONT_DD[0],LEFT_CORRIDOR_FRONT_DD[1],POSITIONS[docking_number-1][0],POSITIONS[docking_number-1][1]};    
    
    }   
    void DefineRouteDocktoPark2(float docking_number)
    {
        if (docking_number<=12 &&(docking_number ==1 || docking_number ==2 || docking_number ==5 || docking_number ==6 || docking_number ==9 || docking_number==10))
            route_dp2 ={LEFT_CORRIDOR_FRONT[0],LEFT_CORRIDOR_FRONT[1],MIDDLE_CORRIDOR_FRONT[0],MIDDLE_CORRIDOR_FRONT[1],PARK2_POSITION[0],PARK2_POSITION[1]};
        else if (docking_number<=12 )
            route_dp2 ={RIGHT_CORRIDOR_FRONT[0],RIGHT_CORRIDOR_FRONT[1],MIDDLE_CORRIDOR_FRONT[0],MIDDLE_CORRIDOR_FRONT[1],PARK2_POSITION[0],PARK2_POSITION[1]};
        else if (docking_number>12 && (docking_number ==13 || docking_number ==14 || docking_number ==17 || docking_number ==18 ))
            route_dp2 ={LEFT_CORRIDOR_BACK[0],LEFT_CORRIDOR_BACK[1],MIDDLE_CORRIDOR_BACK[0],
            MIDDLE_CORRIDOR_BACK[1],MIDDLE_CORRIDOR_FRONT[0],MIDDLE_CORRIDOR_FRONT[1],PARK2_POSITION[0],PARK2_POSITION[1]};
        else if(docking_number>12 )
            route_dp2 ={RIGHT_CORRIDOR_BACK[0],RIGHT_CORRIDOR_BACK[1],MIDDLE_CORRIDOR_BACK[0],MIDDLE_CORRIDOR_BACK[1],MIDDLE_CORRIDOR_FRONT[0],
            MIDDLE_CORRIDOR_FRONT[1],PARK2_POSITION[0],PARK2_POSITION[1]};
    }
private:
    actionlib::SimpleActionClient<service_manager::operationAction> ac_;
    bool task_done_;
    bool success_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "service_manager_node");
    ServiceManager sm;

    int operation_number, operation_done =0, state =1,invalid_number=0,last_operation_number =1;;
    float docking_orientation=0.0,number_bookcase=1000.0, last_bookcase_number=1;

    std::cout<< " --------------------- Bem vindo ao Service Manager! ------------------------------"<<std::endl;
    std::cout<< "Escolhe a operação a ser executada: "<<std::endl;
    std::cout<< "1 - Navegação do parque do MiR para o parque da estrutura"<<std::endl;
    std::cout<< "2 - Navegação até á zona de reposição"<<std::endl;
    std::cout<< "3 - Navegação da zona de reposição até parque da estrutura"<<std::endl;
    std::cout<< "4 - Navegação do parque da estrutura até ao parque do MiR"<<std::endl;
    std::cout<< "Insere o número aqui - > ";

    std::cin >> operation_number;
    if (operation_number ==2) // se a operação for 2 é necessário escolher qual a estante 
    {
        do{
            std::cout<< "Qual a estante onde deve ser realizada a reposição: "<<std::endl;
            sm.desenhaCorredoresComEstantes();
            std::cout<< "Insere o número aqui o número da estante- > ";
            std::cin >> number_bookcase;
            if (number_bookcase>=1 &&number_bookcase<=20 ){ // verificar se o número da estante existe
                docking_orientation = (std::fmod(number_bookcase, 2.0) == 0) ? 0.0 : M_PI; // descobrir a orientação do mir para essa estante
                // executar rotas
                sm.DefineRouteParktoDock(number_bookcase);
                sm.DefineRouteDocktoDock(number_bookcase,last_bookcase_number);
                sm.DefineRouteDocktoPark2(number_bookcase);
            }
        }while(!(number_bookcase>=1 &&number_bookcase<=20) );
        
    }
    while(ros::ok()){
        if (sm.isTaskDone()&& operation_done==0) // se
        {
            switch (operation_number){
                case 1: 
                    sm.sendTask("Go Park2",ROUTE_PARK1_TO_PARK2 ,1,M_PI);
                    operation_done=1;
                    break;
                case 2: 
                    if (last_operation_number ==1)
                        sm.sendTask("Go Dock", route_pd,2,docking_orientation);
                    else 
                        sm.sendTask("Go Dock", route_dd,3,docking_orientation);
                    operation_done=1;
                    break;
                case 3: 
                    sm.sendTask("Go Park2", route_dp2,3,M_PI);
                    operation_done=1;
                    break;
                case 4: 
                    sm.sendTask("Go Park1", ROUTE_PARK2_TO_PARK1,0,0.0);
                    operation_done=1;
                    break;
                default: 
                    invalid_number =1;
                    std::cout<<"Número Inválido, insere novamente!"<<std::endl;
                    break;

            }
        }

            if  ((operation_done==1 && sm.isTaskDone()) || invalid_number==1){
                last_operation_number =operation_number;
                if(number_bookcase>=1 &&number_bookcase<=20) {last_bookcase_number = number_bookcase;}
                std::cout<< "Escolhe a próxima operação a ser executada: "<<std::endl;
                std::cout<< "1 - Navegação do park do MiR para o parque da estrutura"<<std::endl;
                std::cout<< "2 - Navegação até á zona de reposição"<<std::endl;
                std::cout<< "3 - Navegação da zona de reposição até parque da estrutura"<<std::endl;
                std::cout<< "4 - Navegação do parque da estrutura até ao parque do MiR"<<std::endl;
                std::cout<< "Insere o número aqui - > ";
                std::cin >> operation_number;   
                if (operation_number ==2)
                {
                    do{
                        std::cout<< "Qual a estante onde deve ser realizada a reposição: "<<std::endl;
                        sm.desenhaCorredoresComEstantes();
                        std::cout<< "Insere o número aqui o número da estante- > ";
                        std::cin >> number_bookcase;
                        if (number_bookcase>=1 &&number_bookcase<=20 ){
                            docking_orientation = (std::fmod(number_bookcase, 2.0) == 0) ? 0.0 : M_PI;
                            sm.DefineRouteParktoDock(number_bookcase);
                            sm.DefineRouteDocktoDock(number_bookcase,last_bookcase_number);
                            sm.DefineRouteDocktoPark2(number_bookcase);
                        }
                    }while(!(number_bookcase>=1 &&number_bookcase<=20) );
                }
                operation_done=0;
            }
    }
    
    

    ros::spin();
    return 0;
}