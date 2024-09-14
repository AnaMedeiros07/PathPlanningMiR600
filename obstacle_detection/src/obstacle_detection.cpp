#include "obstacle_detection.h"
#include "mir_custom_msgs/distances_mir.h"


//obstacle_detection_class::obstacle_detection_class(){ //construtor
obstacle_detection_class::obstacle_detection_class():tfListener_rear(tfBuffer_rear),tfListener(tfBuffer){

    ros::NodeHandle nh;   //no construtor, ter o handler
    
    //subscrever aos topicos que recebe os dados do Lidar (o Coppelia esta a publicar!)
       sick_front_sub = nh.subscribe("lidar_front_data",100,&obstacle_detection_class::coppelia_sick_front_Callback,this); //
       sick_rear_sub = nh.subscribe("lidar_rear_data",100,&obstacle_detection_class::coppelia_sick_rear_Callback,this);
       dimensions_sub = nh.subscribe("/dimensions",100,&obstacle_detection_class::dimensions_Callback,this);

        compr = 1350.00/1000.00;
        larg =1145.00/1000.00;
        Structure_on=true;
    //inicializar variaveis auxiliares para tratamento dados dos Lidar recebidos do Coppelia:
        //para sensor frente:
            front_lidar_data.assign(684,0.0); //coppelia SickS300
           //front_lidar_data.assign(1651,0.0);   //nanoScan3 Mir real
           // distance_front.assign(684,0.0);
           // angle_front.assign(684,0.0);
           x_robot_frame_front.assign(684,0.0);
           y_robot_frame_front.assign(684,0.0);
           // incre_sensor_front.assign(684,0.0);
           front_seq_receive=0;

            sick_front_data_in.header.frame_id="sick_front";      //id Coppelia
            //sick_front_data_in.header.frame_id="front_laser_link";      //id Mir

        //para sensor tras:
            rear_lidar_data.assign(684,0.0); //coppelia SickS300
            //rear_lidar_data.assign(1651,0.0); //numero de feixes do Lidar nanoScan3
           // distance_rear.assign(684,0.0);
           // angle_rear.assign(684,0.0);
           x_robot_frame_rear.assign(684,0.0);
           y_robot_frame_rear.assign(684,0.0);
           // incre_sensor_rear.assign(684,0.0);
            rear_seq_receive=0;

            sick_rear_data_in.header.frame_id="sick_rear";        //id Coppelia
            //sick_rear_data_in.header.frame_id="back_laser_link";      //id Mir

            //tfListener_rear(tfBuffer_rear){}; //Nao se faz isto em class's...

        //distancias aos obstaculos (apenas setores de interesse aos SDNL):
            dist_obs_setor.assign(n_setores,30.0); //todos os setores a "detetar" 30m (no inicio)

    //inicializar variaveis auxiliares:
        clear_to_start_f=false;   //so iniciar o processo caso a Callback seja executada 1x
        clear_to_start_b=false; 
        //para converter em setores:
            theta_obs.assign(n_setores,0.0);
            x_setor_final.assign(n_setores,0.0);
            y_setor_final.assign(n_setores,0.0);
        //ter distancia do corpo do robo a descontar na distancia medida em cada setor dinamico
            robot_dist.assign(n_setores,0.0);

        //variavel publicacao:
            pub=true;// publicar os obstaculos para o modulo de  (true==quero publicar)

    //variaveis para fazer o "sensor de estacionamento": (15/03)
            //distancias:
                dist_front_obs.assign(longitudinal_points,5.00); //by default is 5 m
                dist_rear_obs.assign(longitudinal_points,5.00);
                dist_right_obs.assign(lateral_points,5.00);
                dist_left_obs.assign(lateral_points,5.00);
       
            //referencias para calculo:
                ref_lateral_points.assign(lateral_points,0.00); 
                ref_longitudinal_points.assign(longitudinal_points,0.00);
                distance_between_points_longitudinal= larg/(longitudinal_points-1); //frente ou tras
                distance_between_points_lateral= compr/(lateral_points-1);          //laterais (dir ou esq)
                min_x_incre=distance_between_points_lateral/2.0;
                min_y_incre=distance_between_points_longitudinal/2.0;

                for(int aux=0;aux<longitudinal_points;aux++){ //para frente e tras do veiculo
                    ref_longitudinal_points[aux]=(aux*distance_between_points_longitudinal)-(larg/2);
                    //ROS_INFO("pontos frente/tras referencia: para %d e de %f",aux,ref_longitudinal_points[aux]);
                }

                for(int aux=0;aux<lateral_points;aux++){ //para laterais do veiculo (esq e dir)
                    ref_lateral_points[aux]=(aux*distance_between_points_lateral)-(compr/2);
                    //ROS_INFO("pontos laterais referencia: para %d e de %f",aux,ref_lateral_points[aux]);
                }

            //para enviar dados ao miar_functions: (16/03)
              
                sector_obs_pub = nh.advertise<sensor_msgs::LaserScan>("sector_dist_obs",1); //publicar obs para SDNL
                distance_obs_pub = nh.advertise<mir_custom_msgs::distances_mir>("distances_obs",1); //publicar para definir distancia seguranca a volta do veiculo
                sequ=0;//sequencia começa em 0

           //determinar dist. minima:
            min_lat_left=0.0;
            min_lat_right=0.0;
            min_front=0.0;
            min_rear=0.0;
            safety_first=true;

            //Publicar  
            safe_to_run = nh.advertise<std_msgs::Bool>("keep_safe_distance",1); //publicar obs para Omni
            clear_to_run_pub.data=true;  //variavel a publicar

            //Em (25/05):
            safe_to_nav_SDNL = nh.advertise<std_msgs::Bool>("keep_safe_distance_SDNL",1); //publicar obs para SDNL;       //topico a publicar (se false-> para; se true->pode andar)
            clear_to_navSDNL_pub.data = true;  //variavel a publicar
}

obstacle_detection_class::~obstacle_detection_class(){
    //destructor

}

void obstacle_detection_class::dimensions_Callback(const geometry_msgs::Vector3::ConstPtr& msgs)
{
    compr = msgs->x;
    larg = msgs->y;

    if (larg - 910.0/1000.0<0.1)
    {
        Structure_on=false;
    }
    else
    {
        Structure_on = true;
    }
    std::cout<<"Structure_On ="<<Structure_on<<std::endl;
}
//Callback que recebe os dados em bruto do sensor frontal(do Coppelia)
void obstacle_detection_class::coppelia_sick_front_Callback(const sensor_msgs::LaserScan::ConstPtr& msg_coppelia_sick_front){
   
    if(msg_coppelia_sick_front->header.seq >= front_seq_receive){//se a msg recebida nao e antiga ou repetida:
        //ROS_INFO("-----------------------------SENSOR FRENTE-----------------------------");
        front_lidar_data=msg_coppelia_sick_front->ranges;
        //incre_sensor_front=msg_coppelia_sick_front->intensities; //o vector 'intensities' foi utilizado para enviar os incrementos (a tropa manda desenrascar xD)
        incre_sensor_front = msg_coppelia_sick_front->angle_increment; 
        min_angle_front=msg_coppelia_sick_front->angle_min;      //!!!Pode dar asneira: se o tempo de execucao disto for superior a taxa de rececao!!!
        front_laser_max_dist=msg_coppelia_sick_front->range_max;

        front_seq_receive=msg_coppelia_sick_front->header.seq; //atualizar sequencia msg recebida
    }
    clear_to_start_f=true;
}

//Callback que recebe os dados em bruto do sensor traseiro(do Coppelia)
void obstacle_detection_class::coppelia_sick_rear_Callback(const sensor_msgs::LaserScan::ConstPtr& msg_coppelia_sick_rear){
   
   if(msg_coppelia_sick_rear->header.seq >= rear_seq_receive){
        //ROS_INFO("-----------------------------SENSOR TRASEIRO-----------------------------");
        rear_lidar_data=msg_coppelia_sick_rear->ranges;
        //incre_sensor_rear=msg_coppelia_sick_rear->intensities; //o vector 'intensities' foi utilizado para enviar os incrementos (a tropa manda desenrascar xD)
        incre_sensor_rear = msg_coppelia_sick_rear->angle_increment;
        min_angle_rear=msg_coppelia_sick_rear->angle_min;      //!!!Pode dar asneira: se o tempo de execucao disto for superior a taxa de rececao!!!
        rear_laser_max_dist=msg_coppelia_sick_rear->range_max;
        
        rear_seq_receive=msg_coppelia_sick_rear->header.seq;
   }
    clear_to_start_b=true;
}


bool obstacle_detection_class::setup_lasers(){

    float lidar_angle_front=min_angle_front;
    theta_laser_front.clear();
    theta_laser_back.clear();
    //ROS_INFO("Tamanho array %d",front_lidar_data.size());

    if(front_lidar_data.size()==0)
        return false;

    for(int i=0;i<front_lidar_data.size();i++)
    {
        theta_laser_front.push_back(lidar_angle_front);
        //ROS_INFO("sector %d angle %f",i,theta_laser_front[i]);
        lidar_angle_front = lidar_angle_front+incre_sensor_front;
    }
    float lidar_angle_back=min_angle_rear;
    for(int i=0;i<rear_lidar_data.size();i++)
    {
        theta_laser_back.push_back(lidar_angle_back);
        lidar_angle_back = lidar_angle_back + incre_sensor_rear;
    }

    return true;
}


//funcao que converte dados entre o referencial do sensor(frente) e o da plataforma
bool obstacle_detection_class::convertFrameFrontLaser(){
    //float angle=min_angle_front;
    sick_front_data_in.header.stamp=ros::Time();
    distance_front.clear();
    angle_front.clear();
    x_robot_frame_front.clear();
    y_robot_frame_front.clear();
   // structure_invalid_pos_front.clear();
    int i=0;

    int minfrontCoppelia=31;    //simulaçao
    int maxfrontCoppelia=633;   //simulação

    for(int pos=minfrontCoppelia;pos<maxfrontCoppelia;pos++){
        //ROS_INFO("Frente: line %d: dist= %f angle = %f",pos,front_lidar_data[pos],theta_laser_front[pos]*180/M_PI);
        //converter em posicoes cartesianas:
        if(Structure_on){
            if (!((pos >= 40 && pos <= 75) || (pos >= 95 && pos <= 130))){
                if(front_lidar_data[pos] < 40 && front_lidar_data[pos] > 0.0 ){
                    sick_front_data_in.point.x=cos(theta_laser_front[pos])*front_lidar_data[pos];
                    sick_front_data_in.point.y=sin(theta_laser_front[pos])*front_lidar_data[pos];

                    try{
                        tfBuffer.transform(sick_front_data_in, sick_front_data_kuka, "mirref_handler",ros::Duration(1.0)); //fica preso ate 1s ate que chegue novo broadcast (maximo)
                        //armazenar as novas posicoes(apos a transformacao):


                        x_robot_frame_front.push_back(sick_front_data_kuka.point.x);
                        y_robot_frame_front.push_back(sick_front_data_kuka.point.y);
                        distance_front.push_back(sqrt(pow(sick_front_data_kuka.point.x, 2) + pow(sick_front_data_kuka.point.y, 2)));
                        angle_front.push_back(atan2(sick_front_data_kuka.point.y, sick_front_data_kuka.point.x));

                    }
                    catch(tf2::TransformException& ex){
                        ROS_ERROR("Received an exception trying to transform a point from \"base_laser\" to \"base_link\": %s", ex.what());
                    }
                    //angle=angle+incre_sensor_front;
                }
            }
        }
        else{
            if(front_lidar_data[pos] < 40 && front_lidar_data[pos] > 0.0 ){
                    sick_front_data_in.point.x=cos(theta_laser_front[pos])*front_lidar_data[pos];
                    sick_front_data_in.point.y=sin(theta_laser_front[pos])*front_lidar_data[pos];

                    try{
                        tfBuffer.transform(sick_front_data_in, sick_front_data_kuka, "mirref_handler",ros::Duration(1.0)); //fica preso ate 1s ate que chegue novo broadcast (maximo)
                        //armazenar as novas posicoes(apos a transformacao):
        


                        x_robot_frame_front.push_back(sick_front_data_kuka.point.x);
                        y_robot_frame_front.push_back(sick_front_data_kuka.point.y);
                        distance_front.push_back(sqrt(pow(sick_front_data_kuka.point.x, 2) + pow(sick_front_data_kuka.point.y, 2)));
                        angle_front.push_back(atan2(sick_front_data_kuka.point.y, sick_front_data_kuka.point.x));

                    }
                    catch(tf2::TransformException& ex){
                        ROS_ERROR("Received an exception trying to transform a point from \"base_laser\" to \"base_link\": %s", ex.what());
                    }
                    //angle=angle+incre_sensor_front;
                }
        }
    }
    return true;
}

//funcao que converte dados entre o referencial do sensor(traseiro) e o da plataforma
bool obstacle_detection_class::convertFrameRearLaser(){
    //float angle=min_angle_rear;
    sick_rear_data_in.header.stamp=ros::Time();
    distance_rear.clear();
    angle_rear.clear();
    x_robot_frame_rear.clear();
    y_robot_frame_rear.clear();
    //structure_invalid_pos_rear.clear();
    int i =0;


    int minrearCoppelia=31;    //simulaçao
    int maxrearCoppelia=640;   //simulação

    for(int pos=minrearCoppelia;pos<maxrearCoppelia;pos++){
        //converter em posicoes cartesianas:
        if(Structure_on){
            if (!((pos >= 40 && pos <= 75) || (pos >= 86 && pos <= 130))){
                if(rear_lidar_data[pos] < 40 && rear_lidar_data[pos] > 0.0){
                    sick_rear_data_in.point.x=cos(theta_laser_back[pos])*rear_lidar_data[pos];
                    sick_rear_data_in.point.y=sin(theta_laser_back[pos])*rear_lidar_data[pos];
                  
                    try{
                        tfBuffer.transform(sick_rear_data_in, sick_rear_data_kuka, "mirref_handler",ros::Duration(1.0)); //fica preso ate 1s ate que chegue novo broadcast (maximo)
                        //armazenar as novas posicoes(apos a transformacao):

                        x_robot_frame_rear.push_back(sick_rear_data_kuka.point.x);
                        y_robot_frame_rear.push_back(sick_rear_data_kuka.point.y);
                        distance_rear.push_back(sqrt(pow(sick_rear_data_kuka.point.x, 2) + pow(sick_rear_data_kuka.point.y, 2)));
                        angle_rear.push_back(atan2(sick_rear_data_kuka.point.y, sick_rear_data_kuka.point.x));

                    }
                    catch(tf2::TransformException& ex){
                        ROS_ERROR("Received an exception trying to transform a point from \"base_laser\" to \"base_link\": %s", ex.what());
                    }
                    //angle=angle+incre_sensor_rear;
                }
            }
        }
        else{
             if(rear_lidar_data[pos] < 40 && rear_lidar_data[pos] > 0.0){
                    sick_rear_data_in.point.x=cos(theta_laser_back[pos])*rear_lidar_data[pos];
                    sick_rear_data_in.point.y=sin(theta_laser_back[pos])*rear_lidar_data[pos];

                    try{
                        tfBuffer.transform(sick_rear_data_in, sick_rear_data_kuka, "mirref_handler",ros::Duration(1.0)); //fica preso ate 1s ate que chegue novo broadcast (maximo)
                        //armazenar as novas posicoes(apos a transformacao):

                        x_robot_frame_rear.push_back(sick_rear_data_kuka.point.x);
                        y_robot_frame_rear.push_back(sick_rear_data_kuka.point.y);
                        distance_rear.push_back(sqrt(pow(sick_rear_data_kuka.point.x, 2) + pow(sick_rear_data_kuka.point.y, 2)));
                        angle_rear.push_back(atan2(sick_rear_data_kuka.point.y, sick_rear_data_kuka.point.x));

                    }
                    catch(tf2::TransformException& ex){
                        ROS_ERROR("Received an exception trying to transform a point from \"base_laser\" to \"base_link\": %s", ex.what());
                    }
                    //angle=angle+incre_sensor_rear;
                }
        }
    }
    return true;
}

bool obstacle_detection_class::getObs_raw(){

    convertFrameFrontLaser();

    convertFrameRearLaser();
    
return true;
}

//esta funcao determina os angulos de cada setor dinamico (para os SDNL) em funcao do nº setores desejados e a gama de sensorizacao
//A funcao tambem calcula a distancia a "descontar" em por cada setor (considerar o proprio corpo do robo)
//NOTA: !!  O nº de setores TEM de ser um NUMERO IMPARE  !!
bool obstacle_detection_class::setup_obs(){

    float aux_inc=range/(n_setores-1.00); // (210*pi/180) /21-1
    float alfa=atan2(larg,compr);  //tem de ser atan2   !!!
    //ROS_INFO("Angulo alfa e de %f", alfa); //para debug

    for(int i=0;i<n_setores;i++){
        theta_obs[i]=-(range/2.0)+aux_inc*i;            //definir os angulos que correspondem aos setores de interesse aos sistemas dinâmicos (n_setores/2 tem de corresponder a 0 graus (frente robo))
        x_setor_final[i]=max_sensor*cos(theta_obs[i]);  //calcular ponto 'x' maximo do setor 'i'
        y_setor_final[i]=max_sensor*sin(theta_obs[i]);  //calcular ponto 'y' maximo do setor 'i'
        //ROS_INFO("Setor dinamica %d para angulo de %f (em graus: %f)",i, theta_obs[i], (theta_obs[i]*180.00)/M_PI);
        
        //caso os setores pertençam à parte frontal ou traseira do veiculo:
        //este tem de ter modulo (pois passa por -pi!!):                        AQUI                AQUI
        if((-alfa<=theta_obs[i] && theta_obs[i]<=alfa) || ((M_PI-alfa)<=fabs(theta_obs[i]) && fabs(theta_obs[i])<=(alfa-M_PI))){//    (-29,14<theta<29,14) ou (+150,86<theta<-150,86)
            robot_dist[i]=fabs((compr/2.00)/cos(theta_obs[i]));
            //ROS_INFO("Tras ou frente do robo. A distancia corpo robo e de %f metros",robot_dist[i]);
//em caso de duvidas, ver caderno na data de 10/03/2022!!!!!
        }
        //caso os setores pertençam às laterais do veiculo:
        else if((alfa<=theta_obs[i] && theta_obs[i]<=(M_PI-alfa)) || ((alfa-M_PI)<=theta_obs[i] && theta_obs[i]<=(-alfa))){
            robot_dist[i]=fabs((larg/2.00)/sin(theta_obs[i]));
            //ROS_INFO("Laterais do robo. A distancia corpo robo e de %f metros",robot_dist[i]);
        }

    }

return true;
}

bool obstacle_detection_class::getdynamic_obs(){

    float angle_min=0.0;
    float angle_max=0.0;
    float x_intersecao=0.0;
    float y_intersecao=0.0;
    float m_reta=0.0;   //declive da reta entre dois lasers consecutivos do sick
    float b_reta=0.0;   //so a reta entre dois lasers consecutivos do sick tem 'b' (os setores saem do (0,0) do eixo central do veiculo)
    float m_setor=0.0;  //declive setor dinamico
    float distance=0.0;
    dist_obs_setor.assign(n_setores,30.00);

    getObs_raw();//para fazer as transformacoes entre referenciais.
    
    if (x_robot_frame_front.size()>0 && y_robot_frame_front.size()>0 && x_robot_frame_rear.size()>0 && y_robot_frame_rear.size()>0 ){
        //sensor frente (considerar todos os lasers-> nenhum deteta o proprio veiculo):
        //comecar por '1' e nao por '0' pois na 1ª iteracao precisa do valor do angulo anterior!!!

        for(int f=1;f<=angle_front.size();f++){  //'f' stands for 'front' (Stonks xD)
    //iniciado a 11/02
        //ROS_INFO("Teste %f",distance_front[f]);
            if(angle_front[f-1]<=angle_front[f]){ //tem que ter estas condicoes pois os angulos nao variam linearmente com os setores (pois os sick estao na lateral e a 45º do centro do veiculo)
                angle_max=angle_front[f];
                angle_min=angle_front[f-1];
            }
            else{   //caso contrario    (ex o setor 200 tem 30º e o setor 201 tem 29º devido à disposicao do obstaculo)
                angle_max=angle_front[f-1];
                angle_min=angle_front[f];
            }

    //nao confundir o 'pos' e o 'f'!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            for(int pos=0;pos<n_setores;pos++){ //percorrer todos os setores dinâmicos definidos
            // ROS_INFO("angle_max = %f angle_min = %f theta_obs[%d] = %f dist1 = %f dist2 = %f",angle_max,angle_min,pos,theta_obs[pos],distance_front[f],distance_front[f-1]);
                if(angle_min < theta_obs[pos] && theta_obs[pos]<angle_max){ //se o angulo do setor esta dentro entre os dois "lasers" do sick:
                //ROS_INFO("Teste %f",distance_front[f]);
                    if((x_robot_frame_front[f]-x_robot_frame_front[f-1])==0.00){  //condicao 1 (ver caderno dia 08/03) ORIGEM !!
                        x_intersecao=x_robot_frame_front[f];
                        m_setor=y_setor_final[pos]/x_setor_final[pos]; //ou m_setor=sin(theta_obs[pos])/cos(theta_obs[pos]); ou ainda m_setor=tan(theta_obs[pos]);
                        y_intersecao=x_intersecao*m_setor;
                    }
                    else if(x_setor_final[pos]==0.00){ //condicao 2 -> quando theta_obs[pos]==90º ou -90º (ver caderno dia 08/03)
                        x_intersecao=0.0; //esta sobreposta ao eixo dos xx
                        m_reta=(y_robot_frame_front[f]-y_robot_frame_front[f-1])/(x_robot_frame_front[f]-x_robot_frame_front[f-1]);
                        b_reta=y_robot_frame_front[f]-m_reta*x_robot_frame_front[f];
                        y_intersecao=b_reta;
                    }
                    else{ //condicao 3 -> o "normal" (ver caderno dia 08/03)
                        m_setor=(y_setor_final[pos]/x_setor_final[pos]);
                        m_reta=((y_robot_frame_front[f]-y_robot_frame_front[f-1])/(x_robot_frame_front[f]-x_robot_frame_front[f-1]));
                        b_reta=y_robot_frame_front[f]-m_reta*x_robot_frame_front[f];
                        x_intersecao=(b_reta/(m_setor-m_reta));
                        y_intersecao=m_setor*x_intersecao;
                    }

                    distance=(sqrt(pow(x_intersecao, 2)+pow(y_intersecao, 2)))-robot_dist[pos]; //ja retirar a distancia do "corpo" do veiculo
                    //if(distance<1)
                    //{
                        
                    //}
                
                    //porquê utilizar o 'distance>0.0'? -> Não esquecer que estamos a subtrair o "corpo" do robo, mas em principio nao deveria de ser, se acontecer algo de errado aconteceu...
                    if(dist_obs_setor[pos]>distance && distance>0.0){//se a ultima distancia armazenada no setor 'pos' for maior do que a nova, ATUALIZAR COM A MENOR DISTANCIA!!!!!!!
                        dist_obs_setor[pos]=distance;

                    }
                    //acho que nao e preciso meter o b_reta,m_reta,m_setor a 0, pois quando sao utilizados antes sao calculados
                }
            }
        }
        //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        //FAZER O FAMOSO 'copy+past' para fazer o mesmo para o sensor tras
        //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        
        for(int r=1;r<=angle_rear.size();r++){  //'r' stands for 'rear' (Stonks xD)
    //iniciado a 11/02
            if(angle_rear[r-1]<angle_rear[r]){ //tem que ter estas condicoes pois os angulos nao variam linearmente com os setores (pois os sick estao na lateral e a 45º do centro do veiculo)
                angle_max=angle_rear[r];
                angle_min=angle_rear[r-1];
            }
            else{   //caso contrario    (ex o setor 200 tem 30º e o setor 201 tem 29º devido à disposicao do obstaculo)
                angle_max=angle_rear[r-1];
                angle_min=angle_rear[r];
            }
            //(ver caderno dia 15/03 para ver a razao deste if....)
            if(angle_max-angle_min<M_PI){ //precisa deste 'if'caso dois lasers do lidar detetassem obstaculo em, p.e. -175º e outro em +175º a gama seria de 350º xD (esta solucao apenas e util caso nao utilizar sensores traseiros para a dinamica SDNL)

    //nao confundir o 'pos' e o 'f'!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                for(int pos=0;pos<n_setores;pos++){ //percorrer todos os setores dinâmicos "impostos"
            //  ROS_INFO("angle_max = %f angle_min = %f theta_obs[%d] = %f dist1 = %f dist2 = %f",angle_max,angle_min,pos,theta_obs[pos],distance_front[f],distance_front[f-1]);
                    if(angle_min<theta_obs[pos] && theta_obs[pos]<angle_max){ //se o angulo do setor esta dentro entre os dois "lasers" do sick:

                        if((x_robot_frame_rear[r]-x_robot_frame_rear[r-1])==0.00){  //condicao 1 (ver caderno dia 08/03)
                            x_intersecao=x_robot_frame_rear[r];
                            m_setor=y_setor_final[pos]/x_setor_final[pos]; //ou m_setor=sin(theta_obs[pos])/cos(theta_obs[pos]); ou ainda m_setor=tan(theta_obs[pos]);
                            y_intersecao=x_intersecao*m_setor;
                        }
                        else if(x_setor_final[pos]==0.00){ //condicao 2 -> quando theta_obs[pos]==90º ou -90º (ver caderno dia 08/03)
                            x_intersecao=0.0; //esta sobreposta ao eixo dos xx
                            m_reta=(y_robot_frame_rear[r]-y_robot_frame_rear[r-1])/(x_robot_frame_rear[r]-x_robot_frame_rear[r-1]);
                            b_reta=y_robot_frame_rear[r]-m_reta*x_robot_frame_rear[r];
                            y_intersecao=b_reta;
                        }
                        else{ //condicao 3 -> o "normal" (ver caderno dia 08/03)
                            m_setor=(y_setor_final[pos]/x_setor_final[pos]);
                            m_reta=((y_robot_frame_rear[r]-y_robot_frame_rear[r-1])/(x_robot_frame_rear[r]-x_robot_frame_rear[r-1]));
                            b_reta=y_robot_frame_rear[r]-m_reta*x_robot_frame_rear[r];
                            x_intersecao=(b_reta/(m_setor-m_reta));
                            y_intersecao=m_setor*x_intersecao;
                        }
                        
                        distance=(sqrt(pow(x_intersecao, 2)+pow(y_intersecao, 2)))-robot_dist[pos]; //ja retirar a distancia do "corpo" do veiculo
                        //porquê utilizar o 'distance>0.0'? -> Não esquecer que estamos a subtrair o "corpo" do robo, mas em principio nao deveria de ser, se acontecer algo de errado aconteceu...
                        if(dist_obs_setor[pos]>distance && distance>0.0){//se a ultima distancia armazenada no setor 'pos' for maior do que a nova, ATUALIZAR COM A MENOR DISTANCIA!!!!!!!
                            dist_obs_setor[pos]=distance;
                            //ROS_INFO("Dinamica obs TRAS: setor %d, laser %d. Theta= %f, min_theta=%f max_theta=%f",pos,r,theta_obs[pos],angle_min,angle_max);
                        }
                    }
                }
            }
        }    


    //*****************************************************************************
    //Parte distancias laterais e frontal e tras (tipo sensores estacionamento???)
    //                               15/03
    //*****************************************************************************

    //Porque 5m (e nao 30)? Pois a intencao e usar como aproximacao final, logo para curtas distancias (talvez em cm e poucos metros)
    dist_front_obs.assign(longitudinal_points,5.00); //by default is 5 m
    dist_rear_obs.assign(longitudinal_points,5.00);

    //parte frente e lateral direita (só sensor frontal):
        for(int sensor_fr=0;sensor_fr<=y_robot_frame_front.size()-1;sensor_fr++){
            //se for parte frontal do veiculo:             //talvez esta parte possa ser apagada.::::::::::::::::::::::::::::::::::::::::::::::::::::::
            if(x_robot_frame_front[sensor_fr]>=(compr/2.0)  && y_robot_frame_front[sensor_fr]>=(-(larg/2.0+min_y_incre)) && y_robot_frame_front[sensor_fr]<=((larg/2.0+min_y_incre))){
                //percorrer frente (uteis):
                for(int f=0;f<longitudinal_points;f++){
                    //se pertence ao "setor" frontal nº 'f':
                    if(fabs(y_robot_frame_front[sensor_fr]-ref_longitudinal_points[f])<=min_y_incre){
                        //distancias frontais estao em 'x'; se a nova distancia e inferior a anterior guardada atualiza:
                        //if((x_robot_frame_front[sensor_fr]-compr/2.0)<dist_front_obs[f]){
                            dist_front_obs[f]=x_robot_frame_front[sensor_fr]-compr/2.0;//tirar o "corpo do veiculo"
                        //}
                    }
                }
            }
            
            
        }   


    for(int sensor_re=0;sensor_re<=y_robot_frame_rear.size()-1;sensor_re++){
        
        //se for parte tras do veiculo:             //talvez esta parte possa ser apagada.::::::::::::::::::::::::::::::::::::::::::::::::::::::
        //as medidas em 'x' neste caso vao ser sempre negativas
        if(x_robot_frame_rear[sensor_re]<=(-compr/2.0) && y_robot_frame_rear[sensor_re]>=(-(larg+min_y_incre)/2.0) && y_robot_frame_rear[sensor_re]<=((larg+min_y_incre)/2.0)){
            //percorrer frente (uteis):
            for(int r=0;r<longitudinal_points;r++){
                //se pertence ao "setor" tras nº 'r':
                if(fabs(y_robot_frame_rear[sensor_re]-ref_longitudinal_points[r])<=min_y_incre){
                    if((fabs(x_robot_frame_rear[sensor_re])-compr/2.0)<dist_rear_obs[r]){
                        dist_rear_obs[r]=fabs(x_robot_frame_rear[sensor_re])-compr/2.0;//tirar o "corpo do veiculo"
                    }
                    //}
                }
            }
        }
    }  

    }
 
return true;
}

//funcao para enviar os dados dos obstaculos para a dinamica de navegacao:
bool obstacle_detection_class::pub_obs_data(){

//**Informacao para os SDNL (navegacao autonoma)**//
  

    sectorsMsgPub.header.seq = sequ;
    sectorsMsgPub.ranges = dist_obs_setor;
    sectorsMsgPub.angle_min = theta_obs.front();
    sectorsMsgPub.angle_max = theta_obs.back();
    sectorsMsgPub.angle_increment = theta_obs[1]-theta_obs[0];
    sectorsMsgPub.range_min = 0.0;
    sectorsMsgPub.range_max = 30.0;
    sectorsMsgPub.intensities = theta_obs;
    sectorsMsgPub.ranges = dist_obs_setor;

//**Informacao para definir a dist. seguranca (tipo sensores estacionamento)**// 
    mir_custom_msgs::distances_mir distancesMsgPub; //relativo aos setores
    distancesMsgPub.header=sequ;
    distancesMsgPub.dist_front=dist_front_obs;
    distancesMsgPub.dist_rear=dist_rear_obs;
   
    sector_obs_pub.publish(sectorsMsgPub);                        //publicar a mensagem relativa aos setores
    distance_obs_pub.publish(distancesMsgPub);     //publicar a mensagem relativa as dist. seguranca
    
    sequ++; //por fim incrementar a seq. de envio

return true;
}


//verificar distancias minimas em 360º do veiculo. Publica 'false' caso o veiculo tenha que parar por seguranca
bool obstacle_detection_class::min_value_vector_simple(){
    min_lat_left = *min_element(dist_left_obs.begin(), dist_left_obs.end());
    min_lat_right = *min_element(dist_right_obs.begin(), dist_right_obs.end());
    min_rear = *min_element(dist_rear_obs.begin(), dist_rear_obs.end());
    min_front = *min_element(dist_front_obs.begin(), dist_front_obs.end());
    
    if(min_lat_left<0.30 || min_lat_right<0.30 || min_rear<0.30 || min_front<0.30){ //parte navegacao 
        clear_to_navSDNL_pub.data = false;
        safe_to_nav_SDNL.publish(clear_to_navSDNL_pub);
        
        if(min_lat_left<0.10 || min_lat_right<0.10 || min_rear<0.10 || min_front<0.20){ //parte da navegacao omnidirecional
            clear_to_run_pub.data=false;
            safe_to_run.publish(clear_to_run_pub);  //parar!!
            ROS_INFO("Mandar parar a navegacao omnidirecional");
            return false;
        }
    }
    else{
        clear_to_navSDNL_pub.data = true;
        safe_to_nav_SDNL.publish(clear_to_navSDNL_pub);
        clear_to_run_pub.data=true;
        safe_to_run.publish(clear_to_run_pub); //pode andar!!
        return true; 
    }
    return true;
}

int main(int argc, char **argv)
{  
    ros::init(argc, argv, "obstacle_detection");
    ros::NodeHandle n_priv("~");

    obstacle_detection_class *localObst;
    localObst = new obstacle_detection_class(); //cria objeto

    ros::Rate rate(20); //20Hz (50ms)      //80Hz (12,5ms)

    while((localObst->clear_to_start_f==false && localObst->clear_to_start_b ==false )){ //fica a espera que o 'positioning' fique operacional
        
        ROS_INFO("Ainda a espera do CoppeliaSim");
        ros::spinOnce();
		rate.sleep();
    }
    
    //definir os angulos dos setores dinamicos:
    localObst->setup_obs();
    localObst->setup_lasers();
    sleep(1);
    while(ros::ok())
    { 
       
        localObst->getdynamic_obs();

     
        localObst->pub_obs_data();  //publica os dados dos sensores (nos topicos 'sector_dist_obs' e 'protec_dist_obs')
        
       
        /*localObst->safety_first=localObst->min_value_vector_simple();
        ROS_INFO("Verificacao obstaculos");
        if(localObst->safety_first){
            ROS_INFO("Seguro Para andar");
        }
        else{
            ROS_INFO("Obstaculos demasiado proximos");
        }*/

        ros::spinOnce();
	    rate.sleep();
    }

}