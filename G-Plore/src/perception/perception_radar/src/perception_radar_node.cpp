#include "perception_radar/perception_radar_node.h"

global::perception::radar::Perception_Radar_Node::Perception_Radar_Node(
ros::NodeHandle& public_node, ros::NodeHandle& private_node):
RadarF_Socket(new UDPSocket(RadarF_Port)),
RadarB_Socket(new UDPSocket(RadarB_Port)),
RadarFL_Socket(new UDPSocket(RadarFL_Port)),
RadarFR_Socket(new UDPSocket(RadarFR_Port)),
RadarBL_Socket(new UDPSocket(RadarBL_Port)),
RadarBR_Socket(new UDPSocket(RadarBR_Port))
{
    if (print_message)
        std::cout << "Perception_Radar_Node start" << std::endl;
    getParameter(public_node, private_node);
    // ros_pub_RadarObjs_info = public_node.advertise<RadarMsg::RadarObjs>(MSG_TOPIC_RADAROBJ, 1);
    Sub_InsMsg=public_node.subscribe<InsMsg::ins_p2>(MSG_TOPIC_GLOBALPOSE, 1, &Perception_Radar_Node::Recv_InsMsg_callack, this);
    Sub_CdmMsg=public_node.subscribe<VehicleMsg::cdm_cmd>(MSG_TOPIC_CDM, 1, &Perception_Radar_Node::Recv_CdmMsg_callack, this);
    RadarF_Thread = new boost::thread(boost::bind(&Perception_Radar_Node::processPendingDatagramsRadarF, this));
    RadarB_Thread = new boost::thread(boost::bind(&Perception_Radar_Node::processPendingDatagramsRadarB, this));
    RadarFL_Thread = new boost::thread(boost::bind(&Perception_Radar_Node::processPendingDatagramsRadarFL, this));
    RadarFR_Thread = new boost::thread(boost::bind(&Perception_Radar_Node::processPendingDatagramsRadarFR, this));
    RadarBL_Thread = new boost::thread(boost::bind(&Perception_Radar_Node::processPendingDatagramsRadarBL, this));
    RadarBR_Thread = new boost::thread(boost::bind(&Perception_Radar_Node::processPendingDatagramsRadarBR, this));
    radar_fusion_thread_ = std::thread(&Perception_Radar_Node::radarFusion, this);
    if (print_message)
        std::cout << "Perception_Radar_Node end" << std::endl;
}

global::perception::radar::Perception_Radar_Node::~Perception_Radar_Node()
{
    // thread join
    RadarF_Thread->join();
    RadarB_Thread->join();
    RadarFL_Thread->join();
    RadarFR_Thread->join();
    RadarBL_Thread->join();
    RadarBR_Thread->join();
    radar_fusion_thread_.join();
    // socket disconnect
    RadarF_Socket->disconnect();
    RadarB_Socket->disconnect();
    RadarFL_Socket->disconnect();
    RadarFR_Socket->disconnect();
    RadarBL_Socket->disconnect();
    RadarBR_Socket->disconnect();
    // free space
     delete RadarF_Thread;
     delete RadarB_Thread;
     delete RadarFL_Thread;
     delete RadarFR_Thread;
     delete RadarBL_Thread;
     delete RadarBR_Thread;
}

float global::perception::radar::Perception_Radar_Node::Obj_Rms(uint8_t num)
{
    if(num ==0)
    {
        return 0.005;
    }
    else if(num ==1)
    {
        return 0.006;
    }
    else if(num ==2)
    {
        return 0.008;
    }
    else if(num ==3)
    {
        return 0.011;
    }
    else if(num ==4)
    {
        return 0.014;
    }
    else if(num ==5)
    {
        return 0.018;
    }
    else if(num ==6)
    {
        return 0.023;
    }
    else if(num ==7)
    {
        return 0.029;
    }
    else if(num ==8)
    {
        return 0.038;
    }
    else if(num ==9)
    {
        return 0.049;
    }
    else if(num ==0XA)
    {
        return 0.063;
    }
    else if(num ==0XB)
    {
        return 0.081;
    }
    else if(num ==0XC)
    {
        return 0.105;
    }
    else if(num ==0XD)
    {
        return 0.135;
    }
    else if(num ==0XE)
    {
        return 0.174;
    }
    else if(num ==0XF)
    {
        return 0.224;
    }
    else if(num ==0X10)
    {
        return 0.288;
    }
    else if(num ==0X11)
    {
        return 0.371;
    }
    else if(num ==0X12)
    {
        return 0.478;
    }
    else if(num ==0X13)
    {
        return 0.616;
    }
    else if(num ==0X14)
    {
        return 0.794;
    }
    else if(num ==0X15)
    {
        return 1.023;
    }
    else if(num ==0X16)
    {
        return 1.317;
    }
    else if(num ==0X17)
    {
        return 1.697;
    }
    else if(num ==0X18)
    {
        return 2.187;
    }
    else if(num ==0X19)
    {
        return 2.817;
    }
    else if(num ==0X1A)
    {
        return 3.63;
    }
    else if(num ==0X1B)
    {
        return 4.767;
    }
    else if(num ==0X1C)
    {
        return 6.025;
    }
    else if(num ==0X1D)
    {
        return 7.762;
    }
    else if(num ==0X1E)
    {
        return 10.0;
    }
    else if(num ==0X1F)
    {
        return 99;
    }
}

float global::perception::radar::Perception_Radar_Node::ObjOri_Rms(uint8_t num)
{
    if(num ==0)
    {
        return 0.005;
    }
    else if(num ==1)
    {
        return 0.007;
    }
    else if(num ==2)
    {
        return 0.010;
    }
    else if(num ==3)
    {
        return 0.014;
    }
    else if(num ==4)
    {
        return 0.020;
    }
    else if(num ==5)
    {
        return 0.029;
    }
    else if(num ==6)
    {
        return 0.041;
    }
    else if(num ==7)
    {
        return 0.058;
    }
    else if(num ==8)
    {
        return 0.082;
    }
    else if(num ==9)
    {
        return 0.116;
    }
    else if(num ==0XA)
    {
        return 0.165;
    }
    else if(num ==0XB)
    {
        return 0.234;
    }
    else if(num ==0XC)
    {
        return 0.332;
    }
    else if(num ==0XD)
    {
        return 0.471;
    }
    else if(num ==0XE)
    {
        return 0.669;
    }
    else if(num ==0XF)
    {
        return 0.949;
    }
    else if(num ==0X10)
    {
        return 1.346;
    }
    else if(num ==0X11)
    {
        return 1.909;
    }
    else if(num ==0X12)
    {
        return 2.709;
    }
    else if(num ==0X13)
    {
        return 3.843;
    }
    else if(num ==0X14)
    {
        return 5.451;
    }
    else if(num ==0X15)
    {
        return 7.734;
    }
    else if(num ==0X16)
    {
        return 10.971;
    }
    else if(num ==0X17)
    {
        return 15.565;
    }
    else if(num ==0X18)
    {
        return 22.081;
    }
    else if(num ==0X19)
    {
        return 31.325;
    }
    else if(num ==0X1A)
    {
        return 44.439;
    }
    else if(num ==0X1B)
    {
        return 63.044;
    }
    else if(num ==0X1C)
    {
        return 89.437;
    }
    else if(num ==0X1D)
    {
        return 126.881;
    }
    else if(num ==0X1E)
    {
        return 180.0;
    }
    else if(num ==0X1F)
    {
        return 400;
    }
}

void global::perception::radar::Perception_Radar_Node::processPendingDatagramsRadarF()
{
    uint16_t iLen;
    uint8_t RadarF_buf[1300];
    std::cout << "RadarF Thread"<< std::endl;
    while(true)
    {
        iLen = RadarF_Socket->recvFrom(RadarF_buf, RADARF_INFO_LENGHT, RadarUrl, RadarF_Port);
        if(iLen == RADARF_INFO_LENGHT)
        {
            memcpy(RadarF_msg,RadarF_buf,RADARF_INFO_LENGHT);
            publishRadarF((void*) RadarF_msg);
        }
    }
}

void global::perception::radar::Perception_Radar_Node::processPendingDatagramsRadarB()
{
    uint16_t iLen;
    uint8_t RadarB_buf[1300];
    std::cout << "RadarB Thread"<< std::endl;
    while(true)
    {
        iLen = RadarB_Socket->recvFrom(RadarB_buf, RADARF_INFO_LENGHT, RadarUrl, RadarB_Port);
        if (iLen == RADARF_INFO_LENGHT)
        {
            memcpy(RadarB_msg,RadarB_buf,RADARF_INFO_LENGHT);
            publishRadarB((void*) RadarB_msg);
        }
    }
}

void global::perception::radar::Perception_Radar_Node::processPendingDatagramsRadarFL()
{
    uint16_t iLen;
    uint8_t RadarFL_buf[1300];
    std::cout << "RadarFL Thread"<< std::endl;
    while(true)
    {
        iLen = RadarFL_Socket->recvFrom(RadarFL_buf, RADARF_INFO_LENGHT, RadarUrl, RadarFL_Port);
        if (iLen == RADARF_INFO_LENGHT) 
        {
            memcpy(RadarFL_msg,RadarFL_buf,RADARF_INFO_LENGHT);
            publishRadarFL((void*) RadarFL_msg);
        }
    }
}

void global::perception::radar::Perception_Radar_Node::processPendingDatagramsRadarFR()
{
    uint16_t iLen;
    uint8_t RadarFR_buf[1300];
    std::cout << "RadarFR Thread"<< std::endl;
    while(true)
    {
        iLen = RadarFR_Socket->recvFrom(RadarFR_buf, RADARF_INFO_LENGHT, RadarUrl, RadarFR_Port);
        if (iLen == RADARF_INFO_LENGHT)
        {
            memcpy(RadarFR_msg,RadarFR_buf,RADARF_INFO_LENGHT);
            publishRadarFR((void*) RadarFR_msg);
        }
    }
}

void global::perception::radar::Perception_Radar_Node::processPendingDatagramsRadarBL()
{
    uint16_t iLen;
    uint8_t RadarBL_buf[1300];
    std::cout << "RadarBL Thread"<< std::endl;
    while(true)
    {
        iLen = RadarBL_Socket->recvFrom(RadarBL_buf, RADARF_INFO_LENGHT, RadarUrl, RadarBL_Port);
        if (iLen == RADARF_INFO_LENGHT)
        {
            memcpy(RadarBL_msg,RadarBL_buf,RADARF_INFO_LENGHT);
            publishRadarBL((void*) RadarBL_msg);
        }
    }
}

void global::perception::radar::Perception_Radar_Node::processPendingDatagramsRadarBR()
{
    uint16_t iLen;
    uint8_t RadarBR_buf[1300];
    std::cout << "RadarBR Thread"<< std::endl;
    while(true){
        iLen = RadarBR_Socket->recvFrom(RadarBR_buf, RADARF_INFO_LENGHT, RadarUrl, RadarBR_Port);
        if (iLen == RADARF_INFO_LENGHT)
        {
            memcpy(RadarBR_msg,RadarBR_buf,RADARF_INFO_LENGHT);
            publishRadarBR((void*) RadarBR_msg);
        }
    }
}

void global::perception::radar::Perception_Radar_Node::publishRadarF(void *data)
{
    unsigned char* buffer = (unsigned char*)data;
    uint8_t i;
    //60a
    ars408_60_obj_0_status_unpack(&objf_0,&buffer[0],8);
    //60b
    for(i=0;i<32;i++)
    {
        ars408_60_obj_1_general_unpack(&objf_1[i],&buffer[18+i*8],8);
    }
    //60C
    for(i=0;i<32;i++)
    {
        ars408_60_obj_2_quality_unpack(&objf_2[i],&buffer[426+i*8],8);
    }
    //60d
    for(i=0;i<32;i++)
    {
        ars408_60_obj_3_extended_unpack(&objf_3[i],&buffer[834+i*8],8);
    }
    //60a
    Radar_msg[0].objNum=objf_0.obj_nof_objects;
    Radar_msg[0].streamTxCnt=objf_0.obj_meas_counter;
    //60b Obj_1_General
    for(i=0;i<obj_max_num;i++)
    {
        Radar_msg[0].ID[i]= objf_1[i].obj_id;
        // std::cout << "radar forward " << (int)i << " id " << (int)Radar_msg[0].ID[i] << std::endl;
        Radar_msg[0].Lat_Pos[i]= (objf_1[i].obj_dist_lat)*0.2-204.6;
        Radar_msg[0].Long_Pos[i]= (objf_1[i].obj_dist_long)*0.2-500;
        Radar_msg[0].Lat_Vel[i]= (objf_1[i].obj_vrel_lat)*0.25-64;
        Radar_msg[0].Long_Vel[i]= (objf_1[i].obj_vrel_long)*0.25-128;
        Radar_msg[0].RCS[i]= (objf_1[i].obj_rcs)*0.5-64;
        Radar_msg[0].Dyn_Prop[i]= (Radar_DynProp)(objf_1[i].obj_dyn_prop);
        if(objf_1[i].obj_dist_lat ==0)
        {
            Radar_msg[0].Lat_Pos[i]=0;
        }
        if(objf_1[i].obj_dist_long ==0)
        {
            Radar_msg[0].Long_Pos[i]=0;
        }
        if(objf_1[i].obj_vrel_lat ==0)
        {
            Radar_msg[0].Lat_Vel[i]=0;
        }
        if(objf_1[i].obj_vrel_long ==0)
        {
            Radar_msg[0].Long_Vel[i]=0;
        }
        if(objf_1[i].obj_rcs ==0)
        {
            Radar_msg[0].RCS[i]=0;
        }
    }
    //Obj_2_Quality 67c
    for(i=0;i<obj_max_num;i++)
    {
        Radar_msg[0].Long_Pos_Stdev[i]= Obj_Rms((objf_2[i].obj_dist_long_rms));
        Radar_msg[0].Lat_Pos_Stdev[i]= Obj_Rms((objf_2[i].obj_dist_lat_rms));
        Radar_msg[0].Long_Vel_Stdev[i]= Obj_Rms((objf_2[i].obj_vrel_long_rms));
        Radar_msg[0].Lat_Vel_Stdev[i]= Obj_Rms((objf_2[i].obj_vrel_lat_rms));
        Radar_msg[0].Long_Acc_Stdev[i]= Obj_Rms((objf_2[i].obj_arel_long_rms));
        Radar_msg[0].Lat_Acc_Stdev[i]= Obj_Rms((objf_2[i].obj_arel_lat_rms));
        Radar_msg[0].Orientation_Stdev[i]= ObjOri_Rms((objf_2[i].obj_orientation_rms));
        if(objf_2[i].obj_dist_long_rms ==0)
        {
           Radar_msg[0].Long_Pos_Stdev[i]=0;
        }
        if(objf_2[i].obj_dist_lat_rms ==0)
        {
            Radar_msg[0].Lat_Pos_Stdev[i]=0;
        }
        if(objf_2[i].obj_vrel_long_rms ==0)
        {
            Radar_msg[0].Long_Vel_Stdev[i]=0;
        }
        if(objf_2[i].obj_vrel_lat_rms ==0)
        {
            Radar_msg[0].Lat_Vel_Stdev[i]=0;
        }
        if(objf_2[i].obj_arel_long_rms ==0)
        {
            Radar_msg[0].Long_Acc_Stdev[i]=0;
        }
        if(objf_2[i].obj_arel_lat_rms ==0)
        {
            Radar_msg[0].Lat_Acc_Stdev[i]=0;
        }
        if((objf_2[i].obj_orientation_rms) ==0)
        {
            Radar_msg[0].Orientation_Stdev[i]=0;
        }
        Radar_msg[0].Prob_Exist[i]= (Radar_Obj_Exist_Prob)(objf_2[i].obj_prob_of_exist);
        Radar_msg[0].Meas_State[i]= (Radar_Meas_State)(objf_2[i].obj_meas_state);
    }
    //Obj_3_Extended 67d
    for(i=0;i<obj_max_num;i++)
    {
        Radar_msg[0].Long_Acc[i]= (objf_3[i].obj_arel_long)*0.01-10;
        Radar_msg[0].Lat_Acc[i]= (objf_3[i].obj_arel_lat)*0.01-2.5;
        Radar_msg[0].Orientation_Angle[i]= (objf_3[i].obj_orientation_angle)*0.4-180;
        if(objf_3[i].obj_arel_long ==0)
        {
            Radar_msg[0].Long_Acc[i]=0;
        }
        if(objf_3[i].obj_arel_lat ==0)
        {
            Radar_msg[0].Lat_Acc[i]=0;
        }
        if(objf_3[i].obj_orientation_angle ==0)
        {
            Radar_msg[0].Orientation_Angle[i]=0;
        }
        Radar_msg[0].Type[i]= (Radar_Obj_Type)(objf_3[i].obj_class);
        Radar_msg[0].Length[i]= (objf_3[i].obj_length)*0.2;
        Radar_msg[0].Width[i]= (objf_3[i].obj_width)*0.2;
    }
    publishRadarObjectList(pub_radarf_msgs_, Radar_msg[0]);
    mutex_queue_radarf_.lock();
    if (!queue_radarf_object_.empty())
        queue_radarf_object_.pop();
    queue_radarf_object_.push(Radar_msg[0]);
    mutex_queue_radarf_.unlock();
}

void global::perception::radar::Perception_Radar_Node::publishRadarB(void *data)
{
    unsigned char* buffer = (unsigned char*)data;
    uint8_t i;
    //67a
    ars408_67_obj_0_status_unpack(&objb_0,&buffer[0],8);
    //67b
    for(i=0;i<32;i++)
    {
        ars408_67_obj_1_general_unpack(&objb_1[i],&buffer[18+i*8],8);
    }
    //67C
    for(i=0;i<32;i++)
    {
        ars408_67_obj_2_quality_unpack(&objb_2[i],&buffer[426+i*8],8);
    }
    //67d
    for(i=0;i<32;i++)
    {
        ars408_67_obj_3_extended_unpack(&objb_3[i],&buffer[834+i*8],8);
    }
    //67a
    Radar_msg[1].objNum=objb_0.obj_nof_objects;
    Radar_msg[1].streamTxCnt=objb_0.obj_meas_counter;
    //67b Obj_1_General
    for(i=0;i<obj_max_num;i++)
    {
        Radar_msg[1].ID[i]=objb_1[i].obj_id;
        Radar_msg[1].Lat_Pos[i]= (objb_1[i].obj_dist_lat)*0.2-204.6;
        Radar_msg[1].Long_Pos[i]= (objb_1[i].obj_dist_long)*0.2-500;
        Radar_msg[1].Lat_Vel[i]= (objb_1[i].obj_vrel_lat)*0.25-64;
        Radar_msg[1].Long_Vel[i]= (objb_1[i].obj_vrel_long)*0.25-128;
        Radar_msg[1].RCS[i]= (objb_1[i].obj_rcs)*0.5-64;
        Radar_msg[1].Dyn_Prop[i]= (Radar_DynProp)(objb_1[i].obj_dyn_prop);
        if(objb_1[i].obj_dist_lat ==0)
        {
            Radar_msg[1].Lat_Pos[i]=0;
        }
        if(objb_1[i].obj_dist_long ==0)
        {
            Radar_msg[1].Long_Pos[i]=0;
        }
        if(objb_1[i].obj_vrel_lat ==0)
        {
            Radar_msg[1].Lat_Vel[i]=0;
        }
        if(objb_1[i].obj_vrel_long ==0)
        {
            Radar_msg[1].Long_Vel[i]=0;
        }
        if(objb_1[i].obj_rcs ==0)
        {
            Radar_msg[1].RCS[i]=0;
        }
    }
    //Obj_2_Quality 67c
    for(i=0;i<obj_max_num;i++)
    {
        Radar_msg[1].Long_Pos_Stdev[i]=Obj_Rms((objb_2[i].obj_dist_long_rms));
        Radar_msg[1].Lat_Pos_Stdev[i]=Obj_Rms((objb_2[i].obj_dist_lat_rms));
        Radar_msg[1].Long_Vel_Stdev[i]=Obj_Rms((objb_2[i].obj_vrel_long_rms));
        Radar_msg[1].Lat_Vel_Stdev[i]=Obj_Rms((objb_2[i].obj_vrel_lat_rms));
        Radar_msg[1].Long_Acc_Stdev[i]=Obj_Rms((objb_2[i].obj_arel_long_rms));
        Radar_msg[1].Lat_Acc_Stdev[i]=Obj_Rms((objb_2[i].obj_arel_lat_rms));
        Radar_msg[1].Orientation_Stdev[i]= ObjOri_Rms((objb_2[i].obj_orientation_rms));
        if(objb_2[i].obj_dist_long_rms ==0)
        {
            Radar_msg[1].Long_Pos_Stdev[i]=0;
        }
        if(objb_2[i].obj_dist_lat_rms ==0)
        {
            Radar_msg[1].Lat_Pos_Stdev[i]=0;
        }
        if(objb_2[i].obj_vrel_long_rms ==0)
        {
            Radar_msg[1].Long_Vel_Stdev[i]=0;
        }
        if(objb_2[i].obj_vrel_lat_rms ==0)
        {
            Radar_msg[1].Lat_Vel_Stdev[i]=0;
        }
        if(objb_2[i].obj_arel_long_rms ==0)
        {
            Radar_msg[1].Long_Acc_Stdev[i]=0;
        }
        if(objb_2[i].obj_arel_lat_rms ==0)
        {
            Radar_msg[1].Lat_Acc_Stdev[i]=0;
        }
        if((objb_2[i].obj_orientation_rms) ==0)
        {
            Radar_msg[1].Orientation_Stdev[i]=0;
        }
        Radar_msg[1].Prob_Exist[i]= (Radar_Obj_Exist_Prob)(objb_2[i].obj_prob_of_exist);
        Radar_msg[1].Meas_State[i]= (Radar_Meas_State)(objb_2[i].obj_meas_state);
    }
    //Obj_3_Extended 67d
    for(i=0;i<obj_max_num;i++)
    {
        Radar_msg[1].Long_Acc[i]= (objb_3[i].obj_arel_long)*0.01-10;
        Radar_msg[1].Lat_Acc[i]= (objb_3[i].obj_arel_lat)*0.01-2.5;
        Radar_msg[1].Orientation_Angle[i]= (objb_3[i].obj_orientation_angle)*0.4-180;
        if(objb_3[i].obj_arel_long ==0)
        {
            Radar_msg[1].Long_Acc[i]=0;
        }
        if(objb_3[i].obj_arel_lat ==0)
        {
            Radar_msg[1].Lat_Acc[i]=0;
        }
        if(objb_3[i].obj_orientation_angle ==0)
        {
            Radar_msg[1].Orientation_Angle[i]=0;
        }
        Radar_msg[1].Type[i]= (Radar_Obj_Type)(objb_3[i].obj_class);
        Radar_msg[1].Length[i]= (objb_3[i].obj_length)*0.2;
        Radar_msg[1].Width[i]= (objb_3[i].obj_width)*0.2;
    }
    // Ros_Show(&Radar_msg[1]);
    publishRadarObjectList(pub_radarb_msgs_, Radar_msg[1]);
    mutex_queue_radarb_.lock();
    if (!queue_radarb_object_.empty())
        queue_radarb_object_.pop();
    queue_radarb_object_.push(Radar_msg[1]);
    mutex_queue_radarb_.unlock();
}

void global::perception::radar::Perception_Radar_Node::publishRadarFL(void *data)
{
    unsigned char* buffer = (unsigned char*)data;
    uint8_t i;

    //66a
    ars408_66_obj_0_status_unpack(&objfl_0,&buffer[0],8);

    //66b
    for(i=0;i<32;i++)
    {
        ars408_66_obj_1_general_unpack(&objfl_1[i],&buffer[18+i*8],8);
    }
    //66C
    for(i=0;i<32;i++)
    {
        ars408_66_obj_2_quality_unpack(&objfl_2[i],&buffer[426+i*8],8);
    }
    //66d
    for(i=0;i<32;i++)
    {
        ars408_66_obj_3_extended_unpack(&objfl_3[i],&buffer[834+i*8],8);
    }

    //60a
    Radar_msg[2].objNum=objfl_0.obj_nof_objects;
    Radar_msg[2].streamTxCnt=objfl_0.obj_meas_counter;

    //60bs1_General
    for(i=0;i<obj_max_num;i++)
    {

        Radar_msg[2].ID[i]= objfl_1[i].obj_id;

        Radar_msg[2].Lat_Pos[i]= (objfl_1[i].obj_dist_lat)*0.2-204.6;
        Radar_msg[2].Long_Pos[i]= (objfl_1[i].obj_dist_long)*0.2-500;
        Radar_msg[2].Lat_Vel[i]= (objfl_1[i].obj_vrel_lat)*0.25-64;
        Radar_msg[2].Long_Vel[i]= (objfl_1[i].obj_vrel_long)*0.25-128;
        Radar_msg[2].RCS[i]= (objfl_1[i].obj_rcs)*0.5-64;

        Radar_msg[2].Dyn_Prop[i]= (Radar_DynProp)(objfl_1[i].obj_dyn_prop);

        if(objfl_1[i].obj_dist_lat ==0)
        {
            Radar_msg[2].Lat_Pos[i]=0;
        }
        if(objfl_1[i].obj_dist_long ==0)
        {
            Radar_msg[2].Long_Pos[i]=0;
        }
        if(objfl_1[i].obj_vrel_lat ==0)
        {
            Radar_msg[2].Lat_Vel[i]=0;
        }
        if(objfl_1[i].obj_vrel_long ==0)
        {
            Radar_msg[2].Long_Vel[i]=0;
        }

        if(objfl_1[i].obj_rcs ==0)
        {
            Radar_msg[2].RCS[i]=0;
        }


    }

    //Obj_2_Quality 67c
    for(i=0;i<obj_max_num;i++)
    {
        Radar_msg[2].Long_Pos_Stdev[i]= Obj_Rms((objfl_2[i].obj_dist_long_rms));
        Radar_msg[2].Lat_Pos_Stdev[i]= Obj_Rms((objfl_2[i].obj_dist_lat_rms));
        Radar_msg[2].Long_Vel_Stdev[i]= Obj_Rms((objfl_2[i].obj_vrel_long_rms));
        Radar_msg[2].Lat_Vel_Stdev[i]= Obj_Rms((objfl_2[i].obj_vrel_lat_rms));
        Radar_msg[2].Long_Acc_Stdev[i]= Obj_Rms((objfl_2[i].obj_arel_long_rms));
        Radar_msg[2].Lat_Acc_Stdev[i]= Obj_Rms((objfl_2[i].obj_arel_lat_rms));
        Radar_msg[2].Orientation_Stdev[i]= ObjOri_Rms((objfl_2[i].obj_orientation_rms));


        if(objfl_2[i].obj_dist_long_rms ==0)
        {
           Radar_msg[2].Long_Pos_Stdev[i]=0;
        }
        if(objfl_2[i].obj_dist_lat_rms ==0)
        {
            Radar_msg[2].Lat_Pos_Stdev[i]=0;
        }
        if(objfl_2[i].obj_vrel_long_rms ==0)
        {
            Radar_msg[2].Long_Vel_Stdev[i]=0;
        }
        if(objfl_2[i].obj_vrel_lat_rms ==0)
        {
            Radar_msg[2].Lat_Vel_Stdev[i]=0;
        }

        if(objfl_2[i].obj_arel_long_rms ==0)
        {
            Radar_msg[2].Long_Acc_Stdev[i]=0;
        }
        if(objfl_2[i].obj_arel_lat_rms ==0)
        {
            Radar_msg[2].Lat_Acc_Stdev[i]=0;
        }

        if((objfl_2[i].obj_orientation_rms) ==0)
        {
            Radar_msg[2].Orientation_Stdev[i]=0;
        }

        Radar_msg[2].Prob_Exist[i]= (Radar_Obj_Exist_Prob)(objfl_2[i].obj_prob_of_exist);
        Radar_msg[2].Meas_State[i]= (Radar_Meas_State)(objfl_2[i].obj_meas_state);

    }
    //Obj_3_Extended 67d
    for(i=0;i<obj_max_num;i++)
    {
        Radar_msg[2].Long_Acc[i]= (objfl_3[i].obj_arel_long)*0.01-10;
        Radar_msg[2].Lat_Acc[i]= (objfl_3[i].obj_arel_lat)*0.01-2.5;
        Radar_msg[2].Orientation_Angle[i]= (objfl_3[i].obj_orientation_angle)*0.4-180;

        if(objfl_3[i].obj_arel_long ==0)
        {
            Radar_msg[2].Long_Acc[i]=0;
        }
        if(objfl_3[i].obj_arel_lat ==0)
        {
            Radar_msg[2].Lat_Acc[i]=0;
        }
        if(objfl_3[i].obj_orientation_angle ==0)
        {
            Radar_msg[2].Orientation_Angle[i]=0;
        }

        Radar_msg[2].Type[i]= (Radar_Obj_Type)(objfl_3[i].obj_class);
        Radar_msg[2].Length[i]= (objfl_3[i].obj_length)*0.2;
        Radar_msg[2].Width[i]= (objfl_3[i].obj_width)*0.2;
    }

//        Ros_Show(&Radar_msg[2]);
    publishRadarObjectList(pub_radarfl_msgs_, Radar_msg[2]);
    mutex_queue_radarfl_.lock();
    if (!queue_radarfl_object_.empty())
        queue_radarfl_object_.pop();
    queue_radarfl_object_.push(Radar_msg[2]);
    mutex_queue_radarfl_.unlock();
}

void global::perception::radar::Perception_Radar_Node::publishRadarFR(void *data)
{
    unsigned char* buffer = (unsigned char*)data;
    uint8_t i;

    //65a
    ars408_65_obj_0_status_unpack(&objfr_0,&buffer[0],8);

    //65b
    for(i=0;i<32;i++)
    {
        ars408_65_obj_1_general_unpack(&objfr_1[i],&buffer[18+i*8],8);
    }
    //65C
    for(i=0;i<32;i++)
    {
        ars408_65_obj_2_quality_unpack(&objfr_2[i],&buffer[426+i*8],8);
    }
    //65d
    for(i=0;i<32;i++)
    {
        ars408_65_obj_3_extended_unpack(&objfr_3[i],&buffer[834+i*8],8);
    }

    //60a
    Radar_msg[3].objNum=objfr_0.obj_nof_objects;
    Radar_msg[3].streamTxCnt=objfr_0.obj_meas_counter;

    //65b Obj_1_General
    for(i=0;i<obj_max_num;i++)
    {

        Radar_msg[3].ID[i]= objfr_1[i].obj_id;

        Radar_msg[3].Lat_Pos[i]= (objfr_1[i].obj_dist_lat)*0.2-204.6;
        Radar_msg[3].Long_Pos[i]= (objfr_1[i].obj_dist_long)*0.2-500;
        Radar_msg[3].Lat_Vel[i]= (objfr_1[i].obj_vrel_lat)*0.25-64;
        Radar_msg[3].Long_Vel[i]= (objfr_1[i].obj_vrel_long)*0.25-128;
        Radar_msg[3].RCS[i]= (objfr_1[i].obj_rcs)*0.5-64;

        Radar_msg[3].Dyn_Prop[i]= (Radar_DynProp)(objfr_1[i].obj_dyn_prop);

        if(objfr_1[i].obj_dist_lat ==0)
        {
            Radar_msg[3].Lat_Pos[i]=0;
        }
        if(objfr_1[i].obj_dist_long ==0)
        {
            Radar_msg[3].Long_Pos[i]=0;
        }
        if(objfr_1[i].obj_vrel_lat ==0)
        {
            Radar_msg[3].Lat_Vel[i]=0;
        }
        if(objfr_1[i].obj_vrel_long ==0)
        {
            Radar_msg[3].Long_Vel[i]=0;
        }

        if(objfr_1[i].obj_rcs ==0)
        {
            Radar_msg[3].RCS[i]=0;
        }


    }

    //Obj_2_Quality 67c
    for(i=0;i<obj_max_num;i++)
    {
        Radar_msg[3].Long_Pos_Stdev[i]= Obj_Rms((objfr_2[i].obj_dist_long_rms));
        Radar_msg[3].Lat_Pos_Stdev[i]= Obj_Rms((objfr_2[i].obj_dist_lat_rms));
        Radar_msg[3].Long_Vel_Stdev[i]= Obj_Rms((objfr_2[i].obj_vrel_long_rms));
        Radar_msg[3].Lat_Vel_Stdev[i]= Obj_Rms((objfr_2[i].obj_vrel_lat_rms));
        Radar_msg[3].Long_Acc_Stdev[i]= Obj_Rms((objfr_2[i].obj_arel_long_rms));
        Radar_msg[3].Lat_Acc_Stdev[i]= Obj_Rms((objfr_2[i].obj_arel_lat_rms));
        Radar_msg[3].Orientation_Stdev[i]= ObjOri_Rms((objfr_2[i].obj_orientation_rms));


        if(objfr_2[i].obj_dist_long_rms ==0)
        {
           Radar_msg[3].Long_Pos_Stdev[i]=0;
        }
        if(objfr_2[i].obj_dist_lat_rms ==0)
        {
            Radar_msg[3].Lat_Pos_Stdev[i]=0;
        }
        if(objfr_2[i].obj_vrel_long_rms ==0)
        {
            Radar_msg[3].Long_Vel_Stdev[i]=0;
        }
        if(objfr_2[i].obj_vrel_lat_rms ==0)
        {
            Radar_msg[3].Lat_Vel_Stdev[i]=0;
        }

        if(objfr_2[i].obj_arel_long_rms ==0)
        {
            Radar_msg[3].Long_Acc_Stdev[i]=0;
        }
        if(objfr_2[i].obj_arel_lat_rms ==0)
        {
            Radar_msg[3].Lat_Acc_Stdev[i]=0;
        }

        if((objfr_2[i].obj_orientation_rms) ==0)
        {
            Radar_msg[3].Orientation_Stdev[i]=0;
        }

        Radar_msg[3].Prob_Exist[i]= (Radar_Obj_Exist_Prob)(objfr_2[i].obj_prob_of_exist);
        Radar_msg[3].Meas_State[i]= (Radar_Meas_State)(objfr_2[i].obj_meas_state);

    }
    //Obj_3_Extended 67d
    for(i=0;i<obj_max_num;i++)
    {
        Radar_msg[3].Long_Acc[i]= (objfr_3[i].obj_arel_long)*0.01-10;
        Radar_msg[3].Lat_Acc[i]= (objfr_3[i].obj_arel_lat)*0.01-2.5;
        Radar_msg[3].Orientation_Angle[i]= (objfr_3[i].obj_orientation_angle)*0.4-180;

        if(objfr_3[i].obj_arel_long ==0)
        {
            Radar_msg[3].Long_Acc[i]=0;
        }
        if(objfr_3[i].obj_arel_lat ==0)
        {
            Radar_msg[3].Lat_Acc[i]=0;
        }
        if(objfr_3[i].obj_orientation_angle ==0)
        {
            Radar_msg[3].Orientation_Angle[i]=0;
        }

        Radar_msg[3].Type[i]= (Radar_Obj_Type)(objfr_3[i].obj_class);
        Radar_msg[3].Length[i]= (objfr_3[i].obj_length)*0.2;
        Radar_msg[3].Width[i]= (objfr_3[i].obj_width)*0.2;
    }
//    Ros_Show(&Radar_msg[3]);
    publishRadarObjectList(pub_radarfr_msgs_, Radar_msg[3]);
    mutex_queue_radarfr_.lock();
    if (!queue_radarfr_object_.empty())
        queue_radarfr_object_.pop();
    queue_radarfr_object_.push(Radar_msg[3]);
    mutex_queue_radarfr_.unlock();
}

void global::perception::radar::Perception_Radar_Node::publishRadarBL(void *data)
{
    unsigned char* buffer = (unsigned char*)data;
    uint8_t i;

    //64a
    ars408_64_obj_0_status_unpack(&objbl_0,&buffer[0],8);

    //64b
    for(i=0;i<32;i++)
    {
        ars408_64_obj_1_general_unpack(&objbl_1[i],&buffer[18+i*8],8);
    }
    //64C
    for(i=0;i<32;i++)
    {
        ars408_64_obj_2_quality_unpack(&objbl_2[i],&buffer[426+i*8],8);
    }
    //64d
    for(i=0;i<32;i++)
    {
        ars408_64_obj_3_extended_unpack(&objbl_3[i],&buffer[834+i*8],8);
    }

    //64a
    Radar_msg[4].objNum=objbl_0.obj_nof_objects;
    Radar_msg[4].streamTxCnt=objbl_0.obj_meas_counter;

    //64b Obj_1_General
    for(i=0;i<obj_max_num;i++)
    {

        Radar_msg[4].ID[i]= objbl_1[i].obj_id;

        Radar_msg[4].Lat_Pos[i]= (objbl_1[i].obj_dist_lat)*0.2-204.6;
        Radar_msg[4].Long_Pos[i]= (objbl_1[i].obj_dist_long)*0.2-500;
        Radar_msg[4].Lat_Vel[i]= (objbl_1[i].obj_vrel_lat)*0.25-64;
        Radar_msg[4].Long_Vel[i]= (objbl_1[i].obj_vrel_long)*0.25-128;
        Radar_msg[4].RCS[i]= (objbl_1[i].obj_rcs)*0.5-64;

        Radar_msg[4].Dyn_Prop[i]= (Radar_DynProp)(objbl_1[i].obj_dyn_prop);

        if(objbl_1[i].obj_dist_lat ==0)
        {
            Radar_msg[4].Lat_Pos[i]=0;
        }
        if(objbl_1[i].obj_dist_long ==0)
        {
            Radar_msg[4].Long_Pos[i]=0;
        }
        if(objbl_1[i].obj_vrel_lat ==0)
        {
            Radar_msg[4].Lat_Vel[i]=0;
        }
        if(objbl_1[i].obj_vrel_long ==0)
        {
            Radar_msg[4].Long_Vel[i]=0;
        }

        if(objbl_1[i].obj_rcs ==0)
        {
            Radar_msg[4].RCS[i]=0;
        }


    }

    //Obj_2_Quality 64c
    for(i=0;i<obj_max_num;i++)
    {
        Radar_msg[4].Long_Pos_Stdev[i]= Obj_Rms((objbl_2[i].obj_dist_long_rms));
        Radar_msg[4].Lat_Pos_Stdev[i]= Obj_Rms((objbl_2[i].obj_dist_lat_rms));
        Radar_msg[4].Long_Vel_Stdev[i]= Obj_Rms((objbl_2[i].obj_vrel_long_rms));
        Radar_msg[4].Lat_Vel_Stdev[i]= Obj_Rms((objbl_2[i].obj_vrel_lat_rms));
        Radar_msg[4].Long_Acc_Stdev[i]= Obj_Rms((objbl_2[i].obj_arel_long_rms));
        Radar_msg[4].Lat_Acc_Stdev[i]= Obj_Rms((objbl_2[i].obj_arel_lat_rms));
        Radar_msg[4].Orientation_Stdev[i]= ObjOri_Rms((objbl_2[i].obj_orientation_rms));


        if(objbl_2[i].obj_dist_long_rms ==0)
        {
           Radar_msg[4].Long_Pos_Stdev[i]=0;
        }
        if(objbl_2[i].obj_dist_lat_rms ==0)
        {
            Radar_msg[4].Lat_Pos_Stdev[i]=0;
        }
        if(objbl_2[i].obj_vrel_long_rms ==0)
        {
            Radar_msg[4].Long_Vel_Stdev[i]=0;
        }
        if(objbl_2[i].obj_vrel_lat_rms ==0)
        {
            Radar_msg[4].Lat_Vel_Stdev[i]=0;
        }

        if(objbl_2[i].obj_arel_long_rms ==0)
        {
            Radar_msg[4].Long_Acc_Stdev[i]=0;
        }
        if(objbl_2[i].obj_arel_lat_rms ==0)
        {
            Radar_msg[4].Lat_Acc_Stdev[i]=0;
        }

        if((objbl_2[i].obj_orientation_rms) ==0)
        {
            Radar_msg[4].Orientation_Stdev[i]=0;
        }

        Radar_msg[4].Prob_Exist[i]= (Radar_Obj_Exist_Prob)(objbl_2[i].obj_prob_of_exist);
        Radar_msg[4].Meas_State[i]= (Radar_Meas_State)(objbl_2[i].obj_meas_state);

    }
    //Obj_3_Extended 64d
    for(i=0;i<obj_max_num;i++)
    {
        Radar_msg[4].Long_Acc[i]= (objbl_3[i].obj_arel_long)*0.01-10;
        Radar_msg[4].Lat_Acc[i]= (objbl_3[i].obj_arel_lat)*0.01-2.5;
        Radar_msg[4].Orientation_Angle[i]= (objbl_3[i].obj_orientation_angle)*0.4-180;

        if(objbl_3[i].obj_arel_long ==0)
        {
            Radar_msg[4].Long_Acc[i]=0;
        }
        if(objbl_3[i].obj_arel_lat ==0)
        {
            Radar_msg[4].Lat_Acc[i]=0;
        }
        if(objbl_3[i].obj_orientation_angle ==0)
        {
            Radar_msg[4].Orientation_Angle[i]=0;
        }

        Radar_msg[4].Type[i]= (Radar_Obj_Type)(objbl_3[i].obj_class);
        Radar_msg[4].Length[i]= (objbl_3[i].obj_length)*0.2;
        Radar_msg[4].Width[i]= (objbl_3[i].obj_width)*0.2;
    }

//    Ros_Show(&Radar_msg[4]);
    publishRadarObjectList(pub_radarbl_msgs_, Radar_msg[4]);
    mutex_queue_radarbl_.lock();
    if (!queue_radarbl_object_.empty())
        queue_radarbl_object_.pop();
    queue_radarbl_object_.push(Radar_msg[4]);
    mutex_queue_radarbl_.unlock();
}

void global::perception::radar::Perception_Radar_Node::publishRadarBR(void *data)
{
    unsigned char* buffer = (unsigned char*)data;
    uint8_t i;

    //63a
    ars408_63_obj_0_status_unpack(&objbr_0,&buffer[0],8);

    //63b
    for(i=0;i<32;i++)
    {
        ars408_63_obj_1_general_unpack(&objbr_1[i],&buffer[18+i*8],8);
    }
    //63C
    for(i=0;i<32;i++)
    {
        ars408_63_obj_2_quality_unpack(&objbr_2[i],&buffer[426+i*8],8);
    }
    //63d
    for(i=0;i<32;i++)
    {
        ars408_63_obj_3_extended_unpack(&objbr_3[i],&buffer[834+i*8],8);
    }

    //63a
    Radar_msg[5].objNum=objbr_0.obj_nof_objects;
    Radar_msg[5].streamTxCnt=objbr_0.obj_meas_counter;

    //64b Obj_1_General
    for(i=0;i<obj_max_num;i++)
    {

        Radar_msg[5].ID[i]= objbr_1[i].obj_id;

        Radar_msg[5].Lat_Pos[i]= (objbr_1[i].obj_dist_lat)*0.2-204.6;
        Radar_msg[5].Long_Pos[i]= (objbr_1[i].obj_dist_long)*0.2-500;
        Radar_msg[5].Lat_Vel[i]= (objbr_1[i].obj_vrel_lat)*0.25-64;
        Radar_msg[5].Long_Vel[i]= (objbr_1[i].obj_vrel_long)*0.25-128;
        Radar_msg[5].RCS[i]= (objbr_1[i].obj_rcs)*0.5-64;

        Radar_msg[5].Dyn_Prop[i]= (Radar_DynProp)(objbr_1[i].obj_dyn_prop);

        if(objbr_1[i].obj_dist_lat ==0)
        {
            Radar_msg[5].Lat_Pos[i]=0;
        }
        if(objbr_1[i].obj_dist_long ==0)
        {
            Radar_msg[5].Long_Pos[i]=0;
        }
        if(objbr_1[i].obj_vrel_lat ==0)
        {
            Radar_msg[5].Lat_Vel[i]=0;
        }
        if(objbr_1[i].obj_vrel_long ==0)
        {
            Radar_msg[5].Long_Vel[i]=0;
        }

        if(objbr_1[i].obj_rcs ==0)
        {
            Radar_msg[5].RCS[i]=0;
        }


    }

    //Obj_2_Quality 64c
    for(i=0;i<obj_max_num;i++)
    {
        Radar_msg[5].Long_Pos_Stdev[i]= Obj_Rms((objbr_2[i].obj_dist_long_rms));
        Radar_msg[5].Lat_Pos_Stdev[i]= Obj_Rms((objbr_2[i].obj_dist_lat_rms));
        Radar_msg[5].Long_Vel_Stdev[i]= Obj_Rms((objbr_2[i].obj_vrel_long_rms));
        Radar_msg[5].Lat_Vel_Stdev[i]= Obj_Rms((objbr_2[i].obj_vrel_lat_rms));
        Radar_msg[5].Long_Acc_Stdev[i]= Obj_Rms((objbr_2[i].obj_arel_long_rms));
        Radar_msg[5].Lat_Acc_Stdev[i]= Obj_Rms((objbr_2[i].obj_arel_lat_rms));
        Radar_msg[5].Orientation_Stdev[i]= ObjOri_Rms((objbr_2[i].obj_orientation_rms));


        if(objbr_2[i].obj_dist_long_rms ==0)
        {
           Radar_msg[5].Long_Pos_Stdev[i]=0;
        }
        if(objbr_2[i].obj_dist_lat_rms ==0)
        {
            Radar_msg[5].Lat_Pos_Stdev[i]=0;
        }
        if(objbr_2[i].obj_vrel_long_rms ==0)
        {
            Radar_msg[5].Long_Vel_Stdev[i]=0;
        }
        if(objbr_2[i].obj_vrel_lat_rms ==0)
        {
            Radar_msg[5].Lat_Vel_Stdev[i]=0;
        }

        if(objbr_2[i].obj_arel_long_rms ==0)
        {
            Radar_msg[5].Long_Acc_Stdev[i]=0;
        }
        if(objbr_2[i].obj_arel_lat_rms ==0)
        {
            Radar_msg[5].Lat_Acc_Stdev[i]=0;
        }

        if((objbr_2[i].obj_orientation_rms) ==0)
        {
            Radar_msg[5].Orientation_Stdev[i]=0;
        }

        Radar_msg[5].Prob_Exist[i]= (Radar_Obj_Exist_Prob)(objbr_2[i].obj_prob_of_exist);
        Radar_msg[5].Meas_State[i]= (Radar_Meas_State)(objbr_2[i].obj_meas_state);

    }
    //Obj_3_Extended 64d
    for(i=0;i<obj_max_num;i++)
    {
        Radar_msg[5].Long_Acc[i]= (objbr_3[i].obj_arel_long)*0.01-10;
        Radar_msg[5].Lat_Acc[i]= (objbr_3[i].obj_arel_lat)*0.01-2.5;
        Radar_msg[5].Orientation_Angle[i]= (objbr_3[i].obj_orientation_angle)*0.4-180;

        if(objbr_3[i].obj_arel_long ==0)
        {
            Radar_msg[5].Long_Acc[i]=0;
        }
        if(objbr_3[i].obj_arel_lat ==0)
        {
            Radar_msg[5].Lat_Acc[i]=0;
        }
        if(objbr_3[i].obj_orientation_angle ==0)
        {
            Radar_msg[5].Orientation_Angle[i]=0;
        }

        Radar_msg[5].Type[i]= (Radar_Obj_Type)(objbr_3[i].obj_class);
        Radar_msg[5].Length[i]= (objbr_3[i].obj_length)*0.2;
        Radar_msg[5].Width[i]= (objbr_3[i].obj_width)*0.2;
    }
    // Ros_Show(&Radar_msg[5]);
    publishRadarObjectList(pub_radarbr_msgs_, Radar_msg[5]);
    mutex_queue_radarbr_.lock();
    if (!queue_radarbr_object_.empty())
        queue_radarbr_object_.pop();
    queue_radarbr_object_.push(Radar_msg[5]);
    mutex_queue_radarbr_.unlock();
}

uint8_t global::perception::radar::Perception_Radar_Node::getRadarFUdpMessage(void* data)
{
    // memcpy(data,RadarF_tx_cmd_msg,RADARF_CMD_LENGHT);
    memcpy(data, RadarF_tx_msg, RADARF_CMD_LENGHT);
    return RADARF_CMD_LENGHT;
}

void global::perception::radar::Perception_Radar_Node::RadarFSendUdp()
{
    uint8_t msgLength = getRadarFUdpMessage((void*)RadarF_tx_cmd_msg);
    RadarF_Socket->sendTo((void*)RadarF_tx_cmd_msg, msgLength, RadarUrl, RadarFL_Port);
}

void global::perception::radar::Perception_Radar_Node::Recv_InsMsg_callack(const InsMsg::ins_p2::ConstPtr &InsMsg)
{
    Global_InsMsg=*InsMsg;
    Radar_Yaw.radar_device_yaw_rate=(uint16_t)((Global_InsMsg.Heading_Speed)/0.01);
    ars408_67_yaw_rate_information_pack(&RadarF_tx_msg[0],&Radar_Yaw,8);
    RadarFSendUdp();
}

void global::perception::radar::Perception_Radar_Node::Recv_CdmMsg_callack(const VehicleMsg::cdm_cmd::ConstPtr &CdmMsg)
{
    Global_CdmMsg = *CdmMsg;
    Radar_SpeedInfo.radar_device_speed_direction=1;
    Radar_SpeedInfo.radar_device_speed=(uint16_t)((Global_CdmMsg.Veh_Spd)/0.1);
    ars408_67_speed_information_pack(&RadarF_tx_msg[8],&Radar_SpeedInfo,8);
}

// void global::perception::radar::Perception_Radar_Node::Ros_Show(Rad_Obj *Radar_msg)
// {
//     uint16_t i;


//     RadarObjs_msg.versionInfo=Radar_msg->versionInfo;
//     RadarObjs_msg.streamDataLen=Radar_msg->streamDataLen;
//     RadarObjs_msg.streamTxCnt=Radar_msg->streamTxCnt;
//     RadarObjs_msg.objNum=Radar_msg->objNum;
//     RadarObjs_msg.sourceTimeStamp=Radar_msg->sourceTimeStamp;

//     for(i=0;i<=obj_max_num;i++)
//     {
//         RadarObjs_msg.Obj[i].ID= i;
//         RadarObjs_msg.Obj[i].Lat_Pos= Radar_msg->Lat_Pos[i];
//         RadarObjs_msg.Obj[i].Long_Pos= Radar_msg->Long_Pos[i];

//         RadarObjs_msg.Obj[i].Lat_Vel= Radar_msg->Lat_Vel[i];
//         RadarObjs_msg.Obj[i].Long_Vel= Radar_msg->Long_Vel[i];

//         RadarObjs_msg.Obj[i].Dyn_Prop= Radar_msg->Dyn_Prop[i];
//         RadarObjs_msg.Obj[i].RCS= Radar_msg->RCS[i];

//         RadarObjs_msg.Obj[i].Meas_State= Radar_msg->Meas_State[i];
//         RadarObjs_msg.Obj[i].Long_Pos_Stdev= Radar_msg->Long_Pos_Stdev[i];

//         RadarObjs_msg.Obj[i].Lat_Pos_Stdev= Radar_msg->Lat_Pos_Stdev[i];
//         RadarObjs_msg.Obj[i].Long_Vel_Stdev= Radar_msg->Long_Vel_Stdev[i];

//         RadarObjs_msg.Obj[i].Lat_Vel_Stdev= Radar_msg->Lat_Vel_Stdev[i];
//         RadarObjs_msg.Obj[i].Long_Acc_Stdev= Radar_msg->Long_Acc_Stdev[i];

//         RadarObjs_msg.Obj[i].Lat_Acc_Stdev= Radar_msg->Lat_Acc_Stdev[i];
//         RadarObjs_msg.Obj[i].Orientation_Stdev= Radar_msg->Orientation_Stdev[i];

//         RadarObjs_msg.Obj[i].Prob_Exist= Radar_msg->Prob_Exist[i];
//         RadarObjs_msg.Obj[i].Orientation_Angle= Radar_msg->Orientation_Angle[i];

//         RadarObjs_msg.Obj[i].Length= Radar_msg->Length[i];
//         RadarObjs_msg.Obj[i].Width= Radar_msg->Width[i];

//         RadarObjs_msg.Obj[i].Long_Acc= Radar_msg->Long_Acc[i];
//         RadarObjs_msg.Obj[i].Lat_Acc= Radar_msg->Lat_Acc[i];
//         RadarObjs_msg.Obj[i].Obj_Type= Radar_msg->Type[i];
//     }
//     struct timeval get_tv;

//     gettimeofday(&get_tv,NULL);
//     RadarObjs_msg.stamp=get_tv.tv_sec*1000000 + get_tv.tv_usec;

//     // ros_pub_RadarObjs_info.publish(RadarObjs_msg);
// //    ros_pub_RadarObjs_info.publish(RadarObjs_msg);
// }

void global::perception::radar::Perception_Radar_Node::getParameter(ros::NodeHandle& public_node, ros::NodeHandle& private_node)
{
    std::vector<float> transform_matrix_radarf, transform_matrix_radarb;
    std::vector<float> transform_matrix_radarfl, transform_matrix_radarbl;
    std::vector<float> transform_matrix_radarfr, transform_matrix_radarbr;
    private_node.param("transform_matrix_radarf", transform_matrix_radarf, transform_matrix_radarf);
    private_node.param("transform_matrix_radarb", transform_matrix_radarb, transform_matrix_radarb);
    private_node.param("transform_matrix_radarfl", transform_matrix_radarfl, transform_matrix_radarfl);
    private_node.param("transform_matrix_radarbl", transform_matrix_radarbl, transform_matrix_radarbl);
    private_node.param("transform_matrix_radarfr", transform_matrix_radarfr, transform_matrix_radarfr);
    private_node.param("transform_matrix_radarbr", transform_matrix_radarbr, transform_matrix_radarbr);
    for (size_t i=0; i<4; i++)
        for (size_t j=0; j<4; j++)
        {
                transform_matrix_radarf_(i, j) = transform_matrix_radarf[4*i+j];
                transform_matrix_radarb_(i, j) = transform_matrix_radarb[4*i+j];
                transform_matrix_radarfl_(i, j) = transform_matrix_radarfl[4*i+j];
                transform_matrix_radarbl_(i, j) = transform_matrix_radarbl[4*i+j];
                transform_matrix_radarfr_(i, j) = transform_matrix_radarfr[4*i+j];
                transform_matrix_radarbr_(i, j) = transform_matrix_radarbr[4*i+j];
        }
    private_node.param("remove_duplication_threshold", remove_duplication_threshold_, remove_duplication_threshold_);
    std::string radarf_topic, radarb_topic, radarfl_topic, radarfr_topic, radarbl_topic, radarbr_topic, radar_fusion_topic;
    private_node.param("radarf_topic", radarf_topic, radarf_topic);
    private_node.param("radarb_topic", radarb_topic, radarb_topic);
    private_node.param("radarfl_topic", radarfl_topic, radarfl_topic);
    private_node.param("radarbl_topic", radarbl_topic, radarbl_topic);
    private_node.param("radarfr_topic", radarfr_topic, radarfr_topic);
    private_node.param("radarbr_topic", radarbr_topic, radarbr_topic);
    private_node.param("radar_fusion_topic", radar_fusion_topic, radar_fusion_topic);
    pub_radarf_msgs_ = public_node.advertise<radar_msgs::RadarObjectList>(radarf_topic, 100);
    pub_radarb_msgs_ = public_node.advertise<radar_msgs::RadarObjectList>(radarb_topic, 100);
    pub_radarfl_msgs_ = public_node.advertise<radar_msgs::RadarObjectList>(radarfl_topic, 100);
    pub_radarfr_msgs_ = public_node.advertise<radar_msgs::RadarObjectList>(radarfr_topic, 100);
    pub_radarbl_msgs_ = public_node.advertise<radar_msgs::RadarObjectList>(radarbl_topic, 100);
    pub_radarbr_msgs_ = public_node.advertise<radar_msgs::RadarObjectList>(radarbr_topic, 100);
    pub_radar_fusion_msgs_ = public_node.advertise<radar_msgs::RadarObjectList>(radar_fusion_topic, 100);
}

void global::perception::radar::Perception_Radar_Node::publishRadarObjectList(ros::Publisher& pub_radar_msgs, Rad_Obj& radar_object_message)
{
    radar_msgs::RadarObjectList radar_objectlist;
    radar_objectlist.header.stamp = ros::Time::now();
    radar_objectlist.header.frame_id = "perception";
    radar_objectlist.ObjNum = radar_object_message.objNum;
    // std::cout << "ObjNum = " << radar_objectlist.ObjNum << std::endl;
    uint8_t obj_num = radar_object_message.objNum > obj_max_num ? obj_max_num : radar_object_message.objNum;
    for (size_t i = 0; i < obj_num; i++)
    {
        radar_msgs::RadarObject radar_object;
        radar_object.ID = (uint32_t)radar_object_message.ID[i];
        // latitude 横向 y轴    longtitude 纵向 x轴
        float center_x = radar_object_message.Long_Pos[i];
        float center_y = radar_object_message.Lat_Pos[i];
        float length = radar_object_message.Length[i];
        float width = radar_object_message.Width[i];
        // std::cout << "center_x = " << center_x << std::endl;
        // std::cout << "center_y = " << center_y << std::endl;
        radar_object.Rel_Pos.x = center_x;
        radar_object.Rel_Pos.y = center_y;
        radar_object.Rel_Pos.z = 0;
        radar_object.Rel_Range.xMin = center_x;
        radar_object.Rel_Range.xMax = center_x + length;
        radar_object.Rel_Range.yMin = center_y - width / 2;
        radar_object.Rel_Range.yMax = center_y + width / 2;
        radar_object.Rel_Range.zMin = -1.0;
        radar_object.Rel_Range.zMax = 1.0;
        radar_object.Rel_Bbox[0].x = radar_object.Rel_Range.xMax;
        radar_object.Rel_Bbox[0].y = radar_object.Rel_Range.yMax;
        radar_object.Rel_Bbox[0].z = radar_object.Rel_Range.zMax;
        radar_object.Rel_Bbox[1].x = radar_object.Rel_Range.xMin;
        radar_object.Rel_Bbox[1].y = radar_object.Rel_Range.yMax;
        radar_object.Rel_Bbox[1].z = radar_object.Rel_Range.zMax;
        radar_object.Rel_Bbox[2].x = radar_object.Rel_Range.xMin;
        radar_object.Rel_Bbox[2].y = radar_object.Rel_Range.yMin;
        radar_object.Rel_Bbox[2].z = radar_object.Rel_Range.zMax;
        radar_object.Rel_Bbox[3].x = radar_object.Rel_Range.xMax;
        radar_object.Rel_Bbox[3].y = radar_object.Rel_Range.yMin;
        radar_object.Rel_Bbox[3].z = radar_object.Rel_Range.zMax;
        radar_object.Rel_Bbox[4].x = radar_object.Rel_Range.xMax;
        radar_object.Rel_Bbox[4].y = radar_object.Rel_Range.yMax;
        radar_object.Rel_Bbox[4].z = radar_object.Rel_Range.zMin;
        radar_object.Rel_Bbox[5].x = radar_object.Rel_Range.xMin;
        radar_object.Rel_Bbox[5].y = radar_object.Rel_Range.yMax;
        radar_object.Rel_Bbox[5].z = radar_object.Rel_Range.zMin;
        radar_object.Rel_Bbox[6].x = radar_object.Rel_Range.xMin;
        radar_object.Rel_Bbox[6].y = radar_object.Rel_Range.yMin;
        radar_object.Rel_Bbox[6].z = radar_object.Rel_Range.zMin;
        radar_object.Rel_Bbox[7].x = radar_object.Rel_Range.xMax;
        radar_object.Rel_Bbox[7].y = radar_object.Rel_Range.yMin;
        radar_object.Rel_Bbox[7].z = radar_object.Rel_Range.zMin;
        radar_object.Rel_Vel.x = radar_object_message.Long_Vel[i];
        radar_object.Rel_Vel.y = radar_object_message.Lat_Vel[i];
        radar_object.Rel_Vel.z = 0;
        radar_objectlist.ObjectList.push_back(radar_object);
    }
    pub_radar_msgs.publish(radar_objectlist);
}

void global::perception::radar::Perception_Radar_Node::radarFusion()
{
    while (true)
    {
        if (queue_radarf_object_.empty() || queue_radarb_object_.empty() ||
        queue_radarfl_object_.empty() || queue_radarbl_object_.empty() ||
        queue_radarfr_object_.empty() || queue_radarbr_object_.empty() )
        {
            usleep(5000);
        }
        else
        {
            std::cout << "radar fusion "  << std::endl;
            std::list<Radar_Object> object_list;
            global::perception::radar::Rad_Obj radarf, radarb, radarfl, radarbl, radarfr, radarbr;
            mutex_queue_radarf_.lock();
            radarf = queue_radarf_object_.front();
            queue_radarf_object_.pop();
            mutex_queue_radarf_.unlock();
            mutex_queue_radarb_.lock();
            radarb = queue_radarb_object_.front();
            queue_radarb_object_.pop();
            mutex_queue_radarb_.unlock();
            mutex_queue_radarfl_.lock();
            radarfl = queue_radarfl_object_.front();
            queue_radarfl_object_.pop();
            mutex_queue_radarfl_.unlock();
            mutex_queue_radarbl_.lock();
            radarbl = queue_radarbl_object_.front();
            queue_radarbl_object_.pop();
            mutex_queue_radarbl_.unlock();
            mutex_queue_radarfr_.lock();
            radarfr = queue_radarfr_object_.front();
            queue_radarfr_object_.pop();
            mutex_queue_radarfr_.unlock();
            mutex_queue_radarbr_.lock();
            radarbr = queue_radarbr_object_.front();
            queue_radarbr_object_.pop();
            mutex_queue_radarbr_.unlock();
            uint8_t obj_num = radarf.objNum > obj_max_num ? obj_max_num : radarf.objNum;
            for (size_t i=0; i<obj_num; i++)
            {
                global::perception::radar::Radar_Object radar_object(radarf, i, transform_matrix_radarf_, 0);
                object_list.push_back(radar_object);
            }
            obj_num = radarfl.objNum > obj_max_num ? obj_max_num : radarfl.objNum;
            for (size_t i=0; i<obj_num; i++)
            {
                global::perception::radar::Radar_Object radar_object(radarfl, i, transform_matrix_radarfl_, 0);
                object_list.push_back(radar_object);
            }
            obj_num = radarfr.objNum > obj_max_num ? obj_max_num : radarfr.objNum;
            for (size_t i=0; i<obj_num; i++)
            {
                global::perception::radar::Radar_Object radar_object(radarfr, i, transform_matrix_radarfr_, 0);
                object_list.push_back(radar_object);
            }
            obj_num = radarb.objNum > obj_max_num ? obj_max_num : radarb.objNum;
            for (size_t i=0; i<obj_num; i++)
            {
                global::perception::radar::Radar_Object radar_object(radarb, i, transform_matrix_radarb_, 0);
                object_list.push_back(radar_object);
            }
            obj_num = radarbl.objNum > obj_max_num ? obj_max_num : radarbl.objNum;
            for (size_t i=0; i<obj_num; i++)
            {
                global::perception::radar::Radar_Object radar_object(radarbl, i, transform_matrix_radarbl_, 0);
                object_list.push_back(radar_object);
            }
            obj_num = radarbr.objNum > obj_max_num ? obj_max_num : radarbr.objNum;
            for (size_t i=0; i<obj_num; i++)
            {
                global::perception::radar::Radar_Object radar_object(radarbr, i, transform_matrix_radarbr_, 0);
                object_list.push_back(radar_object);
            }
            removeDuplication(object_list, remove_duplication_threshold_);
            publishRadarFusionObjectList(object_list);
        }
    }
}

void global::perception::radar::Perception_Radar_Node::publishRadarFusionObjectList(std::list<Radar_Object>& object_list)
{
    radar_msgs::RadarObjectList radar_objectlist;
    radar_objectlist.header.stamp = ros::Time::now();
    radar_objectlist.header.frame_id = "perception";
    radar_objectlist.ObjNum = object_list.size();
    for (std::list<Radar_Object>::iterator iter = object_list.begin();
    iter != object_list.end(); iter++)
    {
        radar_msgs::RadarObject radar_object;
        radar_object.ID = (uint32_t)iter->object_id_;
        // latitude 横向 y轴    longtitude 纵向 x轴
        radar_object.Rel_Pos.x = iter->radar_point_.x_;
        radar_object.Rel_Pos.y = iter->radar_point_.y_;
        radar_object.Rel_Pos.z = iter->radar_point_.z_;
        radar_object.Rel_Range.xMin = iter->xmin_;
        radar_object.Rel_Range.xMax = iter->xmax_;
        radar_object.Rel_Range.yMin = iter->ymin_;
        radar_object.Rel_Range.yMax = iter->ymax_;
        radar_object.Rel_Range.zMin = iter->zmin_;
        radar_object.Rel_Range.zMax = iter->zmax_;
        for (size_t i=0; i<8; i++)
        {
            radar_object.Rel_Bbox[i].x = iter->bounding_box_[i].x_;
            radar_object.Rel_Bbox[i].y = iter->bounding_box_[i].y_;
            radar_object.Rel_Bbox[i].z = iter->bounding_box_[i].z_;
        }
        radar_object.Rel_Vel.x = iter->velocity_.x_;
        radar_object.Rel_Vel.y = iter->velocity_.y_;
        radar_object.Rel_Vel.z = iter->velocity_.z_;
        radar_object.Rel_Acc.x = iter->acceleration_.x_;
        radar_object.Rel_Acc.y = iter->acceleration_.y_;
        radar_object.Rel_Acc.z = iter->acceleration_.z_;
        radar_object.Rel_Pos_Stdev.x = iter->radar_point_stdev_.x_;
        radar_object.Rel_Pos_Stdev.y = iter->radar_point_stdev_.y_;
        radar_object.Rel_Pos_Stdev.z = iter->radar_point_stdev_.z_;
        radar_object.Rel_Vel_Stdev.x = iter->velocity_stdev_.x_;
        radar_object.Rel_Vel_Stdev.y = iter->velocity_stdev_.y_;
        radar_object.Rel_Vel_Stdev.z = iter->velocity_stdev_.z_;
        radar_object.Rel_Acc_Stdev.x = iter->acceleration_stdev_.x_;
        radar_object.Rel_Acc_Stdev.y = iter->acceleration_stdev_.y_;
        radar_object.Rel_Acc_Stdev.z = iter->acceleration_stdev_.z_;
        radar_object.Rel_Angle.heading = iter->angle_.yaw_;
        radar_object.Rel_Angle.pitch = iter->angle_.pitch_;
        radar_object.Rel_Angle.roll = iter->angle_.roll_;
        radar_object.Rel_Angle_Stdev.heading = iter->angle_stdev_.yaw_;
        radar_object.Rel_Angle_Stdev.pitch = iter->angle_stdev_.pitch_;
        radar_object.Rel_Angle_Stdev.roll = iter->angle_stdev_.roll_;
        radar_object.Length = iter->length_;
        radar_object.Width = iter->width_;
        radar_object.Height = iter->height_;
        radar_objectlist.ObjectList.push_back(radar_object);
    }
    pub_radar_fusion_msgs_.publish(radar_objectlist);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "perception_radar_node");
    ros::NodeHandle public_node;
    ros::NodeHandle private_node("~");
    global::perception::radar::Perception_Radar_Node perception_radar_node(
    public_node, private_node);
    ros::Rate loop_rate(20);
    ros::spin();
    return 0;
}