

// Created by cj on 2/24/20.
//

#include "Node.h"


UserNode::UserNode(int argc, char **pArgv)
        :m_Init_argc(argc),
         m_pInit_argv(pArgv)
{
    rosRunning = false;

}
UserNode::~UserNode() {
    if (ros::isStarted()) {
        ros::shutdown();
        ros::waitForShutdown();
    }
}
/**
 *  ros function
 * */
void UserNode::init()
{
    std::string name;

    ros::init(m_Init_argc, m_pInit_argv, "ros_Radar");

    if (!ros::master::check())
        return;

    ros::start();
    ros::Time::init();
    ros::NodeHandle nh;

    name = ros::this_node::getName();

    ros_pub_RadarObjs_info = nh.advertise < RadarMsg::RadarObjs> (MSG_TOPIC_RADAROBJ, 1);

    Sub_InsMsg=nh.subscribe <InsMsg::ins_p2> \
                        (MSG_TOPIC_GLOBALPOSE, 1, &UserNode::Recv_InsMsg_callack, this);

    Sub_CdmMsg=nh.subscribe <VehicleMsg::cdm_cmd> \
                        (MSG_TOPIC_CDM, 1, &UserNode::Recv_CdmMsg_callack, this);

    Radar_thread = new boost::thread(boost::bind(&UserNode::run, this));


}

void UserNode::run()
{
    ros::Rate loop_rate(100);
    ROS_INFO_STREAM("rosrunning...");
    rosRunning = true;
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}

bool UserNode::getReady(){
    return rosRunning;
}

void UserNode::copy_buff(uint8_t *dest, uint8_t *src, uint8_t len)
 {
        uint8_t i;

        for (i = 0; i < len; i++)
        {
                dest[i] = src[i];
        }
 }

float UserNode::ObjOri_Rms(uint8_t num)
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

float UserNode::Obj_Rms(uint8_t num)
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

void UserNode::Ros_Show(Rad_Obj *Radar_msg)
{
    uint16_t i;


    RadarObjs_msg.versionInfo=Radar_msg->versionInfo;
    RadarObjs_msg.streamDataLen=Radar_msg->streamDataLen;
    RadarObjs_msg.streamTxCnt=Radar_msg->streamTxCnt;
    RadarObjs_msg.objNum=Radar_msg->objNum;
    RadarObjs_msg.sourceTimeStamp=Radar_msg->sourceTimeStamp;

    for(i=0;i<=31;i++)
    {
        RadarObjs_msg.Obj[i].ID= i;
        RadarObjs_msg.Obj[i].Lat_Pos= Radar_msg->Lat_Pos[i];
        RadarObjs_msg.Obj[i].Long_Pos= Radar_msg->Long_Pos[i];

        RadarObjs_msg.Obj[i].Lat_Vel= Radar_msg->Lat_Vel[i];
        RadarObjs_msg.Obj[i].Long_Vel= Radar_msg->Long_Vel[i];

        RadarObjs_msg.Obj[i].Dyn_Prop= Radar_msg->Dyn_Prop[i];
        RadarObjs_msg.Obj[i].RCS= Radar_msg->RCS[i];

        RadarObjs_msg.Obj[i].Meas_State= Radar_msg->Meas_State[i];
        RadarObjs_msg.Obj[i].Long_Pos_Stdev= Radar_msg->Long_Pos_Stdev[i];

        RadarObjs_msg.Obj[i].Lat_Pos_Stdev= Radar_msg->Lat_Pos_Stdev[i];
        RadarObjs_msg.Obj[i].Long_Vel_Stdev= Radar_msg->Long_Vel_Stdev[i];

        RadarObjs_msg.Obj[i].Lat_Vel_Stdev= Radar_msg->Lat_Vel_Stdev[i];
        RadarObjs_msg.Obj[i].Long_Acc_Stdev= Radar_msg->Long_Acc_Stdev[i];

        RadarObjs_msg.Obj[i].Lat_Acc_Stdev= Radar_msg->Lat_Acc_Stdev[i];
        RadarObjs_msg.Obj[i].Orientation_Stdev= Radar_msg->Orientation_Stdev[i];

        RadarObjs_msg.Obj[i].Prob_Exist= Radar_msg->Prob_Exist[i];
        RadarObjs_msg.Obj[i].Orientation_Angle= Radar_msg->Orientation_Angle[i];

        RadarObjs_msg.Obj[i].Length= Radar_msg->Length[i];
        RadarObjs_msg.Obj[i].Width= Radar_msg->Width[i];

        RadarObjs_msg.Obj[i].Long_Acc= Radar_msg->Long_Acc[i];
        RadarObjs_msg.Obj[i].Lat_Acc= Radar_msg->Lat_Acc[i];

        RadarObjs_msg.Obj[i].Obj_Type= Radar_msg->Type[i];

    }
    struct timeval get_tv;

    gettimeofday(&get_tv,NULL);
    RadarObjs_msg.stamp=get_tv.tv_sec*1000000 + get_tv.tv_usec;

    ros_pub_RadarObjs_info.publish(RadarObjs_msg);
//    ros_pub_RadarObjs_info.publish(RadarObjs_msg);
}


void UserNode::publishRadarF(void *data)
{
    unsigned char* buffer = (unsigned char*)data;
    uint8_t i;
    std::cout << "Info=" << std::endl;

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
    for(i=0;i<31;i++)
    {

        Radar_msg[0].ID[i]= objf_1[i].obj_id;

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
    for(i=0;i<31;i++)
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
    for(i=0;i<31;i++)
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

    useRadarFCallBack();

//    Ros_Show(&Radar_msg[0]);

}


void UserNode::publishRadarB(void *data)
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
    for(i=0;i<31;i++)
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
    for(i=0;i<31;i++)
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
    for(i=0;i<31;i++)
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

    //        std::cout << "versionInfo=" << Radar_msg->versionInfo << std::endl;
    //        std::cout << "streamDataLen=" << Radar_msg->streamDataLen << std::endl;
    std::cout << "Lat_Acc=" <<Radar_msg[1].Lat_Acc[0] << std::endl;
    //        std::cout << "Long_Pos=" <<RadarObjs_msg.Obj[31].Long_Pos << std::endl;
    //        std::cout << "Lat_Vel=" <<Radar_msg->Lat_Vel[0] << std::endl;
    //        std::cout << "Long_Vel=" <<Radar_msg->Long_Vel[0] << std::endl;

    //        std::cout << "Dyn_Prop=" << Radar_msg->Dyn_Prop[0] << std::endl;
    //        std::cout << "RCS=" << Radar_msg->RCS[0] << std::endl;
    //        std::cout << "Meas_State=" <<Radar_msg->Meas_State[0] << std::endl;

    //no data
    //        std::cout << "Long_Pos_Stdev=" <<Radar_msg->Long_Pos_Stdev[0] << std::endl;
    //        std::cout << "Lat_Pos_Stdev=" <<Radar_msg->Lat_Pos_Stdev[0] << std::endl;

    //        std::cout << "Prob_Exist=" <<Radar_msg->Prob_Exist[0] << std::endl;
    //        std::cout << "Orientation_Angle=" <<Radar_msg->Orientation_Angle[0] << std::endl;

    //          std::cout << "Length=" <<Radar_msg->Length[0] << std::endl;
    //          std::cout << "Width=" <<Radar_msg->Width[0] << std::endl;

    //        struct timeval get_tv;

    //        gettimeofday(&get_tv,NULL);
    //        RadarObjs_msg.stamp=get_tv.tv_sec*1000000 + get_tv.tv_usec;

//    Ros_Show(&Radar_msg[1]);



}


void UserNode::publishRadarFL(void *data)
{
    unsigned char* buffer = (unsigned char*)data;
    uint8_t i;
    std::cout << "FL=" << std::endl;

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
    for(i=0;i<31;i++)
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
    for(i=0;i<31;i++)
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
    for(i=0;i<31;i++)
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

}

void UserNode::publishRadarFR(void *data)
{
    unsigned char* buffer = (unsigned char*)data;
    uint8_t i;
    std::cout << "FR=" << std::endl;

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
    for(i=0;i<31;i++)
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
    for(i=0;i<31;i++)
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
    for(i=0;i<31;i++)
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

}


void UserNode::publishRadarBL(void *data)
{
    unsigned char* buffer = (unsigned char*)data;
    uint8_t i;
    std::cout << "Info=" << std::endl;

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
    for(i=0;i<31;i++)
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
    for(i=0;i<31;i++)
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
    for(i=0;i<31;i++)
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

}

void UserNode::publishRadarBR(void *data)
{
    unsigned char* buffer = (unsigned char*)data;
    uint8_t i;
    std::cout << "Info=" << std::endl;

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
    for(i=0;i<31;i++)
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
    for(i=0;i<31;i++)
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
    for(i=0;i<31;i++)
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
        Ros_Show(&Radar_msg[5]);

}


void UserNode::Recv_InsMsg_callack(const InsMsg::ins_p2::ConstPtr &InsMsg)
{
    Global_InsMsg=*InsMsg;
    Radar_Yaw.radar_device_yaw_rate=(uint16_t)((Global_InsMsg.Heading_Speed)/0.01);
    ars408_67_yaw_rate_information_pack(&RadarF_tx_msg[0],&Radar_Yaw,8);

    useRadarFCallBack();
 //   ars408_67_speed_information_pack();

}

void UserNode::Recv_CdmMsg_callack(const VehicleMsg::cdm_cmd::ConstPtr &CdmMsg)
{
    Global_CdmMsg=*CdmMsg;
    Radar_SpeedInfo.radar_device_speed_direction=1;
    Radar_SpeedInfo.radar_device_speed=(uint16_t)((Global_CdmMsg.Veh_Spd)/0.1);
    ars408_67_speed_information_pack(&RadarF_tx_msg[8],&Radar_SpeedInfo,8);
    
}


void UserNode::useRadarFCallBack(){
    RadarFUdpCallback();
}

uint8_t UserNode::getRadarFUdpMessage(void* data){

//    memcpy(data,RadarF_tx_cmd_msg,RADARF_CMD_LENGHT);

    memcpy(data,RadarF_tx_msg,RADARF_CMD_LENGHT);

    return RADARF_CMD_LENGHT;
}


void UserNode::setRadarFCallBack(udpCallback fun){
        RadarFUdpCallback =fun;
}


