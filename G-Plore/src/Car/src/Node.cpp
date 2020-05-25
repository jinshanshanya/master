

// Created by cj 1/4/20.
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

    ros::init(m_Init_argc, m_pInit_argv, "Vehicle");

    if (!ros::master::check())
        return;

    ros::start();
    ros::Time::init();
    ros::NodeHandle nh;

    name = ros::this_node::getName();

    pub_cdm_info=nh.advertise < VehicleMsg::cdm_cmd> (MSG_TOPIC_CDM, 1);

    Sub_Adm_msg=nh.subscribe <VehicleMsg::adm_cmd> \
                    (MSG_TOPIC_ADM, 1, &UserNode::recv_msg_AdmCMD_callack, this);
    Sub_AdmLat_msg=nh.subscribe <VehicleMsg::adm_lat> \
                    (MSG_TOPIC_ADMLAT, 1, &UserNode::recv_msg_AdmLAT_callack, this);

    p_vehicle_ros_thread = new boost::thread(boost::bind(&UserNode::run, this));


}

void UserNode::run()
{
    ros::Rate loop_rate(20);
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


void UserNode::publishVehicle(void *data)
{
    unsigned char* buffer = (unsigned char*)data;
    uint8_t temp[8]={0};
    uint8_t temp_two[2];



    //0-7 VCU_Fd_0x18FD0821
    memcpy(temp,&buffer[0],8);
//    Unpack_VCU_Fd_0x18FD0821_HaiLuo_extend_CAN_v1(&Vcv_Fd,temp,8);
//    hailuo_extend_vcu_fd_0x18_fd0821_unpack(&VCU_Fd,&buffer[0],8);
    hai_luo_extend_new_vcu_fd_0x18_fd0821_unpack(&VCU_Fd,&buffer[0],8);


    //8-16 0x0_cf00400
    memcpy(temp_two,&buffer[8],2);
//    Unpack_Engine_Spedd_0x0CF00400_HaiLuo_extend_CAN_v1(&Eng_Speed,temp_two,2);
//    hailuo_extend_engine_spedd_0x0_cf00400_unpack(&ENG_Speed,&buffer[8],8);
    hai_luo_extend_new_engine_spedd_0x0_cf00400_unpack(&ENG_Speed,&buffer[8],8);

    //16-23 TCM_GearFd 0x18f00503
 //   hailuo_extend_tcm_gear_fd_unpack(&Gear_fd,&buffer[16],8);
    hai_luo_extend_new_tcm_gear_fd_unpack(&Gear_fd,&buffer[16],8);


    cdm_cmd_msg.Fuel_Signal=VCU_Fd.fuel_signal ;
    cdm_cmd_msg.Veh_Spd=VCU_Fd.veh_spd*0.1 ;
    cdm_cmd_msg.Vehicle_FaultLevel=VCU_Fd.vehicle_fault_level ;

    // cdm_cmd_msg.Compartment_UpLowest=VCU_Fd.compartment_up_lowest ;
    // cdm_cmd_msg.Compartment_UpHighest=VCU_Fd.compartment_up_highest ;

    // cdm_cmd_msg.Compartment_Down=VCU_Fd.compartment_down ;
    // cdm_cmd_msg.Compartment_Up=VCU_Fd.compartment_up ;

    cdm_cmd_msg.Compartment_Fd=VCU_Fd.compartment_fd ;

    cdm_cmd_msg.Load_BrakeFd=VCU_Fd.load_brake_fd ;
    cdm_cmd_msg.DriveModeFd=VCU_Fd.drive_mode_fd ;

    cdm_cmd_msg.Rear_Hydraulic_BrakeFd=VCU_Fd.rear_hydraulic_brake_fd ;

    cdm_cmd_msg.Front_Hydraulic_BrakeFd=VCU_Fd.front_hydraulic_brake_fd ;
    cdm_cmd_msg.Veh_Mass=VCU_Fd.veh_mass ;


    cdm_cmd_msg.Engine_Speed=ENG_Speed.engine_speed*0.125;
    
    cdm_cmd_msg.TCM_GearFd=Gear_fd.tcm_gear_fd*1-125;
    
    if(Gear_fd.tcm_gear_fd ==0)
    {
        cdm_cmd_msg.TCM_GearFd=0;
    }
    
    cdm_cmd_msg.Steer_fd=buffer[24]<<16 | buffer[25]<<8 | buffer[26];
    cdm_cmd_msg.Lat_State=buffer[27];



//    cdm_cmd_msg.Veh_Spd=Vcv_Fd.Veh_Spd ;
//    cdm_cmd_msg.Vehicle_FaultLevel=Vcv_Fd.Vehicle_FaultLevel ;

//    cdm_cmd_msg.Compartment_UpLowest=Vcv_Fd.Compartment_UpLowest ;
//    cdm_cmd_msg.Compartment_UpHighest=Vcv_Fd.Compartment_UpHighest ;

//    cdm_cmd_msg.Compartment_Down=Vcv_Fd.Compartment_Down ;
//    cdm_cmd_msg.Compartment_Up=Vcv_Fd.Compartment_Up ;

//    cdm_cmd_msg.Load_BrakeFd=Vcv_Fd.Load_BrakeFd ;
//    cdm_cmd_msg.DriveModeFd=Vcv_Fd.DriveModeFd ;

//    cdm_cmd_msg.Rear_Hydraulic_BrakeFd=Vcv_Fd.Rear_Hydraulic_BrakeFd ;

//    cdm_cmd_msg.Front_Hydraulic_BrakeFd=Vcv_Fd.Front_Hydraulic_BrakeFd ;
//    cdm_cmd_msg.Veh_Mass=Vcv_Fd.Veh_Mass ;


//    cdm_cmd_msg.Engine_Speed=Eng_Speed.Engine_Speed*0.125;


//    std::cout << "Speed=" << Eng_Speed.Engine_Speed << std::endl;
     std::cout << "gear_fd=" << buffer[16] << std::endl;
    std::cout << "gear_fd1=" << Gear_fd.tcm_gear_fd << std::endl;

    pub_cdm_info.publish(cdm_cmd_msg);
}


void  UserNode::recv_msg_AdmLAT_callack(const VehicleMsg::adm_lat::ConstPtr &AdmLatMsg)
{

//	AdmLat_msg=*AdmLatMsg;
//	Cur_cmd.gps1_curvature_cmd=AdmLat_msg.GPS1_Curvature_cmd;
//	pved_cls_gps1_gmc_pack(&vehicle_tx_msg[16],&Cur_cmd,8);

    uint16_t curvature_cmd=0;
    AdmLat_msg=*AdmLatMsg;
    curvature_cmd=(AdmLat_msg.GPS1_Curvature_cmd+8032)*4;
  
    Cur_cmd.gps1_curvature_cmd=curvature_cmd;
  
    Cur_cmd.gps1_str_cmd_status=AdmLat_msg.Enable_lat;
    pved_cls_gps1_gmc_pack(&vehicle_tx_msg[16],&Cur_cmd,8);
    useVehicleCallBack();
}


void  UserNode::recv_msg_AdmCMD_callack(const VehicleMsg::adm_cmd::ConstPtr &AdmMsg)
{
    adm_msg=*AdmMsg;

    //adm_vcu_control_0x18_fd0621
//    Control_621.acc_ped=adm_msg.AccPed;
//    Control_621.acc_ped_enable=adm_msg.AccPed_enable;

//    Control_621.adm_fault_level=adm_msg.ADM_FaultLevel;
//    Control_621.fog_light=adm_msg.Fog_Light;

//    Control_621.horn=adm_msg.Horn;
//    Control_621.hydraulic_brake=adm_msg.Hydraulic_Brake;

//    Control_621.high_beam_ligh=adm_msg.High_BeamLigh;
//    Control_621.low_beam_light=adm_msg.Low_BeamLight;

//    Control_621.turn_signal=adm_msg.Turn_Signal;
//    Control_621.double_light=adm_msg.Double_Light;

//    Control_621.night_light=adm_msg.Night_Light;
//    Control_621.defroster_control=adm_msg.Defroster_Control;

//    Control_621.wiper_control=adm_msg.Wiper_Control;
//    hailuo_extend_adm_vcu_control_0x18_fd0621_pack(&vehicle_tx_msg[0],&Control_621,8);

    new_621.acc_ped=adm_msg.AccPed;
    new_621.acc_ped_enable=adm_msg.AccPed_enable;

    new_621.adm_fault_level=adm_msg.ADM_FaultLevel;
    new_621.fog_light=adm_msg.Fog_Light;

    new_621.horn=adm_msg.Horn;
    new_621.hydraulic_brake=adm_msg.Hydraulic_Brake;

    new_621.high_beam_ligh=adm_msg.High_BeamLigh;
    new_621.low_beam_light=adm_msg.Low_BeamLight;

    new_621.turn_signal=adm_msg.Turn_Signal;
    new_621.double_light=adm_msg.Double_Light;

    new_621.night_light=adm_msg.Night_Light;
    new_621.defroster_control=adm_msg.Defroster_Control;

    new_621.wiper_control=adm_msg.Wiper_Control;
    new_621.gear=adm_msg.Gear;
    //0-7 0x18_fd0621
    //hai_luo_extend_new_adm_vcu_control_0x18_fd0621_pack(&vehicle_tx_msg[0],&new_621,8);
    hai_luo_extev2_adm_vcu_control_0x18_fd0621_pack(&vehicle_tx_msg[0],&new_621,8);
//    hailuo_extend_adm_vcu_control_0x18_fd0621_pack(&vehicle_tx_msg[0],&Control_621,8);
//    hailuo_extend_adm_vcu_control_0x18_fd0621_pack(&vehicle_tx_msg[0],&new_621,8);




    //adm_vcu_control_0x18_fd0721
//    Control_721.amble_brake=adm_msg.Amble_Brake;

//    Control_721.compartment_control=adm_msg.Compartment_Control;
//    Control_721.emergency_brake=adm_msg.Emergency_Brake;

//    Control_721.load_brake=adm_msg.Load_Brake;
//    Control_721.engine_start=adm_msg.Engine_Start;

//    Control_721.engine_stop=adm_msg.Engine_Stop;
//    Control_721.road_dryor_wet=adm_msg.Road_DryorWet;

//    Control_721.switch_dynamicor_economical=adm_msg.Switch_DynamicorEconomical;
//    Control_721.slope=adm_msg.Slope;

////    //8-15 0x18_fd0721
//    hailuo_extend_adm_vcu_control_0x18_fd0721_pack(&vehicle_tx_msg[8],&Control_721,8);

    new_721.amble_brake=adm_msg.Amble_Brake;

    new_721.compartment_control=adm_msg.Compartment_Control;
    new_721.emergency_brake=adm_msg.Emergency_Brake;

    new_721.load_brake=adm_msg.Load_Brake;
    new_721.engine_start=adm_msg.Engine_Start;

    new_721.engine_stop=adm_msg.Engine_Stop;
    new_721.road_dryor_wet=adm_msg.Road_DryorWet;

    new_721.switch_dynamicor_economical=adm_msg.Switch_DynamicorEconomical;
    new_721.slope=adm_msg.Slope;

    //8-15 0x18_fd0721
    hai_luo_extend_new_adm_vcu_control_0x18_fd0721_pack(&vehicle_tx_msg[8],&new_721,8);


//    Cur_cmd.gps1_curvature_cmd=adm_msg.GPS1_Curvature_cmd;
//    pved_cls_gps1_gmc_pack(&vehicle_tx_msg[16],&Cur_cmd,8);


    useVehicleCallBack();
}



uint8_t UserNode::getVehicleUdpMessage(void* data){

    memcpy(data,vehicle_tx_msg,VEHICLE_CMD_LENGHT);

    return VEHICLE_CMD_LENGHT;
}


void UserNode::setVehicleCallBack(udpCallback fun){
        vehicleUdpCallback =fun;
}
/**
 * set udp callback.
 **/
void UserNode::useVehicleCallBack(){
    vehicleUdpCallback();
}
