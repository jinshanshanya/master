//
// Created by cj on 1/6/20.
//

#ifndef PROJECT_MONITOR_NODE_H
#define PROJECT_MONITOR_NODE_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

#include <sys/time.h>
#include <time.h>
#include <ctime>

#include <ros/ros.h>
#include <ros/network.h>
#include <boost/thread/thread.hpp>
#include <boost/functional.hpp>

// #include "VehicleMsg/vehicle_status.h"
// #include "VehicleMsg/vehcile_cmd.h"
// #include "VehicleMsg/vehicle_upload.h"
// #include "VehicleMsg/vehicle_download.h"
#include "VehicleMsg/cdm_cmd.h"
#include "VehicleMsg/adm_cmd.h"
#include "VehicleMsg/adm_lat.h"

//#include "InsMsg/ins_p2.h"

//#include "InsMsg/RadarObj.h"
//#include "InsMsg/RadarObjs.h"
// #include "RadarMsg/RadarObj.h"
// #include "RadarMsg/RadarObjs.h"

// #include "CameraMsg/CameraLane.h"
// #include "CameraMsg/CameraObj.h"
// #include "CameraMsg/VisObjs.h"

#include "HaiLuo_extend_CAN_v1.h"
#include "hailuo_extend.h"
#include "hai_luo_extev2.h"
#include "pved_cls.h"
#include "hai_luo_extend_new.h"

#define MSG_TOPIC_CDM "CdmCmdMsg"

#define MSG_TOPIC_ADM "AdmCmdMsg"

#define MSG_TOPIC_ADMLAT "AdmLatMsg"


#define VEHICLE_INFO_LENGHT 100
//#define VEHICLE_CMD_LENGHT          34
#define VEHICLE_CMD_LENGHT 100

typedef std::function<void(void)> udpCallback;

class UserNode
{
public:
    UserNode(int argc, char **pArgv);
    ~UserNode();
    void init();
    bool getReady();

    void copy_buff(uint8_t *dest, uint8_t *src, uint8_t len);

    void publishVehicle(void *data);
    //   void publishIns(void *data);
    uint8_t getVehicleUdpMessage(void *data);
    void setVehicleCallBack(udpCallback fun);

private:
    void run();

    void useVehicleCallBack();

    void recv_msg_AdmCMD_callack(const VehicleMsg::adm_cmd::ConstPtr &AdmMsg);
	  void recv_msg_AdmLAT_callack(const VehicleMsg::adm_lat::ConstPtr &AdmLatMsg);

private:
    int m_Init_argc;
    char **m_pInit_argv;

    bool rosRunning;

    boost::thread *p_vehicle_ros_thread;

    VehicleMsg::cdm_cmd cdm_cmd_msg;
    ros::Publisher pub_cdm_info;

    VehicleMsg::adm_cmd adm_msg;
    ros::Subscriber Sub_Adm_msg;

    VehicleMsg::adm_lat AdmLat_msg;
    ros::Subscriber Sub_AdmLat_msg;

    uint8_t vehicle_tx_msg[VEHICLE_CMD_LENGHT];

    udpCallback vehicleUdpCallback;

    VCU_Fd_0x18FD0821_t Vcv_Fd;
    Engine_Spedd_0x0CF00400_t Eng_Speed;

    //cdm-adm
    // struct hailuo_extend_tcm_gear_fd_t Gear_fd;
    // struct hailuo_extend_vcu_fd_0x18_fd0821_t VCU_Fd;
    // struct hailuo_extend_engine_spedd_0x0_cf00400_t ENG_Speed;

    struct hai_luo_extend_new_engine_spedd_0x0_cf00400_t ENG_Speed;
    struct hai_luo_extend_new_tcm_gear_fd_t Gear_fd;
    struct hai_luo_extend_new_vcu_fd_0x18_fd0821_t VCU_Fd;

    //adm-cdm
    struct hailuo_extend_adm_vcu_control_0x18_fd0621_t Control_621;

    struct hailuo_extend_adm_vcu_control_0x18_fd0721_t Control_721;

    //	struct hai_luo_extend_new_adm_vcu_control_0x18_fd0621_t new_621;
    struct hai_luo_extev2_adm_vcu_control_0x18_fd0621_t new_621;

    struct hai_luo_extend_new_adm_vcu_control_0x18_fd0721_t new_721;

    struct pved_cls_gps1_gmc_t Cur_cmd;
};

#endif //PROJECT_MONITOR_NODE_H
