//
// Created by wss on 7/1/17.
//

#ifndef PROJECT_MONITOR_NODE_H
#define PROJECT_MONITOR_NODE_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

#include <sys/time.h>

#include <ros/ros.h>
#include <ros/network.h>
#include <boost/thread/thread.hpp>
#include <boost/functional.hpp>

#include "InsMsg/ins_p2.h"
#include "VehicleMsg/cdm_cmd.h"

#include "RadarMsg/RadarObj.h"
#include "RadarMsg/RadarObjs.h"

#include "ars408_67.h"
#include "ars408_60.h"
#include "ars408_66.h"
#include "ars408_65.h"
#include "ars408_64.h"
#include "ars408_63.h"
#include "ars408_62.h"

#define RADARF_INFO_LENGHT 1300

#define RADARF_CMD_LENGHT 16

#define MSG_TOPIC_GLOBALPOSE "insMsg"

#define MSG_TOPIC_RADAROBJ "RadarObjMsg"

#define MSG_TOPIC_CDM "CdmCmdMsg"

#define IMU_LEN 100

#define INS_INFO_LENGTH 69
//#define VEHICLE_INFO_LENGHT         21
#define VEHICLE_INFO_LENGHT 1300
#define VEHICLE_CMD_LENGHT 34

#define RAD_OBJ_NUM 32
#define RAD_ANG_ONJ_NUM 64

#define RAD_BUS_SIZE 2068
// #define RAD_ANGLE_BUS_SIZE					10086
#define RAD_ANGLE_BUS_SIZE 10598

#define PACKET_TYPE_RADAR1 0
#define PACKET_TYPE_RADAR2 1

typedef enum _Radar_Obj_Type_
{
    Radar_Obj_Type_Point = 0,
    Radar_Obj_Type_Car = 1,
    Radar_Obj_Type_Truck = 2,
    Radar_Obj_Type_Motorcycle = 4,
    Radar_Obj_Type_Bicycle = 5,
    Radar_Obj_Type_Wide = 6
} Radar_Obj_Type;

typedef enum _Radar_Obj_Meas_State_
{
    Radar_Meas_State_Deleted = 0,
    Radar_Meas_State_NewCreated = 1,
    Radar_Meas_State_Measured = 2,
    Radar_Meas_State_Predicted = 4,
    Radar_Meas_State_DeletedForMerged = 5,
    Radar_Meas_State_NewFromMerged = 6
} Radar_Meas_State;

typedef enum _Radar_Obj_Exist_Prob_
{
    Radar_Obj_Exist_Prob_Invalid = 0,
    Radar_Obj_Exist_Prob_Less_25pct = 1,
    Radar_Obj_Exist_Prob_Less_50pct = 2,
    Radar_Obj_Exist_Prob_Less_75pct = 3,
    Radar_Obj_Exist_Prob_Less_90pct = 4,
    Radar_Obj_Exist_Prob_Less_99pct = 5,
    Radar_Obj_Exist_Prob_Less_999pct = 6,
    Radar_Obj_Exist_Prob_equal_100pct = 7
} Radar_Obj_Exist_Prob;

typedef enum _Radar_Obj_DynProp_
{
    Radar_DynProp_Moving = 0,
    Radar_DynProp_Stationary = 1,
    Radar_DynProp_Oncoming = 2,
    Radar_DynProp_Stationary_Candidate = 3,
    Radar_DynProp_DynProp_Unknown = 4,
    Radar_DynProp_Crossing_Stationary = 5,
    Radar_DynProp_Crossing_Moving = 6,
    Radar_DynProp_Stopped = 7
} Radar_DynProp;

//Data Packet Format
//Radar Packets
typedef struct _Radar_Obj_Bus_
{
    uint16_t versionInfo;
    uint16_t streamDataLen;
    uint32_t streamTxCnt;
    uint32_t sourceTimeStamp;
    uint32_t streamTimeStamp;
    uint8_t objNum;
    uint8_t frameInd;
    uint8_t ID[RAD_OBJ_NUM];
    float Long_Pos[RAD_OBJ_NUM];
    float Lat_Pos[RAD_OBJ_NUM];
    float Long_Vel[RAD_OBJ_NUM];
    float Lat_Vel[RAD_OBJ_NUM];
    float Long_Acc[RAD_OBJ_NUM];
    float Lat_Acc[RAD_OBJ_NUM];
    float Long_Pos_Stdev[RAD_OBJ_NUM];
    float Lat_Pos_Stdev[RAD_OBJ_NUM];
    float Long_Vel_Stdev[RAD_OBJ_NUM];
    float Lat_Vel_Stdev[RAD_OBJ_NUM];
    float Long_Acc_Stdev[RAD_OBJ_NUM];
    float Lat_Acc_Stdev[RAD_OBJ_NUM];
    float Orientation_Angle[RAD_OBJ_NUM];
    float Orientation_Stdev[RAD_OBJ_NUM];
    float Length[RAD_OBJ_NUM];
    float Width[RAD_OBJ_NUM];
    float RCS[RAD_OBJ_NUM];
    Radar_Obj_Type Type[RAD_OBJ_NUM];
    Radar_Obj_Exist_Prob Prob_Exist[RAD_OBJ_NUM];
    Radar_Meas_State Meas_State[RAD_OBJ_NUM];
    Radar_DynProp Dyn_Prop[RAD_OBJ_NUM];
} Rad_Obj;

typedef std::function<void(void)> udpCallback;

class UserNode
{
public:
    UserNode(int argc, char **pArgv);
    ~UserNode();
    void init();
    bool getReady();

    void copy_buff(uint8_t *dest, uint8_t *src, uint8_t len);

    void publishRadarF(void *data);
    void publishRadarB(void *data);

    void publishRadarFL(void *data);
    void publishRadarFR(void *data);

    void publishRadarBL(void *data);
    void publishRadarBR(void *data);

    void Ros_Show(Rad_Obj *Radar_msg);

    uint8_t getRadarFUdpMessage(void *data);
    void setRadarFCallBack(udpCallback fun);

private:
    void run();

    void useRadarFCallBack();

    void Imu_Trans();
    float Obj_Rms(uint8_t num);
    float ObjOri_Rms(uint8_t num);

    void Recv_InsMsg_callack(const InsMsg::ins_p2::ConstPtr &InsMsg);

    void Recv_CdmMsg_callack(const VehicleMsg::cdm_cmd::ConstPtr &CdmMsg);


private:
    int m_Init_argc;
    char **m_pInit_argv;

    bool rosRunning;

    boost::thread *Radar_thread;

    RadarMsg::RadarObjs RadarObjs_msg;


    ros::Publisher ros_pub_RadarObjs_info;

    ros::Subscriber Sub_InsMsg;
    InsMsg::ins_p2 Global_InsMsg;

    ros::Subscriber Sub_CdmMsg;
    VehicleMsg::cdm_cmd Global_CdmMsg;

    uint8_t RadarF_tx_msg[RADARF_CMD_LENGHT];
    udpCallback RadarFUdpCallback;

  
    uint8_t imu_buffer[IMU_LEN];
 
    Rad_Obj Radar_msg[6];

    struct ars408_67_obj_0_status_t objb_0;
    struct ars408_67_obj_1_general_t objb_1[50];
    struct ars408_67_obj_2_quality_t objb_2[50];
    struct ars408_67_obj_3_extended_t objb_3[50];

    struct ars408_60_obj_0_status_t objf_0;
    struct ars408_60_obj_1_general_t objf_1[50];
    struct ars408_60_obj_2_quality_t objf_2[50];
    struct ars408_60_obj_3_extended_t objf_3[50];

    struct ars408_66_obj_0_status_t objfl_0;
    struct ars408_66_obj_1_general_t objfl_1[50];
    struct ars408_66_obj_2_quality_t objfl_2[50];
    struct ars408_66_obj_3_extended_t objfl_3[50];

    struct ars408_65_obj_0_status_t objfr_0;
    struct ars408_65_obj_1_general_t objfr_1[50];
    struct ars408_65_obj_2_quality_t objfr_2[50];
    struct ars408_65_obj_3_extended_t objfr_3[50];

    struct ars408_64_obj_0_status_t objbl_0;
    struct ars408_64_obj_1_general_t objbl_1[50];
    struct ars408_64_obj_2_quality_t objbl_2[50];
    struct ars408_64_obj_3_extended_t objbl_3[50];

    struct ars408_63_obj_0_status_t objbr_0;
    struct ars408_63_obj_1_general_t objbr_1[50];
    struct ars408_63_obj_2_quality_t objbr_2[50];
    struct ars408_63_obj_3_extended_t objbr_3[50];

    struct ars408_67_speed_information_t Radar_SpeedInfo;
    struct ars408_67_yaw_rate_information_t Radar_Yaw;

    //  uint8_t RadarF_Txmsg[16]={0};
};

#endif //PROJECT_MONITOR_NODE_H
