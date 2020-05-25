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
#include <time.h>
#include <ctime>


#include <ros/ros.h>
#include <ros/network.h>
#include <boost/thread/thread.hpp>
#include <boost/functional.hpp>

#include "InsMsg/ins_p2.h"


#include "IFS2000_standard_CAN.h"


 #define MSG_TOPIC_GLOBALPOSE               "insMsg"

#define IMU_LEN                     100

#define INS_INFO_LENGTH             100


typedef struct {
    uint16_t Week;
    float Time;
    float Heading;
    float Pitch;
    float Roll;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float acc_x;
    float acc_y;
    float acc_z;
	int16_t Roll_speed :16;
	int16_t Pitch_speed :16;
	int16_t Heading_speed :16;
    double Lat;
    double Lon;
    float Altitude;
    float Ve;
    float Vn;
    float Vu;
    float Base;
    uint8_t NSV1;
    uint8_t NSV2;
    uint8_t Status;
    uint8_t Age;
    uint8_t War;
    //uint8_t CHeck;
} imu_msg;

typedef struct {
	uint16_t sync :16;
	uint8_t length :8;
	uint8_t reserved :8;
	uint16_t week :16;
	uint32_t time :32;
	uint8_t N :8;
	double latitude ;
	double longitude ;
	float altitude ;
	float nor_speed ;
	float eas_speed ;
	float down_speed ;
	float roll ;
	float pitch ;
	float heading ;
	int16_t nor_accelerate :16;
	int16_t eas_accelerate :16;
	int16_t down_accelerate :16;
	int16_t roll_speed :16;
	int16_t pitch_speed :16;
	int16_t heading_speed :16;
	uint8_t flag :8;
	uint8_t checksum :8;
} Packet;


class UserNode
{
public:
    UserNode(int argc, char **pArgv);
    ~UserNode();
    void init();
    bool getReady();

	void copy_buff(uint8_t *dest, uint8_t *src, uint8_t len);

 
    void publishIns(void *data);

private:
	void run();


	void Imu_Trans();


private:
    int m_Init_argc;
    char** m_pInit_argv;

    bool rosRunning;

    boost::thread *p_vehicle_ros_thread;


    InsMsg::ins_p2 global_insMsg;

    ros::Publisher  ros_pub_ins_info;
    
    uint8_t imu_buffer[IMU_LEN];
    imu_msg imu_data;
	Packet imuPacke;

   //struct timeval tv;
    DateTime_t Time_data;
    LatitudeLongitude_t Postion;
    Altitude_t Posalt;

    Velocity_t Velocity;

    AccelVehicle_t AccVehicle;

    HeadingPitchRoll_t Angle_Pos;

    AngRateVehicle_t Angle_Rate;

    VelocityLevel_t Vel_Level;

    time_t  time_temp;
    time_t  time_temp_d;
//    float64_t
    float time_s;
	
};


#endif //PROJECT_MONITOR_NODE_H
