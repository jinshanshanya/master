/*
 * Filename: /home/easydrive/project/glb/admsystem/src/IntegratedDriver/Radar/include/Radar/radar.h
 * Path: /home/easydrive/project/glb/admsystem/src/IntegratedDriver/Radar/include/Radar
 * Created Date: Monday, October 14th 2019, 7:28:44 pm
 * Author: easydrive
 * 
 * Copyright (c) 2019 Your Company
 */

#ifndef _RADAR_H
#define _RADAR_H
#include <stdint.h>

#define RAD_OBJ_NUM 						32
#define RAD_ANG_ONJ_NUM 					64

#define RAD_BUS_SIZE						2068
// #define RAD_ANGLE_BUS_SIZE					10086
#define RAD_ANGLE_BUS_SIZE					10598


#define PACKET_TYPE_RADAR1					0
#define PACKET_TYPE_RADAR2					1



//Type Enum
typedef enum _Radar_Obj_Type_
{
	Radar_Obj_Type_Point 		= 0,
	Radar_Obj_Type_Car 			= 1,
	Radar_Obj_Type_Truck 		= 2,
	Radar_Obj_Type_Motorcycle 	= 4,
	Radar_Obj_Type_Bicycle 		= 5,
	Radar_Obj_Type_Wide 		= 6
} Radar_Obj_Type;

typedef enum _Radar_Obj_Meas_State_
{
	Radar_Meas_State_Deleted			= 0,
	Radar_Meas_State_NewCreated			= 1,
	Radar_Meas_State_Measured			= 2,
	Radar_Meas_State_Predicted 			= 4,
	Radar_Meas_State_DeletedForMerged	= 5,
	Radar_Meas_State_NewFromMerged		= 6
} Radar_Meas_State;

typedef enum _Radar_Obj_Exist_Prob_
{
	Radar_Obj_Exist_Prob_Invalid	  = 0,
	Radar_Obj_Exist_Prob_Less_25pct   = 1,
	Radar_Obj_Exist_Prob_Less_50pct   = 2,
	Radar_Obj_Exist_Prob_Less_75pct   = 3,
	Radar_Obj_Exist_Prob_Less_90pct   = 4,
	Radar_Obj_Exist_Prob_Less_99pct   = 5,
	Radar_Obj_Exist_Prob_Less_999pct  = 6,
	Radar_Obj_Exist_Prob_equal_100pct = 7
} Radar_Obj_Exist_Prob;

typedef enum _Radar_Obj_DynProp_
{
	Radar_DynProp_Moving 					= 0,
	Radar_DynProp_Stationary 				= 1,
	Radar_DynProp_Oncoming 					= 2,
	Radar_DynProp_Stationary_Candidate 		= 3,
	Radar_DynProp_DynProp_Unknown 			= 4,
	Radar_DynProp_Crossing_Stationary 		= 5,
	Radar_DynProp_Crossing_Moving 			= 6,
	Radar_DynProp_Stopped 					= 7
} Radar_DynProp;



//Data Packet Format
//Radar Packets
typedef struct _Radar_Obj_Bus_
{
	uint16_t 				versionInfo;
	uint16_t				streamDataLen;
	uint32_t				streamTxCnt;
	uint32_t				sourceTimeStamp;
	uint32_t				streamTimeStamp;
	uint8_t					objNum;
	uint8_t					frameInd;
	uint8_t 				ID[RAD_OBJ_NUM];
	float					Long_Pos[RAD_OBJ_NUM];
	float					Lat_Pos[RAD_OBJ_NUM];
	float					Long_Vel[RAD_OBJ_NUM];
	float					Lat_Vel[RAD_OBJ_NUM];
	float					Long_Acc[RAD_OBJ_NUM];
	float					Lat_Acc[RAD_OBJ_NUM];
	uint8_t					Long_Pos_Stdev[RAD_OBJ_NUM];
	uint8_t					Lat_Pos_Stdev[RAD_OBJ_NUM];
	uint8_t					Long_Vel_Stdev[RAD_OBJ_NUM];
	uint8_t					Lat_Vel_Stdev[RAD_OBJ_NUM];
	uint8_t					Long_Acc_Stdev[RAD_OBJ_NUM];
	uint8_t					Lat_Acc_Stdev[RAD_OBJ_NUM];
	float					Orientation_Angle[RAD_OBJ_NUM];
	uint8_t					Orientation_Stdev[RAD_OBJ_NUM];
	float					Length[RAD_OBJ_NUM];
	float					Width[RAD_OBJ_NUM];
	float					RCS[RAD_OBJ_NUM];
	Radar_Obj_Type			Type[RAD_OBJ_NUM];
	Radar_Obj_Exist_Prob	Prob_Exist[RAD_OBJ_NUM];
	Radar_Meas_State		Meas_State[RAD_OBJ_NUM];
	Radar_DynProp 			Dyn_Prop[RAD_OBJ_NUM];
} Rad_Obj;

#endif
