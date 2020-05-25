/*
 * Filename: /home/easydrive/project/glb/admsystem/src/IntegratedDriver/Camera/include/Camera/RaCamData.h
 * Path: /home/easydrive/project/glb/admsystem/src/IntegratedDriver/Camera/include/Camera
 * Created Date: Monday, October 14th 2019, 6:29:40 pm
 * Author: easydrive
 * 
 * Copyright (c) 2019 Your Company
 *modified on Oct 16, 2019 by Yuncheng jiang*/

#ifndef _CAMERA_H
#define _CAMERA_H
#include <stdint.h>

#define VIS_OBS_NUM 						6


#define VIS_OBS_BUS_SIZE					788
#define VIS_ROAD_BUS_SIZE					408
#define VIS_BUS_SIZE						1212

#define PACKET_TYPE_CAMERA1					10
#define PACKET_TYPE_CAMERA2					11

typedef enum  _Vison_LaneType_  /* align the lane type with the signal matrix provided by Motiv */
{
        Lane_Type_Single_Dashed 	= 0,
        Lane_Type_Solid 			= 1,
        Lane_Type_Double_Solid		= 2,
        Lane_Type_Dashed_Solid		= 3,
        Lane_Type_Solid_Dashed		= 4,
        Lane_Type_Curb		        = 5,
        Lane_Type_Unknown           = 6,
} Lane_Type;

typedef enum  _Vison_LaneConfidence_  /* align the lane confidence with the signal matrix provided by Motiv */
{
        Lane_Confidence_Low	        = 0,
        Lane_Confidence_Middle 	    = 1,
        Lane_Confidence_High 		= 2,
        Lane_Confidence_Perfect 	= 3,

} Lane_Confidence;



typedef enum _Vision_Obstacle_Type_  /* align the obstacle info with the signal matrix provided by Motiv */
{
        Obs_Type_Undetermined 			= 0,
        Obs_Type_Pedestrian 			= 1,
        Obs_Type_Car 					= 2,
        Obs_Type_Bus					= 3,
        Obs_Type_Truck					= 4,
        Obs_Type_Bicycle  			    = 5,
        Obs_Type_Tricycle 			    = 6,

        // Obs_Type_Animal 				= 7,
        // Obs_Type_General_on_Road_Obj 	= 8,
        // Obs_Type_Undentified_Vehicle 	= 10
} Obs_Type;

typedef enum _Vision_Obstacle_Cut_In_Out_
{
        Cut_In_Out_Undetermined = 0,
        In_Host_Lane 			= 1,
        Out_Host_Lane 			= 2,
        Cut_In 					= 3,
        Cut_Out 				= 4
} Cut_In_Out_Ind;

//Vision Obstacle Packets
typedef struct _Vision_Obstacle_
{
        uint8_t			obsNum;
        uint8_t 		ID[VIS_OBS_NUM];
        Obs_Type 		Obs_Classif[VIS_OBS_NUM];
        float			Long_Pos[VIS_OBS_NUM];
        float			Lat_Pos[VIS_OBS_NUM];
        float			Height[VIS_OBS_NUM];
        float			Width[VIS_OBS_NUM];
        float			Long_Vel[VIS_OBS_NUM];
        float			Lat_Vel[VIS_OBS_NUM];
        float			Long_Acc[VIS_OBS_NUM];
        float			Lat_Acc[VIS_OBS_NUM];
        float           TTC[VIS_OBS_NUM];

        Cut_In_Out_Ind 	Cut_In_Out[VIS_OBS_NUM];
} Vis_ObsInfo;

//Vision Road Packets
typedef struct _Lane_Parameter_
{
        float 		A0;
        float 		A1;
        float 		A2;
        float 		A3;
        float 		Range; //Lane_Line_Range
        Lane_Type	Type;
        Lane_Confidence Confidence; /* add the lane cofidence info in lane parameter struct*/
} lanePara;

typedef struct _Vison_Lane_Information_
{
        lanePara 	Left_Individual;
        lanePara 	Right_Individual;
        lanePara 	Left_Neighbor;
        lanePara 	Right_Neighbor;
} Vis_Lane;

typedef struct _Vision_Road_
{
        uint8_t			TSNum;
        Vis_Lane 		laneInfo;
} Vis_RoadInfo;

typedef struct _Vision_Obj_Bus_
{
        uint16_t 			versionInfo;
        uint16_t			streamDataLen;
        uint32_t			streamTxCnt;
        uint32_t			sourceTimeStamp;
        uint32_t			streamTimeStamp;
        Vis_ObsInfo			visObsInfo;
        Vis_RoadInfo		visRoadInfo;
} Vis_Obj;


#endif
