
// this file were genereted by coderdbc.com web service
// any questions - mailto:coderdbc@gmail.com

//#pragma once


//#ifdef __cplusplus
//extern "C" {
//##endif
//
#include <stdint.h>

// This file must define:
// base monitor struct
// function signature for CRC calculation
// function signature for getting system tick value (100 us step)
//#include "canmonitorutil.h"

//#include "Node.h"


// def @Engine_Spedd_0x0CF00400 CAN Message (217056256)
#define Engine_Spedd_0x0CF00400_IDE (1U)
#define Engine_Spedd_0x0CF00400_DLC (8U)
#define Engine_Spedd_0x0CF00400_CANID (0xCF00400U)
// signal: @Engine_Speed
#ifndef Engine_Speed_CovFactor
#define Engine_Speed_CovFactor 0.125
#endif
#define Engine_Speed_CovS(x) ((int16_t)((x / 0.125)))
typedef struct
{

  int16_t Engine_Speed;                     //  [-] Bits=16.  [ 0     , 8000   ]  Unit:''      Factor= 0.125 
} Engine_Spedd_0x0CF00400_t;

// def @ADM_VCU_Control_0x18FD0621 CAN Message (419235361)
#define ADM_VCU_Control_0x18FD0621_IDE (1U)
#define ADM_VCU_Control_0x18FD0621_DLC (8U)
#define ADM_VCU_Control_0x18FD0621_CANID (0x18FD0621U)
typedef struct
{

  int8_t AccPed;                            //  [-] Bits=08.  [ 0     , 100    ]  Unit:''     

  int8_t AccPed_enable;                     //  [-] Bits=02.  [ 0     , 3      ]  Unit:''     

  int8_t ADM_FaultLevel;                    //  [-] Bits=02.  [ 0     , 3      ]  Unit:''     

  int8_t Fog_Light;                         //  [-] Bits=02.  [ 0     , 3      ]  Unit:''     

  int8_t Horn;                              //  [-] Bits=02.  [ 0     , 3      ]  Unit:''     

  int8_t Hydraulic_Brake;                   //  [-] Bits=08.  [ 0     , 100    ]  Unit:''     

  int8_t High_BeamLigh;                     //  [-] Bits=02.  [ 0     , 3      ]  Unit:''     

  int8_t Low_BeamLight;                     //  [-] Bits=02.  [ 0     , 3      ]  Unit:''     

  int8_t Turn_Signal;                       //  [-] Bits=02.  [ 0     , 3      ]  Unit:''     

  int8_t Double_Light;                      //  [-] Bits=02.  [ -2    , 1      ]  Unit:''     

  int8_t Night_Light;                       //  [-] Bits=02.  [ 0     , 3      ]  Unit:''     

  int8_t Defroster_Control;                 //  [-] Bits=02.  [ 0     , 3      ]  Unit:''     

  int8_t Wiper_Control;                     //  [-] Bits=02.  [ 0     , 3      ]  Unit:''     
} ADM_VCU_Control_0x18FD0621_t;

// def @ADM_VCU_Control_0x18FD0721 CAN Message (419235617)
#define ADM_VCU_Control_0x18FD0721_IDE (1U)
#define ADM_VCU_Control_0x18FD0721_DLC (8U)
#define ADM_VCU_Control_0x18FD0721_CANID (0x18FD0721U)
typedef struct
{

  int8_t Amble_Brake;                       //  [-] Bits=08.  [ 0     , 100    ]  Unit:''     

  int8_t Compartment_Control;               //  [-] Bits=02.  [ 0     , 3      ]  Unit:''     

  int8_t Emergency_Brake;                   //  [-] Bits=01.  [ 0     , 1      ]  Unit:''     

  int8_t Load_Brake;                        //  [-] Bits=01.  [ 0     , 1      ]  Unit:''     

  int8_t Engine_Start;                      //  [-] Bits=01.  [ 0     , 1      ]  Unit:''     

  int8_t Engine_Stop;                       //  [-] Bits=01.  [ 0     , 1      ]  Unit:''     

  int8_t Road_DryorWet;                     //  [-] Bits=01.  [ 0     , 1      ]  Unit:''     

  int8_t Switch_DynamicorEconomical;        //  [-] Bits=01.  [ 0     , 1      ]  Unit:''     

  int8_t Slope;                             //  [-] Bits=02.  [ 0     , 3      ]  Unit:''     
} ADM_VCU_Control_0x18FD0721_t;

// def @VCU_Fd_0x18FD0821 CAN Message (419235873)
#define VCU_Fd_0x18FD0821_IDE (1U)
#define VCU_Fd_0x18FD0821_DLC (8U)
#define VCU_Fd_0x18FD0821_CANID (0x18FD0821U)
typedef struct
{

  int8_t Veh_Mass;                          //  [-] Bits=08.  [ 0     , 255    ]  Unit:''     

  int8_t Front_Hydraulic_BrakeFd;           //  [-] Bits=08.  [ 0     , 100    ]  Unit:''     

  int8_t Rear_Hydraulic_BrakeFd;            //  [-] Bits=08.  [ 0     , 100    ]  Unit:''     

  int8_t DriveModeFd;                       //  [-] Bits=02.  [ 0     , 3      ]  Unit:''     

  int8_t Load_BrakeFd;                      //  [-] Bits=02.  [ 0     , 3      ]  Unit:''     

  int8_t Compartment_Up;                    //  [-] Bits=02.  [ 0     , 3      ]  Unit:''     

  int8_t Compartment_Down;                  //  [-] Bits=02.  [ 0     , 3      ]  Unit:''     

  int8_t Compartment_UpHighest;             //  [-] Bits=02.  [ 0     , 3      ]  Unit:''     

  int8_t Compartment_UpLowest;              //  [-] Bits=02.  [ 0     , 3      ]  Unit:''     

  int8_t Vehicle_FaultLevel;                //  [-] Bits=02.  [ 0     , 3      ]  Unit:''     

  int8_t Veh_Spd;                           //  [-] Bits=08.  [ 0     , 100    ]  Unit:''     
} VCU_Fd_0x18FD0821_t;

uint32_t Unpack_Engine_Spedd_0x0CF00400_HaiLuo_extend_CAN_v1(Engine_Spedd_0x0CF00400_t* _m, const uint8_t* _d, uint8_t dlc_);
uint32_t Pack_Engine_Spedd_0x0CF00400_HaiLuo_extend_CAN_v1(const Engine_Spedd_0x0CF00400_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
uint32_t Unpack_ADM_VCU_Control_0x18FD0621_HaiLuo_extend_CAN_v1(ADM_VCU_Control_0x18FD0621_t* _m, const uint8_t* _d, uint8_t dlc_);
uint32_t Pack_ADM_VCU_Control_0x18FD0621_HaiLuo_extend_CAN_v1(const ADM_VCU_Control_0x18FD0621_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
uint32_t Unpack_ADM_VCU_Control_0x18FD0721_HaiLuo_extend_CAN_v1(ADM_VCU_Control_0x18FD0721_t* _m, const uint8_t* _d, uint8_t dlc_);
uint32_t Pack_ADM_VCU_Control_0x18FD0721_HaiLuo_extend_CAN_v1(const ADM_VCU_Control_0x18FD0721_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
uint32_t Unpack_VCU_Fd_0x18FD0821_HaiLuo_extend_CAN_v1(VCU_Fd_0x18FD0821_t* _m, const uint8_t* _d, uint8_t dlc_);
uint32_t Pack_VCU_Fd_0x18FD0821_HaiLuo_extend_CAN_v1(const VCU_Fd_0x18FD0821_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);

//#ifdef __cplusplus
//}
//##endif

