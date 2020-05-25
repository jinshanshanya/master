
// this file were genereted by coderdbc.com web service
// any questions - mailto:coderdbc@gmail.com

#pragma once


//#ifdef __cplusplus
//extern "C" {
//##endif

#include <stdint.h>

//#include "main.h"

// This file must define:
// base monitor struct
// function signature for CRC calculation
// function signature for getting system tick value (100 us step)
//#include "canmonitorutil.h"



// def @InsStatus CAN Message (1280)
#define InsStatus_IDE (0U)
#define InsStatus_DLC (2U)
#define InsStatus_CANID (0x500U)
#define InsStatus_CYC (10U)
// -- INS????????
typedef struct
{

  // ????????????????
  // 9 - "SBAS Differential" 
  // 6 - "Dead reckoning" 
  // 5 - "RTK float" 
  // 4 - "RTK fixed" 
  // 2 - "RTCM Differential" 
  // 1 - "Autonomous position" 
  // 0 - "Invalid" 

  uint8_t InsPosMode;                       //      Bits=08.  [ 0     , 9      ]  Unit:''     

  // ????????
  uint8_t GpsNumSats;                       //      Bits=08.  [ 0     , 255    ]  Unit:''     
} InsStatus_t;

// def @PosError CAN Message (1281)
#define PosError_IDE (0U)
#define PosError_DLC (5U)
#define PosError_CANID (0x501U)
#define PosError_CYC (10U)
// -- ????????
// signal: @HoriPosErr
#ifndef HoriPosErr_CovFactor
#define HoriPosErr_CovFactor 0.001
#endif
#define HoriPosErr_CovS(x) ((uint16_t)((x / 0.001)))
// signal: @VertPosErr
#ifndef VertPosErr_CovFactor
#define VertPosErr_CovFactor 0.001
#endif
#define VertPosErr_CovS(x) ((uint16_t)((x / 0.001)))
// signal: @HDOP
#ifndef HDOP_CovFactor
#define HDOP_CovFactor 0.1
#endif
#define HDOP_CovS(x) ((uint8_t)((x / 0.1)))
typedef struct
{

  // ????????????
  uint16_t HoriPosErr;                      //      Bits=16.  [ 0     , 65.535 ]  Unit:'m'     Factor= 0.001 

  // ????????????
  uint16_t VertPosErr;                      //      Bits=16.  [ 0     , 65.535 ]  Unit:'m'     Factor= 0.001 

  // ????????????????
  uint8_t HDOP;                             //      Bits=08.  [ 0     , 25.5   ]  Unit:''      Factor= 0.1   
} PosError_t;

// def @GpsLatitudeLongitude CAN Message (1282)
#define GpsLatitudeLongitude_IDE (0U)
#define GpsLatitudeLongitude_DLC (8U)
#define GpsLatitudeLongitude_CANID (0x502U)
#define GpsLatitudeLongitude_CYC (10U)
// -- GPS??????????????
// signal: @GpsPosLat
#ifndef GpsPosLat_CovFactor
#define GpsPosLat_CovFactor 1E-07
#endif
#define GpsPosLat_CovS(x) ((int32_t)((x / 1E-07)))
// signal: @GpsPosLon
#ifndef GpsPosLon_CovFactor
#define GpsPosLon_CovFactor 1E-07
#endif
#define GpsPosLon_CovS(x) ((int32_t)((x / 1E-07)))
typedef struct
{

  // ????????
  int32_t GpsPosLat;                        //  [-] Bits=32.  [ -214.7483648, 214.7483647 ]  Unit:'deg'   Factor= 1E-07 

  // ????????
  int32_t GpsPosLon;                        //  [-] Bits=32.  [ -214.7483648, 214.7483647 ]  Unit:'deg'   Factor= 1E-07 
} GpsLatitudeLongitude_t;

// def @GpsAltitude CAN Message (1283)
#define GpsAltitude_IDE (0U)
#define GpsAltitude_DLC (4U)
#define GpsAltitude_CANID (0x503U)
#define GpsAltitude_CYC (10U)
// -- GPS????????????
// signal: @GpsPosAlt
#ifndef GpsPosAlt_CovFactor
#define GpsPosAlt_CovFactor 0.001
#endif
#define GpsPosAlt_CovS(x) ((int32_t)((x / 0.001)))
typedef struct
{

  // ????????????????????
  int32_t GpsPosAlt;                        //  [-] Bits=32.  [ -2147483.648, 2147483.647 ]  Unit:'m'     Factor= 0.001 
} GpsAltitude_t;

// def @AccelRaw CAN Message (1284)
#define AccelRaw_IDE (0U)
#define AccelRaw_DLC (6U)
#define AccelRaw_CANID (0x504U)
#define AccelRaw_CYC (10U)
// -- IMU????????????
// signal: @AccelRawX
#ifndef AccelRawX_CovFactor
#define AccelRawX_CovFactor 0.01
#endif
#define AccelRawX_CovS(x) ((int16_t)((x / 0.01)))
// signal: @AccelRawY
#ifndef AccelRawY_CovFactor
#define AccelRawY_CovFactor 0.01
#endif
#define AccelRawY_CovS(x) ((int16_t)((x / 0.01)))
// signal: @AccelRawZ
#ifndef AccelRawZ_CovFactor
#define AccelRawZ_CovFactor 0.01
#endif
#define AccelRawZ_CovS(x) ((int16_t)((x / 0.01)))
typedef struct
{

  // IMU????X????????
  int16_t AccelRawX;                        //  [-] Bits=16.  [ -327.68, 327.67 ]  Unit:'m/s/s' Factor= 0.01  

  // IMU????Y????????
  int16_t AccelRawY;                        //  [-] Bits=16.  [ -327.68, 327.67 ]  Unit:'m/s/s' Factor= 0.01  

  // IMU????Z????????
  int16_t AccelRawZ;                        //  [-] Bits=16.  [ -327.68, 327.67 ]  Unit:'m/s/s' Factor= 0.01  
} AccelRaw_t;

// def @AngRateRaw CAN Message (1285)
#define AngRateRaw_IDE (0U)
#define AngRateRaw_DLC (6U)
#define AngRateRaw_CANID (0x505U)
#define AngRateRaw_CYC (10U)
// -- IMU????????????
// signal: @AngRateRawX
#ifndef AngRateRawX_CovFactor
#define AngRateRawX_CovFactor 0.01
#endif
#define AngRateRawX_CovS(x) ((int16_t)((x / 0.01)))
// signal: @AngRateRawY
#ifndef AngRateRawY_CovFactor
#define AngRateRawY_CovFactor 0.01
#endif
#define AngRateRawY_CovS(x) ((int16_t)((x / 0.01)))
// signal: @AngRateRawZ
#ifndef AngRateRawZ_CovFactor
#define AngRateRawZ_CovFactor 0.01
#endif
#define AngRateRawZ_CovS(x) ((int16_t)((x / 0.01)))
typedef struct
{

  // IMU????X????????
  int16_t AngRateRawX;                      //  [-] Bits=16.  [ -327.68, 327.67 ]  Unit:'deg/s' Factor= 0.01  

  // IMU????Y????????
  int16_t AngRateRawY;                      //  [-] Bits=16.  [ -327.68, 327.67 ]  Unit:'deg/s' Factor= 0.01  

  // IMU????Z????????
  int16_t AngRateRawZ;                      //  [-] Bits=16.  [ -327.68, 327.67 ]  Unit:'deg/s' Factor= 0.01  
} AngRateRaw_t;

// def @GpsAntAngles CAN Message (1286)
#define GpsAntAngles_IDE (0U)
#define GpsAntAngles_DLC (2U)
#define GpsAntAngles_CANID (0x506U)
#define GpsAntAngles_CYC (10U)
// -- GPS????????
// signal: @GpsAntHeading
#ifndef GpsAntHeading_CovFactor
#define GpsAntHeading_CovFactor 0.01
#endif
#define GpsAntHeading_CovS(x) ((int16_t)((x / 0.01)))
typedef struct
{

  // GPS??????????
  int16_t GpsAntHeading;                    //  [-] Bits=16.  [ -327.68, 327.67 ]  Unit:'deg'   Factor= 0.01  
} GpsAntAngles_t;

// def @DateTime CAN Message (1536)
#define DateTime_IDE (0U)
#define DateTime_DLC (8U)
#define DateTime_CANID (0x600U)
#define DateTime_CYC (10U)
// -- ??????????UTC??????
// signal: @TimeHSecond
#ifndef TimeHSecond_CovFactor
#define TimeHSecond_CovFactor 0.01
#endif
#define TimeHSecond_CovS(x) ((uint8_t)((x / 0.01)))
typedef struct
{

  // ??
  uint8_t TimeYear;                         //      Bits=08.  [ 0     , 100    ]  Unit:''     

  // ????
  uint8_t TimeCentury;                      //      Bits=08.  [ 0     , 0      ]  Unit:''     

  // ??
  uint8_t TimeMonth;                        //      Bits=08.  [ 0     , 12     ]  Unit:''     

  // ??
  uint8_t TimeDay;                          //      Bits=08.  [ 0     , 31     ]  Unit:''     

  // 0.01??
  uint8_t TimeHSecond;                      //      Bits=08.  [ 0     , 1      ]  Unit:'s'     Factor= 0.01  

  // ??
  uint8_t TimeSecond;                       //      Bits=08.  [ 0     , 60     ]  Unit:'s'    

  // ??
  uint8_t TimeMinute;                       //      Bits=08.  [ 0     , 60     ]  Unit:'min'  

  // ????
  uint8_t TimeHour;                         //      Bits=08.  [ 0     , 24     ]  Unit:'hr'   
} DateTime_t;

// def @LatitudeLongitude CAN Message (1537)
#define LatitudeLongitude_IDE (0U)
#define LatitudeLongitude_DLC (8U)
#define LatitudeLongitude_CANID (0x601U)
#define LatitudeLongitude_CYC (10U)
// -- ??????????
// signal: @PosLat
#ifndef PosLat_CovFactor
#define PosLat_CovFactor 1E-07
#endif
#define PosLat_CovS(x) ((int32_t)((x / 1E-07)))
// signal: @PosLon
#ifndef PosLon_CovFactor
#define PosLon_CovFactor 1E-07
#endif
#define PosLon_CovS(x) ((int32_t)((x / 1E-07)))
typedef struct
{

  // ????????
  int32_t PosLat;                           //  [-] Bits=32.  [ -214.7483648, 214.7483647 ]  Unit:'deg'   Factor= 1E-07 

  // ????????
  int32_t PosLon;                           //  [-] Bits=32.  [ -214.7483648, 214.7483647 ]  Unit:'deg'   Factor= 1E-07 
} LatitudeLongitude_t;

// def @Altitude CAN Message (1538)
#define Altitude_IDE (0U)
#define Altitude_DLC (4U)
#define Altitude_CANID (0x602U)
#define Altitude_CYC (10U)
// -- ????????
// signal: @PosAlt
#ifndef PosAlt_CovFactor
#define PosAlt_CovFactor 0.001
#endif
#define PosAlt_CovS(x) ((int32_t)((x / 0.001)))
typedef struct
{

  // ????????????????????
  int32_t PosAlt;                           //  [-] Bits=32.  [ -2147483.648, 2147483.647 ]  Unit:'m'     Factor= 0.001 
} Altitude_t;

// def @Velocity CAN Message (1539)
#define Velocity_IDE (0U)
#define Velocity_DLC (8U)
#define Velocity_CANID (0x603U)
#define Velocity_CYC (10U)
// -- ??????????????
// signal: @VelNorth
#ifndef VelNorth_CovFactor
#define VelNorth_CovFactor 0.01
#endif
#define VelNorth_CovS(x) ((int16_t)((x / 0.01)))
// signal: @VelEast
#ifndef VelEast_CovFactor
#define VelEast_CovFactor 0.01
#endif
#define VelEast_CovS(x) ((int16_t)((x / 0.01)))
// signal: @VelDown
#ifndef VelDown_CovFactor
#define VelDown_CovFactor 0.01
#endif
#define VelDown_CovS(x) ((int16_t)((x / 0.01)))
// signal: @Speed2D
#ifndef Speed2D_CovFactor
#define Speed2D_CovFactor 0.01
#endif
#define Speed2D_CovS(x) ((uint16_t)((x / 0.01)))
typedef struct
{

  // ????????
  int16_t VelNorth;                         //  [-] Bits=16.  [ -327.68, 327.67 ]  Unit:'m/s'   Factor= 0.01  

  // ????????
  int16_t VelEast;                          //  [-] Bits=16.  [ -327.68, 327.67 ]  Unit:'m/s'   Factor= 0.01  

  // ????????
  int16_t VelDown;                          //  [-] Bits=16.  [ -327.68, 327.67 ]  Unit:'m/s'   Factor= 0.01  

  // ??????????
  uint16_t Speed2D;                         //      Bits=16.  [ 0     , 655.35 ]  Unit:'m/s'   Factor= 0.01  
} Velocity_t;

// def @VelocityLevel CAN Message (1540)
#define VelocityLevel_IDE (0U)
#define VelocityLevel_DLC (4U)
#define VelocityLevel_CANID (0x604U)
#define VelocityLevel_CYC (10U)
// -- ??????????????
// signal: @VelForward
#ifndef VelForward_CovFactor
#define VelForward_CovFactor 0.01
#endif
#define VelForward_CovS(x) ((int16_t)((x / 0.01)))
// signal: @VelLateral
#ifndef VelLateral_CovFactor
#define VelLateral_CovFactor 0.01
#endif
#define VelLateral_CovS(x) ((int16_t)((x / 0.01)))
typedef struct
{

  // ????????
  int16_t VelForward;                       //  [-] Bits=16.  [ -327.68, 327.67 ]  Unit:'m/s'   Factor= 0.01  

  // ????????
  int16_t VelLateral;                       //  [-] Bits=16.  [ -327.68, 327.67 ]  Unit:'m/s'   Factor= 0.01  
} VelocityLevel_t;

// def @AccelVehicle CAN Message (1541)
#define AccelVehicle_IDE (0U)
#define AccelVehicle_DLC (6U)
#define AccelVehicle_CANID (0x605U)
#define AccelVehicle_CYC (10U)
// -- ????????????????
// signal: @AccelX
#ifndef AccelX_CovFactor
#define AccelX_CovFactor 0.01
#endif
#define AccelX_CovS(x) ((int16_t)((x / 0.01)))
// signal: @AccelY
#ifndef AccelY_CovFactor
#define AccelY_CovFactor 0.01
#endif
#define AccelY_CovS(x) ((int16_t)((x / 0.01)))
// signal: @AccelZ
#ifndef AccelZ_CovFactor
#define AccelZ_CovFactor 0.01
#endif
#define AccelZ_CovS(x) ((int16_t)((x / 0.01)))
typedef struct
{

  // X????????
  int16_t AccelX;                           //  [-] Bits=16.  [ -327.68, 327.67 ]  Unit:'m/s/s' Factor= 0.01  

  // Y????????
  int16_t AccelY;                           //  [-] Bits=16.  [ -327.68, 327.67 ]  Unit:'m/s/s' Factor= 0.01  

  // Z????????
  int16_t AccelZ;                           //  [-] Bits=16.  [ -327.68, 327.67 ]  Unit:'m/s/s' Factor= 0.01  
} AccelVehicle_t;

// def @AccelLevel CAN Message (1542)
#define AccelLevel_IDE (0U)
#define AccelLevel_DLC (8U)
#define AccelLevel_CANID (0x606U)
#define AccelLevel_CYC (10U)
// -- ????????????????
// signal: @AccelForward
#ifndef AccelForward_CovFactor
#define AccelForward_CovFactor 0.01
#endif
#define AccelForward_CovS(x) ((int16_t)((x / 0.01)))
// signal: @AccelLateral
#ifndef AccelLateral_CovFactor
#define AccelLateral_CovFactor 0.01
#endif
#define AccelLateral_CovS(x) ((int16_t)((x / 0.01)))
// signal: @AccelDown
#ifndef AccelDown_CovFactor
#define AccelDown_CovFactor 0.01
#endif
#define AccelDown_CovS(x) ((int16_t)((x / 0.01)))
typedef struct
{

  // ??????????
  int16_t AccelForward;                     //  [-] Bits=16.  [ -327.68, 327.67 ]  Unit:'m/s/s' Factor= 0.01  

  // ??????????
  int16_t AccelLateral;                     //  [-] Bits=16.  [ -327.68, 327.67 ]  Unit:'m/s/s' Factor= 0.01  

  // ??????????
  int16_t AccelDown;                        //  [-] Bits=16.  [ -327.68, 327.67 ]  Unit:'m/s/s' Factor= 0.01  
} AccelLevel_t;

// def @HeadingPitchRoll CAN Message (1543)
#define HeadingPitchRoll_IDE (0U)
#define HeadingPitchRoll_DLC (6U)
#define HeadingPitchRoll_CANID (0x607U)
#define HeadingPitchRoll_CYC (10U)
// -- ??????
// signal: @AngleHeading
#ifndef AngleHeading_CovFactor
#define AngleHeading_CovFactor 0.01
#endif
#define AngleHeading_CovS(x) ((uint16_t)((x / 0.01)))
// signal: @AnglePitch
#ifndef AnglePitch_CovFactor
#define AnglePitch_CovFactor 0.01
#endif
#define AnglePitch_CovS(x) ((int16_t)((x / 0.01)))
// signal: @AngleRoll
#ifndef AngleRoll_CovFactor
#define AngleRoll_CovFactor 0.01
#endif
#define AngleRoll_CovS(x) ((int16_t)((x / 0.01)))
typedef struct
{

  // ??????
  uint16_t AngleHeading;                    //      Bits=16.  [ 0     , 360    ]  Unit:'deg'   Factor= 0.01  

  // ??????
  int16_t AnglePitch;                       //  [-] Bits=16.  [ -90   , 90     ]  Unit:'deg'   Factor= 0.01  

  // ??????
  int16_t AngleRoll;                        //  [-] Bits=16.  [ -180  , 180    ]  Unit:'deg'   Factor= 0.01  
} HeadingPitchRoll_t;

// def @AngRateVehicle CAN Message (1544)
#define AngRateVehicle_IDE (0U)
#define AngRateVehicle_DLC (6U)
#define AngRateVehicle_CANID (0x608U)
#define AngRateVehicle_CYC (10U)
// -- ????????????????
// signal: @AngRateX
#ifndef AngRateX_CovFactor
#define AngRateX_CovFactor 0.01
#endif
#define AngRateX_CovS(x) ((int16_t)((x / 0.01)))
// signal: @AngRateY
#ifndef AngRateY_CovFactor
#define AngRateY_CovFactor 0.01
#endif
#define AngRateY_CovS(x) ((int16_t)((x / 0.01)))
// signal: @AngRateZ
#ifndef AngRateZ_CovFactor
#define AngRateZ_CovFactor 0.01
#endif
#define AngRateZ_CovS(x) ((int16_t)((x / 0.01)))
typedef struct
{

  // X????????
  int16_t AngRateX;                         //  [-] Bits=16.  [ -327.68, 327.67 ]  Unit:'deg/s' Factor= 0.01  

  // Y????????
  int16_t AngRateY;                         //  [-] Bits=16.  [ -327.68, 327.67 ]  Unit:'deg/s' Factor= 0.01  

  // Z????????
  int16_t AngRateZ;                         //  [-] Bits=16.  [ -327.68, 327.67 ]  Unit:'deg/s' Factor= 0.01  
} AngRateVehicle_t;

// def @AngRateLevel CAN Message (1545)
#define AngRateLevel_IDE (0U)
#define AngRateLevel_DLC (6U)
#define AngRateLevel_CANID (0x609U)
#define AngRateLevel_CYC (10U)
// -- ????????????????
// signal: @AngRateForward
#ifndef AngRateForward_CovFactor
#define AngRateForward_CovFactor 0.01
#endif
#define AngRateForward_CovS(x) ((int16_t)((x / 0.01)))
// signal: @AngRateLateral
#ifndef AngRateLateral_CovFactor
#define AngRateLateral_CovFactor 0.01
#endif
#define AngRateLateral_CovS(x) ((int16_t)((x / 0.01)))
// signal: @AngRateDown
#ifndef AngRateDown_CovFactor
#define AngRateDown_CovFactor 0.01
#endif
#define AngRateDown_CovS(x) ((int16_t)((x / 0.01)))
typedef struct
{

  // ??????????
  int16_t AngRateForward;                   //  [-] Bits=16.  [ -327.68, 327.67 ]  Unit:'deg/s' Factor= 0.01  

  // ??????????
  int16_t AngRateLateral;                   //  [-] Bits=16.  [ -327.68, 327.67 ]  Unit:'deg/s' Factor= 0.01  

  // ??????????
  int16_t AngRateDown;                      //  [-] Bits=16.  [ -327.68, 327.67 ]  Unit:'deg/s' Factor= 0.01  
} AngRateLevel_t;

// def @AngAccelVehicle CAN Message (1546)
#define AngAccelVehicle_IDE (0U)
#define AngAccelVehicle_DLC (6U)
#define AngAccelVehicle_CANID (0x60AU)
#define AngAccelVehicle_CYC (10U)
// -- ??????????????????
// signal: @AngAccelX
#ifndef AngAccelX_CovFactor
#define AngAccelX_CovFactor 0.1
#endif
#define AngAccelX_CovS(x) ((int16_t)((x / 0.1)))
// signal: @AngAccelY
#ifndef AngAccelY_CovFactor
#define AngAccelY_CovFactor 0.1
#endif
#define AngAccelY_CovS(x) ((int16_t)((x / 0.1)))
// signal: @AngAccelZ
#ifndef AngAccelZ_CovFactor
#define AngAccelZ_CovFactor 0.1
#endif
#define AngAccelZ_CovS(x) ((int16_t)((x / 0.1)))
typedef struct
{

  // X??????????
  int16_t AngAccelX;                        //  [-] Bits=16.  [ -3276.8, 3276.7 ]  Unit:'deg/s/s' Factor= 0.1   

  // Y??????????
  int16_t AngAccelY;                        //  [-] Bits=16.  [ -3276.8, 3276.7 ]  Unit:'deg/s/s' Factor= 0.1   

  // Z??????????
  int16_t AngAccelZ;                        //  [-] Bits=16.  [ -3276.8, 3276.7 ]  Unit:'deg/s/s' Factor= 0.1   
} AngAccelVehicle_t;

// def @AngAccelLevel CAN Message (1547)
#define AngAccelLevel_IDE (0U)
#define AngAccelLevel_DLC (6U)
#define AngAccelLevel_CANID (0x60BU)
#define AngAccelLevel_CYC (10U)
// signal: @AngAccelForward
#ifndef AngAccelForward_CovFactor
#define AngAccelForward_CovFactor 0.1
#endif
#define AngAccelForward_CovS(x) ((int16_t)((x / 0.1)))
// signal: @AngAccelLateral
#ifndef AngAccelLateral_CovFactor
#define AngAccelLateral_CovFactor 0.1
#endif
#define AngAccelLateral_CovS(x) ((int16_t)((x / 0.1)))
// signal: @AngAccelDown
#ifndef AngAccelDown_CovFactor
#define AngAccelDown_CovFactor 0.1
#endif
#define AngAccelDown_CovS(x) ((int16_t)((x / 0.1)))
typedef struct
{

  int16_t AngAccelForward;                  //  [-] Bits=16.  [ -3276.8, 3276.7 ]  Unit:'deg/s/s' Factor= 0.1   

  int16_t AngAccelLateral;                  //  [-] Bits=16.  [ -3276.8, 3276.7 ]  Unit:'deg/s/s' Factor= 0.1   

  int16_t AngAccelDown;                     //  [-] Bits=16.  [ -3276.8, 3276.7 ]  Unit:'deg/s/s' Factor= 0.1   
} AngAccelLevel_t;

// def @PosLocalNE CAN Message (1548)
#define PosLocalNE_IDE (0U)
#define PosLocalNE_DLC (8U)
#define PosLocalNE_CANID (0x60CU)
#define PosLocalNE_CYC (10U)
// -- ??????????????????
// signal: @PosLocalNorth
#ifndef PosLocalNorth_CovFactor
#define PosLocalNorth_CovFactor 0.001
#endif
#define PosLocalNorth_CovS(x) ((int32_t)((x / 0.001)))
// signal: @PosLocalEast
#ifndef PosLocalEast_CovFactor
#define PosLocalEast_CovFactor 0.001
#endif
#define PosLocalEast_CovS(x) ((int32_t)((x / 0.001)))
typedef struct
{

  // ??????????????
  int32_t PosLocalNorth;                    //  [-] Bits=32.  [ -214748.3648, 214748.3647 ]  Unit:'m'     Factor= 0.001 

  // ??????????????
  int32_t PosLocalEast;                     //  [-] Bits=32.  [ -214748.3648, 214748.3647 ]  Unit:'m'     Factor= 0.001 
} PosLocalNE_t;

// def @PosLocalD CAN Message (1549)
#define PosLocalD_IDE (0U)
#define PosLocalD_DLC (4U)
#define PosLocalD_CANID (0x60DU)
#define PosLocalD_CYC (10U)
// -- ????????????????
// signal: @PosLocalDown
#ifndef PosLocalDown_CovFactor
#define PosLocalDown_CovFactor 0.001
#endif
#define PosLocalDown_CovS(x) ((int32_t)((x / 0.001)))
typedef struct
{

  // ??????????????
  int32_t PosLocalDown;                     //  [-] Bits=32.  [ -214748.3648, 214748.3647 ]  Unit:'m'     Factor= 0.001 
} PosLocalD_t;

// def @TrackSlip CAN Message (1550)
#define TrackSlip_IDE (0U)
#define TrackSlip_DLC (4U)
#define TrackSlip_CANID (0x60EU)
#define TrackSlip_CYC (10U)
// -- ??????????????
// signal: @AngleTrack
#ifndef AngleTrack_CovFactor
#define AngleTrack_CovFactor 0.01
#endif
#define AngleTrack_CovS(x) ((uint16_t)((x / 0.01)))
// signal: @AngleSlip
#ifndef AngleSlip_CovFactor
#define AngleSlip_CovFactor 0.01
#endif
#define AngleSlip_CovS(x) ((int16_t)((x / 0.01)))
typedef struct
{

  // ??????
  uint16_t AngleTrack;                      //      Bits=16.  [ 0     , 360    ]  Unit:'deg'   Factor= 0.01  

  // ??????
  int16_t AngleSlip;                        //  [-] Bits=16.  [ -327.68, 327.67 ]  Unit:'deg'   Factor= 0.01  
} TrackSlip_t;

uint32_t Unpack_InsStatus_IFS2000_standard_CAN_(InsStatus_t* _m, const uint8_t* _d, uint8_t dlc_);
uint32_t Pack_InsStatus_IFS2000_standard_CAN_(const InsStatus_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
uint32_t Unpack_PosError_IFS2000_standard_CAN_(PosError_t* _m, const uint8_t* _d, uint8_t dlc_);
uint32_t Pack_PosError_IFS2000_standard_CAN_(const PosError_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
uint32_t Unpack_GpsLatitudeLongitude_IFS2000_standard_CAN_(GpsLatitudeLongitude_t* _m, const uint8_t* _d, uint8_t dlc_);
uint32_t Pack_GpsLatitudeLongitude_IFS2000_standard_CAN_(const GpsLatitudeLongitude_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
uint32_t Unpack_GpsAltitude_IFS2000_standard_CAN_(GpsAltitude_t* _m, const uint8_t* _d, uint8_t dlc_);
uint32_t Pack_GpsAltitude_IFS2000_standard_CAN_(const GpsAltitude_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
uint32_t Unpack_AccelRaw_IFS2000_standard_CAN_(AccelRaw_t* _m, const uint8_t* _d, uint8_t dlc_);
uint32_t Pack_AccelRaw_IFS2000_standard_CAN_(const AccelRaw_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
uint32_t Unpack_AngRateRaw_IFS2000_standard_CAN_(AngRateRaw_t* _m, const uint8_t* _d, uint8_t dlc_);
uint32_t Pack_AngRateRaw_IFS2000_standard_CAN_(const AngRateRaw_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
uint32_t Unpack_GpsAntAngles_IFS2000_standard_CAN_(GpsAntAngles_t* _m, const uint8_t* _d, uint8_t dlc_);
uint32_t Pack_GpsAntAngles_IFS2000_standard_CAN_(const GpsAntAngles_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
uint32_t Unpack_DateTime_IFS2000_standard_CAN_(DateTime_t* _m, const uint8_t* _d, uint8_t dlc_);
uint32_t Pack_DateTime_IFS2000_standard_CAN_(const DateTime_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
uint32_t Unpack_LatitudeLongitude_IFS2000_standard_CAN_(LatitudeLongitude_t* _m, const uint8_t* _d, uint8_t dlc_);
uint32_t Pack_LatitudeLongitude_IFS2000_standard_CAN_(const LatitudeLongitude_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
uint32_t Unpack_Altitude_IFS2000_standard_CAN_(Altitude_t* _m, const uint8_t* _d, uint8_t dlc_);
uint32_t Pack_Altitude_IFS2000_standard_CAN_(const Altitude_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
uint32_t Unpack_Velocity_IFS2000_standard_CAN_(Velocity_t* _m, const uint8_t* _d, uint8_t dlc_);
uint32_t Pack_Velocity_IFS2000_standard_CAN_(const Velocity_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
uint32_t Unpack_VelocityLevel_IFS2000_standard_CAN_(VelocityLevel_t* _m, const uint8_t* _d, uint8_t dlc_);
uint32_t Pack_VelocityLevel_IFS2000_standard_CAN_(const VelocityLevel_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
uint32_t Unpack_AccelVehicle_IFS2000_standard_CAN_(AccelVehicle_t* _m, const uint8_t* _d, uint8_t dlc_);
uint32_t Pack_AccelVehicle_IFS2000_standard_CAN_(const AccelVehicle_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
uint32_t Unpack_AccelLevel_IFS2000_standard_CAN_(AccelLevel_t* _m, const uint8_t* _d, uint8_t dlc_);
uint32_t Pack_AccelLevel_IFS2000_standard_CAN_(const AccelLevel_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
uint32_t Unpack_HeadingPitchRoll_IFS2000_standard_CAN_(HeadingPitchRoll_t* _m, const uint8_t* _d, uint8_t dlc_);
uint32_t Pack_HeadingPitchRoll_IFS2000_standard_CAN_(const HeadingPitchRoll_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
uint32_t Unpack_AngRateVehicle_IFS2000_standard_CAN_(AngRateVehicle_t* _m, const uint8_t* _d, uint8_t dlc_);
uint32_t Pack_AngRateVehicle_IFS2000_standard_CAN_(const AngRateVehicle_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
uint32_t Unpack_AngRateLevel_IFS2000_standard_CAN_(AngRateLevel_t* _m, const uint8_t* _d, uint8_t dlc_);
uint32_t Pack_AngRateLevel_IFS2000_standard_CAN_(const AngRateLevel_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
uint32_t Unpack_AngAccelVehicle_IFS2000_standard_CAN_(AngAccelVehicle_t* _m, const uint8_t* _d, uint8_t dlc_);
uint32_t Pack_AngAccelVehicle_IFS2000_standard_CAN_(const AngAccelVehicle_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
uint32_t Unpack_AngAccelLevel_IFS2000_standard_CAN_(AngAccelLevel_t* _m, const uint8_t* _d, uint8_t dlc_);
uint32_t Pack_AngAccelLevel_IFS2000_standard_CAN_(const AngAccelLevel_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
uint32_t Unpack_PosLocalNE_IFS2000_standard_CAN_(PosLocalNE_t* _m, const uint8_t* _d, uint8_t dlc_);
uint32_t Pack_PosLocalNE_IFS2000_standard_CAN_(const PosLocalNE_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
uint32_t Unpack_PosLocalD_IFS2000_standard_CAN_(PosLocalD_t* _m, const uint8_t* _d, uint8_t dlc_);
uint32_t Pack_PosLocalD_IFS2000_standard_CAN_(const PosLocalD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
uint32_t Unpack_TrackSlip_IFS2000_standard_CAN_(TrackSlip_t* _m, const uint8_t* _d, uint8_t dlc_);
uint32_t Pack_TrackSlip_IFS2000_standard_CAN_(const TrackSlip_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);

//#ifdef __cplusplus
//}
//##endif

