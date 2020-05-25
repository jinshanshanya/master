
// this file were genereted by coderdbc.com web service
// any questions - mailto:coderdbc@gmail.com

//#include "ifs2000_standard_can.h"
#include "IFS2000_standard_CAN.h"
//#include "main.h"

// --------------------------------------------------------------------------
uint32_t Unpack_InsStatus_IFS2000_standard_CAN_(InsStatus_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  _m->InsPosMode = (_d[0] & (0xFFU));
  _m->GpsNumSats = (_d[1] & (0xFFU));
  return InsStatus_CANID;
}

uint32_t Pack_InsStatus_IFS2000_standard_CAN_(const InsStatus_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; i < InsStatus_DLC; _d[i++] = 0);

  _d[0] |= (_m->InsPosMode & (0xFFU));
  _d[1] |= (_m->GpsNumSats & (0xFFU));
  *_len = 2; *_ide = 0;
  return InsStatus_CANID;
}

// --------------------------------------------------------------------------
uint32_t Unpack_PosError_IFS2000_standard_CAN_(PosError_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  _m->HoriPosErr = ((_d[1] & (0xFFU)) << 8) | (_d[0] & (0xFFU));
  _m->VertPosErr = ((_d[3] & (0xFFU)) << 8) | (_d[2] & (0xFFU));
  _m->HDOP = (_d[4] & (0xFFU));
  return PosError_CANID;
}

uint32_t Pack_PosError_IFS2000_standard_CAN_(const PosError_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; i < PosError_DLC; _d[i++] = 0);

  _d[0] |= (_m->HoriPosErr & (0xFFU));
  _d[1] |= ((_m->HoriPosErr >> 8) & (0xFFU));
  _d[2] |= (_m->VertPosErr & (0xFFU));
  _d[3] |= ((_m->VertPosErr >> 8) & (0xFFU));
  _d[4] |= (_m->HDOP & (0xFFU));
  *_len = 5; *_ide = 0;
  return PosError_CANID;
}

// --------------------------------------------------------------------------
uint32_t Unpack_GpsLatitudeLongitude_IFS2000_standard_CAN_(GpsLatitudeLongitude_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  _m->GpsPosLat = ((_d[3] & (0xFFU)) << 24) | ((_d[2] & (0xFFU)) << 16) | ((_d[1] & (0xFFU)) << 8) | (_d[0] & (0xFFU));
  _m->GpsPosLon = ((_d[7] & (0xFFU)) << 24) | ((_d[6] & (0xFFU)) << 16) | ((_d[5] & (0xFFU)) << 8) | (_d[4] & (0xFFU));
  return GpsLatitudeLongitude_CANID;
}

uint32_t Pack_GpsLatitudeLongitude_IFS2000_standard_CAN_(const GpsLatitudeLongitude_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; i < GpsLatitudeLongitude_DLC; _d[i++] = 0);

  _d[0] |= (_m->GpsPosLat & (0xFFU));
  _d[1] |= ((_m->GpsPosLat >> 8) & (0xFFU));
  _d[2] |= ((_m->GpsPosLat >> 16) & (0xFFU));
  _d[3] |= ((_m->GpsPosLat >> 24) & (0xFFU));
  _d[4] |= (_m->GpsPosLon & (0xFFU));
  _d[5] |= ((_m->GpsPosLon >> 8) & (0xFFU));
  _d[6] |= ((_m->GpsPosLon >> 16) & (0xFFU));
  _d[7] |= ((_m->GpsPosLon >> 24) & (0xFFU));
  *_len = 8; *_ide = 0;
  return GpsLatitudeLongitude_CANID;
}

// --------------------------------------------------------------------------
uint32_t Unpack_GpsAltitude_IFS2000_standard_CAN_(GpsAltitude_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  _m->GpsPosAlt = ((_d[3] & (0xFFU)) << 24) | ((_d[2] & (0xFFU)) << 16) | ((_d[1] & (0xFFU)) << 8) | (_d[0] & (0xFFU));
  return GpsAltitude_CANID;
}

uint32_t Pack_GpsAltitude_IFS2000_standard_CAN_(const GpsAltitude_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; i < GpsAltitude_DLC; _d[i++] = 0);

  _d[0] |= (_m->GpsPosAlt & (0xFFU));
  _d[1] |= ((_m->GpsPosAlt >> 8) & (0xFFU));
  _d[2] |= ((_m->GpsPosAlt >> 16) & (0xFFU));
  _d[3] |= ((_m->GpsPosAlt >> 24) & (0xFFU));
  *_len = 4; *_ide = 0;
  return GpsAltitude_CANID;
}

// --------------------------------------------------------------------------
uint32_t Unpack_AccelRaw_IFS2000_standard_CAN_(AccelRaw_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  _m->AccelRawX = ((_d[1] & (0xFFU)) << 8) | (_d[0] & (0xFFU));
  _m->AccelRawY = ((_d[3] & (0xFFU)) << 8) | (_d[2] & (0xFFU));
  _m->AccelRawZ = ((_d[5] & (0xFFU)) << 8) | (_d[4] & (0xFFU));
  return AccelRaw_CANID;
}

uint32_t Pack_AccelRaw_IFS2000_standard_CAN_(const AccelRaw_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; i < AccelRaw_DLC; _d[i++] = 0);

  _d[0] |= (_m->AccelRawX & (0xFFU));
  _d[1] |= ((_m->AccelRawX >> 8) & (0xFFU));
  _d[2] |= (_m->AccelRawY & (0xFFU));
  _d[3] |= ((_m->AccelRawY >> 8) & (0xFFU));
  _d[4] |= (_m->AccelRawZ & (0xFFU));
  _d[5] |= ((_m->AccelRawZ >> 8) & (0xFFU));
  *_len = 6; *_ide = 0;
  return AccelRaw_CANID;
}

// --------------------------------------------------------------------------
uint32_t Unpack_AngRateRaw_IFS2000_standard_CAN_(AngRateRaw_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  _m->AngRateRawX = ((_d[1] & (0xFFU)) << 8) | (_d[0] & (0xFFU));
  _m->AngRateRawY = ((_d[3] & (0xFFU)) << 8) | (_d[2] & (0xFFU));
  _m->AngRateRawZ = ((_d[5] & (0xFFU)) << 8) | (_d[4] & (0xFFU));
  return AngRateRaw_CANID;
}

uint32_t Pack_AngRateRaw_IFS2000_standard_CAN_(const AngRateRaw_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; i < AngRateRaw_DLC; _d[i++] = 0);

  _d[0] |= (_m->AngRateRawX & (0xFFU));
  _d[1] |= ((_m->AngRateRawX >> 8) & (0xFFU));
  _d[2] |= (_m->AngRateRawY & (0xFFU));
  _d[3] |= ((_m->AngRateRawY >> 8) & (0xFFU));
  _d[4] |= (_m->AngRateRawZ & (0xFFU));
  _d[5] |= ((_m->AngRateRawZ >> 8) & (0xFFU));
  *_len = 6; *_ide = 0;
  return AngRateRaw_CANID;
}

// --------------------------------------------------------------------------
uint32_t Unpack_GpsAntAngles_IFS2000_standard_CAN_(GpsAntAngles_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  _m->GpsAntHeading = ((_d[1] & (0xFFU)) << 8) | (_d[0] & (0xFFU));
  return GpsAntAngles_CANID;
}

uint32_t Pack_GpsAntAngles_IFS2000_standard_CAN_(const GpsAntAngles_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; i < GpsAntAngles_DLC; _d[i++] = 0);

  _d[0] |= (_m->GpsAntHeading & (0xFFU));
  _d[1] |= ((_m->GpsAntHeading >> 8) & (0xFFU));
  *_len = 2; *_ide = 0;
  return GpsAntAngles_CANID;
}

// --------------------------------------------------------------------------
uint32_t Unpack_DateTime_IFS2000_standard_CAN_(DateTime_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  _m->TimeYear = (_d[0] & (0xFFU));
  _m->TimeCentury = (_d[1] & (0xFFU));
  _m->TimeMonth = (_d[2] & (0xFFU));
  _m->TimeDay = (_d[3] & (0xFFU));
  _m->TimeHSecond = (_d[4] & (0xFFU));
  _m->TimeSecond = (_d[5] & (0xFFU));
  _m->TimeMinute = (_d[6] & (0xFFU));
  _m->TimeHour = (_d[7] & (0xFFU));
  return DateTime_CANID;
}

uint32_t Pack_DateTime_IFS2000_standard_CAN_(const DateTime_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; i < DateTime_DLC; _d[i++] = 0);

  _d[0] |= (_m->TimeYear & (0xFFU));
  _d[1] |= (_m->TimeCentury & (0xFFU));
  _d[2] |= (_m->TimeMonth & (0xFFU));
  _d[3] |= (_m->TimeDay & (0xFFU));
  _d[4] |= (_m->TimeHSecond & (0xFFU));
  _d[5] |= (_m->TimeSecond & (0xFFU));
  _d[6] |= (_m->TimeMinute & (0xFFU));
  _d[7] |= (_m->TimeHour & (0xFFU));
  *_len = 8; *_ide = 0;
  return DateTime_CANID;
}

// --------------------------------------------------------------------------
uint32_t Unpack_LatitudeLongitude_IFS2000_standard_CAN_(LatitudeLongitude_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  _m->PosLat = ((_d[3] & (0xFFU)) << 24) | ((_d[2] & (0xFFU)) << 16) | ((_d[1] & (0xFFU)) << 8) | (_d[0] & (0xFFU));
  _m->PosLon = ((_d[7] & (0xFFU)) << 24) | ((_d[6] & (0xFFU)) << 16) | ((_d[5] & (0xFFU)) << 8) | (_d[4] & (0xFFU));
  return LatitudeLongitude_CANID;
}

uint32_t Pack_LatitudeLongitude_IFS2000_standard_CAN_(const LatitudeLongitude_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; i < LatitudeLongitude_DLC; _d[i++] = 0);

  _d[0] |= (_m->PosLat & (0xFFU));
  _d[1] |= ((_m->PosLat >> 8) & (0xFFU));
  _d[2] |= ((_m->PosLat >> 16) & (0xFFU));
  _d[3] |= ((_m->PosLat >> 24) & (0xFFU));
  _d[4] |= (_m->PosLon & (0xFFU));
  _d[5] |= ((_m->PosLon >> 8) & (0xFFU));
  _d[6] |= ((_m->PosLon >> 16) & (0xFFU));
  _d[7] |= ((_m->PosLon >> 24) & (0xFFU));
  *_len = 8; *_ide = 0;
  return LatitudeLongitude_CANID;
}

// --------------------------------------------------------------------------
uint32_t Unpack_Altitude_IFS2000_standard_CAN_(Altitude_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  _m->PosAlt = ((_d[3] & (0xFFU)) << 24) | ((_d[2] & (0xFFU)) << 16) | ((_d[1] & (0xFFU)) << 8) | (_d[0] & (0xFFU));
  return Altitude_CANID;
}

uint32_t Pack_Altitude_IFS2000_standard_CAN_(const Altitude_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; i < Altitude_DLC; _d[i++] = 0);

  _d[0] |= (_m->PosAlt & (0xFFU));
  _d[1] |= ((_m->PosAlt >> 8) & (0xFFU));
  _d[2] |= ((_m->PosAlt >> 16) & (0xFFU));
  _d[3] |= ((_m->PosAlt >> 24) & (0xFFU));
  *_len = 4; *_ide = 0;
  return Altitude_CANID;
}

// --------------------------------------------------------------------------
uint32_t Unpack_Velocity_IFS2000_standard_CAN_(Velocity_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  _m->VelNorth = ((_d[1] & (0xFFU)) << 8) | (_d[0] & (0xFFU));
  _m->VelEast = ((_d[3] & (0xFFU)) << 8) | (_d[2] & (0xFFU));
  _m->VelDown = ((_d[5] & (0xFFU)) << 8) | (_d[4] & (0xFFU));
  _m->Speed2D = ((_d[7] & (0xFFU)) << 8) | (_d[6] & (0xFFU));
  return Velocity_CANID;
}

uint32_t Pack_Velocity_IFS2000_standard_CAN_(const Velocity_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; i < Velocity_DLC; _d[i++] = 0);

  _d[0] |= (_m->VelNorth & (0xFFU));
  _d[1] |= ((_m->VelNorth >> 8) & (0xFFU));
  _d[2] |= (_m->VelEast & (0xFFU));
  _d[3] |= ((_m->VelEast >> 8) & (0xFFU));
  _d[4] |= (_m->VelDown & (0xFFU));
  _d[5] |= ((_m->VelDown >> 8) & (0xFFU));
  _d[6] |= (_m->Speed2D & (0xFFU));
  _d[7] |= ((_m->Speed2D >> 8) & (0xFFU));
  *_len = 8; *_ide = 0;
  return Velocity_CANID;
}

// --------------------------------------------------------------------------
uint32_t Unpack_VelocityLevel_IFS2000_standard_CAN_(VelocityLevel_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  _m->VelForward = ((_d[1] & (0xFFU)) << 8) | (_d[0] & (0xFFU));
  _m->VelLateral = ((_d[3] & (0xFFU)) << 8) | (_d[2] & (0xFFU));
  return VelocityLevel_CANID;
}

uint32_t Pack_VelocityLevel_IFS2000_standard_CAN_(const VelocityLevel_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; i < VelocityLevel_DLC; _d[i++] = 0);

  _d[0] |= (_m->VelForward & (0xFFU));
  _d[1] |= ((_m->VelForward >> 8) & (0xFFU));
  _d[2] |= (_m->VelLateral & (0xFFU));
  _d[3] |= ((_m->VelLateral >> 8) & (0xFFU));
  *_len = 4; *_ide = 0;
  return VelocityLevel_CANID;
}

// --------------------------------------------------------------------------
uint32_t Unpack_AccelVehicle_IFS2000_standard_CAN_(AccelVehicle_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  _m->AccelX = ((_d[1] & (0xFFU)) << 8) | (_d[0] & (0xFFU));
  _m->AccelY = ((_d[3] & (0xFFU)) << 8) | (_d[2] & (0xFFU));
  _m->AccelZ = ((_d[5] & (0xFFU)) << 8) | (_d[4] & (0xFFU));
  return AccelVehicle_CANID;
}

uint32_t Pack_AccelVehicle_IFS2000_standard_CAN_(const AccelVehicle_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; i < AccelVehicle_DLC; _d[i++] = 0);

  _d[0] |= (_m->AccelX & (0xFFU));
  _d[1] |= ((_m->AccelX >> 8) & (0xFFU));
  _d[2] |= (_m->AccelY & (0xFFU));
  _d[3] |= ((_m->AccelY >> 8) & (0xFFU));
  _d[4] |= (_m->AccelZ & (0xFFU));
  _d[5] |= ((_m->AccelZ >> 8) & (0xFFU));
  *_len = 6; *_ide = 0;
  return AccelVehicle_CANID;
}

// --------------------------------------------------------------------------
uint32_t Unpack_AccelLevel_IFS2000_standard_CAN_(AccelLevel_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  _m->AccelForward = ((_d[1] & (0xFFU)) << 8) | (_d[0] & (0xFFU));
  _m->AccelLateral = ((_d[3] & (0xFFU)) << 8) | (_d[2] & (0xFFU));
  _m->AccelDown = ((_d[5] & (0xFFU)) << 8) | (_d[4] & (0xFFU));
  return AccelLevel_CANID;
}

uint32_t Pack_AccelLevel_IFS2000_standard_CAN_(const AccelLevel_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; i < AccelLevel_DLC; _d[i++] = 0);

  _d[0] |= (_m->AccelForward & (0xFFU));
  _d[1] |= ((_m->AccelForward >> 8) & (0xFFU));
  _d[2] |= (_m->AccelLateral & (0xFFU));
  _d[3] |= ((_m->AccelLateral >> 8) & (0xFFU));
  _d[4] |= (_m->AccelDown & (0xFFU));
  _d[5] |= ((_m->AccelDown >> 8) & (0xFFU));
  *_len = 8; *_ide = 0;
  return AccelLevel_CANID;
}

// --------------------------------------------------------------------------
uint32_t Unpack_HeadingPitchRoll_IFS2000_standard_CAN_(HeadingPitchRoll_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  _m->AngleHeading = ((_d[1] & (0xFFU)) << 8) | (_d[0] & (0xFFU));
  _m->AnglePitch = ((_d[3] & (0xFFU)) << 8) | (_d[2] & (0xFFU));
  _m->AngleRoll = ((_d[5] & (0xFFU)) << 8) | (_d[4] & (0xFFU));
  return HeadingPitchRoll_CANID;
}

uint32_t Pack_HeadingPitchRoll_IFS2000_standard_CAN_(const HeadingPitchRoll_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; i < HeadingPitchRoll_DLC; _d[i++] = 0);

  _d[0] |= (_m->AngleHeading & (0xFFU));
  _d[1] |= ((_m->AngleHeading >> 8) & (0xFFU));
  _d[2] |= (_m->AnglePitch & (0xFFU));
  _d[3] |= ((_m->AnglePitch >> 8) & (0xFFU));
  _d[4] |= (_m->AngleRoll & (0xFFU));
  _d[5] |= ((_m->AngleRoll >> 8) & (0xFFU));
  *_len = 6; *_ide = 0;
  return HeadingPitchRoll_CANID;
}

// --------------------------------------------------------------------------
uint32_t Unpack_AngRateVehicle_IFS2000_standard_CAN_(AngRateVehicle_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  _m->AngRateX = ((_d[1] & (0xFFU)) << 8) | (_d[0] & (0xFFU));
  _m->AngRateY = ((_d[3] & (0xFFU)) << 8) | (_d[2] & (0xFFU));
  _m->AngRateZ = ((_d[5] & (0xFFU)) << 8) | (_d[4] & (0xFFU));
  return AngRateVehicle_CANID;
}

uint32_t Pack_AngRateVehicle_IFS2000_standard_CAN_(const AngRateVehicle_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; i < AngRateVehicle_DLC; _d[i++] = 0);

  _d[0] |= (_m->AngRateX & (0xFFU));
  _d[1] |= ((_m->AngRateX >> 8) & (0xFFU));
  _d[2] |= (_m->AngRateY & (0xFFU));
  _d[3] |= ((_m->AngRateY >> 8) & (0xFFU));
  _d[4] |= (_m->AngRateZ & (0xFFU));
  _d[5] |= ((_m->AngRateZ >> 8) & (0xFFU));
  *_len = 6; *_ide = 0;
  return AngRateVehicle_CANID;
}

// --------------------------------------------------------------------------
uint32_t Unpack_AngRateLevel_IFS2000_standard_CAN_(AngRateLevel_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  _m->AngRateForward = ((_d[1] & (0xFFU)) << 8) | (_d[0] & (0xFFU));
  _m->AngRateLateral = ((_d[3] & (0xFFU)) << 8) | (_d[2] & (0xFFU));
  _m->AngRateDown = ((_d[5] & (0xFFU)) << 8) | (_d[4] & (0xFFU));
  return AngRateLevel_CANID;
}

uint32_t Pack_AngRateLevel_IFS2000_standard_CAN_(const AngRateLevel_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; i < AngRateLevel_DLC; _d[i++] = 0);

  _d[0] |= (_m->AngRateForward & (0xFFU));
  _d[1] |= ((_m->AngRateForward >> 8) & (0xFFU));
  _d[2] |= (_m->AngRateLateral & (0xFFU));
  _d[3] |= ((_m->AngRateLateral >> 8) & (0xFFU));
  _d[4] |= (_m->AngRateDown & (0xFFU));
  _d[5] |= ((_m->AngRateDown >> 8) & (0xFFU));
  *_len = 6; *_ide = 0;
  return AngRateLevel_CANID;
}

// --------------------------------------------------------------------------
uint32_t Unpack_AngAccelVehicle_IFS2000_standard_CAN_(AngAccelVehicle_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  _m->AngAccelX = ((_d[1] & (0xFFU)) << 8) | (_d[0] & (0xFFU));
  _m->AngAccelY = ((_d[3] & (0xFFU)) << 8) | (_d[2] & (0xFFU));
  _m->AngAccelZ = ((_d[5] & (0xFFU)) << 8) | (_d[4] & (0xFFU));
  return AngAccelVehicle_CANID;
}

uint32_t Pack_AngAccelVehicle_IFS2000_standard_CAN_(const AngAccelVehicle_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; i < AngAccelVehicle_DLC; _d[i++] = 0);

  _d[0] |= (_m->AngAccelX & (0xFFU));
  _d[1] |= ((_m->AngAccelX >> 8) & (0xFFU));
  _d[2] |= (_m->AngAccelY & (0xFFU));
  _d[3] |= ((_m->AngAccelY >> 8) & (0xFFU));
  _d[4] |= (_m->AngAccelZ & (0xFFU));
  _d[5] |= ((_m->AngAccelZ >> 8) & (0xFFU));
  *_len = 6; *_ide = 0;
  return AngAccelVehicle_CANID;
}

// --------------------------------------------------------------------------
uint32_t Unpack_AngAccelLevel_IFS2000_standard_CAN_(AngAccelLevel_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  _m->AngAccelForward = ((_d[1] & (0xFFU)) << 8) | (_d[0] & (0xFFU));
  _m->AngAccelLateral = ((_d[3] & (0xFFU)) << 8) | (_d[2] & (0xFFU));
  _m->AngAccelDown = ((_d[5] & (0xFFU)) << 8) | (_d[4] & (0xFFU));
  return AngAccelLevel_CANID;
}

uint32_t Pack_AngAccelLevel_IFS2000_standard_CAN_(const AngAccelLevel_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; i < AngAccelLevel_DLC; _d[i++] = 0);

  _d[0] |= (_m->AngAccelForward & (0xFFU));
  _d[1] |= ((_m->AngAccelForward >> 8) & (0xFFU));
  _d[2] |= (_m->AngAccelLateral & (0xFFU));
  _d[3] |= ((_m->AngAccelLateral >> 8) & (0xFFU));
  _d[4] |= (_m->AngAccelDown & (0xFFU));
  _d[5] |= ((_m->AngAccelDown >> 8) & (0xFFU));
  *_len = 6; *_ide = 0;
  return AngAccelLevel_CANID;
}

// --------------------------------------------------------------------------
uint32_t Unpack_PosLocalNE_IFS2000_standard_CAN_(PosLocalNE_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  _m->PosLocalNorth = ((_d[3] & (0xFFU)) << 24) | ((_d[2] & (0xFFU)) << 16) | ((_d[1] & (0xFFU)) << 8) | (_d[0] & (0xFFU));
  _m->PosLocalEast = ((_d[7] & (0xFFU)) << 24) | ((_d[6] & (0xFFU)) << 16) | ((_d[5] & (0xFFU)) << 8) | (_d[4] & (0xFFU));
  return PosLocalNE_CANID;
}

uint32_t Pack_PosLocalNE_IFS2000_standard_CAN_(const PosLocalNE_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; i < PosLocalNE_DLC; _d[i++] = 0);

  _d[0] |= (_m->PosLocalNorth & (0xFFU));
  _d[1] |= ((_m->PosLocalNorth >> 8) & (0xFFU));
  _d[2] |= ((_m->PosLocalNorth >> 16) & (0xFFU));
  _d[3] |= ((_m->PosLocalNorth >> 24) & (0xFFU));
  _d[4] |= (_m->PosLocalEast & (0xFFU));
  _d[5] |= ((_m->PosLocalEast >> 8) & (0xFFU));
  _d[6] |= ((_m->PosLocalEast >> 16) & (0xFFU));
  _d[7] |= ((_m->PosLocalEast >> 24) & (0xFFU));
  *_len = 8; *_ide = 0;
  return PosLocalNE_CANID;
}

// --------------------------------------------------------------------------
uint32_t Unpack_PosLocalD_IFS2000_standard_CAN_(PosLocalD_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  _m->PosLocalDown = ((_d[3] & (0xFFU)) << 24) | ((_d[2] & (0xFFU)) << 16) | ((_d[1] & (0xFFU)) << 8) | (_d[0] & (0xFFU));
  return PosLocalD_CANID;
}

uint32_t Pack_PosLocalD_IFS2000_standard_CAN_(const PosLocalD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; i < PosLocalD_DLC; _d[i++] = 0);

  _d[0] |= (_m->PosLocalDown & (0xFFU));
  _d[1] |= ((_m->PosLocalDown >> 8) & (0xFFU));
  _d[2] |= ((_m->PosLocalDown >> 16) & (0xFFU));
  _d[3] |= ((_m->PosLocalDown >> 24) & (0xFFU));
  *_len = 4; *_ide = 0;
  return PosLocalD_CANID;
}

// --------------------------------------------------------------------------
uint32_t Unpack_TrackSlip_IFS2000_standard_CAN_(TrackSlip_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  _m->AngleTrack = ((_d[1] & (0xFFU)) << 8) | (_d[0] & (0xFFU));
  _m->AngleSlip = ((_d[3] & (0xFFU)) << 8) | (_d[2] & (0xFFU));
  return TrackSlip_CANID;
}

uint32_t Pack_TrackSlip_IFS2000_standard_CAN_(const TrackSlip_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; i < TrackSlip_DLC; _d[i++] = 0);

  _d[0] |= (_m->AngleTrack & (0xFFU));
  _d[1] |= ((_m->AngleTrack >> 8) & (0xFFU));
  _d[2] |= (_m->AngleSlip & (0xFFU));
  _d[3] |= ((_m->AngleSlip >> 8) & (0xFFU));
  *_len = 4; *_ide = 0;
  return TrackSlip_CANID;
}

