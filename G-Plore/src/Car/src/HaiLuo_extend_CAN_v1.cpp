
// this file were genereted by coderdbc.com web service
// any questions - mailto:coderdbc@gmail.com

#include "HaiLuo_extend_CAN_v1.h"

// --------------------------------------------------------------------------
uint32_t Unpack_Engine_Spedd_0x0CF00400_HaiLuo_extend_CAN_v1(Engine_Spedd_0x0CF00400_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  _m->Engine_Speed = ((_d[1] & (0xFFU)) << 8) | (_d[0] & (0xFFU));
  return Engine_Spedd_0x0CF00400_CANID;
}

uint32_t Pack_Engine_Spedd_0x0CF00400_HaiLuo_extend_CAN_v1(const Engine_Spedd_0x0CF00400_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; i < Engine_Spedd_0x0CF00400_DLC; _d[i++] = 0);

  _d[0] |= (_m->Engine_Speed & (0xFFU));
  _d[1] |= ((_m->Engine_Speed >> 8) & (0xFFU));
  *_len = 8; *_ide = 1;
  return Engine_Spedd_0x0CF00400_CANID;
}

// --------------------------------------------------------------------------
uint32_t Unpack_ADM_VCU_Control_0x18FD0621_HaiLuo_extend_CAN_v1(ADM_VCU_Control_0x18FD0621_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  _m->AccPed = (_d[0] & (0xFFU));
  _m->AccPed_enable = (_d[1] & (0x03U));
  _m->ADM_FaultLevel = ((_d[1] >> 2) & (0x03U));
  _m->Fog_Light = ((_d[1] >> 4) & (0x03U));
  _m->Horn = ((_d[1] >> 6) & (0x03U));
  _m->Hydraulic_Brake = (_d[2] & (0xFFU));
  _m->High_BeamLigh = (_d[3] & (0x03U));
  _m->Low_BeamLight = ((_d[3] >> 2) & (0x03U));
  _m->Turn_Signal = ((_d[3] >> 4) & (0x03U));
  _m->Double_Light = ((_d[3] >> 6) & (0x03U));
  _m->Night_Light = (_d[4] & (0x03U));
  _m->Defroster_Control = ((_d[4] >> 2) & (0x03U));
  _m->Wiper_Control = ((_d[4] >> 4) & (0x03U));
  return ADM_VCU_Control_0x18FD0621_CANID;
}

uint32_t Pack_ADM_VCU_Control_0x18FD0621_HaiLuo_extend_CAN_v1(const ADM_VCU_Control_0x18FD0621_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; i < ADM_VCU_Control_0x18FD0621_DLC; _d[i++] = 0);

  _d[0] |= (_m->AccPed & (0xFFU));
  _d[1] |= (_m->AccPed_enable & (0x03U)) | ((_m->ADM_FaultLevel & (0x03U)) << 2) | ((_m->Fog_Light & (0x03U)) << 4) | ((_m->Horn & (0x03U)) << 6);
  _d[2] |= (_m->Hydraulic_Brake & (0xFFU));
  _d[3] |= (_m->High_BeamLigh & (0x03U)) | ((_m->Low_BeamLight & (0x03U)) << 2) | ((_m->Turn_Signal & (0x03U)) << 4) | ((_m->Double_Light & (0x03U)) << 6);
  _d[4] |= (_m->Night_Light & (0x03U)) | ((_m->Defroster_Control & (0x03U)) << 2) | ((_m->Wiper_Control & (0x03U)) << 4);
  *_len = 8; *_ide = 1;
  return ADM_VCU_Control_0x18FD0621_CANID;
}

// --------------------------------------------------------------------------
uint32_t Unpack_ADM_VCU_Control_0x18FD0721_HaiLuo_extend_CAN_v1(ADM_VCU_Control_0x18FD0721_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  _m->Amble_Brake = (_d[0] & (0xFFU));
  _m->Compartment_Control = (_d[1] & (0x03U));
  _m->Emergency_Brake = ((_d[1] >> 2) & (0x01U));
  _m->Load_Brake = ((_d[1] >> 3) & (0x01U));
  _m->Engine_Start = ((_d[1] >> 4) & (0x01U));
  _m->Engine_Stop = ((_d[1] >> 5) & (0x01U));
  _m->Road_DryorWet = ((_d[1] >> 6) & (0x01U));
  _m->Switch_DynamicorEconomical = ((_d[1] >> 7) & (0x01U));
  _m->Slope = (_d[2] & (0x03U));
  return ADM_VCU_Control_0x18FD0721_CANID;
}

uint32_t Pack_ADM_VCU_Control_0x18FD0721_HaiLuo_extend_CAN_v1(const ADM_VCU_Control_0x18FD0721_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; i < ADM_VCU_Control_0x18FD0721_DLC; _d[i++] = 0);

  _d[0] |= (_m->Amble_Brake & (0xFFU));
  _d[1] |= (_m->Compartment_Control & (0x03U)) | ((_m->Emergency_Brake & (0x01U)) << 2) | ((_m->Load_Brake & (0x01U)) << 3) | ((_m->Engine_Start & (0x01U)) << 4) | ((_m->Engine_Stop & (0x01U)) << 5) | ((_m->Road_DryorWet & (0x01U)) << 6) | ((_m->Switch_DynamicorEconomical & (0x01U)) << 7);
  _d[2] |= (_m->Slope & (0x03U));
  *_len = 8; *_ide = 1;
  return ADM_VCU_Control_0x18FD0721_CANID;
}

// --------------------------------------------------------------------------
uint32_t Unpack_VCU_Fd_0x18FD0821_HaiLuo_extend_CAN_v1(VCU_Fd_0x18FD0821_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  _m->Veh_Mass = (_d[0] & (0xFFU));
  _m->Front_Hydraulic_BrakeFd = (_d[1] & (0xFFU));
  _m->Rear_Hydraulic_BrakeFd = (_d[2] & (0xFFU));
  _m->DriveModeFd = (_d[3] & (0x03U));
  _m->Load_BrakeFd = ((_d[3] >> 2) & (0x03U));
  _m->Compartment_Up = ((_d[3] >> 4) & (0x03U));
  _m->Compartment_Down = ((_d[3] >> 6) & (0x03U));
  _m->Compartment_UpHighest = (_d[4] & (0x03U));
  _m->Compartment_UpLowest = ((_d[4] >> 2) & (0x03U));
  _m->Vehicle_FaultLevel = ((_d[4] >> 4) & (0x03U));
  _m->Veh_Spd = (_d[5] & (0xFFU));
  return VCU_Fd_0x18FD0821_CANID;
}

uint32_t Pack_VCU_Fd_0x18FD0821_HaiLuo_extend_CAN_v1(const VCU_Fd_0x18FD0821_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; i < VCU_Fd_0x18FD0821_DLC; _d[i++] = 0);

  _d[0] |= (_m->Veh_Mass & (0xFFU));
  _d[1] |= (_m->Front_Hydraulic_BrakeFd & (0xFFU));
  _d[2] |= (_m->Rear_Hydraulic_BrakeFd & (0xFFU));
  _d[3] |= (_m->DriveModeFd & (0x03U)) | ((_m->Load_BrakeFd & (0x03U)) << 2) | ((_m->Compartment_Up & (0x03U)) << 4) | ((_m->Compartment_Down & (0x03U)) << 6);
  _d[4] |= (_m->Compartment_UpHighest & (0x03U)) | ((_m->Compartment_UpLowest & (0x03U)) << 2) | ((_m->Vehicle_FaultLevel & (0x03U)) << 4);
  _d[5] |= (_m->Veh_Spd & (0xFFU));
  *_len = 8; *_ide = 1;
  return VCU_Fd_0x18FD0821_CANID;
}

