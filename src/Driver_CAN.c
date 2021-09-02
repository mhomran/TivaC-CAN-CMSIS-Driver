/*
 * Copyright (c) 2015-2020 Arm Limited. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
 
#include "Driver_CAN.h"
#include "can_memmap.h"

#define ARM_CAN_DRV_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(1,0) // CAN driver version
#define SYSCLK (16000000UL)
#define MAX_STD_ID 0x7FF
#define MAX_XTD_ID 0x1FFFFFFF
#define MAX_OBJ_IDX 31
#define MAX_OBJ_NUM 32
#define VALID_FILTER 1
#define INVALID_FILTER 0
#define MAX_CAN_CONTROLLERS 2

//typedefs
typedef enum {
  CAN_FLAG_SET,
  CAN_FLAG_CLEAR,
  CAN_FLAG_MAX
} CanFlag_t;

typedef enum {
  MSG_OBJ_W,
  MSG_OBJ_R,
  MSG_OBJ_MAX
} MsgObjWR_t;

typedef struct {
  uint32_t Id;
  uint32_t Mask;
  uint32_t Valid;
} CanFilter_t;

//
//   Functions prototypes
//
int32_t 
Can_SetTimeout(volatile uint32_t* Reg, 
              uint32_t FlagPos, 
              CanFlag_t SetClr, 
              uint32_t Timeout);
static inline uint32_t Min(uint32_t A, uint32_t B);
static inline uint32_t Max(uint32_t A, uint32_t B);
static int32_t Can_SetFilterExactId(volatile CanHandle_t* CAN, uint32_t ObjIdx, uint32_t Id);
static int32_t Can_RemoveFilterExactId(volatile CanHandle_t* CAN, uint32_t ObjIdx, uint32_t Id);
static int32_t Can_SetFilterMask(volatile CanHandle_t* CAN, uint32_t ObjIdx, uint32_t Id, uint32_t Mask);
static int32_t Can_RemoveFilterMask(volatile CanHandle_t* CAN, uint32_t ObjIdx, uint32_t Id, uint32_t Mask);
static int32_t Can_WriteReadMsgObj(volatile CanHandle_t* CAN, MsgObjWR_t Flag, uint32_t ObjIdx, uint32_t Mask);
static int32_t Can_SetXtdId(volatile CanHandle_t* CAN, uint32_t XtdId);
static int32_t Can_SetStdId(volatile CanHandle_t* CAN, uint32_t StdId);
static int32_t Can_SetXtdMask(volatile CanHandle_t* CAN, uint32_t XtdMask);
static int32_t Can_SetStdMask(volatile CanHandle_t* CAN, uint32_t StdMask);
static int32_t
ARM_CAN_ObjectSetFilter(volatile CanHandle_t* CAN,
                        uint32_t ObjIdx,
                        ARM_CAN_FILTER_OPERATION operation,
                        uint32_t id,
                        uint32_t arg);
static int32_t
ARM_CAN0_ObjectSetFilter(uint32_t ObjIdx,
                         ARM_CAN_FILTER_OPERATION operation,
                         uint32_t id,
                         uint32_t arg);
static int32_t ARM_CAN_SetMode (volatile CanHandle_t* CAN, ARM_CAN_MODE mode);
static int32_t ARM_CAN0_SetMode (ARM_CAN_MODE mode);
static int32_t 
ARM_CAN0_ObjectConfigure (uint32_t ObjIdx, 
                         ARM_CAN_OBJ_CONFIG ObjCfg);
static int32_t 
ARM_CAN_ObjectConfigure (volatile CanHandle_t* CAN,
                         uint32_t ObjIdx, 
                         ARM_CAN_OBJ_CONFIG ObjCfg); 

static int32_t 
ARM_CAN0_MessageSend(uint32_t ObjIdx, 
                     ARM_CAN_MSG_INFO *MsgInfo, 
                     const uint8_t *Data, 
                     uint8_t Size);
static int32_t 
ARM_CAN_MessageSend(volatile CanHandle_t* CAN,
                    uint32_t ObjIdx, 
                    ARM_CAN_MSG_INFO *MsgInfo, 
                    const uint8_t *Data, 
                    uint8_t Size); 

static void Can_SetData(volatile CanHandle_t* CAN, const uint8_t *Data, uint8_t Size);
static void Can_ReadData(volatile CanHandle_t* CAN, uint8_t *Data, uint8_t Size);

void CAN_Handler(volatile CanHandle_t* CAN);
void CAN0_Handler(void);
void CAN1_Handler(void);
static int32_t ARM_CAN0_PowerControl (ARM_POWER_STATE state);

static int32_t ARM_CAN_PowerControl (volatile CanHandle_t* CAN, ARM_POWER_STATE state);

static int32_t
ARM_CAN0_Initialize(ARM_CAN_SignalUnitEvent_t cb_unit_event,
                    ARM_CAN_SignalObjectEvent_t cb_object_event);
static int32_t
ARM_CAN1_Initialize(ARM_CAN_SignalUnitEvent_t cb_unit_event,
                    ARM_CAN_SignalObjectEvent_t cb_object_event);
static int32_t
ARM_CAN_Initialize(uint8_t CANIdx,
                   ARM_CAN_SignalUnitEvent_t cb_unit_event,
                   ARM_CAN_SignalObjectEvent_t cb_object_event);
static int32_t ARM_CAN0_Uninitialize (void);
static int32_t ARM_CAN1_Uninitialize (void);
static int32_t ARM_CAN_Uninitialize (volatile CanHandle_t* CAN);

uint8_t Can_GetControllerIdx(volatile CanHandle_t* CAN);
static int32_t
ARM_CAN0_MessageRead(uint32_t ObjIdx,
                     ARM_CAN_MSG_INFO *MsgInfo,
                     uint8_t *Data,
                     uint8_t Size);
static int32_t
ARM_CAN1_MessageRead(uint32_t ObjIdx,
                     ARM_CAN_MSG_INFO *MsgInfo,
                     uint8_t *Data,
                     uint8_t Size);

static int32_t
ARM_CAN_MessageRead(volatile CanHandle_t* CAN,
                    uint32_t ObjIdx,
                    ARM_CAN_MSG_INFO *MsgInfo,
                    uint8_t *Data,
                    uint8_t Size);
static int32_t ARM_CAN0_Control(uint32_t control, uint32_t arg);
static int32_t ARM_CAN1_Control(uint32_t control, uint32_t arg);
static int32_t ARM_CAN_Control (volatile CanHandle_t* CAN, uint32_t control, uint32_t arg);

static ARM_CAN_STATUS ARM_CAN0_GetStatus (void);
static ARM_CAN_STATUS ARM_CAN1_GetStatus (void);
static ARM_CAN_STATUS ARM_CAN_GetStatus (volatile CanHandle_t* CAN);

static int32_t 
ARM_CAN0_SetBitrate (ARM_CAN_BITRATE_SELECT select,
                    uint32_t bitrate,
                    uint32_t bit_segments);
static int32_t 
ARM_CAN1_SetBitrate (ARM_CAN_BITRATE_SELECT select,
                    uint32_t bitrate,
                    uint32_t bit_segments);
static int32_t 
ARM_CAN_SetBitrate (volatile CanHandle_t* CAN,
                    ARM_CAN_BITRATE_SELECT select,
                    uint32_t bitrate,
                    uint32_t bit_segments);
// module variable definitons

//driver version
static const ARM_DRIVER_VERSION can_driver_version = { ARM_CAN_API_VERSION, ARM_CAN_DRV_VERSION };

// Driver Capabilities
static const ARM_CAN_CAPABILITIES can_driver_capabilities = {
  32U,  // Number of CAN Objects available
  0U,   // Does not support reentrant calls to ARM_CAN_MessageSend, ARM_CAN_MessageRead, ARM_CAN_ObjectConfigure and abort message sending used by ARM_CAN_Control.
  0U,   // Does not support CAN with Flexible Data-rate mode (CAN_FD)
  1U,   // Does not support restricted operation mode
  1U,   // Does not support bus monitoring mode
  1U,   // Does not support internal loopback mode
  0U,   // Does not support external loopback mode
  0U    // Reserved (must be zero)
};

// Object Capabilities
static const ARM_CAN_OBJ_CAPABILITIES can_object_capabilities = {
  1U,   // Object supports transmission
  1U,   // Object supports reception
  0U,   // Object does not support RTR reception and automatic Data transmission
  0U,   // Object does not support RTR transmission and automatic Data reception
  1U,   // Object does not allow assignment of multiple filters to it
  0U,   // Object does not support exact identifier filtering
  1U,   // Object does not support range identifier filtering
  0U,   // Object does not support mask identifier filtering
  0U,   // Object can not buffer messages
  0U    // Reserved (must be zero)
};

static uint8_t                     can_driver_powered[MAX_CAN_CONTROLLERS];
static uint8_t                     can_driver_initialized[MAX_CAN_CONTROLLERS];
static ARM_CAN_SignalUnitEvent_t   CAN_SignalUnitEvent[MAX_CAN_CONTROLLERS];
static ARM_CAN_SignalObjectEvent_t CAN_SignalObjectEvent[MAX_CAN_CONTROLLERS];
static CanFilter_t gFilters[MAX_CAN_CONTROLLERS][MAX_OBJ_NUM];
static ARM_CAN_OBJ_CONFIG gObjCfgs[MAX_CAN_CONTROLLERS][MAX_OBJ_NUM];

//
//   Functions definitions
//

static ARM_DRIVER_VERSION ARM_CAN_GetVersion (void) {
  // Return driver version
  return can_driver_version;
}

static ARM_CAN_CAPABILITIES ARM_CAN_GetCapabilities (void) {
  // Return driver capabilities
  return can_driver_capabilities;
}


static int32_t 
ARM_CAN0_Initialize(ARM_CAN_SignalUnitEvent_t cb_unit_event,
                    ARM_CAN_SignalObjectEvent_t cb_object_event)
{
  return ARM_CAN_Initialize(0, cb_unit_event, cb_object_event);
}

static int32_t 
ARM_CAN1_Initialize(ARM_CAN_SignalUnitEvent_t cb_unit_event,
                    ARM_CAN_SignalObjectEvent_t cb_object_event)
{
  return ARM_CAN_Initialize(1, cb_unit_event, cb_object_event);
} 

static int32_t 
ARM_CAN_Initialize(uint8_t CANIdx,
                   ARM_CAN_SignalUnitEvent_t cb_unit_event,
                   ARM_CAN_SignalObjectEvent_t cb_object_event) 
{
  if (can_driver_initialized[CANIdx] != 0U) return ARM_DRIVER_OK;

  CAN_SignalUnitEvent[CANIdx]   = cb_unit_event;
  CAN_SignalObjectEvent[CANIdx] = cb_object_event;

  can_driver_initialized[CANIdx] = 1U;

  return ARM_DRIVER_OK;
}

static int32_t 
ARM_CAN0_Uninitialize (void) 
{
  return ARM_CAN_Uninitialize(CAN0);
}
static int32_t 
ARM_CAN1_Uninitialize (void) 
{
  return ARM_CAN_Uninitialize(CAN1);
}
static int32_t 
ARM_CAN_Uninitialize (volatile CanHandle_t* CAN) 
{
  uint8_t CANIdx = Can_GetControllerIdx(CAN);
  can_driver_initialized[CANIdx] = 0U;

  return ARM_DRIVER_OK;
}

static int32_t 
ARM_CAN0_PowerControl (ARM_POWER_STATE state) 
{
  return ARM_CAN_PowerControl(CAN0, state);
}
static int32_t 
ARM_CAN1_PowerControl (ARM_POWER_STATE state) 
{
  return ARM_CAN_PowerControl(CAN1, state);
}

static int32_t 
ARM_CAN_PowerControl (volatile CanHandle_t* CAN,
                      ARM_POWER_STATE state) 
{
  uint8_t CANIdx = Can_GetControllerIdx(CAN);

  switch (state) {
    case ARM_POWER_OFF:
      can_driver_powered[CANIdx] = 0U;
      // Add code to disable interrupts and put peripheral into reset mode,
      // and if possible disable clock
      // ..
      SRCAN |= 1 << CANIdx; //reset
      Can_SetTimeout(&PRCAN, CANIdx, CAN_FLAG_SET, 0xFFFF);
      SRCAN &= ~(1 << CANIdx);

      RCGCCAN &= ~(1 << CANIdx); //disable clock
      Can_SetTimeout(&PRCAN, CANIdx, CAN_FLAG_CLEAR, 0xFFFF);
      break;

    case ARM_POWER_FULL:
      if (can_driver_initialized[CANIdx] == 0U) { return ARM_DRIVER_ERROR; }
      if (can_driver_powered[CANIdx]     != 0U) { return ARM_DRIVER_OK;    }
      
      uint8_t i;
      // Add code to enable clocks, reset variables enable interrupts
      // and put peripheral into operational
      // ..
      RCGCCAN |= (1 << CANIdx); //enable clock
      Can_SetTimeout(&PRCAN, CANIdx, CAN_FLAG_SET, 0xFFFF);

      CAN->CTL.CTL |= CANCTL_IE_Msk | CANCTL_EIE_Msk;

      //enable interrupt in NVIC
      NVICEN1 |= 1 << (NVICEN1_CAN0 + CANIdx);

      can_driver_powered[CANIdx] = 1U;
      can_driver_initialized[CANIdx] = 0U;
      for(i = 0; i < MAX_OBJ_NUM; i++)
        {
          gFilters[CANIdx][i].Valid = INVALID_FILTER;
        }
      break;

    case ARM_POWER_LOW:
      return ARM_DRIVER_ERROR_UNSUPPORTED;
  }

  return ARM_DRIVER_OK;
}

static uint32_t ARM_CAN_GetClock (void) {

  // Add code to return peripheral clock frequency
  // ..
    return 0;
}

static int32_t 
ARM_CAN0_SetBitrate (ARM_CAN_BITRATE_SELECT select,
                    uint32_t bitrate,
                    uint32_t bit_segments) 
{
  return ARM_CAN_SetBitrate(CAN0, select, bitrate, bit_segments);
}
static int32_t 
ARM_CAN1_SetBitrate (ARM_CAN_BITRATE_SELECT select,
                    uint32_t bitrate,
                    uint32_t bit_segments) 
{
  return ARM_CAN_SetBitrate(CAN1, select, bitrate, bit_segments);
}

static int32_t 
ARM_CAN_SetBitrate (volatile CanHandle_t* CAN,
                    ARM_CAN_BITRATE_SELECT select,
                    uint32_t bitrate,
                    uint32_t bit_segments) 
{
  uint32_t BRP;
  uint32_t Phase1;
  uint32_t Phase2;
  uint32_t Prop;
  uint32_t SJW;
  uint32_t N;
  uint8_t CANIdx = Can_GetControllerIdx(CAN);

  if (can_driver_powered[CANIdx] == 0U) { return ARM_DRIVER_ERROR; }

  if(bitrate == 0 || bitrate > (2000000UL))
    {
      return ARM_DRIVER_ERROR_PARAMETER;
    }

  Prop = (bit_segments & ARM_CAN_BIT_PROP_SEG_Msk) >> ARM_CAN_BIT_PROP_SEG_Pos;
  Phase1 = (bit_segments & ARM_CAN_BIT_PHASE_SEG1_Msk) >> ARM_CAN_BIT_PHASE_SEG1_Pos;
  Phase2 = (bit_segments & ARM_CAN_BIT_PHASE_SEG2_Msk) >> ARM_CAN_BIT_PHASE_SEG2_Pos;
  SJW = (bit_segments & ARM_CAN_BIT_SJW_Msk) >> ARM_CAN_BIT_SJW_Pos;
  
  if((8 < Prop || Prop < 1)               ||
     (8 < Phase1 || Phase1 < 1)           ||
     (8 < Phase2 || Phase2 < 1)           ||
     (Phase2 > Max(Phase1, 2))            ||
     (4 < SJW || SJW < 1)                 ||
     (SJW > Min(Min(Phase1, Phase2), 4)))
    {
      return ARM_DRIVER_ERROR_PARAMETER;
    }

  N = 1 + Prop + Phase1 + Phase2;
  BRP = (SYSCLK / bitrate / N) - 1;
  if(BRP > 1024)
    {
      return ARM_CAN_INVALID_BITRATE;
    }
  else if (BRP > 64)
    {
      CAN0->CTL.BRPE = (BRP - 1) >> 6;
      CAN0->CTL.BIT &= ~(CANBIT_BRP_Msk);
      CAN0->CTL.BIT |= (((BRP - 1) & 0x3F) << CANBIT_BRP_Pos);
    }
  else 
    {
      CAN0->CTL.BIT &= ~(CANBIT_BRP_Msk);
      CAN0->CTL.BIT |= ((BRP - 1) << CANBIT_BRP_Pos);
    }
  
  CAN0->CTL.BIT &= ~(CANBIT_TSEG1_Msk);
  CAN0->CTL.BIT |= ((Phase1 + Prop - 1) << CANBIT_TSEG1_Pos);
  CAN0->CTL.BIT &= ~(CANBIT_TSEG2_Msk);
  CAN0->CTL.BIT |= ((Phase2 - 1) << CANBIT_TSEG2_Pos);
  CAN0->CTL.BIT &= ~(CANBIT_SJW_Msk);
  CAN0->CTL.BIT |= ((SJW - 1) << CANBIT_SJW_Pos);

  return ARM_DRIVER_OK;
}

static int32_t 
ARM_CAN0_SetMode(ARM_CAN_MODE mode) 
{
  return ARM_CAN_SetMode(CAN0, mode);
}
static int32_t 
ARM_CAN1_SetMode(ARM_CAN_MODE mode) 
{
  return ARM_CAN_SetMode(CAN1, mode);
}

static int32_t ARM_CAN_SetMode (volatile CanHandle_t* CAN, ARM_CAN_MODE mode) {
  uint8_t CANIdx = Can_GetControllerIdx(CAN);

  if (can_driver_powered[CANIdx] == 0U) { return ARM_DRIVER_ERROR; }

  switch (mode) {
    case ARM_CAN_MODE_INITIALIZATION:
      // Add code to put peripheral into initialization mode
      // ..
      CAN->CTL.CTL |= CANCTL_INIT_Msk | CANCTL_CCE_Msk;
      break;
    case ARM_CAN_MODE_NORMAL:
      // Add code to put peripheral into normal operation mode
      // ..
      CAN->CTL.CTL &= ~CANCTL_INIT_Msk;
      break;
    case ARM_CAN_MODE_RESTRICTED:
      // Add code to put peripheral into restricted operation mode
      // ..
      return ARM_DRIVER_ERROR_UNSUPPORTED;
    case ARM_CAN_MODE_MONITOR:
      // Add code to put peripheral into bus monitoring mode
      // ..
      return ARM_DRIVER_ERROR_UNSUPPORTED;
    case ARM_CAN_MODE_LOOPBACK_INTERNAL:
      // Add code to put peripheral into internal loopback mode
      // ..
      return ARM_DRIVER_ERROR_UNSUPPORTED;
    case ARM_CAN_MODE_LOOPBACK_EXTERNAL:
      // Add code to put peripheral into external loopback mode
      // ..
      CAN->CTL.CTL |= CANCTL_TEST_Msk;
      CAN->CTL.TST |= CANTST_LBACK_Msk;
      CAN->CTL.CTL &= ~CANCTL_INIT_Msk;
      break;
    default:
      return ARM_DRIVER_ERROR_PARAMETER;
  }

  return ARM_DRIVER_OK;
}

static ARM_CAN_OBJ_CAPABILITIES ARM_CAN_ObjectGetCapabilities (uint32_t ObjIdx) {
  // Return object capabilities
  return can_object_capabilities;
}

static int32_t 
ARM_CAN0_ObjectSetFilter(uint32_t ObjIdx,
                         ARM_CAN_FILTER_OPERATION operation, 
                         uint32_t id, 
                         uint32_t arg) 
{
  return ARM_CAN_ObjectSetFilter(CAN0, ObjIdx, operation, id, arg);
}
static int32_t 
ARM_CAN1_ObjectSetFilter(uint32_t ObjIdx,
                         ARM_CAN_FILTER_OPERATION operation, 
                         uint32_t id, 
                         uint32_t arg) 
{
  return ARM_CAN_ObjectSetFilter(CAN1, ObjIdx, operation, id, arg);
}

static int32_t 
ARM_CAN_ObjectSetFilter(volatile CanHandle_t* CAN,
                        uint32_t ObjIdx,
                        ARM_CAN_FILTER_OPERATION operation, 
                        uint32_t id, 
                        uint32_t arg) 
{
  uint8_t CANIdx = Can_GetControllerIdx(CAN);
  if (can_driver_powered[CANIdx] == 0U) { return ARM_DRIVER_ERROR; }

  switch (operation) {
    case ARM_CAN_FILTER_ID_EXACT_ADD:
      return Can_SetFilterExactId(CAN, ObjIdx, id);
    case ARM_CAN_FILTER_ID_EXACT_REMOVE:
      return Can_RemoveFilterExactId(CAN, ObjIdx, id);
    case ARM_CAN_FILTER_ID_MASKABLE_ADD:
      return Can_SetFilterMask(CAN, ObjIdx, id, arg);
    case ARM_CAN_FILTER_ID_MASKABLE_REMOVE:
      return Can_RemoveFilterMask(CAN, ObjIdx, id, arg);
    default:
      return ARM_DRIVER_ERROR_UNSUPPORTED;
  }
}

static int32_t 
ARM_CAN0_ObjectConfigure (uint32_t ObjIdx, 
                         ARM_CAN_OBJ_CONFIG ObjCfg) 
{
  return ARM_CAN_ObjectConfigure(CAN0, ObjIdx, ObjCfg);
}
static int32_t 
ARM_CAN1_ObjectConfigure (uint32_t ObjIdx, 
                         ARM_CAN_OBJ_CONFIG ObjCfg) 
{
  return ARM_CAN_ObjectConfigure(CAN1, ObjIdx, ObjCfg);
}
static int32_t 
ARM_CAN_ObjectConfigure (volatile CanHandle_t* CAN,
                         uint32_t ObjIdx, 
                         ARM_CAN_OBJ_CONFIG ObjCfg) 
{
  int32_t state;
  uint8_t CANIdx = Can_GetControllerIdx(CAN);

  if (can_driver_powered[CANIdx] == 0U) { return ARM_DRIVER_ERROR; }

  //stop the message by marking it as invalid
  state = Can_WriteReadMsgObj(CAN, MSG_OBJ_R, ObjIdx,
   CANIFCMSK_ARB_Msk | CANIFCMSK_CONTROL_Msk);
  if(state != ARM_DRIVER_OK) return state;

  CAN->IF1.ARB2 &= ~CANIFARB2_MSGVAL_Msk;

  state = Can_WriteReadMsgObj(CAN, MSG_OBJ_W, ObjIdx,
     CANIFCMSK_ARB_Msk | CANIFCMSK_CONTROL_Msk);
    if(state != ARM_DRIVER_OK) return state;

    CAN->IF1.MCTL = CANIFMCTL_EOB_Msk;
  switch (ObjCfg) {
    case ARM_CAN_OBJ_INACTIVE:
      CAN->IF1.ARB2 &= ~CANIFARB2_MSGVAL_Msk;
      break;
    case ARM_CAN_OBJ_RX_RTR_TX_DATA:
      CAN->IF1.ARB2 |= CANIFARB2_MSGVAL_Msk | CANIFARB2_DIR_Msk;
      CAN->IF1.MCTL &= ~CANIFMCTL_RXIE_Msk;
      CAN->IF1.MCTL |= CANIFMCTL_RMTEN_Msk | CANIFMCTL_TXIE_Msk;
      break;
    case ARM_CAN_OBJ_TX_RTR_RX_DATA:
      CAN->IF1.ARB2 &= ~CANIFARB2_DIR_Msk;
      CAN->IF1.ARB2 |= CANIFARB2_MSGVAL_Msk;
      CAN->IF1.MCTL &= ~(CANIFMCTL_RMTEN_Msk | CANIFMCTL_TXIE_Msk);
      CAN->IF1.MCTL |= CANIFMCTL_RXIE_Msk;
      break;
    case ARM_CAN_OBJ_TX:
      CAN->IF1.ARB2 |= CANIFARB2_DIR_Msk | CANIFARB2_MSGVAL_Msk;
      CAN->IF1.MCTL &= ~(CANIFMCTL_RMTEN_Msk | CANIFMCTL_RXIE_Msk);
      CAN->IF1.MCTL |= CANIFMCTL_TXIE_Msk;
      break;
    case ARM_CAN_OBJ_RX:
      CAN->IF1.ARB2 &= ~CANIFARB2_DIR_Msk;
      CAN->IF1.ARB2 |= CANIFARB2_MSGVAL_Msk;
      CAN->IF1.MCTL &= ~(CANIFMCTL_RMTEN_Msk | CANIFMCTL_TXIE_Msk);
      CAN->IF1.MCTL |= CANIFMCTL_RXIE_Msk;
      break;
    default:
      return ARM_DRIVER_ERROR_UNSUPPORTED;
  }

  state = Can_WriteReadMsgObj(CAN, MSG_OBJ_W, ObjIdx,
   CANIFCMSK_ARB_Msk | CANIFCMSK_CONTROL_Msk);
  if(state != ARM_DRIVER_OK) return state;
  
  CANIdx = Can_GetControllerIdx(CAN);
  gObjCfgs[CANIdx][ObjIdx] = ObjCfg;

  return ARM_DRIVER_OK;
}

static int32_t 
ARM_CAN0_MessageSend(uint32_t ObjIdx, 
                     ARM_CAN_MSG_INFO *MsgInfo, 
                     const uint8_t *Data, 
                     uint8_t Size) 
{
  return ARM_CAN_MessageSend(CAN0, ObjIdx, MsgInfo, Data, Size);
}
static int32_t 
ARM_CAN1_MessageSend(uint32_t ObjIdx, 
                     ARM_CAN_MSG_INFO *MsgInfo, 
                     const uint8_t *Data, 
                     uint8_t Size) 
{
  return ARM_CAN_MessageSend(CAN1, ObjIdx, MsgInfo, Data, Size);
}

static int32_t 
ARM_CAN_MessageSend(volatile CanHandle_t* CAN,
                    uint32_t ObjIdx, 
                    ARM_CAN_MSG_INFO *MsgInfo, 
                    const uint8_t *Data, 
                    uint8_t Size) 
{
  int32_t state;
  uint8_t CANIdx = Can_GetControllerIdx(CAN);
  
  if (can_driver_powered[CANIdx] == 0U) { return ARM_DRIVER_ERROR; }

  if(
     (gObjCfgs[CANIdx][ObjIdx] != ARM_CAN_OBJ_TX_RTR_RX_DATA &&
      gObjCfgs[CANIdx][ObjIdx] != ARM_CAN_OBJ_TX &&
      gObjCfgs[CANIdx][ObjIdx] != ARM_CAN_OBJ_RX_RTR_TX_DATA) ||
     (gObjCfgs[CANIdx][ObjIdx] == ARM_CAN_OBJ_TX_RTR_RX_DATA && MsgInfo->rtr != 1) ||
     (MsgInfo == NULL) ||
     (MsgInfo->dlc > 8 || MsgInfo->dlc < 1) ||
     ((gObjCfgs[CANIdx][ObjIdx] == ARM_CAN_OBJ_RX_RTR_TX_DATA || gObjCfgs[CANIdx][ObjIdx] == ARM_CAN_OBJ_TX) &&
     (Data == NULL))
     )
    {
      return ARM_DRIVER_ERROR_SPECIFIC;
    }
  
  if((ObjIdx >= 16 && CAN->MSGOBJ.XRQ2 & (1 << ObjIdx)) ||
     (ObjIdx < 16 && CAN->MSGOBJ.XRQ1 & (1 << ObjIdx)))
  {
      return ARM_DRIVER_ERROR_BUSY;
  }

  state = Can_WriteReadMsgObj(CAN, MSG_OBJ_R, ObjIdx,
   CANIFCMSK_ARB_Msk | CANIFCMSK_CONTROL_Msk | CANIFCMSK_DATAA_Msk | 
   CANIFCMSK_DATAB_Msk | CANIFCMSK_MASK_Msk);
  if(state != ARM_DRIVER_OK) return state;

  //Make sure the message is invalid before configuration
  CAN->IF1.ARB2 &= ~CANIFARB2_MSGVAL_Msk;
  state = Can_WriteReadMsgObj(CAN, MSG_OBJ_W, ObjIdx, CANIFCMSK_ARB_Msk);
  if(state != ARM_DRIVER_OK) return state;

  if(MsgInfo->id & ARM_CAN_ID_IDE_Msk)
    {
      state = Can_SetXtdId(CAN, MsgInfo->id & ~(ARM_CAN_ID_IDE_Msk));
      if(state != ARM_DRIVER_OK) return state;
    }
  else
    {
      state = Can_SetStdId(CAN, MsgInfo->id);
      if(state != ARM_DRIVER_OK) return state;
    }

  if(gObjCfgs[CANIdx][ObjIdx] == ARM_CAN_OBJ_TX ||
     gObjCfgs[CANIdx][ObjIdx] == ARM_CAN_OBJ_RX_RTR_TX_DATA)
    {
      Can_SetData(CAN, Data, Min(MsgInfo->dlc, Size));
    }
  else
   {
      CAN->IF1.MCTL &= ~(CANIFMCTL_DLC_Msk);
      CAN->IF1.MCTL |= MsgInfo->dlc << CANIFMCTL_DLC_Pos;
   }

  if(gObjCfgs[CANIdx][ObjIdx] == ARM_CAN_OBJ_TX ||
     gObjCfgs[CANIdx][ObjIdx] == ARM_CAN_OBJ_TX_RTR_RX_DATA)
    {
        CAN->IF1.MCTL |= CANIFMCTL_TXRQST_Msk;
    }

  CAN->IF1.ARB2 |= CANIFARB2_MSGVAL_Msk;

  state = Can_WriteReadMsgObj(CAN, MSG_OBJ_W, ObjIdx,
   CANIFCMSK_ARB_Msk | CANIFCMSK_CONTROL_Msk | CANIFCMSK_DATAA_Msk | 
   CANIFCMSK_DATAB_Msk | CANIFCMSK_MASK_Msk);
  if(state != ARM_DRIVER_OK) return state;

  if(gObjCfgs[CANIdx][ObjIdx] == ARM_CAN_OBJ_TX)
    {
      return ((int32_t)Min(MsgInfo->dlc, Size));
    }
  else 
    {
      return 0;
    }
}

static int32_t 
ARM_CAN0_MessageRead(uint32_t ObjIdx, 
                     ARM_CAN_MSG_INFO *MsgInfo, 
                     uint8_t *Data, 
                     uint8_t Size) 
{
  return ARM_CAN_MessageRead(CAN0, ObjIdx, MsgInfo, Data, Size);
}
static int32_t 
ARM_CAN1_MessageRead(uint32_t ObjIdx, 
                     ARM_CAN_MSG_INFO *MsgInfo, 
                     uint8_t *Data, 
                     uint8_t Size) 
{
  return ARM_CAN_MessageRead(CAN1, ObjIdx, MsgInfo, Data, Size);
}

static int32_t 
ARM_CAN_MessageRead(volatile CanHandle_t* CAN,
                    uint32_t ObjIdx,
                    ARM_CAN_MSG_INFO *MsgInfo,
                    uint8_t *Data,
                    uint8_t Size) 
{
  int32_t state;
  uint8_t dlc;
  uint8_t CANIdx = Can_GetControllerIdx(CAN);

  if (can_driver_powered[CANIdx] == 0U) return ARM_DRIVER_ERROR;

  if(
    (gObjCfgs[CANIdx][ObjIdx] != ARM_CAN_OBJ_TX_RTR_RX_DATA &&
    gObjCfgs[CANIdx][ObjIdx] != ARM_CAN_OBJ_RX) ||
    (gObjCfgs[CANIdx][ObjIdx] == ARM_CAN_OBJ_TX_RTR_RX_DATA && MsgInfo->rtr != 1) ||
    (Data == NULL)
    )
    {
      return ARM_DRIVER_ERROR_SPECIFIC;
    }

    state = Can_WriteReadMsgObj(CAN, MSG_OBJ_R, ObjIdx,
    CANIFCMSK_DATAB_Msk | CANIFCMSK_DATAA_Msk | CANIFCMSK_CONTROL_Msk);
    if(state != ARM_DRIVER_OK) return state;

    if(!(CAN->IF1.MCTL & CANIFMCTL_NEWDAT_Msk))
      {
        return ARM_DRIVER_ERROR_SPECIFIC;
      }
    else
      {
        CAN->IF1.MCTL &= ~CANIFMCTL_NEWDAT_Msk;

        dlc = (CAN->IF1.MCTL & CANIFMCTL_DLC_Msk) >> CANIFMCTL_DLC_Pos;
        Can_ReadData(CAN, Data, Min(dlc, Size));
      }

    state = Can_WriteReadMsgObj(CAN, MSG_OBJ_W, ObjIdx, CANIFCMSK_CONTROL_Msk);
    if(state != ARM_DRIVER_OK) return state;

  return ((int32_t)Min(dlc, Size));
}

static int32_t 
ARM_CAN0_Control(uint32_t control, uint32_t arg) 
{
  return ARM_CAN_Control(CAN0, control, arg);
}
static int32_t 
ARM_CAN1_Control(uint32_t control, uint32_t arg) 
{
  return ARM_CAN_Control(CAN1, control, arg);
}
static int32_t 
ARM_CAN_Control (volatile CanHandle_t* CAN, uint32_t control, uint32_t arg) 
{
  int32_t state;
  uint8_t CANIdx = Can_GetControllerIdx(CAN);

  if (can_driver_powered[CANIdx] == 0U) { return ARM_DRIVER_ERROR; }

  switch (control & ARM_CAN_CONTROL_Msk) {
    case ARM_CAN_ABORT_MESSAGE_SEND:
      state = Can_WriteReadMsgObj(CAN, MSG_OBJ_R, arg, CANIFCMSK_CONTROL_Msk);
      if(state != ARM_DRIVER_OK) return state;
      CAN->IF1.MCTL &= ~CANIFMCTL_TXRQST_Msk;
      state = Can_WriteReadMsgObj(CAN, MSG_OBJ_W, arg, CANIFCMSK_CONTROL_Msk);
      if(state != ARM_DRIVER_OK) return state;
      break;
    case ARM_CAN_CONTROL_RETRANSMISSION:
      if(arg == 0)
        {
          CAN->CTL.CTL |= CANCTL_DAR_Msk;
        }
      else
        {
          CAN->CTL.CTL &= ~CANCTL_DAR_Msk;
        }
      break;
    default:
      // Handle unknown control code
      return ARM_DRIVER_ERROR_UNSUPPORTED;
  }

  return ARM_DRIVER_OK;
}

static ARM_CAN_STATUS 
ARM_CAN0_GetStatus (void)
{
  return ARM_CAN_GetStatus(CAN0);
} 
static ARM_CAN_STATUS 
ARM_CAN1_GetStatus (void)
{
  return ARM_CAN_GetStatus(CAN1);
} 

static ARM_CAN_STATUS 
ARM_CAN_GetStatus (volatile CanHandle_t* CAN) 
{

  ARM_CAN_STATUS status;
  uint32_t LEC;
  status.tx_error_count = (CAN->CTL.ERR & CANERR_TEC_Msk) >> CANERR_TEC_Pos;
  status.rx_error_count = (CAN->CTL.ERR & CANERR_REC_Msk) >> CANERR_REC_Pos;

  if(CAN->CTL.CTL & CANCTL_INIT_Msk)
    {
      status.unit_state = ARM_CAN_UNIT_STATE_INACTIVE;
    }
  else if(CAN->CTL.STS & CANSTS_BOFF_Msk)
    {
      status.unit_state = ARM_CAN_UNIT_STATE_BUS_OFF;
    }
  else if(CAN->CTL.ERR & CANERR_RP_Msk || status.tx_error_count > 127)
    {
      status.unit_state = ARM_CAN_UNIT_STATE_PASSIVE;
    }
  else
    {
      status.unit_state = ARM_CAN_UNIT_STATE_ACTIVE;
    }

  LEC = (CAN->CTL.STS & CANSTS_LEC_Msk) >> CANSTS_LEC_Pos;
  switch(LEC)
    {
      case CANSTS_LEC_NO_ERR:
      status.last_error_code = ARM_CAN_LEC_NO_ERROR;
      break;
      case CANSTS_LEC_STUFF_ERR:
      status.last_error_code = ARM_CAN_LEC_STUFF_ERROR;
      break;
      case CANSTS_LEC_FORMAT_ERR:
      status.last_error_code = ARM_CAN_LEC_FORM_ERROR;
      break;
      case CANSTS_LEC_ACK_ERR:
      status.last_error_code = ARM_CAN_LEC_ACK_ERROR;
      break;
      case CANSTS_LEC_BIT1_ERR:
      status.last_error_code = ARM_CAN_LEC_BIT_ERROR;
      break;
      case CANSTS_LEC_BIT0_ERR:
      status.last_error_code = ARM_CAN_LEC_BIT_ERROR;
      break;
      case CANSTS_LEC_CRC_ERR:
      status.last_error_code = ARM_CAN_LEC_CRC_ERROR;
      break;
      case CANSTS_LEC_NOEVENT_ERR:
      status.last_error_code = ARM_CAN_LEC_NO_ERROR;
      break;
      default:
      //not reachable
      break;
    }
  return status;
}

/**
 * @brief Set timeout for a specifc flag to be cleared or set
 * 
 * @param Reg 
 * @param FlagPos 
 * @param SetClr The flag should be set or cleared
 * @param Timeout 
 * @return int32_t 
 */
int32_t 
Can_SetTimeout(volatile uint32_t* Reg, 
              uint32_t FlagPos, 
              CanFlag_t SetClr, 
              uint32_t Timeout)
{
  uint32_t timeout = 0xFFFF;
  if(SetClr == CAN_FLAG_CLEAR)
    {
      while(timeout != 0 && (*Reg & (1 << FlagPos)))
        {
          timeout--;
        }
    }
  else 
    {
      while(timeout != 0 && !(*Reg & (1 << FlagPos)))
        {
          timeout--;
        }
    }
  if(timeout == 0) 
    {
      return ARM_DRIVER_ERROR_TIMEOUT;
    }
  else 
    {
      return ARM_DRIVER_OK;
    }
}

static inline uint32_t 
Min(uint32_t A, uint32_t B)
{
  return A < B ? A : B;
}

static inline uint32_t 
Max(uint32_t A, uint32_t B)
{
  return A > B ? A : B;
}

static int32_t 
Can_RemoveFilterMask(volatile CanHandle_t* CAN,
                     uint32_t ObjIdx,
                     uint32_t Id,
                     uint32_t Mask)
{
  uint8_t CANIdx = Can_GetControllerIdx(CAN);
  if(gFilters[CANIdx][ObjIdx].Valid == INVALID_FILTER) return ARM_DRIVER_ERROR_SPECIFIC;
  if(gFilters[CANIdx][ObjIdx].Id != Id) return ARM_DRIVER_ERROR_SPECIFIC;
  if(gFilters[CANIdx][ObjIdx].Mask != Mask) return ARM_DRIVER_ERROR_SPECIFIC;
  gFilters[CANIdx][ObjIdx].Valid = INVALID_FILTER;

  return ARM_DRIVER_OK;
}

static int32_t 
Can_RemoveFilterExactId(volatile CanHandle_t* CAN,
                        uint32_t ObjIdx,
                        uint32_t Id)
{
  uint8_t CANIdx = Can_GetControllerIdx(CAN);

  if(gFilters[CANIdx][ObjIdx].Valid == INVALID_FILTER) return ARM_DRIVER_ERROR_SPECIFIC;
  if(gFilters[CANIdx][ObjIdx].Id != gFilters[CANIdx][ObjIdx].Mask) return ARM_DRIVER_ERROR_SPECIFIC;
  if(gFilters[CANIdx][ObjIdx].Id != Id) return ARM_DRIVER_ERROR_SPECIFIC;
  gFilters[CANIdx][ObjIdx].Valid = INVALID_FILTER;

  return ARM_DRIVER_OK;
}

static int32_t 
Can_SetFilterMask(volatile CanHandle_t* CAN,
                     uint32_t ObjIdx,
                     uint32_t Id,
                     uint32_t Mask)
{
  int32_t state;
  uint8_t CANIdx = Can_GetControllerIdx(CAN);

  if(gFilters[CANIdx][ObjIdx].Valid == VALID_FILTER) return ARM_DRIVER_ERROR_BUSY;

  state = Can_WriteReadMsgObj(CAN, MSG_OBJ_R, ObjIdx, CANIFCMSK_CONTROL_Msk);
  if(state != ARM_DRIVER_OK) return state;

  //filter based on whether the id is standard or extended
  CAN->IF1.MSK2 |= CANIFMSK2_MXTD_Msk;
  //don't filter based on the message object direction
  CAN->IF1.MSK2 &= ~CANIFMSK2_MDIR_Msk;

  if(Id & ARM_CAN_ID_IDE_Msk)
    {
      state = Can_SetXtdMask(CAN, Mask);
      if(state != ARM_DRIVER_OK) return state;

      state = Can_SetXtdId(CAN, Id & ~(ARM_CAN_ID_IDE_Msk));
      if(state != ARM_DRIVER_OK) return state;
    }
  else
    {
      state = Can_SetStdMask(CAN, Mask);
      if(state != ARM_DRIVER_OK) return state;

      state = Can_SetStdId(CAN, Id);
      if(state != ARM_DRIVER_OK) return state;
    }

  //Masks are considered
  CAN->IF1.MCTL |= CANIFMCTL_UMASK_Msk;

  state = Can_WriteReadMsgObj(CAN, MSG_OBJ_W, ObjIdx, 
  CANIFCMSK_CONTROL_Msk | CANIFCMSK_ARB_Msk | CANIFCMSK_MASK_Msk);
  if(state != ARM_DRIVER_OK) return state;

  gFilters[CANIdx][ObjIdx].Valid = VALID_FILTER;
  gFilters[CANIdx][ObjIdx].Id = Id;
  gFilters[CANIdx][ObjIdx].Mask = Mask;

  return ARM_DRIVER_OK;
}

static int32_t 
Can_SetFilterExactId(volatile CanHandle_t* CAN,
                     uint32_t ObjIdx,
                     uint32_t Id)
{
  int32_t state;

  if(Id & ARM_CAN_ID_IDE_Msk)
    {
      state = Can_SetFilterMask(CAN, ObjIdx, Id, MAX_XTD_ID);
    }
  else
    {
      state = Can_SetFilterMask(CAN, ObjIdx, Id, MAX_STD_ID);
    }

  return state;
}

static int32_t 
Can_WriteReadMsgObj(volatile CanHandle_t* CAN, MsgObjWR_t Flag, uint32_t ObjIdx, uint32_t Mask)
{
  CAN->IF1.CMSK = Mask; 

  if(Flag == MSG_OBJ_W)
    {
      CAN0->IF1.CMSK |= CANIFCMSK_WRNRD_Msk;
    }
  else 
    {
      // DO NOTHING
    }
  
  CAN->IF1.CRQ  = (ObjIdx + 1) << CANIFCRQ_MNUM_Pos; 

  return Can_SetTimeout(&(CAN->IF1.CRQ), CANIFCRQ_BUSY_Pos, CAN_FLAG_CLEAR, 5);
}

static int32_t 
Can_SetXtdId(volatile CanHandle_t* CAN, uint32_t XtdId)
{
  if(XtdId > MAX_XTD_ID)
    {
      return ARM_DRIVER_ERROR_PARAMETER;
    }
  else
    {
      CAN->IF1.ARB1 &= ~CANIFARB1_XTDID_Msk;
      CAN->IF1.ARB2 &= ~CANIFARB2_ID_Msk;
      CAN->IF1.ARB1 |= (XtdId & 0xFFFF) << CANIFARB1_XTDID_Pos;
      CAN->IF1.ARB2 |= (XtdId >> 16) << CANIFARB2_XTDID_Pos;
      CAN->IF1.ARB2 |= CANIFARB2_XTD_Msk;
      return ARM_DRIVER_OK;
    }
}

static int32_t 
Can_SetStdId(volatile CanHandle_t* CAN, uint32_t StdId)
{
  if(StdId > MAX_STD_ID)
    {
      return ARM_DRIVER_ERROR_PARAMETER;
    }
  else
    {
      CAN->IF1.ARB2 &= ~CANIFARB2_ID_Msk;
      CAN->IF1.ARB2 |= StdId << CANIFARB2_STDID_Pos;
      CAN->IF1.ARB2 &= ~(1 << CANIFARB2_XTD_Pos);
      return ARM_DRIVER_OK;
    }
}

static int32_t 
Can_SetXtdMask(volatile CanHandle_t* CAN, uint32_t XtdMask)
{
  if(XtdMask > MAX_XTD_ID)
    {
      return ARM_DRIVER_ERROR_PARAMETER;
    }
  else
    {
      CAN->IF1.MSK1 &= ~CANIFMSK1_XTD_Msk;
      CAN->IF1.MSK2 &= ~CANIFMSK2_ID_Msk;
      CAN->IF1.MSK1 |= (XtdMask & 0xFFFF) << CANIFMSK1_XTD_Pos;
      CAN->IF1.MSK2 |= (XtdMask >> 16) << CANIFMSK2_XTD_Pos;
      return ARM_DRIVER_OK;
    }
}

static int32_t 
Can_SetStdMask(volatile CanHandle_t* CAN, uint32_t StdMask)
{
  if(StdMask > MAX_STD_ID)
    {
      return ARM_DRIVER_ERROR_PARAMETER;
    }
  else
    {
      CAN->IF1.MSK1 &= ~CANIFMSK1_XTD_Msk;
      CAN->IF1.MSK2 &= ~CANIFMSK2_ID_Msk;
      CAN->IF1.MSK2 |= StdMask << CANIFMSK2_STD_Pos;
      return ARM_DRIVER_OK;
    }
}

static void 
Can_ReadData(volatile CanHandle_t* CAN,
             uint8_t *Data, 
             uint8_t Size)
{
  uint8_t i;

  for(i = 0; i < Size; i++)
    {
      if(i < 2)
        {
          Data[i] = (uint8_t)((CAN->IF1.DA1 >> ((i % 2) * 8)) & 0xFF);
        }
      else if(i < 4)
        {
          Data[i] = (uint8_t)((CAN->IF1.DA2 >> ((i % 2) * 8)) & 0xFF);
        }
      else if(i < 6)
        {
          Data[i] = (uint8_t)((CAN->IF1.DB1 >> ((i % 2) * 8)) & 0xFF);
        }
      else
        {
          Data[i] = (uint8_t)((CAN->IF1.DB2 >> ((i % 2) * 8)) & 0xFF);
        }
    }
}

static void 
Can_SetData(volatile CanHandle_t* CAN,
            const uint8_t *Data, 
            uint8_t Size)
{
  uint8_t i;

  CAN->IF1.MCTL &= ~(CANIFMCTL_DLC_Msk);
  CAN->IF1.MCTL |= Size << CANIFMCTL_DLC_Pos;
  CAN->IF1.DA1 &= ~(0xFFFF);
  CAN->IF1.DA2 &= ~(0xFFFF);
  CAN->IF1.DB1 &= ~(0xFFFF);
  CAN->IF1.DB2 &= ~(0xFFFF);

  for(i = 0; i < Size; i++)
    {
      if(i < 2)
        {
          CAN->IF1.DA1 |= Data[i] << ((i % 2) * 8);
        }
      else if(i < 4)
        {
          CAN->IF1.DA2 |= Data[i] << ((i % 2) * 8);
        }
      else if(i < 6)
        {
          CAN->IF1.DB1 |= Data[i] << ((i % 2) * 8);
        }
      else
        {
          CAN->IF1.DB2 |= Data[i] << ((i % 2) * 8);
        }
    }
}

uint8_t
Can_GetControllerIdx(volatile CanHandle_t* CAN)
{
  if(CAN == CAN0) return 0;
  return 1;
}

// IRQ handlers
void 
CAN0_Handler(void) 
{
  CAN_Handler(CAN0);
}

void 
CAN1_Handler(void) 
{
  CAN_Handler(CAN1);
}

void
CAN_Handler(volatile CanHandle_t* CAN)
{
  uint32_t IntSource;
  uint32_t UnitSource;
  uint32_t ObjIdx;
  uint8_t CANIdx = Can_GetControllerIdx(CAN);
  if (can_driver_powered[CANIdx] == 0U) return;
  if(CAN_SignalObjectEvent[CANIdx] == NULL || CAN_SignalUnitEvent[CANIdx] == NULL) return;

  IntSource = CAN->CTL.INT;

  //this line also clears CANCTLINT register and the active interrupt flag
  UnitSource = CAN->CTL.STS; 

  if(IntSource > MAX_OBJ_NUM)
    {
      if(UnitSource & CANSTS_BOFF_Msk) 
        {
          CAN_SignalUnitEvent[CANIdx](ARM_CAN_EVENT_UNIT_BUS_OFF); 
          return;
        }
      else if(UnitSource & CANSTS_EWARN_Msk)
        {
          CAN_SignalUnitEvent[CANIdx](ARM_CAN_EVENT_UNIT_WARNING); 
          return;
        }
      else 
        {
          //DO NOTHING
        }
    }
  else if (IntSource == 0)
    {
      //NOT REACHABLE
    }
  else 
    {
      ObjIdx = IntSource - 1;
      Can_WriteReadMsgObj(CAN, MSG_OBJ_R, ObjIdx, CANIFCMSK_CONTROL_Msk);
      CAN->IF1.MCTL &= ~CANIFMCTL_INTPND_Msk;

      if(CAN->IF1.MCTL & CANIFMCTL_RXIE_Msk)
        {
          if(CAN->IF1.MCTL & CANIFMCTL_NEWDAT_Msk)
              {
                CAN_SignalObjectEvent[CANIdx](ObjIdx, ARM_CAN_EVENT_RECEIVE);
              }
          if(CAN->IF1.MCTL & CANIFMCTL_MSGLST_Msk)
              {
                CAN_SignalObjectEvent[CANIdx](ObjIdx, ARM_CAN_EVENT_RECEIVE_OVERRUN);
                CAN->IF1.MCTL &= ~CANIFMCTL_MSGLST_Msk;
              }
        }

      if(CAN->IF1.MCTL & CANIFMCTL_TXIE_Msk)
          {
            CAN_SignalObjectEvent[CANIdx](ObjIdx, ARM_CAN_EVENT_SEND_COMPLETE);
          }

      Can_WriteReadMsgObj(CAN, MSG_OBJ_W, ObjIdx, CANIFCMSK_CONTROL_Msk);
    }
}

// CAN driver functions structure

extern \
ARM_DRIVER_CAN Driver_CAN0;
ARM_DRIVER_CAN Driver_CAN0 = {
  ARM_CAN_GetVersion,
  ARM_CAN_GetCapabilities,
  ARM_CAN0_Initialize,
  ARM_CAN0_Uninitialize,
  ARM_CAN0_PowerControl,
  ARM_CAN_GetClock,
  ARM_CAN0_SetBitrate,
  ARM_CAN0_SetMode,
  ARM_CAN_ObjectGetCapabilities,
  ARM_CAN0_ObjectSetFilter,
  ARM_CAN0_ObjectConfigure,
  ARM_CAN0_MessageSend,
  ARM_CAN0_MessageRead,
  ARM_CAN0_Control,
  ARM_CAN0_GetStatus
};

extern \
ARM_DRIVER_CAN Driver_CAN1;
ARM_DRIVER_CAN Driver_CAN1 = {
  ARM_CAN_GetVersion,
  ARM_CAN_GetCapabilities,
  ARM_CAN1_Initialize,
  ARM_CAN1_Uninitialize,
  ARM_CAN1_PowerControl,
  ARM_CAN_GetClock,
  ARM_CAN1_SetBitrate,
  ARM_CAN1_SetMode,
  ARM_CAN_ObjectGetCapabilities,
  ARM_CAN1_ObjectSetFilter,
  ARM_CAN1_ObjectConfigure,
  ARM_CAN1_MessageSend,
  ARM_CAN1_MessageRead,
  ARM_CAN1_Control,
  ARM_CAN1_GetStatus
};

