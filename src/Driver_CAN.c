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

// Driver Version
static const ARM_DRIVER_VERSION can_driver_version = { ARM_CAN_API_VERSION, ARM_CAN_DRV_VERSION };
static inline uint32_t Min(uint32_t A, uint32_t B);
static inline uint32_t Max(uint32_t A, uint32_t B);
static int32_t Can_SetFilterExactId(volatile CanHandle_t* CAN, uint32_t ObjIdx, uint32_t Id);
static int32_t Can_RemoveFilterExactId(volatile CanHandle_t* CAN, uint32_t ObjIdx, uint32_t Id);
static int32_t Can_SetFilterMask(volatile CanHandle_t* CAN, uint32_t ObjIdx, uint32_t Id, uint32_t Mask);
static int32_t Can_RemoveFilterMask(volatile CanHandle_t* CAN, uint32_t ObjIdx, uint32_t Id, uint32_t Mask);
static int32_t Can_WriteReadMsgObj(volatile CanHandle_t* CAN, MsgObjWR_t Flag, uint32_t ObjIdx, uint32_t Mask);
static int32_t Can_SetXtdId(volatile CanHandle_t* CAN, uint32_t ObjIdx, uint32_t XtdId);
static int32_t Can_SetStdId(volatile CanHandle_t* CAN, uint32_t ObjIdx, uint32_t StdId);
static int32_t Can_SetXtdMask(volatile CanHandle_t* CAN, uint32_t ObjIdx, uint32_t XtdMask);
static int32_t Can_SetStdMask(volatile CanHandle_t* CAN, uint32_t ObjIdx, uint32_t StdMask);
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

static uint8_t                     can_driver_powered     = 0U;
static uint8_t                     can_driver_initialized = 0U;
static ARM_CAN_SignalUnitEvent_t   CAN_SignalUnitEvent    = NULL;
static ARM_CAN_SignalObjectEvent_t CAN_SignalObjectEvent  = NULL;
static CanFilter_t gFilters[MAX_OBJ_NUM];

//
//   Functions prototypes
//
int32_t 
Can_SetTimeout(volatile uint32_t* Reg, 
              uint32_t FlagPos, 
              CanFlag_t SetClr, 
              uint32_t Timeout);

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

static int32_t ARM_CAN_Initialize (ARM_CAN_SignalUnitEvent_t   cb_unit_event,
                                   ARM_CAN_SignalObjectEvent_t cb_object_event) {

  if (can_driver_initialized != 0U) { return ARM_DRIVER_OK; }

  CAN_SignalUnitEvent   = cb_unit_event;
  CAN_SignalObjectEvent = cb_object_event;

  can_driver_initialized = 1U;

  return ARM_DRIVER_OK;
}

static int32_t ARM_CAN_Uninitialize (void) {
  can_driver_initialized = 0U;

  return ARM_DRIVER_OK;
}

static int32_t ARM_CAN_PowerControl (ARM_POWER_STATE state) {

  switch (state) {
    case ARM_POWER_OFF:
      can_driver_powered = 0U;
      // Add code to disable interrupts and put peripheral into reset mode,
      // and if possible disable clock
      // ..
      SRCAN |= 1 << SRCAN_CAN0; //reset
      Can_SetTimeout(&PRCAN, PRCAN_CAN0, CAN_FLAG_SET, 0xFFFF);

      RCGCCAN &= ~(1 << RCGCCAN_CAN0); //disable clock
      Can_SetTimeout(&PRCAN, PRCAN_CAN0, CAN_FLAG_CLEAR, 0xFFFF);
      break;

    case ARM_POWER_FULL:
      if (can_driver_initialized == 0U) { return ARM_DRIVER_ERROR; }
      if (can_driver_powered     != 0U) { return ARM_DRIVER_OK;    }
      
      uint8_t i;
      // Add code to enable clocks, reset variables enable interrupts
      // and put peripheral into operational
      // ..
      RCGCCAN |= (1 << 0); //enable clock
      Can_SetTimeout(&PRCAN, PRCAN_CAN0, CAN_FLAG_SET, 0xFFFF);

      can_driver_powered = 1U;
      can_driver_initialized = 0U;
      CAN_SignalUnitEvent    = NULL;
      CAN_SignalObjectEvent  = NULL;
      for(i = 0; i < MAX_OBJ_NUM; i++)
        {
          gFilters[i].Valid = INVALID_FILTER;
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
}

static int32_t ARM_CAN_SetBitrate (ARM_CAN_BITRATE_SELECT select, uint32_t bitrate, uint32_t bit_segments) {

  if (can_driver_powered == 0U) { return ARM_DRIVER_ERROR; }

  // Add code to setup peripheral parameters to generate specified bitrate
  // with specified bit segments
  // ..
  uint32_t BRP;
  uint32_t Phase1;
  uint32_t Phase2;
  uint32_t Prop;
  uint32_t SJW;
  uint32_t N;

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

static int32_t ARM_CAN_SetMode (volatile CanHandle_t* CAN, ARM_CAN_MODE mode) {

  if (can_driver_powered == 0U) { return ARM_DRIVER_ERROR; }

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
ARM_CAN_ObjectSetFilter(volatile CanHandle_t* CAN,
                        uint32_t ObjIdx,
                        ARM_CAN_FILTER_OPERATION operation, 
                        uint32_t id, 
                        uint32_t arg) 
{
  if (can_driver_powered == 0U) { return ARM_DRIVER_ERROR; }

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

static int32_t ARM_CAN_ObjectConfigure (uint32_t ObjIdx, ARM_CAN_OBJ_CONFIG obj_cfg) {

  if (can_driver_powered == 0U) { return ARM_DRIVER_ERROR; }

  switch (obj_cfg) {
    case ARM_CAN_OBJ_INACTIVE:
      // Deactivate object
      // ..
      break;
    case ARM_CAN_OBJ_RX_RTR_TX_DATA:
      // Setup object to automatically return data when RTR with it's ID is received
      // ..
      break;
    case ARM_CAN_OBJ_TX_RTR_RX_DATA:
      // Setup object to send RTR and receive data response
      // ..
      break;
    case ARM_CAN_OBJ_TX:
      // Setup object to be used for sending messages
      // ..
      break;
    case ARM_CAN_OBJ_RX:
      // Setup object to be used for receiving messages
      // ..
      break;
  }

  return ARM_DRIVER_OK;
}

static int32_t ARM_CAN_MessageSend (uint32_t ObjIdx, ARM_CAN_MSG_INFO *msg_info, const uint8_t *data, uint8_t size) {

  if (can_driver_powered == 0U) { return ARM_DRIVER_ERROR; }

  // Add code to send requested message
  // ..

  return ((int32_t)size);
}

static int32_t ARM_CAN_MessageRead (uint32_t ObjIdx, ARM_CAN_MSG_INFO *msg_info, uint8_t *data, uint8_t size) {

  if (can_driver_powered == 0U) { return ARM_DRIVER_ERROR;  }

  // Add code to read previously received message
  // (reception was started when object was configured for reception)
  // ..

  return ((int32_t)size);
}

static int32_t ARM_CAN_Control (uint32_t control, uint32_t arg) {

  if (can_driver_powered == 0U) { return ARM_DRIVER_ERROR; }

  switch (control & ARM_CAN_CONTROL_Msk) {
    case ARM_CAN_ABORT_MESSAGE_SEND:
      // Add code to abort message pending to be sent
      // ..
      break;
    case ARM_CAN_SET_FD_MODE:
      // Add code to enable Flexible Data-rate mode
      // ..
      break;
    case ARM_CAN_SET_TRANSCEIVER_DELAY:
      // Add code to set transceiver delay
      // ..
      break;
    default:
      // Handle unknown control code
      return ARM_DRIVER_ERROR_UNSUPPORTED;
  }

  return ARM_DRIVER_OK;
}

static ARM_CAN_STATUS ARM_CAN_GetStatus (void) {

  // Add code to return device bus and error status
  // ..
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
  if(gFilters[ObjIdx].Valid == INVALID_FILTER) return ARM_DRIVER_ERROR_SPECIFIC;
  if(gFilters[ObjIdx].Id != Id) return ARM_DRIVER_ERROR_SPECIFIC;
  if(gFilters[ObjIdx].Mask != Mask) return ARM_DRIVER_ERROR_SPECIFIC;
  gFilters[ObjIdx].Valid = INVALID_FILTER;

  return ARM_DRIVER_OK;
}

static int32_t 
Can_RemoveFilterExactId(volatile CanHandle_t* CAN,
                        uint32_t ObjIdx,
                        uint32_t Id)
{
  if(gFilters[ObjIdx].Valid == INVALID_FILTER) return ARM_DRIVER_ERROR_SPECIFIC;
  if(gFilters[ObjIdx].Id != gFilters[ObjIdx].Mask) return ARM_DRIVER_ERROR_SPECIFIC;
  if(gFilters[ObjIdx].Id != Id) return ARM_DRIVER_ERROR_SPECIFIC;
  gFilters[ObjIdx].Valid = INVALID_FILTER;

  return ARM_DRIVER_OK;
}

static int32_t 
Can_SetFilterMask(volatile CanHandle_t* CAN,
                     uint32_t ObjIdx,
                     uint32_t Id,
                     uint32_t Mask)
{
  int32_t state;

  if(gFilters[ObjIdx].Valid == VALID_FILTER) return ARM_DRIVER_ERROR_BUSY;

  state = Can_WriteReadMsgObj(CAN, MSG_OBJ_R, ObjIdx, CANIFCMSK_CONTROL_Msk);
  if(state != ARM_DRIVER_OK) return state;

  //filter based on whether the id is standard or extended
  CAN->IF1.MSK2 |= CANIFMSK2_MXTD_Msk;
  //don't filter based on the message object direction
  CAN->IF1.MSK2 &= ~CANIFMSK2_MDIR_Msk;

  if(Id & ARM_CAN_ID_IDE_Msk)
    {
      state = Can_SetXtdMask(CAN, ObjIdx, Mask);
      if(state != ARM_DRIVER_OK) return state;

      state = Can_SetXtdId(CAN, ObjIdx, Id & ~(ARM_CAN_ID_IDE_Msk));
      if(state != ARM_DRIVER_OK) return state;
    }
  else
    {
      state = Can_SetStdMask(CAN, ObjIdx, Mask);
      if(state != ARM_DRIVER_OK) return state;

      state = Can_SetStdId(CAN, ObjIdx, Id);
      if(state != ARM_DRIVER_OK) return state;
    }

  //Masks are considered
  CAN->IF1.MCTL |= CANIFMCTL_UMASK_Msk;

  state = Can_WriteReadMsgObj(CAN, MSG_OBJ_W, ObjIdx, 
  CANIFCMSK_CONTROL_Msk | CANIFCMSK_ARB_Msk | CANIFCMSK_MASK_Msk);
  if(state != ARM_DRIVER_OK) return state;

  gFilters[ObjIdx].Valid = VALID_FILTER;
  gFilters[ObjIdx].Id = Id;
  gFilters[ObjIdx].Mask = Mask;

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
Can_SetXtdId(volatile CanHandle_t* CAN, uint32_t ObjIdx, uint32_t XtdId)
{
  if(XtdId > MAX_XTD_ID)
    {
      return ARM_DRIVER_ERROR_PARAMETER;
    }
  else
    {
      CAN->IF1.ARB1 &= ~CANIFARB1_XTD_Msk;
      CAN->IF1.ARB2 &= ~CANIFARB2_ID_Msk;
      CAN->IF1.ARB1 |= (XtdId & 0xFFFF) << CANIFARB1_XTD_Pos;
      CAN->IF1.ARB2 |= (XtdId >> 16) << CANIFARB2_XTD_Pos;
      CAN->IF1.ARB2 |= CANIFARB2_MXTD_Msk;
      return ARM_DRIVER_OK;
    }
}

static int32_t 
Can_SetStdId(volatile CanHandle_t* CAN, uint32_t ObjIdx, uint32_t StdId)
{
  if(StdId > MAX_STD_ID)
    {
      return ARM_DRIVER_ERROR_PARAMETER;
    }
  else
    {
      CAN->IF1.ARB2 &= ~CANIFARB2_ID_Msk;
      CAN->IF1.ARB2 |= StdId << CANIFARB2_STD_Pos;
      CAN->IF1.ARB2 &= ~(1 << CANIFARB2_MXTD_Pos);
      return ARM_DRIVER_OK;
    }
}

static int32_t 
Can_SetXtdMask(volatile CanHandle_t* CAN, uint32_t ObjIdx, uint32_t XtdMask)
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
Can_SetStdMask(volatile CanHandle_t* CAN, uint32_t ObjIdx, uint32_t StdMask)
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

// IRQ handlers
// Add interrupt routines to handle transmission, reception, error and status interrupts
// ..

// CAN driver functions structure

extern \
ARM_DRIVER_CAN Driver_CAN0;
ARM_DRIVER_CAN Driver_CAN0 = {
  ARM_CAN_GetVersion,
  ARM_CAN_GetCapabilities,
  ARM_CAN_Initialize,
  ARM_CAN_Uninitialize,
  ARM_CAN_PowerControl,
  ARM_CAN_GetClock,
  ARM_CAN_SetBitrate,
  ARM_CAN0_SetMode,
  ARM_CAN_ObjectGetCapabilities,
  ARM_CAN0_ObjectSetFilter,
  ARM_CAN_ObjectConfigure,
  ARM_CAN_MessageSend,
  ARM_CAN_MessageRead,
  ARM_CAN_Control,
  ARM_CAN_GetStatus
};

