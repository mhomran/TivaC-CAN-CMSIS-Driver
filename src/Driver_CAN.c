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

//typedefs
typedef enum {
  CAN_FLAG_SET,
  CAN_FLAG_CLEAR,
} CanFlag_t;

// Driver Version
static const ARM_DRIVER_VERSION can_driver_version = { ARM_CAN_API_VERSION, ARM_CAN_DRV_VERSION };
static inline uint32_t Min(uint32_t A, uint32_t B);
static inline uint32_t Max(uint32_t A, uint32_t B);

// Driver Capabilities
static const ARM_CAN_CAPABILITIES can_driver_capabilities = {
  32U,  // Number of CAN Objects available
  0U,   // Does not support reentrant calls to ARM_CAN_MessageSend, ARM_CAN_MessageRead, ARM_CAN_ObjectConfigure and abort message sending used by ARM_CAN_Control.
  0U,   // Does not support CAN with Flexible Data-rate mode (CAN_FD)
  1U,   // Does not support restricted operation mode
  0U,   // Does not support bus monitoring mode
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

      // Add code to enable clocks, reset variables enable interrupts
      // and put peripheral into operational
      // ..
      RCGCCAN |= (1 << 0); //enable clock
      Can_SetTimeout(&PRCAN, PRCAN_CAN0, CAN_FLAG_SET, 0xFFFF);

      can_driver_powered = 1U;
      can_driver_initialized = 0U;
      CAN_SignalUnitEvent    = NULL;
      CAN_SignalObjectEvent  = NULL;
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
  
  //for testing switch to initialization mode
  CAN0->CTL.CTL |= 1 << CAN_CTL_INIT_Pos || 1 << CAN_CTL_CCE_Pos;

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

static int32_t ARM_CAN_SetMode (ARM_CAN_MODE mode) {

  if (can_driver_powered == 0U) { return ARM_DRIVER_ERROR; }

  switch (mode) {
    case ARM_CAN_MODE_INITIALIZATION:
      // Add code to put peripheral into initialization mode
      // ..
      break;
    case ARM_CAN_MODE_NORMAL:
      // Add code to put peripheral into normal operation mode
      // ..
      break;
    case ARM_CAN_MODE_RESTRICTED:
      // Add code to put peripheral into restricted operation mode
      // ..
      break;
    case ARM_CAN_MODE_MONITOR:
      // Add code to put peripheral into bus monitoring mode
      // ..
      break;
    case ARM_CAN_MODE_LOOPBACK_INTERNAL:
      // Add code to put peripheral into internal loopback mode
      // ..
      break;
    case ARM_CAN_MODE_LOOPBACK_EXTERNAL:
      // Add code to put peripheral into external loopback mode
      // ..
      break;
  }

  return ARM_DRIVER_OK;
}

static ARM_CAN_OBJ_CAPABILITIES ARM_CAN_ObjectGetCapabilities (uint32_t obj_idx) {
  // Return object capabilities
  return can_object_capabilities;
}

static int32_t ARM_CAN_ObjectSetFilter (uint32_t obj_idx, ARM_CAN_FILTER_OPERATION operation, uint32_t id, uint32_t arg) {

  if (can_driver_powered == 0U) { return ARM_DRIVER_ERROR; }

  switch (operation) {
    case ARM_CAN_FILTER_ID_EXACT_ADD:
      // Add code to setup peripheral to receive messages with specified exact ID
      break;
    case ARM_CAN_FILTER_ID_MASKABLE_ADD:
      // Add code to setup peripheral to receive messages with specified maskable ID
      break;
    case ARM_CAN_FILTER_ID_RANGE_ADD:
      // Add code to setup peripheral to receive messages within specified range of IDs
      break;
    case ARM_CAN_FILTER_ID_EXACT_REMOVE:
      // Add code to remove specified exact ID from being received by peripheral
      break;
    case ARM_CAN_FILTER_ID_MASKABLE_REMOVE:
      // Add code to remove specified maskable ID from being received by peripheral
      break;
    case ARM_CAN_FILTER_ID_RANGE_REMOVE:
      // Add code to remove specified range of IDs from being received by peripheral
      break;
  }

  return ARM_DRIVER_OK;
}

static int32_t ARM_CAN_ObjectConfigure (uint32_t obj_idx, ARM_CAN_OBJ_CONFIG obj_cfg) {

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

static int32_t ARM_CAN_MessageSend (uint32_t obj_idx, ARM_CAN_MSG_INFO *msg_info, const uint8_t *data, uint8_t size) {

  if (can_driver_powered == 0U) { return ARM_DRIVER_ERROR; }

  // Add code to send requested message
  // ..

  return ((int32_t)size);
}

static int32_t ARM_CAN_MessageRead (uint32_t obj_idx, ARM_CAN_MSG_INFO *msg_info, uint8_t *data, uint8_t size) {

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
  ARM_CAN_SetMode,
  ARM_CAN_ObjectGetCapabilities,
  ARM_CAN_ObjectSetFilter,
  ARM_CAN_ObjectConfigure,
  ARM_CAN_MessageSend,
  ARM_CAN_MessageRead,
  ARM_CAN_Control,
  ARM_CAN_GetStatus
};

