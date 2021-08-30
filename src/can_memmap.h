/**
 * @file can_memmap.h
 * @author Mohamed Hassanin Mohamed
 * @brief A memory map for the two CAN controllers in TIVA C
 * @version 0.1
 * @date 2021-08-30
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef CAN_MEMMAP_h
#define CAN_MEMMAP_h

#include <inttypes.h>

#define CAN0_BASE 0x40040000
#define CAN1_BASE 0x40041000

#define CAN0 ((volatile CanHandle_t*) CAN0_BASE)
#define CAN1 ((volatile CanHandle_t*) CAN1_BASE)

#define SYSCTL_BASE 0x400FE000
#define SRCAN_OFFSET 0x534
#define PRCAN_OFFSET 0xA34
#define RCGCCAN_OFFSET 0x634

#define SRCAN *((volatile uint32_t*)(SYSCTL_BASE+SRCAN_OFFSET))
#define PRCAN *((volatile uint32_t*)(SYSCTL_BASE+PRCAN_OFFSET))
#define RCGCCAN *((volatile uint32_t*)(SYSCTL_BASE+RCGCCAN_OFFSET))

#define PRCAN_CAN0 0
#define SRCAN_CAN0 0
#define RCGCCAN_CAN0 0
typedef struct {
  uint32_t CRQ;
  uint32_t CMSK;
  uint32_t MSK1;
  uint32_t MSK2;
  uint32_t ARB1;
  uint32_t ARB2;
  uint32_t MCTL;
  uint32_t DA1;
  uint32_t DA2;
  uint32_t DB1;
  uint32_t DB2;
} CanIf_t;

typedef struct {
  uint32_t CTL;
  uint32_t STS;
  uint32_t ERR;
  uint32_t BIT;
  uint32_t INT;
  uint32_t TST;
  uint32_t BRPE;
} CanCtl_t;

typedef struct {
  uint32_t XRQ1;
  uint32_t XRQ2;
  uint32_t reserved0[0x1B];
  uint32_t NWDA1;
  uint32_t NWDA2;
  uint32_t reserved1[0x1B];
  uint32_t MSG1INT;
  uint32_t MSG2INT;
  uint32_t reserved2[0x1B];
  uint32_t MSG1VAL;
  uint32_t MSG2VAL;
} CanMsgObj_t;

typedef struct {
  CanCtl_t CTL;
  CanIf_t IF1;
  uint32_t reserved0[0x37];
  CanIf_t IF2;
  uint32_t reserved1[0x57];
  CanMsgObj_t MSGOBJ;
} CanHandle_t;

#endif
