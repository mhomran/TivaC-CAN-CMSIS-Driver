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

#define CANBIT_BRP_Pos 0
#define CANBIT_BRP_Msk (0x3F << CANBIT_BRP_Pos)
#define CANBIT_SJW_Pos 6
#define CANBIT_SJW_Msk (0x3 << CANBIT_SJW_Pos)
#define CANBIT_TSEG1_Pos 8
#define CANBIT_TSEG1_Msk (0xF << CANBIT_TSEG1_Pos)
#define CANBIT_TSEG2_Pos 12
#define CANBIT_TSEG2_Msk (0x7 << CANBIT_TSEG2_Pos)

#define CANCTL_INIT_Pos 0
#define CANCTL_INIT_Msk (1 << CANCTL_INIT_Pos)
#define CANCTL_IE_Pos 1
#define CANCTL_IE_Msk (1 << CANCTL_IE_Pos)
#define CANCTL_SIE_Pos 2
#define CANCTL_SIE_Msk (1 << CANCTL_SIE_Pos)
#define CANCTL_EIE_Pos 3
#define CANCTL_EIE_Msk (1 << CANCTL_EIE_Pos)
#define CANCTL_DAR_Pos 5
#define CANCTL_DAR_Msk (1 << CANCTL_DAR_Pos)
#define CANCTL_CCE_Pos 6
#define CANCTL_CCE_Msk (1 << CANCTL_CCE_Pos)
#define CANCTL_TEST_Pos 7
#define CANCTL_TEST_Msk (1 << CANCTL_TEST_Pos)

#define CANTST_LBACK_Pos 4
#define CANTST_LBACK_Msk (1 << CANTST_LBACK_Pos)

#define CANSTS_BOFF_Pos 7
#define CANSTS_BOFF_Msk (0x1 << CANSTS_BOFF_Pos)
#define CANSTS_EWARN_Pos 6
#define CANSTS_EWARN_Msk (0x1 << CANSTS_EWARN_Pos)
#define CANSTS_EPASS_Pos 5
#define CANSTS_EPASS_Msk (0x1 << CANSTS_EPASS_Pos)

#define CANERR_TEC_Pos 0
#define CANERR_TEC_Msk (0xFF << CANERR_TEC_Pos)
#define CANERR_REC_Pos 8
#define CANERR_REC_Msk (0x7F << CANERR_REC_Pos)
#define CANERR_RP_Pos 15
#define CANERR_RP_Msk (0x1 << CANERR_RP_Pos)

#define NVIC_BASE (0xE000E000)
#define NVICEN1_OFFSET 0x104
#define NVICEN1 *((volatile uint32_t*)(NVIC_BASE+NVICEN1_OFFSET)) 
#define NVICEN1_CAN0 6
#define NVICEN1_CAN1 7

#define CANIFCMSK_DATAB_Pos 0
#define CANIFCMSK_DATAB_Msk (1 << CANIFCMSK_DATAB_Pos)
#define CANIFCMSK_DATAA_Pos 1
#define CANIFCMSK_DATAA_Msk (1 << CANIFCMSK_DATAA_Pos)
#define CANIFCMSK_NEWDAT_TXRQST_Pos 2
#define CANIFCMSK_NEWDAT_Msk (1 << CANIFCMSK_NEWDAT_Pos)
#define CANIFCMSK_CLRINTPND_Pos 3
#define CANIFCMSK_CLRINTPND_Msk (1 << CANIFCMSK_CLRINTPND_Pos)
#define CANIFCMSK_CONTROL_Pos 4
#define CANIFCMSK_CONTROL_Msk (1 << CANIFCMSK_CONTROL_Pos)
#define CANIFCMSK_ARB_Pos 5
#define CANIFCMSK_ARB_Msk (1 << CANIFCMSK_ARB_Pos)
#define CANIFCMSK_MASK_Pos 6
#define CANIFCMSK_MASK_Msk (1 << CANIFCMSK_MASK_Pos)
#define CANIFCMSK_WRNRD_Pos 7
#define CANIFCMSK_WRNRD_Msk (1 << CANIFCMSK_WRNRD_Pos)

#define CANIFCRQ_MNUM_Pos 0
#define CANIFCRQ_MNUM_Msk (0x3F << CANIFCRQ_MNUM_Pos)
#define CANIFCRQ_BUSY_Pos 15

#define CANIFMSK1_XTD_Pos 0
#define CANIFMSK1_XTD_Msk (0xFFFF << CANIFMSK1_XTD_Pos)
#define CANIFMSK2_XTD_Pos 0
#define CANIFMSK2_STD_Pos 2
#define CANIFMSK2_ID_Pos 0
#define CANIFMSK2_ID_Msk (0x1FFF << CANIFMSK2_ID_Pos)
#define CANIFMSK2_MDIR_Pos 14
#define CANIFMSK2_MDIR_Msk (0x1 << CANIFMSK2_MDIR_Pos)
#define CANIFMSK2_MXTD_Pos 15
#define CANIFMSK2_MXTD_Msk (0x1 << CANIFMSK2_MXTD_Pos)

#define CANIFARB1_XTDID_Pos 0
#define CANIFARB1_XTDID_Msk (0xFFFF << CANIFARB1_XTDID_Pos)
#define CANIFARB2_XTDID_Pos 0
#define CANIFARB2_STDID_Pos 2
#define CANIFARB2_ID_Pos 0
#define CANIFARB2_ID_Msk (0x1FFF << CANIFARB2_ID_Pos)
#define CANIFARB2_DIR_Pos 13
#define CANIFARB2_DIR_Msk (0x1 << CANIFARB2_DIR_Pos)
#define CANIFARB2_XTD_Pos 14
#define CANIFARB2_XTD_Msk (0x1 << CANIFARB2_XTD_Pos)
#define CANIFARB2_MSGVAL_Pos 15
#define CANIFARB2_MSGVAL_Msk (0x1 << CANIFARB2_MSGVAL_Pos)

#define CANIFMCTL_DLC_Pos 0
#define CANIFMCTL_DLC_Msk (0xF << CANIFMCTL_DLC_Pos)
#define CANIFMCTL_EOB_Pos 7
#define CANIFMCTL_EOB_Msk (0x1 << CANIFMCTL_EOB_Pos)
#define CANIFMCTL_TXRQST_Pos 8
#define CANIFMCTL_TXRQST_Msk (0x1 << CANIFMCTL_TXRQST_Pos)
#define CANIFMCTL_RMTEN_Pos 9
#define CANIFMCTL_RMTEN_Msk (0x1 << CANIFMCTL_RMTEN_Pos)
#define CANIFMCTL_RXIE_Pos 10
#define CANIFMCTL_RXIE_Msk (0x1 << CANIFMCTL_RXIE_Pos)
#define CANIFMCTL_TXIE_Pos 11
#define CANIFMCTL_TXIE_Msk (0x1 << CANIFMCTL_TXIE_Pos)
#define CANIFMCTL_UMASK_Pos 12
#define CANIFMCTL_UMASK_Msk (0x1 << CANIFMCTL_UMASK_Pos)
#define CANIFMCTL_INTPND_Pos 13
#define CANIFMCTL_INTPND_Msk (0x1 << CANIFMCTL_INTPND_Pos)
#define CANIFMCTL_MSGLST_Pos 14
#define CANIFMCTL_MSGLST_Msk (0x1 << CANIFMCTL_MSGLST_Pos)
#define CANIFMCTL_NEWDAT_Pos 15
#define CANIFMCTL_NEWDAT_Msk (0x1 << CANIFMCTL_NEWDAT_Pos)

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
  uint32_t reserved0[0x6];
  uint32_t NWDA1;
  uint32_t NWDA2;
  uint32_t reserved1[0x6];
  uint32_t MSG1INT;
  uint32_t MSG2INT;
  uint32_t reserved2[0x6];
  uint32_t MSG1VAL;
  uint32_t MSG2VAL;
} CanMsgObj_t;

typedef struct {
  CanCtl_t CTL;
  uint32_t reserved0;
  CanIf_t IF1;
  uint32_t reserved1[0xD];
  CanIf_t IF2;
  uint32_t reserved2[0x15];
  CanMsgObj_t MSGOBJ;
} CanHandle_t;

#endif
