/*
    ChibiOS/RT - Copyright (C) 2006,2007,2008,2009,2010,
                 2011,2012 Giovanni Di Sirio.

    This file is part of ChibiOS/RT.

    ChibiOS/RT is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.

    ChibiOS/RT is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/**
 * @file    STM32/can_lld.c
 * @brief   STM32 CAN subsystem low level driver source.
 *
 * @addtogroup CAN
 * @{
 */

#include "ch.h"
#include "hal.h"

#if HAL_USE_CAN || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/** @brief ADC1 driver identifier.*/
#if STM32_CAN_USE_CAN1 || defined(__DOXYGEN__)
CANDriver CAND1;
#endif

/*===========================================================================*/
/* Driver local variables.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/**
 * @brief   CAN1 TX interrupt handler.
 *
 * @isr
 */
CH_IRQ_HANDLER(CAN1_TX_IRQHandler) {
  uint32_t tsr;
  flagsmask_t flags;

  CH_IRQ_PROLOGUE();

  tsr = CAN1->TSR;
  /* No more events until a message is transmitted.*/
  CAN1->TSR |= CAN_TSR_RQCP0 | CAN_TSR_RQCP1 | CAN_TSR_RQCP2;

  /* The status of the transmit mailboxes is copied to the listener flags
     mask.*/
  flags = (flagsmask_t) tsr;
  _can_tx_isr_code(&CAND1, flags);

  CH_IRQ_EPILOGUE();
}

/*
 * @brief   CAN1 RX0 interrupt handler.
 *
 * @isr
 */CH_IRQ_HANDLER(CAN1_RX0_IRQHandler) {
	uint32_t rf0r;

  CH_IRQ_PROLOGUE();

  rf0r = CAN1->RF0R;
  if ((rf0r & CAN_RF0R_FMP0) > 0) {
    /* No more receive events until the queue 0 has been emptied.*/
    CAN1->IER &= ~CAN_IER_FMPIE0;

    /* Portable CAN ISR code defined in the high level driver, note, it is
     a macro.*/
    _can_rx_isr_code(&CAND1);
  }
  if ((rf0r & CAN_RF0R_FOVR0) > 0) {
    /* Overflow events handling.*/
    CAN1->RF0R = CAN_RF0R_FOVR0;
    /* Portable CAN ISR code defined in the high level driver, note, it is
     a macro.*/
    _can_error_isr_code(&CAND1, (flagsmask_t) CAN_OVERFLOW_ERROR);
  }

  CH_IRQ_EPILOGUE();
}

/**
 * @brief   CAN1 RX1 interrupt handler.
 *
 * @isr
 */CH_IRQ_HANDLER(CAN1_RX1_IRQHandler) {

  CH_IRQ_PROLOGUE();

  chSysHalt(); /* Not supported (yet).*/

  CH_IRQ_EPILOGUE();
}

/**
 * @brief   CAN1 SCE interrupt handler.
 *
 * @isr
 */CH_IRQ_HANDLER(CAN1_SCE_IRQHandler) {
	uint32_t msr;

  CH_IRQ_PROLOGUE();

  msr = CAN1->MSR;
  CAN1->MSR = CAN_MSR_ERRI | CAN_MSR_WKUI | CAN_MSR_SLAKI;
#if CAN_USE_SLEEP_MODE
  /* Wakeup event.*/
  if (msr & CAN_MSR_WKUI) {
    _can_wakeup_isr_code(&CAND1);
  }
#endif /* CAN_USE_SLEEP_MODE */
  /* Error event.*/
  if (msr & CAN_MSR_ERRI) {
    flagsmask_t flags;
    uint32_t esr = CAN1->ESR;

    CAN1->ESR &= ~CAN_ESR_LEC;
    flags = (flagsmask_t)(esr & 7);
    if ((esr & CAN_ESR_LEC) > 0)
      flags |= CAN_FRAMING_ERROR;
    /* The content of the ESR register is copied unchanged in the upper
       half word of the listener flags mask.*/
		_can_error_isr_code(&CAND1, flags | (flagsmask_t)(esr < 16));
	}

  CH_IRQ_EPILOGUE();
}

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level CAN driver initialization.
 *
 * @notapi
 */
void can_lld_init(void) {

#if STM32_CAN_USE_CAN1
  /* Driver initialization.*/
  canObjectInit(&CAND1);
  CAND1.can = CAN1;
#endif
}

/**
 * @brief   Configures and activates the CAN peripheral.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 *
 * @notapi
 */
void can_lld_start(CANDriver *canp) {

  /* Clock activation.*/
#if STM32_CAN_USE_CAN1
  if (&CAND1 == canp) {
    nvicEnableVector(STM32_CAN1_TX_NUMBER,
                     CORTEX_PRIORITY_MASK(STM32_CAN_CAN1_IRQ_PRIORITY));
    nvicEnableVector(STM32_CAN1_RX0_NUMBER,
                     CORTEX_PRIORITY_MASK(STM32_CAN_CAN1_IRQ_PRIORITY));
    nvicEnableVector(STM32_CAN1_RX1_NUMBER,
                     CORTEX_PRIORITY_MASK(STM32_CAN_CAN1_IRQ_PRIORITY));
    nvicEnableVector(STM32_CAN1_SCE_NUMBER,
                     CORTEX_PRIORITY_MASK(STM32_CAN_CAN1_IRQ_PRIORITY));
    rccEnableCAN1(FALSE);
  }
#endif

	/* Entering initialization mode. */
	canp->state = CAN_STARTING;
	canp->can->MCR = CAN_MCR_INRQ;
	while ((canp->can->MSR & CAN_MSR_INAK) == 0)
		chThdSleepS(1);
	/* BTR initialization.*/
	canp->can->BTR = canp->config->btr;
	/* MCR initialization.*/
	canp->can->MCR = canp->config->mcr;
	/* Filters initialization.*/
	can_lld_clear_filters(canp);
	/* Interrupt sources initialization.*/
	canp->can->IER = CAN_IER_TMEIE | CAN_IER_FMPIE0 | CAN_IER_FMPIE1
			| CAN_IER_WKUIE | CAN_IER_ERRIE | CAN_IER_LECIE | CAN_IER_BOFIE
			| CAN_IER_EPVIE | CAN_IER_EWGIE | CAN_IER_FOVIE0 | CAN_IER_FOVIE1;
}

/**
 * @brief   Deactivates the CAN peripheral.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 *
 * @notapi
 */
void can_lld_stop(CANDriver *canp) {

  /* If in ready state then disables the CAN peripheral.*/
  if (canp->state == CAN_READY) {
#if STM32_CAN_USE_CAN1
    if (&CAND1 == canp) {
      CAN1->MCR = 0x00010002;                   /* Register reset value.    */
      CAN1->IER = 0x00000000;                   /* All sources disabled.    */
      nvicDisableVector(STM32_CAN1_TX_NUMBER);
      nvicDisableVector(STM32_CAN1_RX0_NUMBER);
      nvicDisableVector(STM32_CAN1_RX1_NUMBER);
      nvicDisableVector(STM32_CAN1_SCE_NUMBER);
      rccDisableCAN1(FALSE);
    }
#endif
  }
}

/**
 * @brief   Determines whether a frame can be transmitted.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 *
 * @return The queue space availability.
 * @retval FALSE        no space in the transmit queue.
 * @retval TRUE         transmit slot available.
 *
 * @notapi
 */
bool_t can_lld_can_transmit(CANDriver *canp) {

  return (canp->can->TSR & CAN_TSR_TME) != 0;
}

/**
 * @brief   Inserts a frame into the transmit queue.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 * @param[in] ctfp      pointer to the CAN frame to be transmitted
 *
 * @notapi
 */
void can_lld_transmit(CANDriver *canp, CANTxFrame *ctfp) {
  uint32_t tir;
  CAN_TxMailBox_TypeDef *tmbp;

  /* Id of a free transmission mailbox.*/
  ctfp->mbox = (canp->can->TSR & CAN_TSR_CODE) >> 24;

  /* Pointer to a free transmission mailbox.*/
  tmbp = &canp->can->sTxMailBox[ctfp->mbox];

  /* Preparing the message.*/
  if (ctfp->IDE)
    tir = ((uint32_t)ctfp->EID << 3) | ((uint32_t)ctfp->RTR << 1) |
          CAN_TI0R_IDE;
  else
    tir = ((uint32_t)ctfp->SID << 21) | ((uint32_t)ctfp->RTR << 1);
  tmbp->TDTR = ctfp->DLC;
  tmbp->TDLR = ctfp->data32[0];
  tmbp->TDHR = ctfp->data32[1];
  tmbp->TIR  = tir | CAN_TI0R_TXRQ;
}

/**
 * @brief   Requests a transmission abort.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 * @param[in] mbox      id of the transmission mailbox to request for abort
 *
 * @notapi
 */
void can_lld_abort_transmit(CANDriver *canp, uint32_t mbox) {

	canp->can->TSR |= (CAN_TSR_ABRQ0 << (8 * mbox));
}

/**
 * @brief   Determines whether a frame has been received.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 *
 * @return If a frame has been received.
 * @retval FALSE        no frame has been received.
 * @retval TRUE         a frame has been received.
 *
 * @notapi
 */
bool_t can_lld_can_receive(CANDriver *canp) {

  return (canp->can->RF0R & CAN_RF0R_FMP0) > 0;
}

/**
 * @brief   Receives a frame from the input queue.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 * @param[out] crfp     pointer to the buffer where the CAN frame is copied
 *
 * @notapi
 */
void can_lld_receive(CANDriver *canp, CANRxFrame *crfp) {
  uint32_t r;

  /* Fetches the message.*/
  r = canp->can->sFIFOMailBox[0].RIR;
  crfp->RTR = (r & CAN_RI0R_RTR) >> 1;
  crfp->IDE = (r & CAN_RI0R_IDE) >> 2;
  if (crfp->IDE)
    crfp->EID = r >> 3;
  else
    crfp->SID = r >> 21;
  r = canp->can->sFIFOMailBox[0].RDTR;
  crfp->DLC = r & CAN_RDT0R_DLC;
  crfp->FMI = (uint8_t)(r >> 8);
  crfp->TIME = (uint16_t)(r >> 16);
  crfp->data32[0] = canp->can->sFIFOMailBox[0].RDLR;
  crfp->data32[1] = canp->can->sFIFOMailBox[0].RDHR;

  /* Releases the mailbox.*/
  canp->can->RF0R = CAN_RF0R_RFOM0;

  /* If the queue is empty re-enables the interrupt in order to generate
     events again.*/
  if ((canp->can->RF0R & CAN_RF0R_FMP0) == 0)
    canp->can->IER |= CAN_IER_FMPIE0;
}

/**
 * @brief   Sets CAN filters.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 * @param[in] crfp      pointer to an array of @p CANFilter structures
 * @param[in] num       number of elements into the filters array
 *
 * @notapi
 */
void can_lld_set_filters(CANDriver *canp, const CANFilter *filters,
		uint32_t num) {
	uint32_t i, fmask;
	CAN_FilterRegister_TypeDef * cfp;

	/* Filters initialization.*/
	canp->can->FMR |= CAN_FMR_FINIT;

	canp->can->FA1R = 0;
	canp->can->FM1R = 0;
	canp->can->FS1R = 0;
	canp->can->FFA1R = 0;

	cfp = canp->can->sFilterRegister;
	fmask = 1;

	for (i = 0; i < STM32_CAN_MAX_FILTERS; i++) {
		if (i < num) {
			if (filters[i].mode)
				canp->can->FM1R |= fmask;
			if (filters[i].scale)
				canp->can->FS1R |= fmask;
			if (filters[i].assignment)
				canp->can->FFA1R |= fmask;
			cfp->FR1 = filters[i].register1;
			cfp->FR2 = filters[i].register2;
			canp->can->FA1R |= fmask;
		} else {
			cfp->FR1 = 0;
			cfp->FR2 = 0;
		}

		cfp++;
		fmask <<= 1;
	}
}

/**
 * @brief   Clears all filters and sets a default pass-through filter.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 *
 * @notapi
 */
void can_lld_clear_filters(CANDriver *canp) {
	uint32_t i;

	/* Filters initialization.*/
	canp->can->FMR |= CAN_FMR_FINIT;

	/* Clear all filter registers.*/
	for (i = 0; i < STM32_CAN_MAX_FILTERS; i++) {
		canp->can->sFilterRegister[i].FR1 = 0xFFFFFFFF;
		canp->can->sFilterRegister[i].FR2 = 0xFFFFFFFF;
	}

	/* Set a default pass-through filter.*/
	canp->can->sFilterRegister[0].FR1 = 0;
	canp->can->sFilterRegister[0].FR2 = 0;

	canp->can->FM1R = 0;
	canp->can->FFA1R = 0;
	canp->can->FS1R = 0;
	canp->can->FA1R = 1;

	canp->can->FMR &= ~CAN_FMR_FINIT;
}

#if CAN_USE_SLEEP_MODE || defined(__DOXYGEN__)
/**
 * @brief   Enters the sleep mode.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 *
 * @notapi
 */
void can_lld_sleep(CANDriver *canp) {

  canp->can->MCR |= CAN_MCR_SLEEP;
}

/**
 * @brief   Enforces leaving the sleep mode.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 *
 * @notapi
 */
void can_lld_wakeup(CANDriver *canp) {

  canp->can->MCR &= ~CAN_MCR_SLEEP;
}
#endif /* CAN_USE_SLEEP_MODE */

#endif /* HAL_USE_CAN */

/** @} */
