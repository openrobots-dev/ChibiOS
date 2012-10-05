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
 * @file    can.c
 * @brief   CAN Driver code.
 *
 * @addtogroup CAN
 * @{
 */

#include "ch.h"
#include "hal.h"

#if HAL_USE_CAN || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   CAN Driver initialization.
 * @note    This function is implicitly invoked by @p halInit(), there is
 *          no need to explicitly initialize the driver.
 *
 * @init
 */
void canInit(void) {

  can_lld_init();
}

/**
 * @brief   Initializes the standard part of a @p CANDriver structure.
 *
 * @param[out] canp     pointer to the @p CANDriver object
 *
 * @init
 */
void canObjectInit(CANDriver *canp) {

  canp->state = CAN_STOP;
  canp->config = NULL;
#if CAN_USE_WAIT
  chSemInit(&canp->txsem, 0);
  chSemInit(&canp->rxsem, 0);
#endif /* CAN_USE_WAIT */
#if CAN_USE_EVENTS
  chEvtInit(&canp->rx_event);
  chEvtInit(&canp->tx_event);
  chEvtInit(&canp->error_event);
#endif /* CAN_USE_EVENTS */
#if (CAN_USE_SLEEP_MODE && CAN_USE_EVENTS)
  chEvtInit(&canp->sleep_event);
  chEvtInit(&canp->wakeup_event);
#endif /* CAN_USE_SLEEP_MODE */
}

/**
 * @brief   Configures and activates the CAN peripheral.
 * @note    Activating the CAN bus can be a slow operation this this function
 *          is not atomic, it waits internally for the initialization to
 *          complete.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 * @param[in] config    pointer to the @p CANConfig object. Depending on
 *                      the implementation the value can be @p NULL.
 *
 * @api
 */
void canStart(CANDriver *canp, const CANConfig *config) {

  chDbgCheck(canp != NULL, "canStart");

  chSysLock();
  chDbgAssert((canp->state == CAN_STOP) ||
              (canp->state == CAN_STARTING) ||
              (canp->state == CAN_READY),
              "canStart(), #1", "invalid state");
  while (canp->state == CAN_STARTING)
    chThdSleepS(1);
  if (canp->state == CAN_STOP) {
    canp->config = config;
    can_lld_start(canp);
    canp->state = CAN_READY;
  }
  chSysUnlock();
}

/**
 * @brief   Deactivates the CAN peripheral.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 *
 * @api
 */
void canStop(CANDriver *canp) {

  chDbgCheck(canp != NULL, "canStop");

  chSysLock();
  chDbgAssert((canp->state == CAN_STOP) || (canp->state == CAN_READY),
              "canStop(), #1", "invalid state");
  can_lld_stop(canp);
#if CAN_USE_WAIT
  chSemResetI(&canp->rxsem, 0);
  chSemResetI(&canp->txsem, 0);
  chSchRescheduleS();
#endif /* CAN_USE_WAIT */
  canp->state = CAN_STOP;
  chSysUnlock();
}

/**
 * @brief   CAN frame transmission.
 * @details This asynchronous function enqueues a frame for transmission.
 * @post    When the transmission is completed the configured callback is invoked.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 * @param[in] ctfp      pointer to the CAN frame to be transmitted
 *                      .
 * @return              The operation status.
 * @retval TRUE         The frame was successfully enqueued
 * @retval FALSE        The frame was not enqueued.
 *
 * @api
 */
bool_t canTryTransmit(CANDriver *canp, CANTxFrame *ctfp) {
  bool_t ret;

  chSysLock();
  ret = canTryTransmitI(canp, ctfp);
  chSysUnlock();

  return ret;
}

/**
 * @brief   CAN frame transmission.
 * @details This asynchronous function enqueues a frame for transmission.
 * @post    When the transmission is completed the configured callback is invoked.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 * @param[in] ctfp      pointer to the CAN frame to be transmitted
 *                      .
 * @return              The operation status.
 * @retval TRUE         The frame was successfully enqueued
 * @retval FALSE        The frame was not enqueued.
 *
 * @iclass
 */
bool_t canTryTransmitI(CANDriver *canp, CANTxFrame *ctfp) {

  chDbgCheckClassI();
  chDbgCheck((canp != NULL) && (ctfp != NULL), "canTransmitI");
  chDbgAssert((canp->state == CAN_READY) || (canp->state == CAN_SLEEP),
              "canTryTransmitI(), #1", "invalid state");

  if ((canp->state == CAN_SLEEP) || !can_lld_can_transmit(canp))
    return FALSE;

  can_lld_transmit(canp, ctfp);
  return TRUE;
}

/**
 * @brief   Aborts an enqueued transmission.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 * @param[in] mbx       transmit mailbox to abort
 *
 * @api
 */
void canAbortTransmission(CANDriver *canp, uint32_t mbox) {

  chSysLock();
  canAbortTransmissionI(canp, mbox);
  chSysUnlock();
}

/**
 * @brief   Aborts an enqueued transmission.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 * @param[in] mbx       transmit mailbox to abort
 *
 * @iclass
 */
void canAbortTransmissionI(CANDriver *canp, uint32_t mbox) {

  chDbgCheckClassI();
// FIXME
//  chDbgCheck((canp != NULL) && (mbox < CAN_TX_MBOX_COUNT),
//             "canAbortTransmissionI");
  chDbgCheck((canp != NULL), "canAbortTransmissionI");
  chDbgAssert((canp->state == CAN_READY) || (canp->state == CAN_SLEEP),
              "canAbortTransmissionI(), #1", "invalid state");

  can_lld_abort_transmit(canp, mbox);
}

/**
 * @brief   CAN frame receive.
 * @details This asynchronous function retrives a frame from the receive FIFO.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 * @param[in] crfp      pointer to the buffer where the CAN frame is copied
 *                      .
 * @return              The operation status.
 * @retval TRUE         The frame was successfully retrivied
 * @retval FALSE        No frame to retrive.
 *
 * @api
 */
bool_t canTryReceive(CANDriver *canp, CANRxFrame *crfp) {
  bool_t ret;

  chSysLock();
  ret = canTryReceiveI(canp, crfp);
  chSysUnlock();

  return ret;
}

/**
 * @brief   CAN frame receive.
 * @details This asynchronous function retrives a frame from the receive FIFO.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 * @param[in] crfp      pointer to the buffer where the CAN frame is copied
 *                      .
 * @return              The operation status.
 * @retval TRUE         The frame was successfully retrivied
 * @retval FALSE        No frame to retrive.
 *
 * @iclass
 */
bool_t canTryReceiveI(CANDriver *canp, CANRxFrame *crfp) {

  chDbgCheckClassI();
  chDbgCheck((canp != NULL) && (crfp != NULL), "canReceiveI");
  chDbgAssert((canp->state == CAN_READY) || (canp->state == CAN_SLEEP),
              "canTryReceiveI(), #1", "invalid state");

  if ((canp->state == CAN_SLEEP) || !can_lld_can_receive(canp))
    return FALSE;

  can_lld_receive(canp, crfp);
  return TRUE;
}

/**
 * @brief   Sets CAN filters.
 * @details Configures hardware filters of the CAN interface.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 * @param[in] crfp      pointer to an array of @p CANFilter structures
 * @param[in] num       number of elements into the filters array
 *
 * @api
 */
void canSetFilters(CANDriver *canp, CANFilter *filters, uint32_t num) {

  chSysLock();
  canSetFiltersI(canp, filters, num);
  chSysUnlock();
}

/**
 * @brief   Sets CAN filters.
 * @details Configures hardware filters of the CAN interface.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 * @param[in] crfp      pointer to an array of @p CANFilter structures
 * @param[in] num       number of elements into the filters array
 *
 * @iclass
 */
void canSetFiltersI(CANDriver *canp, CANFilter *filters, uint32_t num) {

  chDbgCheckClassI();
  chDbgCheck((canp != NULL), "canSetFilterI");
  chDbgAssert((canp->state == CAN_READY) || (canp->state == CAN_SLEEP),
              "canSetFilterI(), #1", "invalid state");

  can_lld_set_filters(canp, filters, num);
}

/**
 * @brief   Clears all CAN filters.
 * @details Configures a default pass-through hardware filter on the CAN
 *          interface.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 *
 * @api
 */
void canClearFilters(CANDriver *canp) {

  chSysLock();
  canClearFiltersI(canp);
  chSysUnlock();
}

/**
 * @brief   Clears all CAN filters.
 * @details Configures a default pass-through hardware filter on the CAN
 *          interface.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 *
 * @iclass
 */
void canClearFiltersI(CANDriver *canp) {

  chDbgCheckClassI();
  chDbgCheck((canp != NULL), "canClearFiltersI");
  chDbgAssert((canp->state == CAN_READY) || (canp->state == CAN_SLEEP),
              "canClearFiltersI(), #1", "invalid state");

  can_lld_clear_filters(canp);
}

#if CAN_USE_WAIT
/**
 * @brief   Can frame transmission.
 * @details The specified frame is queued for transmission, if the hardware
 *          queue is full then the invoking thread is queued.
 * @note    Trying to transmit while in sleep mode simply enqueues the thread.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 * @param[in] ctfp      pointer to the CAN frame to be transmitted
 * @param[in] timeout   the number of ticks before the operation timeouts,
 *                      the following special values are allowed:
 *                      - @a TIME_IMMEDIATE immediate timeout.
 *                      - @a TIME_INFINITE no timeout.
 *                      .
 * @return              The operation result.
 * @retval RDY_OK       the frame has been queued for transmission.
 * @retval RDY_TIMEOUT  The operation has timed out.
 * @retval RDY_RESET    The driver has been stopped while waiting.
 *
 * @api
 */
msg_t canTransmitTimeout(CANDriver *canp, CANTxFrame *ctfp, systime_t timeout) {

  chSysLock();
  chDbgAssert((canp->state == CAN_READY) || (canp->state == CAN_SLEEP),
              "canTransmit(), #1", "invalid state");
  while (!canTryTransmitI(canp, ctfp)) {
    msg_t msg = chSemWaitTimeoutS(&canp->txsem, timeout);
    if (msg != RDY_OK) {
      chSysUnlock();
      return msg;
    }
  }
  chSysUnlock();
  return RDY_OK;
}

/**
 * @brief   Can frame receive.
 * @details The function waits until a frame is received.
 * @note    Trying to receive while in sleep mode simply enqueues the thread.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 * @param[out] crfp     pointer to the buffer where the CAN frame is copied
 * @param[in] timeout   the number of ticks before the operation timeouts,
 *                      the following special values are allowed:
 *                      - @a TIME_IMMEDIATE immediate timeout (useful in an
 *                        event driven scenario where a thread never blocks
 *                        for I/O).
 *                      - @a TIME_INFINITE no timeout.
 *                      .
 * @return              The operation result.
 * @retval RDY_OK       a frame has been received and placed in the buffer.
 * @retval RDY_TIMEOUT  The operation has timed out.
 * @retval RDY_RESET    The driver has been stopped while waiting.
 *
 * @api
 */
msg_t canReceiveTimeout(CANDriver *canp, CANRxFrame *crfp, systime_t timeout) {

  chSysLock();
  chDbgAssert((canp->state == CAN_READY) || (canp->state == CAN_SLEEP),
              "canReceive(), #1", "invalid state");
  while (!canTryReceiveI(canp, crfp)) {
    msg_t msg = chSemWaitTimeoutS(&canp->rxsem, timeout);
    if (msg != RDY_OK) {
      chSysUnlock();
      return msg;
    }
  }
  chSysUnlock();
  return RDY_OK;
}
#endif /* CAN_USE_WAIT */

#if CAN_USE_SLEEP_MODE || defined(__DOXYGEN__)
/**
 * @brief   Enters the sleep mode.
 * @details This function puts the CAN driver in sleep mode and broadcasts
 *          the @p sleep_event event source.
 * @pre     In order to use this function the option @p CAN_USE_SLEEP_MODE must
 *          be enabled and the @p CAN_SUPPORTS_SLEEP mode must be supported
 *          by the low level driver.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 *
 * @api
 */
void canSleep(CANDriver *canp) {

  chDbgCheck(canp != NULL, "canSleep");

  chSysLock();
  chDbgAssert((canp->state == CAN_READY) || (canp->state == CAN_SLEEP),
              "canSleep(), #1", "invalid state");
  if (canp->state == CAN_READY) {
    can_lld_sleep(canp);
    canp->state = CAN_SLEEP;
#if CAN_USE_EVENTS
    chEvtBroadcastI(&canp->sleep_event);
#endif
#if CAN_USE_WAIT || CAN_USE_EVENTS
    chSchRescheduleS();
#endif
  }
  chSysUnlock();
}

/**
 * @brief   Enforces leaving the sleep mode.
 * @note    The sleep mode is supposed to be usually exited automatically by
 *          an hardware event.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 */
void canWakeup(CANDriver *canp) {

  chDbgCheck(canp != NULL, "canWakeup");

  chSysLock();
  chDbgAssert((canp->state == CAN_READY) || (canp->state == CAN_SLEEP),
              "canWakeup(), #1", "invalid state");
  if (canp->state == CAN_SLEEP) {
    can_lld_wakeup(canp);
    canp->state = CAN_READY;
#if CAN_USE_EVENTS
    chEvtBroadcastI(&canp->wakeup_event);
#endif
#if CAN_USE_WAIT || CAN_USE_EVENTS
    chSchRescheduleS();
#endif
  }
  chSysUnlock();
}
#endif /* CAN_USE_SLEEP_MODE */

#endif /* HAL_USE_CAN */

/** @} */
