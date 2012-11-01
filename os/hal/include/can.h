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
 * @file    can.h
 * @brief   CAN Driver macros and structures.
 *
 * @addtogroup CAN
 * @{
 */

#ifndef _CAN_H_
#define _CAN_H_

#if HAL_USE_CAN || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @name    CAN status flags
 * @{
 */
#define CAN_LIMIT_WARNING           1   /**< Errors rate warning.            */
#define CAN_LIMIT_ERROR             2   /**< Errors rate error.              */
#define CAN_BUS_OFF_ERROR           4   /**< Bus off condition reached.      */
#define CAN_FRAMING_ERROR           8   /**< Framing error on the CAN bus.   */
#define CAN_OVERFLOW_ERROR          16  /**< Overflow in receive queue.      */
/** @} */

/**
 * @name    CAN TX status flags
 * @{
 */
#define CAN_TX_OK                   1   /**< Transmission completed.         */
#define CAN_ARBITRATION_LOST        2   /**< Arbitration lost.               */
#define CAN_TX_ERROR                4   /**< Transmission error.             */

/** @} */
/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    CAN configuration options
 * @{
 */
/**
 * @brief   Enables synchronous APIs.
 * @note    Disabling this option saves both code and data space.
 */
#if !defined(CAN_USE_WAIT) || defined(__DOXYGEN__)
#define CAN_USE_WAIT                TRUE
#endif

/**
 * @brief   Enables event sources for transmission complete, receive and error events.
 * @note    Disabling this option saves both code and data space.
 */
#if !defined(CAN_USE_EVENTS) || defined(__DOXYGEN__)
#define CAN_USE_EVENTS              TRUE
#endif

/**
 * @brief   Sleep mode related APIs inclusion switch.
 * @details This option can only be enabled if the CAN implementation supports
 *          the sleep mode, see the macro @p CAN_SUPPORTS_SLEEP exported by
 *          the underlying implementation.
 */
#if !defined(CAN_USE_SLEEP_MODE) || defined(__DOXYGEN__)
#define CAN_USE_SLEEP_MODE          TRUE
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if !CH_USE_SEMAPHORES || !CH_USE_EVENTS
#error "CAN driver requires CH_USE_SEMAPHORES and CH_USE_EVENTS"
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Driver state machine possible states.
 */
typedef enum {
  CAN_UNINIT = 0,                           /**< Not initialized.           */
  CAN_STOP = 1,                             /**< Stopped.                   */
  CAN_STARTING = 2,                         /**< Starting.                  */
  CAN_READY = 3,                            /**< Ready.                     */
  CAN_SLEEP = 4,                            /**< Sleep state.               */
} canstate_t;

#include "can_lld.h"

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @name    Macro Functions
 * @{
 */
/**
 * @brief   FIXME.
 *
 * @param[in] canp      FIXME
 * @param[in] mask      FIXME
 *
 * @api
 */
#define canTransmit(canp, ctfp) canTransmitTimeout(canp, ctfp, TIME_INFINITE)

/**
 * @brief   FIXME.
 *
 * @param[in] canp      FIXME
 * @param[in] mask      FIXME
 *
 * @api
 */
#define canReceive(canp, crfp) canReceiveTimeout(canp, crfp, TIME_INFINITE)
/** @} */

/**
 * @name    Low Level driver helper macros
 * @{
 */
#if CAN_USE_WAIT || defined(__DOXYGEN__)
/**
 * @brief   Signals the waiting threads.
 *
 * @param[in] semp      pointer to the semaphore
 *
 * @notapi
 */
#define _can_sem_signal_i(semp) {                                           \
  while (chSemGetCounterI(semp) < 0)                                        \
    chSemSignalI(semp);                                                      \
}
#else /* !CAN_USE_WAIT */
#define _can_sem_signal_i(semp)
#endif /* !CAN_USE_WAIT */

#if CAN_USE_EVENTS
#define _can_evt_broadcast_i(evt) chEvtBroadcastI(evt)
#define _can_evt_broadcast_flags_i(evt, flags) chEvtBroadcastFlagsI(evt, flags)
#else
#define _can_evt_broadcast_i(evt)
#define _can_evt_broadcast_flags_i(evt, flags)
#endif /* !CAN_USE_EVENTS */

/**
 * @brief   Common transmission complete ISR code. 
 * @details This code handles the portable part of the ISR code:
 *          - Callback invocation.
 *          - Waiting thread wakeup, if any.
 *          - Event broadcast.
 *          .
 * @note    This macro is meant to be used in the low level drivers
 *          implementation only.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 *
 * @notapi
 */
#define _can_tx_isr_code(canp, flags) {                                    \
  if ((canp)->config->tx_cb) {                                              \
    (canp)->config->tx_cb(canp, flags);                                     \
  }                                                                         \
  chSysLockFromIsr();                                                       \
  _can_sem_signal_i(&((canp)->txsem));                                      \
  _can_evt_broadcast_flags_i(&((canp)->tx_event), flags);                   \
  chSysUnlockFromIsr();                                                     \
}

/**
 * @brief   Common receive ISR code. 
 * @details This code handles the portable part of the ISR code:
 *          - Callback invocation.
 *          - Waiting thread wakeup, if any.
 *          - Event broadcast.
 *          .
 * @note    This macro is meant to be used in the low level drivers
 *          implementation only.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 *
 * @notapi
 */
#define _can_rx_isr_code(canp) {                                           \
  if ((canp)->config->rx_cb) {                                              \
    (canp)->config->rx_cb(canp);                                            \
  }                                                                         \
  chSysLockFromIsr();                                                       \
  _can_sem_signal_i(&((canp)->rxsem));                                      \
  _can_evt_broadcast_i(&((canp)->rx_event));                                \
  chSysUnlockFromIsr();                                                     \
}

/**
 * @brief   Common error ISR code. 
 * @details This code handles the portable part of the ISR code:
 *          - Callback invocation.
 *          - Waiting thread wakeup, if any.
 *          - Event broadcast.
 *          .
 * @note    This macro is meant to be used in the low level drivers
 *          implementation only.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 *
 * @notapi
 */
#define _can_error_isr_code(canp, flags) {                                 \
  if ((canp)->config->error_cb) {                                           \
    (canp)->config->error_cb(canp, flags);                                  \
  }                                                                         \
  chSysLockFromIsr();                                                       \
  _can_evt_broadcast_flags_i(&((canp)->error_event), flags);                \
  chSysUnlockFromIsr();                                                     \
}

/**
 * @brief   Common wakeup ISR code. 
 * @details This code handles the portable part of the ISR code:
 *          - Waiting thread wakeup, if any.
 *          - Event broadcast.
 *          .
 * @note    This macro is meant to be used in the low level drivers
 *          implementation only.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 *
 * @notapi
 */

#define _can_wakeup_isr_code(canp) {                                        \
  chSysLockFromIsr();                                                       \
  _can_evt_broadcast_i(&((canp)->wakeup_event));                            \
  chSysUnlockFromIsr();                                                     \
}

/** @} */

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void canInit(void);
  void canObjectInit(CANDriver *canp);
  void canStart(CANDriver *canp, const CANConfig *config);
  void canStop(CANDriver *canp);
  bool_t canTryTransmit(CANDriver *canp, CANTxFrame *ctfp);
  bool_t canTryTransmitI(CANDriver *canp, CANTxFrame *ctfp);
  void canAbortTransmission(CANDriver *canp, uint32_t mbox);
  void canAbortTransmissionI(CANDriver *canp, uint32_t mbox);
  bool_t canTryReceive(CANDriver *canp, CANRxFrame *crfp);
  bool_t canTryReceiveI(CANDriver *canp, CANRxFrame *crfp);
  void canSetFilters(CANDriver *canp, CANFilter *filters, uint32_t num);
  void canSetFiltersI(CANDriver *canp, CANFilter *filters, uint32_t num);
  void canClearFilters(CANDriver *canp);
  void canClearFiltersI(CANDriver *canp);
#if CAN_USE_WAIT
  msg_t canTransmitTimeout(CANDriver *canp, CANTxFrame *ctfp, systime_t timeout);
  msg_t canReceiveTimeout(CANDriver *canp, CANRxFrame *crfp, systime_t timeout);
#endif /* CAN_USE_WAIT */
#if CAN_USE_SLEEP_MODE
  void canSleep(CANDriver *canp);
  void canWakeup(CANDriver *canp);
#endif /* CAN_USE_SLEEP_MODE */
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_CAN */

#endif /* _CAN_H_ */

/** @} */
