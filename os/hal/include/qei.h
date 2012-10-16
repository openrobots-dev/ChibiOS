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
 * @file    qei.h
 * @brief   QEI Driver macros and structures.
 *
 * @addtogroup QEI
 * @{
 */

#ifndef _QEI_H_
#define _QEI_H_

#if HAL_USE_QEI || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Driver state machine possible states.
 */
typedef enum {
  QEI_UNINIT = 0,                   /**< Not initialized.                   */
  QEI_STOP = 1,                     /**< Stopped.                           */
  QEI_READY = 2,                    /**< Ready.                             */
  QEI_ACTIVE = 3,                   /**< Active.                            */
} qeistate_t;

/**
 * @brief   Type of a structure representing an QEI driver.
 */
typedef struct QEIDriver QEIDriver;

/**
 * @brief   QEI notification callback type.
 *
 * @param[in] qeip      pointer to a @p QEIDriver object
 */
typedef void (*qeicallback_t)(QEIDriver *qeip);

#include "qei_lld.h"

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @name    Macro Functions
 * @{
 */
/**
 * @brief   Enables the input capture.
 *
 * @param[in] qeip      pointer to the @p QEIDriver object
 *
 * @iclass
 */
#define qeiEnableI(qeip) qei_lld_enable(qeip)

/**
 * @brief   Disables the input capture.
 *
 * @param[in] qeip      pointer to the @p QEIDriver object
 *
 * @iclass
 */
#define qeiDisableI(qeip) qei_lld_disable(qeip)

/**
 * @brief   Returns the counter value.
 *
 * @param[in] qeip      pointer to the @p QEIDriver object
 * @return              The current counter value.
 *
 * @iclass
 */
#define qeiGetCount(qeip) qei_lld_get_count(qeip)

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void qeiInit(void);
  void qeiObjectInit(QEIDriver *qeip);
  void qeiStart(QEIDriver *qeip, const QEIConfig *config);
  void qeiStop(QEIDriver *qeip);
  void qeiEnable(QEIDriver *qeip);
  void qeiDisable(QEIDriver *qeip);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_QEI */

#endif /* _QEI_H_ */

/** @} */
