// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file mc1_calc_params.h
 *
 * @brief This file has definitions used in the application to run motor 1,
 *        calculated based on associated user parameter header file
 *        mc1_user_params.h.
 *
 * Component: BOARD
 *
 */
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Disclaimer ">

/*******************************************************************************
* SOFTWARE LICENSE AGREEMENT
* 
* © [2024] Microchip Technology Inc. and its subsidiaries
* 
* Subject to your compliance with these terms, you may use this Microchip 
* software and any derivatives exclusively with Microchip products. 
* You are responsible for complying with third party license terms applicable to
* your use of third party software (including open source software) that may 
* accompany this Microchip software.
* 
* Redistribution of this Microchip software in source or binary form is allowed 
* and must include the above terms of use and the following disclaimer with the
* distribution and accompanying materials.
* 
* SOFTWARE IS "AS IS." NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY,
* APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT,
* MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT WILL 
* MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR 
* CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO
* THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE 
* POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE FULLEST EXTENT ALLOWED BY
* LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS RELATED TO THE SOFTWARE WILL
* NOT EXCEED AMOUNT OF FEES, IF ANY, YOU PAID DIRECTLY TO MICROCHIP FOR THIS
* SOFTWARE
*
* You agree that you are solely responsible for testing the code and
* determining its suitability.  Microchip has no obligation to modify, test,
* certify, or support the code.
*
*******************************************************************************/
// </editor-fold>

#ifndef __MC1_CALC_PARAMS_H
#define __MC1_CALC_PARAMS_H

#ifdef __cplusplus
extern "C" {
#endif

// <editor-fold defaultstate="collapsed" desc="HEADER FILES ">

#include <stdint.h>

#include "board_service.h"
#include "pwm.h"
#include "general.h"
#include "mc1_user_params.h"

// </editor-fold>

// <editor-fold defaultstate="expanded" desc="DEFINITIONS/MACROS ">
    
#define MC1_LOOPTIME_TCY        LOOPTIME_TCY 
    
#define MC1_LOOPTIME_SEC        LOOPTIME_SEC
      
#define MC1_NORM_DELTAT         MC1_PEAK_SPEED_RPM*POLEPAIRS*(1/30.0)*LOOPTIME_SEC*32768
  

/*Maximum utilizable Voltage Limit in closed loop control*/ /* 0.9* Vdclink/root3 */ 
#define VMAX_CLOSEDLOOP_CONTROL     NORM_VALUE(VOLTAGE_UTIL_FACTOR*MC1_BASE_VOLTAGE*0.577, MC1_BASE_VOLTAGE)    
/* D Control Loop Maximum limit */
#define Q_CURRCNTR_OUTMAX      VMAX_CLOSEDLOOP_CONTROL
/* Q Control Loop Maximum limit */
#define D_CURRCNTR_OUTMAX      VMAX_CLOSEDLOOP_CONTROL   

/* Maximum voltage square */
#define MAX_VOLTAGE_SQUARE   (int16_t)( (float)VMAX_CLOSEDLOOP_CONTROL*VMAX_CLOSEDLOOP_CONTROL/32767 )

/* Voltage factor for dynamic voltage limit calculation */    
#define VMAX_FACTOR     (int16_t)((float)(0.577)*VOLTAGE_UTIL_FACTOR*32767*MC1_PEAK_VOLTAGE/MC1_BASE_VOLTAGE)        

    
/* Flux weakening parameters */
#define FD_WEAK_VOLTAGE_REF  (int16_t)((float)VMAX_CLOSEDLOOP_CONTROL*FW_VOLTAGE_REF_FACTOR)
    
/* DC bus compensation factor */ 
#define DC_LINK_BASE_VOLTAGE    NORM_VALUE(MC1_BASE_VOLTAGE, MC1_PEAK_VOLTAGE)


/** Motor Parameters */  
/* normalized Ld/dt value */
/* Ld = L0 + L1 */
#define     NORM_LDDTBASE       (NORM_L0DT + NORM_L1DT)

/* normalized Lq/dt value */
/* Lq = L0 - L1 */
#define     NORM_LQDTBASE       (NORM_L0DT - NORM_L1DT)

#define     NORM_LDDT_THRESHOLD                                                \
(int16_t)((1-NORM_LDLQ_THRESHOLD) * NORM_LQDTBASE)

/** Fault Parameters  */
#define PEAK_FAULT_CURRENT    NORM_VALUE(PEAK_FAULT_CURRENT_AMPS,MC1_PEAK_CURRENT)   

// </editor-fold>

#ifdef __cplusplus
}
#endif

#endif	/* end of __MC1_CALC_PARAMS_H */
