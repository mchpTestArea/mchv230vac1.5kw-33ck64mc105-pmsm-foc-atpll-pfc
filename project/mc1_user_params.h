// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file mc1_user_params.h
 *
 * @brief This file has definitions to be configured by the user for spinning
 * 		  motor 1 using field oriented control.
 *
 * Component: APPLICATION (motor control 1 - mc1)
 * Motor : ----------Induction Motor
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

#ifndef __MC1_USER_PARAMS_H
#define __MC1_USER_PARAMS_H

#ifdef __cplusplus
extern "C" {
#endif

// <editor-fold defaultstate="collapsed" desc="HEADER FILES ">
#include <stdint.h>
#include "general.h"
// </editor-fold>

// <editor-fold defaultstate="expanded" desc="DEFINITIONS/MACROS ">

/** Define macros for operational Modes */
/* Define OPEN_LOOP_FUNCTIONING for Open loop continuous functioning, 
 * undefine OPEN_LOOP_FUNCTIONING for closed loop functioning  */
#undef OPEN_LOOP_FUNCTIONING 

    
/** Board Parameters */
#define     MC1_PEAK_VOLTAGE            453.3  /* Peak measurement voltage of the board */
#define     MC1_PEAK_CURRENT            22     /* Peak measurement current of the board */
 
    
/* Enter the minimum DC link voltage(V) required to run the motor*/    
#define MC1_MOTOR_MIN_DC_VOLT     100

/** Motor Parameters */

/* Define Motor */    
#define LEADSHINE_EL5_M0400
    
/** The following values are given in the .xlsx file. */ 
    
#ifdef LEADSHINE_EL5_M0400
    #define POLEPAIRS           5  /* Motor's number of pole pairs */
    #define NOMINAL_SPEED_RPM   3000 /* Nominal speed of the motor in RPM */
    #define MAXIMUM_SPEED_RPM   3000 /* Maximum speed of the motor in RPM */
    #define MINIMUM_SPEED_RPM   700 /* Minimum speed of the motor in RPM*/
    /*Motor Rated Line - Line RMS Voltage*/
    #define NOMINAL_VOLTAGE_L_L  (float)220
    /* Motor Rated Phase Current RMS in Amps */
    #define NOMINAL_CURRENT_PHASE_RMS (float) 3

    /* Base values entered in .xlsx file*/
                                     /* Base Current = MC1_PEAK_CURRENT */
    #define MC1_BASE_VOLTAGE    311  /* Vdc Base voltage = Rated voltage*1.414*/
    #define MC1_PEAK_SPEED_RPM  2.5*NOMINAL_SPEED_RPM   /* Base Speed in RPM */

    /* Voltage utilization factor */ 
    /* VOLTAGE_UTIL_FACTOR = Peak_motor_voltage/Available_DC_Voltage */
    #define VOLTAGE_UTIL_FACTOR      (float)0.82

    /* Motor Rated Current Peak in Amps */
    #define NOMINAL_CURRENT_PEAK       (float) (NOMINAL_CURRENT_PHASE_RMS*1.414)

    /* The following values are given in the xls attached file */
    /* ATPLL Estimator parameters */
    #define	NORM_RS	3128
    #define	NORM_RS_QVALUE	15
    #define	NORM_L0DT	13904
    #define	NORM_L1DT	0
    #define	NORM_LDT_QVALUE	12
    #define	NORM_INVKFI_CONST	26751
    #define	NORM_INVKFI_CONST_QVALUE	14
    #define	NORM_DELTA_T	2560
    #define ATPLL_KI_GAIN_FACTOR  268
    #define ATPLL_KP_GAIN       15565 
    #define ATPLL_KP_GAIN_QVALUE   13

    /* PI controllers tuning values - */     
    /* D Control Loop Coefficients */
    #define	Q_CURRCNTR_PTERM        10970
    #define	Q_CURRCNTR_PTERM_SCALE  0
    #define	Q_CURRCNTR_ITERM        309
    #define	Q_CURRCNTR_ITERM_SCALE  0

    /* Q Control Loop Coefficients */
    #define D_CURRCNTR_PTERM        10970
    #define D_CURRCNTR_PTERM_SCALE  0
    #define D_CURRCNTR_ITERM        309
    #define D_CURRCNTR_ITERM_SCALE  0

/**********************  support xls file definitions end *********************/
    
    /* Velocity Control Loop Coefficients */    
    #define SPEEDCNTR_PTERM         Q15(0.401)
    #define	SPEEDCNTR_PTERM_SCALE   1
    #define SPEEDCNTR_ITERM         Q15(0.00021)
    #define SPEEDCNTR_ITERM_SCALE   0
    #define SPEEDCNTR_OUTMAX        NORM_VALUE(NOMINAL_CURRENT_PEAK,MC1_PEAK_CURRENT)
    
    /* Estimated speed filter constant */
    #define KFILTER_VELESTIM 500   

    /* Flux weakening parameters */
    /* Voltage reference factor during field weakening = FW_Voltage_Ref/Nominal_Max_utilizable_voltage
     * FW_VOLTAGE_REF_FACTOR can be increased above 0.9 if required when PFC is ENABLED */  
    #define FW_VOLTAGE_REF_FACTOR      (float)0.93 

    #define FD_WEAK_PI_KP               305
    #define FD_WEAK_PI_KPSCALE          1
    #define FD_WEAK_PI_KI               2
    #define ID_REF_MIN      NORM_VALUE((-NOMINAL_CURRENT_PEAK*0.8),MC1_PEAK_CURRENT)
    #undef ID_REFERNCE_FILTER_ENABLE
    #define FD_WEAK_IDREF_FILT_CONST    1000

#endif                  
   
/**************  support xls file definitions end **************/

/* Speed Reference Ramp parameters*/
#define     SPEED_RAMP_RATE_COUNT      1 /* Speed change rate in counts */
#define     RAMP_UP_TIME_MULTIPLIER    20 /* Sample time multiplier for up count */
#define     RAMP_DN_TIME_MULTIPLIER    20 /* Sample time multiplier for down count */
/* Speed rampe rate(rpm/sec) = 
 * (SPEED_CHANGE_RATE_COUNT/(LOOPTIME_SEC*TIME_MULTIPLIER)) *(MC1_PEAK_SPEED_RPM/32767) */

/* Open loop startup parameters */
/* Lock time for motor's poles alignment 
 * LOCK_TIME_COUNT = Lock_time_sec*PWF_frequency */
#define     LOCK_TIME_COUNT     9000
/* Locking Current in Amps */
#define     LOCK_CURRENT    (float)(1)
    
/* Open Loop Speed Reference Ramp */
#define     OL_SPEED_RAMP_RATE_COUNT    1
#define     OL_SPEED_RAMP_TIME_MULTIPLIER   10
/* Open loop q current reference */
#define     MIN_OPENLOOP_CURRENT    (float)(1.0)
#define     MAX_OPENLOOP_CURRENT    (float)(1.0)
#define     OL_CURRENT_RAMP_RATE_COUNT      1
 
/** Fault Parameters  */
/* Phase Over-current fault limit in Amps*/
#define     PEAK_FAULT_CURRENT_AMPS     (NOMINAL_CURRENT_PEAK*1.6)

/** Estimator-ATPLL Advanced Parameters */
#define     NORM_LDLQ_THRESHOLD     (0.25) /* Saliency of atleast 25% must */  
#define     OMEGA_PI_FILT_CONST 20000
#define     OMEGA_FILT_CONST    2200
#define     EMAG_FILT_CONST     500
#define     LQSIQS_FILT_CONST   3600    
    
/* End speed rpm for open loop to closed loop transition */
#define     END_SPEED_RPM       MINIMUM_SPEED_RPM

// </editor-fold>

#ifdef __cplusplus
}
#endif

#endif	/* end of __MC1_USER_PARAMS_H */
