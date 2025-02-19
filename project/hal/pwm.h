// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file pwm.h
 *
 * @brief This header file lists the functions and definitions to configure the 
 * PWM Module
 * 
 * Definitions in the file are for dsPIC33CK64MC105 MC DIM plugged onto 
 * Motor Control Development board from Microchip
 *
 * Component: PWM
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

#ifndef _PWM_H
#define _PWM_H

#ifdef __cplusplus  // Provide C++ Compatability
    extern "C" {
#endif

// <editor-fold defaultstate="collapsed" desc="HEADER FILES ">
#include <xc.h>
#include <stdint.h>
        
#include "clock.h"
        
// </editor-fold> 
        
// <editor-fold defaultstate="collapsed" desc="DEFINITIONS/MACROS ">
        
#undef ENABLE_PWM_FAULT


// MC PWM MODULE Related Definitions        
#define MC1_PWM_PDC1      PG1DC
#define MC1_PWM_PDC2      PG2DC
#define MC1_PWM_PDC3      PG3DC

   
// PFC PWM MODULE Related Definitions 
#define PFC_PWM_PDC                 PG4DC 
        
//  PFC Period ,Duty related definitions 
        
/* Specify PFC PWM Frequency in Hertz */
#define PFC_PWMFREQUENCY_HZ         64000        
/* Specify PFC PWM Period in seconds, (1/ PWMFREQUENCY_HZ) */
#define PFC_LOOPTIME_SEC            0.000015625
/* Specify PWM Period in micro seconds */
#define PFC_LOOPTIME_MICROSEC       15.625          
/* Specify maximum duty cycle,Make Sure Maximum duty ratio is less than or equal
 * to 0.9 */
#define PFC_MAX_DUTY_PU             0.90    
/* Specify minimum duty cycle */
#define PFC_MIN_DUTY_PU             0   
        
/* Loop time in terms of PWM clock period */
#define PFC_LOOPTIME_TCY            (uint16_t)(FOSC/(PFC_PWMFREQUENCY_HZ*2)-1)
/* Specify ADC Triggering Point w.r.t PWM Output for sensing PFC parameters */
#define PFC_ADC_SAMPLING_POINT      (uint16_t)(PFC_LOOPTIME_TCY-50)

/* Minimum duty cycle in terms of PWM period */       
#define PFC_MIN_DUTY                (uint16_t)(PFC_MIN_DUTY_PU*PFC_LOOPTIME_TCY)        
/* Maximum duty cycle in terms of PWM period */
#define PFC_MAX_DUTY                (uint16_t)(PFC_MAX_DUTY_PU*PFC_LOOPTIME_TCY)
        
#define SYNC_COUNT                  (uint16_t)(0.2*PFC_LOOPTIME_TCY)         
 /*Mention value of PWM Counter value at which MCPWM will be synchronized with PFC PWM*/       
#define PFC_Sync_MC_Value           32768+ SYNC_COUNT 
        
/* Specify PWM Frequency in Hertz */
#define PWMFREQUENCY_HZ         16000
/* Specify dead time in micro seconds */
#define DEADTIME_MICROSEC       2.0
/* Specify PWM Period in seconds, (1/ PWMFREQUENCY_HZ) */
#define LOOPTIME_SEC            0.0000625
/* Specify PWM Period in micro seconds */
#define LOOPTIME_MICROSEC       62.5
        
// Specify bootstrap charging time in Seconds (mention at least 10mSecs)
#define BOOTSTRAP_CHARGING_TIME_SECS 0.01
  
// Calculate Bootstrap charging time in number of PWM Half Cycles
#define BOOTSTRAP_CHARGING_COUNTS (uint16_t)((BOOTSTRAP_CHARGING_TIME_SECS/LOOPTIME_SEC )* 2)
        

#define PWM_FAULT_STATUS        PG1STATbits.FLTACT
        
#define _PWMInterrupt           _PWM1Interrupt
#define ClearPWMIF()            _PWM1IF = 0   
        
#define DDEADTIME               (uint16_t)(DEADTIME_MICROSEC*FOSC_MHZ)
// loop time in terms of PWM clock period
#define LOOPTIME_TCY            (uint16_t)(((PFC_LOOPTIME_TCY+1)*4)-1)

/* Specify ADC Triggering Point w.r.t PWM Output for sensing Motor Currents */
#define ADC_SAMPLING_POINT      0x0000
        
#define MIN_DUTY            (uint16_t)(DDEADTIME + DDEADTIME/2)
#define MAX_DUTY            LOOPTIME_TCY - (uint16_t)(DDEADTIME + DDEADTIME/2)

        
/* Specify Overrides for PWM TOP and BOTTOM ON */
#define PWM_TOP_ON  0x3800
#define PWM_BOT_ON  0x3400
#define PWM_ALL_OFF 0x3000    

// </editor-fold> 
    
// <editor-fold defaultstate="collapsed" desc="INTERFACE FUNCTIONS "> 

void InitPWMGenerators(void);        
        
// </editor-fold>

#ifdef __cplusplus  // Provide C++ Compatibility
    }
#endif
#endif      // end of PWM_H
