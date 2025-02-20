// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file pfc_pi.h
 *
 * @brief This module includes variables and function declaration required 
 * for implementation of PI controller
 * 
 */
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Disclaimer ">
/*******************************************************************************
* Copyright (c) 2017 released Microchip Technology Inc.  All rights reserved.
*
* SOFTWARE LICENSE AGREEMENT:
* 
* Microchip Technology Incorporated ("Microchip") retains all ownership and
* intellectual property rights in the code accompanying this message and in all
* derivatives hereto.  You may use this code, and any derivatives created by
* any person or entity by or on your behalf, exclusively with Microchip's
* proprietary products.  Your acceptance and/or use of this code constitutes
* agreement to the terms and conditions of this notice.
*
* CODE ACCOMPANYING THIS MESSAGE IS SUPPLIED BY MICROCHIP "AS IS".  NO
* WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
* TO, IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY AND FITNESS FOR A
* PARTICULAR PURPOSE APPLY TO THIS CODE, ITS INTERACTION WITH MICROCHIP'S
* PRODUCTS, COMBINATION WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.
*
* YOU ACKNOWLEDGE AND AGREE THAT, IN NO EVENT, SHALL MICROCHIP BE LIABLE,
* WHETHER IN CONTRACT, WARRANTY, TORT (INCLUDING NEGLIGENCE OR BREACH OF
* STATUTORY DUTY),STRICT LIABILITY, INDEMNITY, CONTRIBUTION, OR OTHERWISE,
* FOR ANY INDIRECT, SPECIAL,PUNITIVE, EXEMPLARY, INCIDENTAL OR CONSEQUENTIAL
* LOSS, DAMAGE, FOR COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE CODE,
* HOWSOEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE POSSIBILITY OR
* THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT ALLOWABLE BY LAW,
* MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY RELATED TO THIS CODE,
* SHALL NOT EXCEED THE PRICE YOU PAID DIRECTLY TO MICROCHIP SPECIFICALLY TO
* HAVE THIS CODE DEVELOPED.
*
* You agree that you are solely responsible for testing the code and
* determining its suitability.  Microchip has no obligation to modify, test,
* certify, or support the code.
*
*******************************************************************************/
// </editor-fold>

#ifndef __PFC_PI_H
#define __PFC_PI_H

// <editor-fold defaultstate="collapsed" desc="HEADER FILES ">
    
#ifdef __XC16__  // See comments at the top of this header file
    #include <xc.h>
#endif // __XC16__


// </editor-fold> 

#ifdef __cplusplus  // Provide C++ Compatability
    extern "C" {
#endif
        
// <editor-fold defaultstate="collapsed" desc="CONSTANTS/DEFINTIONS ">
                       
// </editor-fold>         
                
// <editor-fold defaultstate="collapsed" desc="VARIABLE TYPES">
/**
 * State variables related to PI controller 
 */    
typedef struct
{
    int16_t output;         /* PI controller output */        
    int16_t integralOut;    /* Integral output of PI controller*/ 
    int16_t propOut;        /* Proportional output of PI controller*/
    
    int16_t input;          /* Measured signal to the PI controller */
    int16_t reference;      /* Reference signal to the PI controller */
    int16_t error;          /* Error signal to the PI controller 
                             * (Reference - measured)*/

    int16_t kp;             /* Proportional gain coefficient term */
    int16_t ki;             /* Integral gain coefficient term */
    int16_t kpScale;        /* Normalizing term for proportional coefficient */
    int16_t kiScale;        /* Normalizing term for integral coefficient */
    int16_t minOutput;      /* Minimum output limit */      
    int16_t maxOutput;      /* Maximum output limit */
}PFC_PI_T;  

// </editor-fold> 

// <editor-fold defaultstate="collapsed" desc="INTERFACE FUNCTIONS ">

extern void PFC_PIController(PFC_PI_T *,int16_t);   
    
// </editor-fold> 
        
#ifdef __cplusplus  // Provide C++ Compatibility
    }
#endif
#endif      // end of __PFC_PI_H
    