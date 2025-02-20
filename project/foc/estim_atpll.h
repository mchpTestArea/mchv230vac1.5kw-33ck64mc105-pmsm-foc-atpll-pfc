// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file estim_atpll.h
 *
 * @brief This module implements ATPLL(Angle Tracking PLL ) Estimator.
 *
 * Component: ESTIMATOR
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

#ifndef __ESTIM_ATPLL_H
#define __ESTIM_ATPLL_H

#ifdef __cplusplus
    extern "C" {
#endif

// <editor-fold defaultstate="collapsed" desc="HEADER FILES ">

#include <stdint.h>
#include <stdbool.h>

#include "motor_control.h"
#include "measure.h"
#include "foc_control_types.h"
#include "motor_params.h"

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="VARIABLE TYPE DEFINITIONS">

typedef struct
{
    MC_ALPHABETA_T
        qEAlphaBeta,        /* Estimated Back EMF */
        qVIndAlphaBeta,     /* Inductance Drop */
        qVResAlphaBeta,     /* Resistance Drop */
        qLastVAlphaBeta;    /* Last Applied Voltage */

    MC_ALPHABETA_T
        qLastIAlphaBeta,
        qDIAlphaBeta;
           
    MC_DQ_T
        qEdq;               /* Estimated Back EMF */

    MC_SINCOS_T  
        qSinCos;  /* Sine-cosine for estimator */ 
        
    int16_t
        qL1Dt,
        qL0Dt,
        LsDtBaseScale,
        qInvKFIBase,
        invKFIBaseScale,
        feedForwardSpeed,
        feedForwardMode;
        
    int16_t
        qEdNegetive,
        qEerr,              /* Back EMF Error which is forced to zero*/
        qKpGain,            /* Kp Gain */
        qKiFactor,          /* Ki Gain Factor */
        KpGainQvalue,       /* Kp Gain Q value*/
        qPIOut,             /* Unscaled PI Output */
        qOmega,             /* Estimated Speed */
        qOmegaFilt,         /* Filtered Estimated Speed */
        qOmegaPIOut,        /* PI Output */
        qOmegaPIOutFilt,    /* PI Output Filtered */
        qTheta,             /* Estimated Rotor Angle */
        qLaa,               /* Lalpha-alpha */
        qLab,               /* Lalpha-beta */
        qLba,               /* Lbeta-alpha */
        qLbb,               /* Lbeta-beta */
        qL1Cos2Theta,       /* L1 * Cos(2*Theta) */
        qL1Sin2Theta,       /* L1 * Sin(2*Theta) */
        qOmegaL1,           /* Omega L1 */
        qEmag,              /* Magnitude of BEMF */
        qEmagFilt,          /* Filtered BEMF */
        SAL_OBS,            /* Saliency in Observer to be included */
        qEmagFiltConst,     /* BEMF Filter Constant */
        qOmegaFiltConst,    /* Omega Filter Constant */
        qOmegaPIFiltConst;  /* PI Output Filter Constant */

    int32_t
        qKiOut,             /* I controller output */
        qKpOut,             /* P controller output */
        qEaCosTheta,        /* Ealpha * cos(Theta) */
        qEaSinTheta,        /* Ealpha * sin(Theta) */
        qEbCosTheta,        /* Ebeta *  cos(Theta) */
        qEbSinTheta,        /* Ebeta *  sin(Theta) */
        qOmegaPIOutFiltTemp,/* PI Controller output filter state variable */
        qOmegaFiltStateVar, /* Speed in high precision */
        qThetaStateVar,     /* Angle in high precision */
        qEdFiltStateVar,    /* Ed filter variable */
        qEqFiltStateVar,    /* Eq filter variable */
        qEmagFiltStateVar;  /* Emag filter variable */

    const MCAPP_CONTROL_T *pCtrlParam;
    const MC_ALPHABETA_T *pIAlphaBeta;
    const MC_ALPHABETA_T *pVAlphaBeta;
    const MCAPP_MOTOR_T *pMotor;

} MCAPP_ESTIMATOR_ATPLL_T;

// </editor-fold>

// <editor-fold defaultstate="expanded" desc="INTERFACE FUNCTIONS ">

void MCAPP_EstimatorATPLLInit (MCAPP_ESTIMATOR_ATPLL_T *);
void MCAPP_EstimatorATPLL (MCAPP_ESTIMATOR_ATPLL_T *);

// </editor-fold>

#ifdef __cplusplus
    }
#endif

#endif /* end of __ESTIM_ATPLL_H */
