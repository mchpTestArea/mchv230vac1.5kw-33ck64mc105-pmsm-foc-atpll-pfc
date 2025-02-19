// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file estim_atpll.c
 *
 * @brief This module implements ATPLL(Angle Tracking PLL ) Estimator.
 * This is a sensor-less speed observer based on motor back EMF.
 *
 * Component: ESTIMATOR
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

// <editor-fold defaultstate="collapsed" desc="HEADER FILES ">

#include <stdint.h>
#include <stdbool.h>

/* _Q15abs and _Q15sqrt function use */
#include <libq.h>

#include "estim_atpll.h"

// </editor-fold>

/**
* <B> Function: void MCAPP_EstimatorATPLLInit(MCAPP_ATPLL_ESTIMATOR_T *)  </B>
*
* @brief Function to reset ATPLL Estimator Data Structure variables.
*
* @param    pointer to the data structure containing ATPLL Estimator parameters.
* @return   none.
* @example
* <CODE> MMCAPP_EstimatorATPLLInit(&estimator); </CODE>
*
*/
void MCAPP_EstimatorATPLLInit(MCAPP_ESTIMATOR_ATPLL_T *pEstim)
{
    pEstim->qOmegaFiltStateVar = 0;
    pEstim->qThetaStateVar = 0;
    pEstim->qKiOut = 0;
    pEstim->qOmegaPIOutFiltTemp = 0;
    pEstim->qEdFiltStateVar = 0;
    pEstim->qEqFiltStateVar = 0;
    pEstim->qTheta = 0;
    pEstim->qOmegaFilt = 0;
}

/**
* <B> Function: void MCAPP_EstimatorATPLL(MCAPP_ATPLL_ESTIMATOR_T *)  </B>
*
* @brief Observer to determine rotor speed and position based on
* motor parameters and feedbacks.
*
* @param    pointer to the data structure containing ATPLL Estimator parameters.
* @return   none.
* @example
* <CODE> MCAPP_EstimatorATPLL(&estimator); </CODE>
*
*/   

void MCAPP_EstimatorATPLL(MCAPP_ESTIMATOR_ATPLL_T *pEstim)
{
    const MCAPP_MOTOR_T *pMotor = pEstim->pMotor;
    const MC_ALPHABETA_T *pIAlphaBeta = pEstim->pIAlphaBeta;
    const MCAPP_CONTROL_T *pCtrlParam = pEstim->pCtrlParam;

    int16_t
            filtErr,        /* Filter error */
            eAlSqr,         /* E_alpha square */
            eBeSqr,         /* E_beta square */
            kiInFactor;     /* KI for estimator */
    
    MC_SINCOS_T   sincosTwoTheta;        /* Sine-cosine */  
    
    if(pEstim->SAL_OBS)
    {/* If motor has significant saliency ,execute :*/
        int16_t twoTheta = pEstim->qTheta << 1;
        MC_CalculateSineCosine_Assembly_Ram(twoTheta, &sincosTwoTheta);
        pEstim->qL1Cos2Theta = (int16_t)(__builtin_mulss(pEstim->qL1Dt,
                                                    sincosTwoTheta.cos) >> 15);
        pEstim->qL1Sin2Theta = (int16_t)(__builtin_mulss(pEstim->qL1Dt,
                                                    sincosTwoTheta.sin) >> 15);

        pEstim->qLaa = pEstim->qL0Dt + pEstim->qL1Cos2Theta;
        pEstim->qLab = pEstim->qL1Sin2Theta;
        pEstim->qLba = pEstim->qL1Sin2Theta;
        pEstim->qLbb = pEstim->qL0Dt - pEstim->qL1Cos2Theta;
    }

    pEstim->qLastVAlphaBeta.alpha = pEstim->pVAlphaBeta->alpha;
    pEstim->qLastVAlphaBeta.beta = pEstim->pVAlphaBeta->beta;
    
    /*
       Stator voltage equations:
       Valpha = Rs * Ialpha + d((Ls/dT) * (Ialpha)) + BEMFalpha
       Vbeta  = Rs * Ibeta + d((Ls/dT) * (Ibeta)) + BEMFbeta
    */
    pEstim->qDIAlphaBeta.alpha = (pIAlphaBeta->alpha - pEstim->qLastIAlphaBeta.alpha);
    pEstim->qDIAlphaBeta.beta = (pIAlphaBeta->beta - pEstim->qLastIAlphaBeta.beta);
    
    /* Update  LastIalpha and LastIbeta */
    pEstim->qLastIAlphaBeta.alpha   = pIAlphaBeta->alpha;
    pEstim->qLastIAlphaBeta.beta	= pIAlphaBeta->beta;
    
    if(pEstim->SAL_OBS) /* There is significant saliency in the motor */
    {
        /* If motor has significant saliency ,execute : */
        pEstim->qVIndAlphaBeta.alpha =
            (int16_t)( ( (__builtin_mulss(pEstim->qLaa, pEstim->qDIAlphaBeta.alpha)) +
                           (__builtin_mulss(pEstim->qLab, pEstim->qDIAlphaBeta.beta)) )
                                >> (pEstim->LsDtBaseScale) );

        pEstim->qVIndAlphaBeta.beta =
            (int16_t)(((__builtin_mulss(pEstim->qLba, pEstim->qDIAlphaBeta.alpha)) +
                           (__builtin_mulss(pEstim->qLbb, pEstim->qDIAlphaBeta.beta)))
                                >> (pEstim->LsDtBaseScale ));
    }
    else
    {
        pEstim->qVIndAlphaBeta.alpha = (int16_t) (__builtin_mulss(pEstim->qL0Dt, 
                        pEstim->qDIAlphaBeta.alpha) >> (pEstim->LsDtBaseScale));
    
        pEstim->qVIndAlphaBeta.beta = (int16_t) (__builtin_mulss(pEstim->qL0Dt, 
                        pEstim->qDIAlphaBeta.beta) >> (pEstim->LsDtBaseScale));
    }

    pEstim->qVResAlphaBeta.alpha =
            (int16_t)(__builtin_mulss( pMotor->qRs, pIAlphaBeta->alpha) >>
                            (pMotor->qRsScale ));

    pEstim->qVResAlphaBeta.beta =
            (int16_t)(__builtin_mulss( pMotor->qRs, pIAlphaBeta->beta) >>
                            (pMotor->qRsScale ));

    pEstim->qEAlphaBeta.alpha = pEstim->qLastVAlphaBeta.alpha -
                                pEstim->qVIndAlphaBeta.alpha -
                                pEstim->qVResAlphaBeta.alpha;

    pEstim->qEAlphaBeta.beta = pEstim->qLastVAlphaBeta.beta -
                                pEstim->qVIndAlphaBeta.beta -
                                pEstim->qVResAlphaBeta.beta;

    MC_CalculateSineCosine_Assembly_Ram(pEstim->qTheta, &pEstim->qSinCos);
    /*  Park_BEMF.d =  Clark_BEMF.alpha*cos(Angle) + Clark_BEMF.beta*sin(Rho)
       Park_BEMF.q = -Clark_BEMF.alpha*sin(Angle) + Clark_BEMF.beta*cos(Rho)*/
    MC_TransformPark_Assembly(&pEstim->qEAlphaBeta, &pEstim->qSinCos, &pEstim->qEdq);

    pEstim->qEdNegetive = - pEstim->qEdq.d;
    
    /* Two point averaging */
    pEstim->qEerr = (pEstim->qEerr + pEstim->qEdNegetive ) >> 1;
    
    /* Filter BEMF magnitude */    
    eAlSqr = (int16_t)(__builtin_mulss(pEstim->qEAlphaBeta.alpha,
                    pEstim->qEAlphaBeta.alpha) >> 15);
    eBeSqr = (int16_t)(__builtin_mulss(pEstim->qEAlphaBeta.beta,
                    pEstim->qEAlphaBeta.beta) >> 15);

    pEstim->qEmag = (_Q15sqrt(eAlSqr + eBeSqr));

    filtErr = pEstim->qEmag - pEstim->qEmagFilt;
    pEstim->qEmagFiltStateVar += 
                            __builtin_mulss(filtErr, pEstim->qEmagFiltConst);
    pEstim->qEmagFilt = (int16_t)(pEstim->qEmagFiltStateVar >> 15);

    /*  Calculate deltaOmega needed to align estimator with BEMF */

    /* Compute PI Output */
    /* Kp Output */
    pEstim->qKpOut = (__builtin_mulss(pEstim->qEerr,
                                pEstim->qKpGain) <<(15 - pEstim->KpGainQvalue));

    kiInFactor = (int16_t)(__builtin_mulss(pCtrlParam->qVelRef, 
                                                        pEstim->qKiFactor)>>15);
    /* Ki Output */
    pEstim->qKiOut += __builtin_mulss(pEstim->qEerr ,kiInFactor) ;
    
    pEstim->qPIOut = (int16_t)( (pEstim->qKpOut + pEstim->qKiOut) >> 15);
    
    /* Compensating effect of BEMF constant */
    pEstim->qOmegaPIOut =
                (int16_t)(__builtin_mulss(pEstim->qPIOut, pEstim->qInvKFIBase)>>
									(pEstim->invKFIBaseScale));

    /* Filter on PI Output */
    filtErr = pEstim->qOmegaPIOut - pEstim->qOmegaPIOutFilt;
    pEstim->qOmegaPIOutFiltTemp +=
                            __builtin_mulss(filtErr, pEstim->qOmegaPIFiltConst);
    pEstim->qOmegaPIOutFilt = (int16_t)(pEstim->qOmegaPIOutFiltTemp >> 15);

    if(pEstim->feedForwardMode == 1)
    {
        pEstim->feedForwardSpeed = (int16_t)(__builtin_mulss(pEstim->qEmagFilt , 
                pEstim->qInvKFIBase) >> (pEstim->invKFIBaseScale));
    }
    else
    {
        pEstim->feedForwardSpeed = pCtrlParam->qVelRef;
    }
    
    /* Calculate Speed and angle */
    pEstim->qOmega = (int16_t)( pEstim->qOmegaPIOutFilt + pEstim->feedForwardSpeed );
    
    pEstim->qThetaStateVar += 
                        __builtin_mulss(pEstim->qOmega, pCtrlParam->normDeltaT);

    pEstim->qTheta = (int16_t)( pEstim->qThetaStateVar >> 15);
    
    /* Filter on estimated speed for outer speed loop */
    filtErr = pEstim->qOmega - pEstim->qOmegaFilt;

    pEstim->qOmegaFiltStateVar += 
                            (__builtin_mulss(pEstim->qOmegaFiltConst, filtErr));

    pEstim->qOmegaFilt = (int16_t)(pEstim->qOmegaFiltStateVar >> 15);
}
