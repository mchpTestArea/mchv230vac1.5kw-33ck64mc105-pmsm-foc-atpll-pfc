// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file foc.c
 *
 * @brief This module implements Field Oriented Control(FOC).
 *
 * Component: FOC
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

// <editor-fold defaultstate="collapsed" desc="HEADER FILES ">

#include <stdint.h>
#include <stdbool.h>

#include <libq.h>

#include "foc.h"
#include "general.h"

#include "id_ref.h"
#include "estim_atpll.h"
#include "port_config.h"
#include "mc1_calc_params.h"
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Definitions ">
#define Q14_SQRT_3  28377 /* sqrt(3)  */
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="STATIC FUNCTIONS ">

static void MCAPP_FOCFeedbackPath(MCAPP_FOC_T *);
static void MCAPP_FOCForwardPath(MCAPP_FOC_T *);
static void MCAPP_SpeedReferenceRamp(MCAPP_CONTROL_T *);
static void MCAPP_CalculateModulationSiganl(MC_ABC_T *, MC_ABC_T *);
static void MCAPP_DCLinkVoltageCompensation(MC_ABC_T *, MC_ABC_T *, int16_t* );

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="GLOBAL FUNCTIONS ">


// </editor-fold>

/**
* <B> Function: void MCAPP_FOCInit(MCAPP_FOC_T *)  </B>
*
* @brief Executes FOC Parameters Initialization.
*
* @param Pointer to the data structure containing FOC parameters.
* @return none.
* @example
* <CODE> MCAPP_FOCInit(&mc); </CODE>
*
*/
void MCAPP_FOCInit(MCAPP_FOC_T *pFOC)
{
    MCAPP_CONTROL_T *pCtrlParam = &pFOC->ctrlParam;
    
    MCAPP_ControllerPIInit(&pFOC->piDCurrent);
    MCAPP_ControllerPIInit(&pFOC->piQCurrent);
    MCAPP_ControllerPIInit(&pFOC->piSpeed);
    
    MCAPP_FluxWeakeningControlInit(&pFOC->fluxControl);
    MCAPP_EstimatorATPLLInit(&pFOC->estimATPLL);
    
    pCtrlParam->lockTime = 0;
    pCtrlParam->speedRampSkipCnt = 0;
    pCtrlParam->OLTheta = 0;
    pCtrlParam->qVelRef = 0;
    pCtrlParam->OLThetaSum = 0;
    pCtrlParam->qIdRef = 0;
    pCtrlParam->qIqRef = 0;
    
    pFOC->faultStatus = 0;
    
    pFOC->pPWMDuty->dutycycle3 = 0;
    pFOC->pPWMDuty->dutycycle2 = 0;
    pFOC->pPWMDuty->dutycycle1 = 0;
}

/**
* <B> Function: void MCAPP_FOCControlLoop(MCAPP_FOC_T *)  </B>
*
* @brief Executes Speed and Current Control Loops and performs actions
*        associated with control state transition required for FOC.
*
* @param Pointer to the data structure containing FOC parameters.
* @return none.
* @example
* <CODE> MCAPP_FOCControlLoop(&mc); </CODE>
*
*/
void MCAPP_FOCStateMachine(MCAPP_FOC_T *pFOC)
{
    MCAPP_CONTROL_T *pCtrlParam = &pFOC->ctrlParam;

    switch (pFOC->focState)
    {
        case FOC_INIT:
            MCAPP_FOCInit(pFOC); 
            break;
            
        case FOC_RTR_LOCK:
            MCAPP_FOCFeedbackPath(pFOC); 

            pFOC->estimInterface.qTheta = 0;
            
            pCtrlParam->qIqRef = 0;
            pCtrlParam->qIdRef =  pCtrlParam->lockCurrent;
                    
            MCAPP_FOCForwardPath(pFOC);
            
            if (pCtrlParam->lockTime <= pCtrlParam->lockTimeLimit)
            {
                pCtrlParam->lockTime++;
            }
            else
            {
                pCtrlParam->lockTime = 0;
                pCtrlParam->speedRampSkipCnt = 0;  
                
                pFOC->focState = FOC_OPEN_LOOP;
                /* Reset open loop parameters */
                pCtrlParam->OLThetaSum = 0;
                pCtrlParam->OLTheta = (int16_t)(pCtrlParam->OLThetaSum >> 15);            
                pFOC->estimInterface.qTheta  = pCtrlParam->OLTheta;
            }
            
        break;
        
        case FOC_OPEN_LOOP:
            MCAPP_FOCFeedbackPath(pFOC); 
            
            if(pCtrlParam->OLCurrent < pCtrlParam->OLCurrentMax)
            {
                pCtrlParam->OLCurrent += pCtrlParam->OLCurrentRampRate;
            }
            
            if(pCtrlParam->qVelRef < pFOC->pMotor->qMaxOLSpeed)
            {
                if(pCtrlParam->speedRampSkipCnt >= 
                                pCtrlParam->speedRampSkipCntLimit)
                {
                    pCtrlParam->speedRampSkipCnt = 0;
                    pCtrlParam->qVelRef += pCtrlParam->OLSpeedRampRate;
                }
                else
                {
                    pCtrlParam->speedRampSkipCnt++;
                }
            }
            else if((pCtrlParam->openLoop == 0)&&
                    (pFOC->estimATPLL.qOmegaFilt > pFOC->pMotor->qMaxOLSpeed))
            {
                pFOC->estimInterface.qThetaOffset = pCtrlParam->OLTheta 
                                                  - pFOC->estimATPLL.qTheta;

                /* Reset speed PI controller */
                MCAPP_ControllerPIReset(&pFOC->piSpeed, pFOC->idq.q);                 
                pCtrlParam->speedRampSkipCnt = 0;          
                pFOC->focState = FOC_CLOSE_LOOP;
            }
            
            MCAPP_EstimatorATPLL(&pFOC->estimATPLL);

            /* Calculate open loop theta */
            pCtrlParam->OLThetaSum += __builtin_mulss(pCtrlParam->qVelRef, 
                                                        pCtrlParam->normDeltaT);           
            pCtrlParam->OLTheta = (int16_t)(pCtrlParam->OLThetaSum >> 15);
            
            pFOC->estimInterface.qTheta  = pCtrlParam->OLTheta;

            pCtrlParam->qIqRef = pCtrlParam->OLCurrent;
            
            if(pCtrlParam->qIdRef > 0)
            {
                pCtrlParam->qIdRef --;
            }
                
            MCAPP_FOCForwardPath(pFOC);
            break;  
            
        case FOC_CLOSE_LOOP:
            
            MCAPP_FOCFeedbackPath(pFOC);
            
            MCAPP_EstimatorATPLL(&pFOC->estimATPLL);
            
            /* Correct Theta Offset */            
            if(pFOC->estimInterface.qThetaOffset > 10)
            {
                pFOC->estimInterface.qThetaOffset-= 10;
            }
            else if(pFOC->estimInterface.qThetaOffset < - 10)
            {
                pFOC->estimInterface.qThetaOffset+= 10;
            }
            else
            {
                pFOC->estimInterface.qThetaOffset = 0;
            }
            pFOC->estimInterface.qTheta  = pFOC->estimATPLL.qTheta + pFOC->estimInterface.qThetaOffset;                      
            pFOC->estimInterface.qVelEstim = pFOC->estimATPLL.qOmegaFilt ;
            
            MCAPP_SpeedReferenceRamp(pCtrlParam);
            
            /* Execute Outer Speed Loop - Iq Reference Generation */
            MCAPP_ControllerPIUpdate(pCtrlParam->qVelRef, pFOC->estimInterface.qVelEstim, 
            &pFOC->piSpeed, MCAPP_SAT_NONE, &pCtrlParam->qIqRef,
            pCtrlParam->qVelRef);
            
            /* Id Reference generation- Flux Weakening  */
            MCAPP_FluxWeakeningControl(&pFOC->fluxControl);
            pCtrlParam->qIdRef = pFOC->fluxControl.feedBackFW.IdRef;
 
            MCAPP_FOCForwardPath(pFOC);
            break;

        case FOC_FAULT:
                    
            break;
        
        default:
            pFOC->focState = FOC_FAULT;
            break;

    } /* End Of switch - case */
}

/**
* <B> Function: void MCAPP_FOCFeedbackPath (MCAPP_FOC_T *)  </B>
*
* @brief Function executing Field Oriented Control(FOC).
*
* @param Pointer to the data structure containing FOC parameters.
* @return none.
* @example
* <CODE> MCAPP_FOCFeedbackPath(&mc); </CODE>
*
*/
static void MCAPP_FOCFeedbackPath(MCAPP_FOC_T *pFOC)
{
    pFOC->iabc.a = *(pFOC->pIa);
    pFOC->iabc.b = *(pFOC->pIb);
    pFOC->iabc.c = -pFOC->iabc.a - pFOC->iabc.b;
    
    /* Perform Clark & Park transforms to generate d axis and q axis currents */
    MC_TransformClarke_Assembly(&pFOC->iabc, &pFOC->ialphabeta);
    
    MC_TransformPark_Assembly(&pFOC->ialphabeta, &pFOC->sincosTheta, 
                                    &pFOC->idq);
}

/**
* <B> Function: void MCAPP_FOCForwardPath(MCAPP_FOC_T *)  </B>
*
* @brief Executes Current Control Loops required for FOC.
*
* @param Pointer to the data structure containing FOC parameters.
* @return none.
* @example
* <CODE> MCAPP_FOCForwardPath(&mc); </CODE>
*
*/

static void MCAPP_FOCForwardPath(MCAPP_FOC_T *pFOC)
{
    int16_t vqSquaredLimit, vdSquared, vPhaseMax, vMaxSquare;
    
    /** Execute inner current control loops */
    /* Execute PI Control of Q axis. */
    MCAPP_ControllerPIUpdate(pFOC->ctrlParam.qIdRef,  pFOC->idq.d, 
            &pFOC->piDCurrent, MCAPP_SAT_NONE, &pFOC->vdq.d,
            pFOC->ctrlParam.qIdRef);

    /* Generate Q axis current reference based on available voltage and D axis
       voltage */
    vPhaseMax = (int16_t)(__builtin_mulss(VMAX_FACTOR, (*pFOC->pVdc))>>15);
    vMaxSquare = (int16_t)(__builtin_mulss(vPhaseMax, vPhaseMax)>>15);
    vdSquared  = (int16_t)(__builtin_mulss(pFOC->vdq.d, pFOC->vdq.d)>>15);
    vqSquaredLimit = vMaxSquare - vdSquared;  
    pFOC->piDCurrent.outMax = vPhaseMax;
    pFOC->piDCurrent.outMin = -vPhaseMax; 
    pFOC->piQCurrent.outMax = _Q15sqrt(vqSquaredLimit);
    pFOC->piQCurrent.outMin = -(pFOC->piQCurrent.outMax); 
    
    /* Execute PI Control of Q axis. */ 
    MCAPP_ControllerPIUpdate(pFOC->ctrlParam.qIqRef,  pFOC->idq.q, 
            &pFOC->piQCurrent, MCAPP_SAT_NONE, &pFOC->vdq.q,
            pFOC->ctrlParam.qIqRef);
    
    /* Calculate sin and cos of theta (angle) */
    MC_CalculateSineCosine_Assembly_Ram(pFOC->estimInterface.qTheta, 
                                            &pFOC->sincosTheta);

    /* Perform inverse Clarke and Park transforms and generate phase voltages.*/
    MC_TransformParkInverse_Assembly(&pFOC->vdq, &pFOC->sincosTheta, 
                                                        &pFOC->valphabeta);
    
    /* Calculate Vr1,Vr2,Vr3 from qValpha, qVbeta */
    MC_TransformClarkeInverse_Assembly(&pFOC->valphabeta, &pFOC->vabc);
        
    /* DC Link voltage compensation */
    MCAPP_DCLinkVoltageCompensation(&pFOC->vabc, &pFOC->vabcCompDC, pFOC->pVdc);
    
    /* Calculate modulation signal input for MC_CalculateSpaceVector_Assembly */
    MCAPP_CalculateModulationSiganl(&pFOC->vabcCompDC, &pFOC->vabcScaled);

    /* Execute space vector modulation and generate PWM duty cycles */
    MC_CalculateSpaceVector_Assembly(&pFOC->vabcScaled, pFOC->pwmPeriod,
                                                    pFOC->pPWMDuty);
}



/**
* <B> Function: void MCAPP_SpeedReferenceRamp(MCAPP_CONTROL_T *)  </B>
*
*/
static void MCAPP_SpeedReferenceRamp(MCAPP_CONTROL_T *pCtrlParam)
{
    int16_t deltaSpeed = pCtrlParam->qVelRef - 
                                (int16_t)pCtrlParam->qTargetVelocity;
    if(deltaSpeed < 0)
    {
        if(pCtrlParam->speedRampSkipCnt >= pCtrlParam->speedRampIncLimit)
        {
            pCtrlParam->speedRampSkipCnt = 0;
            pCtrlParam->qVelRef = pCtrlParam->qVelRef + pCtrlParam->CLSpeedRampRate;
        }
    }
    else if(deltaSpeed > 0)
    {
        if(pCtrlParam->speedRampSkipCnt >= pCtrlParam->speedRampDecLimit)
        {
            pCtrlParam->speedRampSkipCnt = 0;
            pCtrlParam->qVelRef = pCtrlParam->qVelRef - pCtrlParam->CLSpeedRampRate;
        }
    }
    else
    {
        pCtrlParam->speedRampSkipCnt = 0;
    }
    pCtrlParam->speedRampSkipCnt++;
}


/**
* <B> Function: void MCAPP_CalculateModulationSiganl(MC_ABC_T *, MC_ABC_T *)  </B>
*
*/
static void MCAPP_CalculateModulationSiganl(MC_ABC_T *pabcIn, MC_ABC_T *pvabcOut)
{
    /* Multiply by Root(3) */
    pvabcOut->a = (int16_t) (__builtin_mulss(pabcIn->a, Q14_SQRT_3) >> 14);
    pvabcOut->b = (int16_t) (__builtin_mulss(pabcIn->b, Q14_SQRT_3) >> 14);
    pvabcOut->c = (int16_t) (__builtin_mulss(pabcIn->c, Q14_SQRT_3) >> 14);
}

/**
 * <B> Function: void MCAPP_DCLinkVoltageCompensation(MC_ABC_T *, MC_ABC_T *, int16_t *)  </B>
 *  @brief Executes DC voltage compensation
 * 
 */
static void MCAPP_DCLinkVoltageCompensation(MC_ABC_T *pvabc, MC_ABC_T *pdabc, int16_t *pvdc)
{
    int16_t vdcRatio, vdcScale=0;
    const int16_t vdc = *pvdc ;
    
    if (vdc > DC_LINK_BASE_VOLTAGE)
    {
        vdcRatio = __builtin_divf(DC_LINK_BASE_VOLTAGE, vdc);
        vdcScale = 0;
    }
    else if (vdc > (DC_LINK_BASE_VOLTAGE>>1))
    {
        vdcRatio = __builtin_divf((DC_LINK_BASE_VOLTAGE>>1), vdc);
        vdcScale = 1;
    }
    else
    {
        vdcRatio = 0;
        vdcScale = 0;
    }
    pdabc->a = (int16_t) (__builtin_mulss(pvabc->a, vdcRatio) >> (15-vdcScale));
    pdabc->b = (int16_t) (__builtin_mulss(pvabc->b, vdcRatio) >> (15-vdcScale));
    pdabc->c = (int16_t) (__builtin_mulss(pvabc->c, vdcRatio) >> (15-vdcScale));
}
