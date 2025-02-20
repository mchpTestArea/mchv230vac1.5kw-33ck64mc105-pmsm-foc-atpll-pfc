// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file id_ref.c
 *
 * @brief This module implements flux control or d axis current reference 
 * generation calculations.
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
#include <libq.h>
#include "id_ref.h"
#include "mc1_user_params.h"
#include "mc1_calc_params.h"
#include "general.h"

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="STATIC FUNCTIONS ">
static void MCAPP_FluxControlVoltFeedback(MCAPP_FLUX_WEAKENING_VOLT_FB_T *);
static void MCAPP_FluxControlVoltFeedbackInit(MCAPP_FLUX_WEAKENING_VOLT_FB_T *);
// </editor-fold>

// <editor-fold defaultstate="expanded" desc="INTERFACE FUNCTIONS ">

/**
* <B> Function: MCAPP_FluxWeakeningControlInit() </B>
*
* @brief Function initializes the Field Weakening Parameters.
*        
*/
void MCAPP_FluxWeakeningControlInit(MCAPP_ID_REFERENCE_T *pIdRefGen)
{
    MCAPP_FLUX_WEAKENING_VOLT_FB_T *pFeedBackFW = &pIdRefGen->feedBackFW;

    MCAPP_FluxControlVoltFeedbackInit(pFeedBackFW);
}
/**
* <B> Function: FieldWeakening(float, float ) </B>
*
* @brief Function implements equation based field weakening algorithm.
*        
* @param motor speed.
* @return d-axis current reference corresponding to the motor speed.
* 
* @example
* <CODE> id = FieldWeakening(omega); </CODE>
*
*/


void MCAPP_FluxWeakeningControl(MCAPP_ID_REFERENCE_T *pIdRefGen)
{
    MCAPP_FLUX_WEAKENING_VOLT_FB_T *pFeedBackFW = &pIdRefGen->feedBackFW;

    /* Calculate Flux Weakening Control current */

    MCAPP_FluxControlVoltFeedback(pFeedBackFW);
}

/**
* <B> Function: MCAPP_FluxControlVoltFeedbackInit(MCAPP_FLUX_WEAKENING_VOLT_FB_T * )  </B>
*
* @brief Function to reset variables used for Field Weakening Control.
*        .
*
* @param Pointer to the data structure containing Field Weakening Control
*        parameters.
* @return   none.
* @example
* <CODE> MCAPP_FluxControlVoltFeedbackInit(&fieldWeak); </CODE>
*
*/
static void  MCAPP_FluxControlVoltFeedbackInit(MCAPP_FLUX_WEAKENING_VOLT_FB_T *pFdWeak)
{    
    pFdWeak->FWeakPI.integrator = 0;
    pFdWeak->IdRefFiltStateVar = 0;
    pFdWeak->IdRef = 0;
}

/**
* <B> Function: MCAPP_FluxControlVoltFeedback(MCAPP_FLUX_WEAKENING_VOLT_FB_T * )  </B>
*
* @brief Function implementing Field Weakening Control
*
* @param Pointer to the data structure containing Field Weakening Control
*        parameters.
* @return   none.
* @example
* <CODE> MCAPP_FluxControlVoltFeedback(&fieldWeak); </CODE>
*
*/

static void MCAPP_FluxControlVoltFeedback(MCAPP_FLUX_WEAKENING_VOLT_FB_T *pFdWeak)
{    
    const MC_DQ_T *pVdq         = pFdWeak->pVdq;
    const MCAPP_MOTOR_T *pMotor = pFdWeak->pMotor;
    MCAPP_CONTROL_T *pCtrlParam = pFdWeak->pCtrlParam;

    int16_t vdSqr, vqSqr, IdRefOut; 

    /* Compute voltage vector magnitude */
    vdSqr  = (int16_t)(__builtin_mulss(pVdq->d, pVdq->d) >> 15);
    vqSqr  = (int16_t)(__builtin_mulss(pVdq->q, pVdq->q) >> 15);
    pFdWeak->voltageMag = _Q15sqrt(vdSqr+vqSqr);
    
    if((pCtrlParam->qVelRef > (pMotor->qNominalSpeed>>1)))
    { 
        /* Compute PI output: pFdWeak->IdRef */
        MCAPP_ControllerPIUpdate(pFdWeak->voltageMagRef, pFdWeak->voltageMag, 
            &pFdWeak->FWeakPI, MCAPP_SAT_NONE, &IdRefOut, pFdWeak->voltageMagRef); 
    }
    else
    {
        IdRefOut = 0 ;
        
        /* Reset PI integrator to Nominal Idrefernce value for smooth transition
         * when switching to PI output computation. */
        MCAPP_ControllerPIReset(&pFdWeak->FWeakPI, 0);
    }

    /*Filter for the FW Idref current*/
#ifdef ID_REFERNCE_FILTER_ENABLE
    pFdWeak->IdRefFiltStateVar +=
                    __builtin_mulss((IdRefOut - pFdWeak->IdRefFilt),
                                        pFdWeak->IdRefFiltConst);
    pFdWeak->IdRefFilt = (int16_t)(pFdWeak->IdRefFiltStateVar >> 15);
    pFdWeak->IdRef = pFdWeak->IdRefFilt;
#else
    pFdWeak->IdRef = IdRefOut;
#endif
}



// </editor-fold>
