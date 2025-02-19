// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file mc1_init.c
 *
 * @brief This module initializes data structure holding motor control
 * parameters required to run motor 1 using field oriented control.
 * In this application to initialize variable required to run the application.
 *
 * Component: APPLICATION (Motor Control 1 - mc1)
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
#include <string.h>

#include "mc1_init.h"
#include "mc1_calc_params.h"
#include "mc_app_types.h"

#include "board_service.h"
#include "fault.h"
#include "generic_load.h"
#include "generic_load_types.h"

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="STATIC FUNCTIONS ">

static void MCAPP_MC1ControlSchemeConfig(MC1APP_DATA_T *);
static void MCAPP_MC1LoadConfig(MC1APP_DATA_T *);
static void MCAPP_MC1FeedbackConfig(MC1APP_DATA_T *);
static void MCAPP_MC1LoadStartTransition(MCAPP_CONTROL_SCHEME_T *, 
                                            MCAPP_LOAD_T *);
static void MCAPP_MC1LoadStopTransition(MCAPP_CONTROL_SCHEME_T *, 
                                            MCAPP_LOAD_T *);
static void MCAPP_MC1OutputConfig(MC1APP_DATA_T *);

// </editor-fold>

/**
* <B> Function: MCAPP_MC1ParamsInit (MC1APP_DATA_T *)  </B>
*
* @brief Function to reset variables used for current offset measurement.
*
* @param Pointer to the Application data structure required for 
* controlling motor 1.
* @return none.
* @example
* <CODE> MCAPP_MC1ParamsInit(&mc1); </CODE>
*
*/
void MCAPP_MC1ParamsInit(MC1APP_DATA_T *pMCData)
{    
    /* Reset all variables in the data structure to '0' */
    memset(pMCData,0,sizeof(MC1APP_DATA_T));

    pMCData->pControlScheme = &pMCData->controlScheme;
    pMCData->pMotorInputs = &pMCData->motorInputs;
    pMCData->pLoad = &pMCData->load;
    pMCData->pMotor = &pMCData->motor;
    pMCData->pPWMDuty = &pMCData->PWMDuty;
    
    /* Configure Feedbacks */
    MCAPP_MC1FeedbackConfig(pMCData);
    
    /* Configure Control Scheme */
    MCAPP_MC1ControlSchemeConfig(pMCData);
    
    /* Configure Load */
    MCAPP_MC1LoadConfig(pMCData);
    
    /* Configure Outputs */
    MCAPP_MC1OutputConfig(pMCData);

    /* Set motor control state as 'MTR_INIT' */
    pMCData->appState = MCAPP_INIT;
}

void MCAPP_MC1FeedbackConfig(MC1APP_DATA_T *pMCData)
{
    pMCData->HAL_MotorInputsRead = HAL_MC1MotorInputsRead;
    
    pMCData->motorInputs.measureVdc.dcMinRun = 
                        NORM_VALUE(MC1_MOTOR_MIN_DC_VOLT, MC1_PEAK_VOLTAGE);
    
    pMCData->motorInputs.measureVdc.dcMaxStop = 
                        NORM_VALUE(MC1_MOTOR_MIN_DC_VOLT, MC1_PEAK_VOLTAGE);

}

void MCAPP_MC1ControlSchemeConfig(MC1APP_DATA_T *pMCData)
{
    MCAPP_CONTROL_SCHEME_T *pControlScheme;
    MCAPP_MEASURE_T *pMotorInputs;
    MCAPP_MOTOR_T *pMotor;
    MCAPP_FAULT_T *pFault;
    
    pControlScheme = pMCData->pControlScheme;
    pMotorInputs = pMCData->pMotorInputs;
    pMotor = pMCData->pMotor;
    pFault = &pMCData->fault;
    
    /* Configure Inputs */  
    pControlScheme->pIa = &pMotorInputs->measureCurrent.Ia;
    pControlScheme->pIb = &pMotorInputs->measureCurrent.Ib;
    pControlScheme->pVdc = &pMotorInputs->measureVdc.value;  
    pControlScheme->pMotor = pMCData->pMotor;
    
    /* Initialize IMotor parameters */
    pMotor->polePairs       = POLEPAIRS;
    pMotor->qRs             = NORM_RS;
    pMotor->qRsScale        = NORM_RS_QVALUE;
    pMotor->qLsDt           = NORM_L0DT;
    pMotor->qLsDtScale      = NORM_LDT_QVALUE;  
    pMotor->qNominalSpeed   = NORM_VALUE(NOMINAL_SPEED_RPM, MC1_PEAK_SPEED_RPM);
    pMotor->qMaxSpeed       = NORM_VALUE(MAXIMUM_SPEED_RPM, MC1_PEAK_SPEED_RPM);
    pMotor->qMaxOLSpeed     = NORM_VALUE(END_SPEED_RPM, MC1_PEAK_SPEED_RPM);
    pMotor->qMinSpeed       = NORM_VALUE(MINIMUM_SPEED_RPM, MC1_PEAK_SPEED_RPM);

    pMotor->qRatedCurrent = NORM_VALUE(NOMINAL_CURRENT_PEAK, MC1_PEAK_CURRENT); 

    /* Initialize fault parameters */
    pFault->overCurrentFaultLimit = PEAK_FAULT_CURRENT;
    
    
    /* Initialize FOC control parameters */
#ifdef  OPEN_LOOP_FUNCTIONING
    pControlScheme->ctrlParam.openLoop = 1;
#else
    pControlScheme->ctrlParam.openLoop = 0;
#endif   
    
    pControlScheme->ctrlParam.lockTimeLimit = LOCK_TIME_COUNT;
    pControlScheme->ctrlParam.lockCurrent = 
                    NORM_VALUE(LOCK_CURRENT, MC1_PEAK_CURRENT);

    pControlScheme->ctrlParam.OLCurrent = 
                    NORM_VALUE(MIN_OPENLOOP_CURRENT, MC1_PEAK_CURRENT);
    pControlScheme->ctrlParam.OLCurrentMax = 
                    NORM_VALUE(MAX_OPENLOOP_CURRENT, MC1_PEAK_CURRENT);
    pControlScheme->ctrlParam.OLCurrentRampRate = OL_CURRENT_RAMP_RATE_COUNT;

    pControlScheme->ctrlParam.speedRampSkipCntLimit = OL_SPEED_RAMP_TIME_MULTIPLIER;
    pControlScheme->ctrlParam.OLSpeedRampRate = OL_SPEED_RAMP_RATE_COUNT;

    pControlScheme->ctrlParam.qTargetVelocity = pMotor->qMaxOLSpeed;
    
    pControlScheme->ctrlParam.CLSpeedRampRate = SPEED_RAMP_RATE_COUNT;
    pControlScheme->ctrlParam.speedRampIncLimit = RAMP_UP_TIME_MULTIPLIER;
    pControlScheme->ctrlParam.speedRampDecLimit = RAMP_DN_TIME_MULTIPLIER;   
    
    pControlScheme->ctrlParam.normDeltaT = NORM_DELTA_T;

    
    /* Initialize PI controller used for D axis current control */
    pControlScheme->piDCurrent.kp = D_CURRCNTR_PTERM;
    pControlScheme->piDCurrent.nkp = D_CURRCNTR_PTERM_SCALE;
    pControlScheme->piDCurrent.ki = D_CURRCNTR_ITERM;
    pControlScheme->piDCurrent.nki = D_CURRCNTR_ITERM_SCALE;
    pControlScheme->piDCurrent.outMax = D_CURRCNTR_OUTMAX;
    pControlScheme->piDCurrent.outMin = (-pControlScheme->piDCurrent.outMax);
    pControlScheme->piDCurrent.kc =  Q15(0.99999);

    /* Initialize PI controller used for Q axis current control */
    pControlScheme->piQCurrent.kp = Q_CURRCNTR_PTERM;
    pControlScheme->piQCurrent.nkp = Q_CURRCNTR_PTERM_SCALE;
    pControlScheme->piQCurrent.ki = Q_CURRCNTR_ITERM;
    pControlScheme->piQCurrent.nki = Q_CURRCNTR_ITERM_SCALE;
    pControlScheme->piQCurrent.outMax = Q_CURRCNTR_OUTMAX;
    pControlScheme->piQCurrent.outMin = (-pControlScheme->piQCurrent.outMax);
    pControlScheme->piQCurrent.kc =  Q15(0.99999);

    /* Initialize PI controller used for speed control */
    pControlScheme->piSpeed.kp = SPEEDCNTR_PTERM;
    pControlScheme->piSpeed.ki = SPEEDCNTR_ITERM;
    pControlScheme->piSpeed.nkp = SPEEDCNTR_PTERM_SCALE;
    pControlScheme->piSpeed.nki = SPEEDCNTR_ITERM_SCALE;
    pControlScheme->piSpeed.outMax = SPEEDCNTR_OUTMAX;
    pControlScheme->piSpeed.outMin = (-pControlScheme->piSpeed.outMax);
    pControlScheme->piSpeed.kc =  Q15(0.99999);

    /* Initialize ATPLL Estimator */
    pControlScheme->estimATPLL.pCtrlParam  = &pControlScheme->ctrlParam;
    pControlScheme->estimATPLL.pIAlphaBeta = &pControlScheme->ialphabeta;
    pControlScheme->estimATPLL.pVAlphaBeta = &pControlScheme->valphabeta;
    pControlScheme->estimATPLL.pMotor      = pMCData->pMotor;

    pControlScheme->estimATPLL.qInvKFIBase = NORM_INVKFI_CONST;
    pControlScheme->estimATPLL.invKFIBaseScale = NORM_INVKFI_CONST_QVALUE;
    pControlScheme->estimATPLL.qL0Dt = NORM_L0DT;
    pControlScheme->estimATPLL.qL1Dt = NORM_L1DT;
    pControlScheme->estimATPLL.LsDtBaseScale = NORM_LDT_QVALUE;
    pControlScheme->estimATPLL.qKiFactor = ATPLL_KI_GAIN_FACTOR;
    pControlScheme->estimATPLL.qKpGain = ATPLL_KP_GAIN;
    pControlScheme->estimATPLL.KpGainQvalue = ATPLL_KP_GAIN_QVALUE;
    pControlScheme->estimATPLL.qOmegaPIFiltConst = OMEGA_PI_FILT_CONST;
    pControlScheme->estimATPLL.qOmegaFiltConst = OMEGA_FILT_CONST;
    pControlScheme->estimATPLL.qEmagFiltConst = EMAG_FILT_CONST;
    pControlScheme->estimATPLL.feedForwardMode = 0;
    
    if(NORM_LDDTBASE <= NORM_LDDT_THRESHOLD )
    {/* Enable Saliency Observer if motor has significant saliency */
        pControlScheme->estimATPLL.SAL_OBS = 1;
    }
    else
    {/* Disable Saliency Observer if motor does not have significant saliency */
        pControlScheme->estimATPLL.SAL_OBS = 0;
    }
	
	    
    
    /* Initialize field weakening controller 2*/ 
    pControlScheme->fluxControl.feedBackFW.pCtrlParam = &pControlScheme->ctrlParam;
    pControlScheme->fluxControl.feedBackFW.pMotor = pMCData->pMotor;
    pControlScheme->fluxControl.feedBackFW.pVdq = &pControlScheme->vdq;
    pControlScheme->fluxControl.feedBackFW.voltageMagRef = FD_WEAK_VOLTAGE_REF;
    pControlScheme->fluxControl.feedBackFW.FWeakPI.kp = FD_WEAK_PI_KP;
    pControlScheme->fluxControl.feedBackFW.FWeakPI.ki = FD_WEAK_PI_KI; 
    pControlScheme->fluxControl.feedBackFW.FWeakPI.kc = Q15(0.9999);
    pControlScheme->fluxControl.feedBackFW.FWeakPI.nkp = FD_WEAK_PI_KPSCALE;
    pControlScheme->fluxControl.feedBackFW.FWeakPI.nki = 0;
    pControlScheme->fluxControl.feedBackFW.FWeakPI.outMax = 0;
    pControlScheme->fluxControl.feedBackFW.FWeakPI.outMin = ID_REF_MIN;
    pControlScheme->fluxControl.feedBackFW.IdRefFiltConst = FD_WEAK_IDREF_FILT_CONST;
    pControlScheme->fluxControl.feedBackFW.IdRefMin = ID_REF_MIN;
    

    /* Output Initializations */
    pControlScheme->pwmPeriod = MC1_LOOPTIME_TCY;
    pControlScheme->pPWMDuty = pMCData->pPWMDuty;

    /* Initialize application structure */
    pMCData->MCAPP_ControlSchemeInit = MCAPP_FOCInit;
    pMCData->MCAPP_ControlStateMachine = MCAPP_FOCStateMachine;

    pMCData->MCAPP_InputsInit = MCAPP_MeasureCurrentInit;
    pMCData->MCAPP_MeasureOffset = MCAPP_MeasureCurrentOffset;
    pMCData->MCAPP_GetProcessedInputs = MCAPP_MeasureCurrentCalibrate;
    pMCData->MCAPP_IsOffsetMeasurementComplete = 
                                       MCAPP_MeasureCurrentOffsetStatus;
}

/**
* <B> Function: MCAPP_MC1LoadConfig (MC1APP_DATA_T *)  </B>
*
* @brief Function to reset variables used for current offset measurement.
*
* @param Pointer to the FOC data structure required for controlling motor 1.
* @return none.
* @example
* <CODE> MCAPP_MC1LoadConfig(&mcData); </CODE>
*
*/

void MCAPP_MC1LoadConfig(MC1APP_DATA_T *pMCData)
{   
	pMCData->MCAPP_LoadInit = MCAPP_GenericLoadInit;
    pMCData->MCAPP_LoadStateMachine = MCAPP_GenericLoadStateMachine;
    pMCData->MCAPP_IsLoadReadyToStart = MCAPP_IsGenericLoadReadyToStart;
    pMCData->MCAPP_IsLoadReadyToStop = MCAPP_IsGenericLoadReadyToStop;
    
    pMCData->MCAPP_LoadStartTransition = MCAPP_MC1LoadStartTransition;
    pMCData->MCAPP_LoadStopTransition = MCAPP_MC1LoadStopTransition;
}


void MCAPP_MC1LoadStartTransition(MCAPP_CONTROL_SCHEME_T *pControlScheme, 
                                    MCAPP_LOAD_T *pLoad)
{
    pControlScheme->focState =  FOC_RTR_LOCK; 
}

void MCAPP_MC1LoadStopTransition(MCAPP_CONTROL_SCHEME_T *pControlScheme, 
                                    MCAPP_LOAD_T *pLoad)
{
    pLoad->state = GENERIC_LOAD_STOP;
}

void MCAPP_MC1OutputConfig(MC1APP_DATA_T *pMCData)
{
    pMCData->HAL_PWMSetDutyCycles = HAL_MC1PWMSetDutyCycles;
    pMCData->HAL_PWMEnableOutputs = HAL_MC1PWMEnableOutputs;
    pMCData->HAL_PWMDisableOutputs = HAL_MC1PWMDisableOutputs;
    pMCData->MCAPP_HALSetVoltageVector = HAL_MC1SetVoltageVector;
}