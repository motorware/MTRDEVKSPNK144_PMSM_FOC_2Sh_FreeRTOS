/*******************************************************************************
*
* Copyright 2006-2015 Freescale Semiconductor, Inc.
* Copyright 2016-2017 NXP
*
****************************************************************************//*!
*
* @file     actuate_s32k.c
*
* @date     March-28-2017
*
* @brief    Header file for actuator module
*
*******************************************************************************/
/******************************************************************************
| Includes
-----------------------------------------------------------------------------*/
#include "S32K144.h"
#include "actuate_s32k.h"
#include "peripherals_config.h"
#include "gflib.h"
#include "gmclib.h"
#include "gdflib.h"

/******************************************************************************
| External declarations
-----------------------------------------------------------------------------*/

/******************************************************************************
| Defines and macros            (scope: module-local)
-----------------------------------------------------------------------------*/

/******************************************************************************
| Typedefs and structures       (scope: module-local)
-----------------------------------------------------------------------------*/

/******************************************************************************
| Global variable definitions   (scope: module-exported)
-----------------------------------------------------------------------------*/

/******************************************************************************
| Global variable definitions   (scope: module-local)
-----------------------------------------------------------------------------*/

/******************************************************************************
| Function prototypes           (scope: module-local)
-----------------------------------------------------------------------------*/

/******************************************************************************
| Function implementations      (scope: module-local)
-----------------------------------------------------------------------------*/

/******************************************************************************
| Function implementations      (scope: module-exported)
-----------------------------------------------------------------------------*/

/**************************************************************************//*!
@brief Unmask PWM output and set 50% dytucyle 

@param[in,out]  

@return
******************************************************************************/
tBool ACTUATE_EnableOutput(void)
{
	SWLIBS_3Syst_FLT fltpwm;

	tBool statePWM;

	// Apply 0.5 duty cycle
	fltpwm.fltArg1 = 0.5F;
	fltpwm.fltArg2 = 0.5F;
	fltpwm.fltArg3 = 0.5F;
	
	statePWM = ACTUATE_SetDutycycle(&fltpwm);

    // Enable PWM
	FTM_DRV_MaskOutputChannels(INST_FLEXTIMER_PWM3, 0x0, true);

	return(statePWM);
}


/**************************************************************************//*!
@brief Mask PWM output and set 50% dytucyle 

@param[in,out]  

@return
******************************************************************************/
tBool ACTUATE_DisableOutput(void)
{
	SWLIBS_3Syst_FLT fltpwm;

	tBool statePWM;

	// Apply 0.5 duty cycle
	fltpwm.fltArg1 = 0.5F;
	fltpwm.fltArg2 = 0.5F;
	fltpwm.fltArg3 = 0.5F;
	
	statePWM = ACTUATE_SetDutycycle(&fltpwm);

    /* Disable PWM */
	FTM_DRV_MaskOutputChannels(INST_FLEXTIMER_PWM3, 0x3F, true);

	return(statePWM);
}

/**************************************************************************//*!
@brief Set PWM dytycyle, the dutycycle will by updated on next reload event

@param[in,out]  

@return
******************************************************************************/
tBool ACTUATE_SetDutycycle(SWLIBS_3Syst_FLT *fltpwm)
{
	tBool statePwm = true;

	FTM3->CONTROLS[0].CnV = MLIB_Mul_FLT(fltpwm->fltArg1, 2000.0F);
	FTM3->CONTROLS[2].CnV = MLIB_Mul_FLT(fltpwm->fltArg2, 2000.0F);
	FTM3->CONTROLS[4].CnV = MLIB_Mul_FLT(fltpwm->fltArg3, 2000.0F);
	FTM3->SYNC |= FTM_SYNC_SWSYNC_MASK;

	statePwm = false;

	return(statePwm);
}

/* End of file */
