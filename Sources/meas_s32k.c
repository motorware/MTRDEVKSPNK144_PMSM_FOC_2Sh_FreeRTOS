/*******************************************************************************
*
* Copyright 2006-2015 Freescale Semiconductor, Inc.
* Copyright 2016-2017 NXP
*
****************************************************************************//*!
*
* @file     meas_s32k.c
*
* @date     March-28-2017
*
* @brief    Header file for measurement module
*
*******************************************************************************/
/******************************************************************************
| Includes
-----------------------------------------------------------------------------*/
#include "meas_s32k.h"

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
@brief			Measurement module software initialization

@param[in,out]  *ptr

@return			# true - when Module initialization ended successfully
            	# false - when Module initialization is ongoing, or error occurred

@details		Clear variables needed for both calibration as well as run time
				measurement.

@note			It is not intended to be executed when application is in run mode.

@warning
******************************************************************************/
tBool MEAS_Clear(measModule_t *ptr)
{
    ptr->measured.fltPhA.filt   = 0.0F;
    ptr->measured.fltPhA.raw    = 0.0F;
    ptr->measured.fltPhB.filt   = 0.0F;
    ptr->measured.fltPhB.raw    = 0.0F;
    ptr->measured.fltPhC.filt   = 0.0F;
    ptr->measured.fltPhC.raw    = 0.0F;
    ptr->measured.fltUdcb.filt  = 0.0F;
    ptr->measured.fltUdcb.raw   = 0.0F;
    ptr->measured.fltTemp.filt  = 0.0F;
    ptr->measured.fltTemp.raw   = 0.0F;

    ptr->offset.fltPhA.fltOffset = MLIB_Div(I_MAX, 2);
	ptr->offset.fltPhB.fltOffset = MLIB_Div(I_MAX, 2);
	ptr->offset.fltPhC.fltOffset = MLIB_Div(I_MAX, 2);
    
    ptr->flag.R             	= 0;

    ptr->param.u16CalibSamples  = 0;
    ptr->flag.B.calibInitDone   = 0;
    ptr->flag.B.calibDone       = 0;

    return 1;
}

/**************************************************************************//*!
@brief      	3Phase current measurement software calibration routine.

@param[in,out]  *ptr    	Pointer to structure of measurement module variables and
                        	parameters
@param[in]      svmSector	Space Vector Modulation Sector

@return     	# true - when Calibration ended successfully
            	# false - when Calibration is ongoing

@details    	This function performs offset calibration for 3 phase current measurement
				during the calibration phase of the application. It is not intended to be
				executed when application is in run mode.

@warning
******************************************************************************/
tBool MEAS_CalibCurrentSense(measModule_t *ptr)
{
    if (!(ptr->flag.B.calibInitDone))
    {
        ptr->calibCntr = 1<< (ptr->param.u16CalibSamples + 4); // +4 in order to accommodate settling time of the filter

        ptr->measured.fltPhA.filt   = 0x0;
        ptr->measured.fltPhB.filt   = 0x0;
        ptr->measured.fltPhC.filt   = 0x0;

        ptr->offset.fltPhA.filtParam.fltAcc		= MLIB_Div(I_MAX, 2);
        ptr->offset.fltPhB.filtParam.fltAcc		= MLIB_Div(I_MAX, 2);
        ptr->offset.fltPhC.filtParam.fltAcc		= MLIB_Div(I_MAX, 2);

        ptr->flag.B.calibDone       = 0;
        ptr->flag.B.calibInitDone   = 1;
    }

    if (!(ptr->flag.B.calibDone))
    {
    	/* --------------------------------------------------------------
         * Phase A - DC offset data filtering using MA recursive filter
         * ------------------------------------------------------------ */
    	ptr->offset.fltPhA.fltOffset  = GDFLIB_FilterMA(ptr->measured.fltPhA.raw, &ptr->offset.fltPhA.filtParam);

        /* --------------------------------------------------------------
         * Phase B - DC offset data filtering using MA recursive filter
         * ------------------------------------------------------------ */
    	ptr->offset.fltPhB.fltOffset  = GDFLIB_FilterMA(ptr->measured.fltPhB.raw, &ptr->offset.fltPhB.filtParam);

        /* --------------------------------------------------------------
         * Phase C - DC offset data filtering using MA recursive filter
         * ------------------------------------------------------------ */
    	ptr->offset.fltPhC.fltOffset  = GDFLIB_FilterMA(ptr->measured.fltPhC.raw, &ptr->offset.fltPhC.filtParam);

        if ((--ptr->calibCntr)<=0)
        {
        	ptr->flag.B.calibDone       = 1;    // end of DC offset calibration
        }
    }
    return (ptr->flag.B.calibDone);
}

/**************************************************************************//*!
@brief      	3-phase current measurement reading.

@param[in,out]  *ptr    Pointer to structure of module variables and
                        parameters

@return    		# true - when measurement ended successfully
            	# false - when measurement is ongoing, or error occurred.

@details    	This function performs measurement of three phase currents from
            	two shunt resistors. Because a non-zero length PWM pulse width
            	is required for successful current sample, this approach can not
            	be utilized up to full PWM dutycycle.

@note

@warning
******************************************************************************/
tBool MEAS_Get3PhCurrent(measModule_t *ptr, SWLIBS_3Syst_FLT *i)
{
	uint16_t PhaseA_Current;
	uint16_t PhaseB_Current;

	// Read ADC0_CH0 value - PhaseA Current
	PhaseA_Current = ((ADC0->R[0]) & ADC_R_D_MASK) >> ADC_R_D_SHIFT;
	// Read ADC1_CH2 value - PhaseB Current
	PhaseB_Current = ((ADC1->R[1]) & ADC_R_D_MASK) >> ADC_R_D_SHIFT;

	ptr->measured.fltPhA.raw = MLIB_Mul((MLIB_Div((tFloat)(PhaseA_Current & 0x00000FFF), (tFloat)0x00000FFF)), I_MAX);
	ptr->measured.fltPhB.raw = MLIB_Mul((MLIB_Div((tFloat)(PhaseB_Current & 0x00000FFF), (tFloat)0x00000FFF)), I_MAX);

	i->fltArg1 = MLIB_Mul(MLIB_Sub(ptr->offset.fltPhA.fltOffset,ptr->measured.fltPhA.raw), 2.0F);
	i->fltArg2 = MLIB_Mul(MLIB_Sub(ptr->offset.fltPhB.fltOffset,ptr->measured.fltPhB.raw), 2.0F);
	i->fltArg3 = MLIB_Neg_FLT(MLIB_Add(i->fltArg1,i->fltArg2));

	return(1);
}

/**************************************************************************//*!
@brief      	DCB Voltage measurement routine.

@param[in,out]  *ptr    Pointer to structure of module variables and
                        parameters

@return     	# true - when measurement ended successfully
            	# false - when measurement is ongoing, or error occurred.

@details    	This function performs measurement of DCBus Voltage.
            
@note

@warning
******************************************************************************/
tBool MEAS_GetUdcVoltage(measModule_t *ptr, GDFLIB_FILTER_MA_T *uDcbFilter)
{
	uint16_t DCBus_Voltage;

	//	Read ADC1_CH0 value - DC Bus Voltage
	DCBus_Voltage = (ADC1->R[0] & ADC_R_D_MASK) >> ADC_R_D_SHIFT;

	ptr->measured.fltUdcb.raw = MLIB_Mul(((tFloat)MLIB_Div((tFloat)(DCBus_Voltage & 0x00000FFF), (tFloat)0x00000FFF)), U_DCB_MAX);
	ptr->measured.fltUdcb.filt  = GDFLIB_FilterMA(ptr->measured.fltUdcb.raw, uDcbFilter);

	return(1);
}

/* End of file */
