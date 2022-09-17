/*******************************************************************************
*
* Copyright 2006-2015 Freescale Semiconductor, Inc.
* Copyright 2016-2017 NXP
*
****************************************************************************//*!
*
* @file     meas_s32k.h
*
* @date     March-28-2017
*
* @brief    Header file for measurement module
*
*******************************************************************************/
#ifndef _MEAS_S32K_H_
#define _MEAS_S32K_H_

#include "PMSM_appconfig.h"
#include "s32k144.h"
#include "peripherals_config.h"
#include "gflib.h"
#include "gmclib.h"
#include "gdflib.h"

/******************************************************************************
| Defines and macros            (scope: module-local)
-----------------------------------------------------------------------------*/


/******************************************************************************
| Typedefs and structures       (scope: module-local)
-----------------------------------------------------------------------------*/

typedef struct
{
    tFloat    raw;   /*! raw value */
    tFloat    filt;  /*! filtered value */
}meas_t;

/*------------------------------------------------------------------------*//*!
@brief  Structure containing measured raw values
*//*-------------------------------------------------------------------------*/
typedef struct
{
	meas_t    fltPhA;     // DC offset measured on phase A current
	meas_t    fltPhB;     // DC offset measured on phase B current
	meas_t    fltPhC;     // DC offset measured on phase C current
	meas_t    fltUdcb;    // DC offset measured on DC bus voltage
	meas_t    fltTemp;    // DC offset measured on temperature
}measResult_t;

typedef struct
{
    tFloat    				fltOffset;   /*! raw value */
    GDFLIB_FILTER_MA_T_FLT	filtParam;	 /*! filter parameters */
}offsetBasic_t;


/*------------------------------------------------------------------------*//*!
@brief  Structure containing variables for software DC offset calibration.
*//*-------------------------------------------------------------------------*/
typedef struct
{
    offsetBasic_t    fltPhA;         // DC offset measured on phase A current
    offsetBasic_t    fltPhB;         // DC offset measured on phase B current
    offsetBasic_t    fltPhC;         // DC offset measured on phase C current
    offsetBasic_t    fltUdcb;        // DC offset measured on DC bus voltage
    offsetBasic_t    fltTemp;        // DC offset measured on temperature
}offset_t;


/*------------------------------------------------------------------------*//*!
@brief  Structure containing variables to configure Calibration on the application
        level.
*//*-------------------------------------------------------------------------*/
typedef struct
{
	tU16     u16CalibSamples; // Number of samples taken for calibration
}calibParam_t;

/*------------------------------------------------------------------------*//*!
@brief  Union containing module operation flags.
*//*-------------------------------------------------------------------------*/
typedef union
{
    tU16 R;
    struct {
        tU16               :14;// RESERVED
        tU16 calibDone     :1; // DC offset calibration done
        tU16 calibInitDone :1; // initial setup for DC offset calibration done
    } B;
}calibFlags_t;

/*------------------------------------------------------------------------*//*!
@brief  Module structure containing measurement related variables.
*//*-------------------------------------------------------------------------*/
typedef struct
{
    measResult_t  		measured;
    offset_t     		offset;
    calibParam_t      	param;
    calibFlags_t      	flag;
	tU16 				calibCntr;
}measModule_t;

/******************************************************************************
| Exported Variables
-----------------------------------------------------------------------------*/

/******************************************************************************
| Exported function prototypes
-----------------------------------------------------------------------------*/
extern tBool MEAS_Clear(measModule_t *ptr);
extern tBool MEAS_CalibCurrentSense(measModule_t *ptr);
extern tBool MEAS_Get3PhCurrent(measModule_t *ptr, SWLIBS_3Syst_FLT *i);
extern tBool MEAS_GetUdcVoltage(measModule_t *ptr, GDFLIB_FILTER_MA_T *uDcbFilter);

/******************************************************************************
| Inline functions
-----------------------------------------------------------------------------*/

#endif /* _MEAS_S32K_H_ */
