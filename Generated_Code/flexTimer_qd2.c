/* ###################################################################
**     This component module is generated by Processor Expert. Do not modify it.
**     Filename    : flexTimer_qd2.c
**     Project     : MCSPTE1AK144_PMSM_FOC_2Sh_FreeRTOS
**     Processor   : S32K144_100
**     Component   : ftm_qd
**     Version     : Component SDK_S32K1xx_15, Driver 01.00, CPU db: 3.00.000
**     Repository  : SDK_S32K1xx_15
**     Compiler    : GNU C Compiler
**     Date/Time   : 2020-04-29, 11:30, # CodeGen: 1
**
**     Copyright 1997 - 2015 Freescale Semiconductor, Inc. 
**     Copyright 2016-2017 NXP 
**     All Rights Reserved.
**     
**     THIS SOFTWARE IS PROVIDED BY NXP "AS IS" AND ANY EXPRESSED OR
**     IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
**     OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
**     IN NO EVENT SHALL NXP OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
**     INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
**     (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
**     SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
**     HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
**     STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
**     IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
**     THE POSSIBILITY OF SUCH DAMAGE.
** ###################################################################*/
/*!
** @file flexTimer_qd2.c
** @version 01.00
*/         
/*!
**  @addtogroup flexTimer_qd2_module flexTimer_qd2 module documentation
**  @{
*/         

/* Module flexTimer_qd2.
 *
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 8.4, external symbol defined without a prior
 * declaration.
 * The symbols are declared in the driver header as external; the header is not included 
 * by this file.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.7, External variable could be made static.
 * The external variable will be used in other source files in application code.
 */

#include "flexTimer_qd2.h"



ftm_quad_decode_config_t flexTimer_qd2_QuadDecoderConfig =
{
    FTM_QUAD_PHASE_ENCODE,
    0U,
    2047U,
    {
        false,
        0U,
        FTM_QUAD_PHASE_NORMAL, /* Phase A polarity */
    },
    {
        false,
        0U,
        FTM_QUAD_PHASE_NORMAL, /* Phase B polarity */
    }
};

/* Global configuration of flexTimer_qd2 */
ftm_user_config_t  flexTimer_qd2_InitConfig =
{
    {
        true,   /* Software trigger state */
        false,  /* Hardware trigger 1 state */
        false,  /* Hardware trigger 2 state */
        false,  /* Hardware trigger 3 state */
        false, /* Max loading point state */
        false, /* Min loading point state */
        FTM_SYSTEM_CLOCK, /* Update mode for INVCTRL register */
        FTM_SYSTEM_CLOCK, /* Update mode for SWOCTRL register */
        FTM_SYSTEM_CLOCK, /* Update mode for OUTMASK register */
        FTM_SYSTEM_CLOCK, /* Update mode for CNTIN register */
        false, /* Automatic clear of the trigger*/
        FTM_UPDATE_NOW, /* Synchronization point */
    },
     FTM_MODE_QUADRATURE_DECODER, /*!< Mode of operation for FTM */
     FTM_CLOCK_DIVID_BY_1, /* FTM clock prescaler */
     FTM_CLOCK_SOURCE_EXTERNALCLK,   /* FTM clock source */
     FTM_BDM_MODE_11, /* FTM debug mode */
     false, /* Interrupt state */
     false /* Initialization trigger */
};

/* END flexTimer_qd2. */

/*!
** @}
*/
/*
** ###################################################################
**
**     This file was created by Processor Expert 10.1 [05.21]
**     for the Freescale S32K series of microcontrollers.
**
** ###################################################################
*/

