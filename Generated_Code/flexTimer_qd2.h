/* ###################################################################
**     This component module is generated by Processor Expert. Do not modify it.
**     Filename    : flexTimer_qd2.h
**     Project     : MCSPTE1AK144_PMSM_FOC_2Sh_FreeRTOS
**     Processor   : S32K144_100
**     Component   : ftm_qd
**     Version     : Component SDK_S32K1xx_15, Driver 01.00, CPU db: 3.00.000
**     Repository  : SDK_S32K1xx_15
**     Compiler    : GNU C Compiler
**     Date/Time   : 2020-04-29, 11:30, # CodeGen: 1
**     Contents    :
**         FTM_DRV_Init                - status_t FTM_DRV_Init(uint32_t instance,const ftm_user_config_t * info,...
**         FTM_DRV_Deinit              - status_t FTM_DRV_Deinit(uint32_t instance);
**         FTM_DRV_GetDefaultConfig    - void FTM_DRV_GetDefaultConfig(ftm_user_config_t * const config);
**         FTM_DRV_QuadDecodeStart     - status_t FTM_DRV_QuadDecodeStart(uint32_t instance,const...
**         FTM_DRV_QuadDecodeStop      - status_t FTM_DRV_QuadDecodeStop(uint32_t instance);
**         FTM_DRV_QuadGetState        - ftm_quad_decoder_state_t FTM_DRV_QuadGetState(uint32_t instance);
**         FTM_QD_DRV_GetDefaultConfig - void FTM_QD_DRV_GetDefaultConfig(ftm_quad_decode_config_t * const config);
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
** @file flexTimer_qd2.h
** @version 01.00
*/         
/*!
**  @addtogroup flexTimer_qd2_module flexTimer_qd2 module documentation
**  @{
*/         
#ifndef flexTimer_qd2_H
#define flexTimer_qd2_H

/* MODULE flexTimer_qd2.
 *
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 2.5, Global macro not referenced.
 * The global macro will be used in function call of the module.
 */

/* Include inherited beans */
#include "clockMan1.h"
#include "Cpu.h"
/*! @brief Device instance number */
#define INST_FLEXTIMER_QD2 2U


extern ftm_quad_decode_config_t flexTimer_qd2_QuadDecoderConfig;

/* Global configuration of flexTimer_qd2 */
extern ftm_user_config_t  flexTimer_qd2_InitConfig;

#endif
/* ifndef flexTimer_qd2_H */
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