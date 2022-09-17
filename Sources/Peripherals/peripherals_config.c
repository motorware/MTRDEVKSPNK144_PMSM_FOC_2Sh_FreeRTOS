/*******************************************************************************
*
* Copyright 2006-2015 Freescale Semiconductor, Inc.
* Copyright 2016-2017 NXP
*
****************************************************************************//*!
*
* @file     peripherals_config.c
*
* @date     March-28-2017
*
* @brief    MCU Peripherals Configuration
*
*******************************************************************************/
/*******************************************************************************
* Includes
*******************************************************************************/
#include "peripherals_config.h"
#include "ftm_hw_access.h"
ftm_state_t statePwm;

/*******************************************************************************
*
* Function: 	void McuIntConfig(void)
*
* Description:  Enables an interrupt for a given IRQ number.
* 				For more details see configuration in Processor Expert.
*
*******************************************************************************/
void McuIntConfig(void)
{
    INT_SYS_EnableIRQ(PDB0_IRQn);						// Enable PDB0 interrupt
    INT_SYS_EnableIRQ(PDB1_IRQn);						// Enable PDB1 interrupt
    INT_SYS_EnableIRQ(ADC0_IRQn);						// Enable ADC0 interrupt
    INT_SYS_EnableIRQ(ADC1_IRQn);						// Enable ADC1 interrupt
    INT_SYS_EnableIRQ(PORTE_IRQn);						// Enable PORTE interrupt

    INT_SYS_SetPriority(ADC1_IRQn, 5);
    INT_SYS_SetPriority(LPSPI0_IRQn, 6);
}

/*******************************************************************************
*
* Function: 	void McuSimConfig(void)
*
* Description:  This function configures SIM module
*
*******************************************************************************/
void McuSimConfig(void)
{
	/* Enable interleaved mode for ADC0_SE5 and ADC1_SE15 channels on PTB1 pin */
	//SIM_HAL_SetAdcInterleaveSel(SIM, 0b0010);
	SIM->CHIPCTL |= SIM_CHIPCTL_ADC_INTERLEAVE_EN(0b0010);

}

/*******************************************************************************
*
* Function: 	void McuTrigmuxConfig(void)
*
* Description:  This function configures the user defined target modules
* 				with the corresponding source triggers. For more details see
* 				configuration in Processor Expert.
*
*******************************************************************************/
void McuTrigmuxConfig(void)
{
	/* TRGMUX module initialization */
	TRGMUX_DRV_Init(INST_TRGMUX1, &trgmux1_InitConfig0);
}

/*******************************************************************************
*
* Function: 	void McuPinsConfig(void)
*
* Description:  This function configures MCU pins with the options provided
* 				in the provided structure. For more details see configuration
* 				in Processor Expert.
*
*******************************************************************************/

void McuPinsConfig(void)
{
	/* Enable interrupt when rising edge is detected on PTE10 */
	PINS_DRV_SetPinIntSel(PORTE, 10u, PORT_INT_RISING_EDGE);
}

/*******************************************************************************
*
* Function: 	void McuLpuartConfig(void)
*
* Description:  This function configures LPUART module. LPUART is used
* 				as a communication interface between MCU and FreeMASTER.
* 				For more details see configuration in Processor Expert.
*
*******************************************************************************/
void McuLpuartConfig(void)
{
	/* LPUART module initialization */
	LPUART_DRV_Init(INST_LPUART1, &lpuart1_State, &lpuart1_InitConfig0);
}

/*******************************************************************************
*
* Function: 	void McuAdcConfig(void)
*
* Description:  This function configures ADC0 and ADC1 module.
* 				For more details see configuration in Processor Expert.
*
*******************************************************************************/
void McuAdcConfig(void)
{
	/* ADC0 module initialization */
    ADC_DRV_ConfigConverter(INST_ADCONV0, &adConv0_ConvConfig0);
	/* ADC1 module initialization */
    ADC_DRV_ConfigConverter(INST_ADCONV1, &adConv1_ConvConfig0);

    /* AD4 input channel is used for PhaseA stator current sensing */
    ADC_DRV_ConfigChan(INST_ADCONV0, 0, &adConv0_ChnConfig0);
    /* AD7 input channel is used for DC bus voltage sensing */
    ADC_DRV_ConfigChan(INST_ADCONV1, 0, &adConv1_ChnConfig0);
    /* AD15 input channel is used for PhaseB stator current sensing */
    ADC_DRV_ConfigChan(INST_ADCONV1, 1, &adConv1_ChnConfig1);
}

/*******************************************************************************
*
* Function: 	void McuPdbConfig(void)
*
* Description:  This function configures PDB0 and PDB1 module.
* 				For more details see configuration in Processor Expert.
*
*******************************************************************************/
void McuPdbConfig(void)
{
	/* PDB0 module initialization */
	PDB_DRV_Init(INST_PDB0, &pdb0_InitConfig0);
	/* PDB1 module initialization */
	PDB_DRV_Init(INST_PDB1, &pdb1_InitConfig0);

	/* PDB0 CH0 pre-trigger0 initialization */
	PDB_DRV_ConfigAdcPreTrigger(INST_PDB0, 0, &pdb0_AdcTrigInitConfig0);
	/* PDB1 CH0 pre-trigger0 initialization */
	PDB_DRV_ConfigAdcPreTrigger(INST_PDB1, 0, &pdb1_AdcTrigInitConfig0);
	/* PDB1 CH0 pre-trigger1 initialization */
	PDB_DRV_ConfigAdcPreTrigger(INST_PDB1, 0, &pdb1_AdcTrigInitConfig1);

	/* Set PDB0 modulus value */
	PDB_DRV_SetTimerModulusValue(INST_PDB0, 5000);
	/* Set PDB1 modulus value */
	PDB_DRV_SetTimerModulusValue(INST_PDB1, 5000);

	/* PDB0 CH0 pre-trigger0 delay set to sense PhaseA stator current in the middle of the PWM cycle */
	PDB_DRV_SetAdcPreTriggerDelayValue(INST_PDB0, 0, 0, 2000);
    /* PDB1 CH0 pre-trigger0 delay set to sense DC bus voltage at the beginning of the PWM cycle */
    PDB_DRV_SetAdcPreTriggerDelayValue(INST_PDB1, 0, 0, 0);
	/* PDB1 CH0 pre-trigger1 delay set to sense PhaseB stator current in the middle of the PWM cycle */
	PDB_DRV_SetAdcPreTriggerDelayValue(INST_PDB1, 0, 1, 2000);

	/* Set PDB1 interrupt delay value */
	PDB_DRV_SetValueForTimerInterrupt(INST_PDB1, 4999);

	// enable PDB before LDOK
	PDB_DRV_Enable(INST_PDB0);
	// enable PDB before LDOK
	PDB_DRV_Enable(INST_PDB1);

	/* Load PDB0 configuration */
	PDB_DRV_LoadValuesCmd(INST_PDB0);
	/* Load PDB1 configuration */
	PDB_DRV_LoadValuesCmd(INST_PDB1);
}

/*******************************************************************************
*
* Function: 	void McuFtmConfig(void)
*
* Description:  This function configures FTM3 and FTM2 modules.
* 				FTM3 is configured to work as a complementary
* 				6-channel center-aligned PWM generator.
* 				FTM2 is configured to work in quadrature decoder mode
* 				to process Phase A and Phase B Encoder signals.
* 				For more details see configuration in Processor Expert.
*
* Note:         FTM3 even channels have inverted polarity due to the inverted
* 				high-side logic of the MC34GD3000.
*
*******************************************************************************/
void McuFtmConfig(void)
{
	ftm_state_t ftm2State;

	/* FTM2 module initialized to work in quadrature decoder mode, specifically PhaseA and PhaseB mode */
	FTM_DRV_Init(INST_FLEXTIMER_QD2, &flexTimer_qd2_InitConfig, &ftm2State);

	/* FTM3 module initialized as PWM signals generator */
	FTM_DRV_Init(INST_FLEXTIMER_PWM3, &flexTimer_pwm3_InitConfig, &statePwm);

	/* FTM3 module PWM initialization */
	FTM_DRV_InitPwm(INST_FLEXTIMER_PWM3, &flexTimer_pwm3_PwmConfig);

	/* Mask all FTM3 channels to disable PWM output */
	FTM_DRV_MaskOutputChannels(INST_FLEXTIMER_PWM3, 0x3F, true);
}

/*******************************************************************************
*
* Function:     void McuCacheConfig(void)
*
* Description:  This function enables Cache memory
*
*******************************************************************************/
void McuCacheConfig(void)
{
    // Enable Cache !
    // Flush and enable I cache and write buffer
    LMEM->PCCCR = LMEM_PCCCR_ENCACHE_MASK | LMEM_PCCCR_INVW1_MASK
                | LMEM_PCCCR_INVW0_MASK   | LMEM_PCCCR_GO_MASK;
}
