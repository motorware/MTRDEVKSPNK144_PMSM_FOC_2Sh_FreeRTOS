/*******************************************************************************
*
* (c) Copyright 2019 NXP Semiconductors
*
****************************************************************************//*!
*
* @file     main.c
*
* @brief    Main thread module source file.
*
*******************************************************************************/
/*******************************************************************************
*
*******************************************************************************/
#include "Cpu.h"
#include "clockMan1.h"
#include "pin_mux.h"
#include "FreeRTOS.h"
#include "device_registers.h"

volatile int exit_code = 0;
/* User includes (#include below this line is not maintained by Processor Expert) */

#include <stdint.h>
#include <stdbool.h>
#include "main.h"
#include "task.h"

#include "pmsm_main.h"
#include "peripherals_config.h"

/*******************************************************************************
*
*******************************************************************************/
BaseType_t ret;
TaskHandle_t xRemoteTaskHandle;

/*******************************************************************************
*
*******************************************************************************/
int main(void)
{
    BaseType_t ret;

    /*** Processor Expert internal initialization. DON'T REMOVE THIS CODE!!! ***/
    #ifdef PEX_RTOS_INIT
    PEX_RTOS_INIT();                   /* Initialization of the selected RTOS. Macro is defined by the RTOS component. */
    #endif
    /*** End of Processor Expert internal initialization.                    ***/

    /* Enable cache */
    McuCacheConfig();
    /* Initialize and configure clocks */
    CLOCK_SYS_Init(g_clockManConfigsArr, CLOCK_MANAGER_CONFIG_CNT, g_clockManCallbacksArr, CLOCK_MANAGER_CALLBACK_CNT);
    CLOCK_SYS_UpdateConfiguration(0U, CLOCK_MANAGER_POLICY_AGREEMENT);

    /* Initialize pins */
    PINS_DRV_Init(NUM_OF_CONFIGURED_PINS, g_pin_mux_InitConfigArr);

    /* Create Init Task */
    ret = xTaskCreate(
        pmsm_init,
        "pmsm_init_tsk",
        256u,
        NULL,
        4u,
        NULL
    );
    configASSERT(ret == pdPASS);

    /* Start FreeRTOS scheduler */
    vTaskStartScheduler();

  /*** Don't write any code pass this line, or it will be deleted during code generation. ***/
  /*** RTOS startup code. Macro PEX_RTOS_START is defined by the RTOS component. DON'T MODIFY THIS CODE!!! ***/
  #ifdef PEX_RTOS_START
    PEX_RTOS_START();                  /* Startup of the selected RTOS. Macro is defined by the RTOS component. */
  #endif
  /*** End of RTOS startup code.  ***/
  /*** Processor Expert end of main routine. DON'T MODIFY THIS CODE!!! ***/
  for(;;) {
    if(exit_code != 0) {
      break;
    }
  }
  return exit_code;
  /*** Processor Expert end of main routine. DON'T WRITE CODE BELOW!!! ***/
}
