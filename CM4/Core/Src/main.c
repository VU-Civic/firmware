#ifdef CORE_CM4

// Header Inclusions ---------------------------------------------------------------------------------------------------

#include "main.h"
#include "ai.h"
#include "audio.h"
#include "cellular.h"
#include "gps.h"
#include "imu.h"
#include "system.h"
#include "usb.h"


// Main Application Function -------------------------------------------------------------------------------------------

int main(void)
{
   // Enable the HSEM peripheral clock and HSEM interrupts
   SET_BIT(RCC->AHB4ENR, RCC_AHB4ENR_HSEMEN);
   (void)READ_BIT(RCC->AHB4ENR, RCC_AHB4ENR_HSEMEN);
   SET_BIT(HSEM_COMMON->IER, 1U);
   __SEV(); __WFE();

   // Put this core into STOP mode until the CM7 core notifies us
   MODIFY_REG(PWR->CR1, PWR_CR1_LPDS, PWR_MAINREGULATOR_ON);
   CLEAR_BIT(PWR->CPU2CR, PWR_CPU2CR_PDDS_D2);
   SET_BIT(SCB->SCR, SCB_SCR_SLEEPDEEP_Msk);
   __DSB(); __ISB(); __WFE();
   CLEAR_BIT(SCB->SCR, SCB_SCR_SLEEPDEEP_Msk);
   SET_BIT(HSEM->C2ICR, 1U);

   // Configure the CM4 instruction cache through the ART accelerator
   SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_ARTEN);
   (void)READ_BIT(RCC->AHB1ENR, RCC_AHB1ENR_ARTEN);
   MODIFY_REG(ART->CTR, ART_CTR_PCACHEADDR, ((0x08100000UL >> 12U) & 0x000FFF00UL));
   SET_BIT(ART->CTR, ART_CTR_EN);

   // Set the NVIC interrupt group priority
   NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

   // Configure a 1ms SysTick interrupt timer which will be disabled later
   SystemCoreClockUpdate();
   SysTick_Config(SystemCoreClock / 1000UL);
   NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), TICK_INT_PRIORITY, 0U));
   uwTickPrio = TICK_INT_PRIORITY;

   // Wait for at least 1.5s before powering on the cellular modem
   HAL_Delay(1600);
   cell_power_on();

   // Initialize user peripherals
   usb_init();
   audio_init();
   imu_init();
   gps_init();
   ai_comms_init();
   cell_init();
   opusenc_init();

   // Start user peripherals
   imu_start();
   ai_comms_start();
   cpu_init();

   // Loop forever
   uint8_t processing_occurred;
   while (1)
   {
      // Carry out slow processing operations
      processing_occurred = audio_process_new_data(CELL_AUDIO_NO_TRANSMIT); // TODO: Call with correct parameter
      processing_occurred = cell_update_state() || processing_occurred;

      // Put the CPU to sleep if no processing occurred
      if (!processing_occurred)
         cpu_sleep();
   }
   return 0;
}

#endif  // #ifdef CORE_CM4
