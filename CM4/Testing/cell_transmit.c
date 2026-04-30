#ifdef CORE_CM4

// Header Inclusions ---------------------------------------------------------------------------------------------------

#include "main.h"
#include "ai.h"
#include "audio.h"
#include "cellular.h"
#include "gps.h"
#include "imu.h"
#include "shot_detector.h"
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

   // Read all non-volatile configuration settings
   chip_read_config();

   // Initialize user peripherals
   usb_init();
   audio_init();
   imu_init();
   gps_init();
   cell_init();
   ai_comms_init();
   opusenc_init();
   shot_detector_init();

   // Start user peripherals
   imu_start();
   ai_comms_start();
   audio_start();
   cpu_init();

   // Wait for AI communications to stabilize
   while (!ai_comms_finalize())
   {
      audio_process_new_data(0);
      cell_update_state();
      chip_update_config();
   }

   // Set up a fake alert message for testing
   uint32_t transmit_countdown = 600;
   alert_message_t alert_message = { .device_id = device_info.device_id, .num_events = 2 };
   alert_message.events[0].timestamp = 1777220538.234;
   alert_message.events[0].confidence = alert_message.events[0].magnitude = 0.9f;
   alert_message.events[0].angle_of_arrival[0] = 0.123f;
   alert_message.events[0].angle_of_arrival[1] = 0.03f;
   alert_message.events[0].angle_of_arrival[2] = 0.93f;
   alert_message.events[1] = alert_message.events[0];
   alert_message.events[1].timestamp = 1777220538.827;
   alert_message.events[1].confidence = alert_message.events[1].magnitude = 0.34f;

   // Loop forever
   while (1)
   {
      // Carry out slow processing operations
      ai_comms_validate();
      const uint8_t clip_id = audio_process_new_data(transmit_countdown > 1800);
      if (!--transmit_countdown)
      {
         transmit_countdown = 1818;
         alert_message.audio_clip_id = clip_id;
         cell_transmit_alert(&alert_message);
      }
      cell_update_state();
      chip_update_config();

      // Put the CPU to sleep if nothing left to process
      __disable_irq();
      if (!audio_new_data_available() && !cell_pending_events())
         cpu_sleep();
      __enable_irq();
   }
   return 0;
}

#endif  // #ifdef CORE_CM4
