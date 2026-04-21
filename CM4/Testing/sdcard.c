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


// SD Card Application Defaults ----------------------------------------------------------------------------------------

#define DEVICE_STATUS_UPDATE_INTERVAL_MINUTES                  5
#define STORAGE_AUDIO_CLIP_MIN_SECONDS                         60
#define STORAGE_PROBABILITY_THRESHOLD                          0


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

   // Override device configuration settings with the desired application values
   chip_read_config();
   device_info.device_config.audio_clip_length_seconds = STORAGE_AUDIO_CLIP_MIN_SECONDS;
   device_info.device_config.device_status_transmission_interval_minutes = DEVICE_STATUS_UPDATE_INTERVAL_MINUTES;
   device_info.device_config.storage_classification_threshold = STORAGE_PROBABILITY_THRESHOLD;

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
      audio_process_new_data(CELL_AUDIO_NO_TRANSMIT);
      cell_update_state();
   }

   // Loop forever
   while (1)
   {
      // Carry out slow processing operations
      ai_comms_validate();
      const uint8_t clip_id = audio_process_new_data(CELL_AUDIO_NO_TRANSMIT);
      cell_update_state();
      shot_detector_process_detections(clip_id);

      // Put the CPU to sleep if nothing left to process
      __disable_irq();
      if (!audio_new_data_available() && !cell_pending_events() && !shot_detector_pending_processing())
         cpu_sleep();
      __enable_irq();
   }
   return 0;
}

#endif  // #ifdef CORE_CM4
