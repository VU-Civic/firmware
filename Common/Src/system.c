// Header Inclusions ---------------------------------------------------------------------------------------------------

#include "system.h"


// Shared Application Variables for Both Cores -------------------------------------------------------------------------

__attribute__((section (".data_packet")))
volatile data_packet_t data;


// Shared Application Variables for Core CM4 ---------------------------------------------------------------------------

#ifdef CORE_CM4
volatile device_info_t device_info;
#endif


// Public API Functions ------------------------------------------------------------------------------------------------

void chip_reset(void)
{
   ;
}

void cpu_init(void)
{
#ifdef CORE_CM7

   // Ensure the main regulator never turns off and the domain never enters standby mode
   MODIFY_REG(PWR->CR1, PWR_CR1_LPDS | PWR_CR1_SVOS, PWR_MAINREGULATOR_ON | PWR_REGULATOR_SVOS_SCALE3);
   CLEAR_BIT(PWR->CPUCR, (PWR_CPUCR_PDDS_D1 | PWR_CPUCR_PDDS_D3));

   // Auto-sleep the CPU after servicing an ISR
   MODIFY_REG(SCB->SCR, SCB_SCR_SLEEPDEEP_Msk, SCB_SCR_SLEEPONEXIT_Msk);

   // Disable SysTick interrupts
   CLEAR_BIT(SysTick->CTRL, SysTick_CTRL_TICKINT_Msk);

#else

   // Disable SysTick interrupts
   CLEAR_BIT(SysTick->CTRL, SysTick_CTRL_TICKINT_Msk);

#endif
}

void cpu_sleep(void)
{
   // Put the CPU to sleep until awoken by an interrupt
   __WFI();
}
