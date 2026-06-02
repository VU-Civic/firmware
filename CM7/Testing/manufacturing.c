#ifdef CORE_CM7

// Header Inclusions ---------------------------------------------------------------------------------------------------

#include "main.h"
#include "audio.h"
#include "onset_detection.h"
#include "system.h"


// Clock Configuration Functions ---------------------------------------------------------------------------------------

static void SystemClock_Config(void)
{
   // Configure the power supply and main internal regulator
   MODIFY_REG(PWR->CR3, PWR_SUPPLY_CONFIG_MASK, PWR_LDO_SUPPLY);
   while (!__HAL_PWR_GET_FLAG(PWR_FLAG_ACTVOSRDY));
   __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
   while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY));
   SET_BIT(PWR->CR1, PWR_CR1_DBP);
   __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

   // Initialize the RCC oscillators
   __HAL_RCC_HSE_CONFIG(RCC_HSE_BYPASS);
   while (!__HAL_RCC_GET_FLAG(RCC_FLAG_HSERDY));
   SET_BIT(PWR->CR1, PWR_CR1_DBP);
   while (!READ_BIT(PWR->CR1, PWR_CR1_DBP));
   __HAL_RCC_LSE_CONFIG(RCC_LSE_ON);
   while (!__HAL_RCC_GET_FLAG(RCC_FLAG_LSERDY));
   if (__HAL_RCC_GET_SYSCLK_SOURCE() != RCC_CFGR_SWS_PLL1)
   {
      __HAL_RCC_PLL_DISABLE();
      while (__HAL_RCC_GET_FLAG(RCC_FLAG_PLLRDY));
      __HAL_RCC_PLL_CONFIG(RCC_PLLSOURCE_HSE, 5, 160, 2, 20, 20);
      __HAL_RCC_PLLFRACN_DISABLE();
      __HAL_RCC_PLLFRACN_CONFIG(0);
      __HAL_RCC_PLL_VCIRANGE(RCC_PLL1VCIRANGE_2) ;
      __HAL_RCC_PLL_VCORANGE(RCC_PLL1VCOWIDE) ;
      __HAL_RCC_PLLCLKOUT_ENABLE(RCC_PLL1_DIVP);
      __HAL_RCC_PLLCLKOUT_ENABLE(RCC_PLL1_DIVQ);
      __HAL_RCC_PLLCLKOUT_ENABLE(RCC_PLL1_DIVR);
      __HAL_RCC_PLLFRACN_ENABLE();
      __HAL_RCC_PLL_ENABLE();
      while (!__HAL_RCC_GET_FLAG(RCC_FLAG_PLLRDY));
   }
   else
   {
      const uint32_t temp1_pllckcfg = RCC->PLLCKSELR, temp2_pllckcfg = RCC->PLL1DIVR;
      if (!((READ_BIT(temp1_pllckcfg, RCC_PLLCKSELR_PLLSRC) != RCC_PLLSOURCE_HSE) ||
           ((READ_BIT(temp1_pllckcfg, RCC_PLLCKSELR_DIVM1) >> RCC_PLLCKSELR_DIVM1_Pos) != 5) ||
           (READ_BIT(temp2_pllckcfg, RCC_PLL1DIVR_N1) != (160 - 1U)) ||
           ((READ_BIT(temp2_pllckcfg, RCC_PLL1DIVR_P1) >> RCC_PLL1DIVR_P1_Pos) != (2 - 1U)) ||
           ((READ_BIT(temp2_pllckcfg, RCC_PLL1DIVR_Q1) >> RCC_PLL1DIVR_Q1_Pos) != (20 - 1U)) ||
           ((READ_BIT(temp2_pllckcfg, RCC_PLL1DIVR_R1) >> RCC_PLL1DIVR_R1_Pos) != (20 - 1U))))
      {
         if (0 != ((RCC->PLL1FRACR & RCC_PLL1FRACR_FRACN1) >> RCC_PLL1FRACR_FRACN1_Pos))
         {
            __HAL_RCC_PLLFRACN_DISABLE();
            __HAL_RCC_PLLFRACN_CONFIG(0);
            __HAL_RCC_PLLFRACN_ENABLE();
         }
      }
   }

   // Initialize the CPU, AHB, and APB buses clocks
   while (FLASH_LATENCY_2 > __HAL_FLASH_GET_LATENCY())
      __HAL_FLASH_SET_LATENCY(FLASH_LATENCY_2);
   if (RCC_APB3_DIV2 > (RCC->D1CFGR & RCC_D1CFGR_D1PPRE))
      MODIFY_REG(RCC->D1CFGR, RCC_D1CFGR_D1PPRE, RCC_APB3_DIV2);
   if (RCC_APB1_DIV2 > (RCC->D2CFGR & RCC_D2CFGR_D2PPRE1))
      MODIFY_REG(RCC->D2CFGR, RCC_D2CFGR_D2PPRE1, RCC_APB1_DIV2);
   if (RCC_APB2_DIV2 > (RCC->D2CFGR & RCC_D2CFGR_D2PPRE2))
      MODIFY_REG(RCC->D2CFGR, RCC_D2CFGR_D2PPRE2, RCC_APB2_DIV2);
   if (RCC_APB4_DIV2 > (RCC->D3CFGR & RCC_D3CFGR_D3PPRE))
      MODIFY_REG(RCC->D3CFGR, RCC_D3CFGR_D3PPRE, RCC_APB4_DIV2);
   if (RCC_HCLK_DIV2 > (RCC->D1CFGR & RCC_D1CFGR_HPRE))
      MODIFY_REG(RCC->D1CFGR, RCC_D1CFGR_HPRE, RCC_HCLK_DIV2);
   MODIFY_REG(RCC->D1CFGR, RCC_D1CFGR_D1CPRE, RCC_SYSCLK_DIV1);
   MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, RCC_SYSCLKSOURCE_PLLCLK);
   while (__HAL_RCC_GET_SYSCLK_SOURCE() != (RCC_SYSCLKSOURCE_PLLCLK << RCC_CFGR_SWS_Pos));
   if (RCC_HCLK_DIV2 < (RCC->D1CFGR & RCC_D1CFGR_HPRE))
      MODIFY_REG(RCC->D1CFGR, RCC_D1CFGR_HPRE, RCC_HCLK_DIV2);
   while (FLASH_LATENCY_2 < __HAL_FLASH_GET_LATENCY())
      __HAL_FLASH_SET_LATENCY(FLASH_LATENCY_2);
   if (RCC_APB3_DIV2 < (RCC->D1CFGR & RCC_D1CFGR_D1PPRE))
      MODIFY_REG(RCC->D1CFGR, RCC_D1CFGR_D1PPRE, RCC_APB3_DIV2);
   if (RCC_APB1_DIV2 < (RCC->D2CFGR & RCC_D2CFGR_D2PPRE1))
      MODIFY_REG(RCC->D2CFGR, RCC_D2CFGR_D2PPRE1, RCC_APB1_DIV2);
   if (RCC_APB2_DIV2 < (RCC->D2CFGR & RCC_D2CFGR_D2PPRE2))
      MODIFY_REG(RCC->D2CFGR, RCC_D2CFGR_D2PPRE2, RCC_APB2_DIV2);
   if (RCC_APB4_DIV2 < (RCC->D3CFGR & RCC_D3CFGR_D3PPRE))
      MODIFY_REG(RCC->D3CFGR, RCC_D3CFGR_D3PPRE, RCC_APB4_DIV2);
   SystemCoreClockUpdate();
   SysTick_Config(SystemCoreClock / 1000UL);
}

static void PeriphCommonClock_Config(void)
{
   // Initialize the peripherals clock
   CLEAR_BIT(RCC->CR, RCC_CR_PLL3ON);
   while (__HAL_RCC_GET_FLAG(RCC_FLAG_PLL3RDY));
   __HAL_RCC_PLL3_CONFIG(5, 48, 5, 5, 5);
   __HAL_RCC_PLL3_VCIRANGE(RCC_PLL3VCIRANGE_2);
   __HAL_RCC_PLL3_VCORANGE(RCC_PLL3VCOMEDIUM);
   __HAL_RCC_PLL3FRACN_DISABLE();
   __HAL_RCC_PLL3FRACN_CONFIG(0);
   __HAL_RCC_PLL3FRACN_ENABLE();
   __HAL_RCC_PLL3CLKOUT_ENABLE(RCC_PLL3_DIVP);
   __HAL_RCC_PLL3CLKOUT_ENABLE(RCC_PLL3_DIVQ);
   __HAL_RCC_PLL3_ENABLE();
   while (!__HAL_RCC_GET_FLAG(RCC_FLAG_PLL3RDY));
   MODIFY_REG(RCC->D2CCIP2R, RCC_D2CCIP2R_USBSEL, RCC_USBCLKSOURCE_PLL3);
   MODIFY_REG(RCC->D1CCIPR, RCC_D1CCIPR_CKPERSEL, RCC_CLKPSOURCE_HSI);
}

static void MPU_Config(void)
{
   // Disable the MPU
   __DMB();
   CLEAR_BIT(SCB->SHCSR, SCB_SHCSR_MEMFAULTENA_Msk);
   WRITE_REG(MPU->CTRL, 0);

   // Initialize the entire SRAM as protected under the following policy
   WRITE_REG(MPU->RNR, MPU_REGION_NUMBER0);
   CLEAR_BIT(MPU->RASR, MPU_RASR_ENABLE_Msk);
   WRITE_REG(MPU->RBAR, 0x0);
   WRITE_REG(MPU->RASR, ((MPU_INSTRUCTION_ACCESS_DISABLE << MPU_RASR_XN_Pos) | (MPU_REGION_NO_ACCESS << MPU_RASR_AP_Pos) |
                         (MPU_TEX_LEVEL0 << MPU_RASR_TEX_Pos) | (MPU_ACCESS_SHAREABLE << MPU_RASR_S_Pos) |
                         (MPU_ACCESS_NOT_CACHEABLE << MPU_RASR_C_Pos) | (MPU_ACCESS_NOT_BUFFERABLE << MPU_RASR_B_Pos) |
                         (0x87 << MPU_RASR_SRD_Pos) |  (MPU_REGION_SIZE_4GB << MPU_RASR_SIZE_Pos) | (MPU_REGION_ENABLE << MPU_RASR_ENABLE_Pos)));
   WRITE_REG(MPU->CTRL, (MPU_PRIVILEGED_DEFAULT | MPU_CTRL_ENABLE_Msk));
   SET_BIT(SCB->SHCSR, SCB_SHCSR_MEMFAULTENA_Msk);
   __DSB(); __ISB();
}

void Error_Handler(void)
{
   __disable_irq();
   while (1);
}


// Main Application Function -------------------------------------------------------------------------------------------

int main(void)
{
   // Configure the Memory Protection Unit
   MPU_Config();

   // Enable the Instruction Cache
   if (!READ_BIT(SCB->CCR, SCB_CCR_IC_Msk))
   {
      __DSB(); __ISB();
      WRITE_REG(SCB->ICIALLU, 0UL);
      __DSB(); __ISB();
      CLEAR_BIT(SCB->CCR, SCB_CCR_IC_Msk);
      __DSB(); __ISB();
   }

   // Wait until CORE_CM4 boots and enters STOP mode
   while (__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET);

   // Set the NVIC interrupt group priority
   NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

   // Configure a 1ms SysTick interrupt timer which will be disabled later
   SystemCoreClockUpdate();
   SysTick_Config(SystemCoreClock / 1000UL);
   NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), TICK_INT_PRIORITY, 0U));
   uwTickPrio = TICK_INT_PRIORITY;

   // Configure the system and peripheral clocks
   SystemClock_Config();
   PeriphCommonClock_Config();

   // Enable the HSEM clock and wake up CORE_CM4
   SET_BIT(RCC->AHB4ENR, RCC_AHB4ENR_HSEMEN);
   (void)READ_BIT(RCC->AHB4ENR, RCC_AHB4ENR_HSEMEN);
   if ((HSEM->RLR[0] == (HSEM_CR_COREID_CURRENT | HSEM_RLR_LOCK)))
      HSEM->R[0] = HSEM_CR_COREID_CURRENT;
   while (__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET);

   // Initialize user peripherals and algorithms
   chip_initialize_unused_pins();
   onset_detection_init();
   audio_init();
   audio_start();
   cpu_init();

   // Loop forever
   while (1)
   {
      // Should sleep forever, application is fully interrupt-based
      cpu_sleep();
   }
   return 0;
}

#endif  // #ifdef CORE_CM7
