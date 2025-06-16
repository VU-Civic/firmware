/*#include "main.h"
#include "memorymap.h"
#include "audio.h"
#include "onset_detection.h"
#include "system.h"

#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U)
#endif

void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Config(void);

int main(void)
{
  MPU_Config();
  SCB_EnableICache();
  while (__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) {}
  HAL_Init();
  SystemClock_Config();
  PeriphCommonClock_Config();
  __HAL_RCC_HSEM_CLK_ENABLE();
  HAL_HSEM_FastTake(HSEM_ID_0);
  HAL_HSEM_Release(HSEM_ID_0,0);
  while (__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) {}

  // Initialize user peripherals and algorithms
  onset_detection_init();
  audio_init();
  audio_start();
  cpu_init();

  while (1)
  {
     cpu_sleep();
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 160;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 20;
  RCC_OscInitStruct.PLL.PLLR = 20;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_SPI2;
  PeriphClkInitStruct.PLL3.PLL3M = 5;
  PeriphClkInitStruct.PLL3.PLL3N = 48;
  PeriphClkInitStruct.PLL3.PLL3P = 5;
  PeriphClkInitStruct.PLL3.PLL3Q = 5;
  PeriphClkInitStruct.PLL3.PLL3R = 5;
  PeriphClkInitStruct.PLL3.PLL3RGE = RCC_PLL3VCIRANGE_2;
  PeriphClkInitStruct.PLL3.PLL3VCOSEL = RCC_PLL3VCOMEDIUM;
  PeriphClkInitStruct.PLL3.PLL3FRACN = 0;
  PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL3;
  PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_PLL3;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};
  HAL_MPU_Disable();
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}*/




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
      MODIFY_REG(RCC->D1CFGR, RCC_D1CFGR_D1PPRE,RCC_APB3_DIV2);
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
   __HAL_RCC_PLL3CLKOUT_ENABLE(RCC_PLL3_DIVQ);
   __HAL_RCC_PLL3CLKOUT_ENABLE(RCC_PLL3_DIVP);
   __HAL_RCC_PLL3_ENABLE();
   while (!__HAL_RCC_GET_FLAG(RCC_FLAG_PLL3RDY));
   MODIFY_REG(RCC->D2CCIP2R, RCC_D2CCIP2R_USBSEL, RCC_USBCLKSOURCE_PLL3);
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
   WRITE_REG(MPU->RASR, ((MPU_INSTRUCTION_ACCESS_DISABLE << MPU_RASR_XN_Pos) | (MPU_REGION_NO_ACCESS << MPU_RASR_AP_Pos)   |
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
