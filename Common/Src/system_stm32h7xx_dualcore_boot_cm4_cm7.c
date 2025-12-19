#include "stm32h7xx.h"
#include <string.h>

#ifndef HSE_VALUE
#define HSE_VALUE    ((uint32_t)25000000)
#endif

#ifndef CSI_VALUE
#define CSI_VALUE    ((uint32_t)4000000)
#endif

#ifndef HSI_VALUE
#define HSI_VALUE    ((uint32_t)64000000)
#endif


/************************* Miscellaneous Configuration ************************/
/* Note: Following vector table addresses must be defined in line with linker
         configuration. */
/*!< Uncomment the following line if you need to relocate the vector table
     anywhere in FLASH BANK1 or AXI SRAM, else the vector table is kept at the automatic
     remap of boot address selected */
/* #define USER_VECT_TAB_ADDRESS */

#if defined(USER_VECT_TAB_ADDRESS)
#if defined(CORE_CM4)
/*!< Uncomment the following line if you need to relocate your vector Table
     in D2 AXI SRAM else user remap will be done in FLASH BANK2. */
/* #define VECT_TAB_SRAM */
#if defined(VECT_TAB_SRAM)
#define VECT_TAB_BASE_ADDRESS   D2_AXISRAM_BASE   /*!< Vector Table base address field.
                                                       This value must be a multiple of 0x400. */
#define VECT_TAB_OFFSET         0x00000000U       /*!< Vector Table base offset field.
                                                       This value must be a multiple of 0x400. */
#else
#define VECT_TAB_BASE_ADDRESS   FLASH_BANK2_BASE  /*!< Vector Table base address field.
                                                       This value must be a multiple of 0x400. */
#define VECT_TAB_OFFSET         0x00000000U       /*!< Vector Table base offset field.
                                                       This value must be a multiple of 0x400. */
#endif /* VECT_TAB_SRAM */
#elif defined(CORE_CM7)
/*!< Uncomment the following line if you need to relocate your vector Table
     in D1 AXI SRAM else user remap will be done in FLASH BANK1. */
/* #define VECT_TAB_SRAM */
#if defined(VECT_TAB_SRAM)
#define VECT_TAB_BASE_ADDRESS   D1_AXISRAM_BASE   /*!< Vector Table base address field.
                                                       This value must be a multiple of 0x400. */
#define VECT_TAB_OFFSET         0x00000000U       /*!< Vector Table base offset field.
                                                       This value must be a multiple of 0x400. */
#else
#define VECT_TAB_BASE_ADDRESS   FLASH_BANK1_BASE  /*!< Vector Table base address field.
                                                       This value must be a multiple of 0x400. */
#define VECT_TAB_OFFSET         0x00000000U       /*!< Vector Table base offset field.
                                                       This value must be a multiple of 0x400. */
#endif /* VECT_TAB_SRAM */
#else
#error Please #define CORE_CM4 or CORE_CM7
#endif /* CORE_CM4 */
#endif /* USER_VECT_TAB_ADDRESS */
/******************************************************************************/

// System domain clock variables (updated by SystemCoreClockUpdate())
uint32_t SystemCoreClock = 64000000;
uint32_t SystemD2Clock = 64000000;
const  uint8_t D1CorePrescTable[16] = {0, 0, 0, 0, 1, 2, 3, 4, 1, 2, 3, 4, 6, 7, 8, 9};

// ITCM RAM location variables
extern const unsigned char _sitcmram;
extern const unsigned char _eitcmram;
extern const unsigned char _itcm_data;

// System initialization function called automatically by assembly code before entering main()
void SystemInit(void)
{
   // FPU settings
#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
   SCB->CPACR |= ((3UL << (10*2))|(3UL << (11*2)));
#endif

   // Enable SEVONPEND so that an interrupt coming from the CPU(n) interrupt signal is
   // detectable by the CPU after a WFI/WFE instruction
   SCB->SCR |= SCB_SCR_SEVONPEND_Msk;

#if defined(CORE_CM7)

   // Reset the RCC clock configuration to its initial default state
   if(FLASH_LATENCY_DEFAULT  > (READ_BIT((FLASH->ACR), FLASH_ACR_LATENCY)))
      MODIFY_REG(FLASH->ACR, FLASH_ACR_LATENCY, (uint32_t)(FLASH_LATENCY_DEFAULT));
   RCC->CR |= RCC_CR_HSION;
   RCC->CFGR = 0x00000000;
   RCC->CR &= 0xEAF6ED7FU;
   if(FLASH_LATENCY_DEFAULT  < (READ_BIT((FLASH->ACR), FLASH_ACR_LATENCY)))
      MODIFY_REG(FLASH->ACR, FLASH_ACR_LATENCY, (uint32_t)(FLASH_LATENCY_DEFAULT));
   RCC->D1CFGR = 0x00000000;
   RCC->D2CFGR = 0x00000000;
   RCC->D3CFGR = 0x00000000;
   RCC->PLLCKSELR = 0x02020200;
   RCC->PLLCFGR = 0x01FF0000;
   RCC->PLL1DIVR = 0x01010280;
   RCC->PLL1FRACR = 0x00000000;
   RCC->PLL2DIVR = 0x01010280;
   RCC->PLL2FRACR = 0x00000000;
   RCC->PLL3DIVR = 0x01010280;
   RCC->PLL3FRACR = 0x00000000;
   RCC->CR &= 0xFFFBFFFFU;
   RCC->CIER = 0x00000000;

   // Enable CortexM7 HSEM EXTI line (line 78)
   EXTI_D2->EMR3 |= 0x4000UL;
   if((DBGMCU->IDCODE & 0xFFFF0000U) < 0x20000000U)
      *((__IO uint32_t*)0x51008108) = 0x000000001U;

#endif /* CORE_CM7*/

#if defined(CORE_CM4)

   // Configure the Vector Table location add offset address for Cortex-M4
#if defined(USER_VECT_TAB_ADDRESS)
   SCB->VTOR = VECT_TAB_BASE_ADDRESS | VECT_TAB_OFFSET; /* Vector Table Relocation in Internal D2 AXI-RAM or in Internal FLASH */
#endif /* USER_VECT_TAB_ADDRESS */

#elif defined(CORE_CM7)

   if(READ_BIT(RCC->AHB3ENR, RCC_AHB3ENR_FMCEN) == 0U)
   {
      // Enable the FMC interface clock
      SET_BIT(RCC->AHB3ENR, RCC_AHB3ENR_FMCEN);

      // Disable the FMC bank1 (enabled after reset) to prevent CPU speculation access on this bank
      // which blocks the use of FMC during 24us
      FMC_Bank1_R->BTCR[0] = 0x000030D2;

      // Disable the FMC interface clock
      CLEAR_BIT(RCC->AHB3ENR, RCC_AHB3ENR_FMCEN);
   }

   // Configure the Vector Table location as ITCM for Cortex-M7
   memcpy((void*)&_sitcmram, &_itcm_data, (int)(&_eitcmram - &_sitcmram));
   SCB->VTOR = 0x00000000;

#endif /* CORE_CM4 */
}

void SystemCoreClockUpdate (void)
{
   uint32_t pllp, pllsource, pllm, pllfracen, hsivalue, common_system_clock;
   float fracn1, pllvco;

   // Get SYSCLK source
   switch (RCC->CFGR & RCC_CFGR_SWS)
   {
      case RCC_CFGR_SWS_HSI:
         common_system_clock = (uint32_t)(HSI_VALUE >> ((RCC->CR & RCC_CR_HSIDIV)>> 3));
         break;
      case RCC_CFGR_SWS_CSI:
         common_system_clock = CSI_VALUE;
         break;
      case RCC_CFGR_SWS_HSE:
         common_system_clock = HSE_VALUE;
         break;
      case RCC_CFGR_SWS_PLL1:
         // PLL_VCO = (HSE_VALUE or HSI_VALUE or CSI_VALUE/ PLLM) * PLLN
         // SYSCLK = PLL_VCO / PLLR
         pllsource = (RCC->PLLCKSELR & RCC_PLLCKSELR_PLLSRC);
         pllm = ((RCC->PLLCKSELR & RCC_PLLCKSELR_DIVM1)>> 4);
         pllfracen = ((RCC->PLLCFGR & RCC_PLLCFGR_PLL1FRACEN)>>RCC_PLLCFGR_PLL1FRACEN_Pos);
         fracn1 = (float)(uint32_t)(pllfracen* ((RCC->PLL1FRACR & RCC_PLL1FRACR_FRACN1)>> 3));
         if (pllm)
         {
            switch (pllsource)
            {
               case RCC_PLLCKSELR_PLLSRC_HSI:
                  hsivalue = (HSI_VALUE >> ((RCC->CR & RCC_CR_HSIDIV)>> 3));
                  pllvco = ( (float)hsivalue / (float)pllm) * ((float)(uint32_t)(RCC->PLL1DIVR & RCC_PLL1DIVR_N1) + (fracn1/(float)0x2000) +(float)1 );
                  break;
               case RCC_PLLCKSELR_PLLSRC_CSI:
                  pllvco = ((float)CSI_VALUE / (float)pllm) * ((float)(uint32_t)(RCC->PLL1DIVR & RCC_PLL1DIVR_N1) + (fracn1/(float)0x2000) +(float)1 );
                  break;
               case RCC_PLLCKSELR_PLLSRC_HSE:
                  pllvco = ((float)HSE_VALUE / (float)pllm) * ((float)(uint32_t)(RCC->PLL1DIVR & RCC_PLL1DIVR_N1) + (fracn1/(float)0x2000) +(float)1 );
                  break;
               default:
                  hsivalue = (HSI_VALUE >> ((RCC->CR & RCC_CR_HSIDIV)>> 3)) ;
                  pllvco = ((float)hsivalue / (float)pllm) * ((float)(uint32_t)(RCC->PLL1DIVR & RCC_PLL1DIVR_N1) + (fracn1/(float)0x2000) +(float)1 );
                  break;
            }
            pllp = (((RCC->PLL1DIVR & RCC_PLL1DIVR_P1) >>9) + 1U);
            common_system_clock = (uint32_t)(float)(pllvco / (float)pllp);
         }
         else
            common_system_clock = 0U;
         break;
      default:
         common_system_clock = (uint32_t)(HSI_VALUE >> ((RCC->CR & RCC_CR_HSIDIV)>> 3));
         break;
  }

   // Compute SystemClock frequency
   uint32_t tmp = D1CorePrescTable[(RCC->D1CFGR & RCC_D1CFGR_D1CPRE)>> RCC_D1CFGR_D1CPRE_Pos];
   common_system_clock >>= tmp;
   SystemD2Clock = (common_system_clock >> ((D1CorePrescTable[(RCC->D1CFGR & RCC_D1CFGR_HPRE)>> RCC_D1CFGR_HPRE_Pos]) & 0x1FU));
#ifdef CORE_CM4
   SystemCoreClock = SystemD2Clock;
#else
   SystemCoreClock = common_system_clock;
#endif
}

void ExitRun0Mode(void)
{
   // Exit Run* mode by disabling SMPS and enabling LDO
   PWR->CR3 = (PWR->CR3 & ~PWR_CR3_SMPSEN) | PWR_CR3_LDOEN;

   // Wait till voltage level flag is set
   while ((PWR->CSR1 & PWR_CSR1_ACTVOSRDY) == 0U);
}
