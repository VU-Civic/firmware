// Header Inclusions ---------------------------------------------------------------------------------------------------

#include <errno.h>
#include <sys/stat.h>
#include <sys/times.h>
#include "system.h"


// Shared Application Variables for Both Cores -------------------------------------------------------------------------

__attribute__((section (".user_nvm")))
non_volatile_data_t non_volatile_data;

__attribute__((section (".data_packet")))
volatile data_packet_t data;


// Shared Application Variables for Core CM4 ---------------------------------------------------------------------------

#ifdef CORE_CM4
volatile device_info_t device_info;
#endif


// C Standard Library Replacement Functions ----------------------------------------------------------------------------

static uint8_t *__sbrk_heap_end = NULL;

extern int __io_putchar(int ch) __attribute__((weak));
extern int __io_getchar(void) __attribute__((weak));

char *__env[1] = { 0 };
char **environ = __env;

void initialise_monitor_handles() {}
int _getpid(void) { return 1; }
int _kill(int pid, int sig) { errno = EINVAL; return -1; }
void _exit (int status) { _kill(status, -1); while(1); }
__attribute__((weak)) int _read(int file, char *ptr, int len) { for (int idx = 0; idx < len; ++idx) *ptr++ = __io_getchar(); return len; }
__attribute__((weak)) int _write(int file, char *ptr, int len) { for (int idx = 0; idx < len; ++idx) __io_putchar(*ptr++); return len; }
int _close(int file) { return -1; }
int _fstat(int file, struct stat *st) { st->st_mode = S_IFCHR; return 0; }
int _isatty(int file) { return 1; }
int _lseek(int file, int ptr, int dir) { return 0; }
int _open(char *path, int flags, ...) { return -1; }
int _wait(int *status) { errno = ECHILD; return -1; }
int _unlink(char *name) { errno = ENOENT; return -1; }
int _times(struct tms *buf) { return -1; }
int _stat(char *file, struct stat *st) { st->st_mode = S_IFCHR; return 0; }
int _link(char *old, char *new) { errno = EMLINK; return -1; }
int _fork(void) { errno = EAGAIN; return -1; }
int _execve(char *name, char **argv, char **env) { errno = ENOMEM; return -1; }

void *_sbrk(ptrdiff_t incr)
{
   // Symbols defined in the linker script
   extern uint8_t _end;
   extern uint8_t _estack;
   extern uint32_t _Min_Stack_Size;

   // Stack and heap limits
   const uint32_t stack_limit = (uint32_t)&_estack - (uint32_t)&_Min_Stack_Size;
   const uint8_t *max_heap = (uint8_t *)stack_limit;
   if (!__sbrk_heap_end)
      __sbrk_heap_end = &_end;

   // Protect heap from growing into the reserved MSP stack
   if ((__sbrk_heap_end + incr) > max_heap)
   {
      errno = ENOMEM;
      return (void*)-1;
   }

   // Return newly allocated heap memory
   uint8_t *prev_heap_end = __sbrk_heap_end;
   __sbrk_heap_end += incr;
   return (void*)prev_heap_end;
}


// ARM Cortex Processor Interrupt and Exception Handlers ---------------------------------------------------------------
// TODO: Should probably reboot if any of these occur (or alternately use a watchdog to auto-restart)

void NMI_Handler(void) { while(1); }
void HardFault_Handler(void) { while (1); }
void MemManage_Handler(void) { while (1); }
void BusFault_Handler(void) { while (1); }
void UsageFault_Handler(void) { while (1); }
void SVC_Handler(void) {}
void DebugMon_Handler(void) {}
void PendSV_Handler(void) {}
void SysTick_Handler(void) { HAL_IncTick(); }


// Shared Peripheral Interrupt Handlers --------------------------------------------------------------------------------

#ifdef CORE_CM4
#if REV_ID != REV_A

void IMU_Int_IRQHandler(void);
void AI_Int_IRQHandler(void);

void EXTI9_5_IRQHandler(void)
{
   if (READ_BIT(EXTI->C2PR1, IMU_INT_Pin))
      IMU_Int_IRQHandler();
   if (READ_BIT(EXTI->C2PR1, FROM_AI_INT_Pin))
      AI_Int_IRQHandler();
}

#endif
#endif


// Public API Functions ------------------------------------------------------------------------------------------------

void chip_reset(void)
{
   // Fully reset both cores
   NVIC_SystemReset();
}

non_volatile_data_t chip_read_non_volatile(void)
{
   // Ensure that the data being read represents the most currently stored data
   non_volatile_data_t nvm_data = non_volatile_data;
   __DSB();
   return nvm_data;
}

void chip_save_non_volatile(const non_volatile_data_t *nvm_data)
{
   // TODO: TEST THIS
   // Determine which flash sector the user memory lies in
   uint32_t sector, flash_address = (uint32_t)&non_volatile_data;
   if ((flash_address < 0x08120000) && (flash_address >= 0x08100000))
      sector = FLASH_SECTOR_0;
   else if ((flash_address < 0x08140000) && (flash_address >= 0x08120000))
      sector = FLASH_SECTOR_1;
   else if ((flash_address < 0x08160000) && (flash_address >= 0x08140000))
      sector = FLASH_SECTOR_2;
   else if ((flash_address < 0x08180000) && (flash_address >= 0x08160000))
      sector = FLASH_SECTOR_3;
   else if ((flash_address < 0x081A0000) && (flash_address >= 0x08180000))
      sector = FLASH_SECTOR_4;
   else if ((flash_address < 0x081C0000) && (flash_address >= 0x081A0000))
      sector = FLASH_SECTOR_5;
   else if ((flash_address < 0x081E0000) && (flash_address >= 0x081C0000))
      sector = FLASH_SECTOR_6;
   else
      sector = FLASH_SECTOR_7;

   // Unlock the flash memory bank and clear any pending interrupts
   // TODO: DO I NEED TO UNLOCK BANK 1???
   /*if (READ_BIT(FLASH->CR1, FLASH_CR_LOCK))
   {
      WRITE_REG(FLASH->KEYR1, FLASH_KEY1);
      WRITE_REG(FLASH->KEYR1, FLASH_KEY2);
   }*/
   if (READ_BIT(FLASH->CR2, FLASH_CR_LOCK))
   {
      WRITE_REG(FLASH->KEYR2, FLASH_KEY1);
      WRITE_REG(FLASH->KEYR2, FLASH_KEY2);
   }
   WRITE_REG(FLASH->CCR2, ((FLASH_SR_EOP | FLASH_SR_OPERR | FLASH_SR_WRPERR | FLASH_SR_PGSERR) & 0x7FFFFFFFU));

   // Erase the user flash sector
   CLEAR_BIT(FLASH->CR2, (FLASH_CR_PSIZE | FLASH_CR_SNB));
   SET_BIT(FLASH->CR2, (FLASH_CR_SER | VOLTAGE_RANGE_3 | (sector << FLASH_CR_SNB_Pos) | FLASH_CR_START));
   while (__HAL_FLASH_GET_FLAG_BANK2(FLASH_FLAG_QW_BANK2));
   __HAL_FLASH_CLEAR_FLAG_BANK2(FLASH_FLAG_EOP_BANK2);
   CLEAR_BIT(FLASH->CR2, (FLASH_CR_SER | FLASH_CR_SNB));

   // Program the user flash sector word by word and re-lock the flash
   volatile uint32_t *src_addr = (volatile uint32_t*)nvm_data;
   volatile uint32_t *const end_addr = src_addr + sizeof(non_volatile_data_t);
   while (src_addr < end_addr)
   {
      volatile uint32_t *dest_addr = (volatile uint32_t*)flash_address;
      while (__HAL_FLASH_GET_FLAG_BANK2(FLASH_FLAG_QW_BANK2));
      __HAL_FLASH_CLEAR_FLAG_BANK2(FLASH_FLAG_EOP_BANK2);
      SET_BIT(FLASH->CR2, FLASH_CR_PG);
      __ISB(); __DSB();
      for (uint8_t row_index = FLASH_NB_32BITWORD_IN_FLASHWORD; row_index; --row_index)
      {
         *dest_addr = *src_addr;
         dest_addr++;
         src_addr++;
      }
      __ISB(); __DSB();
      while (__HAL_FLASH_GET_FLAG_BANK2(FLASH_FLAG_QW_BANK2));
      __HAL_FLASH_CLEAR_FLAG_BANK2(FLASH_FLAG_EOP_BANK2);
      CLEAR_BIT(FLASH->CR2, FLASH_CR_PG);
      flash_address += 32;
   }
   SET_BIT(FLASH->CR2, FLASH_CR_LOCK);
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
   __DSB();
   __WFI();
   __ISB();
}
