// Header Inclusions ---------------------------------------------------------------------------------------------------

#include <errno.h>
#include <sys/stat.h>
#include <sys/times.h>
#include "system.h"


// Shared Application Variables for Both Cores -------------------------------------------------------------------------

__attribute__ ((section (".user_nvm")))
non_volatile_data_t non_volatile_data;

__attribute__ ((section (".data_packet")))
volatile data_packet_container_t data;


// Shared Application Variables for Core CM4 ---------------------------------------------------------------------------

#ifdef CORE_CM4
volatile device_info_t device_info = { .firmware_date = FIRMWARE_BUILD_TIMESTAMP };
#endif


// C Standard Library Replacement Functions ----------------------------------------------------------------------------

static uint8_t *__sbrk_heap_end = NULL;

extern int __io_putchar(int ch) __attribute__ ((weak));
extern int __io_getchar(void) __attribute__ ((weak));

char *__env[1] = { 0 };
char **environ = __env;

void initialise_monitor_handles() {}
int _getpid(void) { return 1; }
int _kill(int pid, int sig) { errno = EINVAL; return -1; }
void _exit (int status) { _kill(status, -1); while(1); }
__attribute__ ((weak)) int _read(int file, char *ptr, int len) { for (int idx = 0; idx < len; ++idx) *ptr++ = __io_getchar(); return len; }
__attribute__ ((weak)) int _write(int file, char *ptr, int len) { for (int idx = 0; idx < len; ++idx) __io_putchar(*ptr++); return len; }
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

void NMI_Handler(void) { while(1); }
void HardFault_Handler(void) { while (1); }
void MemManage_Handler(void) { while (1); }
void BusFault_Handler(void) { while (1); }
void UsageFault_Handler(void) { while (1); }
void SVC_Handler(void) {}
void DebugMon_Handler(void) {}
void PendSV_Handler(void) {}
void SysTick_Handler(void) { HAL_IncTick(); }


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

void chip_initialize_unused_pins(void)
{
#ifdef CORE_CM7

   // Enable all GPIO clocks
   SET_BIT(RCC->AHB4ENR, RCC_AHB4ENR_GPIOAEN);
   (void)READ_BIT(RCC->AHB4ENR, RCC_AHB4ENR_GPIOAEN);
   SET_BIT(RCC->AHB4ENR, RCC_AHB4ENR_GPIOBEN);
   (void)READ_BIT(RCC->AHB4ENR, RCC_AHB4ENR_GPIOBEN);
   SET_BIT(RCC->AHB4ENR, RCC_AHB4ENR_GPIOCEN);
   (void)READ_BIT(RCC->AHB4ENR, RCC_AHB4ENR_GPIOCEN);
   SET_BIT(RCC->AHB4ENR, RCC_AHB4ENR_GPIODEN);
   (void)READ_BIT(RCC->AHB4ENR, RCC_AHB4ENR_GPIODEN);
   SET_BIT(RCC->AHB4ENR, RCC_AHB4ENR_GPIOEEN);
   (void)READ_BIT(RCC->AHB4ENR, RCC_AHB4ENR_GPIOEEN);
   SET_BIT(RCC->AHB4ENR, RCC_AHB4ENR_GPIOFEN);
   (void)READ_BIT(RCC->AHB4ENR, RCC_AHB4ENR_GPIOFEN);
   SET_BIT(RCC->AHB4ENR, RCC_AHB4ENR_GPIOGEN);
   (void)READ_BIT(RCC->AHB4ENR, RCC_AHB4ENR_GPIOGEN);
   SET_BIT(RCC->AHB4ENR, RCC_AHB4ENR_GPIOHEN);
   (void)READ_BIT(RCC->AHB4ENR, RCC_AHB4ENR_GPIOHEN);

   // Set all unused pins an analog inputs with pull-down enabled
   const gpio_pin_t unused_pins[] = UNUSED_PINS;
   for (uint32_t i = 0; i < (sizeof(unused_pins) / sizeof(unused_pins[0])); ++i)
   {
      uint32_t position = 32 - __builtin_clz(unused_pins[i].pin) - 1;
      MODIFY_REG(unused_pins[i].port->OSPEEDR, (GPIO_OSPEEDR_OSPEED0 << (position * 2U)), (GPIO_SPEED_FREQ_LOW << (position * 2U)));
      MODIFY_REG(unused_pins[i].port->OTYPER, (GPIO_OTYPER_OT0 << position), (((GPIO_MODE_ANALOG & OUTPUT_TYPE) >> OUTPUT_TYPE_Pos) << position));
      MODIFY_REG(unused_pins[i].port->PUPDR, (GPIO_PUPDR_PUPD0 << (position * 2U)), (GPIO_PULLDOWN << (position * 2U)));
      MODIFY_REG(unused_pins[i].port->MODER, (GPIO_MODER_MODE0 << (position * 2U)), ((GPIO_MODE_ANALOG & GPIO_MODE) << (position * 2U)));
   }

   // Disable all GPIO clocks
   CLEAR_BIT(RCC->AHB4ENR, RCC_AHB4ENR_GPIOAEN);
   CLEAR_BIT(RCC->AHB4ENR, RCC_AHB4ENR_GPIOBEN);
   CLEAR_BIT(RCC->AHB4ENR, RCC_AHB4ENR_GPIOCEN);
   CLEAR_BIT(RCC->AHB4ENR, RCC_AHB4ENR_GPIODEN);
   CLEAR_BIT(RCC->AHB4ENR, RCC_AHB4ENR_GPIOEEN);
   CLEAR_BIT(RCC->AHB4ENR, RCC_AHB4ENR_GPIOFEN);
   CLEAR_BIT(RCC->AHB4ENR, RCC_AHB4ENR_GPIOGEN);
   CLEAR_BIT(RCC->AHB4ENR, RCC_AHB4ENR_GPIOHEN);

#endif
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

   // Enable an independent watchdog that resets if not fed within 1 second
   SET_BIT(DBGMCU->APB4FZ1, DBGMCU_APB4FZ1_DBG_IWDG1);
   WRITE_REG(IWDG1->KR, IWDG_KEY_ENABLE);
   WRITE_REG(IWDG1->KR, IWDG_KEY_WRITE_ACCESS_ENABLE);
   WRITE_REG(IWDG1->PR, IWDG_PRESCALER_32);
   WRITE_REG(IWDG1->RLR, 1000);
   while (READ_BIT(IWDG1->SR, (IWDG_SR_WVU | IWDG_SR_RVU | IWDG_SR_PVU)));
   WRITE_REG(IWDG1->KR, IWDG_KEY_RELOAD);

#else

   // Disable SysTick interrupts
   CLEAR_BIT(SysTick->CTRL, SysTick_CTRL_TICKINT_Msk);

   // Enable an independent watchdog that resets if not fed within 1 second
   SET_BIT(DBGMCU->APB4FZ2, DBGMCU_APB4FZ2_DBG_IWDG2);
   WRITE_REG(IWDG2->KR, IWDG_KEY_ENABLE);
   WRITE_REG(IWDG2->KR, IWDG_KEY_WRITE_ACCESS_ENABLE);
   WRITE_REG(IWDG2->PR, IWDG_PRESCALER_32);
   WRITE_REG(IWDG2->RLR, 1000);
   while (READ_BIT(IWDG2->SR, (IWDG_SR_WVU | IWDG_SR_RVU | IWDG_SR_PVU)));
   WRITE_REG(IWDG2->KR, IWDG_KEY_RELOAD);

#endif
}

void cpu_sleep(void)
{
   // Put the CPU to sleep until awoken by an interrupt
   __DSB();
   __WFI();
   __ISB();
}

void cpu_feed_watchdog(void)
{
   // Reset the independent watchdog timer
#ifdef CORE_CM7
   WRITE_REG(IWDG1->KR, IWDG_KEY_RELOAD);
#else
   WRITE_REG(IWDG2->KR, IWDG_KEY_RELOAD);
#endif
}
