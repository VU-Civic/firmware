// Header Inclusions ---------------------------------------------------------------------------------------------------

#include <errno.h>
#include <sys/stat.h>
#include <sys/times.h>
#include "system.h"


// Shared Application Variables for Both Cores -------------------------------------------------------------------------

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
// TODO: Should probably reboot if any of these occur

void NMI_Handler(void) { while(1); }
void HardFault_Handler(void) { while (1); }
void MemManage_Handler(void) { while (1); }
void BusFault_Handler(void) { while (1); }
void UsageFault_Handler(void) { while (1); }
void SVC_Handler(void) {}
void DebugMon_Handler(void) {}
void PendSV_Handler(void) {}
void SysTick_Handler(void) { HAL_IncTick(); }  // TODO: Let's get rid of HAL altogether


// Public API Functions ------------------------------------------------------------------------------------------------

void chip_reset(void)
{
   // TODO: Implement this (also prob should activate some sort of watchdog timer)
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
