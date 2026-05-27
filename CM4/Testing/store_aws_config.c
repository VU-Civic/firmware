#ifdef CORE_CM4

// ============================================================
// AWS IoT Core Certificate Provisioning Utility
// ============================================================
//
// PURPOSE: One-time utility to load TLS credentials into the
//   u-blox modem's NVM (security profile CELL_AWS_TLS_PROFILE)
//   so that the device can authenticate with AWS IoT Core.
//
// USAGE:
//   1. Obtain credentials from the AWS IoT Core console:
//        IoT Core → Security → Certificates → Create certificate
//      Download:
//        - Device certificate    (xxx-certificate.pem.crt)
//        - Device private key    (xxx-private.pem.key)
//        (Amazon Root CA 1 is already bundled in this file)
//
//   2. Copy the two downloaded files into CM4/Testing/certs/:
//        CM4/Testing/certs/device_cert.pem
//        CM4/Testing/certs/device_key.pem
//
//   3. Build and flash with:
//        make provision BOARD_REV=<rev>
//      Optionally override the cert directory:
//        make provision BOARD_REV=<rev> CERT_DIR=path/to/certs
//      Paths are relative to the CM4/ directory.
//
//   4. Boot the device.  Provisioning runs automatically.
//      Result is shown via the GPS status LED:
//        success  →  LED steady ON
//        failure  →  LED rapid blink (100 ms)
//
//   5. Flash normal firmware with:
//        make all BOARD_REV=<rev>
//      Credentials remain in modem NVM across power cycles.
//
// NOTE: The device certificate and private key are unique per
//   device.  Do not share private keys between units.
//   The CM4/Testing/certs/ directory is .gitignored.
// ============================================================

#include "main.h"
#include "cellular.h"
#include "system.h"


// ------------------------------------------------------------
// Compile-time certificate embedding
//
// INCBIN embeds the raw bytes of a file from disk directly into
// the .rodata section at compile time.  The Makefile's
// 'provision' target defines DEVICE_CERT_FILE and DEVICE_KEY_FILE
// to the paths of the PEM files (relative to CM4/).
//
// If those macros are not defined (manual build), the fallback
// placeholder strings are used instead.
// ------------------------------------------------------------

#define INCBIN(sym, file) \
   __asm__( \
      ".section .rodata,\"a\",%progbits\n" \
      ".balign 4\n" \
      ".global prov_" #sym "_start\n" \
      "prov_" #sym "_start:\n" \
      ".incbin \"" STRINGIZE(file) "\"\n" \
      ".byte 0\n" \
      ".balign 4\n" \
      ".global prov_" #sym "_end\n" \
      "prov_" #sym "_end:\n" \
      ".previous\n" \
   ); \
   extern const char prov_##sym##_start[]; \
   extern const char prov_##sym##_end[]

#if defined(DEVICE_CERT_FILE) && defined(DEVICE_KEY_FILE)

INCBIN(device_cert, DEVICE_CERT_FILE);
INCBIN(device_key,  DEVICE_KEY_FILE);
#define DEVICE_CERT_PEM  prov_device_cert_start
#define DEVICE_KEY_PEM   prov_device_key_start

#else

static const char DEVICE_CERT_PEM[] =
  "-----BEGIN CERTIFICATE-----\n"
  "REPLACE_WITH_YOUR_DEVICE_CERTIFICATE_CONTENT\n"
  "-----END CERTIFICATE-----\n";

static const char DEVICE_KEY_PEM[] =
  "-----BEGIN RSA PRIVATE KEY-----\n"
  "REPLACE_WITH_YOUR_DEVICE_PRIVATE_KEY_CONTENT\n"
  "-----END RSA PRIVATE KEY-----\n";

#endif  // defined(DEVICE_CERT_FILE) && defined(DEVICE_KEY_FILE)


// Amazon Root CA 1 (public certificate, safe to bundle in source)
static const char AWS_ROOT_CA_PEM[] =
  "-----BEGIN CERTIFICATE-----\n"
  "MIIDQTCCAimgAwIBAgITBmyfz5m/jAo54vB4ikPmljZbyjANBgkqhkiG9w0BAQsF\n"
  "ADA5MQswCQYDVQQGEwJVUzEPMA0GA1UEChMGQW1hem9uMRkwFwYDVQQDExBBbWF6\n"
  "b24gUm9vdCBDQSAxMB4XDTE1MDUyNjAwMDAwMFoXDTM4MDExNzAwMDAwMFowOTEL\n"
  "MAkGA1UEBhMCVVMxDzANBgNVBAoTBkFtYXpvbjEZMBcGA1UEAxMQQW1hem9uIFJv\n"
  "b3QgQ0EgMTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBALJ4gHHKeNXj\n"
  "ca9HgFB0fW7Y14h29Jlo91ghYPl0hAEvrAIthtOgQ3pOsqTQNroBvo3bSMgHFzZM\n"
  "9O6II8c+6zf1tRn4SWiw3te5djgdYZ6k/oI2peVKVuRF4fn9tBb6dNqcmzU5L/qw\n"
  "IFAGbHrQgLKm+a/sRxmPUDgH3KKHOVj4utWp+UhnMJbulHheb4mjUcAwhmahRWa6\n"
  "VOujw5H5SNz/0egwLX0tdHA114gk957EWW67c4cX8jJGKLhD+rcdqsq08p8kDi1L\n"
  "93FcXmn/6pUCyziKrlA4b9v7LWIbxcceVOF34GfID5yHI9Y/QCB/IIDEgEw+OyQm\n"
  "jgSubJrIqg0CAwEAAaNCMEAwDwYDVR0TAQH/BAUwAwEB/zAOBgNVHQ8BAf8EBAMC\n"
  "AYYwHQYDVR0OBBYEFIQYzIU07LwMlJQuCFmcx7IQTgoIMA0GCSqGSIb3DQEBCwUA\n"
  "A4IBAQCY8jdaQZChGsV2USggNiMOruYou6r4lK5IpDB/G/wkjUu0yKGX9rbxenDI\n"
  "U5PMCCjjmCXPI6T53iHTfIUJrU6adTrCC2qJeHZERxhlbI1Bjjt/msv0tadQ1wUs\n"
  "N+gDS63pYaACbvXy8MWy7Vu33PqUXHeeE6V/Uq2V8viTO96LXFvKWlJbYK8U90vv\n"
  "o/ufQJVtMVT8QtPHRh8jrdkPSHCa2XV4cdFyQzR1bldZwgJcJmApzyMZFo6IQ6XU\n"
  "5MsI+yMRQ+hDKXJioaldXgjUkK642M4UwtBV8ob2xJNDd2ZhwLnoQdeXeGADbkpy\n"
  "rqXRfboQnoZsG4q5WTP468SQvvG5\n"
  "-----END CERTIFICATE-----\n";


// ------------------------------------------------------------
// Internal constants
// ------------------------------------------------------------

#define CERT_NAME_ROOT_CA      "aws_root_ca"
#define CERT_NAME_CLIENT_CERT  "client_cert"
#define CERT_NAME_CLIENT_KEY   "client_key"

#define USECMNG_TYPE_ROOT_CA      0
#define USECMNG_TYPE_CLIENT_CERT  1
#define USECMNG_TYPE_CLIENT_KEY   2

#define USECPRF_PARAM_VALIDATION  0
#define USECPRF_PARAM_TLS_VERSION 1
#define USECPRF_PARAM_CIPHER      2
#define USECPRF_PARAM_CA_NAME     3
#define USECPRF_PARAM_SNI_HOST    4
#define USECPRF_PARAM_CERT_NAME   5
#define USECPRF_PARAM_KEY_NAME    6

// Shared scratch buffer for building AT commands
static char prov_at_buf[160];


// ------------------------------------------------------------
// Private helpers
// ------------------------------------------------------------

static void delete_cert(uint8_t type, const char *name)
{
  // Ignore result — cert may not exist yet on first provisioning
  const uint32_t len = (uint32_t)snprintf(prov_at_buf, sizeof(prov_at_buf), "AT+USECMNG=2,%u,\"%s\"\r", type, name) + 1;
  cell_at_cmd(prov_at_buf, len, 2000);
}

static uint8_t store_cert(uint8_t type, const char *name, const char *pem)
{
  // Build: AT+USECMNG=0,<type>,"<name>",<byte_count>
  const uint32_t pem_len = (uint32_t)strlen(pem);
  const uint32_t cmd_len = (uint32_t)snprintf(prov_at_buf, sizeof(prov_at_buf), "AT+USECMNG=0,%u,\"%s\",%lu\r", type, name, pem_len) + 1;

  // Send command and wait for the '>' data-entry prompt
  if (!cell_at_cert_write_begin(prov_at_buf, cmd_len, 5000))
    return 0;

  // Stream the PEM bytes (must match pem_len exactly)
  cell_at_cert_write_data(pem, pem_len);

  // Block until OK (+USECMNG URC precedes it; both arrive in one idle-line event)
  return cell_at_cert_write_end(15000);
}

static uint8_t profile_set_int(uint8_t param, uint32_t value)
{
  const uint32_t len = (uint32_t)snprintf(prov_at_buf, sizeof(prov_at_buf), "AT+USECPRF=%u,%u,%lu\r", CELL_MQTT_TLS_PROFILE, param, value) + 1;
  return cell_at_cmd(prov_at_buf, len, 1000);
}

static uint8_t profile_set_str(uint8_t param, const char *value)
{
  const uint32_t len = (uint32_t)snprintf(prov_at_buf, sizeof(prov_at_buf), "AT+USECPRF=%u,%u,\"%s\"\r", CELL_MQTT_TLS_PROFILE, param, value) + 1;
  return cell_at_cmd(prov_at_buf, len, 1000);
}


// ------------------------------------------------------------
// Public API
// ------------------------------------------------------------

uint8_t cell_store_aws_config(void)
{
  // Step 1: Remove any previously stored credentials for clean re-provisioning
  delete_cert(USECMNG_TYPE_ROOT_CA, CERT_NAME_ROOT_CA);
  delete_cert(USECMNG_TYPE_CLIENT_CERT, CERT_NAME_CLIENT_CERT);
  delete_cert(USECMNG_TYPE_CLIENT_KEY, CERT_NAME_CLIENT_KEY);

  // Step 2: Store the three TLS credentials into modem NVM
  if (!store_cert(USECMNG_TYPE_ROOT_CA, CERT_NAME_ROOT_CA, AWS_ROOT_CA_PEM)) return 0;
  if (!store_cert(USECMNG_TYPE_CLIENT_CERT, CERT_NAME_CLIENT_CERT, DEVICE_CERT_PEM)) return 0;
  if (!store_cert(USECMNG_TYPE_CLIENT_KEY, CERT_NAME_CLIENT_KEY, DEVICE_KEY_PEM)) return 0;

  if (!profile_set_int(USECPRF_PARAM_VALIDATION, 1)) return 0;  // Validate server cert against root CA
  if (!profile_set_int(USECPRF_PARAM_TLS_VERSION, 3)) return 0;  // Require TLS 1.2 minimum
  if (!profile_set_int(USECPRF_PARAM_CIPHER, 0)) return 0;  // Automatic cipher suite selection
  if (!profile_set_str(USECPRF_PARAM_CA_NAME, CERT_NAME_ROOT_CA)) return 0;  // Trusted root CA
  if (!profile_set_str(USECPRF_PARAM_SNI_HOST, CELL_MQTT_BROKER_HOST)) return 0;  // SNI / server hostname validation
  if (!profile_set_str(USECPRF_PARAM_CERT_NAME, CERT_NAME_CLIENT_CERT)) return 0;  // Client certificate
  if (!profile_set_str(USECPRF_PARAM_KEY_NAME, CERT_NAME_CLIENT_KEY)) return 0;  // Client private key

  return 1;
}


// ------------------------------------------------------------
// Standalone provisioning entry point
// ------------------------------------------------------------

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

  // Configure a 1ms SysTick interrupt timer for HAL_Delay
  SystemCoreClockUpdate();
  SysTick_Config(SystemCoreClock / 1000UL);
  NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), TICK_INT_PRIORITY, 0U));
  uwTickPrio = TICK_INT_PRIORITY;

  // Enable GPIO clocks and configure the GPS status LED as push-pull output
  SET_BIT(RCC->AHB4ENR, RCC_AHB4ENR_GPIOAEN | RCC_AHB4ENR_GPIOBEN | RCC_AHB4ENR_GPIODEN);
  (void)READ_BIT(RCC->AHB4ENR, RCC_AHB4ENR_GPIODEN);
  const uint32_t led_pos = 32U - (uint32_t)__builtin_clz(LED_GPS_STATUS_Pin) - 1U;
  CLEAR_BIT(LED_GPS_STATUS_GPIO_Port->PUPDR, (3UL << (led_pos * 2U)));
  MODIFY_REG(LED_GPS_STATUS_GPIO_Port->MODER, (3UL << (led_pos * 2U)), (1UL << (led_pos * 2U)));

  // Wait for at least 1.5s before powering on the cellular modem
  HAL_Delay(1600);
  cell_power_on();

  // Read non-volatile configuration (needed by cell_init internals)
  chip_read_config();

  // Initialize the cellular modem (sets up UART, DMA, timers, and modem config)
  cell_init();

  // Provision the TLS certificates and security profile into modem NVM
  const uint8_t success = cell_store_aws_config();

  // Signal result via GPS status LED: steady ON = success, rapid blink = failure
  if (success)
  {
    WRITE_REG(LED_GPS_STATUS_GPIO_Port->BSRR, LED_GPS_STATUS_Pin);
    while (1) cpu_feed_watchdog();
  }
  else
  {
    while (1)
    {
      WRITE_REG(LED_GPS_STATUS_GPIO_Port->BSRR, LED_GPS_STATUS_Pin);
      HAL_Delay(100);
      WRITE_REG(LED_GPS_STATUS_GPIO_Port->BSRR, (uint32_t)LED_GPS_STATUS_Pin << 16U);
      HAL_Delay(100);
      cpu_feed_watchdog();
    }
  }
  return 0;
}

#endif  // #ifdef CORE_CM4
