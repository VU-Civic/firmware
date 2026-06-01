#ifdef CORE_CM4

// Header Inclusions ---------------------------------------------------------------------------------------------------

#include <arm_math.h>
#include "cellular.h"
#include "system.h"


// Cellular Data Type Definitions --------------------------------------------------------------------------------------

#define CELL_UART_CONCAT(a,t,n,c)         cell_uart_concat(a,t,n,c)
#define cell_uart_concat(a,t,n,c)         a ## t ## n ## c

#define CELL_UART_IRQHandler              cell_uart_irq1(CELL_UART_TYPE, CELL_UART_NUMBER)
#define cell_uart_irq1(t,n)               cell_uart_irq(t,n)
#define cell_uart_irq(t,n)                t ## n ## _IRQHandler

#define CELL_UART                         cell_uart1(CELL_UART_TYPE, CELL_UART_NUMBER)
#define cell_uart1(t,n)                   cell_uart(t,n)
#define cell_uart(t,n)                    t ## n

#define CELL_TIMER_MS_PER_TICK            2U
#define CELL_TIMER_20MS_COUNT             5U
#define CELL_TIMER_1MIN_COUNT             15360U

#define CELL_MODEM_STATUS_BOOT_TIME_MS    16000
#define CELL_MODEM_DEFAULT_BAUD_RATE      115200
#define CELL_MODEM_DESIRED_BAUD_RATE      921600

#define CELL_MAX_AT_COMMAND_SIZE          3072
#define CELL_MAX_RX_PACKET_SIZE           1024
#define CELL_MAX_NETWORK_RESPONSE_MS      5500

#define CLIENT_ID_IMEI_OFFSET             14
#define SIM_CARD_ID_MIN_LENGTH            18
#define SIM_CARD_ID_MAX_LENGTH            22
#define UMQTTSN_ERROR_CLASS_CODE          14
#define MQTTSN_PUBLISH_MSG_QOS_OFFSET     14

#define MQTT_SUBSCRIBE_MSG_MAX_SIZE       60
#define MQTT_TOPIC_MAX_SIZE               40

#define CELL_GREETING_MSG                 "+UUSTATUS: READY\r"
#define CELL_ACK_MSG                      "OK\r"
#define CELL_ERROR_MSG                    "ERROR\r"
#define CELL_IMEI_MSG                     "+CGSN:"
#define CELL_BAUD_RATE_MSG                "+IPR:"
#define CELL_CME_ERROR_MSG                "+CME ERROR:"
#define CELL_CGREG_RESPONSE_MSG           "+CGREG:"
#define CELL_CEREG_RESPONSE_MSG           "+CEREG:"
#define CELL_COPS_RESPONSE_MSG            "+COPS:"
#define CELL_PDP_ACTIVE_MSG               "+CGACT:"
#define CELL_SIM_CARD_ID_MSG              "+CCID:"
#define CELL_NETWORK_EVENT_MSG            "+CGEV:"
#define CELL_TEMP_ALERT_MSG               "+UUSTS:"
#define CELL_SIGNAL_QUALITY_MSG           "+CSQ:"
#define CELL_SIGNAL_QUALITY_EXT_MSG       "+CESQ:"
#define CELL_MQTT_MSG                     "+UUMQTTC:"
#define CELL_MQTT_SN_MSG                  "+UUMQTTSNC:"
#define CELL_MQTT_ERROR_MSG               "+UMQTTSNER:"

#define CELL_SOFT_RESET_MSG               "AT+CFUN=16\r"
#define CELL_GET_SIM_CARD_ID_MSG          "AT+CCID?\r"
#define CELL_DISABLE_RADIO_MSG            "AT+CFUN=0\r"
#define CELL_ENABLE_RADIO_MSG             "AT+CFUN=1\r"
#define CELL_REBOOT_RADIO_MSG             "AT+CFUN=1,1\r"
#define CELL_GET_FW_VERSION_MSG           "AT+CGMR\r"
#define CELL_DISABLE_ECHO_MSG             "ATE0\r"
#define CELL_USE_NUMERIC_ERRORS_MSG       "ATV0\r"
#define CELL_GET_SIGNAL_QUALITY_MSG       "AT+CSQ\r"
#define CELL_GET_SIGNAL_QUALITY_EXT_MSG   "AT+CESQ\r"
#define CELL_USE_NUMERIC_MT_ERRORS_MSG    "AT+CMEE=1\r"
#define CELL_GET_BAUD_RATE_MSG            "AT+IPR?\r"
#define CELL_SET_BAUD_RATE_MSG            "AT+IPR=" STRINGIZE(CELL_MODEM_DESIRED_BAUD_RATE) "\r"
#define CELL_ENABLE_TEMP_CTRL_MSG         "AT+USTS=2\r"
#define CELL_DISABLE_PSM_SLEEP_MSG        "AT+SYSNV=1,\"psm_sleep_en\",0\r"
#define CELL_PDP_CONFIG_MSG               "AT+CGDCONT=1,\"IPV4V6\",\"" STRINGIZE(CELL_MQTT_BROKER_APN) "\"\r"
#define CELL_INIT_BEARER_CFG_MSG          "AT+CFGDFTPDN=3,0,\"" STRINGIZE(CELL_MQTT_BROKER_APN) "\"\r"
#define CELL_PDP_ACTIVATE_MSG             "AT+CGACT=1,1\r"
#define CELL_PDP_DEACTIVATE_MSG           "AT+CGACT=0,1\r"
#define CELL_POLL_PDP_ACTIVE_MSG          "AT+CGACT?\r"
#define CELL_POLL_CGREG_MSG               "AT+CGREG?\r"
#define CELL_POLL_CEREG_MSG               "AT+CEREG?\r"
#define CELL_POLL_COPS_MSG                "AT+COPS?\r"
#define CELL_REPORT_REG_EVENTS_MSG        "AT+CMER=2,0,0,2,1\r"
#define CELL_REPORT_REG_ERRORS_MSG        "AT+CGEREP=2,1\r"
#define CELL_SUBSCRIBE_CREG_MSG           "AT+CREG=1\r"
#define CELL_SUBSCRIBE_CGREG_MSG          "AT+CGREG=1\r"
#define CELL_SUBSCRIBE_CEREG_MSG          "AT+CEREG=1\r"
#define CELL_RETRIEVE_IMEI_MSG            "AT+CGSN=1\r"
#define CELL_RETRIEVE_IMSI_MSG            "AT+CIMI\r"
#define CELL_ENABLE_STATUS_LED_MSG        "AT+UGPIOC=16,2\r"
#define CELL_FACTORY_RESET1_MSG           "AT+UFACTORY=2,1\r"
#define CELL_FACTORY_RESET2_MSG           "AT&F0\r"
#define CELL_STORE_PERSISTENT_MSG         "AT&W0\r"

#define CELL_MQTTSN_SET_CLIENT_ID_MSG     "AT+UMQTTSN=0,\"000000000000000\"\r"
#define CELL_MQTTSN_SET_RESP_TIMEOUT_MSG  "AT+UMQTTSN=11," STRINGIZE(CELL_SERVER_RESPONSE_TIMEOUT_SECONDS) "\r"
#define CELL_SET_SERVER_ADDR_MSG          "AT+UMQTTSN=2,\"" CELL_MQTT_BROKER_HOST "\"," STRINGIZE(CELL_MQTT_BROKER_PORT) "\r"
#define CELL_SET_CONN_TIMEOUT_MSG         "AT+UMQTTSN=8," STRINGIZE(CELL_CONNECTION_TIMEOUT_SECONDS) "\r"
#define CELL_SET_CLEAN_SESSION_MSG        "AT+UMQTTSN=10,1\r"
#define CELL_READ_MQTTSN_CONFIG_MSG       "AT+UMQTTSN?\r"
#define CELL_SAVE_MQTTSN_CONFIG_MSG       "AT+UMQTTSNNV=2\r"
#define CELL_LOAD_MQTTSN_CONFIG_MSG       "AT+UMQTTSNNV=1\r"
#define CELL_MQTTSN_SET_AUTO_PING_MSG     "AT+UMQTTSNC=10,0\r"
#define CELL_MQTTSN_CONNECT_MSG           "AT+UMQTTSNC=1\r"
#define CELL_MQTTSN_REGISTER_MSG          "AT+UMQTTSNC=2,"
#define CELL_MQTTSN_PUBLISH_INFO_MSG      "AT+UMQTTSNC=4,0,0,1,1,\"" STRINGIZE(CELL_MQTT_DEVICES_TOPIC) "\",\""
#define CELL_MQTTSN_PUBLISH_ALERT_MSG     "AT+UMQTTSNC=4,1,0,1,1,\"" STRINGIZE(CELL_MQTT_ALERT_TOPIC) "\",\""
#define CELL_MQTTSN_PUBLISH_AUDIO_MSG     "AT+UMQTTSNC=4,0,0,0,1,\"" STRINGIZE(CELL_MQTT_EVIDENCE_TOPIC) "\",\""
#define CELL_MQTTSN_PUB_BINARY_INFO_MSG   "AT+UMQTTSNC=12,1,0,1,\"" STRINGIZE(CELL_MQTT_DEVICES_TOPIC) "\","
#define CELL_MQTTSN_PUB_BINARY_ALERT_MSG  "AT+UMQTTSNC=12,1,0,1,\"" STRINGIZE(CELL_MQTT_ALERT_TOPIC) "\","
#define CELL_MQTTSN_PUB_BINARY_AUDIO_MSG  "AT+UMQTTSNC=12,0,0,1,\"" STRINGIZE(CELL_MQTT_EVIDENCE_TOPIC) "\","
#define CELL_MQTTSN_SUBSCRIBE_MSG         "AT+UMQTTSNC=5,1,1,\"" STRINGIZE(CELL_MQTT_CONTROL_TOPIC) "\"\r"
#define CELL_MQTTSN_READ_MSG              "AT+UMQTTSNC=9,1\r"
#define CELL_GET_MQTTSN_ERROR_MSG         "AT+UMQTTSNER\r"

#define CELL_READ_MQTT_CONFIG_MSG         "AT+UMQTT?\r"
#define CELL_SAVE_MQTT_CONFIG_MSG         "AT+UMQTTNV=2\r"
#define CELL_LOAD_MQTT_CONFIG_MSG         "AT+UMQTTNV=1\r"
#define CELL_MQTT_SET_CLIENT_ID_MSG       "AT+UMQTT=0,\"000000000000000\"\r"
#define CELL_MQTT_SET_SERVER_ADDR_MSG     "AT+UMQTT=2,\"" CELL_MQTT_BROKER_HOST "\"," STRINGIZE(CELL_MQTT_BROKER_PORT) "\r"
#define CELL_MQTT_SET_KEEPALIVE_MSG       "AT+UMQTT=10," STRINGIZE(CELL_CONNECTION_TIMEOUT_SECONDS) ",0\r"
#define CELL_MQTT_SET_SECURE_MSG          "AT+UMQTT=11,1," STRINGIZE(CELL_MQTT_TLS_PROFILE) "\r"
#define CELL_MQTT_SET_CLEAN_SESSION_MSG   "AT+UMQTT=12,1\r"
#define CELL_MQTT_SET_RESP_TIMEOUT_MSG    "AT+UMQTT=13," STRINGIZE(CELL_SERVER_RESPONSE_TIMEOUT_SECONDS) "\r"
#define CELL_MQTT_SET_AUTO_PING_MSG       "AT+UMQTTC=8,0\r"
#define CELL_MQTT_CONNECT_MSG             "AT+UMQTTC=1\r"
#define CELL_MQTT_READ_MSG                "AT+UMQTTC=6,1\r"

#define CGEV_PDN_ACT_MSG                  "ME PDN ACT 1"
#define CGEV_PDN_DEACT_MSG                "ME PDN DEACT 1"

#define AT_RESPONSE_OK                    "0\r"
#define AT_RESPONSE_NO_CARRIER            "3\r"
#define AT_RESPONSE_ERROR                 "4\r"
#define AT_RESPONSE_COMMAND_ABORTED       "3000\r"

typedef struct
{
   evidence_message_t message;
   uint16_t length;
} cell_audio_tx_item_t;

typedef struct
{
   const uint8_t *data;
   uint32_t len;
   char qos;
} pub_params_t;

#ifdef USE_MQTT_SN

typedef enum
{
   MQTT_DISCONNECT = 0,
   MQTT_CONNECT = 1,
   MQTT_REGISTER = 2,
   MQTT_UNKNOWN = 3,
   MQTT_PUBLISH = 4,
   MQTT_SUBSCRIBE = 5,
   MQTT_UNSUBSCRIBE = 6,
   MQTT_WILL_TOPIC = 7,
   MQTT_WILL_UPDATE = 8,
   MQTT_READ = 9,
   MQTT_PING = 10,
   MQTT_PUBLISH_FILE = 11,
   MQTT_PUBLISH_BINARY = 12,
   MQTT_DONE
} mqtt_operation_t;

#else

typedef enum
{
   MQTT_DISCONNECT = 0,
   MQTT_CONNECT = 1,
   MQTT_PUBLISH = 2,
   MQTT_PUBLISH_FILE = 3,
   MQTT_SUBSCRIBE = 4,
   MQTT_UNSUBSCRIBE = 5,
   MQTT_READ = 6,
   MQTT_SET_READ_MODE = 7,
   MQTT_PING = 8,
   MQTT_PUBLISH_BINARY = 9,
   MQTT_DONE
} mqtt_operation_t;

#endif  // #ifdef USE_MQTT_SN

typedef enum
{
   CELL_STEP_IDLE = 0,
   CELL_STEP_WAIT_SIG_QUAL,
#ifdef CELL_MQTT_USE_BINARY_PUBLISH
   CELL_STEP_WAIT_PUB_PROMPT,
#endif
   CELL_STEP_WAIT_PUB_CMD_ACK,
   CELL_STEP_WAIT_PUB_NET_ACK,
   CELL_STEP_WAIT_MSG_READ,
} cell_step_t;

typedef enum
{
   CELL_PUB_DEVICE_INFO = 0,
   CELL_PUB_ALERT,
   CELL_PUB_AUDIO,
} cell_pub_type_t;


// Static Cellular Variables -------------------------------------------------------------------------------------------

#define CELL_RX_BUFFER_SIZE_HALF       (2 * CELL_MAX_RX_PACKET_SIZE)
#define CELL_RX_BUFFER_SIZE_FULL       (2 * CELL_RX_BUFFER_SIZE_HALF)

#define CELL_ALERT_TX_RING_SIZE        4
#define CELL_AUDIO_TX_RING_SIZE        3

__attribute__ ((aligned (4)))
static char cell_rx_buffer[CELL_RX_BUFFER_SIZE_FULL];

static dma_int_registers_t *dma_int_registers;

#ifdef CELL_MQTT_USE_BINARY_PUBLISH
static char publish_info_message[MQTT_SUBSCRIBE_MSG_MAX_SIZE];
static char publish_alert_message[MQTT_SUBSCRIBE_MSG_MAX_SIZE];
static char publish_audio_message[MQTT_SUBSCRIBE_MSG_MAX_SIZE];
static uint32_t info_message_length, alert_prefix_length, audio_prefix_length, cell_pending_cmd_len;
#endif

static evidence_message_t evidence_message;
static alert_message_t cell_alert_tx_ring[CELL_ALERT_TX_RING_SIZE];
static cell_audio_tx_item_t cell_audio_tx_ring[CELL_AUDIO_TX_RING_SIZE];
static char publish_message_buffer[CELL_MAX_AT_COMMAND_SIZE];
static volatile uint32_t cell_alert_tx_head, cell_alert_tx_count;
static volatile uint32_t cell_audio_tx_head, cell_audio_tx_count;
static volatile cell_step_t cell_step = CELL_STEP_IDLE;
static volatile cell_pub_type_t cell_pending_pub = CELL_PUB_DEVICE_INFO;
static volatile mqtt_operation_t mqtt_operation_awaiting_ack = MQTT_DONE;
static volatile uint8_t mqtt_result = 0, pending_messages = 0, connectivity_changed = 0, signal_power = 255;
static volatile uint8_t cell_modem_available = 0, configure_modem = 0, mqtt_connected = 0, signal_quality = 255;
static volatile uint8_t valid_cgreg = 0, valid_cereg = 0, valid_pdp = 0, temperature_alert = 0, reading_imsi = 0;
static volatile uint8_t cell_busy = 0, device_info_update = 0, mqtt_configured = 0, prompt_received = 0;
static volatile uint8_t device_update_timer_count = 0, bad_network_conn_timer_count = 0, bad_pdp_timer_count = 0;
static volatile uint8_t command_acked = 0, command_nacked = 0, timed_out = 0, in_holdoff_period = 0;
static volatile uint8_t mqtt_connect_pending = 0, bad_mqtt_conn_timer_count = 0, mqtt_subscribed = 0;
static volatile char sim_id[SIM_CARD_ID_MAX_LENGTH+1], *incoming_message = 0;
static volatile uint32_t incoming_message_length, baud_rate = 0, cme_error = 0;

#ifndef USE_MQTT_SN
static char broker_topic_info[MQTT_TOPIC_MAX_SIZE], broker_topic_alert[MQTT_TOPIC_MAX_SIZE], broker_topic_audio[MQTT_TOPIC_MAX_SIZE];
#endif  // #ifndef USE_MQTT_SN


// Private Helper Functions --------------------------------------------------------------------------------------------

static char* find_start_of_message(char* msg, uint32_t msg_preamble_size, uint16_t* max_msg_len)
{
   // Search for the first non-space character
   msg += msg_preamble_size;
   uint16_t bytes_remaining = *max_msg_len - msg_preamble_size;
   while (bytes_remaining && (*msg == ' '))
   {
      --bytes_remaining;
      ++msg;
   }

   // Update the maximum remaining message length and return the new message start
   *max_msg_len = bytes_remaining;
   return msg;
}

static char* find_end_of_message(char* msg, uint16_t* max_msg_len)
{
   // Search for the first end-of-message character
   uint16_t bytes_remaining = *max_msg_len;
   while (bytes_remaining && (*msg != '\r'))
   {
      --bytes_remaining;
      ++msg;
   }

   // End the message with a NULL character and return the new message end
   if (bytes_remaining)
   {
      *(msg++) = '\0';
      --bytes_remaining;
   }
   *max_msg_len = bytes_remaining;
   return msg;
}

static void set_command_timeout(uint32_t timeout_ticks)
{
   // Ensure that the timeout counter is stopped and reset
   __set_PRIMASK(1) ;
   SET_BIT(RCC->APB4RSTR, RCC_APB4RSTR_LPTIM4RST);
   CLEAR_BIT(RCC->APB4RSTR, RCC_APB4RSTR_LPTIM4RST);
   MODIFY_REG(LPTIM4->CFGR, (LPTIM_CFGR_CKSEL | LPTIM_CFGR_TRIGEN | LPTIM_CFGR_PRELOAD | LPTIM_CFGR_WAVPOL | LPTIM_CFGR_PRESC | LPTIM_CFGR_COUNTMODE), LPTIM_PRESCALER_DIV64);
   WRITE_REG(LPTIM4->IER, LPTIM_IT_ARRM);
   SET_BIT(LPTIM4->CR, LPTIM_CR_ENABLE);
   __set_PRIMASK(0);

   // Initiate the timeout counter for the specified number of ticks
   timed_out = 0;
   WRITE_REG(LPTIM4->ARR, timeout_ticks);
   while (READ_REG(LPTIM4->ARR) != timeout_ticks);
   SET_BIT(LPTIM4->CR, LPTIM_CR_SNGSTRT);
}

static void cell_toggle_power(uint8_t power_on)
{
   // Assert the nPWR_ON pin for at least 2s to switch on or at least 3.1s to switch off
   WRITE_REG(CELL_NPWR_ON_GPIO_Port->BSRR, (uint32_t)CELL_NPWR_ON_Pin << 16U);
   HAL_Delay(power_on ? 2050 : 3150);
   WRITE_REG(CELL_NPWR_ON_GPIO_Port->BSRR, CELL_NPWR_ON_Pin);
}

static void cell_send_command(char *command, uint32_t command_len)
{
   // Wait until the hold-off time has elapsed between command reception and transmission
   while (in_holdoff_period)
      cpu_sleep();

   // Initiate command transfer using DMA
   WRITE_REG(dma_int_registers->IFCR, (0x3FUL << 22));
   WRITE_REG(DMA2_Stream3->M0AR, (uint32_t)command);
   WRITE_REG(DMA2_Stream3->NDTR, command_len-1);
   SET_BIT(DMA2_Stream3->CR, DMA_SxCR_EN);
}

static uint8_t cell_send_command_await_response(char *command, uint32_t command_len, uint32_t timeout_ms)
{
   // Send the command and wait until an ACK or NACK is received or the timeout is reached
   command_acked = command_nacked = 0;
   cell_send_command(command, command_len);
   set_command_timeout(timeout_ms / CELL_TIMER_MS_PER_TICK);
   while (!command_acked && !command_nacked && !timed_out)
   {
      cpu_feed_watchdog();
      cpu_sleep();
   }
   return command_acked;
}

static void cell_mqtt_connect(void)
{
   // Do not attempt another connect while a prior attempt is still in progress
   if (cell_busy)
   {
      connectivity_changed = 1;
      return;
   }
   else if (mqtt_connect_pending)
      return;

   // Disable auto-pinging since we should already be sending device status messages regularly
#ifdef USE_MQTT_SN
   cell_send_command_await_response(CELL_MQTTSN_SET_AUTO_PING_MSG, sizeof(CELL_MQTTSN_SET_AUTO_PING_MSG), 1000);
#else
   cell_send_command_await_response(CELL_MQTT_SET_AUTO_PING_MSG, sizeof(CELL_MQTT_SET_AUTO_PING_MSG), 1000);
#endif

   // Issue the connect command; the CONNACK URC arrives asynchronously via the UART ISR
   if (!mqtt_connected)
   {
      mqtt_operation_awaiting_ack = MQTT_CONNECT;
#ifdef USE_MQTT_SN
      if (cell_send_command_await_response(CELL_MQTTSN_CONNECT_MSG, sizeof(CELL_MQTTSN_CONNECT_MSG), 3000))
#else
      if (cell_send_command_await_response(CELL_MQTT_CONNECT_MSG, sizeof(CELL_MQTT_CONNECT_MSG), 5000))
#endif
         mqtt_connect_pending = 1;
      else
         mqtt_operation_awaiting_ack = MQTT_DONE;
   }
}

static void cell_mqtt_subscribe(void)
{
   // Do not attempt to subscribe if modem is currently busy with another task
   if (cell_busy)
   {
      connectivity_changed = 1;
      return;
   }

   // Create the relevant subscription messages
#ifndef USE_MQTT_SN
   for (int i = 0; i < 2; ++i)
   {
      char mqtt_subscribe_msg[MQTT_SUBSCRIBE_MSG_MAX_SIZE];
      if (i == 0)
         snprintf(mqtt_subscribe_msg, sizeof(mqtt_subscribe_msg), "AT+UMQTTC=4,1,\"" CELL_MQTT_TOPIC_NAMESPACE "/%.*s/" CELL_MQTT_CONTROL_TOPIC "\"\r", CELL_IMEI_LENGTH, (const char*)data.packets[0].imei);
      else
         snprintf(mqtt_subscribe_msg, sizeof(mqtt_subscribe_msg), "AT+UMQTTC=4,1,\"" CELL_MQTT_TOPIC_NAMESPACE "/-1/" CELL_MQTT_CONTROL_TOPIC "\"\r");
#endif

      // Try to subscribe in a loop until success or the network disconnects
      cell_busy = 1;
      do
      {
         // Issue the subscription command and wait until it is acknowledged by the network
         mqtt_result = 0;
         mqtt_operation_awaiting_ack = MQTT_SUBSCRIBE;
#ifdef USE_MQTT_SN
         cell_send_command_await_response(CELL_MQTTSN_SUBSCRIBE_MSG, sizeof(CELL_MQTTSN_SUBSCRIBE_MSG), 1000);
#else
         cell_send_command_await_response(mqtt_subscribe_msg, (uint32_t)strlen(mqtt_subscribe_msg) + 1, 1000);
#endif
         set_command_timeout(5000 / CELL_TIMER_MS_PER_TICK);
         while (mqtt_connected && !timed_out && (mqtt_operation_awaiting_ack != MQTT_DONE))
         {
            cpu_feed_watchdog();
            cpu_sleep();
         }
      } while (!mqtt_result && mqtt_connected);
      mqtt_subscribed = mqtt_result;
      cell_busy = 0;

#ifndef USE_MQTT_SN
   }
#endif
}

#ifndef CELL_MQTT_USE_BINARY_PUBLISH

static uint32_t hex_encode_binary_data(char *output, const uint8_t *input, uint32_t input_num_bytes)
{
   // Format the input data bytes as hexadecimal output
   uint32_t *output32 = (uint32_t*)output;
   for (uint32_t i = input_num_bytes / sizeof(uint16_t); i > 0; --i)
   {
      uint32_t x = (input[1] << 16U) | input[0];
      x = (((x & 0x00f000f0) >> 4) | ((x & 0x000f000f) << 8)) + 0x06060606;
      const uint32_t m = ((x & 0x10101010) >> 4) + 0x7f7f7f7f;
      *(output32++) = x + ((m & 0x2a2a2a2a) | (~m & 0x31313131));
      input += 2;
   }
   return input_num_bytes * 2;
}

static uint32_t base64_encode_binary_data(char* output, const uint8_t* input, uint32_t inputNumBytes)
{
  // The standard Base64 alphabet table
  static const char encodingTable[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

  // Iterate through all input characters in groups of 3 bytes
  uint32_t inIdx = 0, outIdx = 0;
  const uint32_t threeBytePackets = inputNumBytes / 3;
  for (uint32_t i = 0; i < threeBytePackets; ++i, inIdx += 3, outIdx += 4)
  {
    const uint32_t a = input[inIdx], b = input[inIdx + 1], c = input[inIdx + 2];
    const uint32_t triple = (a << 16) + (b << 8) + c;
    output[outIdx] = encodingTable[(triple >> 18) & 0x3F];
    output[outIdx + 1] = encodingTable[(triple >> 12) & 0x3F];
    output[outIdx + 2] = encodingTable[(triple >> 6) & 0x3F];
    output[outIdx + 3] = encodingTable[triple & 0x3F];
  }

  // Pad and encode any remaining bytes
  const uint32_t remaining = inputNumBytes - inIdx;
  if (remaining == 1)
  {
    const uint32_t triple = (uint32_t)input[inIdx] << 16;
    output[outIdx] = encodingTable[(triple >> 18) & 0x3F];
    output[outIdx + 1] = encodingTable[(triple >> 12) & 0x3F];
    output[outIdx + 2] = '=';
    output[outIdx + 3] = '=';
    outIdx += 4;
  }
  else if (remaining == 2)
  {
    const uint32_t triple = ((uint32_t)input[inIdx] << 16) | ((uint32_t)input[inIdx + 1] << 8);
    output[outIdx] = encodingTable[(triple >> 18) & 0x3F];
    output[outIdx + 1] = encodingTable[(triple >> 12) & 0x3F];
    output[outIdx + 2] = encodingTable[(triple >> 6) & 0x3F];
    output[outIdx + 3] = '=';
    outIdx += 4;
  }
  return outIdx;
}

#endif  // #ifndef CELL_MQTT_USE_BINARY_PUBLISH

static void cell_advance_tx_rings(void)
{
   // Advance alert ring buffer if an alert was just sent
   if (cell_pending_pub == CELL_PUB_ALERT)
   {
      if (cell_alert_tx_count)
      {
         cell_alert_tx_head = (cell_alert_tx_head + 1) % CELL_ALERT_TX_RING_SIZE;
         --cell_alert_tx_count;
      }
   }

   // Advance audio ring buffer if an audio packet was just sent
   else if (cell_pending_pub == CELL_PUB_AUDIO)
   {
      if (cell_audio_tx_count)
      {
         cell_audio_tx_head = (cell_audio_tx_head + 1) % CELL_AUDIO_TX_RING_SIZE;
         --cell_audio_tx_count;
      }
   }
}

static pub_params_t cell_get_pub_params(cell_pub_type_t pub_type)
{
   pub_params_t params = { NULL, 0, '0' };
   switch (pub_type)
   {
      case CELL_PUB_DEVICE_INFO:
         params.data = (const uint8_t*)&device_info;
         params.len = sizeof(device_info_t);
         params.qos = device_info.device_config.mqtt_device_info_qos ? '1' : '0';
         break;
      case CELL_PUB_ALERT: {
         const alert_message_t *alert = &cell_alert_tx_ring[cell_alert_tx_head];
         params.data = (const uint8_t*)alert;
         params.len = sizeof(alert_message_t) - sizeof(alert->events) + (sizeof(event_info_t) * alert->num_events);
         params.qos = device_info.device_config.mqtt_alert_qos ? '1' : '0';
         break;
      }
      case CELL_PUB_AUDIO: {
         const cell_audio_tx_item_t *item = &cell_audio_tx_ring[cell_audio_tx_head];
         params.data = (const uint8_t*)&item->message;
         params.len = offsetof(evidence_message_t, data) + item->length;
         params.qos = device_info.device_config.mqtt_audio_qos ? '1' : '0';
         break;
      }
   }
   return params;
}

static void cell_start_nonblocking_publish(cell_pub_type_t pub_type)
{
   // Set up the common publishing state
   cell_pending_pub = pub_type;
   cell_busy = 1;
   mqtt_result = 0;
   command_acked = command_nacked = 0;
   const pub_params_t params = cell_get_pub_params(pub_type);

#ifdef CELL_MQTT_USE_BINARY_PUBLISH

   // Transmit the command header and wait for the binary data prompt
   prompt_received = 0;
   mqtt_operation_awaiting_ack = MQTT_PUBLISH_BINARY;
   switch (pub_type)
   {
      case CELL_PUB_DEVICE_INFO:
         cell_send_command(publish_info_message, info_message_length);
         break;
      case CELL_PUB_ALERT: {
         const uint32_t cmd_len = (uint32_t)snprintf(publish_message_buffer, sizeof(publish_message_buffer), "%.*s%u\r", (int)alert_prefix_length, publish_alert_message, (unsigned)params.len) + 1;
         cell_send_command(publish_message_buffer, cmd_len);
         break;
      }
      case CELL_PUB_AUDIO: {
         // Audio length varies per packet so build the full command dynamically
         const uint32_t cmd_len = (uint32_t)snprintf(publish_message_buffer, sizeof(publish_message_buffer), "%.*s%u\r", (int)audio_prefix_length, publish_audio_message, (unsigned)params.len) + 1;
         cell_send_command(publish_message_buffer, cmd_len);
         break;
      }
   }
   cell_pending_cmd_len = params.len;
   set_command_timeout(1000 / CELL_TIMER_MS_PER_TICK);
   cell_step = CELL_STEP_WAIT_PUB_PROMPT;

#else

   // Build the encoded command in the shared buffer and transmit it
   uint32_t message_len = 0;
   mqtt_operation_awaiting_ack = MQTT_PUBLISH;

#ifdef USE_MQTT_SN

   // Build up the publish command
   static const char* const mqttsn_prefixes[3] = { CELL_MQTTSN_PUBLISH_INFO_MSG, CELL_MQTTSN_PUBLISH_ALERT_MSG, CELL_MQTTSN_PUBLISH_AUDIO_MSG };
   static const uint32_t mqttsn_prefix_sizes[3] = { sizeof(CELL_MQTTSN_PUBLISH_INFO_MSG), sizeof(CELL_MQTTSN_PUBLISH_ALERT_MSG), sizeof(CELL_MQTTSN_PUBLISH_AUDIO_MSG) };
   const char *prefix = mqttsn_prefixes[pub_type];
   const uint32_t prefix_size = mqttsn_prefix_sizes[pub_type];
   arm_copy_q7((q7_t*)prefix, (q7_t*)publish_message_buffer, prefix_size);
   publish_message_buffer[MQTTSN_PUBLISH_MSG_QOS_OFFSET] = params.qos;
   const uint32_t encoded = (pub_type == CELL_PUB_AUDIO) ?
      base64_encode_binary_data(publish_message_buffer + prefix_size - 1, params.data, params.len) :
      hex_encode_binary_data(publish_message_buffer + prefix_size - 1, params.data, params.len);
   arm_copy_q7((q7_t*)"\"\r", (q7_t*)publish_message_buffer + prefix_size - 1 + encoded, 2);
   message_len = 2 + prefix_size + encoded;

#else

   // Build up the publish command
   static const char * const broker_topics[3] = { broker_topic_info, broker_topic_alert, broker_topic_audio };
   const uint32_t pfx = (uint32_t)snprintf(publish_message_buffer, sizeof(publish_message_buffer), "AT+UMQTTC=2,%c,0,\"%s\",\"", params.qos, broker_topics[pub_type]);
   const uint32_t encoded = (pub_type == CELL_PUB_AUDIO) ?
      base64_encode_binary_data(publish_message_buffer + pfx, params.data, params.len) :
      hex_encode_binary_data(publish_message_buffer + pfx, params.data, params.len);
   arm_copy_q7((q7_t*)"\"\r", (q7_t*)publish_message_buffer + pfx + encoded, 2);
   message_len = pfx + encoded + 3;

#endif  // #ifdef USE_MQTT_SN

   // Transmit the full publish command
   if (mqtt_connected && message_len)
   {
      cell_send_command(publish_message_buffer, message_len);
      set_command_timeout(1000 / CELL_TIMER_MS_PER_TICK);
      cell_step = CELL_STEP_WAIT_PUB_CMD_ACK;
   }
   else
   {
      mqtt_operation_awaiting_ack = MQTT_DONE;
      cell_busy = 0;
      cell_step = CELL_STEP_IDLE;
      cell_advance_tx_rings();
   }

#endif  // #ifdef CELL_MQTT_USE_BINARY_PUBLISH
}

static void cell_reset_modem(uint8_t hard_reset)
{
   // Reset modem according to the requested method
   if (hard_reset)
   {
      // Assert the nRESET pin for at least 50ms to trigger a reset
      WRITE_REG(CELL_NRESET_GPIO_Port->BSRR, (uint32_t)CELL_NRESET_Pin << 16U);
      set_command_timeout(100 / CELL_TIMER_MS_PER_TICK);
      cell_modem_available = 0;
      while (!timed_out) cpu_feed_watchdog();
      WRITE_REG(CELL_NRESET_GPIO_Port->BSRR, CELL_NRESET_Pin);
   }
   else
   {
      cell_send_command_await_response(CELL_SOFT_RESET_MSG, sizeof(CELL_SOFT_RESET_MSG), 1000);
      cell_modem_available = 0;
   }

   // Wait until the modem comes back online or times out
   set_command_timeout(CELL_MODEM_STATUS_BOOT_TIME_MS / CELL_TIMER_MS_PER_TICK);
   while (!cell_modem_available && !timed_out)
   {
      cpu_feed_watchdog();
      cpu_sleep();
   }
}

static void cell_reboot_firmware(uint8_t pdp_reboot)
{
   // Reboot modem according to the requested method
   bad_network_conn_timer_count = bad_pdp_timer_count = 0;
   bad_mqtt_conn_timer_count = mqtt_connect_pending = 0;
   if (pdp_reboot)
   {
      cell_send_command_await_response(CELL_PDP_DEACTIVATE_MSG, sizeof(CELL_PDP_DEACTIVATE_MSG), 1000);
      cell_send_command_await_response(CELL_PDP_ACTIVATE_MSG, sizeof(CELL_PDP_ACTIVATE_MSG), 40000);
   }
   else
   {
      cell_send_command_await_response(CELL_DISABLE_RADIO_MSG, sizeof(CELL_DISABLE_RADIO_MSG), 1000);
      cell_send_command_await_response(CELL_ENABLE_RADIO_MSG, sizeof(CELL_ENABLE_RADIO_MSG), 1000);
   }
}

static uint8_t cell_configure_modem(void)
{
   // Always poll for the device IMEI to use as the device ID
   while ((data.packets[0].imei[0] == 0) && (data.packets[0].imei[1] == 0) && (data.packets[0].imei[2] == 0))
      for (uint32_t retries = 0; (retries < 3) && !cell_send_command_await_response(CELL_RETRIEVE_IMEI_MSG, sizeof(CELL_RETRIEVE_IMEI_MSG), 1000); ++retries);
   device_info.device_id = strtoull((char*)data.packets[0].imei, NULL, 10);

   // Poll for the SIM card ID to verify that a card is present
   for (uint32_t retries = 0; (retries < 20) && !cell_send_command_await_response(CELL_GET_SIM_CARD_ID_MSG, sizeof(CELL_GET_SIM_CARD_ID_MSG), 1000); ++retries)
      HAL_Delay(50);
   const uint8_t sim_present = command_acked;

   // Continue configuring modem if the SIM was validated
   if (sim_present)
   {
      // Update all error and event reporting messages
      for (uint32_t retries = 0; (retries < 3) && !cell_send_command_await_response(CELL_USE_NUMERIC_MT_ERRORS_MSG, sizeof(CELL_USE_NUMERIC_MT_ERRORS_MSG), 500); ++retries);
      for (uint32_t retries = 0; (retries < 3) && !cell_send_command_await_response(CELL_REPORT_REG_ERRORS_MSG, sizeof(CELL_REPORT_REG_ERRORS_MSG), 500); ++retries);
      for (uint32_t retries = 0; (retries < 3) && !cell_send_command_await_response(CELL_SUBSCRIBE_CGREG_MSG, sizeof(CELL_SUBSCRIBE_CGREG_MSG), 500); ++retries);
      for (uint32_t retries = 0; (retries < 3) && !cell_send_command_await_response(CELL_SUBSCRIBE_CEREG_MSG, sizeof(CELL_SUBSCRIBE_CEREG_MSG), 500); ++retries);

      // Read and store the IMSI for device identification
      reading_imsi = 1;
      for (uint32_t retries = 0; (retries < 3) && !cell_send_command_await_response(CELL_RETRIEVE_IMSI_MSG, sizeof(CELL_RETRIEVE_IMSI_MSG), 1000); ++retries);
      reading_imsi = 0;

#ifndef USE_MQTT_SN

      // Build IMEI-based topic strings and AT command buffers (done once per modem init)
      snprintf(broker_topic_info, sizeof(broker_topic_info), CELL_MQTT_TOPIC_NAMESPACE "/%.*s/" CELL_MQTT_DEVICES_TOPIC, CELL_IMEI_LENGTH, (const char*)data.packets[0].imei);
      snprintf(broker_topic_alert, sizeof(broker_topic_alert), CELL_MQTT_TOPIC_NAMESPACE "/%.*s/" CELL_MQTT_ALERT_TOPIC, CELL_IMEI_LENGTH, (const char*)data.packets[0].imei);
      snprintf(broker_topic_audio, sizeof(broker_topic_audio), CELL_MQTT_TOPIC_NAMESPACE "/%.*s/" CELL_MQTT_EVIDENCE_TOPIC, CELL_IMEI_LENGTH, (const char*)data.packets[0].imei);
#ifdef CELL_MQTT_USE_BINARY_PUBLISH
      info_message_length = (uint32_t)snprintf(publish_info_message, sizeof(publish_info_message), "AT+UMQTTC=9,%c,0,\"%s\",%u\r", (device_info.device_config.mqtt_device_info_qos > 0) ? '1' : '0', broker_topic_info, (unsigned)sizeof(device_info_t)) + 1;
      alert_prefix_length = (uint32_t)snprintf(publish_alert_message, sizeof(publish_alert_message), "AT+UMQTTC=9,%c,0,\"%s\",", (device_info.device_config.mqtt_alert_qos > 0) ? '1' : '0', broker_topic_alert);
      audio_prefix_length = (uint32_t)snprintf(publish_audio_message, sizeof(publish_audio_message), "AT+UMQTTC=9,%c,0,\"%s\",", (device_info.device_config.mqtt_audio_qos > 0) ? '1' : '0', broker_topic_audio);
#endif

      // Verify that the MQTT configuration is correctly set up
      for (uint32_t retries = 0; (retries < 3) && !cell_send_command_await_response(CELL_LOAD_MQTT_CONFIG_MSG, sizeof(CELL_LOAD_MQTT_CONFIG_MSG), 1000); ++retries);
      for (uint32_t retries = 0; (retries < 3) && !cell_send_command_await_response(CELL_READ_MQTT_CONFIG_MSG, sizeof(CELL_READ_MQTT_CONFIG_MSG), 1000); ++retries);
      if (!mqtt_configured)
      {
         char set_client_id_msg[] = CELL_MQTT_SET_CLIENT_ID_MSG;
         memcpy((char*)set_client_id_msg + CLIENT_ID_IMEI_OFFSET, (char*)data.packets[0].imei, CELL_IMEI_LENGTH);
         for (uint32_t retries = 0; (retries < 3) && !cell_send_command_await_response(set_client_id_msg, sizeof(set_client_id_msg), 500); ++retries);
         for (uint32_t retries = 0; (retries < 3) && !cell_send_command_await_response(CELL_MQTT_SET_SERVER_ADDR_MSG, sizeof(CELL_MQTT_SET_SERVER_ADDR_MSG), 2000); ++retries);
         for (uint32_t retries = 0; (retries < 3) && !cell_send_command_await_response(CELL_MQTT_SET_KEEPALIVE_MSG, sizeof(CELL_MQTT_SET_KEEPALIVE_MSG), 500); ++retries);
         for (uint32_t retries = 0; (retries < 3) && !cell_send_command_await_response(CELL_MQTT_SET_SECURE_MSG, sizeof(CELL_MQTT_SET_SECURE_MSG), 500); ++retries);
         for (uint32_t retries = 0; (retries < 3) && !cell_send_command_await_response(CELL_MQTT_SET_CLEAN_SESSION_MSG, sizeof(CELL_MQTT_SET_CLEAN_SESSION_MSG), 500); ++retries);
         for (uint32_t retries = 0; (retries < 3) && !cell_send_command_await_response(CELL_MQTT_SET_RESP_TIMEOUT_MSG, sizeof(CELL_MQTT_SET_RESP_TIMEOUT_MSG), 500); ++retries);
         for (uint32_t retries = 0; (retries < 3) && !cell_send_command_await_response(CELL_SAVE_MQTT_CONFIG_MSG, sizeof(CELL_SAVE_MQTT_CONFIG_MSG), 1000); ++retries);
         for (uint32_t retries = 0; (retries < 3) && !cell_send_command_await_response(CELL_PDP_CONFIG_MSG, sizeof(CELL_PDP_CONFIG_MSG), 500); ++retries);
         for (uint32_t retries = 0; (retries < 3) && !cell_send_command_await_response(CELL_DISABLE_RADIO_MSG, sizeof(CELL_DISABLE_RADIO_MSG), 500); ++retries);
         for (uint32_t retries = 0; (retries < 3) && !cell_send_command_await_response(CELL_INIT_BEARER_CFG_MSG, sizeof(CELL_INIT_BEARER_CFG_MSG), 500); ++retries);
         for (uint32_t retries = 0; (retries < 3) && !cell_send_command_await_response(CELL_ENABLE_RADIO_MSG, sizeof(CELL_ENABLE_RADIO_MSG), 500); ++retries);
         mqtt_configured = 1;
      }

#else  // USE_MQTT_SN

      // Verify that the MQTT-SN configuration is correctly set up
      for (uint32_t retries = 0; (retries < 3) && !cell_send_command_await_response(CELL_LOAD_MQTTSN_CONFIG_MSG, sizeof(CELL_LOAD_MQTTSN_CONFIG_MSG), 1000); ++retries);
      for (uint32_t retries = 0; (retries < 3) && !cell_send_command_await_response(CELL_READ_MQTTSN_CONFIG_MSG, sizeof(CELL_READ_MQTTSN_CONFIG_MSG), 1000); ++retries);
      if (!mqtt_configured)
      {
         char set_client_id_msg[] = CELL_MQTTSN_SET_CLIENT_ID_MSG;
         memcpy((char*)set_client_id_msg + CLIENT_ID_IMEI_OFFSET, (char*)data.packets[0].imei, CELL_IMEI_LENGTH);
         for (uint32_t retries = 0; (retries < 3) && !cell_send_command_await_response(set_client_id_msg, sizeof(set_client_id_msg), 500); ++retries);
         for (uint32_t retries = 0; (retries < 3) && !cell_send_command_await_response(CELL_SET_SERVER_ADDR_MSG, sizeof(CELL_SET_SERVER_ADDR_MSG), 500); ++retries);
         for (uint32_t retries = 0; (retries < 3) && !cell_send_command_await_response(CELL_SET_CONN_TIMEOUT_MSG, sizeof(CELL_SET_CONN_TIMEOUT_MSG), 500); ++retries);
         for (uint32_t retries = 0; (retries < 3) && !cell_send_command_await_response(CELL_SET_CLEAN_SESSION_MSG, sizeof(CELL_SET_CLEAN_SESSION_MSG), 500); ++retries);
         for (uint32_t retries = 0; (retries < 3) && !cell_send_command_await_response(CELL_MQTTSN_SET_RESP_TIMEOUT_MSG, sizeof(CELL_MQTTSN_SET_RESP_TIMEOUT_MSG), 500); ++retries);
         for (uint32_t retries = 0; (retries < 3) && !cell_send_command_await_response(CELL_SAVE_MQTTSN_CONFIG_MSG, sizeof(CELL_SAVE_MQTTSN_CONFIG_MSG), 1000); ++retries);
         for (uint32_t retries = 0; (retries < 3) && !cell_send_command_await_response(CELL_PDP_CONFIG_MSG, sizeof(CELL_PDP_CONFIG_MSG), 500); ++retries);
         for (uint32_t retries = 0; (retries < 3) && !cell_send_command_await_response(CELL_DISABLE_RADIO_MSG, sizeof(CELL_DISABLE_RADIO_MSG), 500); ++retries);
         for (uint32_t retries = 0; (retries < 3) && !cell_send_command_await_response(CELL_INIT_BEARER_CFG_MSG, sizeof(CELL_INIT_BEARER_CFG_MSG), 500); ++retries);
         for (uint32_t retries = 0; (retries < 3) && !cell_send_command_await_response(CELL_ENABLE_RADIO_MSG, sizeof(CELL_ENABLE_RADIO_MSG), 500); ++retries);
         mqtt_configured = 1;
      }

#endif  // #ifndef USE_MQTT_SN

      // Manually poll to ensure that connectivity status flags are properly initialized at boot
      *(uint64_t*)evidence_message.device_id = strtoull((char*)data.packets[0].imei, NULL, 10);
      for (uint32_t retries = 0; (retries < 3) && !cell_send_command_await_response(CELL_POLL_CGREG_MSG, sizeof(CELL_POLL_CGREG_MSG), 500); ++retries);
      for (uint32_t retries = 0; (retries < 3) && !cell_send_command_await_response(CELL_POLL_CEREG_MSG, sizeof(CELL_POLL_CEREG_MSG), 500); ++retries);
      for (uint32_t retries = 0; (retries < 3) && !cell_send_command_await_response(CELL_POLL_PDP_ACTIVE_MSG, sizeof(CELL_POLL_PDP_ACTIVE_MSG), 1000); ++retries);
   }
   else  // No SIM card present
   {
#ifndef SIM_REQUIRED

      // Stop all peripherals related to cellular functionality
      HAL_NVIC_DisableIRQ(LPTIM5_IRQn);
      HAL_NVIC_DisableIRQ(LPTIM4_IRQn);
      HAL_NVIC_DisableIRQ(LPTIM3_IRQn);
      HAL_NVIC_DisableIRQ(CELL_UART_CONCAT(, CELL_UART_TYPE, CELL_UART_NUMBER, _IRQn));
      CLEAR_BIT(LPTIM5->CR, LPTIM_CR_ENABLE);
      CLEAR_BIT(LPTIM4->CR, LPTIM_CR_ENABLE);
      CLEAR_BIT(LPTIM3->CR, LPTIM_CR_ENABLE);
      CLEAR_BIT(CELL_UART->CR1, USART_CR1_UE);
      CLEAR_BIT(DMA2_Stream3->CR, DMA_SxCR_EN);
      CLEAR_BIT(DMA2_Stream1->CR, DMA_SxCR_EN);
      CLEAR_BIT(RCC->APB4ENR, RCC_APB4ENR_LPTIM5EN);
      CLEAR_BIT(RCC->APB4ENR, RCC_APB4ENR_LPTIM4EN);
      CLEAR_BIT(RCC->APB4ENR, RCC_APB4ENR_LPTIM3EN);
      CLEAR_BIT(RCC->APB1LENR, CELL_UART_CONCAT(RCC_APB1LENR_, CELL_UART_TYPE, CELL_UART_NUMBER, EN));

#else
      chip_reset();
#endif
   }

   // Clear the configuration request flag
   configure_modem = 0;
   return sim_present;
}

static void update_device_configuration(volatile char* config_data, uint32_t data_length, uint8_t test_mode_message)
{
   // Parse the updated configuration values from the JSON message
   uint8_t correct_device_intent = 0;
   config_data_t new_config = device_info.device_config;
   while (data_length && (*config_data != '{')) { ++config_data; --data_length; }
   if (data_length) { ++config_data; --data_length; }
   while (data_length)
   {
      // Parse the key
      while (data_length && (*config_data != '"') && (*config_data != '}')) { ++config_data; --data_length; }
      if (!data_length || (*config_data == '}')) break;
      ++config_data; --data_length;
      const char* const key = (const char*)config_data;
      while (data_length && (*config_data != '"')) { ++config_data; --data_length; }
      if (!data_length) break;
      *config_data = '\0';
      ++config_data; --data_length;

      // Parse the value
      uint8_t quoted_val = 0;
      while (data_length && ((*config_data == ':') || (*config_data == ' ') || (*config_data == '\t'))) { ++config_data; --data_length; }
      if (!data_length) break;
      if (*config_data == '"') { quoted_val = 1; ++config_data; if (!--data_length) break; }
      const char* const val = (const char*)config_data;
      if (quoted_val)
         while (data_length && (*config_data != '"')) { ++config_data; --data_length; }
      else
         while (data_length && (*config_data != ',') && (*config_data != '}')) { ++config_data; --data_length; }
      if (!data_length) break;

      // Save the delimiter and null-terminate the value
      const char delim = (char)*config_data;
      *config_data = '\0';
      ++config_data; --data_length;

      // Process the key-value pair
      if ((strcmp(key, "id") == 0) && (((val[0] == '-') && (val[1] == '1')) || (strtoull(val, NULL, 10) == device_info.device_id))) correct_device_intent = 1;
      else if (test_mode_message) { if (strcmp(key, "enabled") == 0) new_config.test_mode_start_time = (strcasecmp(val, "true") == 0) ? (uint32_t)data.packets[0].timestamp : 0; }
      else
      {
         if (strcmp(key, "info_qos") == 0) new_config.mqtt_device_info_qos = (uint8_t)atoi(val);
         else if (strcmp(key, "alert_qos") == 0) new_config.mqtt_alert_qos = (uint8_t)atoi(val);
         else if (strcmp(key, "audio_qos") == 0) new_config.mqtt_audio_qos = (uint8_t)atoi(val);
         else if (strcmp(key, "shot_min") == 0) new_config.shot_detection_min_threshold = (float)atof(val);
         else if (strcmp(key, "shot_good") == 0) new_config.shot_detection_good_threshold = (float)atof(val);
         else if (strcmp(key, "store_thresh") == 0) new_config.storage_classification_threshold = (float)atof(val);
         else if (strcmp(key, "clip_sec") == 0) new_config.audio_clip_length_seconds = (uint8_t)atoi(val);
         else if (strcmp(key, "status_min") == 0) new_config.device_status_transmission_interval_minutes = (uint8_t)atoi(val);
      }

      // Check if parsing is complete
      if (delim == '}') break;
      if (quoted_val)
      {
         while (data_length && (*config_data != ',') && (*config_data != '}')) { ++config_data; --data_length; }
         if (!data_length) break;
         if (*config_data == '}') break;
         ++config_data; --data_length;
      }
   }

   // Do not update if this message was intended for a different device
   if (!correct_device_intent)
      return;

   // Update active runtime parameters if this was not a test mode message
   if (!test_mode_message)
   {
      // Update the device reporting timer if status reporting has changed
      if (new_config.device_status_transmission_interval_minutes && !device_info.device_config.device_status_transmission_interval_minutes)
         SET_BIT(LPTIM3->CR, LPTIM_CR_CNTSTRT);
      else if (!new_config.device_status_transmission_interval_minutes && device_info.device_config.device_status_transmission_interval_minutes)
      {
         CLEAR_BIT(LPTIM3->CR, LPTIM_CR_ENABLE);
         while (READ_BIT(LPTIM3->CR, LPTIM_CR_ENABLE));
         SET_BIT(LPTIM3->CR, LPTIM_CR_ENABLE);
         WRITE_REG(LPTIM3->ARR, CELL_TIMER_1MIN_COUNT);
         while (READ_REG(LPTIM3->ARR) != CELL_TIMER_1MIN_COUNT);
      }

      // Update the AI configuration parameters
      data.packets[0].ai_config.audio_clip_length_seconds = data.packets[1].ai_config.audio_clip_length_seconds = new_config.audio_clip_length_seconds;
      data.packets[0].ai_config.storage_classification_threshold = data.packets[1].ai_config.storage_classification_threshold = new_config.storage_classification_threshold;
   }

   // Copy the new configuration to non-volatile storage
   device_info.device_config = new_config;
   chip_save_config(0);

   // Signal that a device status message should be sent to acknowledge receipt of the configuration change
   device_update_timer_count = 0;
   device_info_update = 1;
}

static char* handle_mqtt_message(char* msg, uint16_t max_msg_len)
{
   // Parse the broker operation code
#ifdef USE_MQTT_SN
   char *msg_start = find_start_of_message(msg, sizeof(CELL_MQTT_SN_MSG) - 1, &max_msg_len);
#else
   char *msg_start = find_start_of_message(msg, sizeof(CELL_MQTT_MSG) - 1, &max_msg_len);
#endif
   const uint32_t opcode_length = ((msg_start[1] == ',') ? 1 : 2);
   msg_start[opcode_length] = '\0';
   const mqtt_operation_t mqtt_operation = (mqtt_operation_t)atoi(msg_start);

   // Handle the operation
   if ((mqtt_operation == MQTT_DISCONNECT) || (mqtt_operation == MQTT_CONNECT))
   {
      connectivity_changed = 1;
      mqtt_result = (msg_start[opcode_length + 1] == '1');
      mqtt_connected = (mqtt_operation == MQTT_CONNECT) && mqtt_result;
      msg = find_end_of_message(msg_start, &max_msg_len) + 1;
      mqtt_subscribed = !mqtt_connected;
      if (mqtt_operation == MQTT_CONNECT)
         mqtt_connect_pending = 0;
   }
   else if (mqtt_operation == MQTT_READ)
   {
      msg = find_end_of_message(msg_start, &max_msg_len) + 1;
      if (mqtt_operation_awaiting_ack == MQTT_READ)
      {
         char *message_end = msg - 3, *search = message_end;
         while ((search > msg_start) && (*search != '{'))
            --search;
         if (*search == '{')
         {
            incoming_message = search;
            incoming_message_length = (uint32_t)(message_end - search);
         }
         if (pending_messages)
            --pending_messages;
      }
      else
         pending_messages = (uint8_t)atoi(msg_start + opcode_length + 1);
   }
   else
   {
      mqtt_result = (msg_start[opcode_length + 1] == '1');
      msg = find_end_of_message(msg_start, &max_msg_len) + 1;
   }
   if (mqtt_operation == mqtt_operation_awaiting_ack)
      mqtt_operation_awaiting_ack = MQTT_DONE;
   return msg;
}

static void cell_enqueue_audio(uint16_t evidence_message_length)
{
   // Enqueue the current evidence message into the audio ring buffer
   const uint32_t next_slot = (cell_audio_tx_head + cell_audio_tx_count) % CELL_AUDIO_TX_RING_SIZE;
   if (cell_audio_tx_count < CELL_AUDIO_TX_RING_SIZE)
   {
      cell_audio_tx_ring[next_slot].message = evidence_message;
      cell_audio_tx_ring[next_slot].length = evidence_message_length;
      ++cell_audio_tx_count;
   }
   else
   {
      // Overwrite the oldest evidence message if full
      cell_audio_tx_ring[cell_audio_tx_head].message = evidence_message;
      cell_audio_tx_ring[cell_audio_tx_head].length = evidence_message_length;
      cell_audio_tx_head = (cell_audio_tx_head + 1) % CELL_AUDIO_TX_RING_SIZE;
   }
}

static uint16_t cell_process_message(char* msg, uint16_t max_msg_len)
{
   // Search for the proper start of the message
   const char* const orig_msg = msg;
   while (max_msg_len && ((*msg == '\r') || (*msg == '\n')))
   {
      --max_msg_len;
      ++msg;
   }

   // Determine what type of message was received
   if (max_msg_len && (mqtt_operation_awaiting_ack == MQTT_PUBLISH_BINARY) && (*msg == '>'))
   {
      prompt_received = 1;
      msg += 1;
   }
   else if ((max_msg_len >= sizeof(CELL_CME_ERROR_MSG)) && (memcmp(msg, CELL_CME_ERROR_MSG, sizeof(CELL_CME_ERROR_MSG) - 1) == 0))
   {
      /*in_holdoff_period = 0;
      cell_send_command(CELL_GET_MQTTSN_ERROR_MSG, sizeof(CELL_GET_MQTTSN_ERROR_MSG));*/
      char *msg_start = find_start_of_message(msg, sizeof(CELL_CME_ERROR_MSG) - 1, &max_msg_len);
      msg = find_end_of_message(msg_start, &max_msg_len) + 1;
      cme_error = (uint16_t)atoi(msg_start);
      if (cme_error == 53)
      {
         if (mqtt_operation_awaiting_ack == MQTT_CONNECT)
         {
            mqtt_connected = 1;
            connectivity_changed = 1;
         }
         else if (mqtt_operation_awaiting_ack == MQTT_READ)
            pending_messages = 0;
      }
      cell_modem_available = 1;
      command_nacked = 1;
   }
   else if (((max_msg_len >= (sizeof(AT_RESPONSE_OK) - 1)) && (memcmp(msg, AT_RESPONSE_OK, sizeof(AT_RESPONSE_OK) - 1) == 0)) ||
            ((max_msg_len >= (sizeof(CELL_ACK_MSG) - 1)) && (memcmp(msg, CELL_ACK_MSG, sizeof(CELL_ACK_MSG) - 1) == 0)))
   {
      command_acked = 1;
      cell_modem_available = 1;
      msg += ((msg[0] == AT_RESPONSE_OK[0]) ? sizeof(AT_RESPONSE_OK) : sizeof(CELL_ACK_MSG));
   }
   else if (((max_msg_len >= (sizeof(AT_RESPONSE_ERROR) - 1)) && (memcmp(msg, AT_RESPONSE_ERROR, sizeof(AT_RESPONSE_ERROR) - 1) == 0)) ||
            ((max_msg_len >= (sizeof(CELL_ERROR_MSG) - 1)) && (memcmp(msg, CELL_ERROR_MSG, sizeof(CELL_ERROR_MSG) - 1) == 0)))
   {
      command_nacked = 1;
      cell_modem_available = 1;
      msg += ((msg[0] == AT_RESPONSE_ERROR[0]) ? sizeof(AT_RESPONSE_ERROR) : sizeof(CELL_ERROR_MSG));
   }
   else if ((max_msg_len >= (6 + sizeof(CELL_NETWORK_EVENT_MSG))) && (memcmp(msg, CELL_NETWORK_EVENT_MSG, sizeof(CELL_NETWORK_EVENT_MSG) - 1) == 0))
   {
      char *msg_start = find_start_of_message(msg, sizeof(CELL_NETWORK_EVENT_MSG) - 1, &max_msg_len);
      msg = find_end_of_message(msg_start, &max_msg_len) + 1;
      if (memcmp(msg_start, CGEV_PDN_ACT_MSG, sizeof(CGEV_PDN_ACT_MSG)-1) == 0)
         valid_pdp = 1;
      else if (memcmp(msg_start, CGEV_PDN_DEACT_MSG, sizeof(CGEV_PDN_DEACT_MSG)-1) == 0)
         valid_pdp = 0;
      connectivity_changed = 1;
   }
#ifdef USE_MQTT_SN
   else if ((max_msg_len >= (1 + sizeof(CELL_MQTT_SN_MSG))) && (memcmp(msg, CELL_MQTT_SN_MSG, sizeof(CELL_MQTT_SN_MSG) - 1) == 0))
#else
   else if ((max_msg_len >= (1 + sizeof(CELL_MQTT_MSG))) && (memcmp(msg, CELL_MQTT_MSG, sizeof(CELL_MQTT_MSG) - 1) == 0))
#endif
      msg = handle_mqtt_message(msg, max_msg_len);
   else if ((max_msg_len >= (1 + sizeof(CELL_MQTT_ERROR_MSG))) && (memcmp(msg, CELL_MQTT_ERROR_MSG, sizeof(CELL_MQTT_ERROR_MSG) - 1) == 0))
   {
      char *msg_start = find_start_of_message(msg, sizeof(CELL_MQTT_ERROR_MSG) - 1, &max_msg_len);
      msg = msg_start;
      while (--max_msg_len && (*(++msg_start) != ','));
      *msg_start = '\0';
      const uint32_t error_class = (uint32_t)atoi(msg);
      msg = find_end_of_message(msg_start, &max_msg_len) + 1;
      const uint32_t error_code = (uint32_t)atoi(msg_start + 1);
      if (error_class == UMQTTSN_ERROR_CLASS_CODE)
         cme_error = error_code;
   }
   else if ((max_msg_len >= (3 + sizeof(CELL_SIGNAL_QUALITY_MSG))) && (memcmp(msg, CELL_SIGNAL_QUALITY_MSG, sizeof(CELL_SIGNAL_QUALITY_MSG) - 1) == 0))
   {
      msg = find_start_of_message(msg, sizeof(CELL_SIGNAL_QUALITY_MSG) - 1, &max_msg_len);
      char *msg_ptr = msg; while (--max_msg_len && (*(++msg_ptr) != ',')); *msg_ptr = '\0';
      const uint8_t sig_power_ind = (uint8_t)atoi(msg);
      signal_power = (sig_power_ind == 99) ? 255 : ((3 * sig_power_ind) + 7);
      msg = find_end_of_message(msg_ptr, &max_msg_len) + 1;
      const uint8_t sig_qual_ind = (uint8_t)atoi(msg_ptr+1);
      signal_quality = (sig_qual_ind == 99) ? 255 : sig_qual_ind;
   }
   else if ((max_msg_len >= (4 + sizeof(CELL_PDP_ACTIVE_MSG))) && (memcmp(msg, CELL_PDP_ACTIVE_MSG, sizeof(CELL_PDP_ACTIVE_MSG) - 1) == 0))
   {
      msg = find_start_of_message(msg, sizeof(CELL_PDP_ACTIVE_MSG) - 1, &max_msg_len) + 5;
      if (msg[-5] == '1')
      {
         valid_pdp = (msg[-3] == '1');
         connectivity_changed = 1;
      }
   }
   else if ((max_msg_len >= (1 + sizeof(CELL_CEREG_RESPONSE_MSG))) && (memcmp(msg, CELL_CEREG_RESPONSE_MSG, sizeof(CELL_CEREG_RESPONSE_MSG) - 1) == 0))
   {
      msg = find_start_of_message(msg, sizeof(CELL_CEREG_RESPONSE_MSG) - 1, &max_msg_len);
      const uint8_t is_urc = msg[1] != ',';
      const char response = is_urc ? msg[0] : msg[2];
      valid_cereg = (response == '1') || (response == '5');
      msg += (is_urc ? 3 : 5);
      connectivity_changed = 1;
   }
   else if ((max_msg_len >= (1 + sizeof(CELL_CGREG_RESPONSE_MSG))) && (memcmp(msg, CELL_CGREG_RESPONSE_MSG, sizeof(CELL_CGREG_RESPONSE_MSG) - 1) == 0))
   {
      msg = find_start_of_message(msg, sizeof(CELL_CGREG_RESPONSE_MSG) - 1, &max_msg_len);
      const uint8_t is_urc = msg[1] != ',';
      const char response = is_urc ? msg[0] : msg[2];
      valid_cgreg = (response == '1') || (response == '5');
      msg += (is_urc ? 3 : 5);
      connectivity_changed = 1;
   }
   else if ((max_msg_len >= (3 + sizeof(CELL_TEMP_ALERT_MSG))) && (memcmp(msg, CELL_TEMP_ALERT_MSG, sizeof(CELL_TEMP_ALERT_MSG) - 1) == 0))
   {
      msg = find_start_of_message(msg, sizeof(CELL_TEMP_ALERT_MSG) - 1, &max_msg_len) + 5;
      temperature_alert = (msg[-3] == '1') ? 1 : ((msg[-3] == '2') ? 2 : 0);
   }
   else if ((max_msg_len >= (sizeof(CELL_GREETING_MSG) - 1)) && (memcmp(msg, CELL_GREETING_MSG, sizeof(CELL_GREETING_MSG) - 1) == 0))
   {
      // Always reconfigure modem upon receiving a greeting message since this means a modem reboot must have occurred
      configure_modem = 1;
      cell_modem_available = 1;
#ifdef FORCE_FACTORY_RESET
      in_holdoff_period = 0;
      if (temperature_alert)
         cell_send_command(CELL_FACTORY_RESET2_MSG, sizeof(CELL_FACTORY_RESET2_MSG));
      else
         cell_send_command(CELL_FACTORY_RESET1_MSG, sizeof(CELL_FACTORY_RESET1_MSG));
      temperature_alert = !temperature_alert;
#endif
      msg += sizeof(CELL_GREETING_MSG);
   }
   else if ((max_msg_len >= (1 + sizeof(CELL_BAUD_RATE_MSG))) && (memcmp(msg, CELL_BAUD_RATE_MSG, sizeof(CELL_BAUD_RATE_MSG) - 1) == 0))
   {
      char *msg_start = find_start_of_message(msg, sizeof(CELL_BAUD_RATE_MSG) - 1, &max_msg_len);
      msg = find_end_of_message(msg_start, &max_msg_len) + 1;
      baud_rate = (uint32_t)atoi(msg_start);
   }
   else if ((max_msg_len >= (1 + SIM_CARD_ID_MIN_LENGTH + sizeof(CELL_SIM_CARD_ID_MSG))) && (memcmp(msg, CELL_SIM_CARD_ID_MSG, sizeof(CELL_SIM_CARD_ID_MSG) - 1) == 0))
   {
      msg = find_start_of_message(msg, sizeof(CELL_SIM_CARD_ID_MSG) - 1, &max_msg_len);
      for (int i = 0; (i < sizeof(sim_id)) && max_msg_len && (*msg != '\r'); ++i)
         sim_id[i] = *(msg++);
      msg += 2;
   }
   else if ((max_msg_len >= (1 + CELL_IMEI_LENGTH + sizeof(CELL_IMEI_MSG))) && (memcmp(msg, CELL_IMEI_MSG, sizeof(CELL_IMEI_MSG) - 1) == 0))
   {
      msg = find_start_of_message(msg, sizeof(CELL_IMEI_MSG) - 1, &max_msg_len) + 1;
      memcpy((char*)data.packets[0].imei, msg, CELL_IMEI_LENGTH);
      memcpy((char*)data.packets[1].imei, msg, CELL_IMEI_LENGTH);
      msg += CELL_IMEI_LENGTH + 3;
   }
   else if ((max_msg_len >= (12 + sizeof(CELL_SIGNAL_QUALITY_EXT_MSG))) && (memcmp(msg, CELL_SIGNAL_QUALITY_EXT_MSG, sizeof(CELL_SIGNAL_QUALITY_EXT_MSG) - 1) == 0))
   {
      msg = find_start_of_message(msg, sizeof(CELL_SIGNAL_QUALITY_EXT_MSG) - 1, &max_msg_len);
      char *msg_end = msg; while (--max_msg_len && (*(++msg_end) != ',')); *(msg_end++) = '\0';
      const uint8_t rxlev = (uint8_t)atoi(msg);
      msg = msg_end; while (--max_msg_len && (*(++msg_end) != ',')); *(msg_end++) = '\0';
      const uint8_t ber = (uint8_t)atoi(msg);
      msg = msg_end; while (--max_msg_len && (*(++msg_end) != ',')); *(msg_end++) = '\0';
      const uint8_t rscp = (uint8_t)atoi(msg);
      msg = msg_end; while (--max_msg_len && (*(++msg_end) != ',')); *(msg_end++) = '\0';
      const uint8_t ecn0 = (uint8_t)atoi(msg);
      msg = msg_end; while (--max_msg_len && (*(++msg_end) != ',')); *(msg_end++) = '\0';
      const uint8_t rsrq = (uint8_t)atoi(msg);
      msg = msg_end; while (--max_msg_len && (*(++msg_end) != '\r')); *(msg_end++) = '\0';
      const uint8_t rsrp = (uint8_t)atoi(msg);
      signal_power = (rsrp != 255) ? rsrp : ((rscp != 255) ? rscp : rxlev);
      signal_quality = (rsrq != 255) ? rsrq : ((ecn0 != 255) ? ecn0 : ber);
      msg += 1;
   }
   else if ((max_msg_len > CELL_IMSI_LENGTH) && reading_imsi)
   {
      memcpy((char*)data.packets[0].imsi, msg, CELL_IMSI_LENGTH);
      memcpy((char*)data.packets[1].imsi, msg, CELL_IMSI_LENGTH);
      memcpy((char*)device_info.imsi, msg, CELL_IMSI_LENGTH);
      msg += CELL_IMSI_LENGTH + 1;
      reading_imsi = 0;
   }
   else
      msg = find_end_of_message(msg, &max_msg_len);

   // Return the length of the received message
   return (uint16_t)(msg - orig_msg);
}

static void cell_process_network_message(volatile char* message, uint32_t message_length)
{
   // Process the message depending on its type
   if ((message_length >= 16) && (memcmp((char*)message, "{\"type\"", 7) == 0))
   {
      volatile char *type_start = message + 7;
      while ((*(++type_start) == ':') || (*type_start == ' ') || (*type_start == '\t') || (*type_start == '"'));
      switch ((mqtt_device_message_t)(*type_start))
      {
         case MQTT_DEVICE_CONFIG_UPDATE:
            update_device_configuration(message, message_length, 0);
            break;
         case MQTT_DEVICE_TEST_MODE:
            update_device_configuration(message, message_length, 1);
            break;
         default:
            break;
      }
   }
}


// Interrupt Service Routines ------------------------------------------------------------------------------------------

void LPTIM3_IRQHandler(void)
{
   // Clear the interrupt and set the device update flag
   WRITE_REG(LPTIM3->ICR, LPTIM_FLAG_ARRM);
   if (++device_update_timer_count >= device_info.device_config.device_status_transmission_interval_minutes)
   {
      device_update_timer_count = 0;
      device_info_update = 1;
   }

   // Check whether there has been a network connectivity issue for too long
   if (valid_cgreg || valid_cereg)
   {
      bad_network_conn_timer_count = 0;
      if (valid_pdp)
      {
         bad_pdp_timer_count = 0;
         if (!mqtt_connected)
         {
            // Retry MQTT connect if the broker CONNACK was never received or was silently dropped
            if (++bad_mqtt_conn_timer_count >= CELL_BAD_MQTT_TIMEOUT_MINUTES)
            {
               bad_mqtt_conn_timer_count = 0;
               mqtt_connect_pending = 0;
               connectivity_changed = 1;
            }
         }
         else
            bad_mqtt_conn_timer_count = 0;
      }
      else
      {
         bad_mqtt_conn_timer_count = 0;
         if (++bad_pdp_timer_count >= CELL_BAD_PDP_TIMEOUT_MINUTES)
            connectivity_changed = 1;
      }
   }
   else if (++bad_network_conn_timer_count >= CELL_BAD_CONN_TIMEOUT_MINUTES)
      connectivity_changed = 1;
}

void LPTIM4_IRQHandler(void)
{
   // Clear the interrupt and set the timeout flag
   WRITE_REG(LPTIM4->ICR, LPTIM_FLAG_ARRM);
   timed_out = 1;
}

void LPTIM5_IRQHandler(void)
{
   // Clear the interrupt and the hold-off period flag
   WRITE_REG(LPTIM5->ICR, LPTIM_FLAG_ARRM);
   in_holdoff_period = 0;
}

void CELL_UART_IRQHandler(void)
{
   // Create static indices to keep track of the location of unprocessed data
   static uint32_t previous_data_index = 0;
   static int32_t bytes_received;
   static char *msg_begin;

   // Freeze the contents of the DMA byte counter to ensure packet boundaries are maintained
   const uint32_t data_end_index = sizeof(cell_rx_buffer) - DMA2_Stream1->NDTR;
   __DMB();

   // Ensure that this interrupt was due to an idle line
   if (READ_BIT(CELL_UART->ISR, USART_ISR_IDLE))
   {
      // Clear the interrupt and set the transmission hold-off period flag
      WRITE_REG(CELL_UART->ICR, UART_CLEAR_IDLEF);
      in_holdoff_period = 1;

      // Determine whether data wrapping is necessary due to the circular buffer
      if (data_end_index >= previous_data_index)
      {
         bytes_received = (int32_t)(data_end_index - previous_data_index);
         msg_begin = cell_rx_buffer + previous_data_index;
      }
      else
      {
         bytes_received = (int32_t)(sizeof(cell_rx_buffer) - previous_data_index + data_end_index);
         memmove(cell_rx_buffer + previous_data_index - data_end_index, cell_rx_buffer + previous_data_index, sizeof(cell_rx_buffer) - previous_data_index);
         memmove(cell_rx_buffer + sizeof(cell_rx_buffer) - data_end_index, cell_rx_buffer, data_end_index);
         msg_begin = cell_rx_buffer + previous_data_index - data_end_index;
      }

      // Process newly received data until no bytes remain
      while (bytes_received > 0)
      {
         const uint16_t msg_length = cell_process_message(msg_begin, (uint16_t)bytes_received);
         bytes_received -= msg_length;
         msg_begin += msg_length;
      }
      previous_data_index = data_end_index;

      // Start the ~20ms hold-off timer to allow modem to send unsolicited messages
      SET_BIT(LPTIM5->CR, LPTIM_CR_SNGSTRT);
   }
}


// Public API Functions ------------------------------------------------------------------------------------------------

void cell_power_on(void)
{
   // Initialize the various GPIO clocks
   SET_BIT(RCC->AHB4ENR, RCC_AHB4ENR_GPIOAEN);
   (void)READ_BIT(RCC->AHB4ENR, RCC_AHB4ENR_GPIOAEN);
#if REV_ID == REV_A
   SET_BIT(RCC->AHB4ENR, RCC_AHB4ENR_GPIOEEN);
   (void)READ_BIT(RCC->AHB4ENR, RCC_AHB4ENR_GPIOEEN);
#endif

   // Initialize the GPIO power pin
   WRITE_REG(CELL_NPWR_ON_GPIO_Port->BSRR, CELL_NPWR_ON_Pin);
   uint32_t position = 32 - __builtin_clz(CELL_NPWR_ON_Pin) - 1;
   MODIFY_REG(CELL_NPWR_ON_GPIO_Port->OSPEEDR, (GPIO_OSPEEDR_OSPEED0 << (position * 2U)), (GPIO_SPEED_FREQ_LOW << (position * 2U)));
   MODIFY_REG(CELL_NPWR_ON_GPIO_Port->OTYPER, (GPIO_OTYPER_OT0 << position), (((GPIO_MODE_OUTPUT_OD & OUTPUT_TYPE) >> OUTPUT_TYPE_Pos) << position));
   CLEAR_BIT(CELL_NPWR_ON_GPIO_Port->PUPDR, (GPIO_PUPDR_PUPD0 << (position * 2U)));
   MODIFY_REG(CELL_NPWR_ON_GPIO_Port->MODER, (GPIO_MODER_MODE0 << (position * 2U)), ((GPIO_MODE_OUTPUT_OD & GPIO_MODE) << (position * 2U)));

   // Power on the cellular modem
   cell_toggle_power(1);
}

void cell_init(void)
{
   // Initialize the various MQTT publish messages
   memset(&evidence_message, 0, sizeof(evidence_message));
#if defined(CELL_MQTT_USE_BINARY_PUBLISH) && defined(USE_MQTT_SN)
   info_message_length = sizeof(CELL_MQTTSN_PUB_BINARY_INFO_MSG);
   memcpy(publish_info_message, CELL_MQTTSN_PUB_BINARY_INFO_MSG, sizeof(CELL_MQTTSN_PUB_BINARY_INFO_MSG));
   info_message_length += sprintf(publish_info_message + sizeof(CELL_MQTTSN_PUB_BINARY_INFO_MSG) - 1, "%u\r", sizeof(device_info_t));
   alert_prefix_length = sizeof(CELL_MQTTSN_PUB_BINARY_ALERT_MSG) - 1;
   memcpy(publish_alert_message, CELL_MQTTSN_PUB_BINARY_ALERT_MSG, alert_prefix_length + 1);
   audio_prefix_length = sizeof(CELL_MQTTSN_PUB_BINARY_AUDIO_MSG) - 1;
   memcpy(publish_audio_message, CELL_MQTTSN_PUB_BINARY_AUDIO_MSG, audio_prefix_length + 1);
#endif

   // Initialize the DMA2 clock
   cell_modem_available = 0;
   SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_DMA2EN);
   (void)READ_BIT(RCC->AHB1ENR, RCC_AHB1ENR_DMA2EN);

   // Initialize the CELL UART peripheral clock
   SET_BIT(RCC->APB1LENR, CELL_UART_CONCAT(RCC_APB1LENR_, CELL_UART_TYPE, CELL_UART_NUMBER, EN));
   (void)READ_BIT(RCC->APB1LENR, CELL_UART_CONCAT(RCC_APB1LENR_, CELL_UART_TYPE, CELL_UART_NUMBER, EN));
   MODIFY_REG(RCC->D2CCIP2R, RCC_D2CCIP2R_USART28SEL, (uint32_t)RCC_USART234578CLKSOURCE_D2PCLK1);

   // Initialize the SYSCFG clock
   SET_BIT(RCC->APB4ENR, RCC_APB4ENR_SYSCFGEN);
   (void)READ_BIT(RCC->APB4ENR, RCC_APB4ENR_SYSCFGEN);

   // Initialize the LPTIM peripheral clocks
   MODIFY_REG(RCC->D3CCIPR, RCC_D3CCIPR_LPTIM345SEL, RCC_LPTIM345CLKSOURCE_LSE);
   SET_BIT(RCC->APB4ENR, RCC_APB4ENR_LPTIM3EN);
   (void)READ_BIT(RCC->APB4ENR, RCC_APB4ENR_LPTIM3EN);
   SET_BIT(RCC->APB4ENR, RCC_APB4ENR_LPTIM4EN);
   (void)READ_BIT(RCC->APB4ENR, RCC_APB4ENR_LPTIM4EN);
   SET_BIT(RCC->APB4ENR, RCC_APB4ENR_LPTIM5EN);
   (void)READ_BIT(RCC->APB4ENR, RCC_APB4ENR_LPTIM5EN);

   // Initialize the non-peripheral GPIO pins
   WRITE_REG(CELL_NRESET_GPIO_Port->BSRR, CELL_NRESET_Pin);
   uint32_t position = 32 - __builtin_clz(CELL_NRESET_Pin) - 1;
   MODIFY_REG(CELL_NRESET_GPIO_Port->OSPEEDR, (GPIO_OSPEEDR_OSPEED0 << (position * 2U)), (GPIO_SPEED_FREQ_LOW << (position * 2U)));
   MODIFY_REG(CELL_NRESET_GPIO_Port->OTYPER, (GPIO_OTYPER_OT0 << position), (((GPIO_MODE_OUTPUT_OD & OUTPUT_TYPE) >> OUTPUT_TYPE_Pos) << position));
   CLEAR_BIT(CELL_NRESET_GPIO_Port->PUPDR, (GPIO_PUPDR_PUPD0 << (position * 2U)));
   MODIFY_REG(CELL_NRESET_GPIO_Port->MODER, (GPIO_MODER_MODE0 << (position * 2U)), ((GPIO_MODE_OUTPUT_OD & GPIO_MODE) << (position * 2U)));
#if REV_ID == REV_A
   WRITE_REG(RPI_PWR_SWITCH_GPIO_Port->BSRR, RPI_PWR_SWITCH_Pin);
   position = 32 - __builtin_clz(RPI_PWR_SWITCH_Pin) - 1;
   MODIFY_REG(RPI_PWR_SWITCH_GPIO_Port->OSPEEDR, (GPIO_OSPEEDR_OSPEED0 << (position * 2U)), (GPIO_SPEED_FREQ_LOW << (position * 2U)));
   MODIFY_REG(RPI_PWR_SWITCH_GPIO_Port->OTYPER, (GPIO_OTYPER_OT0 << position), (((GPIO_MODE_OUTPUT_OD & OUTPUT_TYPE) >> OUTPUT_TYPE_Pos) << position));
   CLEAR_BIT(RPI_PWR_SWITCH_GPIO_Port->PUPDR, (GPIO_PUPDR_PUPD0 << (position * 2U)));
   MODIFY_REG(RPI_PWR_SWITCH_GPIO_Port->MODER, (GPIO_MODER_MODE0 << (position * 2U)), ((GPIO_MODE_OUTPUT_OD & GPIO_MODE) << (position * 2U)));
   position = 32 - __builtin_clz(RPI_NPWR_OFF_Pin) - 1;
   MODIFY_REG(RPI_NPWR_OFF_GPIO_Port->PUPDR, (GPIO_PUPDR_PUPD0 << (position * 2U)), (GPIO_PULLUP << (position * 2U)));
   MODIFY_REG(RPI_NPWR_OFF_GPIO_Port->MODER, (GPIO_MODER_MODE0 << (position * 2U)), ((GPIO_MODE_INPUT & GPIO_MODE) << (position * 2U)));
#endif

   // Initialize the CELL UART GPIO pins
   position = 32 - __builtin_clz(CELL_RTS_Pin) - 1;
   MODIFY_REG(CELL_RTS_GPIO_Port->OSPEEDR, (GPIO_OSPEEDR_OSPEED0 << (position * 2U)), (GPIO_SPEED_FREQ_MEDIUM << (position * 2U)));
   MODIFY_REG(CELL_RTS_GPIO_Port->OTYPER, (GPIO_OTYPER_OT0 << position), (((GPIO_MODE_AF_PP & OUTPUT_TYPE) >> OUTPUT_TYPE_Pos) << position));
   CLEAR_BIT(CELL_RTS_GPIO_Port->PUPDR, (GPIO_PUPDR_PUPD0 << (position * 2U)));
   MODIFY_REG(CELL_RTS_GPIO_Port->AFR[position >> 3U], (0xFU << ((position & 0x07U) * 4U)), (CELL_UART_AF << ((position & 0x07U) * 4U)));
   MODIFY_REG(CELL_RTS_GPIO_Port->MODER, (GPIO_MODER_MODE0 << (position * 2U)), ((GPIO_MODE_AF_PP & GPIO_MODE) << (position * 2U)));
   position = 32 - __builtin_clz(CELL_CTS_Pin) - 1;
   MODIFY_REG(CELL_CTS_GPIO_Port->OSPEEDR, (GPIO_OSPEEDR_OSPEED0 << (position * 2U)), (GPIO_SPEED_FREQ_MEDIUM << (position * 2U)));
   MODIFY_REG(CELL_CTS_GPIO_Port->OTYPER, (GPIO_OTYPER_OT0 << position), (((GPIO_MODE_AF_PP & OUTPUT_TYPE) >> OUTPUT_TYPE_Pos) << position));
   CLEAR_BIT(CELL_CTS_GPIO_Port->PUPDR, (GPIO_PUPDR_PUPD0 << (position * 2U)));
   MODIFY_REG(CELL_CTS_GPIO_Port->AFR[position >> 3U], (0xFU << ((position & 0x07U) * 4U)), (CELL_UART_AF << ((position & 0x07U) * 4U)));
   MODIFY_REG(CELL_CTS_GPIO_Port->MODER, (GPIO_MODER_MODE0 << (position * 2U)), ((GPIO_MODE_AF_PP & GPIO_MODE) << (position * 2U)));
   position = 32 - __builtin_clz(CELL_TX_Pin) - 1;
   MODIFY_REG(CELL_TX_GPIO_Port->OSPEEDR, (GPIO_OSPEEDR_OSPEED0 << (position * 2U)), (GPIO_SPEED_FREQ_MEDIUM << (position * 2U)));
   MODIFY_REG(CELL_TX_GPIO_Port->OTYPER, (GPIO_OTYPER_OT0 << position), (((GPIO_MODE_AF_PP & OUTPUT_TYPE) >> OUTPUT_TYPE_Pos) << position));
   CLEAR_BIT(CELL_TX_GPIO_Port->PUPDR, (GPIO_PUPDR_PUPD0 << (position * 2U)));
   MODIFY_REG(CELL_TX_GPIO_Port->AFR[position >> 3U], (0xFU << ((position & 0x07U) * 4U)), (CELL_UART_AF << ((position & 0x07U) * 4U)));
   MODIFY_REG(CELL_TX_GPIO_Port->MODER, (GPIO_MODER_MODE0 << (position * 2U)), ((GPIO_MODE_AF_PP & GPIO_MODE) << (position * 2U)));
   position = 32 - __builtin_clz(CELL_RX_Pin) - 1;
   MODIFY_REG(CELL_RX_GPIO_Port->OSPEEDR, (GPIO_OSPEEDR_OSPEED0 << (position * 2U)), (GPIO_SPEED_FREQ_MEDIUM << (position * 2U)));
   MODIFY_REG(CELL_RX_GPIO_Port->OTYPER, (GPIO_OTYPER_OT0 << position), (((GPIO_MODE_AF_PP & OUTPUT_TYPE) >> OUTPUT_TYPE_Pos) << position));
   CLEAR_BIT(CELL_RX_GPIO_Port->PUPDR, (GPIO_PUPDR_PUPD0 << (position * 2U)));
   MODIFY_REG(CELL_RX_GPIO_Port->AFR[position >> 3U], (0xFU << ((position & 0x07U) * 4U)), (CELL_UART_AF << ((position & 0x07U) * 4U)));
   MODIFY_REG(CELL_RX_GPIO_Port->MODER, (GPIO_MODER_MODE0 << (position * 2U)), ((GPIO_MODE_AF_PP & GPIO_MODE) << (position * 2U)));

   // Initialize DMA2 Stream1 for CELL UART RX
   CLEAR_BIT(DMA2_Stream1->CR, DMA_SxCR_EN);
   while (READ_BIT(DMA2_Stream1->CR, DMA_SxCR_EN));
   MODIFY_REG(DMA2_Stream1->CR,
              (DMA_SxCR_MBURST | DMA_SxCR_PBURST | DMA_SxCR_PL | DMA_SxCR_MSIZE | DMA_SxCR_PSIZE | DMA_SxCR_MINC | DMA_SxCR_PINC | DMA_SxCR_CIRC | DMA_SxCR_DIR | DMA_SxCR_CT | DMA_SxCR_DBM),
              (DMA_PERIPH_TO_MEMORY | DMA_MINC_ENABLE | DMA_PDATAALIGN_BYTE | DMA_MDATAALIGN_BYTE | DMA_CIRCULAR | DMA_PRIORITY_MEDIUM | DMA_SxCR_TRBUFF));
   CLEAR_BIT(DMA2_Stream1->FCR, (DMA_SxFCR_DMDIS | DMA_SxFCR_FTH));
   WRITE_REG(DMAMUX1_Channel9->CCR, CELL_UART_CONCAT(DMA_REQUEST_, CELL_UART_TYPE, CELL_UART_NUMBER, _RX));
   WRITE_REG(DMAMUX1_ChannelStatus->CFR, (1UL << (9 & 0x1FU)));

   // Initialize DMA2 Stream3 for CELL UART TX
   CLEAR_BIT(DMA2_Stream3->CR, DMA_SxCR_EN);
   while (READ_BIT(DMA2_Stream3->CR, DMA_SxCR_EN));
   MODIFY_REG(DMA2_Stream3->CR,
              (DMA_SxCR_MBURST | DMA_SxCR_PBURST | DMA_SxCR_PL | DMA_SxCR_MSIZE | DMA_SxCR_PSIZE | DMA_SxCR_MINC | DMA_SxCR_PINC | DMA_SxCR_CIRC | DMA_SxCR_DIR | DMA_SxCR_CT | DMA_SxCR_DBM),
              (DMA_MEMORY_TO_PERIPH | DMA_MINC_ENABLE | DMA_PDATAALIGN_BYTE | DMA_MDATAALIGN_BYTE | DMA_PRIORITY_MEDIUM | DMA_SxCR_TRBUFF));
   CLEAR_BIT(DMA2_Stream3->FCR, (DMA_SxFCR_DMDIS | DMA_SxFCR_FTH));
   dma_int_registers = (dma_int_registers_t*)((uint32_t)DMA2_Stream3 & (uint32_t)(~0x3FFU));
   WRITE_REG(DMAMUX1_Channel11->CCR, CELL_UART_CONCAT(DMA_REQUEST_, CELL_UART_TYPE, CELL_UART_NUMBER, _TX));
   WRITE_REG(DMAMUX1_ChannelStatus->CFR, (1UL << (11 & 0x1FU)));

   // Initialize the CELL UART peripheral
   CLEAR_BIT(CELL_UART->CR1, USART_CR1_UE);
   while (READ_BIT(CELL_UART->CR1, USART_CR1_UE));
   MODIFY_REG(CELL_UART->CR1, (USART_CR1_M | USART_CR1_PCE | USART_CR1_PS | USART_CR1_TE | USART_CR1_RE | USART_CR1_OVER8 | USART_CR1_FIFOEN), UART_MODE_TX_RX);
   CLEAR_BIT(CELL_UART->CR2, USART_CR2_STOP);
   MODIFY_REG(CELL_UART->CR3, (USART_CR3_RTSE | USART_CR3_CTSE | USART_CR3_ONEBIT | USART_CR3_TXFTCFG | USART_CR3_RXFTCFG), UART_HWCONTROL_RTS_CTS);
   CLEAR_BIT(CELL_UART->PRESC, USART_PRESC_PRESCALER);
   WRITE_REG(CELL_UART->BRR, (uint16_t)UART_DIV_SAMPLING16(HAL_RCC_GetPCLK1Freq(), CELL_MODEM_DESIRED_BAUD_RATE, 0));
   CLEAR_BIT(CELL_UART->CR2, (USART_CR2_LINEN | USART_CR2_CLKEN));
   CLEAR_BIT(CELL_UART->CR3, (USART_CR3_SCEN | USART_CR3_HDSEL | USART_CR3_IREN));
   SET_BIT(CELL_UART->CR1, USART_CR1_UE);
   if (READ_BIT(CELL_UART->CR1, USART_CR1_TE))
      while (!READ_BIT(CELL_UART->ISR, USART_ISR_TEACK));
   if (READ_BIT(CELL_UART->CR1, USART_CR1_RE))
      while (!READ_BIT(CELL_UART->ISR, USART_ISR_REACK));
   NVIC_SetPriority(CELL_UART_CONCAT(, CELL_UART_TYPE, CELL_UART_NUMBER, _IRQn), NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
   NVIC_EnableIRQ(CELL_UART_CONCAT(, CELL_UART_TYPE, CELL_UART_NUMBER, _IRQn));

   // Configure the CELL UART RX DMA buffer addresses and sizes
   WRITE_REG(DMA2_Stream1->PAR, (uint32_t)&CELL_UART->RDR);
   WRITE_REG(DMA2_Stream1->M0AR, (uint32_t)cell_rx_buffer);
   WRITE_REG(DMA2_Stream1->NDTR, sizeof(cell_rx_buffer));
   CLEAR_BIT(DMA2_Stream1->CR, (DMA_IT_TC | DMA_IT_TE | DMA_IT_DME | DMA_IT_HT));

   // Configure the CELL UART TX DMA peripheral address
   WRITE_REG(DMA2_Stream3->PAR, (uint32_t)&CELL_UART->TDR);
   CLEAR_BIT(DMA2_Stream3->CR, (DMA_IT_TC | DMA_IT_TE | DMA_IT_DME | DMA_IT_HT));

   // Initialize the LPTIM3 peripheral as the device info update timer
   CLEAR_BIT(LPTIM3->CR, LPTIM_CR_ENABLE);
   while (READ_BIT(LPTIM3->CR, LPTIM_CR_ENABLE));
   MODIFY_REG(LPTIM3->CFGR, (LPTIM_CFGR_CKSEL | LPTIM_CFGR_TRIGEN | LPTIM_CFGR_PRELOAD | LPTIM_CFGR_WAVPOL | LPTIM_CFGR_PRESC | LPTIM_CFGR_COUNTMODE), LPTIM_PRESCALER_DIV128);
   WRITE_REG(LPTIM3->IER, LPTIM_IT_ARRM);
   SET_BIT(LPTIM3->CR, LPTIM_CR_ENABLE);
   WRITE_REG(LPTIM3->ARR, CELL_TIMER_1MIN_COUNT);
   while (READ_REG(LPTIM3->ARR) != CELL_TIMER_1MIN_COUNT);
   NVIC_SetPriority(LPTIM3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
   NVIC_EnableIRQ(LPTIM3_IRQn);

   // Initialize the LPTIM4 peripheral as the command timeout timer
   CLEAR_BIT(LPTIM4->CR, LPTIM_CR_ENABLE);
   while (READ_BIT(LPTIM4->CR, LPTIM_CR_ENABLE));
   MODIFY_REG(LPTIM4->CFGR, (LPTIM_CFGR_CKSEL | LPTIM_CFGR_TRIGEN | LPTIM_CFGR_PRELOAD | LPTIM_CFGR_WAVPOL | LPTIM_CFGR_PRESC | LPTIM_CFGR_COUNTMODE), LPTIM_PRESCALER_DIV64);
   WRITE_REG(LPTIM4->IER, LPTIM_IT_ARRM);
   SET_BIT(LPTIM4->CR, LPTIM_CR_ENABLE);
   NVIC_SetPriority(LPTIM4_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
   NVIC_EnableIRQ(LPTIM4_IRQn);

   // Initialize the LPTIM5 peripheral as the hold-off timer
   CLEAR_BIT(LPTIM5->CR, LPTIM_CR_ENABLE);
   while (READ_BIT(LPTIM5->CR, LPTIM_CR_ENABLE));
   MODIFY_REG(LPTIM5->CFGR, (LPTIM_CFGR_CKSEL | LPTIM_CFGR_TRIGEN | LPTIM_CFGR_PRELOAD | LPTIM_CFGR_WAVPOL | LPTIM_CFGR_PRESC | LPTIM_CFGR_COUNTMODE), (LPTIM_PRESCALER_DIV128 | LPTIM_CFGR_TIMOUT));
   WRITE_REG(LPTIM5->IER, LPTIM_IT_ARRM);
   SET_BIT(LPTIM5->CR, LPTIM_CR_ENABLE);
   WRITE_REG(LPTIM5->ARR, CELL_TIMER_20MS_COUNT);
   while (READ_REG(LPTIM5->ARR) != CELL_TIMER_20MS_COUNT);
   NVIC_SetPriority(LPTIM5_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
   NVIC_EnableIRQ(LPTIM5_IRQn);

   // Start listening for packets using DMA until an idle line is detected
   SET_BIT(DMA2_Stream1->CR, DMA_SxCR_EN);
   ATOMIC_SET_BIT(CELL_UART->CR3, (USART_CR3_DMAR | USART_CR3_DMAT));
   WRITE_REG(CELL_UART->ICR, UART_CLEAR_IDLEF);
   ATOMIC_SET_BIT(CELL_UART->CR1, USART_CR1_IDLEIE);

   // Wait for the cellular modem to become available
   set_command_timeout(CELL_MODEM_STATUS_BOOT_TIME_MS / CELL_TIMER_MS_PER_TICK);
   while (!cell_modem_available && !timed_out);
#ifdef FORCE_FACTORY_RESET
   cell_reset_modem(1);
   while (!timed_out);
#endif

   // Take additional steps if the modem is still not online
   while (!cell_modem_available)
   {
      // Send a command twice, once for auto-bauding and once for validation
      cell_send_command_await_response(CELL_DISABLE_ECHO_MSG, sizeof(CELL_DISABLE_ECHO_MSG), 1000);
      cell_send_command_await_response(CELL_DISABLE_ECHO_MSG, sizeof(CELL_DISABLE_ECHO_MSG), 1000);

      // Hard-reset modem if still no response
      if (!cell_modem_available)
         cell_reset_modem(1);

      // Determine if basic persistent modem parameters need to be configured
      if (cell_modem_available)
      {
         // If auto-baud is enabled, persistent parameters have not been configured
         for (uint32_t retries = 0; (retries < 3) && !cell_send_command_await_response(CELL_GET_BAUD_RATE_MSG, sizeof(CELL_GET_BAUD_RATE_MSG), 500); ++retries);
         if (baud_rate != CELL_MODEM_DESIRED_BAUD_RATE)
         {
            // Update all relevant configuration parameters
            for (uint32_t retries = 0; (retries < 3) && !cell_send_command_await_response(CELL_SET_BAUD_RATE_MSG, sizeof(CELL_SET_BAUD_RATE_MSG), 500); ++retries);
            for (uint32_t retries = 0; (retries < 3) && !cell_send_command_await_response(CELL_DISABLE_ECHO_MSG, sizeof(CELL_DISABLE_ECHO_MSG), 500); ++retries);
            for (uint32_t retries = 0; (retries < 3) && !cell_send_command_await_response(CELL_USE_NUMERIC_ERRORS_MSG, sizeof(CELL_USE_NUMERIC_ERRORS_MSG), 500); ++retries);
            for (uint32_t retries = 0; (retries < 3) && !cell_send_command_await_response(CELL_DISABLE_PSM_SLEEP_MSG, sizeof(CELL_DISABLE_PSM_SLEEP_MSG), 500); ++retries);
            for (uint32_t retries = 0; (retries < 3) && !cell_send_command_await_response(CELL_ENABLE_STATUS_LED_MSG, sizeof(CELL_ENABLE_STATUS_LED_MSG), 500); ++retries);
            for (uint32_t retries = 0; (retries < 3) && !cell_send_command_await_response(CELL_STORE_PERSISTENT_MSG, sizeof(CELL_STORE_PERSISTENT_MSG), 2000); ++retries);
            cell_reset_modem(0);
         }
      }
   }

   // Configure all general non-persistent modem parameters
   if (cell_configure_modem())
   {
      // Start a continuous device info update timer if enabled
      if (device_info.device_config.device_status_transmission_interval_minutes)
         SET_BIT(LPTIM3->CR, LPTIM_CR_CNTSTRT);
   }
}

void cell_update_state(void)
{
   // Blocking reconfiguration takes priority; reset non-blocking state to avoid stale operations
   if (configure_modem)
   {
      cell_step = CELL_STEP_IDLE;
      cell_busy = 0;
      mqtt_operation_awaiting_ack = MQTT_DONE;
      cell_configure_modem();
   }

   // Handle connectivity changes (blocking is acceptable here)
   if (connectivity_changed)
   {
      // Check whether the device was successfully registered on a network
      connectivity_changed = 0;
      if (valid_cgreg || valid_cereg)
      {
         // Ensure that each dependent network connection is made in order
         if (!valid_pdp)
         {
            if (bad_pdp_timer_count >= CELL_BAD_PDP_TIMEOUT_MINUTES)
               cell_reboot_firmware(1);
            else
               cell_send_command_await_response(CELL_PDP_ACTIVATE_MSG, sizeof(CELL_PDP_ACTIVATE_MSG), 40000);
         }
         else if (!mqtt_connected)
            cell_mqtt_connect();
         else if (!mqtt_subscribed)
         {
            cell_send_command_await_response(CELL_GET_SIGNAL_QUALITY_MSG, sizeof(CELL_GET_SIGNAL_QUALITY_MSG), 1000);
            cell_mqtt_subscribe();
         }
      }
      else if (bad_network_conn_timer_count >= CELL_BAD_CONN_TIMEOUT_MINUTES)
         cell_reboot_firmware(0);
   }

   // Auto-disable test mode and mark a device info update as pending
   if (device_info.device_config.test_mode_start_time && ((uint32_t)data.packets[0].timestamp >= (device_info.device_config.test_mode_start_time + TEST_MODE_AUTO_DISABLE_SECONDS)))
   {
      device_update_timer_count = 0;
      device_info.device_config.test_mode_start_time = 0;
      device_info_update = 1;
      chip_save_config(0);
   }

   // Advance the non-blocking steady-state publish state machine
   switch (cell_step)
   {
      case CELL_STEP_WAIT_SIG_QUAL:
         // Signal quality poll complete — start the device info publish
         if (command_acked || command_nacked || timed_out)
            cell_start_nonblocking_publish(CELL_PUB_DEVICE_INFO);
         return;

#ifdef CELL_MQTT_USE_BINARY_PUBLISH

      case CELL_STEP_WAIT_PUB_PROMPT:
         // Waiting for the binary data prompt character '>'
         if (!prompt_received && !timed_out)
            return;
         if (prompt_received)
         {
            // Prompt received, send the binary payload
            const char *data_ptr = NULL;
            uint32_t data_len = cell_pending_cmd_len;
            in_holdoff_period = 0;
            switch (cell_pending_pub)
            {
               case CELL_PUB_DEVICE_INFO: data_ptr = (const char*)&device_info; break;
               case CELL_PUB_ALERT: data_ptr = (const char*)&cell_alert_tx_ring[cell_alert_tx_head]; break;
               case CELL_PUB_AUDIO: data_ptr = (const char*)&cell_audio_tx_ring[cell_audio_tx_head].message; break;
            }
            command_acked = command_nacked = 0;
            cell_send_command((char*)data_ptr, data_len + 1);
            set_command_timeout(1000 / CELL_TIMER_MS_PER_TICK);
            cell_step = CELL_STEP_WAIT_PUB_CMD_ACK;
         }
         else
         {
            // Timed out waiting for prompt
            mqtt_operation_awaiting_ack = MQTT_DONE;
            cell_busy = 0;
            cell_step = CELL_STEP_IDLE;
            cell_advance_tx_rings();
         }
         return;

#endif

      case CELL_STEP_WAIT_PUB_CMD_ACK:
         // Waiting for the AT command-level "OK" acknowledgment
         if (command_acked && mqtt_connected)
         {
#ifndef USE_MQTT_SN
            // For QoS 0, AWS IoT Core sends no PUBACK URC — advance immediately
            const uint8_t pub_qos = (cell_pending_pub == CELL_PUB_ALERT) ? device_info.device_config.mqtt_alert_qos : ((cell_pending_pub == CELL_PUB_AUDIO) ? device_info.device_config.mqtt_audio_qos :  device_info.device_config.mqtt_device_info_qos);
            if (pub_qos == 0)
            {
               mqtt_operation_awaiting_ack = MQTT_DONE;
               cell_busy = 0;
               cell_step = CELL_STEP_IDLE;
               cell_advance_tx_rings();
               return;
            }
#endif
            set_command_timeout(CELL_MAX_NETWORK_RESPONSE_MS / CELL_TIMER_MS_PER_TICK);
            cell_step = CELL_STEP_WAIT_PUB_NET_ACK;
         }
         else if (command_acked || command_nacked || timed_out)
         {
            mqtt_operation_awaiting_ack = MQTT_DONE;
            cell_busy = 0;
            cell_step = CELL_STEP_IDLE;
            cell_advance_tx_rings();
         }
         return;

      case CELL_STEP_WAIT_PUB_NET_ACK:
         // Waiting for the network-level publish acknowledgment URC
         if (mqtt_connected && !timed_out && (mqtt_operation_awaiting_ack != MQTT_DONE))
            return;
         cell_busy = 0;
         cell_step = CELL_STEP_IDLE;
         cell_advance_tx_rings();
         return;

      case CELL_STEP_WAIT_MSG_READ:
         // Waiting for the broker message read response
         if (!command_acked && !command_nacked && !timed_out)
            return;
         if (incoming_message)
            cell_process_network_message(incoming_message, incoming_message_length);
         cell_busy = 0;
         cell_step = CELL_STEP_IDLE;
         return;

      case CELL_STEP_IDLE:
      default:
         break;
   }

   // Start new steady-state operations when the modem is free and connected
   if (cell_busy || !mqtt_connected)
      return;

   // Priority 1: publish a pending device info update (poll signal quality first if PDP is active)
   if (device_info_update)
   {
      device_info_update = 0;
      if (valid_pdp)
      {
         command_acked = command_nacked = 0;
         cell_send_command(CELL_GET_SIGNAL_QUALITY_MSG, sizeof(CELL_GET_SIGNAL_QUALITY_MSG));
         set_command_timeout(1000 / CELL_TIMER_MS_PER_TICK);
         cell_step = CELL_STEP_WAIT_SIG_QUAL;
      }
      else
         cell_start_nonblocking_publish(CELL_PUB_DEVICE_INFO);
      return;
   }

   // Priority 2: poll the modem to read each buffered message from the broker
   if (pending_messages)
   {
      cell_busy = 1;
      incoming_message = 0;
      incoming_message_length = 0;
      mqtt_operation_awaiting_ack = MQTT_READ;
      command_acked = command_nacked = 0;
#ifdef USE_MQTT_SN
      cell_send_command(CELL_MQTTSN_READ_MSG, sizeof(CELL_MQTTSN_READ_MSG));
#else
      cell_send_command(CELL_MQTT_READ_MSG, sizeof(CELL_MQTT_READ_MSG));
#endif
      set_command_timeout(16000 / CELL_TIMER_MS_PER_TICK);
      cell_step = CELL_STEP_WAIT_MSG_READ;
      return;
   }

   // Priority 3: drain the outgoing alert ring buffer
   if (cell_alert_tx_count)
   {
      cell_start_nonblocking_publish(CELL_PUB_ALERT);
      return;
   }

   // Priority 4: drain the outgoing audio ring buffer
   if (cell_audio_tx_count)
   {
      cell_start_nonblocking_publish(CELL_PUB_AUDIO);
      return;
   }
}

uint8_t cell_pending_events(void)
{
   // Return whether there are any pending events to be handled
   return configure_modem || connectivity_changed || device_info_update || pending_messages ||
          cell_alert_tx_count || cell_audio_tx_count || (cell_step != CELL_STEP_IDLE);
}

void cell_update_device_details(void)
{
   // Update the device info packet with the most current details if not busy
   if (!cell_busy)
   {
      device_info.lat = data.packets[0].lat; device_info.lon = data.packets[0].lon; device_info.ht = data.packets[0].ht;
      device_info.q1 = data.packets[0].q1; device_info.q2 = data.packets[0].q2; device_info.q3 = data.packets[0].q3;
      device_info.signal_power = signal_power;
      device_info.signal_quality = signal_quality;
   }
}

void cell_transmit_alert(alert_message_t *alert)
{
   // Piggyback the current device details onto the event alert message
   device_update_timer_count = 0;
   alert->cell_signal_power = signal_power;
   alert->cell_signal_quality = signal_quality;
   alert->sensor_q1 = device_info.q1;
   alert->sensor_q2 = device_info.q2;
   alert->sensor_q3 = device_info.q3;
   alert->sensor_lat = device_info.lat;
   alert->sensor_lon = device_info.lon;
   alert->sensor_ht = device_info.ht;

   // Enqueue the alert into a ring buffer
   if (cell_alert_tx_count < CELL_ALERT_TX_RING_SIZE)
   {
      const uint32_t slot = (cell_alert_tx_head + cell_alert_tx_count) % CELL_ALERT_TX_RING_SIZE;
      cell_alert_tx_ring[slot] = *alert;
      ++cell_alert_tx_count;
   }
   else
   {
      // Overwrite the oldest entry if full
      cell_alert_tx_ring[cell_alert_tx_head] = *alert;
      cell_alert_tx_head = (cell_alert_tx_head + 1) % CELL_ALERT_TX_RING_SIZE;
   }
}

uint8_t cell_transmit_audio(const opus_frame_t *restrict audio_frame, uint8_t is_final_frame)
{
   // Append the audio frame to the current evidence packet, splitting if it would overflow
   static uint16_t evidence_message_idx = 0;
   const uint16_t space_remaining = CELL_EVIDENCE_MAX_PAYLOAD_SIZE - evidence_message_idx;
   const uint16_t to_copy = audio_frame->num_encoded_bytes + offsetof(opus_frame_t, encoded_data);
   if (space_remaining < to_copy)
   {
      arm_copy_q7((q7_t*)audio_frame, (q7_t*)evidence_message.data + evidence_message_idx, space_remaining);
      cell_enqueue_audio(evidence_message_idx + space_remaining);
      evidence_message_idx = to_copy - space_remaining;
      arm_copy_q7((q7_t*)audio_frame + space_remaining, (q7_t*)evidence_message.data, evidence_message_idx);
      ++evidence_message.message_idx_and_final;
   }
   else
   {
      arm_copy_q7((q7_t*)audio_frame, (q7_t*)evidence_message.data + evidence_message_idx, to_copy);
      evidence_message_idx += to_copy;
   }

   // Enqueue the evidence packet if full or if this is the final audio frame
   const uint8_t transmitted_clip_id = evidence_message.clip_id;
   if (is_final_frame || (evidence_message_idx == CELL_EVIDENCE_MAX_PAYLOAD_SIZE))
   {
      evidence_message.message_idx_and_final |= is_final_frame ? CELL_MQTT_MESSAGE_FINAL_MASK : 0;
      cell_enqueue_audio(evidence_message_idx);

      evidence_message_idx = 0;
      if (is_final_frame)
      {
         evidence_message.clip_id++;
         evidence_message.message_idx_and_final = 0;
      }
      else
         evidence_message.message_idx_and_final++;
   }
   return transmitted_clip_id;
}

uint8_t cell_is_busy(void)
{
   // Return whether the cellular modem is currently in use
   return cell_busy;
}

uint8_t cell_at_cert_write_begin(const char *cmd, uint32_t cmd_len, uint32_t timeout_ms)
{
   // Send an AT+USECMNG=0,... command and block until the '>' data prompt is received
   mqtt_operation_awaiting_ack = MQTT_PUBLISH_BINARY;
   prompt_received = 0;
   command_acked = command_nacked = 0;
   cell_send_command((char*)cmd, cmd_len);
   set_command_timeout(timeout_ms / CELL_TIMER_MS_PER_TICK);
   while (!prompt_received && !command_nacked && !timed_out)
   {
      cpu_feed_watchdog();
      cpu_sleep();
   }
   return prompt_received;
}

void cell_at_cert_write_data(const char *data, uint32_t len)
{
   // Send the raw PEM byte stream after the '>' prompt has been received
   mqtt_operation_awaiting_ack = MQTT_DONE;
   in_holdoff_period = 0;
   command_acked = command_nacked = 0;
   cell_send_command((char*)data, len + 1);
}

uint8_t cell_at_cert_write_end(uint32_t timeout_ms)
{
   // Block until the modem confirms certificate storage with an OK response
   set_command_timeout(timeout_ms / CELL_TIMER_MS_PER_TICK);
   while (!command_acked && !command_nacked && !timed_out)
   {
      cpu_feed_watchdog();
      cpu_sleep();
   }
   return command_acked;
}

uint8_t cell_at_cmd(const char *cmd, uint32_t cmd_len, uint32_t timeout_ms)
{
   return cell_send_command_await_response((char*)cmd, cmd_len, timeout_ms);
}

#endif  // #ifdef CORE_CM4
