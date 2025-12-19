#ifdef CORE_CM4

// Header Inclusions ---------------------------------------------------------------------------------------------------

#include "gps.h"
#include "system.h"


// GPS UBX Protocol Definitions ----------------------------------------------------------------------------------------

#define GPS_CFG_RESPONSE_TIMEOUT_MS    1000

#define UBX_MSG_SYNC1_OFFSET           0
#define UBX_MSG_SYNC2_OFFSET           1
#define UBX_MSG_CLASS_OFFSET           2
#define UBX_MSG_ID_OFFSET              3
#define UBX_MSG_LEN_OFFSET             4
#define UBX_MSG_PAYLOAD_OFFSET         6

#define UBX_MAX_PAYLOAD_SIZE           400
#define UBX_MSG_CHKSUM_LEN             2
#define UBX_PACKET_OVERHEAD            (UBX_MSG_PAYLOAD_OFFSET + UBX_MSG_CHKSUM_LEN)
#define UBX_MAX_PACKET_SIZE            (UBX_MAX_PAYLOAD_SIZE + UBX_PACKET_OVERHEAD)

#define UBX_SYNC1_CHAR                 0xB5
#define UBX_SYNC2_CHAR                 0x62
#define UBX_SYNC                       UBX_SYNC1_CHAR, UBX_SYNC2_CHAR
#define UBX_MSG_LEN_PLACEHOLDER        0x00, 0x00
#define UBX_MSG_CKSUM_PLACEHOLDER      0x00, 0x00

#define UBX_NAV_SIG_MSG                0x01, 0x43

#define UBX_CFG_RESET                  0x06, 0x04
#define UBX_CFG_RESET_DATA             0x00, 0x00, 0x01, 0x00

#define UBX_CFG_VALGET_MSG             0x06, 0x8B
#define UBX_CFG_VALSET_MSG             0x06, 0x8A
#define UBX_CFG_VALGET_BEGIN           0x00, 0x00, 0x00, 0x00
#define UBX_CFG_VALGET_RESPONSE        0x01, 0x00, 0x00, 0x00
#define UBX_CFG_VALSET_BEGIN           0x00, 0x03, 0x00, 0x00

#define CFG_I2C_ENABLED                0x03, 0x00, 0x51, 0x10
#define CFG_SPI_ENABLED                0x06, 0x00, 0x64, 0x10
#define CFG_UART1INPROT_UBX            0x01, 0x00, 0x73, 0x10
#define CFG_UART1OUTPROT_UBX           0x01, 0x00, 0x74, 0x10
#define CFG_UART1INPROT_NMEA           0x02, 0x00, 0x73, 0x10
#define CFG_UART1OUTPROT_NMEA          0x02, 0x00, 0x74, 0x10
#define CFG_UART1_BAUDRATE             0x01, 0x00, 0x52, 0x40

#define CFG_SIGNAL_GPS_ENA             0x1F, 0x00, 0x31, 0x10
#define CFG_SIGNAL_GPS_L1CA_ENA        0x01, 0x00, 0x31, 0x10
#define CFG_SIGNAL_SBAS_ENA            0x20, 0x00, 0x31, 0x10
#define CFG_SIGNAL_SBAS_L1CA_ENA       0x05, 0x00, 0x31, 0x10
#define CFG_SIGNAL_GAL_ENA             0x21, 0x00, 0x31, 0x10
#define CFG_SIGNAL_GAL_E1_ENA          0x07, 0x00, 0x31, 0x10
#define CFG_SIGNAL_BDS_ENA             0x22, 0x00, 0x31, 0x10
#define CFG_SIGNAL_BDS_B1_ENA          0x0D, 0x00, 0x31, 0x10
#define CFG_SIGNAL_BDS_B1C_ENA         0x0F, 0x00, 0x31, 0x10
#define CFG_SIGNAL_GLO_ENA             0x25, 0x00, 0x31, 0x10
#define CFG_SIGNAL_GLO_L1_ENA          0x18, 0x00, 0x31, 0x10

#define CFG_MSGOUT_UBX_NAV_PVT_UART1   0x07, 0x00, 0x91, 0x20
#define CFG_MSGOUT_UBX_TIM_TM2_UART1   0x79, 0x01, 0x91, 0x20

#define CFG_NAVSPG_DYNMODEL            0x21, 0x00, 0x11, 0x20
#define CFG_NAVSPG_FIXMODE             0x11, 0x00, 0x11, 0x20
#define CFG_NAVSPG_INIFIX3D            0x13, 0x00, 0x11, 0x10
#define CFG_NAVSPG_UTCSTANDARD         0x1C, 0x00, 0x11, 0x20
#define CFG_SBAS_USE_RANGING           0x03, 0x00, 0x36, 0x10
#define CFG_SBAS_USE_DIFFCORR          0x04, 0x00, 0x36, 0x10
#define CFG_SBAS_USE_INTEGRITY         0x05, 0x00, 0x36, 0x10
#define CFG_TP_TP1_ENA                 0x07, 0x00, 0x05, 0x10
#define CFG_TP_TIMEGRID_TP1            0x0C, 0x00, 0x05, 0x20
#define CFG_RATE_MEAS                  0x01, 0x00, 0x21, 0x30
#define CFG_RATE_NAV                   0x02, 0x00, 0x21, 0x30
#define CFG_RATE_TIMEREF               0x03, 0x00, 0x21, 0x20

#define LNA_GAIN_NORMAL                0x00
#define LNA_GAIN_LOW                   0x01
#define LNA_GAIN_BYPASS                0x02
#define CFG_HW_RF_LNA_MODE             0x57, 0x00, 0xA3, 0x20
#define HW_RF_LNA_MODE                 LNA_GAIN_NORMAL  // TODO: CHECK IF CHANGING THIS HELPS (AFTER V_BCKP IS CONNECTED SO THAT BBR DOESN'T GET ERASED DURING HW RESET)

#define UBX_GET_INTERFACE_CFG_DATA     CFG_I2C_ENABLED, CFG_SPI_ENABLED, CFG_UART1INPROT_UBX, CFG_UART1OUTPROT_UBX, CFG_UART1INPROT_NMEA, CFG_UART1OUTPROT_NMEA
#define UBX_SET_INTERFACE_CFG_DATA     CFG_I2C_ENABLED, 0x00, CFG_SPI_ENABLED, 0x00, CFG_UART1INPROT_UBX, 0x01, CFG_UART1OUTPROT_UBX, 0x01, CFG_UART1INPROT_NMEA, 0x00, CFG_UART1OUTPROT_NMEA, 0x00

#define UBX_GET_GNSS_CFG_DATA          CFG_SIGNAL_GPS_ENA, CFG_SIGNAL_GPS_L1CA_ENA, CFG_SIGNAL_SBAS_ENA, CFG_SIGNAL_SBAS_L1CA_ENA, CFG_SIGNAL_GAL_ENA, CFG_SIGNAL_GAL_E1_ENA, CFG_SIGNAL_BDS_ENA, CFG_SIGNAL_BDS_B1_ENA, CFG_SIGNAL_BDS_B1C_ENA, CFG_SIGNAL_GLO_ENA, CFG_SIGNAL_GLO_L1_ENA
#define UBX_SET_GNSS_CFG_DATA          CFG_SIGNAL_GPS_ENA, 0x01, CFG_SIGNAL_GPS_L1CA_ENA, 0x01, CFG_SIGNAL_SBAS_ENA, 0x01, CFG_SIGNAL_SBAS_L1CA_ENA, 0x01, CFG_SIGNAL_GAL_ENA, 0x01, CFG_SIGNAL_GAL_E1_ENA, 0x01, CFG_SIGNAL_BDS_ENA, 0x01, CFG_SIGNAL_BDS_B1_ENA, 0x00, CFG_SIGNAL_BDS_B1C_ENA, 0x01, CFG_SIGNAL_GLO_ENA, 0x01, CFG_SIGNAL_GLO_L1_ENA, 0x01

#define UBX_GET_MSG_CFG_DATA           CFG_MSGOUT_UBX_NAV_PVT_UART1, CFG_MSGOUT_UBX_TIM_TM2_UART1
#define UBX_SET_MSG_CFG_DATA           CFG_MSGOUT_UBX_NAV_PVT_UART1, 0x01, CFG_MSGOUT_UBX_TIM_TM2_UART1, 0x01

#define UBX_GET_GEN_CFG_DATA           CFG_NAVSPG_DYNMODEL, CFG_NAVSPG_FIXMODE, CFG_NAVSPG_INIFIX3D, CFG_NAVSPG_UTCSTANDARD, CFG_SBAS_USE_RANGING, CFG_SBAS_USE_DIFFCORR, CFG_SBAS_USE_INTEGRITY, CFG_TP_TP1_ENA, CFG_TP_TIMEGRID_TP1, CFG_RATE_MEAS, CFG_RATE_NAV, CFG_RATE_TIMEREF
#define UBX_SET_GEN_CFG_DATA           CFG_NAVSPG_DYNMODEL, 0x02, CFG_NAVSPG_FIXMODE, 0x03, CFG_NAVSPG_INIFIX3D, 0x01, CFG_NAVSPG_UTCSTANDARD, 0x03, CFG_SBAS_USE_RANGING, 0x00, CFG_SBAS_USE_DIFFCORR, 0x01, CFG_SBAS_USE_INTEGRITY, 0x00, CFG_TP_TP1_ENA, 0x01, CFG_TP_TIMEGRID_TP1, 0x00, CFG_RATE_MEAS, 0xF4, 0x01, CFG_RATE_NAV, 0x02, 0x00, CFG_RATE_TIMEREF, 0x00

#define UBX_GET_LNA_DATA               CFG_HW_RF_LNA_MODE
#define UBX_SET_LNA_DATA               CFG_HW_RF_LNA_MODE, HW_RF_LNA_MODE

#define FIX_TYPE_2D                    0x02
#define FIX_TYPE_3D                    0x03

#define LNA_MSG_GAIN_OFFSET            8


// GPS Data Type Definitions -------------------------------------------------------------------------------------------

typedef enum
{
   UBX_PACKET_INIT_STATE,
   UBX_PACKET_SYNC_STATE,
   UBX_PACKET_CLASS_STATE,
   UBX_PACKET_ID_STATE,
   UBX_PACKET_LEN1_STATE,
   UBX_PACKET_LEN2_STATE,
   UBX_PACKET_PAYLOAD_STATE,
   UBX_PACKET_CK_A_STATE,
   UBX_PACKET_CK_B_STATE
} ubx_packet_state_t;

typedef enum
{
   UBX_MSG_UNKNOWN,
   UBX_MON_VER,
   UBX_NAV_PVT,
   UBX_TIM_TM2,
   UBX_CFG_VALGET,
   UBX_NAV_SIG,
   UBX_ACK_ACK
} ubx_message_type_t;

typedef enum
{
   GPS_BAUD_4800 = 4800,
   GPS_BAUD_9600 = 9600,
   GPS_BAUD_19200 = 19200,
   GPS_BAUD_38400 = 38400,
   GPS_BAUD_57600 = 57600,
   GPS_BAUD_115200 = 115200,
   GPS_BAUD_230400 = 230400,
   GPS_BAUD_460800 = 460800,
   GPS_BAUD_921600 = 921600,
} gps_baud_t;

typedef struct __attribute__ ((packed))
{
   uint32_t iTOW;
   uint16_t year;
   uint8_t month, day, hour, min, sec;
   union {
      uint8_t valid;
      struct __attribute__ ((packed)) {
         uint8_t validDate: 1;
         uint8_t validTime: 1;
         uint8_t fullyResolved: 1;
         uint8_t validMag: 1;
      };
   };
   uint32_t tAcc;
   int32_t nano;
   uint8_t fixType;
   union {
      uint8_t flags;
      struct __attribute__ ((packed)) {
         uint8_t gnssFixOK: 1;
         uint8_t diffSoln: 1;
         uint8_t psmState: 3;
         uint8_t headVehValid: 1;
         uint8_t carrSoln: 2;
      };
   };
   union {
      uint8_t flags2;
      struct __attribute__ ((packed)) {
         uint8_t confirmedAvai: 1;
         uint8_t confirmedDate: 1;
         uint8_t confirmedTime: 1;
      };
   };
   uint8_t numSV;
   int32_t lon, lat, height, hMSL;
   uint32_t hAcc, vAcc;
   int32_t velN, velE, velD, gSpeed, headMot;
   uint32_t sAcc, headAcc;
   uint16_t pDOP;
   union {
      uint16_t flags3;
      struct __attribute__ ((packed)) {
         uint8_t invalidLlh: 1;
         uint8_t lastCorrectionAge: 4;
         uint8_t unused: 8;
         uint8_t authTime: 1;
      };
   };
   uint8_t reserved0[4];
   int32_t headVeh;
   int16_t magDec;
   uint16_t magAcc;
} ubx_nav_pvt_t;

typedef struct __attribute__ ((packed))
{
   uint8_t ch;
   union {
      uint8_t flags;
      struct __attribute__ ((packed)) {
         uint8_t mode: 1;
         uint8_t run: 1;
         uint8_t newFallingEdge: 1;
         uint8_t timeBase: 2;
         uint8_t utc: 1;
         uint8_t time: 1;
         uint8_t newRisingEdge: 1;
      };
   };
   uint16_t count, wnR, wnF;
   uint32_t towMsR, towSubMsR, towMsF, towSubMsF;
   int32_t accEst;
} ubx_tim_tm2_t;

typedef struct __attribute__ ((packed))
{
   uint8_t gnssId, svId, sigId, freqId;
   int16_t prRes;
   uint8_t cno, qualityInd, corrSource, ionoModel;
   union {
      uint16_t flags;
      struct __attribute__ ((packed)) {
         uint8_t health: 2;
         uint8_t prSmoothed: 1;
         uint8_t prUsed: 1;
         uint8_t crUsed: 1;
         uint8_t doUsed: 1;
         uint8_t prCorrUsed: 1;
         uint8_t crCorrUsed: 1;
         uint8_t doCorrUsed: 1;
         uint8_t authStatus: 1;
      };
   };
   uint8_t reserved1[4];
} ubx_nav_sig_data_t;

typedef struct __attribute__ ((packed))
{
   uint32_t iTOW;
   uint8_t version, numSigs, reserved0[2];
   ubx_nav_sig_data_t data[1];
} ubx_nav_sig_t;


// GPS Static Variables ------------------------------------------------------------------------------------------------

#define GPS_UART_CONCAT(a,t,n,c)   gps_uart_concat(a,t,n,c)
#define gps_uart_concat(a,t,n,c)   a ## t ## n ## c

#define GPS_UART_IRQHandler        gps_uart_irq1(GPS_UART_TYPE, GPS_UART_NUMBER)
#define gps_uart_irq1(t,n)         gps_uart_irq(t,n)
#define gps_uart_irq(t,n)          t ## n ## _IRQHandler

#define GPS_UART                   gps_uart1(GPS_UART_TYPE, GPS_UART_NUMBER)
#define gps_uart1(t,n)             gps_uart(t,n)
#define gps_uart(t,n)              t ## n

#define GPS_RX_BUFFER_SIZE_HALF        (2 * UBX_MAX_PACKET_SIZE)
#define GPS_RX_BUFFER_SIZE_FULL        (2 * GPS_RX_BUFFER_SIZE_HALF)

__attribute__ ((aligned (4)))
static uint8_t gps_rx_buffer[GPS_RX_BUFFER_SIZE_FULL];

static volatile float lat_degrees = -1000.0f, lon_degrees = -1000.0f, height_meters = -1000.0f;
static volatile uint8_t gps_timepulse_fired = 0;
static volatile double next_timestamp = 0.0;


// Private Helper Functions --------------------------------------------------------------------------------------------

inline static double tm2_to_gps_timestamp(uint16_t week_number, uint32_t tow_ms, uint32_t tow_sub_ms)
{
   return 315964800.0 + ((double)week_number * 604800.0) + ((double)tow_ms * 0.001) + ((double)tow_sub_ms * 0.000000001);
}

inline static void gps_set_led_status(uint8_t on)
{
   // Set the GPS status LED to the requested state
   WRITE_REG(LED_GPS_STATUS_GPIO_Port->BSRR, on ? LED_GPS_STATUS_Pin : ((uint32_t)LED_GPS_STATUS_Pin << 16U));
}

static void gps_transmit(const uint8_t *data, uint16_t data_len)
{
   // Loop until all bytes have been transmitted
   while (data_len)
   {
      // Wait until ready to transmit
      while (!READ_BIT(GPS_UART->ISR, UART_FLAG_TXE));
      GPS_UART->TDR = *data;
      data++;
     --data_len;
   }
}

static void calculate_length_and_checksum(uint8_t *packet_start, size_t total_size)
{
   // Store the total packet length minus the framing and checksum
   *((uint16_t*)(packet_start + UBX_MSG_LEN_OFFSET)) = (uint16_t)(total_size - UBX_MSG_PAYLOAD_OFFSET - UBX_MSG_CHKSUM_LEN);
   uint8_t *const packet_end = packet_start + total_size - UBX_MSG_CHKSUM_LEN;
   packet_start += UBX_MSG_CLASS_OFFSET;

   // Compute and store the packet checksum
   uint8_t ck_a = 0, ck_b = 0;
   while (packet_start < packet_end)
   {
      ck_a += *(packet_start++);
      ck_b += ck_a;
   }
   packet_end[0] = ck_a;
   packet_end[1] = ck_b;
}

static ubx_message_type_t gps_process_message(const uint8_t* msg, uint16_t max_msg_len)
{
   // Identify the type of message received and copy it to the appropriate global location
   const uint16_t msg_len = *(uint16_t*)(msg + UBX_MSG_LEN_OFFSET);
   if (msg_len > max_msg_len)
      return UBX_MSG_UNKNOWN;
   else if ((msg[UBX_MSG_CLASS_OFFSET] == 0x01) && (msg[UBX_MSG_ID_OFFSET] == 0x07) && (msg_len == sizeof(ubx_nav_pvt_t)))
   {
      static uint8_t initial_fix_found = 0;
      const ubx_nav_pvt_t *message = (const ubx_nav_pvt_t*)(msg + UBX_MSG_PAYLOAD_OFFSET);
      if (message->gnssFixOK && ((message->fixType == FIX_TYPE_3D) || ((message->fixType == FIX_TYPE_2D) && !initial_fix_found)) &&
            (-900000000 <= message->lat) && (message->lat <= 900000000) && (-1800000000 <= message->lon) && (message->lon <= 1800000000))
      {
         initial_fix_found |= (message->fixType == FIX_TYPE_3D);
         lat_degrees = (float)message->lat * 1.0e-7f;
         lon_degrees = (float)message->lon * 1.0e-7f;
         height_meters = (float)message->height * 1.0e-3f;
         gps_set_led_status(1);
      }
      else
         gps_set_led_status(0);
      return UBX_NAV_PVT;
   }
   else if ((msg[UBX_MSG_CLASS_OFFSET] == 0x0D) && (msg[UBX_MSG_ID_OFFSET] == 0x03) && (msg_len == sizeof(ubx_tim_tm2_t)))
   {
      const ubx_tim_tm2_t *message = (const ubx_tim_tm2_t*)(msg + UBX_MSG_PAYLOAD_OFFSET);
      if (message->time && message->newRisingEdge && (message->towMsR <= 604800000))
         next_timestamp = tm2_to_gps_timestamp(message->wnR, message->towMsR, message->towSubMsR);
      return UBX_TIM_TM2;
   }
   else if ((msg[UBX_MSG_CLASS_OFFSET] == 0x06) && (msg[UBX_MSG_ID_OFFSET] == 0x8B))
      return UBX_CFG_VALGET;
   else if ((msg[UBX_MSG_CLASS_OFFSET] == 0x0A) && (msg[UBX_MSG_ID_OFFSET] == 0x04))
      return UBX_MON_VER;
   else if ((msg[UBX_MSG_CLASS_OFFSET] == 0x05) && ((msg[UBX_MSG_ID_OFFSET] == 0x00) || (msg[UBX_MSG_ID_OFFSET] == 0x01)))
      return UBX_ACK_ACK;
   return UBX_MSG_UNKNOWN;
}

static ubx_message_type_t gps_process_next_char(void)
{
   // Create static variables to keep track of GPS message reception progress
   static uint16_t gps_rx_idx = 0, gps_rx_len = 0;
   static uint8_t rx_byte, gps_rx_ck_a = 0, gps_rx_ck_b = 0;
   static ubx_packet_state_t ubx_packet_state = UBX_PACKET_INIT_STATE;

   // Read the next GPS message character
   if (!READ_BIT(GPS_UART->ISR, UART_FLAG_RXNE))
   {
      WRITE_REG(GPS_UART->ICR, UART_CLEAR_OREF | UART_CLEAR_RTOF | UART_CLEAR_NEF | UART_CLEAR_PEF | UART_CLEAR_FEF);
      return UBX_MSG_UNKNOWN;
   }
   rx_byte = (uint8_t)(GPS_UART->RDR & 0x00FFU);

   // Process incoming UBX bytes character-by-character
   switch (ubx_packet_state)
   {
      case UBX_PACKET_INIT_STATE:
         if (rx_byte == UBX_SYNC1_CHAR)
         {
            ubx_packet_state = UBX_PACKET_SYNC_STATE;
            gps_rx_idx = gps_rx_ck_a = gps_rx_ck_b = 0;
            gps_rx_buffer[gps_rx_idx++] = rx_byte;
         }
         break;
      case UBX_PACKET_SYNC_STATE:
         if (rx_byte == UBX_SYNC1_CHAR)
            ubx_packet_state = UBX_PACKET_SYNC_STATE;
         else if (rx_byte == UBX_SYNC2_CHAR)
         {
            gps_rx_buffer[gps_rx_idx++] = rx_byte;
            ubx_packet_state = UBX_PACKET_CLASS_STATE;
         }
         else
            ubx_packet_state = UBX_PACKET_INIT_STATE;
         break;
      case UBX_PACKET_CLASS_STATE:
         gps_rx_buffer[gps_rx_idx++] = rx_byte;
         gps_rx_ck_a += rx_byte;
         gps_rx_ck_b += gps_rx_ck_a;
         ubx_packet_state = UBX_PACKET_ID_STATE;
         break;
      case UBX_PACKET_ID_STATE:
         gps_rx_buffer[gps_rx_idx++] = rx_byte;
         gps_rx_ck_a += rx_byte;
         gps_rx_ck_b += gps_rx_ck_a;
         ubx_packet_state = UBX_PACKET_LEN1_STATE;
         break;
      case UBX_PACKET_LEN1_STATE:
         gps_rx_buffer[gps_rx_idx++] = rx_byte;
         gps_rx_ck_a += rx_byte;
         gps_rx_ck_b += gps_rx_ck_a;
         gps_rx_len = rx_byte;
         ubx_packet_state = UBX_PACKET_LEN2_STATE;
         break;
      case UBX_PACKET_LEN2_STATE:
         gps_rx_buffer[gps_rx_idx++] = rx_byte;
         gps_rx_ck_a += rx_byte;
         gps_rx_ck_b += gps_rx_ck_a;
         gps_rx_len |= (rx_byte << 8);
         if (gps_rx_len > UBX_MAX_PACKET_SIZE)
            ubx_packet_state = UBX_PACKET_INIT_STATE;
         else
            ubx_packet_state = (gps_rx_len > 0) ? UBX_PACKET_PAYLOAD_STATE : UBX_PACKET_CK_A_STATE;
         break;
      case UBX_PACKET_PAYLOAD_STATE:
         gps_rx_buffer[gps_rx_idx++] = rx_byte;
         gps_rx_ck_a += rx_byte;
         gps_rx_ck_b += gps_rx_ck_a;
         if ((gps_rx_idx - UBX_MSG_PAYLOAD_OFFSET) == gps_rx_len)
            ubx_packet_state = UBX_PACKET_CK_A_STATE;
         break;
      case UBX_PACKET_CK_A_STATE:
         if (gps_rx_ck_a != rx_byte)
            ubx_packet_state = UBX_PACKET_INIT_STATE;
         else
         {
            gps_rx_buffer[gps_rx_idx++] = rx_byte;
            ubx_packet_state = UBX_PACKET_CK_B_STATE;
         }
         break;
      case UBX_PACKET_CK_B_STATE:
         ubx_packet_state = UBX_PACKET_INIT_STATE;
         if (gps_rx_ck_b == rx_byte)
         {
            gps_rx_buffer[gps_rx_idx++] = rx_byte;
            return gps_process_message(gps_rx_buffer, gps_rx_len);
         }
         break;
      default:
         ubx_packet_state = UBX_PACKET_INIT_STATE;
         break;
   }
   return UBX_MSG_UNKNOWN;
}

static void gps_reset(void)
{
   // Force a hardware reset
   uint8_t ubx_cfg_rst[] = { UBX_SYNC, UBX_CFG_RESET, UBX_MSG_LEN_PLACEHOLDER, UBX_CFG_RESET_DATA, UBX_MSG_CKSUM_PLACEHOLDER };
   calculate_length_and_checksum(ubx_cfg_rst, sizeof(ubx_cfg_rst));
   gps_transmit(ubx_cfg_rst, sizeof(ubx_cfg_rst));
   HAL_Delay(200);
}

static uint8_t gps_transmit_and_await_response(ubx_message_type_t response_type, const uint8_t *message, uint16_t message_length)
{
   // Transmit the GPS message
   gps_transmit(message, message_length);

   // Await a response until the indicated timeout has elapsed
   const uint32_t tick_start = HAL_GetTick();
   while (gps_process_next_char() != response_type)
   {
      if ((HAL_GetTick() - tick_start) >= GPS_CFG_RESPONSE_TIMEOUT_MS)
         return 0;
   }
   return 1;
}

static void gps_send_and_receive(ubx_message_type_t response_type, const uint8_t *message, uint16_t message_length)
{
   // Attempt transmission with increasing levels of error correction
   for (uint8_t success = 0, retry = 0; !success && (retry < 7); ++retry)
   {
      // Reset the GPS or core chip if no response after 3 and 6 retries
      if (retry == 3)
         gps_reset();
      else if (retry == 6)
         chip_reset();

      // Wait until timeout or response received
      success = gps_transmit_and_await_response(response_type, message, message_length);
   }
}

static void gps_wait_until_ready(void)
{
   // Test the communication interface by polling the UBX-MON-VER message
   const uint8_t ubx_mon_ver[] = { UBX_SYNC, 0x0A, 0x04, 0x00, 0x00, 0x0E, 0x34 };
   gps_send_and_receive(UBX_MON_VER, ubx_mon_ver, sizeof(ubx_mon_ver));
}

static uint8_t gps_verify_or_set_interface_config(void)
{
   // Ensure that only UBX communications take place only over the UART interface
   // CFG-I2C-ENABLED=0, CFG-SPI-ENABLED=0, CFG-UART1INPROT-UBX=1, CFG-UART1OUTPROT-UBX=1, CFG-UART1INPROT-NMEA=0, CFG-UART1OUTPROT-NMEA=0
   uint8_t configuration_changed = 0;
   uint8_t ubx_interface_cfg_get[] = { UBX_SYNC, UBX_CFG_VALGET_MSG, UBX_MSG_LEN_PLACEHOLDER, UBX_CFG_VALGET_BEGIN, UBX_GET_INTERFACE_CFG_DATA, UBX_MSG_CKSUM_PLACEHOLDER };
   uint8_t ubx_interface_cfg_expected[] = { UBX_SYNC, UBX_CFG_VALGET_MSG, UBX_MSG_LEN_PLACEHOLDER, UBX_CFG_VALGET_RESPONSE, UBX_SET_INTERFACE_CFG_DATA, UBX_MSG_CKSUM_PLACEHOLDER };
   calculate_length_and_checksum(ubx_interface_cfg_get, sizeof(ubx_interface_cfg_get));
   calculate_length_and_checksum(ubx_interface_cfg_expected, sizeof(ubx_interface_cfg_expected));
   gps_send_and_receive(UBX_CFG_VALGET, ubx_interface_cfg_get, sizeof(ubx_interface_cfg_get));
   if (memcmp(gps_rx_buffer, ubx_interface_cfg_expected, sizeof(ubx_interface_cfg_expected)))
   {
      uint8_t ubx_interface_cfg_set[] = { UBX_SYNC, UBX_CFG_VALSET_MSG, UBX_MSG_LEN_PLACEHOLDER, UBX_CFG_VALSET_BEGIN, UBX_SET_INTERFACE_CFG_DATA, UBX_MSG_CKSUM_PLACEHOLDER };
      calculate_length_and_checksum(ubx_interface_cfg_set, sizeof(ubx_interface_cfg_set));
      gps_send_and_receive(UBX_ACK_ACK, ubx_interface_cfg_set, sizeof(ubx_interface_cfg_set));
      configuration_changed = 1;
   }
   return configuration_changed;
}

static uint8_t gps_verify_or_set_gnss_config(void)
{
   // CFG-SIGNAL-GPS_ENA=1, CFG-SIGNAL-GPS_L1CA_ENA=1, CFG-SIGNAL-SBAS_ENA=1, CFG-SIGNAL-SBAS_L1CA_ENA=1
   // CFG-SIGNAL-GAL_ENA=1, CFG-SIGNAL-GAL_E1_ENA=1, CFG-SIGNAL-BDS_ENA=1, CFG-SIGNAL-BDS_B1_ENA=0, CFG-SIGNAL-BDS_B1C_ENA=1
   // CFG-SIGNAL-GLO_ENA=1, CFG-SIGNAL-GLO_L1_ENA=1
   uint8_t configuration_changed = 0;
   uint8_t ubx_gnss_cfg_get[] = { UBX_SYNC, UBX_CFG_VALGET_MSG, UBX_MSG_LEN_PLACEHOLDER, UBX_CFG_VALGET_BEGIN, UBX_GET_GNSS_CFG_DATA, UBX_MSG_CKSUM_PLACEHOLDER };
   uint8_t ubx_gnss_cfg_expected[] = { UBX_SYNC, UBX_CFG_VALGET_MSG, UBX_MSG_LEN_PLACEHOLDER, UBX_CFG_VALGET_RESPONSE, UBX_SET_GNSS_CFG_DATA, UBX_MSG_CKSUM_PLACEHOLDER };
   calculate_length_and_checksum(ubx_gnss_cfg_get, sizeof(ubx_gnss_cfg_get));
   calculate_length_and_checksum(ubx_gnss_cfg_expected, sizeof(ubx_gnss_cfg_expected));
   gps_send_and_receive(UBX_CFG_VALGET, ubx_gnss_cfg_get, sizeof(ubx_gnss_cfg_get));
   if (memcmp(gps_rx_buffer, ubx_gnss_cfg_expected, sizeof(ubx_gnss_cfg_expected)))
   {
      uint8_t ubx_gnss_cfg_set[] = { UBX_SYNC, UBX_CFG_VALSET_MSG, UBX_MSG_LEN_PLACEHOLDER, UBX_CFG_VALSET_BEGIN, UBX_SET_GNSS_CFG_DATA, UBX_MSG_CKSUM_PLACEHOLDER };
      calculate_length_and_checksum(ubx_gnss_cfg_set, sizeof(ubx_gnss_cfg_set));
      gps_send_and_receive(UBX_ACK_ACK, ubx_gnss_cfg_set, sizeof(ubx_gnss_cfg_set));
      configuration_changed = 1;
      HAL_Delay(500);
   }
   return configuration_changed;
}

static uint8_t gps_verify_or_set_configuration(void)
{
   // Ensure that all messages are disabled except for UBX-NAV-PVT and UBX-TIM-TM2
   uint8_t configuration_changed = 0;
   uint8_t ubx_msg_cfg_get[] = { UBX_SYNC, UBX_CFG_VALGET_MSG, UBX_MSG_LEN_PLACEHOLDER, UBX_CFG_VALGET_BEGIN, UBX_GET_MSG_CFG_DATA, UBX_MSG_CKSUM_PLACEHOLDER };
   uint8_t ubx_msg_cfg_expected[] = { UBX_SYNC, UBX_CFG_VALGET_MSG, UBX_MSG_LEN_PLACEHOLDER, UBX_CFG_VALGET_RESPONSE, UBX_SET_MSG_CFG_DATA, UBX_MSG_CKSUM_PLACEHOLDER };
   calculate_length_and_checksum(ubx_msg_cfg_get, sizeof(ubx_msg_cfg_get));
   calculate_length_and_checksum(ubx_msg_cfg_expected, sizeof(ubx_msg_cfg_expected));
   gps_send_and_receive(UBX_CFG_VALGET, ubx_msg_cfg_get, sizeof(ubx_msg_cfg_get));
   if (memcmp(gps_rx_buffer, ubx_msg_cfg_expected, sizeof(ubx_msg_cfg_expected)))
   {
      uint8_t ubx_msg_cfg_set[] = { UBX_SYNC, UBX_CFG_VALSET_MSG, UBX_MSG_LEN_PLACEHOLDER, UBX_CFG_VALSET_BEGIN, UBX_SET_MSG_CFG_DATA, UBX_MSG_CKSUM_PLACEHOLDER };
      calculate_length_and_checksum(ubx_msg_cfg_set, sizeof(ubx_msg_cfg_set));
      gps_send_and_receive(UBX_ACK_ACK, ubx_msg_cfg_set, sizeof(ubx_msg_cfg_set));
      configuration_changed = 1;
   }

   // Ensure that the GPS configuration parameters are set as expected
   // CFG-NAVSPG-DYNMODEL=2 (Stationary), CFG-NAVSPG-FIXMODE=3 (Auto 2D/3D), CFG-NAVSPG-INIFIX3D=1 (True)
   // CFG-NAVSPG-UTCSTANDARD=3 (USNO, derived from GPS time), CFG-SBAS-USE_RANGING=0, CFG-SBAS-USE_DIFFCORR=1
   // CFG-SBAS-USE_INTEGRITY=0, CFG-TP-TP1_ENA=1 (Timepulse enabled), CFG-TP-TIMEGRID_TP1=0 (UTC time reference for timepulse)
   // CFG-RATE-MEAS=500 (2Hz), CFG-RATE-NAV=2 (1Hz), CFG-RATE-TIMEREF=0 (align measurements to UTC time)
   uint8_t ubx_general_cfg_get[] = { UBX_SYNC, UBX_CFG_VALGET_MSG, UBX_MSG_LEN_PLACEHOLDER, UBX_CFG_VALGET_BEGIN, UBX_GET_GEN_CFG_DATA, UBX_MSG_CKSUM_PLACEHOLDER };
   uint8_t ubx_general_cfg_expected[] = { UBX_SYNC, UBX_CFG_VALGET_MSG, UBX_MSG_LEN_PLACEHOLDER, UBX_CFG_VALGET_RESPONSE, UBX_SET_GEN_CFG_DATA, UBX_MSG_CKSUM_PLACEHOLDER };
   calculate_length_and_checksum(ubx_general_cfg_get, sizeof(ubx_general_cfg_get));
   calculate_length_and_checksum(ubx_general_cfg_expected, sizeof(ubx_general_cfg_expected));
   gps_send_and_receive(UBX_CFG_VALGET, ubx_general_cfg_get, sizeof(ubx_general_cfg_get));
   if (memcmp(gps_rx_buffer, ubx_general_cfg_expected, sizeof(ubx_general_cfg_expected)))
   {
      uint8_t ubx_general_cfg_set[] = { UBX_SYNC, UBX_CFG_VALSET_MSG, UBX_MSG_LEN_PLACEHOLDER, UBX_CFG_VALSET_BEGIN, UBX_SET_GEN_CFG_DATA, UBX_MSG_CKSUM_PLACEHOLDER };
      calculate_length_and_checksum(ubx_general_cfg_set, sizeof(ubx_general_cfg_set));
      gps_send_and_receive(UBX_ACK_ACK, ubx_general_cfg_set, sizeof(ubx_general_cfg_set));
      configuration_changed = 1;
   }
   return configuration_changed;
}

static uint8_t gps_verify_or_set_lna_gain(void)
{
   // Ensure that the LNA gain is set correctly
   uint8_t configuration_changed = 0;
   uint8_t ubx_valget_lna[] = { UBX_SYNC, UBX_CFG_VALGET_MSG, UBX_MSG_LEN_PLACEHOLDER, UBX_CFG_VALGET_BEGIN, UBX_GET_LNA_DATA, UBX_MSG_CKSUM_PLACEHOLDER };
   calculate_length_and_checksum(ubx_valget_lna, sizeof(ubx_valget_lna));
   gps_send_and_receive(UBX_CFG_VALGET, ubx_valget_lna, sizeof(ubx_valget_lna));
   if (gps_rx_buffer[UBX_MSG_PAYLOAD_OFFSET + LNA_MSG_GAIN_OFFSET] != HW_RF_LNA_MODE)
   {
      uint8_t ubx_valset_lna[] = { UBX_SYNC, UBX_CFG_VALSET_MSG, UBX_MSG_LEN_PLACEHOLDER, UBX_CFG_VALSET_BEGIN, UBX_SET_LNA_DATA, UBX_MSG_CKSUM_PLACEHOLDER };
      calculate_length_and_checksum(ubx_valset_lna, sizeof(ubx_valset_lna));
      gps_send_and_receive(UBX_ACK_ACK, ubx_valset_lna, sizeof(ubx_valset_lna));
      configuration_changed = 1;
      HAL_Delay(500);
   }
   return configuration_changed;
}

static void gps_poll_signal_data(void)
{
   // Send a request for data related to the currently visible satellite constellation
   uint8_t ubx_poll_nav_sig_msg[] = { UBX_SYNC, UBX_NAV_SIG_MSG, UBX_MSG_LEN_PLACEHOLDER, UBX_MSG_CKSUM_PLACEHOLDER };
   calculate_length_and_checksum(ubx_poll_nav_sig_msg, sizeof(ubx_poll_nav_sig_msg));
   gps_send_and_receive(UBX_NAV_SIG, ubx_poll_nav_sig_msg, sizeof(ubx_poll_nav_sig_msg));
}


// Interrupt Service Routines ------------------------------------------------------------------------------------------

void EXTI3_IRQHandler(void)
{
   // Clear the interrupt and set the timepulse notification flag
   WRITE_REG(EXTI->C2PR1, GPS_TIMEPULSE_Pin);
   gps_timepulse_fired = 1;
}

void GPS_UART_IRQHandler(void)
{
   // Create static indices to keep track of the location of unprocessed data
   static uint32_t previous_data_index = 0;
   static int32_t bytes_received;
   static uint8_t *msg_begin;

   // Freeze the contents of the DMA byte counter to ensure packet boundaries are maintained
   const uint32_t data_end_index = sizeof(gps_rx_buffer) - DMA1_Stream2->NDTR;
   __DMB();

   // Ensure that this interrupt was due to an idle line
   if (READ_BIT(GPS_UART->ISR, USART_ISR_IDLE))
   {
      // Clear the interrupt
      WRITE_REG(GPS_UART->ICR, UART_CLEAR_IDLEF);

      // Determine whether data wrapping is necessary due to the circular buffer
      if (data_end_index >= previous_data_index)
      {
         bytes_received = (int32_t)(data_end_index - previous_data_index);
         msg_begin = gps_rx_buffer + previous_data_index;
      }
      else
      {
         bytes_received = (int32_t)(sizeof(gps_rx_buffer) - previous_data_index + data_end_index);
         memmove(gps_rx_buffer + previous_data_index - data_end_index, gps_rx_buffer + previous_data_index, sizeof(gps_rx_buffer) - previous_data_index);
         memmove(gps_rx_buffer + sizeof(gps_rx_buffer) - data_end_index, gps_rx_buffer, data_end_index);
         msg_begin = gps_rx_buffer + previous_data_index - data_end_index;
      }

      // Process newly received data until no messages remain
      while ((bytes_received > 0) && (gps_process_message(msg_begin, (uint16_t)bytes_received) != UBX_MSG_UNKNOWN))
      {
         const uint16_t msg_length = *(uint16_t*)(msg_begin + UBX_MSG_LEN_OFFSET) + UBX_MSG_PAYLOAD_OFFSET + UBX_MSG_CHKSUM_LEN;
         bytes_received -= msg_length;
         msg_begin += msg_length;
      }
      previous_data_index = data_end_index;
   }
}


// Public API Functions ------------------------------------------------------------------------------------------------

void gps_init(void)
{
   // Initialize the various GPIO clocks
#if REV_ID == REV_A
   SET_BIT(RCC->AHB4ENR, RCC_AHB4ENR_GPIOAEN);
   (void)READ_BIT(RCC->AHB4ENR, RCC_AHB4ENR_GPIOAEN);
   SET_BIT(RCC->AHB4ENR, RCC_AHB4ENR_GPIODEN);
   (void)READ_BIT(RCC->AHB4ENR, RCC_AHB4ENR_GPIODEN);
#else
   SET_BIT(RCC->AHB4ENR, RCC_AHB4ENR_GPIOBEN);
   (void)READ_BIT(RCC->AHB4ENR, RCC_AHB4ENR_GPIOBEN);
   SET_BIT(RCC->AHB4ENR, RCC_AHB4ENR_GPIOEEN);
   (void)READ_BIT(RCC->AHB4ENR, RCC_AHB4ENR_GPIOEEN);
#endif

   // Initialize the DMA1 clock
   SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_DMA1EN);
   (void)READ_BIT(RCC->AHB1ENR, RCC_AHB1ENR_DMA1EN);

   // Initialize the GPS UART peripheral clock
   SET_BIT(RCC->APB1LENR, GPS_UART_CONCAT(RCC_APB1LENR_, GPS_UART_TYPE, GPS_UART_NUMBER, EN));
   (void)READ_BIT(RCC->APB1LENR, GPS_UART_CONCAT(RCC_APB1LENR_, GPS_UART_TYPE, GPS_UART_NUMBER, EN));
   MODIFY_REG(RCC->D2CCIP2R, RCC_D2CCIP2R_USART28SEL, (uint32_t)RCC_USART234578CLKSOURCE_D2PCLK1);

   // Initialize the SYSCFG clock
   SET_BIT(RCC->APB4ENR, RCC_APB4ENR_SYSCFGEN);
   (void)READ_BIT(RCC->APB4ENR, RCC_APB4ENR_SYSCFGEN);

   // Initialize the non-peripheral GPIO pins
   WRITE_REG(LED_GPS_STATUS_GPIO_Port->BSRR, (uint32_t)LED_GPS_STATUS_Pin << 16U);
   uint32_t position = 32 - __builtin_clz(LED_GPS_STATUS_Pin) - 1;
   MODIFY_REG(LED_GPS_STATUS_GPIO_Port->OSPEEDR, (GPIO_OSPEEDR_OSPEED0 << (position * 2U)), (GPIO_SPEED_FREQ_LOW << (position * 2U)));
   MODIFY_REG(LED_GPS_STATUS_GPIO_Port->OTYPER, (GPIO_OTYPER_OT0 << position), (((GPIO_MODE_OUTPUT_PP & OUTPUT_TYPE) >> OUTPUT_TYPE_Pos) << position));
   CLEAR_BIT(LED_GPS_STATUS_GPIO_Port->PUPDR, (GPIO_PUPDR_PUPD0 << (position * 2U)));
   MODIFY_REG(LED_GPS_STATUS_GPIO_Port->MODER, (GPIO_MODER_MODE0 << (position * 2U)), ((GPIO_MODE_OUTPUT_PP & GPIO_MODE) << (position * 2U)));
   position = 32 - __builtin_clz(GPS_TIMEPULSE_Pin) - 1;
   uint32_t iocurrent = GPS_TIMEPULSE_Pin & (1UL << position);
   CLEAR_BIT(GPS_TIMEPULSE_GPIO_Port->PUPDR, (GPIO_PUPDR_PUPD0 << (position * 2U)));
   MODIFY_REG(GPS_TIMEPULSE_GPIO_Port->MODER, (GPIO_MODER_MODE0 << (position * 2U)), ((GPIO_MODE_IT_RISING & GPIO_MODE) << (position * 2U)));
#if REV_ID == REV_A
   CLEAR_BIT(SYSCFG->EXTICR[position >> 2U], (0x0FUL << (4U * (position & 0x03U))));
#else
   MODIFY_REG(SYSCFG->EXTICR[position >> 2U], (0x0FUL << (4U * (position & 0x03U))), (4UL << (4U * (position & 0x03U))));
#endif
   SET_BIT(EXTI->RTSR1, iocurrent);
   CLEAR_BIT(EXTI->FTSR1, iocurrent);
   CLEAR_BIT(EXTI_D2->EMR1, iocurrent);
   SET_BIT(EXTI_D2->IMR1, iocurrent);
   NVIC_SetPriority(EXTI3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
   NVIC_EnableIRQ(EXTI3_IRQn);

   // Initialize the GPS UART GPIO pins
   position = 32 - __builtin_clz(GPS_TX_Pin) - 1;
   MODIFY_REG(GPS_TX_GPIO_Port->OSPEEDR, (GPIO_OSPEEDR_OSPEED0 << (position * 2U)), (GPIO_SPEED_FREQ_MEDIUM << (position * 2U)));
   MODIFY_REG(GPS_TX_GPIO_Port->OTYPER, (GPIO_OTYPER_OT0 << position), (((GPIO_MODE_AF_PP & OUTPUT_TYPE) >> OUTPUT_TYPE_Pos) << position));
   CLEAR_BIT(GPS_TX_GPIO_Port->PUPDR, (GPIO_PUPDR_PUPD0 << (position * 2U)));
   MODIFY_REG(GPS_TX_GPIO_Port->AFR[position >> 3U], (0xFU << ((position & 0x07U) * 4U)), (GPS_UART_AF << ((position & 0x07U) * 4U)));
   MODIFY_REG(GPS_TX_GPIO_Port->MODER, (GPIO_MODER_MODE0 << (position * 2U)), ((GPIO_MODE_AF_PP & GPIO_MODE) << (position * 2U)));
   position = 32 - __builtin_clz(GPS_RX_Pin) - 1;
   MODIFY_REG(GPS_RX_GPIO_Port->OSPEEDR, (GPIO_OSPEEDR_OSPEED0 << (position * 2U)), (GPIO_SPEED_FREQ_MEDIUM << (position * 2U)));
   MODIFY_REG(GPS_RX_GPIO_Port->OTYPER, (GPIO_OTYPER_OT0 << position), (((GPIO_MODE_AF_PP & OUTPUT_TYPE) >> OUTPUT_TYPE_Pos) << position));
   CLEAR_BIT(GPS_RX_GPIO_Port->PUPDR, (GPIO_PUPDR_PUPD0 << (position * 2U)));
   MODIFY_REG(GPS_RX_GPIO_Port->AFR[position >> 3U], (0xFU << ((position & 0x07U) * 4U)), (GPS_UART_AF << ((position & 0x07U) * 4U)));
   MODIFY_REG(GPS_RX_GPIO_Port->MODER, (GPIO_MODER_MODE0 << (position * 2U)), ((GPIO_MODE_AF_PP & GPIO_MODE) << (position * 2U)));

   // Initialize DMA1 Stream2 for the GPS UART
   CLEAR_BIT(DMA1_Stream2->CR, DMA_SxCR_EN);
   while (READ_BIT(DMA1_Stream2->CR, DMA_SxCR_EN));
   MODIFY_REG(DMA1_Stream2->CR,
              (DMA_SxCR_MBURST | DMA_SxCR_PBURST | DMA_SxCR_PL | DMA_SxCR_MSIZE | DMA_SxCR_PSIZE | DMA_SxCR_MINC | DMA_SxCR_PINC | DMA_SxCR_CIRC | DMA_SxCR_DIR | DMA_SxCR_CT | DMA_SxCR_DBM),
              (DMA_PERIPH_TO_MEMORY | DMA_MINC_ENABLE | DMA_PDATAALIGN_BYTE | DMA_MDATAALIGN_BYTE | DMA_CIRCULAR | DMA_PRIORITY_LOW | DMA_SxCR_TRBUFF));
   CLEAR_BIT(DMA1_Stream2->FCR, (DMA_SxFCR_DMDIS | DMA_SxFCR_FTH));
   WRITE_REG(DMAMUX1_Channel2->CCR, GPS_UART_CONCAT(DMA_REQUEST_, GPS_UART_TYPE, GPS_UART_NUMBER, _RX));
   WRITE_REG(DMAMUX1_ChannelStatus->CFR, (1UL << (2 & 0x1FU)));

   // Initialize the GPS UART peripheral
   CLEAR_BIT(GPS_UART->CR1, USART_CR1_UE);
   while (READ_BIT(GPS_UART->CR1, USART_CR1_UE));
   MODIFY_REG(GPS_UART->CR1, (USART_CR1_M | USART_CR1_PCE | USART_CR1_PS | USART_CR1_TE | USART_CR1_RE | USART_CR1_OVER8 | USART_CR1_FIFOEN), UART_MODE_TX_RX);
   CLEAR_BIT(GPS_UART->CR2, USART_CR2_STOP);
   CLEAR_BIT(GPS_UART->CR3, USART_CR3_RTSE | USART_CR3_CTSE | USART_CR3_ONEBIT | USART_CR3_TXFTCFG | USART_CR3_RXFTCFG);
   CLEAR_BIT(GPS_UART->PRESC, USART_PRESC_PRESCALER);
   WRITE_REG(GPS_UART->BRR, (uint16_t)UART_DIV_SAMPLING16(HAL_RCC_GetPCLK1Freq(), 38400, 0));
   CLEAR_BIT(GPS_UART->CR2, (USART_CR2_LINEN | USART_CR2_CLKEN));
   CLEAR_BIT(GPS_UART->CR3, (USART_CR3_SCEN | USART_CR3_HDSEL | USART_CR3_IREN));
   SET_BIT(GPS_UART->CR1, USART_CR1_UE);
   if (READ_BIT(GPS_UART->CR1, USART_CR1_TE))
      while (!READ_BIT(GPS_UART->ISR, USART_ISR_TEACK));
   if (READ_BIT(GPS_UART->CR1, USART_CR1_RE))
      while (!READ_BIT(GPS_UART->ISR, USART_ISR_REACK));
   NVIC_SetPriority(GPS_UART_CONCAT(, GPS_UART_TYPE, GPS_UART_NUMBER, _IRQn), NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
   NVIC_EnableIRQ(GPS_UART_CONCAT(, GPS_UART_TYPE, GPS_UART_NUMBER, _IRQn));

   // Configure the GPS UART DMA buffer addresses and sizes
   WRITE_REG(DMA1_Stream2->PAR, (uint32_t)&GPS_UART->RDR);
   WRITE_REG(DMA1_Stream2->M0AR, (uint32_t)gps_rx_buffer);
   WRITE_REG(DMA1_Stream2->NDTR, sizeof(gps_rx_buffer));
   CLEAR_BIT(DMA1_Stream2->CR, (DMA_IT_TC | DMA_IT_TE | DMA_IT_DME | DMA_IT_HT));

   // Ensure that all configuration parameters are correctly set
   uint8_t config_changed = 1;
   while (config_changed)
   {
     gps_wait_until_ready();
     config_changed = gps_verify_or_set_interface_config();
     config_changed = gps_verify_or_set_gnss_config() || config_changed;
     config_changed = gps_verify_or_set_configuration() || config_changed;
     config_changed = gps_verify_or_set_lna_gain() || config_changed;
#if REV_ID > REV_B
     if (config_changed)
       gps_reset();
#endif
   }

   // Start listening for packets using DMA until an idle line is detected
   SET_BIT(DMA1_Stream2->CR, DMA_SxCR_EN);
   ATOMIC_SET_BIT(GPS_UART->CR3, USART_CR3_DMAR);
   WRITE_REG(GPS_UART->ICR, UART_CLEAR_IDLEF);
   ATOMIC_SET_BIT(GPS_UART->CR1, USART_CR1_IDLEIE);
}

uint8_t gps_get_timepulse_fired(void)
{
   // Return and reset the most recent GPS timepulse flag
   const uint8_t fired = gps_timepulse_fired;
   gps_timepulse_fired = 0;
   return fired;
}

void gps_update_packet_timestamp(uint8_t interpolate)
{
   // Either interpolate the timestamp of the GPS packet or update it from the time-mark packet
   if (interpolate)
   {
      const double interpolated_timestamp = data.packets[0].timestamp + (1.0 / AUDIO_NUM_DMAS_PER_CLIP);
      data.packets[0].timestamp = data.packets[1].timestamp = interpolated_timestamp;
   }
   else
   {
      data.packets[0].timestamp = data.packets[1].timestamp = 1.0 + next_timestamp;
      next_timestamp = 0.0;
   }
}

void gps_update_packet_llh(void)
{
   // Update the most recently received GPS position
   data.packets[0].lat = data.packets[1].lat = lat_degrees;
   data.packets[0].lon = data.packets[1].lon = lon_degrees;
   data.packets[0].ht = data.packets[1].ht = height_meters;
}

#endif  // #ifdef CORE_CM4
