#ifdef CORE_CM7

// Header Inclusions ---------------------------------------------------------------------------------------------------

#include "audio.h"
#include "onset_detection.h"


// PDM-to-TDM Converter Chip Register Definitions ----------------------------------------------------------------------

#define PAGE_CFG_REG                0x00
#define SW_RESET_REG                0x01
#define SLEEP_CFG_REG               0x02
#define ASI_CFG0_REG                0x07
#define ASI_CFG1_REG                0x08
#define ASI_CFG2_REG                0x09
#define ASI_CH1_REG                 0x0B
#define ASI_CH2_REG                 0x0C
#define ASI_CH3_REG                 0x0D
#define ASI_CH4_REG                 0x0E
#define MST_CFG0_REG                0x13
#define MST_CFG1_REG                0x14
#define ASI_STS_REG                 0x15
#define CLK_SRC_REG                 0x16
#define PDMCLK_CFG_REG              0x1F
#define PDMIN_CFG_REG               0x20
#define GPIO_CFG0_REG               0x21
#define GPO_CFG0_REG                0x22
#if REV_ID == REV_A
  #define GPO_CFG1_REG              0x23
  #define GPO_CFG2_REG              0x24
  #define GPO_CFG3_REG              0x25
#endif
#define GPO_VAL_REG                 0x29
#define GPIO_MON_REG                0x2A
#define GPI_CFG0_REG                0x2B
#if REV_ID == REV_A
  #define GPI_CFG1_REG              0x2C
#endif
#define GPI_MON_REG                 0x2F
#define INT_CFG_REG                 0x32
#define INT_MASK0_REG               0x33
#define INT_LTCH0_REG               0x36
#define BIAS_CFG_REG                0x3B
#define CH1_CFG0_REG                0x3C
#define CH1_CFG2_REG                0x3E
#define CH1_CFG3_REG                0x3F
#define CH1_CFG4_REG                0x40
#define CH2_CFG0_REG                0x41
#define CH2_CFG2_REG                0x43
#define CH2_CFG3_REG                0x44
#define CH2_CFG4_REG                0x45
#define CH3_CFG0_REG                0x46
#define CH3_CFG2_REG                0x48
#define CH3_CFG3_REG                0x49
#define CH3_CFG4_REG                0x4A
#define CH4_CFG0_REG                0x4B
#define CH4_CFG2_REG                0x4D
#define CH4_CFG3_REG                0x4E
#define CH4_CFG4_REG                0x4F
#define DSP_CFG0_REG                0x6B
#define DSP_CFG1_REG                0x6C
#define IN_CH_EN_REG                0x73
#define ASI_OUT_CH_EN_REG           0x74
#define PWR_CFG_REG                 0x75
#define DEV_STS0_REG                0x76
#define DEV_STS1_REG                0x77
#define I2C_CKSUM_REG               0x7E

// I2C Definitions
#define I2C_MICS_DEVICE_ADDRESS     156


// Audio Definitions and Static Variables ------------------------------------------------------------------------------

__attribute__((aligned (4), section (".dtcm")))
static int16_t audio_data[2][AUDIO_NUM_CHANNELS][AUDIO_BUFFER_SAMPLES / AUDIO_NUM_CHANNELS];

__attribute__((aligned (4), section (".ramd1")))
static MDMA_LinkNodeTypeDef mdma_audio_xfers[2];

__attribute__((aligned (4), section (".ramd1")))
static MDMA_LinkNodeTypeDef mdma_gps_xfers[AUDIO_NUM_DMAS_PER_CLIP];

__attribute__((aligned (4), section (".ramd1")))
static uint32_t gps_trigger_states[AUDIO_NUM_DMAS_PER_CLIP];

__attribute__((aligned (32), section (".ramd1")))
static int16_t audio_raw[2][AUDIO_BUFFER_SAMPLES];


// Interrupt Service Routines ------------------------------------------------------------------------------------------

__attribute__((section(".itcm")))
void MDMA_IRQHandler(void)
{
   // Ensure that the interrupt is relevant to this CPU
   if (READ_BIT(MDMA_Channel0->CISR, (MDMA_FLAG_TE | MDMA_FLAG_BRT)))
   {
      // Clear the interrupt
      WRITE_REG(MDMA_Channel0->CIFCR, (MDMA_FLAG_TE | MDMA_FLAG_BRT));

      // Store metadata to indicate when full clips have been received
      data.audio_read_index = (READ_BIT(DMA1_Stream0->CR, DMA_SxCR_CT) == 0U);
      data.audio_clip_complete = READ_BIT(GPS_TIME_TRIGGER_GPIO_Port->IDR, GPS_TIME_TRIGGER_Pin);

      // Initiate transfer of audio channels to the other core
      WRITE_REG(MDMA_Channel2->CBNDTR, sizeof(data.audio[0]) & MDMA_CBNDTR_BNDT);
      WRITE_REG(MDMA_Channel2->CSAR, (uint32_t)audio_data[data.audio_read_index][0]);
      WRITE_REG(MDMA_Channel2->CDAR, (uint32_t)data.audio[data.audio_read_index]);
      SET_BIT(MDMA_Channel2->CCR, (MDMA_IT_TE | MDMA_IT_BT | MDMA_CCR_EN | MDMA_CCR_SWRQ));

      // Process the newly received audio
      onset_detection_invoke(audio_data[data.audio_read_index]);
   }
}


// Private Helper Functions --------------------------------------------------------------------------------------------

static void audio_write_reg(uint8_t reg, uint8_t data)
{
#if REV_ID == REV_A

   // Write the single data byte to the indicated register using SPI
   const uint8_t xfer_buffer[2] = { (reg << 1) & 0xFE, data };
   volatile uint16_t *ptxdr_16bits = (volatile uint16_t*)&(SPI2->TXDR);
   MODIFY_REG(SPI2->CR2, SPI_CR2_TSIZE, 2);
   SET_BIT(SPI2->CR1, SPI_CR1_SPE);
   SET_BIT(SPI2->CR1, SPI_CR1_CSTART);
   while (!READ_BIT(SPI2->SR, SPI_FLAG_TXP));
   *ptxdr_16bits = *((const uint16_t*)xfer_buffer);
   while (!READ_BIT(SPI2->SR, SPI_FLAG_EOT));
   CLEAR_BIT(SPI2->CR1, SPI_CR1_SPE);
   SET_BIT(SPI2->IFCR, SPI_IFCR_EOTC | SPI_IFCR_TXTFC | SPI_IFCR_UDRC | SPI_IFCR_OVRC | SPI_IFCR_MODFC | SPI_IFCR_TIFREC);

#else

   // Write register number to the TXDR register and initiate the transfer
   WRITE_REG(I2C2->TXDR, reg);
   MODIFY_REG(I2C2->CR2,
              (I2C_CR2_NBYTES | I2C_CR2_RELOAD | I2C_CR2_AUTOEND | I2C_CR2_RD_WRN | I2C_CR2_START | I2C_CR2_STOP),
              (((2 << I2C_CR2_NBYTES_Pos) & I2C_CR2_NBYTES) | I2C_AUTOEND_MODE | I2C_GENERATE_START_WRITE));

   // Wait until ready to transmit the data byte
   uint8_t success = 1;
   while (success && !READ_BIT(I2C2->ISR, I2C_FLAG_TXIS))
      if (READ_BIT(I2C2->ISR, I2C_FLAG_AF | I2C_FLAG_ARLO | I2C_FLAG_BERR))
         success = 0;
   if (success)
      WRITE_REG(I2C2->TXDR, data);

   // Wait until transmission completes and clear flags
   while (success && !READ_BIT(I2C2->ISR, I2C_FLAG_STOPF))
      if (READ_BIT(I2C2->ISR, I2C_FLAG_AF | I2C_FLAG_ARLO | I2C_FLAG_BERR))
         success = 0;
   WRITE_REG(I2C2->ICR, (I2C_FLAG_STOPF | I2C_FLAG_AF | I2C_FLAG_ARLO | I2C_FLAG_BERR | I2C_FLAG_OVR | I2C_FLAG_PECERR | I2C_FLAG_TCR | I2C_FLAG_TC));
   CLEAR_BIT(I2C2->CR2, (I2C_CR2_HEAD10R | I2C_CR2_NBYTES | I2C_CR2_RELOAD | I2C_CR2_AUTOEND | I2C_CR2_RD_WRN));

#endif
}


// Public API Functions ------------------------------------------------------------------------------------------------

void audio_init(void)
{
   // Initialize the GPS time-request trigger states
   memset(&mdma_gps_xfers, 0, sizeof(mdma_gps_xfers));
   memset(&mdma_audio_xfers, 0, sizeof(mdma_audio_xfers));
   gps_trigger_states[AUDIO_NUM_DMAS_PER_CLIP - 1] = GPS_TIME_TRIGGER_Pin;
   for (uint32_t i = 0; i < AUDIO_NUM_DMAS_PER_CLIP - 1; ++i)
      gps_trigger_states[i] = (uint32_t)GPS_TIME_TRIGGER_Pin << 16U;

   // Initialize the various GPIO clocks
#if REV_ID == REV_A
   SET_BIT(RCC->AHB4ENR, RCC_AHB4ENR_GPIOAEN);
   (void)READ_BIT(RCC->AHB4ENR, RCC_AHB4ENR_GPIOAEN);
   SET_BIT(RCC->AHB4ENR, RCC_AHB4ENR_GPIODEN);
   (void)READ_BIT(RCC->AHB4ENR, RCC_AHB4ENR_GPIODEN);
#endif
   SET_BIT(RCC->AHB4ENR, RCC_AHB4ENR_GPIOBEN);
   (void)READ_BIT(RCC->AHB4ENR, RCC_AHB4ENR_GPIOBEN);
   SET_BIT(RCC->AHB4ENR, RCC_AHB4ENR_GPIOEEN);
   (void)READ_BIT(RCC->AHB4ENR, RCC_AHB4ENR_GPIOEEN);

   // Initialize the DMA1 clock
   SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_DMA1EN);
   (void)READ_BIT(RCC->AHB1ENR, RCC_AHB1ENR_DMA1EN);

   // Initialize the MDMA controller clock
   SET_BIT(RCC->AHB3ENR, RCC_AHB3ENR_MDMAEN);
   (void)READ_BIT(RCC->AHB3ENR, RCC_AHB3ENR_MDMAEN);

   // Initialize the SPI2 or I2C2 peripheral clock
#if REV_ID == REV_A
   SET_BIT(RCC->APB1LENR, RCC_APB1LENR_SPI2EN);
   (void)READ_BIT(RCC->APB1LENR, RCC_APB1LENR_SPI2EN);
#else
   MODIFY_REG(RCC->D2CCIP2R, RCC_D2CCIP2R_I2C123SEL, RCC_I2C123CLKSOURCE_D2PCLK1);
   SET_BIT(RCC->APB1LENR, RCC_APB1LENR_I2C2EN);
   (void)READ_BIT(RCC->APB1LENR, RCC_APB1LENR_I2C2EN);
#endif

   // Initialize the SAI peripheral clock
   CLEAR_BIT(RCC->CR, RCC_CR_PLL2ON);
   while (__HAL_RCC_GET_FLAG(RCC_FLAG_PLL2RDY) != 0U);
   __HAL_RCC_PLL2_CONFIG(21, 289, 14, 14, 14);
   __HAL_RCC_PLL2_VCIRANGE(RCC_PLL2VCIRANGE_0) ;
   __HAL_RCC_PLL2_VCORANGE(RCC_PLL2VCOMEDIUM) ;
   CLEAR_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLL2FRACEN);
   __HAL_RCC_PLL2FRACN_CONFIG(113);
   SET_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLL2FRACEN);
   __HAL_RCC_PLL2CLKOUT_ENABLE(RCC_PLL2_DIVP);
   SET_BIT(RCC->CR, RCC_CR_PLL2ON);
   while (__HAL_RCC_GET_FLAG(RCC_FLAG_PLL2RDY) == 0U);
   MODIFY_REG(RCC->D2CCIP1R, RCC_D2CCIP1R_SAI23SEL, RCC_SAI23CLKSOURCE_PLL2);
   SET_BIT(RCC->APB2ENR, RCC_APB2ENR_SAI2EN);
   (void)READ_BIT(RCC->APB2ENR, RCC_APB2ENR_SAI2EN);

   // Initialize the non-peripheral GPIO pins
   WRITE_REG(GPS_TIME_TRIGGER_GPIO_Port->BSRR, (uint32_t)GPS_TIME_TRIGGER_Pin << 16U);
   uint32_t position = 32 - __builtin_clz(GPS_TIME_TRIGGER_Pin) - 1;
   MODIFY_REG(GPS_TIME_TRIGGER_GPIO_Port->OSPEEDR, (GPIO_OSPEEDR_OSPEED0 << (position * 2U)), (GPIO_SPEED_FREQ_VERY_HIGH << (position * 2U)));
   MODIFY_REG(GPS_TIME_TRIGGER_GPIO_Port->OTYPER, (GPIO_OTYPER_OT0 << position), (((GPIO_MODE_OUTPUT_PP & OUTPUT_TYPE) >> OUTPUT_TYPE_Pos) << position));
   CLEAR_BIT(GPS_TIME_TRIGGER_GPIO_Port->PUPDR, (GPIO_PUPDR_PUPD0 << (position * 2U)));
   MODIFY_REG(GPS_TIME_TRIGGER_GPIO_Port->MODER, (GPIO_MODER_MODE0 << (position * 2U)), ((GPIO_MODE_OUTPUT_PP & GPIO_MODE) << (position * 2U)));
#if REV_ID == REV_A
   MICS_NRESET_GPIO_Port->BSRR = (uint32_t)MICS_NRESET_Pin << 16U;
   position = 32 - __builtin_clz(MICS_NRESET_Pin) - 1;
   MODIFY_REG(MICS_NRESET_GPIO_Port->OSPEEDR, (GPIO_OSPEEDR_OSPEED0 << (position * 2U)), (GPIO_SPEED_FREQ_LOW << (position * 2U)));
   MODIFY_REG(MICS_NRESET_GPIO_Port->OTYPER, (GPIO_OTYPER_OT0 << position), (((GPIO_MODE_OUTPUT_PP & OUTPUT_TYPE) >> OUTPUT_TYPE_Pos) << position));
   CLEAR_BIT(MICS_NRESET_GPIO_Port->PUPDR, (GPIO_PUPDR_PUPD0 << (position * 2U)));
   MODIFY_REG(MICS_NRESET_GPIO_Port->MODER, (GPIO_MODER_MODE0 << (position * 2U)), ((GPIO_MODE_OUTPUT_PP & GPIO_MODE) << (position * 2U)));
#endif

   // Initialize the SPI2 or I2C2 GPIO pins
#if REV_ID == REV_A
   position = 32 - __builtin_clz(MICS_CS_Pin) - 1;
   MODIFY_REG(MICS_CS_GPIO_Port->OSPEEDR, (GPIO_OSPEEDR_OSPEED0 << (position * 2U)), (GPIO_SPEED_FREQ_MEDIUM << (position * 2U)));
   MODIFY_REG(MICS_CS_GPIO_Port->OTYPER, (GPIO_OTYPER_OT0 << position), (((GPIO_MODE_AF_PP & OUTPUT_TYPE) >> OUTPUT_TYPE_Pos) << position));
   CLEAR_BIT(MICS_CS_GPIO_Port->PUPDR, (GPIO_PUPDR_PUPD0 << (position * 2U)));
   MODIFY_REG(MICS_CS_GPIO_Port->AFR[position >> 3U], (0xFU << ((position & 0x07U) * 4U)), (GPIO_AF5_SPI2 << ((position & 0x07U) * 4U)));
   MODIFY_REG(MICS_CS_GPIO_Port->MODER, (GPIO_MODER_MODE0 << (position * 2U)), ((GPIO_MODE_AF_PP & GPIO_MODE) << (position * 2U)));
   position = 32 - __builtin_clz(MICS_SCK_Pin) - 1;
   MODIFY_REG(MICS_SCK_GPIO_Port->OSPEEDR, (GPIO_OSPEEDR_OSPEED0 << (position * 2U)), (GPIO_SPEED_FREQ_MEDIUM << (position * 2U)));
   MODIFY_REG(MICS_SCK_GPIO_Port->OTYPER, (GPIO_OTYPER_OT0 << position), (((GPIO_MODE_AF_PP & OUTPUT_TYPE) >> OUTPUT_TYPE_Pos) << position));
   CLEAR_BIT(MICS_SCK_GPIO_Port->PUPDR, (GPIO_PUPDR_PUPD0 << (position * 2U)));
   MODIFY_REG(MICS_SCK_GPIO_Port->AFR[position >> 3U], (0xFU << ((position & 0x07U) * 4U)), (GPIO_AF5_SPI2 << ((position & 0x07U) * 4U)));
   MODIFY_REG(MICS_SCK_GPIO_Port->MODER, (GPIO_MODER_MODE0 << (position * 2U)), ((GPIO_MODE_AF_PP & GPIO_MODE) << (position * 2U)));
   position = 32 - __builtin_clz(MICS_MISO_Pin) - 1;
   MODIFY_REG(MICS_MISO_GPIO_Port->OSPEEDR, (GPIO_OSPEEDR_OSPEED0 << (position * 2U)), (GPIO_SPEED_FREQ_MEDIUM << (position * 2U)));
   MODIFY_REG(MICS_MISO_GPIO_Port->OTYPER, (GPIO_OTYPER_OT0 << position), (((GPIO_MODE_AF_PP & OUTPUT_TYPE) >> OUTPUT_TYPE_Pos) << position));
   CLEAR_BIT(MICS_MISO_GPIO_Port->PUPDR, (GPIO_PUPDR_PUPD0 << (position * 2U)));
   MODIFY_REG(MICS_MISO_GPIO_Port->AFR[position >> 3U], (0xFU << ((position & 0x07U) * 4U)), (GPIO_AF5_SPI2 << ((position & 0x07U) * 4U)));
   MODIFY_REG(MICS_MISO_GPIO_Port->MODER, (GPIO_MODER_MODE0 << (position * 2U)), ((GPIO_MODE_AF_PP & GPIO_MODE) << (position * 2U)));
   position = 32 - __builtin_clz(MICS_MOSI_Pin) - 1;
   MODIFY_REG(MICS_MOSI_GPIO_Port->OSPEEDR, (GPIO_OSPEEDR_OSPEED0 << (position * 2U)), (GPIO_SPEED_FREQ_MEDIUM << (position * 2U)));
   MODIFY_REG(MICS_MOSI_GPIO_Port->OTYPER, (GPIO_OTYPER_OT0 << position), (((GPIO_MODE_AF_PP & OUTPUT_TYPE) >> OUTPUT_TYPE_Pos) << position));
   CLEAR_BIT(MICS_MOSI_GPIO_Port->PUPDR, (GPIO_PUPDR_PUPD0 << (position * 2U)));
   MODIFY_REG(MICS_MOSI_GPIO_Port->AFR[position >> 3U], (0xFU << ((position & 0x07U) * 4U)), (GPIO_AF5_SPI2 << ((position & 0x07U) * 4U)));
   MODIFY_REG(MICS_MOSI_GPIO_Port->MODER, (GPIO_MODER_MODE0 << (position * 2U)), ((GPIO_MODE_AF_PP & GPIO_MODE) << (position * 2U)));
#else
   position = 32 - __builtin_clz(MICS_SCL_Pin) - 1;
   MODIFY_REG(MICS_SCL_GPIO_Port->OSPEEDR, (GPIO_OSPEEDR_OSPEED0 << (position * 2U)), (GPIO_SPEED_FREQ_MEDIUM << (position * 2U)));
   MODIFY_REG(MICS_SCL_GPIO_Port->OTYPER, (GPIO_OTYPER_OT0 << position), (((GPIO_MODE_AF_OD & OUTPUT_TYPE) >> OUTPUT_TYPE_Pos) << position));
   CLEAR_BIT(MICS_SCL_GPIO_Port->PUPDR, (GPIO_PUPDR_PUPD0 << (position * 2U)));
   MODIFY_REG(MICS_SCL_GPIO_Port->AFR[position >> 3U], (0xFU << ((position & 0x07U) * 4U)), (GPIO_AF4_I2C2 << ((position & 0x07U) * 4U)));
   MODIFY_REG(MICS_SCL_GPIO_Port->MODER, (GPIO_MODER_MODE0 << (position * 2U)), ((GPIO_MODE_AF_OD & GPIO_MODE) << (position * 2U)));
   position = 32 - __builtin_clz(MICS_SDA_Pin) - 1;
   MODIFY_REG(MICS_SDA_GPIO_Port->OSPEEDR, (GPIO_OSPEEDR_OSPEED0 << (position * 2U)), (GPIO_SPEED_FREQ_MEDIUM << (position * 2U)));
   MODIFY_REG(MICS_SDA_GPIO_Port->OTYPER, (GPIO_OTYPER_OT0 << position), (((GPIO_MODE_AF_OD & OUTPUT_TYPE) >> OUTPUT_TYPE_Pos) << position));
   CLEAR_BIT(MICS_SDA_GPIO_Port->PUPDR, (GPIO_PUPDR_PUPD0 << (position * 2U)));
   MODIFY_REG(MICS_SDA_GPIO_Port->AFR[position >> 3U], (0xFU << ((position & 0x07U) * 4U)), (GPIO_AF4_I2C2 << ((position & 0x07U) * 4U)));
   MODIFY_REG(MICS_SDA_GPIO_Port->MODER, (GPIO_MODER_MODE0 << (position * 2U)), ((GPIO_MODE_AF_OD & GPIO_MODE) << (position * 2U)));
#endif

   // Initialize the SAI GPIO pins
   position = 32 - __builtin_clz(AUDIO_SD_Pin) - 1;
   MODIFY_REG(AUDIO_SD_GPIO_Port->OSPEEDR, (GPIO_OSPEEDR_OSPEED0 << (position * 2U)), (GPIO_SPEED_FREQ_VERY_HIGH << (position * 2U)));
   MODIFY_REG(AUDIO_SD_GPIO_Port->OTYPER, (GPIO_OTYPER_OT0 << position), (((GPIO_MODE_AF_PP & OUTPUT_TYPE) >> OUTPUT_TYPE_Pos) << position));
   CLEAR_BIT(AUDIO_SD_GPIO_Port->PUPDR, (GPIO_PUPDR_PUPD0 << (position * 2U)));
   MODIFY_REG(AUDIO_SD_GPIO_Port->AFR[position >> 3U], (0xFU << ((position & 0x07U) * 4U)), (GPIO_AF10_SAI2 << ((position & 0x07U) * 4U)));
   MODIFY_REG(AUDIO_SD_GPIO_Port->MODER, (GPIO_MODER_MODE0 << (position * 2U)), ((GPIO_MODE_AF_PP & GPIO_MODE) << (position * 2U)));
   position = 32 - __builtin_clz(AUDIO_CLK_Pin) - 1;
   MODIFY_REG(AUDIO_CLK_GPIO_Port->OSPEEDR, (GPIO_OSPEEDR_OSPEED0 << (position * 2U)), (GPIO_SPEED_FREQ_VERY_HIGH << (position * 2U)));
   MODIFY_REG(AUDIO_CLK_GPIO_Port->OTYPER, (GPIO_OTYPER_OT0 << position), (((GPIO_MODE_AF_PP & OUTPUT_TYPE) >> OUTPUT_TYPE_Pos) << position));
   CLEAR_BIT(AUDIO_CLK_GPIO_Port->PUPDR, (GPIO_PUPDR_PUPD0 << (position * 2U)));
   MODIFY_REG(AUDIO_CLK_GPIO_Port->AFR[position >> 3U], (0xFU << ((position & 0x07U) * 4U)), (GPIO_AF10_SAI2 << ((position & 0x07U) * 4U)));
   MODIFY_REG(AUDIO_CLK_GPIO_Port->MODER, (GPIO_MODER_MODE0 << (position * 2U)), ((GPIO_MODE_AF_PP & GPIO_MODE) << (position * 2U)));
   position = 32 - __builtin_clz(AUDIO_FSYNC_Pin) - 1;
   MODIFY_REG(AUDIO_FSYNC_GPIO_Port->OSPEEDR, (GPIO_OSPEEDR_OSPEED0 << (position * 2U)), (GPIO_SPEED_FREQ_VERY_HIGH << (position * 2U)));
   MODIFY_REG(AUDIO_FSYNC_GPIO_Port->OTYPER, (GPIO_OTYPER_OT0 << position), (((GPIO_MODE_AF_PP & OUTPUT_TYPE) >> OUTPUT_TYPE_Pos) << position));
   CLEAR_BIT(AUDIO_FSYNC_GPIO_Port->PUPDR, (GPIO_PUPDR_PUPD0 << (position * 2U)));
   MODIFY_REG(AUDIO_FSYNC_GPIO_Port->AFR[position >> 3U], (0xFU << ((position & 0x07U) * 4U)), (GPIO_AF10_SAI2 << ((position & 0x07U) * 4U)));
   MODIFY_REG(AUDIO_FSYNC_GPIO_Port->MODER, (GPIO_MODER_MODE0 << (position * 2U)), ((GPIO_MODE_AF_PP & GPIO_MODE) << (position * 2U)));

   // Initialize the SPI2 or I2C2 peripheral
#if REV_ID == REV_A
   CLEAR_BIT(SPI2->CR1, SPI_CR1_SPE);
   while (READ_BIT(SPI2->CR1, SPI_CR1_SPE));
   uint32_t crc_length = SPI2->CFG1 & SPI_CFG1_CRCSIZE;
   MODIFY_REG(SPI2->CR1, SPI_CR1_MASRX, SPI_MASTER_RX_AUTOSUSP_DISABLE);
   WRITE_REG(SPI2->CFG1, (SPI_BAUDRATEPRESCALER_16 | crc_length | SPI_FIFO_THRESHOLD_02DATA | SPI_DATASIZE_8BIT));
   WRITE_REG(SPI2->CFG2, (SPI_CFG2_AFCNTR | SPI_NSS_PULSE_ENABLE | SPI_NSS_HARD_OUTPUT | SPI_POLARITY_LOW | SPI_PHASE_2EDGE | SPI_MODE_MASTER | SPI_MASTER_KEEP_IO_STATE_ENABLE | SPI_DIRECTION_2LINES_TXONLY));
   CLEAR_BIT(SPI2->I2SCFGR, SPI_I2SCFGR_I2SMOD);
#else
   WRITE_REG(I2C2->CR1, 0);
   WRITE_REG(I2C2->TIMINGR, 0x10911A50);
   CLEAR_BIT(I2C2->OAR1, I2C_OAR1_OA1EN);
   WRITE_REG(I2C2->OAR1, (I2C_OAR1_OA1EN | I2C_MICS_DEVICE_ADDRESS));
   WRITE_REG(I2C2->OAR2, 0);
   MODIFY_REG(I2C2->CR2, (I2C_CR2_ADD10 | I2C_CR2_SADD), (uint32_t)I2C_MICS_DEVICE_ADDRESS);
   SET_BIT(I2C2->CR1, I2C_CR1_PE);
#endif

   // Set up the SAI peripheral DMA
   CLEAR_BIT(DMA1_Stream0->CR, DMA_SxCR_EN);
   while (READ_BIT(DMA1_Stream0->CR, DMA_SxCR_EN));
   MODIFY_REG(DMA1_Stream0->CR,
              (DMA_SxCR_MBURST | DMA_SxCR_PBURST | DMA_SxCR_PL | DMA_SxCR_MSIZE | DMA_SxCR_PSIZE | DMA_SxCR_MINC | DMA_SxCR_PINC | DMA_SxCR_CIRC | DMA_SxCR_DIR | DMA_SxCR_CT | DMA_SxCR_DBM),
              (DMA_PERIPH_TO_MEMORY | DMA_MINC_ENABLE | DMA_PDATAALIGN_HALFWORD | DMA_MDATAALIGN_HALFWORD | DMA_CIRCULAR | DMA_DOUBLE_BUFFER_M0 | DMA_PRIORITY_VERY_HIGH | DMA_MBURST_INC8 | DMA_PBURST_INC8));
   MODIFY_REG(DMA1_Stream0->FCR, (DMA_SxFCR_DMDIS | DMA_SxFCR_FTH), DMA_FIFOMODE_ENABLE | DMA_FIFO_THRESHOLD_FULL);
   WRITE_REG(DMAMUX1_Channel0->CCR, DMA_REQUEST_SAI2_B);
   WRITE_REG(DMAMUX1_ChannelStatus->CFR, 1UL);

   // Set up the SAI peripheral
   CLEAR_BIT(SAI2_Block_B->CR1, SAI_xCR1_SAIEN);
   while (READ_BIT(SAI2_Block_B->CR1, SAI_xCR1_SAIEN));
   const uint32_t freq = 10U * HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SAI2);
   const uint32_t mckdiv = freq / (AUDIO_SAMPLE_RATE_HZ * 256U) / 10U;
   WRITE_REG(SAI2->GCR, 0);
   MODIFY_REG(SAI2_Block_B->CR1,
              (SAI_xCR1_MODE | SAI_xCR1_PRTCFG |  SAI_xCR1_DS | SAI_xCR1_LSBFIRST | SAI_xCR1_CKSTR | SAI_xCR1_SYNCEN | SAI_xCR1_MONO | SAI_xCR1_OUTDRIV | SAI_xCR1_DMAEN | SAI_xCR1_NODIV | SAI_xCR1_MCKDIV | SAI_xCR1_OSR | ((HAL_GetREVID() >= REV_ID_B) ? SAI_xCR1_MCKEN : 0)),
              (SAI_MODEMASTER_RX | SAI_DATASIZE_16 | (mckdiv << 20)));
   MODIFY_REG(SAI2_Block_B->CR2, (SAI_xCR2_FTH | SAI_xCR2_FFLUSH | SAI_xCR2_COMP | SAI_xCR2_CPL), SAI_FIFOTHRESHOLD_FULL);
   MODIFY_REG(SAI2_Block_B->FRCR, (SAI_xFRCR_FRL | SAI_xFRCR_FSALL | SAI_xFRCR_FSDEF | SAI_xFRCR_FSPOL | SAI_xFRCR_FSOFF), ((64 - 1U) | SAI_FS_BEFOREFIRSTBIT | SAI_FS_ACTIVE_HIGH));
   MODIFY_REG(SAI2_Block_B->SLOTR, (SAI_xSLOTR_FBOFF | SAI_xSLOTR_SLOTSZ | SAI_xSLOTR_NBSLOT | SAI_xSLOTR_SLOTEN), (0xFFFF0000 | ((4 - 1U) << 8)));

   // Set up the MDMA Channel 0 (audio reordering) peripheral
   CLEAR_BIT(MDMA_Channel0->CCR, MDMA_CCR_EN);
   while (READ_BIT(MDMA_Channel0->CCR, MDMA_CCR_EN));
   WRITE_REG(MDMA_Channel0->CCR, (MDMA_PRIORITY_HIGH | MDMA_LITTLE_ENDIANNESS_PRESERVE));
   WRITE_REG(MDMA_Channel0->CTCR, (MDMA_SRC_INC_DOUBLEWORD | MDMA_DEST_INC_HALFWORD | MDMA_SRC_DATASIZE_HALFWORD | MDMA_DEST_DATASIZE_HALFWORD | MDMA_DATAALIGN_PACKENABLE | MDMA_SOURCE_BURST_64BEATS | MDMA_DEST_BURST_64BEATS | ((128 - 1U) << MDMA_CTCR_TLEN_Pos) | MDMA_REPEAT_BLOCK_TRANSFER));
   WRITE_REG(MDMA_Channel0->CBNDTR, (MDMA_CBNDTR_BRSUM | ((sizeof(int16_t) * AUDIO_BUFFER_SAMPLES / AUDIO_NUM_CHANNELS) & MDMA_CBNDTR_BNDT) | (((AUDIO_NUM_CHANNELS - 1U) << MDMA_CBNDTR_BRC_Pos) & MDMA_CBNDTR_BRC)));
   WRITE_REG(MDMA_Channel0->CBRUR, (sizeof(int16_t) * (AUDIO_BUFFER_SAMPLES - 1)));
   WRITE_REG(MDMA_Channel0->CSAR, (uint32_t)audio_raw[0]);
   WRITE_REG(MDMA_Channel0->CDAR, (uint32_t)audio_data[0]);
   WRITE_REG(MDMA_Channel0->CTBR, MDMA_CTBR_DBUS);
   WRITE_REG(MDMA_Channel0->CLAR, (uint32_t)&mdma_audio_xfers[1]);
   HAL_NVIC_SetPriority(MDMA_IRQn, 0, 0);
   HAL_NVIC_EnableIRQ(MDMA_IRQn);

   // Set up linked list mode for MDMA Channel 0 to enable continuous triggering
   WRITE_REG(mdma_audio_xfers[0].CTCR, (MDMA_SRC_INC_DOUBLEWORD | MDMA_DEST_INC_HALFWORD | MDMA_SRC_DATASIZE_HALFWORD | MDMA_DEST_DATASIZE_HALFWORD | MDMA_DATAALIGN_PACKENABLE | MDMA_SOURCE_BURST_64BEATS | MDMA_DEST_BURST_64BEATS | ((128 - 1U) << MDMA_CTCR_TLEN_Pos) | MDMA_REPEAT_BLOCK_TRANSFER));
   WRITE_REG(mdma_audio_xfers[0].CBNDTR, (MDMA_CBNDTR_BRSUM | ((sizeof(int16_t) * AUDIO_BUFFER_SAMPLES / AUDIO_NUM_CHANNELS) & MDMA_CBNDTR_BNDT) | (((AUDIO_NUM_CHANNELS - 1U) << MDMA_CBNDTR_BRC_Pos) & MDMA_CBNDTR_BRC)));
   WRITE_REG(mdma_audio_xfers[0].CBRUR, (sizeof(int16_t) * (AUDIO_BUFFER_SAMPLES - 1)));
   WRITE_REG(mdma_audio_xfers[0].CSAR, (uint32_t)audio_raw[0]);
   WRITE_REG(mdma_audio_xfers[0].CDAR, (uint32_t)audio_data[0]);
   WRITE_REG(mdma_audio_xfers[0].CTBR, MDMA_CTBR_DBUS);
   WRITE_REG(mdma_audio_xfers[0].CLAR, (uint32_t)&mdma_audio_xfers[1]);
   mdma_audio_xfers[1] = mdma_audio_xfers[0];
   WRITE_REG(mdma_audio_xfers[1].CSAR, (uint32_t)audio_raw[1]);
   WRITE_REG(mdma_audio_xfers[1].CDAR, (uint32_t)audio_data[1]);
   WRITE_REG(mdma_audio_xfers[1].CLAR, (uint32_t)&mdma_audio_xfers[0]);

   // Set up the MDMA Channel 1 (GPS triggering) peripheral
   CLEAR_BIT(MDMA_Channel1->CCR, MDMA_CCR_EN);
   while (READ_BIT(MDMA_Channel1->CCR, MDMA_CCR_EN));
   WRITE_REG(MDMA_Channel1->CCR, (MDMA_PRIORITY_VERY_HIGH | MDMA_LITTLE_ENDIANNESS_PRESERVE));
   WRITE_REG(MDMA_Channel1->CTCR, (MDMA_SRC_INC_DISABLE | MDMA_DEST_INC_DISABLE | MDMA_SRC_DATASIZE_WORD | MDMA_DEST_DATASIZE_WORD | MDMA_DATAALIGN_PACKENABLE | MDMA_SOURCE_BURST_SINGLE | MDMA_DEST_BURST_SINGLE | ((sizeof(uint32_t) - 1U) << MDMA_CTCR_TLEN_Pos) | MDMA_BUFFER_TRANSFER));
   WRITE_REG(MDMA_Channel1->CBNDTR, (sizeof(uint32_t) & MDMA_CBNDTR_BNDT));
   WRITE_REG(MDMA_Channel1->CMAR, (uint32_t)&DMA1->LIFCR);
   WRITE_REG(MDMA_Channel1->CMDR, (DMA_LIFCR_CTCIF0 | DMA_LIFCR_CTEIF0 | DMA_LIFCR_CFEIF0));
   WRITE_REG(MDMA_Channel1->CSAR, (uint32_t)&gps_trigger_states[0]);
   WRITE_REG(MDMA_Channel1->CDAR, (uint32_t)&GPS_TIME_TRIGGER_GPIO_Port->BSRR);
   WRITE_REG(MDMA_Channel1->CLAR, (uint32_t)&mdma_gps_xfers[1]);

   // Set up linked list mode for MDMA Channel 1 to enable continuous triggering
   WRITE_REG(mdma_gps_xfers[0].CTCR, (MDMA_SRC_INC_DISABLE | MDMA_DEST_INC_DISABLE | MDMA_SRC_DATASIZE_WORD | MDMA_DEST_DATASIZE_WORD | MDMA_DATAALIGN_PACKENABLE | MDMA_SOURCE_BURST_SINGLE | MDMA_DEST_BURST_SINGLE | ((sizeof(uint32_t) - 1U) << MDMA_CTCR_TLEN_Pos) | MDMA_BUFFER_TRANSFER));
   WRITE_REG(mdma_gps_xfers[0].CBNDTR, (sizeof(uint32_t) & MDMA_CBNDTR_BNDT));
   WRITE_REG(mdma_gps_xfers[0].CMAR, (uint32_t)&DMA1->LIFCR);
   WRITE_REG(mdma_gps_xfers[0].CMDR, (DMA_LIFCR_CTCIF0 | DMA_LIFCR_CTEIF0 | DMA_LIFCR_CFEIF0));
   WRITE_REG(mdma_gps_xfers[0].CSAR, (uint32_t)&gps_trigger_states[0]);
   WRITE_REG(mdma_gps_xfers[0].CDAR, (uint32_t)&GPS_TIME_TRIGGER_GPIO_Port->BSRR);
   WRITE_REG(mdma_gps_xfers[0].CLAR, (uint32_t)&mdma_gps_xfers[1]);
   for (uint32_t i = 1; i < AUDIO_NUM_DMAS_PER_CLIP; ++i)
   {
      mdma_gps_xfers[i] = mdma_gps_xfers[i-1];
      WRITE_REG(mdma_gps_xfers[i].CSAR, (uint32_t)&gps_trigger_states[i]);
      WRITE_REG(mdma_gps_xfers[i].CLAR, (uint32_t)&mdma_gps_xfers[i+1]);
   }
   WRITE_REG(mdma_gps_xfers[AUDIO_NUM_DMAS_PER_CLIP - 1].CLAR, (uint32_t)&mdma_gps_xfers[0]);

   // Set up the MDMA Channel 2 (core-to-core transfer) peripheral
   CLEAR_BIT(MDMA_Channel2->CCR, MDMA_CCR_EN);
   while (READ_BIT(MDMA_Channel2->CCR, MDMA_CCR_EN));
   WRITE_REG(MDMA_Channel2->CCR, (MDMA_PRIORITY_HIGH | MDMA_LITTLE_ENDIANNESS_PRESERVE));
   WRITE_REG(MDMA_Channel2->CTCR, (MDMA_SRC_INC_WORD | MDMA_DEST_INC_WORD | MDMA_SRC_DATASIZE_WORD | MDMA_DEST_DATASIZE_WORD | MDMA_DATAALIGN_PACKENABLE | MDMA_SOURCE_BURST_32BEATS | MDMA_DEST_BURST_32BEATS | ((128 - 1U) << MDMA_CTCR_TLEN_Pos) | MDMA_BLOCK_TRANSFER | MDMA_CTCR_SWRM));
   WRITE_REG(MDMA_Channel2->CTBR, MDMA_CTBR_SBUS);
   WRITE_REG(MDMA_Channel2->CLAR, 0);

   // Configure the DMA buffer addresses and sizes for double buffer mode
   WRITE_REG(DMA1_Stream0->PAR, (uint32_t)&SAI2_Block_B->DR);
   WRITE_REG(DMA1_Stream0->M0AR, (uint32_t)audio_raw[0]);
   WRITE_REG(DMA1_Stream0->M1AR, (uint32_t)audio_raw[1]);
   WRITE_REG(DMA1_Stream0->NDTR, AUDIO_BUFFER_SAMPLES);
   CLEAR_BIT(DMA1_Stream0->CR, (DMA_IT_TC | DMA_IT_TE | DMA_IT_DME | DMA_IT_HT));
   SET_BIT(SAI2_Block_B->IMR, (SAI_IT_OVRUDR | SAI_IT_WCKCFG));

   // Allow chip voltage to stabilize, then wake up the audio chip
   HAL_Delay(500);
#if REV_ID == REV_A
   MICS_NRESET_GPIO_Port->BSRR = MICS_NRESET_Pin;
   HAL_Delay(2);
#endif
   audio_write_reg(SLEEP_CFG_REG, 0b00000001);
   HAL_Delay(2);

   // Switch to register page 0
   audio_write_reg(PAGE_CFG_REG, 0x00);

   // Setup the internal regulator and VREF voltage
   audio_write_reg(BIAS_CFG_REG, 0x62);

   // Configure device to operate in slave mode
   audio_write_reg(MST_CFG0_REG, 0b00000110);

   // Set audio serial format to TDM and data word length to 16 bits
   audio_write_reg(ASI_CFG0_REG, 0b00000000);

   // Configure data offset from FSYNC as 1 cycle
   audio_write_reg(ASI_CFG1_REG, 0b00000001);

   // Disable bus error detection and shutdown
   audio_write_reg(ASI_CFG2_REG, 0b00100000);

   // Configure all volume and gain parameters
   audio_write_reg(CH1_CFG0_REG, 0x40);
   audio_write_reg(CH1_CFG2_REG, AUDIO_DIGITAL_GAIN);
   audio_write_reg(CH2_CFG0_REG, 0x40);
   audio_write_reg(CH3_CFG0_REG, 0x40);
   audio_write_reg(CH4_CFG0_REG, 0x40);
   audio_write_reg(DSP_CFG1_REG, 0b10000000);

   // Disable unused GPIO pins and configure used PDM pins
#if REV_ID == REV_A
   audio_write_reg(GPIO_CFG0_REG, 0x00);
   audio_write_reg(GPO_CFG0_REG, 0x41);
   audio_write_reg(GPO_CFG1_REG, 0x41);
   audio_write_reg(GPI_CFG0_REG, 0x54);
#else
   audio_write_reg(GPIO_CFG0_REG, 0x41);
   audio_write_reg(GPO_CFG0_REG, 0x41);
   audio_write_reg(GPI_CFG0_REG, 0x45);
#endif

   // Configure PDM clock to output at 3.072MHz and latch on the appropriate edges
   audio_write_reg(PDMCLK_CFG_REG, 0x40);
#if REV_ID == REV_A
   audio_write_reg(PDMIN_CFG_REG, 0xC0);
#else
   audio_write_reg(PDMIN_CFG_REG, 0x80);
#endif

   // Configure output channel slot assignments
   audio_write_reg(ASI_CH1_REG, 0x00);
   audio_write_reg(ASI_CH2_REG, 0x01);
   audio_write_reg(ASI_CH3_REG, 0x02);
   audio_write_reg(ASI_CH4_REG, 0x03);

   // Configure enabled channels 1-4
   audio_write_reg(IN_CH_EN_REG, 0b11110000);
   audio_write_reg(ASI_OUT_CH_EN_REG, 0b11110000);

   // Power up all enabled audio channels
   audio_write_reg(PWR_CFG_REG, 0b01100100);

   // Update metadata for the data structure shared between cores
   data.audio_clip_complete = data.audio_read_index = 0;
}

void audio_start(void)
{
   // Initiate automatic reordering and transferring of audio channels using MDMA
   WRITE_REG(MDMA_Channel2->CIFCR, (MDMA_FLAG_TE | MDMA_FLAG_CTC | MDMA_CISR_BRTIF | MDMA_CISR_BTIF | MDMA_CISR_TCIF));
   WRITE_REG(MDMA_Channel1->CIFCR, (MDMA_FLAG_TE | MDMA_FLAG_CTC | MDMA_CISR_BRTIF | MDMA_CISR_BTIF | MDMA_CISR_TCIF));
   WRITE_REG(MDMA_Channel0->CIFCR, (MDMA_FLAG_TE | MDMA_FLAG_CTC | MDMA_CISR_BRTIF | MDMA_CISR_BTIF | MDMA_CISR_TCIF));
   SET_BIT(MDMA_Channel0->CCR, (MDMA_IT_TE | MDMA_IT_BRT | MDMA_CCR_EN));
   SET_BIT(MDMA_Channel1->CCR, MDMA_CCR_EN);

   // Begin reading audio using DMA
   SET_BIT(DMA1_Stream0->CR, DMA_SxCR_EN);
   SET_BIT(SAI2_Block_B->CR1, (SAI_xCR1_DMAEN | SAI_xCR1_SAIEN));
}

#else

// Header Inclusions ---------------------------------------------------------------------------------------------------

#include "ai.h"
#include "audio.h"
#include "cellular.h"
#include "gps.h"
#include "imu.h"
#include "usb.h"


// Static Audio Variables ----------------------------------------------------------------------------------------------

static volatile uint8_t new_audio_received;


// Interrupt Service Routines ------------------------------------------------------------------------------------------

void MDMA_IRQHandler(void)
{
   // Create static variables to avoid stack allocation within an ISR
   static const uint16_t raw_packet_size = sizeof(data.audio[0]);
   static const uint16_t metadata_packet_size = raw_packet_size + offsetof(data_packet_t, audio_read_index) - offsetof(data_packet_t, timestamp);

   // Ensure that the interrupt is relevant to this CPU
   if (READ_BIT(MDMA_Channel2->CISR, (MDMA_FLAG_TE | MDMA_FLAG_BT)))
   {
      // Clear the interrupt and set the new data flag
      WRITE_REG(MDMA_Channel2->CIFCR, (MDMA_FLAG_TE | MDMA_FLAG_BT));
      new_audio_received = 1;

      // Transmit new audio data for external processing
      ai_send((uint8_t*)data.audio[data.audio_read_index], raw_packet_size);
      if (data.audio_clip_complete)
      {
         // Update the audio metadata before transmitting
         gps_update_packet_llh();
         gps_update_packet_timestamp();
         imu_update_packet_orientation();
         cell_update_device_details();
         usb_send((uint8_t*)data.audio[data.audio_read_index], metadata_packet_size);
      }
      else
         usb_send((uint8_t*)data.audio[data.audio_read_index], raw_packet_size);
   }
}


// Public API Functions ------------------------------------------------------------------------------------------------

void audio_init(void)
{
   // Initialize the shared audio data structure
   const uint8_t packet_delimiter[] = PACKET_END_DELIMITER;
   memset((void*)&data, 0, sizeof(data));
   for (int i = 0; i < sizeof(packet_delimiter); ++i)
      data.delimiter[i] = packet_delimiter[i];
   new_audio_received = 0;

   // Enable data transfer completion interrupts
   NVIC_SetPriority(MDMA_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
   NVIC_EnableIRQ(MDMA_IRQn);
}

uint8_t audio_process_new_data(cell_audio_transmit_command_t transmit_evidence)
{
   // Only proceed if there is new unprocessed audio data
   const uint8_t data_processed = new_audio_received;
   if (new_audio_received)
   {
      // Encode the audio data
      new_audio_received = 0;
      const opus_frame_t *result_begin, *result_end;
      opusenc_encode((int16_t*)data.audio[data.audio_read_index], &result_begin, &result_end);

      // Transmit historical data if new evidence transmission was requested
      if (transmit_evidence == CELL_AUDIO_TRANSMIT_BEGIN)
      {
         const opus_frame_t *history_start = opusenc_get_history(), *frame = opusenc_get_history();
         do {
            cell_transmit_audio(frame, 0);
            frame = frame->next;
         } while (frame != history_start);
      }

      // Optionally transmit the newly encoded data over the cellular network
      if (transmit_evidence != CELL_AUDIO_NO_TRANSMIT)
         for (const opus_frame_t *frame = result_begin; frame != result_end; frame = frame->next)
            cell_transmit_audio(frame, (transmit_evidence == CELL_AUDIO_TRANSMIT_END) && (frame->next == result_end));
   }

   // Return whether this function call actually processed anything
   return data_processed;
}

#endif  // #ifdef CORE_CM7
