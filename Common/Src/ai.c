#ifdef CORE_CM4

// Header Inclusions ---------------------------------------------------------------------------------------------------

#include "ai.h"


#if REV_ID != REV_A

// AI Data Type Definitions --------------------------------------------------------------------------------------------

#define I2C_AI_DEVICE_ADDRESS      144
#define AI_INT_IRQn                EXTI9_5_IRQn


// Static AI Communication Variables -----------------------------------------------------------------------------------

__attribute__ ((section (".ramd3")))
static volatile ai_result_t ai_result_buffers[2];

static volatile ai_result_t *ai_result;
static volatile uint8_t ai_result_write_idx, ai_interrupt_received;

static dma_int_registers_t *spi2_dma_int_registers;
static dma_int_registers_t *i2c3_dma_int_registers;


// Interrupt Service Routines ------------------------------------------------------------------------------------------

void DMA2_Stream2_IRQHandler(void)
{
   // Clear the interrupt register contents
   const uint32_t register_contents = spi2_dma_int_registers->ISR;
   WRITE_REG(spi2_dma_int_registers->IFCR, register_contents);

   // Enable the SPI end-of-transfer interrupt if DMA transfer complete
   if (READ_BIT(register_contents, (DMA_FLAG_TCIF0_4 << DMA_STREAM2_6_INDEX)))
      SET_BIT(SPI2->IER, SPI_IT_EOT);
}

void SPI2_IRQHandler(void)
{
   // Check if an SPI end-of-transfer has occurred
   const uint32_t trigger = SPI2->IER & SPI2->SR;
   if (READ_BIT(trigger, SPI_FLAG_EOT))
   {
      // Clear interrupt flags and disable the end-of-transfer interrupt
      SET_BIT(SPI2->IFCR, (SPI_IFCR_EOTC | SPI_IFCR_TXTFC | SPI_IFCR_UDRC | SPI_IFCR_OVRC | SPI_IFCR_MODFC | SPI_IFCR_TIFREC | SPI_IFCR_SUSPC));
      CLEAR_BIT(SPI2->IER, SPI_IT_EOT);
      CLEAR_BIT(SPI2->CR1, SPI_CR1_SPE);
      CLEAR_BIT(SPI2->CFG1, SPI_CFG1_TXDMAEN);
   }
}

void AI_INT_IRQHandler(void)
{
   // Clear the interrupt and set the interrupt-received flag
   WRITE_REG(EXTI->C2PR1, FROM_AI_INT_Pin);
   ai_interrupt_received = 1;
}

void I2C3_ER_IRQHandler(void)
{
   // Reset the DMA reading structure
   CLEAR_BIT(DMA2_Stream0->CR, DMA_SxCR_EN);
   WRITE_REG(i2c3_dma_int_registers->IFCR, i2c3_dma_int_registers->ISR);
   WRITE_REG(DMA2_Stream0->M0AR, (uint32_t)&ai_result_buffers[ai_result_write_idx]);
   WRITE_REG(DMA2_Stream0->NDTR, sizeof(ai_result_buffers[0]));
   SET_BIT(DMA2_Stream0->CR, DMA_SxCR_EN);

   // Clear all I2C3 error interrupts so that reading can continue
   WRITE_REG(I2C3->ICR, (I2C_FLAG_ARLO | I2C_FLAG_BERR | I2C_FLAG_OVR | I2C_FLAG_PECERR | I2C_FLAG_AF | I2C_FLAG_STOPF));
   MODIFY_REG(I2C3->CR2, (I2C_CR2_NACK | I2C_CR2_NBYTES), ((sizeof(ai_result_t) << I2C_CR2_NBYTES_Pos) & I2C_CR2_NBYTES));
}

void I2C3_EV_IRQHandler(void)
{
   // Determine which I2C3 event caused this interrupt
   if (READ_BIT(I2C3->ISR, I2C_FLAG_STOPF))
   {
      // Update the pointer to the current AI detection data
      ai_result = &ai_result_buffers[ai_result_write_idx];
      ai_result_write_idx = (ai_result_write_idx + 1) % 2;
      WRITE_REG(I2C3->ICR, (I2C_FLAG_AF | I2C_FLAG_STOPF));

      // Prepare the DMA to read the next AI detection data
      CLEAR_BIT(DMA2_Stream0->CR, DMA_SxCR_EN);
      WRITE_REG(i2c3_dma_int_registers->IFCR, i2c3_dma_int_registers->ISR);
      WRITE_REG(DMA2_Stream0->M0AR, (uint32_t)&ai_result_buffers[ai_result_write_idx]);
      WRITE_REG(DMA2_Stream0->NDTR, sizeof(ai_result_buffers[0]));
      SET_BIT(DMA2_Stream0->CR, DMA_SxCR_EN);

      // Initiate the AI data read
      MODIFY_REG(I2C3->CR2, (I2C_CR2_NACK | I2C_CR2_NBYTES), ((sizeof(ai_result_t) << I2C_CR2_NBYTES_Pos) & I2C_CR2_NBYTES));
   }
}


// Private Helper Functions --------------------------------------------------------------------------------------------

static void validate_comms(void)
{
   // Wait 5 seconds for the AI interrupt line to be asserted low
   for (uint32_t i = 0; READ_BIT(FROM_AI_INT_GPIO_Port->IDR, FROM_AI_INT_Pin) && (i <= 50); ++i)
      HAL_Delay(100);

   // Force a system reset if not asserted within 5 seconds
   if (READ_BIT(FROM_AI_INT_GPIO_Port->IDR, FROM_AI_INT_Pin))
      NVIC_SystemReset();
}


// Public API Functions ------------------------------------------------------------------------------------------------

void ai_comms_init(void)
{
   // Initialize the AI static variables
   ai_result = NULL;
   ai_result_write_idx = 0;

   // Initialize the various GPIO clocks
   SET_BIT(RCC->AHB4ENR, RCC_AHB4ENR_GPIOAEN);
   (void)READ_BIT(RCC->AHB4ENR, RCC_AHB4ENR_GPIOAEN);
   SET_BIT(RCC->AHB4ENR, RCC_AHB4ENR_GPIOBEN);
   (void)READ_BIT(RCC->AHB4ENR, RCC_AHB4ENR_GPIOBEN);
   SET_BIT(RCC->AHB4ENR, RCC_AHB4ENR_GPIOCEN);
   (void)READ_BIT(RCC->AHB4ENR, RCC_AHB4ENR_GPIOCEN);

   // Initialize the DMA2 clock
   SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_DMA2EN);
   (void)READ_BIT(RCC->AHB1ENR, RCC_AHB1ENR_DMA2EN);

   // Initialize the SPI2 peripheral clock
   MODIFY_REG(RCC->D2CCIP1R, RCC_D2CCIP1R_SPI123SEL, (uint32_t)RCC_SPI123CLKSOURCE_CLKP);
   SET_BIT(RCC->APB1LENR, RCC_APB1LENR_SPI2EN);
   (void)READ_BIT(RCC->APB1LENR, RCC_APB1LENR_SPI2EN);

   // Initialize the I2C3 peripheral clock
   MODIFY_REG(RCC->D2CCIP2R, RCC_D2CCIP2R_I2C123SEL, RCC_I2C3CLKSOURCE_D2PCLK1);
   SET_BIT(RCC->APB1LENR, RCC_APB1LENR_I2C3EN);
   (void)READ_BIT(RCC->APB1LENR, RCC_APB1LENR_I2C3EN);

   // Initialize the SYSCFG clock
   SET_BIT(RCC->APB4ENR, RCC_APB4ENR_SYSCFGEN);
   (void)READ_BIT(RCC->APB4ENR, RCC_APB4ENR_SYSCFGEN);

   // Initialize the non-peripheral GPIO pins
   uint32_t position = 32 - __builtin_clz(FROM_AI_INT_Pin) - 1;
   MODIFY_REG(FROM_AI_INT_GPIO_Port->PUPDR, (GPIO_PUPDR_PUPD0 << (position * 2U)), (GPIO_PULLUP << (position * 2U)));
#ifdef AI_TO_HOST_INT
   uint32_t iocurrent = FROM_AI_INT_Pin & (1UL << position);
   MODIFY_REG(FROM_AI_INT_GPIO_Port->MODER, (GPIO_MODER_MODE0 << (position * 2U)), ((GPIO_MODE_IT_FALLING & GPIO_MODE) << (position * 2U)));
   MODIFY_REG(SYSCFG->EXTICR[position >> 2U], (0x0FUL << (4U * (position & 0x03U))), (GPIO_GET_INDEX(FROM_AI_INT_GPIO_Port) << (4U * (position & 0x03U))));
   CLEAR_BIT(EXTI->RTSR1, iocurrent);
   SET_BIT(EXTI->FTSR1, iocurrent);
   CLEAR_BIT(EXTI_D2->EMR1, iocurrent);
   SET_BIT(EXTI_D2->IMR1, iocurrent);
#else
   MODIFY_REG(FROM_AI_INT_GPIO_Port->MODER, (GPIO_MODER_MODE0 << (position * 2U)), ((GPIO_MODE_INPUT & GPIO_MODE) << (position * 2U)));
#endif

   // Initialize the SPI2 GPIO pins
   position = 32 - __builtin_clz(TO_AI_CS_Pin) - 1;
   MODIFY_REG(TO_AI_CS_GPIO_Port->OSPEEDR, (GPIO_OSPEEDR_OSPEED0 << (position * 2U)), (GPIO_SPEED_FREQ_HIGH << (position * 2U)));
   MODIFY_REG(TO_AI_CS_GPIO_Port->OTYPER, (GPIO_OTYPER_OT0 << position), (((GPIO_MODE_AF_PP & OUTPUT_TYPE) >> OUTPUT_TYPE_Pos) << position));
   CLEAR_BIT(TO_AI_CS_GPIO_Port->PUPDR, (GPIO_PUPDR_PUPD0 << (position * 2U)));
   MODIFY_REG(TO_AI_CS_GPIO_Port->AFR[position >> 3U], (0xFU << ((position & 0x07U) * 4U)), (GPIO_AF5_SPI2 << ((position & 0x07U) * 4U)));
   MODIFY_REG(TO_AI_CS_GPIO_Port->MODER, (GPIO_MODER_MODE0 << (position * 2U)), ((GPIO_MODE_AF_PP & GPIO_MODE) << (position * 2U)));
   position = 32 - __builtin_clz(TO_AI_SCK_Pin) - 1;
   MODIFY_REG(TO_AI_SCK_GPIO_Port->OSPEEDR, (GPIO_OSPEEDR_OSPEED0 << (position * 2U)), (GPIO_SPEED_FREQ_HIGH << (position * 2U)));
   MODIFY_REG(TO_AI_SCK_GPIO_Port->OTYPER, (GPIO_OTYPER_OT0 << position), (((GPIO_MODE_AF_PP & OUTPUT_TYPE) >> OUTPUT_TYPE_Pos) << position));
   CLEAR_BIT(TO_AI_SCK_GPIO_Port->PUPDR, (GPIO_PUPDR_PUPD0 << (position * 2U)));
   MODIFY_REG(TO_AI_SCK_GPIO_Port->AFR[position >> 3U], (0xFU << ((position & 0x07U) * 4U)), (GPIO_AF5_SPI2 << ((position & 0x07U) * 4U)));
   MODIFY_REG(TO_AI_SCK_GPIO_Port->MODER, (GPIO_MODER_MODE0 << (position * 2U)), ((GPIO_MODE_AF_PP & GPIO_MODE) << (position * 2U)));
   position = 32 - __builtin_clz(TO_AI_MISO_Pin) - 1;
   MODIFY_REG(TO_AI_MISO_GPIO_Port->OSPEEDR, (GPIO_OSPEEDR_OSPEED0 << (position * 2U)), (GPIO_SPEED_FREQ_HIGH << (position * 2U)));
   MODIFY_REG(TO_AI_MISO_GPIO_Port->OTYPER, (GPIO_OTYPER_OT0 << position), (((GPIO_MODE_AF_PP & OUTPUT_TYPE) >> OUTPUT_TYPE_Pos) << position));
   CLEAR_BIT(TO_AI_MISO_GPIO_Port->PUPDR, (GPIO_PUPDR_PUPD0 << (position * 2U)));
   MODIFY_REG(TO_AI_MISO_GPIO_Port->AFR[position >> 3U], (0xFU << ((position & 0x07U) * 4U)), (GPIO_AF5_SPI2 << ((position & 0x07U) * 4U)));
   MODIFY_REG(TO_AI_MISO_GPIO_Port->MODER, (GPIO_MODER_MODE0 << (position * 2U)), ((GPIO_MODE_AF_PP & GPIO_MODE) << (position * 2U)));
   position = 32 - __builtin_clz(TO_AI_MOSI_Pin) - 1;
   MODIFY_REG(TO_AI_MOSI_GPIO_Port->OSPEEDR, (GPIO_OSPEEDR_OSPEED0 << (position * 2U)), (GPIO_SPEED_FREQ_HIGH << (position * 2U)));
   MODIFY_REG(TO_AI_MOSI_GPIO_Port->OTYPER, (GPIO_OTYPER_OT0 << position), (((GPIO_MODE_AF_PP & OUTPUT_TYPE) >> OUTPUT_TYPE_Pos) << position));
   CLEAR_BIT(TO_AI_MOSI_GPIO_Port->PUPDR, (GPIO_PUPDR_PUPD0 << (position * 2U)));
   MODIFY_REG(TO_AI_MOSI_GPIO_Port->AFR[position >> 3U], (0xFU << ((position & 0x07U) * 4U)), (GPIO_AF5_SPI2 << ((position & 0x07U) * 4U)));
   MODIFY_REG(TO_AI_MOSI_GPIO_Port->MODER, (GPIO_MODER_MODE0 << (position * 2U)), ((GPIO_MODE_AF_PP & GPIO_MODE) << (position * 2U)));

   // Initialize the I2C3 GPIO pins
   position = 32 - __builtin_clz(FROM_AI_SCL_Pin) - 1;
   MODIFY_REG(FROM_AI_SCL_GPIO_Port->OSPEEDR, (GPIO_OSPEEDR_OSPEED0 << (position * 2U)), (GPIO_SPEED_FREQ_MEDIUM << (position * 2U)));
   MODIFY_REG(FROM_AI_SCL_GPIO_Port->OTYPER, (GPIO_OTYPER_OT0 << position), (((GPIO_MODE_AF_OD & OUTPUT_TYPE) >> OUTPUT_TYPE_Pos) << position));
#if REV_ID > REV_B
   CLEAR_BIT(FROM_AI_SCL_GPIO_Port->PUPDR, (GPIO_PUPDR_PUPD0 << (position * 2U)));
#else
   MODIFY_REG(FROM_AI_SCL_GPIO_Port->PUPDR, (GPIO_PUPDR_PUPD0 << (position * 2U)), (GPIO_PULLUP << (position * 2U)));
#endif
   MODIFY_REG(FROM_AI_SCL_GPIO_Port->AFR[position >> 3U], (0xFU << ((position & 0x07U) * 4U)), (GPIO_AF4_I2C3 << ((position & 0x07U) * 4U)));
   MODIFY_REG(FROM_AI_SCL_GPIO_Port->MODER, (GPIO_MODER_MODE0 << (position * 2U)), ((GPIO_MODE_AF_OD & GPIO_MODE) << (position * 2U)));
   position = 32 - __builtin_clz(FROM_AI_SDA_Pin) - 1;
   MODIFY_REG(FROM_AI_SDA_GPIO_Port->OSPEEDR, (GPIO_OSPEEDR_OSPEED0 << (position * 2U)), (GPIO_SPEED_FREQ_MEDIUM << (position * 2U)));
   MODIFY_REG(FROM_AI_SDA_GPIO_Port->OTYPER, (GPIO_OTYPER_OT0 << position), (((GPIO_MODE_AF_OD & OUTPUT_TYPE) >> OUTPUT_TYPE_Pos) << position));
#if REV_ID > REV_B
   CLEAR_BIT(FROM_AI_SDA_GPIO_Port->PUPDR, (GPIO_PUPDR_PUPD0 << (position * 2U)));
#else
   MODIFY_REG(FROM_AI_SDA_GPIO_Port->PUPDR, (GPIO_PUPDR_PUPD0 << (position * 2U)), (GPIO_PULLUP << (position * 2U)));
#endif
   MODIFY_REG(FROM_AI_SDA_GPIO_Port->AFR[position >> 3U], (0xFU << ((position & 0x07U) * 4U)), (GPIO_AF4_I2C3 << ((position & 0x07U) * 4U)));
   MODIFY_REG(FROM_AI_SDA_GPIO_Port->MODER, (GPIO_MODER_MODE0 << (position * 2U)), ((GPIO_MODE_AF_OD & GPIO_MODE) << (position * 2U)));

   // Initialize the SPI2 peripheral
   CLEAR_BIT(SPI2->CR1, SPI_CR1_SPE);
   while (READ_BIT(SPI2->CR1, SPI_CR1_SPE));
   uint32_t crc_length = SPI2->CFG1 & SPI_CFG1_CRCSIZE;
   MODIFY_REG(SPI2->CR1, SPI_CR1_MASRX, SPI_MASTER_RX_AUTOSUSP_DISABLE);
   WRITE_REG(SPI2->CFG1, (SPI_BAUDRATEPRESCALER_2 | crc_length | SPI_FIFO_THRESHOLD_04DATA | SPI_DATASIZE_32BIT));
   WRITE_REG(SPI2->CFG2, (SPI_NSS_PULSE_ENABLE | SPI_NSS_HARD_OUTPUT | SPI_POLARITY_LOW | SPI_PHASE_1EDGE | SPI_MODE_MASTER | SPI_MASTER_KEEP_IO_STATE_ENABLE | SPI_DIRECTION_2LINES_TXONLY));
   CLEAR_BIT(SPI2->I2SCFGR, SPI_I2SCFGR_I2SMOD);

   // Initialize DMA2 Stream2 for SPI2 TX
   CLEAR_BIT(DMA2_Stream2->CR, DMA_SxCR_EN);
   while (READ_BIT(DMA2_Stream2->CR, DMA_SxCR_EN));
   MODIFY_REG(DMA2_Stream2->CR,
             (DMA_SxCR_MBURST | DMA_SxCR_PBURST | DMA_SxCR_PL | DMA_SxCR_MSIZE | DMA_SxCR_PSIZE | DMA_SxCR_MINC | DMA_SxCR_PINC | DMA_SxCR_CIRC | DMA_SxCR_DIR | DMA_SxCR_CT | DMA_SxCR_DBM),
             (DMA_MEMORY_TO_PERIPH | DMA_MINC_ENABLE | DMA_PDATAALIGN_WORD | DMA_MDATAALIGN_WORD | DMA_PRIORITY_HIGH | DMA_MBURST_INC4 | DMA_PBURST_INC4));
   MODIFY_REG(DMA2_Stream2->FCR, (DMA_SxFCR_DMDIS | DMA_SxFCR_FTH), DMA_FIFOMODE_ENABLE | DMA_FIFO_THRESHOLD_FULL);
   spi2_dma_int_registers = (dma_int_registers_t*)((uint32_t)DMA2_Stream2 & (uint32_t)(~0x3FFU));
   spi2_dma_int_registers->IFCR = 0x3FUL << DMA_STREAM2_6_INDEX;
   WRITE_REG(DMAMUX1_Channel10->CCR, DMA_REQUEST_SPI2_TX);
   WRITE_REG(DMAMUX1_ChannelStatus->CFR, (1UL << (10 & 0x1FU)));

   // Configure the SPI2 TX DMA peripheral address
   WRITE_REG(DMA2_Stream2->PAR, (uint32_t)&SPI2->TXDR);
   CLEAR_BIT(DMA2_Stream2->CR, (DMA_IT_TC | DMA_IT_TE | DMA_IT_DME | DMA_IT_HT));

   // Initialize DMA2 Stream0 for I2C3_RX
   CLEAR_BIT(DMA2_Stream0->CR, DMA_SxCR_EN);
   while (READ_BIT(DMA2_Stream0->CR, DMA_SxCR_EN));
   MODIFY_REG(DMA2_Stream0->CR,
             (DMA_SxCR_MBURST | DMA_SxCR_PBURST | DMA_SxCR_PL | DMA_SxCR_MSIZE | DMA_SxCR_PSIZE | DMA_SxCR_MINC | DMA_SxCR_PINC | DMA_SxCR_CIRC | DMA_SxCR_DIR | DMA_SxCR_CT | DMA_SxCR_DBM),
             (DMA_PERIPH_TO_MEMORY | DMA_MINC_ENABLE | DMA_PDATAALIGN_BYTE | DMA_MDATAALIGN_BYTE | DMA_PRIORITY_LOW | DMA_MBURST_SINGLE | DMA_PBURST_SINGLE));
   i2c3_dma_int_registers = (dma_int_registers_t*)((uint32_t)DMA2_Stream0 & (uint32_t)(~0x3FFU));
   i2c3_dma_int_registers->IFCR = 0x3FUL << DMA_STREAM0_4_INDEX;
   WRITE_REG(DMAMUX1_Channel8->CCR, DMA_REQUEST_I2C3_RX);
   WRITE_REG(DMAMUX1_ChannelStatus->CFR, (1UL << (8 & 0x1FU)));

   // Configure the I2C3 DMA buffer addresses and sizes
   WRITE_REG(DMA2_Stream0->PAR, (uint32_t)&I2C3->RXDR);
   WRITE_REG(DMA2_Stream0->M0AR, (uint32_t)&ai_result_buffers[0]);
   WRITE_REG(DMA2_Stream0->NDTR, sizeof(ai_result_buffers[0]));
   CLEAR_BIT(DMA2_Stream0->CR, (DMA_IT_TC | DMA_IT_TE | DMA_IT_DME | DMA_IT_HT));

   // Set up the I2C3 peripheral
   WRITE_REG(I2C3->CR1, 0);
   WRITE_REG(I2C3->TIMINGR, 0x0080123A);
   CLEAR_BIT(I2C3->OAR1, I2C_OAR1_OA1EN);
   WRITE_REG(I2C3->OAR1, (I2C_OAR1_OA1EN | I2C_AI_DEVICE_ADDRESS));
   WRITE_REG(I2C3->OAR2, 0);
   MODIFY_REG(I2C3->CR2, (I2C_CR2_ADD10 | I2C_CR2_SADD | I2C_CR2_NBYTES | I2C_CR2_RELOAD | I2C_AUTOEND_MODE), I2C_CR2_NACK);

   // Enable the SPI2 and DMA2 Stream2 interrupts
   NVIC_SetPriority(DMA2_Stream2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
   NVIC_EnableIRQ(DMA2_Stream2_IRQn);
   NVIC_SetPriority(SPI2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 1));
   NVIC_EnableIRQ(SPI2_IRQn);
}

void ai_comms_start(void)
{
   // Enable the I2C3 interrupts and peripheral
   NVIC_SetPriority(I2C3_ER_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 2));
   NVIC_EnableIRQ(I2C3_ER_IRQn);
   NVIC_SetPriority(I2C3_EV_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 2));
   NVIC_EnableIRQ(I2C3_EV_IRQn);
   SET_BIT(DMA2_Stream0->CR, DMA_SxCR_EN);
   WRITE_REG(I2C3->CR1, (I2C_CR1_ERRIE | I2C_CR1_STOPIE | I2C_CR1_NOSTRETCH | I2C_CR1_RXDMAEN | I2C_CR1_PE));//I2C_CR1_ADDRIE
   MODIFY_REG(I2C3->CR2, I2C_CR2_NACK, ((sizeof(ai_result_t) << I2C_CR2_NBYTES_Pos) & I2C_CR2_NBYTES));

   // Enable incoming AI interrupts
#ifdef AI_TO_HOST_INT
   WRITE_REG(EXTI->C2PR1, FROM_AI_INT_Pin);
   NVIC_SetPriority(AI_INT_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 3));
   NVIC_EnableIRQ(AI_INT_IRQn);
#endif
}

void ai_send(const uint8_t *data, uint16_t data_length)
{
   // Initiate data transfer using DMA
   CLEAR_BIT(SPI2->CFG1, SPI_CFG1_TXDMAEN);
   CLEAR_BIT(DMA2_Stream2->CR, DMA_SxCR_EN);
   WRITE_REG(spi2_dma_int_registers->IFCR, (0x3FUL << DMA_STREAM2_6_INDEX));
   WRITE_REG(DMA2_Stream2->NDTR, data_length / 4);
   WRITE_REG(DMA2_Stream2->M0AR, (uint32_t)data);
   MODIFY_REG(DMA2_Stream2->CR, (DMA_IT_TC | DMA_IT_TE | DMA_IT_DME | DMA_IT_HT), (DMA_IT_TC | DMA_IT_TE | DMA_IT_DME));
   SET_BIT(DMA2_Stream2->CR, DMA_SxCR_EN);
   MODIFY_REG(SPI2->CR2, SPI_CR2_TSIZE, data_length / 4);
   SET_BIT(SPI2->CFG1, SPI_CFG1_TXDMAEN);
   SET_BIT(SPI2->CR1, SPI_CR1_SPE);
   SET_BIT(SPI2->CR1, SPI_CR1_CSTART);
}

void ai_process_detections(void)
{
   // Validate that AI communications are functioning properly
   validate_comms();

   // Process any new AI event detections
   if (ai_result)
   {
      memcpy((uint8_t*)&device_info.ai_firmware_version, (uint8_t*)ai_result->ai_firmware_version, AI_FIRMWARE_VERSION_LENGTH);
      ai_result = NULL;
   }
}

#else

void ai_comms_init(void) {}
void ai_comms_start(void) {}
void ai_send(const uint8_t *data, uint16_t data_length) {}
void ai_process_detections(void) {}

#endif  // #if REV_ID != REV_A

#endif  // #ifdef CORE_CM4
