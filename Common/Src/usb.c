#ifdef CORE_CM4

// Header Inclusions ---------------------------------------------------------------------------------------------------

#include "usb.h"
#include "usbd_cdc.h"


// USB Device Definitions ----------------------------------------------------------------------------------------------

#define DEVICE_ID1                   (UID_BASE)
#define DEVICE_ID2                   (UID_BASE + 0x4)
#define DEVICE_ID3                   (UID_BASE + 0x8)

#define DEVICE_SERIAL0               0x11223344
#define DEVICE_SERIAL1               0x55667788
#define DEVICE_SERIAL2               0x12345678

#define USB_SIZE_STRING_SERIAL       0x1A
#define USB_NUM_ENDPOINTS            3

#define USB_LANGID_STRING            1033
#define USB_CONFIGURATION_STRING     "CDC Config"
#define USB_INTERFACE_STRING         "CDC Interface"

#define CLEAR_FLAG(REG, INT)         ((REG) &= (INT))


// USB Function Forward Definitions ------------------------------------------------------------------------------------

static uint8_t* USB_DeviceDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t* USB_LangIDStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t* USB_ManufacturerStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t* USB_ProductStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t* USB_SerialStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t* USB_ConfigStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t* USB_InterfaceStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);

static int8_t CDC_Init(void);
static int8_t CDC_DeInit(void);
static int8_t CDC_Control(uint8_t cmd, uint8_t *pbuf, uint16_t length);
static int8_t CDC_Receive(uint8_t *pbuf, uint32_t *len);
static int8_t CDC_TransmitCplt(uint8_t *pbuf, uint32_t *len, uint8_t epnum);


// USB Static Descriptors and Configuration Variables ------------------------------------------------------------------

static USBD_HandleTypeDef usb_cdc_device;
static USB_OTG_DeviceTypeDef *usb_device = (USB_OTG_DeviceTypeDef*)((uint32_t)USB_OTG_FS + USB_OTG_DEVICE_BASE);
static USB_OTG_OUTEndpointTypeDef* out_endpoints[USB_NUM_ENDPOINTS] = {
   (USB_OTG_OUTEndpointTypeDef*)((uint32_t)USB_OTG_FS + USB_OTG_OUT_ENDPOINT_BASE + (0 * USB_OTG_EP_REG_SIZE)),
   (USB_OTG_OUTEndpointTypeDef*)((uint32_t)USB_OTG_FS + USB_OTG_OUT_ENDPOINT_BASE + (1 * USB_OTG_EP_REG_SIZE)),
   (USB_OTG_OUTEndpointTypeDef*)((uint32_t)USB_OTG_FS + USB_OTG_OUT_ENDPOINT_BASE + (2 * USB_OTG_EP_REG_SIZE))
};
static USB_OTG_INEndpointTypeDef* in_endpoints[USB_NUM_ENDPOINTS] = {
   (USB_OTG_INEndpointTypeDef*)((uint32_t)USB_OTG_FS + USB_OTG_IN_ENDPOINT_BASE + (0 * USB_OTG_EP_REG_SIZE)),
   (USB_OTG_INEndpointTypeDef*)((uint32_t)USB_OTG_FS + USB_OTG_IN_ENDPOINT_BASE + (1 * USB_OTG_EP_REG_SIZE)),
   (USB_OTG_INEndpointTypeDef*)((uint32_t)USB_OTG_FS + USB_OTG_IN_ENDPOINT_BASE + (2 * USB_OTG_EP_REG_SIZE))
};
static volatile uint32_t *fifo_base = (volatile uint32_t*)((uint32_t)USB_OTG_FS + USB_OTG_FIFO_BASE);
static USB_EPTypeDef out_endpoint_params[USB_NUM_ENDPOINTS];
static USB_EPTypeDef in_endpoint_params[USB_NUM_ENDPOINTS];
static uint32_t setup_buffer[12];

__attribute__ ((aligned (4)))
static uint8_t usb_receive_buffer[2048];

__attribute__ ((aligned (4)))
static uint8_t usb_string_buffer[USBD_MAX_STR_DESC_SIZ];

__attribute__ ((aligned (4)))
static uint8_t usb_serial_buffer[USB_SIZE_STRING_SERIAL] = {
   USB_SIZE_STRING_SERIAL,
   USB_DESC_TYPE_STRING,
};

static USBD_DescriptorsTypeDef usb_descriptors = {
   USB_DeviceDescriptor,
   USB_LangIDStrDescriptor,
   USB_ManufacturerStrDescriptor,
   USB_ProductStrDescriptor,
   USB_SerialStrDescriptor,
   USB_ConfigStrDescriptor,
   USB_InterfaceStrDescriptor
};

__attribute__ ((aligned (4)))
static uint8_t usb_device_descriptor[USB_LEN_DEV_DESC] = {
   0x12,                       // bLength
   USB_DESC_TYPE_DEVICE,       // bDescriptorType
   0x00,                       // bcdUSB
   0x02,
   0x02,                       // bDeviceClass
   0x02,                       // bDeviceSubClass
   0x00,                       // bDeviceProtocol
   USB_MAX_EP0_SIZE,           // bMaxPacketSize
   LOBYTE(USB_VID),            // idVendor
   HIBYTE(USB_VID),            // idVendor
   LOBYTE(USB_PID),            // idProduct
   HIBYTE(USB_PID),            // idProduct
   0x00,                       // bcdDevice Rel. 2.00
   0x02,
   USBD_IDX_MFC_STR,           // Index of manufacturer  string
   USBD_IDX_PRODUCT_STR,       // Index of product string
   USBD_IDX_SERIAL_STR,        // Index of serial number string
   USBD_MAX_NUM_CONFIGURATION  // bNumConfigurations
};

__attribute__ ((aligned (4)))
static uint8_t usb_lang_descriptor[USB_LEN_LANGID_STR_DESC] = {
   USB_LEN_LANGID_STR_DESC,
   USB_DESC_TYPE_STRING,
   LOBYTE(USB_LANGID_STRING),
   HIBYTE(USB_LANGID_STRING)
};

static USBD_CDC_ItfTypeDef usb_interface_ops = {
   CDC_Init,
   CDC_DeInit,
   CDC_Control,
   CDC_Receive,
   CDC_TransmitCplt
};

static USBD_CDC_LineCodingTypeDef usb_line_coding = { 115200, 0x00, 0x00, 0x08 };


// Lower Level USB Driver Interface ------------------------------------------------------------------------------------

static void USB_LL_Read(uint8_t *dest, uint32_t len)
{
   // Read all word-aligned bytes from the FIFO
   const uint32_t count32b = len >> 2;
   for (uint32_t i = 0; i < count32b; ++i)
   {
      __UNALIGNED_UINT32_WRITE(dest, *fifo_base);
      dest += 4;
   }

   // Read remaining non-word-aligned bytes
   uint32_t pData, remaining_bytes = len % 4;
   if (remaining_bytes)
   {
      __UNALIGNED_UINT32_WRITE(&pData, *fifo_base);
      for (uint32_t i = 0; remaining_bytes; ++i, --remaining_bytes)
         *(dest++) = (uint8_t)(pData >> (8U * i));
   }
}

static void USB_LL_EP0_Write(void)
{
   // Initiate a write to the control endpoint
   if (!READ_BIT(out_endpoints[0]->DOEPCTL, USB_OTG_DOEPCTL_EPENA))
   {
      WRITE_REG(out_endpoints[0]->DOEPTSIZ, ((USB_OTG_DOEPTSIZ_PKTCNT & (1U << 19)) | (3U * 8U) | USB_OTG_DOEPTSIZ_STUPCNT));
      WRITE_REG(out_endpoints[0]->DOEPDMA, (uint32_t)setup_buffer);
      SET_BIT(out_endpoints[0]->DOEPCTL, (USB_OTG_DOEPCTL_EPENA | USB_OTG_DOEPCTL_USBAEP));
   }
}

static void USB_LL_FlushTx(uint32_t num)
{
   // Flush all data currently in the TX FIFO
   for (uint32_t retries = 0; (retries < 10000) && !READ_BIT(USB_OTG_FS->GRSTCTL, USB_OTG_GRSTCTL_AHBIDL); ++retries);
   WRITE_REG(USB_OTG_FS->GRSTCTL, (USB_OTG_GRSTCTL_TXFFLSH | (num << 6)));
   for (uint32_t retries = 0; (retries < 10000) && READ_BIT(USB_OTG_FS->GRSTCTL, USB_OTG_GRSTCTL_TXFFLSH); ++retries);
}

static void USB_LL_FlushRx(void)
{
   // Flush all data currently in the RX FIFO
   for (uint32_t retries = 0; (retries < 10000) && !READ_BIT(USB_OTG_FS->GRSTCTL, USB_OTG_GRSTCTL_AHBIDL); ++retries);
   WRITE_REG(USB_OTG_FS->GRSTCTL, USB_OTG_GRSTCTL_RXFFLSH);
   for (uint32_t retries = 0; (retries < 10000) && READ_BIT(USB_OTG_FS->GRSTCTL, USB_OTG_GRSTCTL_RXFFLSH); ++retries);
}

USBD_StatusTypeDef USBD_LL_Init(USBD_HandleTypeDef *pdev)
{
   // Disable USB interrupts
   CLEAR_BIT(USB_OTG_FS->GAHBCFG, USB_OTG_GAHBCFG_GINT);

   // Select the FS Embedded PHY and reset
   volatile uint32_t count = 0;
   SET_BIT(USB_OTG_FS->GUSBCFG, USB_OTG_GUSBCFG_PHYSEL);
   do { if (++count > HAL_USB_TIMEOUT) return USBD_FAIL; } while (!READ_BIT(USB_OTG_FS->GRSTCTL, USB_OTG_GRSTCTL_AHBIDL));
   SET_BIT(USB_OTG_FS->GRSTCTL, USB_OTG_GRSTCTL_CSRST);
   count = 0;
   do { if (++count > HAL_USB_TIMEOUT) return USBD_FAIL; } while (READ_BIT(USB_OTG_FS->GRSTCTL, USB_OTG_GRSTCTL_CSRST));

   // Activate the USB Transceiver
   SET_BIT(USB_OTG_FS->GCCFG, USB_OTG_GCCFG_PWRDWN);

   // Reserve 18 FIFO Locations for the DMA buffers
   MODIFY_REG(USB_OTG_FS->GDFIFOCFG, (0xFFFFU << 16), (0x3EEU << 16));
   SET_BIT(USB_OTG_FS->GAHBCFG, (USB_OTG_GAHBCFG_HBSTLEN_2 | USB_OTG_GAHBCFG_DMAEN));

   // Force USB Device Mode
   count = 0;
   MODIFY_REG(USB_OTG_FS->GUSBCFG, USB_OTG_GUSBCFG_FHMOD, USB_OTG_GUSBCFG_FDMOD);
   do { HAL_Delay(10); } while ((((USB_OTG_FS->GINTSTS) & 0x1U) != USB_DEVICE_MODE) && ((count += 10) < HAL_USB_CURRENT_MODE_MAX_DELAY_MS));

   // Initialize all endpoint and FIFO structures
   memset(in_endpoint_params, 0, sizeof(in_endpoint_params));
   memset(out_endpoint_params, 0, sizeof(out_endpoint_params));
   for (uint8_t i = 0; i < USB_NUM_ENDPOINTS; ++i)
   {
      in_endpoint_params[i].is_in = 1;
      in_endpoint_params[i].num = i;
      in_endpoint_params[i].tx_fifo_num = i;
      out_endpoint_params[i].num = i;
   }
   for (uint32_t i = 0; i < 15; ++i)
      USB_OTG_FS->DIEPTXF[i] = 0;

   // Deactivate VBUS sensing
   SET_BIT(usb_device->DCTL, USB_OTG_DCTL_SDIS);
   CLEAR_BIT(USB_OTG_FS->GCCFG, USB_OTG_GCCFG_VBDEN);
   SET_BIT(USB_OTG_FS->GOTGCTL, (USB_OTG_GOTGCTL_BVALOEN | USB_OTG_GOTGCTL_BVALOVAL));

   // Restart the PHY clock and set the core to Full Speed
   WRITE_REG(*(volatile uint32_t*)((uint32_t)USB_OTG_FS + USB_OTG_PCGCCTL_BASE), 0);
   SET_BIT(usb_device->DCFG, USB_OTG_SPEED_FULL);

   // Flush the FIFOs
   USB_LL_FlushTx(0x10U);
   USB_LL_FlushRx();

   // Clear all pending device interrupts
   WRITE_REG(usb_device->DIEPMSK, 0U);
   WRITE_REG(usb_device->DOEPMSK, 0U);
   WRITE_REG(usb_device->DAINTMSK, 0U);
   for (uint8_t i = 0; i < USB_NUM_ENDPOINTS; ++i)
   {
      USB_OTG_INEndpointTypeDef* inep = in_endpoints[i];
      if (READ_BIT(inep->DIEPCTL, USB_OTG_DIEPCTL_EPENA))
         WRITE_REG(inep->DIEPCTL, (i ? (USB_OTG_DIEPCTL_EPDIS | USB_OTG_DIEPCTL_SNAK) : USB_OTG_DIEPCTL_SNAK));
      else
         WRITE_REG(inep->DIEPCTL, 0U);
      WRITE_REG(inep->DIEPTSIZ, 0U);
      WRITE_REG(inep->DIEPINT, 0xFB7FU);

      USB_OTG_OUTEndpointTypeDef* outep = out_endpoints[i];
      if (READ_BIT(outep->DOEPCTL, USB_OTG_DOEPCTL_EPENA))
         WRITE_REG(outep->DOEPCTL, (i ? (USB_OTG_DOEPCTL_EPDIS | USB_OTG_DOEPCTL_SNAK) : USB_OTG_DOEPCTL_SNAK));
      else
         WRITE_REG(outep->DOEPCTL, 0U);
      WRITE_REG(outep->DOEPTSIZ, 0U);
      WRITE_REG(outep->DOEPINT, 0xFB7FU);
   }
   CLEAR_BIT(usb_device->DIEPMSK, USB_OTG_DIEPMSK_TXFURM);
   WRITE_REG(USB_OTG_FS->GINTMSK, 0U);
   WRITE_REG(USB_OTG_FS->GINTSTS, 0xBFFFFFFFU);
   SET_BIT(USB_OTG_FS->GINTMSK, (USB_OTG_GINTMSK_USBSUSPM | USB_OTG_GINTMSK_USBRST | USB_OTG_GINTMSK_ENUMDNEM | USB_OTG_GINTMSK_IEPINT | USB_OTG_GINTMSK_OEPINT | USB_OTG_GINTMSK_WUIM));

   // Restart the USB device clock
   CLEAR_BIT(*(volatile uint32_t*)((uint32_t)USB_OTG_FS + USB_OTG_PCGCCTL_BASE), (USB_OTG_PCGCCTL_STOPCLK | USB_OTG_PCGCCTL_GATECLK));
   SET_BIT(usb_device->DCTL, USB_OTG_DCTL_SDIS);

   // Set up the DMA configuration
   WRITE_REG(USB_OTG_FS->GRXFSIZ, 0x80);
   WRITE_REG(USB_OTG_FS->DIEPTXF0_HNPTXFSIZ, (((uint32_t)0x40 << 16) | USB_OTG_FS->GRXFSIZ));
   WRITE_REG(USB_OTG_FS->DIEPTXF[0], (((uint32_t)0x80 << 16) | (USB_OTG_FS->GRXFSIZ + (USB_OTG_FS->DIEPTXF0_HNPTXFSIZ >> 16))));
   return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_DeInit(USBD_HandleTypeDef *pdev)
{
   // Clear all pending interrupts and masks
   for (uint32_t i = 0; i < USB_NUM_ENDPOINTS; ++i)
   {
      WRITE_REG(in_endpoints[i]->DIEPINT, 0xFB7FU);
      WRITE_REG(out_endpoints[i]->DOEPINT, 0xFB7FU);
   }
   WRITE_REG(usb_device->DIEPMSK, 0U);
   WRITE_REG(usb_device->DOEPMSK, 0U);
   WRITE_REG(usb_device->DAINTMSK, 0U);

   // Flush the RX FIFO
   USB_LL_FlushRx();

   // Flush the TX TIFO
   USB_LL_FlushTx(0x10U);
   return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_Start(USBD_HandleTypeDef *pdev)
{
   // Un-gate and restore the PHY clock
   SET_BIT(USB_OTG_FS->GAHBCFG, USB_OTG_GAHBCFG_GINT);
   CLEAR_BIT(*(volatile uint32_t *)((uint32_t)USB_OTG_FS + USB_OTG_PCGCCTL_BASE), (USB_OTG_PCGCCTL_STOPCLK | USB_OTG_PCGCCTL_GATECLK));
   CLEAR_BIT(usb_device->DCTL, USB_OTG_DCTL_SDIS);
   return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_Stop(USBD_HandleTypeDef *pdev)
{
   // Un-gate and restore the PHY clock
   CLEAR_BIT(USB_OTG_FS->GAHBCFG, USB_OTG_GAHBCFG_GINT);
   CLEAR_BIT(*(volatile uint32_t *)((uint32_t)USB_OTG_FS + USB_OTG_PCGCCTL_BASE), (USB_OTG_PCGCCTL_STOPCLK | USB_OTG_PCGCCTL_GATECLK));
   SET_BIT(usb_device->DCTL, USB_OTG_DCTL_SDIS);

   // Flush the TX TIFO
   USB_LL_FlushTx(0x10U);
   return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_OpenEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr, uint8_t ep_type, uint16_t ep_mps)
{
   // Configure the specified endpoint
   USB_EPTypeDef *ep;
   if (READ_BIT(ep_addr, 0x80U))
   {
      ep = &in_endpoint_params[ep_addr & EP_ADDR_MSK];
      ep->is_in = 1;
   }
   else
   {
      ep = &out_endpoint_params[ep_addr & EP_ADDR_MSK];
      ep->is_in = 0;
   }
   ep->num = ep_addr & EP_ADDR_MSK;
   const uint32_t epnum = (uint32_t)ep->num;
   ep->maxpacket = (uint32_t)ep_mps & 0x7FFU;
   ep->type = ep_type;
   if (ep_type == EP_TYPE_BULK)
      ep->data_pid_start = 0;

   // Activate the endpoint
   if (ep->is_in)
   {
      ep->tx_fifo_num = ep->num;
      SET_BIT(usb_device->DAINTMSK, (USB_OTG_DAINTMSK_IEPM & (1UL << (epnum & EP_ADDR_MSK))));
      if (!READ_BIT(in_endpoints[epnum]->DIEPCTL, USB_OTG_DIEPCTL_USBAEP))
         SET_BIT(in_endpoints[epnum]->DIEPCTL, ((ep->maxpacket & USB_OTG_DIEPCTL_MPSIZ) | ((uint32_t)ep->type << 18) | (epnum << 22) | USB_OTG_DIEPCTL_SD0PID_SEVNFRM | USB_OTG_DIEPCTL_USBAEP));
   }
   else
   {
      SET_BIT(usb_device->DAINTMSK, (USB_OTG_DAINTMSK_OEPM & ((1UL << (epnum & EP_ADDR_MSK)) << 16)));
      if (!READ_BIT(out_endpoints[epnum]->DOEPCTL, USB_OTG_DOEPCTL_USBAEP))
         SET_BIT(out_endpoints[epnum]->DOEPCTL, ((ep->maxpacket & USB_OTG_DOEPCTL_MPSIZ) | ((uint32_t)ep->type << 18) | USB_OTG_DIEPCTL_SD0PID_SEVNFRM | USB_OTG_DOEPCTL_USBAEP));
   }
   return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_CloseEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
   // Configure the specified endpoint
   USB_EPTypeDef *ep;
   if (READ_BIT(ep_addr, 0x80U))
   {
      ep = &in_endpoint_params[ep_addr & EP_ADDR_MSK];
      ep->is_in = 1;
   }
   else
   {
      ep = &out_endpoint_params[ep_addr & EP_ADDR_MSK];
      ep->is_in = 0;
   }
   ep->num = ep_addr & EP_ADDR_MSK;

   // Deactivate the endpoint
   const uint32_t epnum = (uint32_t)ep->num;
   if (ep->is_in)
   {
      if (READ_BIT(in_endpoints[epnum]->DIEPCTL, USB_OTG_DIEPCTL_EPENA))
      {
         SET_BIT(in_endpoints[epnum]->DIEPCTL, USB_OTG_DIEPCTL_SNAK);
         SET_BIT(in_endpoints[epnum]->DIEPCTL, USB_OTG_DIEPCTL_EPDIS);
      }
      CLEAR_BIT(usb_device->DEACHMSK, (USB_OTG_DAINTMSK_IEPM & (1UL << (epnum & EP_ADDR_MSK))));
      CLEAR_BIT(usb_device->DAINTMSK, (USB_OTG_DAINTMSK_IEPM & (1UL << (epnum & EP_ADDR_MSK))));
      CLEAR_BIT(in_endpoints[epnum]->DIEPCTL, (USB_OTG_DIEPCTL_USBAEP | USB_OTG_DIEPCTL_MPSIZ | USB_OTG_DIEPCTL_TXFNUM | USB_OTG_DIEPCTL_SD0PID_SEVNFRM | USB_OTG_DIEPCTL_EPTYP));
   }
   else
   {
      if (READ_BIT(out_endpoints[epnum]->DOEPCTL, USB_OTG_DOEPCTL_EPENA))
      {
         SET_BIT(out_endpoints[epnum]->DOEPCTL, USB_OTG_DOEPCTL_SNAK);
         SET_BIT(out_endpoints[epnum]->DOEPCTL, USB_OTG_DOEPCTL_EPDIS);
      }
      CLEAR_BIT(usb_device->DEACHMSK, (USB_OTG_DAINTMSK_OEPM & ((1UL << (ep->num & EP_ADDR_MSK)) << 16)));
      CLEAR_BIT(usb_device->DAINTMSK, (USB_OTG_DAINTMSK_OEPM & ((1UL << (ep->num & EP_ADDR_MSK)) << 16)));
      CLEAR_BIT(out_endpoints[epnum]->DOEPCTL, (USB_OTG_DOEPCTL_USBAEP | USB_OTG_DOEPCTL_MPSIZ | USB_OTG_DOEPCTL_SD0PID_SEVNFRM | USB_OTG_DOEPCTL_EPTYP));
   }
   return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_FlushEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
   // Flush the correct indicated FIFO
   if (READ_BIT(ep_addr, 0x80U))
      USB_LL_FlushTx(((uint32_t)ep_addr & EP_ADDR_MSK) << 6);
   else
      USB_LL_FlushRx();
   return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_StallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
   // Configure the specified endpoint
   USB_EPTypeDef *ep;
   if (READ_BIT(ep_addr, 0x80U))
   {
      ep = &in_endpoint_params[ep_addr & EP_ADDR_MSK];
      ep->is_in = 1;
   }
   else
   {
      ep = &out_endpoint_params[ep_addr];
      ep->is_in = 0;
   }
   ep->is_stall = 1;
   ep->num = ep_addr & EP_ADDR_MSK;

   // Set up the stall condition
   const uint32_t epnum = (uint32_t)ep->num;
   if (ep->is_in)
   {
      if (!READ_BIT(in_endpoints[epnum]->DIEPCTL, USB_OTG_DIEPCTL_EPENA) && epnum)
         CLEAR_BIT(in_endpoints[epnum]->DIEPCTL, USB_OTG_DIEPCTL_EPDIS);
      SET_BIT(in_endpoints[epnum]->DIEPCTL, USB_OTG_DIEPCTL_STALL);
   }
   else
   {
      if (!READ_BIT(out_endpoints[epnum]->DOEPCTL, USB_OTG_DOEPCTL_EPENA) && epnum)
         CLEAR_BIT(out_endpoints[epnum]->DOEPCTL, USB_OTG_DOEPCTL_EPDIS);
      SET_BIT(out_endpoints[epnum]->DOEPCTL, USB_OTG_DOEPCTL_STALL);
   }
   if (!epnum)
      USB_LL_EP0_Write();
   return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_ClearStallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
   // Configure the specified endpoint
   USB_EPTypeDef *ep;
   if (READ_BIT(ep_addr, 0x80U))
   {
      ep = &in_endpoint_params[ep_addr & EP_ADDR_MSK];
      ep->is_in = 1;
   }
   else
   {
      ep = &out_endpoint_params[ep_addr & EP_ADDR_MSK];
      ep->is_in = 0;
   }
   ep->is_stall = 0;
   ep->num = ep_addr & EP_ADDR_MSK;

   // Clear the stall condition
   const uint32_t epnum = (uint32_t)ep->num;
   if (ep->is_in)
   {
      CLEAR_BIT(in_endpoints[epnum]->DIEPCTL, USB_OTG_DIEPCTL_STALL);
      if ((ep->type == EP_TYPE_INTR) || (ep->type == EP_TYPE_BULK))
         SET_BIT(in_endpoints[epnum]->DIEPCTL, USB_OTG_DIEPCTL_SD0PID_SEVNFRM);
   }
   else
   {
      CLEAR_BIT(out_endpoints[epnum]->DOEPCTL, USB_OTG_DOEPCTL_STALL);
      if ((ep->type == EP_TYPE_INTR) || (ep->type == EP_TYPE_BULK))
         SET_BIT(out_endpoints[epnum]->DOEPCTL, USB_OTG_DOEPCTL_SD0PID_SEVNFRM);
   }
   return USBD_OK;
}

uint8_t USBD_LL_IsStallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
   // Return whether the endpoint is currently stalled
   return READ_BIT(ep_addr, 0x80U) ? in_endpoint_params[ep_addr & 0x7F].is_stall : out_endpoint_params[ep_addr & 0x7F].is_stall;
}

USBD_StatusTypeDef USBD_LL_SetUSBAddress(USBD_HandleTypeDef *pdev, uint8_t dev_addr)
{
   // Set the desired USB address
   MODIFY_REG(usb_device->DCFG, USB_OTG_DCFG_DAD, (((uint32_t)dev_addr << 4) & USB_OTG_DCFG_DAD));
   return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_Transmit(USBD_HandleTypeDef *pdev, uint8_t ep_addr, uint8_t *pbuf, uint32_t size)
{
   // Initiate a transfer over USB
   USB_EPTypeDef *const ep = &in_endpoint_params[ep_addr & EP_ADDR_MSK];
   ep->xfer_buff = pbuf;
   ep->xfer_len = size;
   ep->xfer_count = 0;
   ep->is_in = 1;
   ep->num = ep_addr & EP_ADDR_MSK;
   ep->dma_addr = (uint32_t)pbuf;

   // Set up the transfer size and parameters
   USB_OTG_INEndpointTypeDef *const ep_in = in_endpoints[ep->num];
   if (!ep->xfer_len)
      MODIFY_REG(ep_in->DIEPTSIZ, (USB_OTG_DIEPTSIZ_PKTCNT | USB_OTG_DIEPTSIZ_XFRSIZ), (USB_OTG_DIEPTSIZ_PKTCNT & (1U << 19)));
   else
   {
      CLEAR_BIT(ep_in->DIEPTSIZ, (USB_OTG_DIEPTSIZ_PKTCNT | USB_OTG_DIEPTSIZ_XFRSIZ));
      if (!ep->num)
      {
         if (ep->xfer_len > ep->maxpacket)
            ep->xfer_len = ep->maxpacket;
         SET_BIT(ep_in->DIEPTSIZ, (USB_OTG_DIEPTSIZ_PKTCNT & (1U << 19)));
      }
      else
      {
         const uint16_t pktcnt = (uint16_t)((ep->xfer_len + ep->maxpacket - 1U) / ep->maxpacket);
         SET_BIT(ep_in->DIEPTSIZ, (USB_OTG_DIEPTSIZ_PKTCNT & ((uint32_t)pktcnt << 19)));
      }
      SET_BIT(ep_in->DIEPTSIZ, (USB_OTG_DIEPTSIZ_XFRSIZ & ep->xfer_len));
   }
   WRITE_REG(ep_in->DIEPDMA, ep->dma_addr);
   SET_BIT(ep_in->DIEPCTL, (USB_OTG_DIEPCTL_CNAK | USB_OTG_DIEPCTL_EPENA));
   return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_PrepareReceive(USBD_HandleTypeDef *pdev, uint8_t ep_addr, uint8_t *pbuf, uint32_t size)
{
   // Initiate a transfer over USB
   USB_EPTypeDef *const ep = &out_endpoint_params[ep_addr & EP_ADDR_MSK];
   ep->xfer_buff = pbuf;
   ep->xfer_len = size;
   ep->xfer_count = 0;
   ep->is_in = 0;
   ep->num = ep_addr & EP_ADDR_MSK;
   ep->dma_addr = (uint32_t)pbuf;

   // Set up the transfer size and parameters
   USB_OTG_OUTEndpointTypeDef *const ep_out = out_endpoints[ep->num];
   CLEAR_BIT(ep_out->DOEPTSIZ, (USB_OTG_DOEPTSIZ_XFRSIZ | USB_OTG_DOEPTSIZ_PKTCNT));
   if (!ep->num)
   {
      if (ep->xfer_len > 0U)
         ep->xfer_len = ep->maxpacket;
      ep->xfer_size = ep->maxpacket;
      SET_BIT(ep_out->DOEPTSIZ, ((USB_OTG_DOEPTSIZ_XFRSIZ & ep->xfer_size) | (USB_OTG_DOEPTSIZ_PKTCNT & (1U << 19))));
   }
   else
   {
      if (!ep->xfer_len)
         SET_BIT(ep_out->DOEPTSIZ, ((USB_OTG_DOEPTSIZ_XFRSIZ & ep->maxpacket) | (USB_OTG_DOEPTSIZ_PKTCNT & (1U << 19))));
      else
      {
         const uint16_t pktcnt = (uint16_t)((ep->xfer_len + ep->maxpacket - 1U) / ep->maxpacket);
         ep->xfer_size = ep->maxpacket * pktcnt;
         SET_BIT(ep_out->DOEPTSIZ, ((USB_OTG_DOEPTSIZ_PKTCNT & ((uint32_t)pktcnt << 19)) | (USB_OTG_DOEPTSIZ_XFRSIZ & ep->xfer_size)));
      }
   }
   WRITE_REG(ep_out->DOEPDMA, (uint32_t)ep->xfer_buff);
   SET_BIT(ep_out->DOEPCTL, (USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA));
   return USBD_OK;
}

uint32_t USBD_LL_GetRxDataSize(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
   // Return the amount of data ready to be read
   return out_endpoint_params[ep_addr & EP_ADDR_MSK].xfer_count;
}

void *USBD_static_malloc(uint32_t size)
{
   static uint32_t mem[(sizeof(USBD_CDC_HandleTypeDef)/4)+1];
   return mem;
}

void USBD_static_free(void *p) {}


// Private Helper Functions --------------------------------------------------------------------------------------------

static void int_to_unicode(uint32_t value, uint8_t *pbuf, uint8_t len)
{
   // Convert an integer value to a Unicode string
   for (uint8_t idx = 0; idx < len; ++idx)
   {
      pbuf[2*idx] = (((value >> 28)) < 0xA) ? ((value >> 28) + '0') : ((value >> 28) + 'A' - 10);
      pbuf[2*idx + 1] = 0;
      value <<= 4;
   }
}

static uint8_t* USB_DeviceDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
   // Return a pointer to the USB device descriptor
   *length = sizeof(usb_device_descriptor);
   return usb_device_descriptor;
}

static uint8_t* USB_LangIDStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
   // Return a pointer to the USB language descriptor
   *length = sizeof(usb_lang_descriptor);
   return usb_lang_descriptor;
}

static uint8_t* USB_ProductStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
   // Return a pointer to the USB string descriptor
   USBD_GetString((uint8_t*)USB_PRODUCT_STRING, usb_string_buffer, length);
   return usb_string_buffer;
}

static uint8_t* USB_ManufacturerStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
   // Return a pointer to the USB manufacturer descriptor
   USBD_GetString((uint8_t*)USB_MANUFACTURER_STRING, usb_string_buffer, length);
   return usb_string_buffer;
}

static uint8_t* USB_SerialStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
   // Return a pointer to the USB serial descriptor
   *length = USB_SIZE_STRING_SERIAL;
   uint32_t deviceserial0 = DEVICE_SERIAL0;
   uint32_t deviceserial1 = DEVICE_SERIAL1;
   uint32_t deviceserial2 = DEVICE_SERIAL2;
   deviceserial0 += deviceserial2;
   int_to_unicode(deviceserial0, &usb_serial_buffer[2], 8);
   int_to_unicode(deviceserial1, &usb_serial_buffer[18], 4);
   return (uint8_t*)usb_serial_buffer;
}

static uint8_t* USB_ConfigStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
   // Return a pointer to the USB configuration string descriptor
   USBD_GetString((uint8_t *)USB_CONFIGURATION_STRING, usb_string_buffer, length);
   return usb_string_buffer;
}

static uint8_t* USB_InterfaceStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
   // Return a pointer to the USB interface string descriptor
   USBD_GetString((uint8_t *)USB_INTERFACE_STRING, usb_string_buffer, length);
   return usb_string_buffer;
}

static int8_t CDC_Init(void)
{
   // Set up the USB reception buffer
   USBD_CDC_SetRxBuffer(&usb_cdc_device, usb_receive_buffer);
   return USBD_OK;
}

static int8_t CDC_DeInit(void)
{
   // Should never need to de-initialize
   return USBD_OK;
}

static int8_t CDC_Control(uint8_t cmd, uint8_t *pbuf, uint16_t length)
{
   // Handle the USB control command based on its type
   switch (cmd)
   {
      case CDC_SET_LINE_CODING:
         usb_line_coding.bitrate = (uint32_t)(pbuf[0] | (pbuf[1] << 8) | (pbuf[2] << 16) | (pbuf[3] << 24));
         usb_line_coding.format = pbuf[4];
         usb_line_coding.paritytype = pbuf[5];
         usb_line_coding.datatype = pbuf[6];
         break;
      case CDC_GET_LINE_CODING:
         pbuf[0] = (uint8_t)(usb_line_coding.bitrate);
         pbuf[1] = (uint8_t)(usb_line_coding.bitrate >> 8);
         pbuf[2] = (uint8_t)(usb_line_coding.bitrate >> 16);
         pbuf[3] = (uint8_t)(usb_line_coding.bitrate >> 24);
         pbuf[4] = usb_line_coding.format;
         pbuf[5] = usb_line_coding.paritytype;
         pbuf[6] = usb_line_coding.datatype;
         break;
      default:
         break;
   }
   return USBD_OK;
}

static int8_t CDC_Receive(uint8_t *pbuf, uint32_t *len)
{
   // Prepare to receive a USB packet
   // TODO: Can we get rid of setting the Rx buffer here, just use one global one?
   USBD_CDC_SetRxBuffer(&usb_cdc_device, pbuf);
   USBD_CDC_ReceivePacket(&usb_cdc_device);
   return USBD_OK;
}

static uint8_t CDC_Transmit(uint8_t *pbuf, uint16_t len)
{
   // Prepare to transmit the specified USB packet
   USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)usb_cdc_device.pClassData;
   if (hcdc->TxState)
      return USBD_BUSY;
   USBD_CDC_SetTxBuffer(&usb_cdc_device, pbuf, len);
   return USBD_CDC_TransmitPacket(&usb_cdc_device);
}

static int8_t CDC_TransmitCplt(uint8_t *pbuf, uint32_t *len, uint8_t epnum)
{
   // Nothing to do upon completion of a transmission
   return USBD_OK;
}

static void usb_set_led_status(uint8_t on)
{
   // Set the USB status LED to the requested state
   LED_USB_STATUS_GPIO_Port->BSRR = on ? LED_USB_STATUS_Pin : ((uint32_t)LED_USB_STATUS_Pin << 16U);
}


// Interrupt Service Routines ------------------------------------------------------------------------------------------

void OTG_FS_IRQHandler(void)
{
   // Ensure that this was a valid interrupt
   if (READ_BIT(USB_OTG_FS->GINTSTS, USB_OTG_FS->GINTMSK))
   {
      if (READ_BIT(USB_OTG_FS->GINTSTS, (USB_OTG_FS->GINTMSK & USB_OTG_GINTSTS_MMIS)))
         CLEAR_FLAG(USB_OTG_FS->GINTSTS, USB_OTG_GINTSTS_MMIS);
      if (READ_BIT(USB_OTG_FS->GINTSTS, (USB_OTG_FS->GINTMSK & USB_OTG_GINTSTS_RXFLVL)))
      {
         CLEAR_BIT(USB_OTG_FS->GINTMSK, USB_OTG_GINTSTS_RXFLVL);
         const uint32_t reg_val = USB_OTG_FS->GRXSTSP;
         USB_EPTypeDef *const ep = &out_endpoint_params[reg_val & USB_OTG_GRXSTSP_EPNUM];
         if (((reg_val & USB_OTG_GRXSTSP_PKTSTS) >> 17) == STS_DATA_UPDT)
         {
            if (READ_BIT(reg_val, USB_OTG_GRXSTSP_BCNT))
            {
               USB_LL_Read(ep->xfer_buff, (reg_val & USB_OTG_GRXSTSP_BCNT) >> 4);
               ep->xfer_buff += (reg_val & USB_OTG_GRXSTSP_BCNT) >> 4;
               ep->xfer_count += (reg_val & USB_OTG_GRXSTSP_BCNT) >> 4;
            }
         }
         else if (((reg_val & USB_OTG_GRXSTSP_PKTSTS) >> 17) == STS_SETUP_UPDT)
         {
            USB_LL_Read((uint8_t*)setup_buffer, 8U);
            ep->xfer_count += (reg_val & USB_OTG_GRXSTSP_BCNT) >> 4;
         }
         SET_BIT(USB_OTG_FS->GINTMSK, USB_OTG_GINTSTS_RXFLVL);
      }
      if (READ_BIT(USB_OTG_FS->GINTSTS, (USB_OTG_FS->GINTMSK & USB_OTG_GINTSTS_OEPINT)))
      {
         for (uint32_t epnum = 0, ep_intr = (usb_device->DAINT & usb_device->DAINTMSK & 0xFFFF0000U) >> 16; ep_intr; ep_intr >>= 1U, ++epnum)
         {
            if (READ_BIT(ep_intr, 0x1U))
            {
               USB_OTG_OUTEndpointTypeDef *const ep_out = out_endpoints[epnum];
               const uint32_t epint = READ_BIT(ep_out->DOEPINT, usb_device->DOEPMSK);
               if (READ_BIT(epint, USB_OTG_DOEPINT_XFRC))
               {
                  WRITE_REG(ep_out->DOEPINT, USB_OTG_DOEPINT_XFRC);
                  const uint32_t int_reg = ep_out->DOEPINT;
                  if (READ_BIT(int_reg, USB_OTG_DOEPINT_STUP))
                  {
                     if (READ_BIT(int_reg, USB_OTG_DOEPINT_STPKTRX))
                        WRITE_REG(ep_out->DOEPINT, USB_OTG_DOEPINT_STPKTRX);
                  }
                  else if (READ_BIT(int_reg, USB_OTG_DOEPINT_OTEPSPR))
                     WRITE_REG(ep_out->DOEPINT, USB_OTG_DOEPINT_OTEPSPR);
                  else if (!READ_BIT(int_reg, (USB_OTG_DOEPINT_STUP | USB_OTG_DOEPINT_OTEPSPR)))
                  {
                     if (READ_BIT(int_reg, USB_OTG_DOEPINT_STPKTRX))
                        WRITE_REG(ep_out->DOEPINT, USB_OTG_DOEPINT_STPKTRX);
                     else
                     {
                        USB_EPTypeDef *const ep = &out_endpoint_params[epnum];
                        ep->xfer_count = ep->xfer_size - READ_BIT(ep_out->DOEPTSIZ, USB_OTG_DOEPTSIZ_XFRSIZ);
                        if (!epnum)
                        {
                           if (!ep->xfer_len)
                              USB_LL_EP0_Write();
                           else
                              ep->xfer_buff += ep->xfer_count;
                        }
                        USBD_LL_DataOutStage(&usb_cdc_device, (uint8_t)epnum, out_endpoint_params[epnum].xfer_buff);
                     }
                  }
               }
               if (READ_BIT(epint, USB_OTG_DOEPINT_STUP))
               {
                  WRITE_REG(ep_out->DOEPINT, USB_OTG_DOEPINT_STUP);
                  if (READ_BIT(ep_out->DOEPINT, USB_OTG_DOEPINT_STPKTRX))
                     WRITE_REG(ep_out->DOEPINT, USB_OTG_DOEPINT_STPKTRX);
                  USBD_LL_SetupStage(&usb_cdc_device, (uint8_t*)setup_buffer);
                  USB_LL_EP0_Write();
               }
               if (READ_BIT(epint, USB_OTG_DOEPINT_OTEPDIS))
                  WRITE_REG(ep_out->DOEPINT, USB_OTG_DOEPINT_OTEPDIS);
               if (READ_BIT(epint, USB_OTG_DOEPINT_EPDISD))
               {
                  if (READ_BIT(USB_OTG_FS->GINTSTS, USB_OTG_GINTSTS_BOUTNAKEFF))
                     SET_BIT(usb_device->DCTL, USB_OTG_DCTL_CGONAK);
                  WRITE_REG(ep_out->DOEPINT, USB_OTG_DOEPINT_EPDISD);
               }
               if (READ_BIT(epint, USB_OTG_DOEPINT_OTEPSPR))
                  WRITE_REG(ep_out->DOEPINT, USB_OTG_DOEPINT_OTEPSPR);
               if (READ_BIT(epint, USB_OTG_DOEPINT_NAK))
                  WRITE_REG(ep_out->DOEPINT, USB_OTG_DOEPINT_NAK);
            }
         }
      }
      if (READ_BIT(USB_OTG_FS->GINTSTS, (USB_OTG_FS->GINTMSK & USB_OTG_GINTSTS_IEPINT)))
      {
         for (uint32_t epnum = 0, ep_intr = usb_device->DAINT & usb_device->DAINTMSK & 0xFFFFU; ep_intr; ep_intr >>= 1U, ++epnum)
         {
            if (READ_BIT(ep_intr, 0x1U))
            {USB_OTG_INEndpointTypeDef *const ep_in = in_endpoints[epnum];
               const uint32_t epint = READ_BIT(ep_in->DIEPINT, (usb_device->DIEPMSK | (((usb_device->DIEPEMPMSK >> (epnum & EP_ADDR_MSK)) & 0x1U) << 7)));
               if (READ_BIT(epint, USB_OTG_DIEPINT_XFRC))
               {
                  CLEAR_BIT(usb_device->DIEPEMPMSK, (uint32_t)(0x1UL << (epnum & EP_ADDR_MSK)));
                  WRITE_REG(ep_in->DIEPINT, USB_OTG_DIEPINT_XFRC);
                  in_endpoint_params[epnum].xfer_buff += in_endpoint_params[epnum].maxpacket;
                  if (!epnum && !in_endpoint_params[epnum].xfer_len)
                     USB_LL_EP0_Write();
                  USBD_LL_DataInStage(&usb_cdc_device, (uint8_t)epnum, in_endpoint_params[epnum].xfer_buff);
               }
               if (READ_BIT(epint, USB_OTG_DIEPINT_TOC))
                  WRITE_REG(ep_in->DIEPINT, USB_OTG_DIEPINT_TOC);
               if (READ_BIT(epint, USB_OTG_DIEPINT_ITTXFE))
                  WRITE_REG(ep_in->DIEPINT, USB_OTG_DIEPINT_ITTXFE);
               if (READ_BIT(epint, USB_OTG_DIEPINT_INEPNE))
                  WRITE_REG(ep_in->DIEPINT, USB_OTG_DIEPINT_INEPNE);
               if (READ_BIT(epint, USB_OTG_DIEPINT_EPDISD))
               {
                  USB_LL_FlushTx(epnum);
                  WRITE_REG(ep_in->DIEPINT, USB_OTG_DIEPINT_EPDISD);
               }
               if (READ_BIT(epint, USB_OTG_DIEPINT_TXFE))
               {
                  USB_EPTypeDef *const ep = &in_endpoint_params[epnum];
                  uint32_t len = ep->xfer_len - ep->xfer_count;
                  if (len > ep->maxpacket)
                    len = ep->maxpacket;
                  uint32_t len32b = (len + 3U) / 4U;
                  while ((READ_BIT(ep_in->DTXFSTS, USB_OTG_DTXFSTS_INEPTFSAV) >= len32b) && (ep->xfer_count < ep->xfer_len) && ep->xfer_len)
                  {
                     len = ep->xfer_len - ep->xfer_count;
                     if (len > ep->maxpacket)
                        len = ep->maxpacket;
                     len32b = (len + 3U) / 4U;
                     ep->xfer_buff += len;
                     ep->xfer_count += len;
                  }
                  if (ep->xfer_len <= ep->xfer_count)
                     CLEAR_BIT(usb_device->DIEPEMPMSK, (0x1UL << (epnum & EP_ADDR_MSK)));
               }
            }
         }
      }
      if (READ_BIT(USB_OTG_FS->GINTSTS, (USB_OTG_FS->GINTMSK & USB_OTG_GINTSTS_WKUINT)))
      {
         CLEAR_BIT(usb_device->DCTL, USB_OTG_DCTL_RWUSIG);
         USBD_LL_Resume(&usb_cdc_device);
         CLEAR_FLAG(USB_OTG_FS->GINTSTS, USB_OTG_GINTSTS_WKUINT);
      }
      if (READ_BIT(USB_OTG_FS->GINTSTS, (USB_OTG_FS->GINTMSK & USB_OTG_GINTSTS_USBSUSP)))
      {
         if (READ_BIT(usb_device->DSTS, USB_OTG_DSTS_SUSPSTS))
         {
            USBD_LL_Suspend(&usb_cdc_device);
            SET_BIT(*(volatile uint32_t *)((uint32_t)USB_OTG_FS + USB_OTG_PCGCCTL_BASE), USB_OTG_PCGCCTL_STOPCLK);
         }
         CLEAR_FLAG(USB_OTG_FS->GINTSTS, USB_OTG_GINTSTS_USBSUSP);
      }
      if (READ_BIT(USB_OTG_FS->GINTSTS, (USB_OTG_FS->GINTMSK & USB_OTG_GINTSTS_USBRST)))
      {
         CLEAR_BIT(usb_device->DCTL, USB_OTG_DCTL_RWUSIG);
         USB_LL_FlushTx(0x10U);
         for (uint32_t i = 0; i < USB_NUM_ENDPOINTS; ++i)
         {
            WRITE_REG(in_endpoints[i]->DIEPINT, 0xFB7FU);
            CLEAR_BIT(in_endpoints[i]->DIEPCTL, USB_OTG_DIEPCTL_STALL);
            WRITE_REG(out_endpoints[i]->DOEPINT, 0xFB7FU);
            CLEAR_BIT(out_endpoints[i]->DOEPCTL, USB_OTG_DOEPCTL_STALL);
            SET_BIT(out_endpoints[i]->DOEPCTL, USB_OTG_DOEPCTL_SNAK);
         }
         SET_BIT(usb_device->DAINTMSK, 0x10001U);
         SET_BIT(usb_device->DOEPMSK, (USB_OTG_DOEPMSK_STUPM | USB_OTG_DOEPMSK_XFRCM | USB_OTG_DOEPMSK_EPDM | USB_OTG_DOEPMSK_OTEPSPRM | USB_OTG_DOEPMSK_NAKM));
         SET_BIT(usb_device->DIEPMSK, (USB_OTG_DIEPMSK_TOM | USB_OTG_DIEPMSK_XFRCM | USB_OTG_DIEPMSK_EPDM));
         CLEAR_BIT(usb_device->DCFG, USB_OTG_DCFG_DAD);
         USB_LL_EP0_Write();
         CLEAR_FLAG(USB_OTG_FS->GINTSTS, USB_OTG_GINTSTS_USBRST);
      }
      if (READ_BIT(USB_OTG_FS->GINTSTS, (USB_OTG_FS->GINTMSK & USB_OTG_GINTSTS_ENUMDNE)))
      {
         CLEAR_BIT(in_endpoints[0]->DIEPCTL, USB_OTG_DIEPCTL_MPSIZ);
         SET_BIT(usb_device->DCTL, USB_OTG_DCTL_CGINAK);
         MODIFY_REG(USB_OTG_FS->GUSBCFG, USB_OTG_GUSBCFG_TRDT, (uint32_t)((0x6U << 10) & USB_OTG_GUSBCFG_TRDT));
         USBD_LL_SetSpeed(&usb_cdc_device, USBD_SPEED_FULL);
         USBD_LL_Reset(&usb_cdc_device);
         CLEAR_FLAG(USB_OTG_FS->GINTSTS, USB_OTG_GINTSTS_ENUMDNE);
      }
      if (READ_BIT(USB_OTG_FS->GINTSTS, (USB_OTG_FS->GINTMSK & USB_OTG_GINTSTS_BOUTNAKEFF)))
         CLEAR_BIT(USB_OTG_FS->GINTMSK, USB_OTG_GINTMSK_GONAKEFFM);
      if (READ_BIT(USB_OTG_FS->GINTSTS, (USB_OTG_FS->GINTMSK & USB_OTG_GINTSTS_SRQINT)))
      {
         USBD_LL_DevConnected(&usb_cdc_device);
         CLEAR_FLAG(USB_OTG_FS->GINTSTS, USB_OTG_GINTSTS_SRQINT);
      }
      if (READ_BIT(USB_OTG_FS->GINTSTS, (USB_OTG_FS->GINTMSK & USB_OTG_GINTSTS_OTGINT)))
      {
         const uint32_t reg_val = USB_OTG_FS->GOTGINT;
         if (READ_BIT(reg_val, USB_OTG_GOTGINT_SEDET))
            USBD_LL_DevDisconnected(&usb_cdc_device);
         SET_BIT(USB_OTG_FS->GOTGINT, reg_val);
      }
   }
}


// Public API Functions ------------------------------------------------------------------------------------------------

void usb_init(void)
{
   // Initialize the various GPIO clocks
   SET_BIT(RCC->AHB4ENR, RCC_AHB4ENR_GPIOAEN);
   (void)READ_BIT(RCC->AHB4ENR, RCC_AHB4ENR_GPIOAEN);
   SET_BIT(RCC->AHB4ENR, RCC_AHB4ENR_GPIODEN);
   (void)READ_BIT(RCC->AHB4ENR, RCC_AHB4ENR_GPIODEN);
   MODIFY_REG(RCC->AHB1LPENR, RCC_AHB1LPENR_USB2OTGHSULPILPEN, RCC_AHB1LPENR_USB2OTGHSLPEN);

   // Initialize the non-peripheral GPIO pins
   LED_USB_STATUS_GPIO_Port->BSRR = (uint32_t)LED_USB_STATUS_Pin << 16U;
   uint32_t position = 32 - __builtin_clz(LED_USB_STATUS_Pin) - 1;
   MODIFY_REG(LED_USB_STATUS_GPIO_Port->OSPEEDR, (GPIO_OSPEEDR_OSPEED0 << (position * 2U)), (GPIO_SPEED_FREQ_LOW << (position * 2U)));
   MODIFY_REG(LED_USB_STATUS_GPIO_Port->OTYPER, (GPIO_OTYPER_OT0 << position), (((GPIO_MODE_OUTPUT_PP & OUTPUT_TYPE) >> OUTPUT_TYPE_Pos) << position));
   CLEAR_BIT(LED_USB_STATUS_GPIO_Port->PUPDR, (GPIO_PUPDR_PUPD0 << (position * 2U)));
   MODIFY_REG(LED_USB_STATUS_GPIO_Port->MODER, (GPIO_MODER_MODE0 << (position * 2U)), ((GPIO_MODE_OUTPUT_PP & GPIO_MODE) << (position * 2U)));

   // Initialize the USB GPIO pins
   position = 32 - __builtin_clz(USB_DM_Pin) - 1;
   MODIFY_REG(USB_DM_GPIO_Port->OSPEEDR, (GPIO_OSPEEDR_OSPEED0 << (position * 2U)), (GPIO_SPEED_FREQ_VERY_HIGH << (position * 2U)));
   MODIFY_REG(USB_DM_GPIO_Port->OTYPER, (GPIO_OTYPER_OT0 << position), (((GPIO_MODE_AF_PP & OUTPUT_TYPE) >> OUTPUT_TYPE_Pos) << position));
   CLEAR_BIT(USB_DM_GPIO_Port->PUPDR, (GPIO_PUPDR_PUPD0 << (position * 2U)));
   MODIFY_REG(USB_DM_GPIO_Port->AFR[position >> 3U], (0xFU << ((position & 0x07U) * 4U)), (GPIO_AF10_OTG1_FS << ((position & 0x07U) * 4U)));
   MODIFY_REG(USB_DM_GPIO_Port->MODER, (GPIO_MODER_MODE0 << (position * 2U)), ((GPIO_MODE_AF_PP & GPIO_MODE) << (position * 2U)));
   position = 32 - __builtin_clz(USB_DP_Pin) - 1;
   MODIFY_REG(USB_DP_GPIO_Port->OSPEEDR, (GPIO_OSPEEDR_OSPEED0 << (position * 2U)), (GPIO_SPEED_FREQ_VERY_HIGH << (position * 2U)));
   MODIFY_REG(USB_DP_GPIO_Port->OTYPER, (GPIO_OTYPER_OT0 << position), (((GPIO_MODE_AF_PP & OUTPUT_TYPE) >> OUTPUT_TYPE_Pos) << position));
   CLEAR_BIT(USB_DP_GPIO_Port->PUPDR, (GPIO_PUPDR_PUPD0 << (position * 2U)));
   MODIFY_REG(USB_DP_GPIO_Port->AFR[position >> 3U], (0xFU << ((position & 0x07U) * 4U)), (GPIO_AF10_OTG1_FS << ((position & 0x07U) * 4U)));
   MODIFY_REG(USB_DP_GPIO_Port->MODER, (GPIO_MODER_MODE0 << (position * 2U)), ((GPIO_MODE_AF_PP & GPIO_MODE) << (position * 2U)));

   // Initialize the USB peripheral clock and interrupts
   SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_USB2OTGHSEN);
   (void)READ_BIT(RCC->AHB1ENR, RCC_AHB1ENR_USB2OTGHSEN);
   NVIC_SetPriority(OTG_FS_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
   NVIC_EnableIRQ(OTG_FS_IRQn);

   // Initialize the USB device middleware
   USBD_Init(&usb_cdc_device, &usb_descriptors, DEVICE_FS);
   USBD_RegisterClass(&usb_cdc_device, &USBD_CDC);
   USBD_CDC_RegisterInterface(&usb_cdc_device, &usb_interface_ops);
   USBD_Start(&usb_cdc_device);

   // Turn on the internal regulator to provide power to our external USB interface
   SET_BIT(PWR->CR3, PWR_CR3_USBREGEN);
   while (!READ_BIT(PWR->CR3, PWR_CR3_USB33RDY));
   SET_BIT(PWR->CR3, PWR_CR3_USB33DEN);
}

void usb_send(uint8_t* data, uint16_t data_length)
{
   usb_set_led_status(CDC_Transmit(data, data_length) == USBD_OK);
}

#endif  // #ifdef CORE_CM4
