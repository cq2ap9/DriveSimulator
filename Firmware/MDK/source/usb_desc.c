/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : usb_desc.c
* Author             : MCD Application Team
* Version            : V2.2.0
* Date               : 06/13/2008
* Description        : Descriptors for Joystick Mouse Demo
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "usb_lib.h"
#include "usb_desc.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Extern variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/* USB Standard Device Descriptor */
const u8 Joystick_DeviceDescriptor[JOYSTICK_SIZ_DEVICE_DESC] =
  {
    0x12,                       /*bLength */
    USB_DEVICE_DESCRIPTOR_TYPE, /*bDescriptorType*/
    0x00,                       /*bcdUSB */
    0x02,
    0x00,                       /*bDeviceClass*/
    0x00,                       /*bDeviceSubClass*/
    0x00,                       /*bDeviceProtocol*/
    0x40,                       /*bMaxPacketSize40*/
    0x43,                       /*idVendor (0x0443)*/
    0x04,
    0x43,                       /*idProduct = 0x5743*/
    0x57,
    0x00,                       /*bcdDevice rel. 2.00*/
    0x01,
    1,                          /*Index of string descriptor describing
                                                  manufacturer */
    2,                          /*Index of string descriptor describing
                                                 product*/
    3,                          /*Index of string descriptor describing the
                                                 device serial number */
    0x01                        /*bNumConfigurations*/
  }
  ; /* Joystick_DeviceDescriptor */


/* USB Configuration Descriptor */
/*   All Descriptors (Configuration, Interface, Endpoint, Class, Vendor */
const u8 Joystick_ConfigDescriptor[JOYSTICK_SIZ_CONFIG_DESC] =
  {
    0x09, /* bLength: Configuation Descriptor size */
    USB_CONFIGURATION_DESCRIPTOR_TYPE, /* bDescriptorType: Configuration */
    JOYSTICK_SIZ_CONFIG_DESC,
    /* wTotalLength: Bytes returned */
    0x00,
    0x01,         /*bNumInterfaces: 1 interface*/
    0x01,         /*bConfigurationValue: Configuration value*/
    0x00,         /*iConfiguration: Index of string descriptor describing
                                     the configuration*/
    0xE0,         /*bmAttributes: bus powered */
    0x32,         /*MaxPower 100 mA: this current is used for detecting Vbus*/

    /************** Descriptor of Joystick Mouse interface ****************/
    /* 09 */
    0x09,         /*bLength: Interface Descriptor size*/
    USB_INTERFACE_DESCRIPTOR_TYPE,/*bDescriptorType: Interface descriptor type*/
    0x00,         /*bInterfaceNumber: Number of Interface*/
    0x00,         /*bAlternateSetting: Alternate setting*/
    0x02,         /*bNumEndpoints2一个收一个发*/	 
    0x03,         /*bInterfaceClass: HID*/
    0x00,         /*bInterfaceSubClass : 1=BOOT, 0=no boot*/
    0x00,         /*nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse*/
    0x00,         /*iInterface: Index of string descriptor*/
    /******************** Descriptor of Joystick Mouse HID ********************/
    /* 18 */
    0x09,         /*bLength: HID Descriptor size*/
    HID_DESCRIPTOR_TYPE, /*bDescriptorType: HID*/
    0x00,         /*bcdHID: HID Class Spec release number*/
    0x01,
    0x00,         /*bCountryCode: Hardware target country*/
    0x01,         /*bNumDescriptors: Number of HID class descriptors to follow*/
    0x22,         /*bDescriptorType*/
    JOYSTICK_SIZ_REPORT_DESC,/*wItemLength: Total length of Report descriptor*/
    0x00,
    /******************** Descriptor of Joystick Mouse endpoint ********************/
    /* 27 *输入端点 */
    0x07,          /*bLength: Endpoint Descriptor size*/
    USB_ENDPOINT_DESCRIPTOR_TYPE, /*bDescriptorType:*/

    0x81,          /*bEndpointAddress: Endpoint Address (IN)*/
    0x03,          /*bmAttributes: Interrupt endpoint*/
    JOYSTICK_ENDP_MaxPacketSize, /*wMaxPacketSize: 7 Byte max */
    0x00,
    0x20,          /*bInterval: Polling Interval (32 ms)*/
    /* 34 *输出端点 */
	0x07,	/* bLength: Endpoint Descriptor size */
    USB_ENDPOINT_DESCRIPTOR_TYPE,	/* bDescriptorType: */
			/*	Endpoint descriptor type */
    0x01,	/* bEndpointAddress: */
			/*	Endpoint Address (OUT) */
    0x03,	/* bmAttributes: Interrupt endpoint */
    0x02,	/* wMaxPacketSize: 2 Bytes max  */
    0x00,
    0x20,	/* bInterval: Polling Interval (20 ms) */
    /* 41 */
  }
  ; /* MOUSE_ConfigDescriptor */
const u8 Joystick_ReportDescriptor[JOYSTICK_SIZ_REPORT_DESC] =
  {
		0x05,
		0x01,	//	USAGE_PAGE (Generic Desktop)
		0x09,
		0x04,	//	USAGE (Joystick)
		0xa1,
		0x01,	//	COLLECTION (Application)
		0xa1,
		0x01,	//	COLLECTION (Application)
		0x05,
		0x01,	//	USAGE_PAGE (Generic Desktop)
		0x09,
		0x01,	//	USAGE (Pointer)
		0xa1,
		0x00,	//	COLLECTION (Physical)
		0x05,
		0x01,	//	USAGE_PAGE (Generic Desktop)
		0x09,
		0x30,	//	USAGE (X)         					<- 8-bit X轴 方向盘
		0x09,
		0x31,	//	USAGE (Y)         					<- 8-bit Y轴 离合器
		0x09,
		0x32,	//	USAGE (Z)         					<- 8-bit Z轴 刹车
		0x09,
		0x35,	//	USAGE (Rz)         					<- 8-bit RZ轴 油门
		0x15,
		0x00,	//	LOGICAL_MINIMUM (0)
		0x26,
		0xff,
		0x00,	//	LOGICAL_MAXIMUM (255)
		0x75,
		0x08,	//	REPORT_SIZE (8)
		0x95,
		0x04,	//	REPORT_COUNT (4)
		0x81,
		0x02,	//	INPUT (Data,Var,Abs)         <- 8-bit axis * 4
		0x05,
		0x09,	//	USAGE_PAGE (Button)
		0x19,
		0x01,	//	USAGE_MINIMUM (Button 1)
		0x29,
		0x80,	//	USAGE_MAXIMUM (Button 20)
		0x15,
		0x00,	//	LOGICAL_MINIMUM (0)
		0x25,
		0x01,	//	LOGICAL_MAXIMUM (1)
		0x75,
		0x01,	//	REPORT_SIZE (1)
		0x95,
		0x14,	//	REPORT_COUNT (20)
		0x81,
		0x02,	//	INPUT (Data,Var,Abs)         <- Buttons * 20
		0x75,
		0x01,	//	REPORT_SIZE (1)
		0x95,
		0x04,	//	REPORT_COUNT (4)
		0x81,
		0x03,	//	INPUT (Cnst,Var,Abs)         <- 填充位 20+4=3*8
		0xc0,	//	END_COLLECTION
		0xc0,	//	END_COLLECTION
		0x06, 
		0x00, 
		0xFF,	//	USAGE_PAGE (Vendor Defined Page 1)  
		0x09, 
		0x01,	//	USAGE (Vendor Usage 1) 
		0x15,
		0x00,	//	LOGICAL_MINIMUM (0)
		0x26,
		0xff,
		0x00,	//	LOGICAL_MAXIMUM (255)
		0x75,
		0x08,	//	REPORT_SIZE (8)
		0x95,
		0x02,	//	REPORT_COUNT (2)               <- 64 OUTPUT bytes
		0x91,
		0x02,	//	OUTPUT (Data,Var,Abs)           <- And it ends here.
		0xc0	//	END_COLLECTION
  }
  ; /* Joystick_ReportDescriptor */

/* USB String Descriptors (optional) */
const u8 Joystick_StringLangID[JOYSTICK_SIZ_STRING_LANGID] =
  {
    JOYSTICK_SIZ_STRING_LANGID,
    USB_STRING_DESCRIPTOR_TYPE,
    0x09,
    0x04
  }
  ; /* LangID = 0x0409: U.S. English */

const u8 Joystick_StringVendor[JOYSTICK_SIZ_STRING_VENDOR] =
  {
    JOYSTICK_SIZ_STRING_VENDOR, /* Size of Vendor string */
    USB_STRING_DESCRIPTOR_TYPE,  /* bDescriptorType*/
    /* Manufacturer: "STMicroelectronics" */
    'S', 0, 'T', 0, 'M', 0, 'i', 0, 'c', 0, 'r', 0, 'o', 0, 'e', 0,
    'l', 0, 'e', 0, 'c', 0, 't', 0, 'r', 0, 'o', 0, 'n', 0, 'i', 0,
    'c', 0, 's', 0
  };

const u8 Joystick_StringProduct[JOYSTICK_SIZ_STRING_PRODUCT] =
  {
    JOYSTICK_SIZ_STRING_PRODUCT,          /* bLength */
    USB_STRING_DESCRIPTOR_TYPE,        /* bDescriptorType */
    'D', 0, 'r', 0, 'i', 0, 'v', 0, 'e', 0, ' ', 0, 'S', 0,
    'i', 0, 'm', 0, 'u', 0, 'l', 0, 'a', 0, 't', 0, 'o', 0,
		'r',0
  };
u8 Joystick_StringSerial[JOYSTICK_SIZ_STRING_SERIAL] =
  {
    JOYSTICK_SIZ_STRING_SERIAL,           /* bLength */
    USB_STRING_DESCRIPTOR_TYPE,        /* bDescriptorType */
    'S', 0, 'T', 0, 'M', 0, '3', 0, '2', 0, '1', 0, '0', 0
  };

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/

