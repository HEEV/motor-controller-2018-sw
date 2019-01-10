#include <stm32f3xx_hal_usart.h>
// from USB_DEVICE/App directory
#include <usbd_cdc_if.h>
// USART function pointer
typedef void (*USARTIface_tx)(USART_HandleTypeDef* iface, uint8_t* buff, uint16_t len);
typedef void (*USARTIface_rx)(USART_HandleTypeDef* iface, uint8_t* buff, uint16_t len);
// USB CDC function pointer
typedef void (*USBIface_tx)(uint8_t* buff, uint16_t len);
typedef void (*USBIface_rx)(uint8_t* buff, uint16_t* len);
