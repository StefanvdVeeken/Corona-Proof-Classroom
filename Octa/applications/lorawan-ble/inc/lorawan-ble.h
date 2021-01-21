#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "platform.h"

struct UartMessage {
  uint8_t b1, b2, b3, b4;
};

osThreadId defaultTaskHandle;
osThreadId murata_rx_processing_handle;
osThreadId uart_rx_processing_handle;
osTimerId iwdgTimId;
osTimerId loraWANTimId;
osTimerId moduleCheckTimId;
osMutexId txMutexId;
osMutexId murata_rx_process_mutex_id;
osMutexId uart_rx_process_mutex_id;

void IWDG_feed(void const *argument);
void LoRaWAN_send();
void check_modules(void const *argument);
void murata_process_rx_response(void const *argument);
void process_rx_uart(void);
void Dualstack_ApplicationCallback(void);
void UART_BLE_Callback(void);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */