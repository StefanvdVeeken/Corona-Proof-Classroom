#include "lorawan-ble.h"
#include "murata.h"

#define IWDG_INTERVAL           5    //seconds
#define LORAWAN_INTERVAL        90   //seconds
#define MODULE_CHECK_INTERVAL   3600 //seconds
#define NUMBER_OF_SENSORS       4
#define BUFFER_LENGTH           6

uint16_t LoRaWAN_Counter = 0;
uint8_t murata_init = 0;
uint64_t short_UID;
uint8_t murata_data_ready = 0;

uint8_t uartSensorData[BUFFER_LENGTH]; // 6 bytes = length of 1 complete message
struct UartMessage allMessages[NUMBER_OF_SENSORS]; // 5 should be the total number of different sensors this octa can receive data from
uint8_t messagesIndex = 0;
volatile uint8_t data_ready = 0;
uint8_t recentData = 0;

int main(void)
{
  Initialize_Platform();
  printINF("before start\r\n");
  // LORAWAN
  murata_init = Murata_Initialize(short_UID);
  UART_SetApplicationCallback(&Dualstack_ApplicationCallback, (uint8_t)MURATA_CONNECTOR);

  //BLE
  UART_BLE_SetRxCallback(UART_BLE_Callback);
  HAL_UART_Receive_IT(&BLE_UART, &uartSensorData, BUFFER_LENGTH);

  if (murata_init)
  {
    printINF("Murata dualstack module init OK\r\n\r\n");
  }
  
  // TX MUTEX ensuring no transmits are happening at the same time
  osMutexDef(txMutex);
  txMutexId = osMutexCreate(osMutex(txMutex));

  osThreadDef(murata_rx_processing, murata_process_rx_response, osPriorityNormal, 0, 512);
  murata_rx_processing_handle = osThreadCreate(osThread(murata_rx_processing), NULL);

  osThreadDef(uart_rx_processing, process_rx_uart, osPriorityNormal, 0, 512);
  uart_rx_processing_handle = osThreadCreate(osThread(uart_rx_processing), NULL);
  osMutexDef(uartMutex);
  uart_rx_process_mutex_id = osMutexCreate(osMutex(uartMutex));

  // pass processing thread handle to murata driver
  Murata_SetProcessingThread(murata_rx_processing_handle);

  //feed IWDG every 5 seconds
  IWDG_feed(NULL);
  osTimerDef(iwdgTim, IWDG_feed);
  iwdgTimId = osTimerCreate(osTimer(iwdgTim), osTimerPeriodic, NULL);
  osTimerStart(iwdgTimId, IWDG_INTERVAL * 1000);

  // LoraWAN timer
  osTimerDef(loraWANTim, LoRaWAN_send);
  loraWANTimId = osTimerCreate(osTimer(loraWANTim), osTimerPeriodic, NULL);
  osTimerStart(loraWANTimId, LORAWAN_INTERVAL * 1000);

  osTimerDef(moduleCheckTim, check_modules);
  moduleCheckTimId = osTimerCreate(osTimer(moduleCheckTim), osTimerPeriodic, NULL);
  osTimerStart(moduleCheckTimId, MODULE_CHECK_INTERVAL * 1000);
  
  //Join before starting the kernel
  Murata_LoRaWAN_Join();
  
  printINF("START \r\n");
  osKernelStart();

  while (1)
  { 
  }
}

 void LoRaWAN_send()
 {
   if (murata_init)
   {
     osMutexWait(txMutexId, osWaitForever);
     if(recentData == 1)
     {
       // Only send of there is recent data
      uint8_t loraMessage[NUMBER_OF_SENSORS*4];
      uint8_t i = 0;

      for(int j = 0; j < NUMBER_OF_SENSORS; j++){
        loraMessage[i++] = allMessages[j].b1;
        loraMessage[i++] = allMessages[j].b2;
        loraMessage[i++] = allMessages[j].b3;
        loraMessage[i++] = allMessages[j].b4;
      }
      
      if(!Murata_LoRaWAN_Send((uint8_t *)loraMessage, i))
      {
        murata_init++;
        if(murata_init == 10)
          murata_init == 0;
      }
      else
      {
        murata_init = 1;
      }
      recentData = 0;
    }
    else{
      printINF("No new data has come in since last LoRaWAN message has been sent\r\n");
    }
    osMutexRelease(txMutexId);
    LoRaWAN_Counter++;
   }
  else{
    printINF("murata not initialized, not sending\r\n");
  }
}

void check_modules(void const *argument)
{
  printINF("checking the status of the modules\r\n");
  if (!murata_init)
  {
    // LORAWAN
    murata_init = Murata_Initialize(short_UID);
    Murata_toggleResetPin();
  }
}

void murata_process_rx_response(void const *argument)
{
  uint32_t startProcessing;
  while (1)
  {
    // Wait to be notified that the transmission is complete.  Note the first
    //parameter is pdTRUE, which has the effect of clearing the task's notification
    //value back to 0, making the notification value act like a binary (rather than
    //a counting) semaphore.
    startProcessing = ulTaskNotifyTake(pdTRUE, osWaitForever);
    if (startProcessing == 1)
    {
      // The transmission ended as expected.
      while(murata_data_ready)
      {
        printINF("processing murata fifo\r\n");
        murata_data_ready = !Murata_process_fifo();
        osDelay(50);
      }
    }
    else
    {
    }
    osDelay(1);
  }
  osThreadTerminate(NULL);
}

void process_rx_uart()
{
  uint32_t startProcessing;
  uint8_t found;
  struct UartMessage newMessage;
  while(1)
  {
      if(data_ready == 1)
      {
        osMutexWait(uart_rx_process_mutex_id, osWaitForever);
        found = 0;
        newMessage = (struct UartMessage) {.b1 = uartSensorData[2], .b2 = uartSensorData[3], .b3 = uartSensorData[4], .b4 = uartSensorData[5]}; // ignore first 2 incoming bytes, those are manufacturing ID
        for(int i = 0; i <= messagesIndex; i++)
        {
          if(memcmp(&allMessages[i], &newMessage, 2) == 0){
            if (memcmp(&allMessages[i], &newMessage, sizeof(newMessage)) == 0)
            {
              // check whether same message already exist
              // Same sensor, same message
              printINF("Found, doing nothing. Message %d %d %d %d\r\n", newMessage.b1, newMessage.b2, newMessage.b3, newMessage.b4);
              found = 1;
              break;
            }
            else{
              // Same sensor, different message
              printINF("Same sensor, different message \r\n");
              allMessages[i] = newMessage;
              recentData = 1;
            }
          }
        }
        if(found == 0)
        {
          // Message from new sensor
          printINF("New, adding message: %d %d %d %d\r\n", newMessage.b1, newMessage.b2, newMessage.b3, newMessage.b4);
          allMessages[messagesIndex] = newMessage;
          messagesIndex++;
          recentData = 1;
        }
      }
      data_ready = 0;
      osMutexRelease(uart_rx_process_mutex_id);
      osDelay(1);
  }
  osThreadTerminate(NULL);
}

void Dualstack_ApplicationCallback(void)
{
  murata_data_ready = 1;
}

void UART_BLE_Callback()
{
  data_ready = 1;
  HAL_UART_Receive_IT(&BLE_UART, &uartSensorData, BUFFER_LENGTH);
}