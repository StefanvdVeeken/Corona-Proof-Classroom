#include "lorawan-ble.h"
#include "murata.h"

#define IWDG_INTERVAL           5    //seconds
#define LORAWAN_INTERVAL        30   //seconds
#define MODULE_CHECK_INTERVAL   3600 //seconds

uint16_t LoRaWAN_Counter = 0;
uint8_t murata_init = 0;
uint64_t short_UID;
uint8_t murata_data_ready = 0;

float SHTData[2];
uint8_t temperature;
int main(void)
{
  Initialize_Platform();
  printINF("before start\r\n");
  // LORAWAN
  murata_init = Murata_Initialize(short_UID);
  UART_SetApplicationCallback(&Dualstack_ApplicationCallback, (uint8_t)MURATA_CONNECTOR);

  //BLE
  UART_BLE_SetRxCallback(UART_BLE_Callback);
  HAL_UART_Receive_IT(&BLE_UART, &temperature, 1);

  if (murata_init)
  {
    printINF("Murata dualstack module init OK\r\n\r\n");
  }
  
  // TX MUTEX ensuring no transmits are happening at the same time
  osMutexDef(txMutex);
  txMutexId = osMutexCreate(osMutex(txMutex));

  osThreadDef(murata_rx_processing, murata_process_rx_response, osPriorityNormal, 0, 512);
  murata_rx_processing_handle = osThreadCreate(osThread(murata_rx_processing), NULL);

  // pass processing thread handle to murata driver
  Murata_SetProcessingThread(murata_rx_processing_handle);

  //feed IWDG every 5 seconds
  IWDG_feed(NULL);
  osTimerDef(iwdgTim, IWDG_feed);
  iwdgTimId = osTimerCreate(osTimer(iwdgTim), osTimerPeriodic, NULL);
  osTimerStart(iwdgTimId, IWDG_INTERVAL * 1000);

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

 void LoRaWAN_send(void const *argument)
 {
   if (murata_init)
   {
     uint8_t loraMessage[4];
     uint8_t i = 0;
    //uint16 counter to uint8 array (little endian)
    //counter (large) type byte
    loraMessage[i++] = 'T'; //HEX code : 54
    loraMessage[i++] = (uint8_t)1;
    loraMessage[i++] = (uint8_t)10;
    loraMessage[i++] = (uint8_t)100;
    // float_union.fl = SHTData[0];
    // loraMessage[i++] = float_union.bytes.b1;
    // loraMessage[i++] = float_union.bytes.b2;
    // loraMessage[i++] = float_union.bytes.b3;
    // loraMessage[i++] = float_union.bytes.b4;
    // float_union.fl = SHTData[1];
    // loraMessage[i++] = float_union.bytes.b1;
    // loraMessage[i++] = float_union.bytes.b2;
    // loraMessage[i++] = float_union.bytes.b3;
    // loraMessage[i++] = float_union.bytes.b4;
    // uint16LittleEndian.integer = accDataRaw[0];
    // loraMessage[i++] = uint16LittleEndian.byte[0];
    // loraMessage[i++] = uint16LittleEndian.byte[1];
    // uint16LittleEndian.integer = accDataRaw[1];
    // loraMessage[i++] = uint16LittleEndian.byte[0];
    // loraMessage[i++] = uint16LittleEndian.byte[1];
    // uint16LittleEndian.integer = accDataRaw[2];
    // loraMessage[i++] = uint16LittleEndian.byte[0];
    // loraMessage[i++] = uint16LittleEndian.byte[1];


    osMutexWait(txMutexId, osWaitForever);
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

void Dualstack_ApplicationCallback(void)
{
  murata_data_ready = 1;
}

void UART_BLE_Callback(){
  //printINF("Temperature %d \r\n", temperature);
  HAL_UART_Receive_IT(&BLE_UART, &temperature, 1);
}