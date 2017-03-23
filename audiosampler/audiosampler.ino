#include <Arduino.h>

#include <assert.h>
#include <FreeRTOS_ARM.h>
#include <USBCDC.h>
#include <IPAddress.h>
#include <PowerDueWiFi.h>

#include "ADCConfig.h"
#include "ADCSampler.h"
#include "ADCTrigger.h"
#include "ADCClock.h"
#include "synchronization.h"

QueueHandle_t xBufferQueue;
QueueHandle_t xSerialQueue;
QueueHandle_t xTcpQueue;
ADCTrigger trigger;

inline void initPins(){
  pinMode(CLOCK_GPIO_OUT, OUTPUT);
}

volatile int pinState = HIGH;
inline void togglePin(int state){
  digitalWrite(CLOCK_GPIO_OUT, state);
  pinState = state;
}

void adc_full(uint8_t index, uint16_t *readyBuffer, uint16_t size){
  // clock goes tick!
  ADCClock.tick();
  
  // if even, toggle low
  // if odd, toggle high
  togglePin(ADCClock.getMajorTicks()%2);
  xQueueSendToBackFromISR(xBufferQueue, &index, NULL);
}

/**
 *  Handles each buffer received
 */
void BufferHandlerTask(void *arg){
  ADCClock.init(BUFFER_SIZE);
  ADCClock.reset();
  
  ADCSampler.init();
  ADCSampler.setChannel(ADC_CHANNEL_MIC);
  ADCSampler.setSamplingFrequency(ADC_SAMPLE_RATE);
  ADCSampler.setInterruptCallback(adc_full);
  ADCSampler.reset();
  ADCSampler.start();
  uint8_t bufIndex;
  uint8_t bufCount = 0;
  SerialUSB2.println("Sampling start!");
  while(1){
    bufIndex = 0;
    if(xQueueReceive(xBufferQueue, &bufIndex, portMAX_DELAY)){
      trigger.feed(ADCSampler.getBufferAtIndex(bufIndex), BUFFER_SIZE);
      if(bufCount > 0 || trigger.isTriggered()){
        trigger.reset();
        // we want to output 8 buffers
        if(bufCount == 0){
          bufCount = 8; // start sending out buffers
          
          // start by sending the previous 1 buffers
          sendBuffer(bufIndex-1);
          bufCount--;
        }
        // for now, do the dumb thing of sending it all through the serial port
        // the samples are in uint16_t format, we can only write by uint8_t formats
        // reinterpret as a byte buffer and specify size in bytes 
        sendBuffer(bufIndex);
        bufCount--;
      }
    }
  }
  ADCSampler.stop();
}

void sendBuffer(uint8_t bufIndex){
  // don't block when sending
  xQueueSendToBack(xSerialQueue, &bufIndex, 0);
  xQueueSendToBack(xTcpQueue, &bufIndex, 0);
}

void SerialStreamOut(void *arg){
  SerialUSB.begin(0);
  uint8_t bufIndex;
  while(1){
    if(xQueueReceive(xSerialQueue, &bufIndex, portMAX_DELAY)){
      SerialUSB.write((uint8_t *)ADCSampler.getBufferAtIndex(bufIndex), BUFFER_SIZE*BYTES_PER_SAMPLE);
    }
  }
}

void TcpStreamOut(void *arg){
  struct sockaddr_in serverAddr;  
  socklen_t socklen;
  memset(&serverAddr, 0, sizeof(serverAddr));
  
  serverAddr.sin_len = sizeof(serverAddr);
  serverAddr.sin_family = AF_INET;
  serverAddr.sin_port = htons(4000);
  inet_pton(AF_INET, SERVER_IP, &(serverAddr.sin_addr));
  
  int s = lwip_socket(AF_INET, SOCK_STREAM, 0);
  if(lwip_connect(s, (struct sockaddr *)&serverAddr, sizeof(serverAddr))){
    SerialUSB.println("Failed to connect to server");
    assert(false);
  }
  
  uint8_t bufIndex;
  while(1){
    if(xQueueReceive(xTcpQueue, &bufIndex, portMAX_DELAY)){
      lwip_write(s, (uint8_t *)ADCSampler.getBufferAtIndex(bufIndex), BUFFER_SIZE*BYTES_PER_SAMPLE);
    }
  }
  // close socket after everything is done
  lwip_close(s);
  SerialUSB.println("socket closed");
}

void onError(int errorCode){
  SerialUSB2.print("Error received: ");
  SerialUSB2.println(errorCode);
}

void onReady(){
  SerialUSB2.println("Device ready");  
  SerialUSB2.print("Device IP: ");
  SerialUSB2.println(IPAddress(PowerDueWiFi.getDeviceIP()));  

  startTasks();
}

void startTasks(void){
  xTaskCreate(BufferHandlerTask, NULL, configMINIMAL_STACK_SIZE, NULL, 1, NULL);
  xTaskCreate(SerialStreamOut, NULL, configMINIMAL_STACK_SIZE, NULL, 1, NULL);
  xTaskCreate(TcpStreamOut, NULL, configMINIMAL_STACK_SIZE, NULL, 1, NULL);
  startSyncTasks(2);
}

void setup(){
  initPins();
  
  SerialUSB.begin(0);
  SerialUSB2.begin(0);
  while(!SerialUSB2);
  
  xBufferQueue = xQueueCreate(BUFFER_QUEUE_LENGTH, sizeof(uint8_t));
  xSerialQueue = xQueueCreate(BUFFER_QUEUE_LENGTH, sizeof(uint8_t));
  xTcpQueue = xQueueCreate(BUFFER_QUEUE_LENGTH, sizeof(uint8_t));
  
  PowerDueWiFi.init(WIFI_SSID, WIFI_PASS);
  PowerDueWiFi.setCallbacks(onReady, onError);

  // startTasks();

  vTaskStartScheduler();
  SerialUSB2.println("Insufficient memory");
  while(1);
}

void loop(){
  // do nothing
}