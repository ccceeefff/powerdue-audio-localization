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
#include "PowerDue.h"
#include "packet.h"
#include "synchronization.h"

QueueHandle_t xBufferQueue;
QueueHandle_t xSerialQueue;
QueueHandle_t xTcpQueue;
ADCTrigger trigger;

// store the timestamps for each buffer to report later
tstamp_t tstamps[NUM_BUFFERS];

#define PACKET_BUFFER 2

AcousticEvent_t capturedEvent[PACKET_BUFFER];
void initPacket(){
  for(int p=0; p < PACKET_BUFFER; p++){
    capturedEvent[p].pktHeader = EVENT_PACKET_HEADER;
    for(int i=0; i < DEVICE_ID_LENGTH; i++){
      capturedEvent[p].deviceID[i] = DEVICE_ID[i];
    }
    for(int i=0; i < RESERVED_SPACE; i++){
      capturedEvent[p].reserved[i] = 0;   // clear out the reserved space
    }
    capturedEvent[p].samplingFrequency = ADC_SAMPLE_RATE;  
    capturedEvent[p].pktFooter = EVENT_PACKET_FOOTER;
  }
  
  SerialUSB2.println("Packet initialized");
}

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
  
  // get current time!
  tstamps[index] = ADCClock.getTime();
  
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
  uint8_t currPacket = 0;
  SerialUSB2.println("Sampling start!");
  while(1){
    bufIndex = 0;
    if(xQueueReceive(xBufferQueue, &bufIndex, portMAX_DELAY)){
      trigger.feed(ADCSampler.getBufferAtIndex(bufIndex), BUFFER_SIZE);
      if(bufCount > 0 || trigger.isTriggered()){
        trigger.reset();
        // we want to output 8 buffers
        if(bufCount == 0){
          pd_rgb_led(PD_BLUE);
          bufCount = BUFFERS_TO_SEND; // start sending out buffers
          
          uint8_t ind = 0;
          if(bufIndex == 0){
            ind = NUM_BUFFERS-1;
          } else {
            ind = bufIndex-1;
          }
          
          // start by sending the previous 1 buffers
          capturedEvent[currPacket].timestamp = tstamps[ind];
          memcpy( 
                  &(capturedEvent[currPacket].samples[BUFFER_SIZE * (BUFFERS_TO_SEND-bufCount)]), 
                  (uint8_t *)ADCSampler.getBufferAtIndex(ind),
                  BUFFER_SIZE * BYTES_PER_SAMPLE
                );
          bufCount--;
        }
        // for now, do the dumb thing of sending it all through the serial port
        // the samples are in uint16_t format, we can only write by uint8_t formats
        // reinterpret as a byte buffer and specify size in bytes       
        memcpy( 
                &(capturedEvent[currPacket].samples[BUFFER_SIZE * (BUFFERS_TO_SEND-bufCount)]), 
                (uint8_t *)ADCSampler.getBufferAtIndex(bufIndex),
                BUFFER_SIZE * BYTES_PER_SAMPLE
              );
        bufCount--;
        if(bufCount == 0){
          // send out the buffer
          sendPacket(currPacket);
          currPacket = (currPacket+1)%PACKET_BUFFER;  
          pd_rgb_led(PD_RED);        
        }
      }
    }
  }
  ADCSampler.stop();
}

void sendPacket(uint8_t bufIndex){
  // don't block when sending
  xQueueSendToBack(xSerialQueue, &bufIndex, 0);
  xQueueSendToBack(xTcpQueue, &bufIndex, 0);
}

void SerialStreamOut(void *arg){
  SerialUSB.begin(0);
  uint8_t bufIndex;
  while(1){
    if(xQueueReceive(xSerialQueue, &bufIndex, portMAX_DELAY)){
      uint8_t *buf = (uint8_t *)(&capturedEvent[bufIndex]);
      SerialUSB.write(buf, PACKET_LENGTH);
    }
  }
}

void TcpStreamOut(void *arg){
  struct sockaddr_in serverAddr;  
  socklen_t socklen;
  // outer loop to establish connection
  while(1){
    memset(&serverAddr, 0, sizeof(serverAddr));
    
    serverAddr.sin_len = sizeof(serverAddr);
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(4000);
    inet_pton(AF_INET, SERVER_IP, &(serverAddr.sin_addr));
    
    int s = lwip_socket(AF_INET, SOCK_STREAM, 0);
    int r = 0;
    while(r = lwip_connect(s, (struct sockaddr *)&serverAddr, sizeof(serverAddr))){
      SerialUSB2.println("Failed to connect to server");
      SerialUSB2.print("Error code: ");
      SerialUSB2.println(r);
      lwip_close(s);
      lwip_shutdown(s, SHUT_RDWR);
      // flush the receive queue so we don't block the rest of the system
      xQueueReset(xTcpQueue);
      
      vTaskDelay(pdMS_TO_TICKS(2000)); // wait a bit and try again
    }
    
    uint8_t bufIndex;
    
    // inner loop to handle sending data
    while(1){
      if(xQueueReceive(xTcpQueue, &bufIndex, portMAX_DELAY)){
        pd_rgb_led(PD_GREEN);
        if(lwip_write(s, (uint8_t *)&capturedEvent[bufIndex], PACKET_LENGTH) < 0){
          // an error occurred during write.
          // did we disconnect? break out and try to reconnect.
          SerialUSB2.println("Failed to send. Trying to reconnect.");
          break;
        }
        pd_rgb_led(PD_RED);
      }
    }
    
    // close socket after everything is done
    lwip_close(s);
    lwip_shutdown(s, SHUT_RDWR);
    SerialUSB.println("socket closed");
  }
  
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
  SerialUSB.begin(0);
  SerialUSB2.begin(0);
  while(!SerialUSB2);
  
  initPins();
  initPacket();
  pd_rgb_led_init();
  
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