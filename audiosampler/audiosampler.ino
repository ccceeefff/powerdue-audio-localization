#include <Arduino.h>

#include <FreeRTOS_ARM.h>
#include <USBCDC.h>

#include "ADCSampler.h"
#include "ADCTrigger.h"

#define CLOCK_GPIO_OUT 51
#define BUFFER_QUEUE_LENGTH 5

#define ADC_CHANNEL_MIC 2
#define ADC_SAMPLE_RATE 44100

QueueHandle_t xBufferQueue;
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
  togglePin(!pinState);
  xQueueSendToBackFromISR(xBufferQueue, &index, NULL);
}

/**
 *  Handles each buffer received
 */
void BufferHandlerTask(void *arg){
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
          SerialUSB.write((uint8_t *)ADCSampler.getBufferAtIndex(bufIndex-1), BUFFER_SIZE*BYTES_PER_SAMPLE);
          bufCount--;
        }
        // for now, do the dumb thing of sending it all through the serial port
        // the samples are in uint16_t format, we can only write by uint8_t formats
        // reinterpret as a byte buffer and specify size in bytes 
        SerialUSB.write((uint8_t *)ADCSampler.getBufferAtIndex(bufIndex), BUFFER_SIZE*BYTES_PER_SAMPLE);
        bufCount--;
      }
    }
  }
  ADCSampler.stop();
}

void setup(){
  initPins();
  
  SerialUSB.begin(0);
  SerialUSB2.begin(0);
  while(!SerialUSB2);
  
  xBufferQueue = xQueueCreate(BUFFER_QUEUE_LENGTH, sizeof(uint8_t));

  xTaskCreate(BufferHandlerTask, NULL, configMINIMAL_STACK_SIZE, NULL, 1, NULL);

  vTaskStartScheduler();
  SerialUSB2.println("Insufficient memory");
  while(1);
}

void loop(){
  // do nothing
}