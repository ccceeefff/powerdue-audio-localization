#include <Arduino.h>

#include <FreeRTOS_ARM.h>
#include <USBCDC.h>

#include "ADCSampler.h"

#define CLOCK_GPIO_OUT 51
#define BUFFER_QUEUE_LENGTH 5

QueueHandle_t xBufferQueue;

inline void initPins(){
  pinMode(CLOCK_GPIO_OUT, OUTPUT);
}

volatile int pinState = HIGH;
inline void togglePin(int state){
  digitalWrite(CLOCK_GPIO_OUT, state);
  pinState = state;
}

void adc_full(uint16_t *readyBuffer, uint16_t size){
  togglePin(!pinState);
  xQueueSendToBackFromISR(xBufferQueue, &readyBuffer, NULL);
}

/**
 *  Handles each buffer received
 */
void BufferHandlerTask(void *arg){
  ADCSampler.init();
  ADCSampler.setChannel(2);
  ADCSampler.setSamplingFrequency(44100);
  ADCSampler.setInterruptCallback(adc_full);
  ADCSampler.reset();
  ADCSampler.start();
  uint16_t *sampleBuffer;
  SerialUSB2.println("Sampling start!");
  while(1){
    sampleBuffer = NULL;
    if(xQueueReceive(xBufferQueue, &sampleBuffer, portMAX_DELAY)){
      // for now, do the dumb thing of sending it all through the serial port
      // the samples are in uint16_t format, we can only write by uint8_t formats
      // reinterpret as a byte buffer and specify size in bytes 
      SerialUSB.write((uint8_t *)sampleBuffer, BUFFER_SIZE*BYTES_PER_SAMPLE);
    }
  }
  ADCSampler.stop();
}

void setup(){
  initPins();
  
  SerialUSB.begin(0);
  SerialUSB2.begin(0);
  while(!SerialUSB2);
  
  xBufferQueue = xQueueCreate(BUFFER_QUEUE_LENGTH, sizeof(uint16_t *));

  xTaskCreate(BufferHandlerTask, NULL, configMINIMAL_STACK_SIZE, NULL, 1, NULL);

  vTaskStartScheduler();
  SerialUSB2.println("Insufficient memory");
  while(1);
}

void loop(){
  // do nothing
}