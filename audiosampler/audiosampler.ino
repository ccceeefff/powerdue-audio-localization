#include <Arduino.h>

#include "ADCSampler.h"

#define CLOCK_GPIO_OUT 51

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
  SerialUSB.print("full: ");
  SerialUSB.println(micros());
}

void setup(){
  initPins();
  
  while(!SerialUSB);
  
  ADCSampler.init();
  ADCSampler.setChannel(5);
  ADCSampler.setSamplingFrequency(44100);
  ADCSampler.setInterruptCallback(adc_full);
  ADCSampler.reset();
  ADCSampler.start();
}

void loop(){
  
}