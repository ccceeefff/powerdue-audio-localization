#include "ADCClock.h"

#include <assert.h>
#include <Arduino.h>

ADCClockClass::ADCClockClass(void){
  _resolution = 1;
  reset();
}

void ADCClockClass::init(uint16_t resolution){
  if(!((resolution != 0) && ((resolution & (~resolution + 1)) == resolution))){
    SerialUSB.println("Resolution must be a power of 2");
    assert(false);
  }
  // get index of most significant bit 
  while (resolution >>= 1) {
    _resolution++;
  }
}

void ADCClockClass::reset(){
  _offset = 0;
  _majorTicks = 0;
}

void ADCClockClass::tick(){
  _majorTicks++;
}

void ADCClockClass::addOffset(int offset, int delay){
  _offset += offset;
  _delay = delay;
}

int ADCClockClass::getOffset(){
  return _offset;
}

int ADCClockClass::getNetworkDelay(){
  return _delay;
}

uint64_t ADCClockClass::getTime(){
  return (_majorTicks <<_resolution) + getMinorTicks() + _offset;
}

uint32_t ADCClockClass::getMajorTicks(){
  return _majorTicks + ((getMinorTicks() + _offset) >> _resolution);
}

uint16_t ADCClockClass::getMinorTicks(){
  /** 
   * ADC_RCR is the remaining number of samples to take for the current buffer.
   * The inverse of that would be the number of samples it has currently taken.
   */
  return ((1<<_resolution) - ADC->ADC_RCR);
}

ADCClockClass ADCClock;