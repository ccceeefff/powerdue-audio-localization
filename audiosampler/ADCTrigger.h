#ifndef _ADCTRIGGER_H
#define _ADCTRIGGER_H

#include "ADCConfig.h"

/*
 * A class for performing event detection on a continuous wave signal.
 * Event detection is performed by simple anomaly detection based on
 * the gaussian distribution of signal energy of noise.
 */
class ADCTrigger
{
  public:
    ADCTrigger();
    
    void feed(uint16_t *buffer, uint16_t len);
    bool isTriggered();
    void reset();
    
  protected:
    // returns this windows total energy
    float updateMeanAndEnergy(uint16_t *buffer, uint16_t len);
    float getSignalMean();
    float getEnergyMean();
    float getEnergyStd();
    
  private:
    float _mean[WINDOW_COUNT];    // mean of the past n-windows
    
    float _energy[WINDOW_COUNT];   // energy level for the past n-windows
    
    uint8_t _currentIndex;
    uint8_t _windowCount;
    uint8_t _triggerCount;
    bool _isTriggered;
};

#endif  //_ADCTRIGGER_H