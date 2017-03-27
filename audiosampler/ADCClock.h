#ifndef _ADCCLOCK_H
#define _ADCCLOCK_H

#include "ADCConfig.h"

/**
 *  A clock implementation that runs based on the ADC sample time.
 *  Each major tick is represented by the ADC DMA buffer being filled up.
 *  Minor ticks are counted in the sample number of each buffer.
 *
 *  This class is closely correlated to the ADCSampler. Be sure to understand
 *  how each component is tied together to keep sample timing in line.
 */
class ADCClockClass
{
  public:
    // resolution - defined by the number of minor ticks per major tick
    // typically set to the number of samples per buffer
    // NOTE: Must be a power of two!!
    void init(uint16_t resolution);
    void reset();   // resets the counter to 0
    
    void tick();    // increment the clock by 1 major tick
    
    // returns time in number of samples taken since epoch (includes offset)
    tstamp_t getTime();
    uint32_t getMajorTicks();
    uint16_t getMinorTicks();
    
    // NTP details
    void addOffset(int offset, int delay);   // offset clock by N samples
    int getOffset();        // get current NTP offset
    int getNetworkDelay();  // get network delay associated with offset
    
  public: // protected, do not use externally to this class
    ADCClockClass(void);
    
  private:
    uint16_t _resolution;
    int _offset;  // offset from reference clock
    int _delay;   // network delay associated with offset
    uint32_t _majorTicks;
};

extern ADCClockClass ADCClock;

#endif // _ADCCLOCK_H