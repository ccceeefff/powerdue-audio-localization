#ifndef ADCCONFIG_H
#define ADCCONFIG_H

#include <inttypes.h>

/*
 * Device Parameters
 */

#define DEVICE_ID "PDue0002"    // use this to uniquely identify your device

/*
 * Application Parameters
 */

// update these
// #define WIFI_SSID "PowerDue"
// #define WIFI_PASS "powerdue"
// #define SERVER_IP "10.230.12.1"

#define WIFI_SSID "YouGotMe"
#define WIFI_PASS "y0ug0tmeg00d"
#define SERVER_IP "192.168.1.9"

#define CLOCK_GPIO_OUT 51
#define BUFFER_QUEUE_LENGTH 8
#define BUFFERS_TO_SEND 8

/*
 * Clock parameters
 */

/* defines the size of a timestamp that's sent over the network */
typedef uint64_t tstamp_t;

// This defines whether this device is the master clock or not
// 0 - this is a slave clock
// 1 - this is the master clock
#define MASTER_CLOCK 0

// The udp port to use for synchronization
#define SYNC_PORT 12345

#if MASTER_CLOCK == 0
  // If this is a slave clock, this must be defined properly
  // #define MASTER_CLOCK_IP "10.230.12.8"  // e.g. "10.230.12.10" ?
  #define MASTER_CLOCK_IP "192.168.1.15"  // e.g. "10.230.12.10" ?
#endif

// Time period between synchronization trials (in milliseconds)
#define SYNC_FREQUENCY  2000

/*
 *  ADCSampler parameters
 */

#define NUM_BUFFERS   16
#define BUFFER_SIZE   128
#define BYTES_PER_SAMPLE 2

#define ADC_CHANNEL_MIC 2       // mic is connected to A2
#define ADC_SAMPLE_RATE 50000

/*
 * ADCTrigger parameters
 */
#define WINDOW_COUNT 8
#define MIN_WINDOW_COUNT 24

#endif  //ADCCONFIG_H