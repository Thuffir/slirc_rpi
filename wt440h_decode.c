/***********************************************************************************************************************
 *
 * WT440H Receiver / Decoder for Raspberry Pi
 *
 * (C) 2015 Gergely Budai
 *
 * This is free and unencumbered software released into the public domain.
 *
 * Anyone is free to copy, modify, publish, use, compile, sell, or
 * distribute this software, either in source code form or as a compiled
 * binary, for any purpose, commercial or non-commercial, and by any
 * means.
 *
 * In jurisdictions that recognize copyright laws, the author or authors
 * of this software dedicate any and all copyright interest in the
 * software to the public domain. We make this dedication for the benefit
 * of the public at large and to the detriment of our heirs and
 * successors. We intend this dedication to be an overt act of
 * relinquishment in perpetuity of all present and future rights to this
 * software under copyright law.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * For more information, please refer to <http://unlicense.org/>
 *
 **********************************************************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>
#include <sys/time.h>

// Bit definitions
#define BIT_MSK                   1
#define BIT_TIMEOUT_MSK           2

// Device file
#define DEV_FILE                  "/dev/wt440h"

// Suppress identical messages within this timeframe in uS
#define SUPPRESS_TIME             1000000

// Decoded WT440H Message
typedef struct {
  uint8_t houseCode;
  uint8_t channel;
  uint8_t status;
  uint8_t batteryLow;
  uint8_t humidity;
  uint8_t tempInteger;
  uint8_t tempFraction;
  uint8_t sequneceNr;
  uint32_t timeStamp;
} WT440hDataType;

// Device file descriptor
int dev;

/***********************************************************************************************************************
 * Init functions
 **********************************************************************************************************************/
void Init(void)
{
  // Open device file for reading
  dev = open(DEV_FILE, O_RDONLY);
  if(dev == -1) {
    perror("open()");
    exit(EXIT_FAILURE);
  }
}

/***********************************************************************************************************************
 * Decode received bits into a WT440H Message
 **********************************************************************************************************************/
WT440hDataType RxData(void)
{
  // Preamble bits
  static const uint8_t preamble[] = {1, 1, 0, 0};
  // Received bit info
  unsigned char bit;
  // Decoded WT440H data
  WT440hDataType data = { 0 };
  // Bit number counter
  uint8_t bitNr;
  // Checksum
  uint8_t checksum = 0;

  // Data is 36 bits long
  for(bitNr = 0; bitNr < 36;) {
    // Read Bit
    if(read(dev, &bit, sizeof(bit)) != sizeof(bit)) {
      printf("read()");
      exit(EXIT_FAILURE);
    }

    // Only the first bit can have the bit timeout flag set
    if(bitNr > 0) {
      // Check timeout flag
      if(bit & BIT_TIMEOUT_MSK) {
        // printf("Bit timeout bit %u\n", bitNr);
        bitNr = 0;
        checksum = 0;
        memset(&data, 0, sizeof(data));
        continue;
      }
    }
    // Remove timeout flag
    bit &= BIT_MSK;

    // Preamble [0 .. 3]
    if(bitNr <= 3) {
      if(bit != preamble[bitNr]) {
        // printf("Wrong preamble %u at bit %u\n", bitInfo.bit, bitNr);
        bitNr = 0;
        checksum = 0;
        memset(&data, 0, sizeof(data));
        continue;
      }
    }
    // Housecode [4 .. 7]
    else if((bitNr >= 4) && (bitNr <= 7)) {
      data.houseCode = (data.houseCode << 1) | bit;
    }
    // Channel [8 .. 9]
    else if((bitNr >= 8) && (bitNr <= 9)) {
      data.channel = (data.channel << 1) | bit;
    }
    // Status [10 .. 11]
    else if((bitNr >= 10) && (bitNr <= 11)) {
      data.status = (data.status << 1) | bit;
    }
    // Battery Low [12]
    else if(bitNr == 12) {
      data.batteryLow = bit;
    }
    // Humidity [13 .. 19]
    else if((bitNr >= 13) && (bitNr <= 19)) {
      data.humidity = (data.humidity << 1) | bit;
    }
    // Temperature (Integer part) [20 .. 27]
    else if((bitNr >= 20) && (bitNr <= 27)) {
      data.tempInteger = (data.tempInteger << 1) | bit;
    }
    // Temperature (Fractional part) [28 .. 31]
    else if((bitNr >= 28) && (bitNr <= 31)) {
      data.tempFraction = (data.tempFraction << 1) | bit;
    }
    // Message Sequence [32 .. 33]
    else if((bitNr >= 32) && (bitNr <= 33)) {
      data.sequneceNr = (data.sequneceNr << 1) | bit;
    }

    // Update checksum
    checksum ^= bit << (bitNr & 1);
    // and check checksum if appropriate
    if(bitNr == 35) {
      // If checksum correct
      if(checksum == 0) {
        // Record reception Timestamp
        struct timeval tv;
        gettimeofday(&tv, NULL);
        data.timeStamp = (tv.tv_sec * 1000000) + tv.tv_usec;
      }
      // Checksum error
      else {
        // printf("Checksum error\n");
        bitNr = 0;
        checksum = 0;
        memset(&data, 0, sizeof(data));
        continue;
      }
    }

    // Increment bit pointer
    bitNr++;
  }

  return data;
}

/***********************************************************************************************************************
 * Main
 **********************************************************************************************************************/
int main(void)
{
  // Decoded WT440H data and the previous one
  WT440hDataType data, prevData = { 0 };

  // Do init stuff
  Init();

  // Receive and decode messages
  while(1) {
    // Wait for Message
    data = RxData();
    // Check if a message is a duplicate of a last one
    if((data.houseCode != prevData.houseCode) || (data.channel != prevData.channel) ||
        (data.status != prevData.status) || (data.batteryLow != prevData.batteryLow) ||
        (data.humidity != prevData.humidity) || (data.tempInteger != prevData.tempInteger) ||
        (data.tempFraction != prevData.tempFraction) || ((data.timeStamp - prevData.timeStamp) >= SUPPRESS_TIME)) {
      // If no duplicate, print
      printf("%u %u %u %u %u %.1f\n", data.houseCode, data.channel + 1, data.status, data.batteryLow, data.humidity,
        ((double)data.tempInteger - (double)50) + ((double)data.tempFraction / (double)16));
      fflush(stdout);
    }
    // Remember old message
    prevData = data;
  }

  return 0;
}
