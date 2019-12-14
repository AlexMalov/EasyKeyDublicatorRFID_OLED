/*
OneWireSlave v1.1 by Joshua Fuller - Modified based on versions noted below for Digispark
OneWireSlave v1.0 by Alexander Gordeyev
It is based on Jim's Studt OneWire library v2.0
Permission is hereby granted, free of charge, to any person obtaining
a copy of this software and associated documentation files (the
"Software"), to deal in the Software without restriction, including
without limitation the rights to use, copy, modify, merge, publish,
distribute, sublicense, and/or sell copies of the Software, and to
permit persons to whom the Software is furnished to do so, subject to
the following conditions:
The above copyright notice and this permission notice shall be
included in all copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
Much of the code was inspired by Derek Yerger's code, though I don't
think much of that remains.  In any event that was..
    (copyleft) 2006 by Derek Yerger - Free to distribute freely.
The CRC code was excerpted and inspired by the Dallas Semiconductor
sample code bearing this copyright.
//---------------------------------------------------------------------------
// Copyright (C) 2000 Dallas Semiconductor Corporation, All Rights Reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a
// copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation
// the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the
// Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included
// in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
// OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY,  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
// IN NO EVENT SHALL DALLAS SEMICONDUCTOR BE LIABLE FOR ANY CLAIM, DAMAGES
// OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
// ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
// OTHER DEALINGS IN THE SOFTWARE.
//
// Except as contained in this notice, the name of Dallas Semiconductor
// shall not be used except as stated in the Dallas Semiconductor
// Branding Policy.
//---------------------------------------------------------------------------
*/

#include "OneWireSlave.h"

//#define microsecondsToClockCycles(a) ( (a) * clockCyclesPerMicrosecond() )

//ORIG: #define TIMESLOT_WAIT_RETRY_COUNT microsecondsToClockCycles(120) / 10L
//FULL: #define TIMESLOT_WAIT_RETRY_COUNT ( ((120) * (8000000L / 1000L)) / 1000L )
//WORKING: #define TIMESLOT_WAIT_RETRY_COUNT microsecondsToClockCycles(240000) / 10L

//These are the major change from original, we now wait quite a bit longer for some things
#define TIMESLOT_WAIT_RETRY_COUNT microsecondsToClockCycles(120) / 10L
//This TIMESLOT_WAIT_READ_RETRY_COUNT is new, and only used when waiting for the line to go low on a read
//It was derived from knowing that the Arduino based master may go up to 130 micros more than our wait after reset
#define TIMESLOT_WAIT_READ_RETRY_COUNT microsecondsToClockCycles(135)

void OneWireSlave::ISRPIN() {
  (*static_OWS_instance).MasterResetPulseDetection();
}

uint8_t _pin;

OneWireSlave::OneWireSlave(uint8_t pin) {
	_pin = pin;
	pinMode(_pin, INPUT);
	pin_bitmask = PIN_TO_BITMASK(_pin);
	baseReg = PIN_TO_BASEREG(_pin);
	
	//#define dsslavepin _pin
  //attachInterrupt(dsslaveassignedint, &ISRPIN, CHANGE);
}

volatile long previous = 0;
volatile long old_previous = 0;
volatile long diff = 0;

void OneWireSlave::MasterResetPulseDetection() {
  old_previous = previous;
  previous = micros();
  diff = previous - old_previous;
  if (diff >= lowmark && diff <= highmark) {
    waitForRequestInterrupt(false);
  }
}

bool OneWireSlave::owsprint() {
	//waitForRequestInterrupt(false);
	//Serial.println("done");
  delayMicroseconds(25);
  
  IO_REG_TYPE mask = pin_bitmask;
	volatile IO_REG_TYPE *reg IO_REG_ASM = PIN_TO_BASEREG(_pin);

  errno = ONEWIRE_NO_ERROR;
  noInterrupts();
  DIRECT_WRITE_LOW(reg, mask);
  DIRECT_MODE_OUTPUT(reg, mask);    // drive output low
  interrupts();

  delayMicroseconds(125);
  noInterrupts();
  DIRECT_MODE_INPUT(reg, mask);     // allow it to float
  interrupts();

  delayMicroseconds(300 - 50);
  
  uint8_t retries = 25;
  
//  if (!DIRECT_READ(reg, mask)) {
//    //continue;
//  }
  
//  while(!DIRECT_READ(reg, mask));
//  do {
//		if ( retries-- == 0) {
//			//return FALSE;
//			delayMicroseconds(2);
//		}
//  } while(!DIRECT_READ(reg, mask));

	//Serial.println(DIRECT_READ(reg, mask));
	
	delayMicroseconds(50);

	//while (recvAndProcessCmd()) {};
	recvAndProcessCmd();
}

void OneWireSlave::init(unsigned char rom[8]) {
	for (int i=0; i<7; i++)
    this->rom[i] = rom[i];
  this->rom[7] = crc8(this->rom, 7);
}

void OneWireSlave::setScratchpad(unsigned char scratchpad[9]) {
  for (int i=0; i<8; i++)
    this->scratchpad[i] = scratchpad[i];
  this->scratchpad[8] = crc8(this->scratchpad, 8);
}

void OneWireSlave::setPower(uint8_t power) {
  this->power = power;
}

void OneWireSlave::setResolution(uint8_t resolution) {
	switch (resolution) {
      case 12:
      	this->scratchpad[4] = TEMP_12_BIT;
        break;
      case 11:
      	this->scratchpad[4] = TEMP_11_BIT;
        break;
      case 10:
      	this->scratchpad[4] = TEMP_10_BIT;
        break;
      case 9:
      	this->scratchpad[4] = TEMP_9_BIT;
        break;
	}
	this->scratchpad[8] = crc8(this->scratchpad, 8);
}

uint8_t OneWireSlave::getResolution() {
	switch (scratchpad[4]) {
      case TEMP_12_BIT:
        return 12;
        
      case TEMP_11_BIT:
        return 11;
        
      case TEMP_10_BIT:
        return 10;
        
      case TEMP_9_BIT:
        return 9;
	}
}

void (*user44hFunc)(void);

void OneWireSlave::attach44h(void (*userFunction44h)(void)) {
	user44hFunc = userFunction44h;
	this->scratchpad[8] = crc8(this->scratchpad, 8);
}

void (*user48hFunc)(void);

void OneWireSlave::attach48h(void (*userFunction48h)(void)) {
	user48hFunc = userFunction48h;
}

void (*userB8hFunc)(void);

void OneWireSlave::attachB8h(void (*userFunctionB8h)(void)) {
	userB8hFunc = userFunctionB8h;
}

bool OneWireSlave::waitForRequest(bool ignore_errors) {
  errno = ONEWIRE_NO_ERROR;

  for (;;) {
    //delayMicroseconds(40);
    //Once reset is done, it waits another 30 micros
    //Master wait is 65, so we have 35 more to send our presence now that reset is done
    if (!waitReset(0) ) {
      continue;
    }
    //Reset is complete, tell the master we are prsent
    // This will pull the line low for 125 micros (155 micros since the reset) and 
    //  then wait another 275 plus whatever wait for the line to go high to a max of 480
    // This has been modified from original to wait for the line to go high to a max of 480.
    if (!presence() ) {
      continue;
    }
    //Now that the master should know we are here, we will get a command from the line
    //Because of our changes to the presence code, the line should be guranteed to be high
    if (recvAndProcessCmd() ) {
      return TRUE;
    }
    else if ((errno == ONEWIRE_NO_ERROR) || ignore_errors) {
      continue;
    }
    else {
      return FALSE;
    }
  }
}

bool OneWireSlave::waitForRequestInterrupt(bool ignore_errors) {
  errno = ONEWIRE_NO_ERROR;
  //owsprint();
  //Reset is detected from the Interrupt by counting time between the Level-Changes
  //Once reset is done, it waits another 30 micros
  //Master wait is 65, so we have 35 more to send our presence now that reset is done
  //delayMicroseconds(30);		good working!!!
	delayMicroseconds(25);
  //Reset is complete, tell the master we are prsent
  // This will pull the line low for 125 micros (155 micros since the reset) and 
  //  then wait another 275 plus whatever wait for the line to go high to a max of 480
  // This has been modified from original to wait for the line to go high to a max of 480.
  while (!presence(50) ) {};	//50	//45 arbeitet schon sehr gut
  //Now that the master should know we are here, we will get a command from the line
  //Because of our changes to the presence code, the line should be guranteed to be high
  while (recvAndProcessCmd() ) {};
  if ((errno == ONEWIRE_NO_ERROR) || ignore_errors) {
    //continue;
  }
  else {
    return FALSE;
  }
}

bool OneWireSlave::recvAndProcessCmd() {
	char addr[8];
  uint8_t oldSREG = 0;
  uint16_t raw = 0;

  for (;;) {
    uint8_t cmd = recv();
    switch (cmd) {
      case 0xF0: // SEARCH ROM
        search();
        //delayMicroseconds(6900);
        return FALSE;
      case 0xEC: // ALARM SEARCH
      	raw = ((scratchpad[1] << 8) | scratchpad[0]) >> 4;
      	if ( raw <= scratchpad[3] || raw >= scratchpad[2] )
      		search();
        return FALSE;
      case 0x33: // READ ROM
        sendData(rom, 8);
        if (errno != ONEWIRE_NO_ERROR)
          return FALSE;
        break;
      case 0x55: // MATCH ROM - Choose/Select ROM
        recvData(addr, 8);
        if (errno != ONEWIRE_NO_ERROR)
          return FALSE;
        for (int i=0; i<8; i++)
          if (rom[i] != addr[i])
            return FALSE;
        duty();
      case 0xCC: // SKIP ROM
      	duty();
      	if (errno != ONEWIRE_NO_ERROR)
          return FALSE;
        return TRUE;
      default: // Unknow command
        if (errno == ONEWIRE_NO_ERROR)
          break; // skip if no error
        else
          return FALSE;
    }
  }
}

bool OneWireSlave::duty() {
	uint8_t done = recv();
	
	switch (done) {
		case 0xBE: // READ SCREATCHPAD
			sendData(scratchpad, 9);
			if (errno != ONEWIRE_NO_ERROR)
				return FALSE;
			break;
		case 0xB4: // READ POWERSOURCE
			sendBit(power);
			if (errno != ONEWIRE_NO_ERROR)
				return FALSE;
			break;
		case 0x44: // CONVERT SENSOR
			user44hFunc();
			if (errno != ONEWIRE_NO_ERROR)
				return FALSE;
			break;
		case 0x48: // CONVERT SENSOR
			user48hFunc();
			if (errno != ONEWIRE_NO_ERROR)
				return FALSE;
			break;
		case 0xB8: // CONVERT SENSOR
			userB8hFunc();
			if (errno != ONEWIRE_NO_ERROR)
				return FALSE;
			break;
		case 0x4E: // WRITE SCREATCHPAD
			recvData(temp_scratchpad, 3);
			setScratchpad_external(temp_scratchpad);
			if (errno != ONEWIRE_NO_ERROR)
				return FALSE;
			break;
		default:
			break;
			if (errno == ONEWIRE_NO_ERROR)
				break; // skip if no error
			else
				return FALSE;
	return TRUE;
	}
}

void OneWireSlave::setScratchpad_external(char temp_scratchpad[3]) {
  for (int i=2; i<5; i++)
    this->scratchpad[i] = temp_scratchpad[i-2];
  this->scratchpad[8] = crc8(this->scratchpad, 8);
}

void OneWireSlave::setTemperature(unsigned char scratchpadtemperature[2]) {
	for (int i=0; i<2; i++)
    this->scratchpad[i] = scratchpadtemperature[i];
  this->scratchpad[8] = crc8(this->scratchpad, 8);
}

bool OneWireSlave::search() {
  uint8_t bitmask;
  uint8_t bit_send, bit_recv;

  for (int i=0; i<8; i++) {
    for (bitmask = 0x01; bitmask; bitmask <<= 1) {
      bit_send = (bitmask & rom[i])?1:0;
      sendBit(bit_send);
      sendBit(!bit_send);
      bit_recv = recvBit();
      if (errno != ONEWIRE_NO_ERROR)
        return FALSE;
      if (bit_recv != bit_send)
        return FALSE;
    }
  }
  return TRUE;
}

bool OneWireSlave::waitReset(uint16_t timeout_ms) {
  IO_REG_TYPE mask = pin_bitmask;
	volatile IO_REG_TYPE *reg IO_REG_ASM = baseReg;
	//volatile IO_REG_TYPE *reg IO_REG_ASM = PIN_TO_BASEREG(_pin);
	
  unsigned long time_stamp;

  errno = ONEWIRE_NO_ERROR;
  noInterrupts();
  DIRECT_MODE_INPUT(reg, mask);
  interrupts();

  //Wait for the line to fall
  if (timeout_ms != 0) {
    time_stamp = micros() + timeout_ms*1000;
    while (DIRECT_READ(reg, mask)) {
      if (micros() > time_stamp) {
        errno = ONEWIRE_WAIT_RESET_TIMEOUT;
        return FALSE;
      }
    }
  } else {
    //Will wait forever for the line to fall
    while (DIRECT_READ(reg, mask)) {};
  }
  
  //Set to wait for rise up to 540 micros
  //Master code sets the line low for 500 micros
  //TODO The actual documented max is 640, not 540
  time_stamp = micros() + 540;
  
  //Wait for the rise on the line up to 540 micros
  while (DIRECT_READ(reg, mask) == 0) {
    if (micros() > time_stamp) {
      errno = ONEWIRE_VERY_LONG_RESET;
      return FALSE;
    }
  }
  
  //If the master pulled low for exactly 500, then this will be 40 wait time
  // Recommended for master is 480, which would be 60 here then
  // Max is 640, which makes this negative, but it returns above as a "ONEWIRE_VERY_LONG_RESET"
  // this gives an extra 10 to 30 micros befor calling the reset invalid
  if ((time_stamp - micros()) > 70) {
    errno = ONEWIRE_VERY_SHORT_RESET;
    return FALSE;
  }
  
  //Master will now delay for 65 to 70 recommended or max of 75 before it's "presence" check
  // and then read the pin value (checking for a presence on the line)
  // then wait another 490 (so, 500 + 64 + 490 = 1054 total without consideration of actual op time) on Arduino, 
  // but recommended is 410 with total reset length of 480 + 70 + 410 (or 480x2=960)
  delayMicroseconds(30);
  //Master wait is 65, so we have 35 more to send our presence now that reset is done
  return TRUE;
}
bool OneWireSlave::waitReset() {
  return waitReset(1000);
}

bool OneWireSlave::presence(uint8_t delta) {
  IO_REG_TYPE mask = pin_bitmask;
	volatile IO_REG_TYPE *reg IO_REG_ASM = baseReg;
	//volatile IO_REG_TYPE *reg IO_REG_ASM = PIN_TO_BASEREG(_pin);

  //Reset code already waited 30 prior to calling this
  // Master will not read until 70 recommended, but could read as early as 60
  // so we should be well enough ahead of that. Arduino waits 65
  errno = ONEWIRE_NO_ERROR;
  noInterrupts();
  DIRECT_WRITE_LOW(reg, mask);
  DIRECT_MODE_OUTPUT(reg, mask);    // drive output low
  interrupts();

  //Delaying for another 125 (orignal was 120) with the line set low is a total of at least 155 micros
  // total since reset high depends on commands done prior, is technically a little longer
  delayMicroseconds(125);
  noInterrupts();
  DIRECT_MODE_INPUT(reg, mask);     // allow it to float
  interrupts();

  //Default "delta" is 25, so this is 275 in that condition, totaling to 155+275=430 since the reset rise
  // docs call for a total of 480 possible from start of rise before reset timing is completed
  //This gives us 50 micros to play with, but being early is probably best for timing on read later
  //delayMicroseconds(300 - delta);
  delayMicroseconds(300 - delta);
  
  //Modified to wait a while (roughly 50 micros) for the line to go high
  // since the above wait is about 430 micros, this makes this 480 closer
  // to the 480 standard spec and the 490 used on the Arduino master code
  // anything longer then is most likely something going wrong.
  uint8_t retries = 25;
  while (!DIRECT_READ(reg, mask));
  do {
	if ( retries-- == 0)
		//return FALSE;
	delayMicroseconds(2); 
  } while(!DIRECT_READ(reg, mask));
  /*
  if ( !DIRECT_READ(reg, mask)) {
      errno = ONEWIRE_PRESENCE_LOW_ON_LINE;
      return FALSE;
  } else
      return TRUE;
  */
}
bool OneWireSlave::presence() {
  return presence(25);
}

uint8_t OneWireSlave::sendData(char buf[], uint8_t len) {
  uint8_t bytes_sended = 0;

  for (int i=0; i<len; i++) {
    send(buf[i]);
    if (errno != ONEWIRE_NO_ERROR)
      break;
    bytes_sended++;
  }
  return bytes_sended;
}

uint8_t OneWireSlave::recvData(char buf[], uint8_t len) {
  uint8_t bytes_received = 0;
  
  for (int i=0; i<len; i++) {
    buf[i] = recv();
    if (errno != ONEWIRE_NO_ERROR)
      break;
    bytes_received++;
  }
  return bytes_received;
}

void OneWireSlave::send(uint8_t v) {
  errno = ONEWIRE_NO_ERROR;
  for (uint8_t bitmask = 0x01; bitmask && (errno == ONEWIRE_NO_ERROR); bitmask <<= 1)
  	sendBit((bitmask & v)?1:0);
}

uint8_t OneWireSlave::recv() {
  uint8_t r = 0;

  errno = ONEWIRE_NO_ERROR;
  for (uint8_t bitmask = 0x01; bitmask && (errno == ONEWIRE_NO_ERROR); bitmask <<= 1)
    if (recvBit())
      r |= bitmask;
  return r;
}

void OneWireSlave::sendBit(uint8_t v) {
  IO_REG_TYPE mask = pin_bitmask;
	volatile IO_REG_TYPE *reg IO_REG_ASM = baseReg;
	//volatile IO_REG_TYPE *reg IO_REG_ASM = PIN_TO_BASEREG(_pin);

  noInterrupts();
  DIRECT_MODE_INPUT(reg, mask);
  //waitTimeSlot waits for a low to high transition followed by a high to low within the time-out
  uint8_t wt = waitTimeSlot();
  if (wt != 1 ) { //1 is success, others are failure
    if (wt == 10) {
      errno = ONEWIRE_READ_TIMESLOT_TIMEOUT_LOW;
    } else {
      errno = ONEWIRE_READ_TIMESLOT_TIMEOUT_HIGH;
    }
    interrupts();
    return;
  }
  if (v & 1)
    delayMicroseconds(30);
  else {
  	noInterrupts();
    DIRECT_WRITE_LOW(reg, mask);
    DIRECT_MODE_OUTPUT(reg, mask);
    delayMicroseconds(30);
    DIRECT_WRITE_HIGH(reg, mask);
    interrupts();
  }
  interrupts();
  return;
}

uint8_t OneWireSlave::recvBit(void) {
  IO_REG_TYPE mask = pin_bitmask;
	volatile IO_REG_TYPE *reg IO_REG_ASM = baseReg;
	//volatile IO_REG_TYPE *reg IO_REG_ASM = PIN_TO_BASEREG(_pin);
  uint8_t r;

  noInterrupts();
  DIRECT_MODE_INPUT(reg, mask);
  //waitTimeSlotRead is a customized version of the original which was also
  // used by the "write" side of things.
  uint8_t wt = waitTimeSlotRead();
  if (wt != 1 ) { //1 is success, others are failure
    if (wt == 10) {
      errno = ONEWIRE_READ_TIMESLOT_TIMEOUT_LOW;
    } else {
      errno = ONEWIRE_READ_TIMESLOT_TIMEOUT_HIGH;
    }
    interrupts();
    return 0;
  }
  delayMicroseconds(30);
  //TODO Consider reading earlier: delayMicroseconds(15);
  r = DIRECT_READ(reg, mask);
  interrupts();
  return r;
}

uint8_t OneWireSlave::waitTimeSlot() {
  IO_REG_TYPE mask = pin_bitmask;
	volatile IO_REG_TYPE *reg IO_REG_ASM = baseReg;
	//volatile IO_REG_TYPE *reg IO_REG_ASM = PIN_TO_BASEREG(_pin);
  uint16_t retries;

  //Wait for a 0 to rise to 1 on the line for timeout duration
  //If the line is already high, this is basically skipped
  retries = TIMESLOT_WAIT_RETRY_COUNT;
  //While line is low, retry
  while ( !DIRECT_READ(reg, mask))
    if (--retries == 0)
      return 10;
          
  //Wait for a fall form 1 to 0 on the line for timeout duration
  retries = TIMESLOT_WAIT_RETRY_COUNT;
  while ( DIRECT_READ(reg, mask));
    if (--retries == 0)
      return 20;

  return 1;
}

//This is a copy of what was orig just "waitTimeSlot"
// it is customized for the reading side of things
uint8_t OneWireSlave::waitTimeSlotRead() {
  IO_REG_TYPE mask = pin_bitmask;
	volatile IO_REG_TYPE *reg IO_REG_ASM = baseReg;
	//volatile IO_REG_TYPE *reg IO_REG_ASM = PIN_TO_BASEREG(_pin);
	
  uint16_t retries;

  //Wait for a 0 to rise to 1 on the line for timeout duration
  //If the line is already high, this is basically skipped
  retries = TIMESLOT_WAIT_RETRY_COUNT;
  //While line is low, retry
  while ( !DIRECT_READ(reg, mask))
    if (--retries == 0)
      return 10;
          
  //TODO Seems to me that the above loop should drop out immediately because
  // The line is already high as our wait after presence is relatively short
  // So now it just waits a short period for the write of a bit to start
  // Unfortunately per "recommended" this is 55 micros to 130 micros more
  // more than what we may have already waited.
          
  //Wait for a fall form 1 to 0 on the line for timeout duration
  retries = TIMESLOT_WAIT_READ_RETRY_COUNT;
  while ( DIRECT_READ(reg, mask));
    if (--retries == 0)
      return 20;

  return 1;
}

#if ONEWIRESLAVE_CRC
// The 1-Wire CRC scheme is described in Maxim Application Note 27:
// "Understanding and Using Cyclic Redundancy Checks with Maxim iButton Products"

#if ONEWIRESLAVE_CRC8_TABLE
// This table comes from Dallas sample code where it is freely reusable,
// though Copyright (C) 2000 Dallas Semiconductor Corporation
static const uint8_t PROGMEM dscrc_table[] = {
      0, 94,188,226, 97, 63,221,131,194,156,126, 32,163,253, 31, 65,
    157,195, 33,127,252,162, 64, 30, 95,  1,227,189, 62, 96,130,220,
     35,125,159,193, 66, 28,254,160,225,191, 93,  3,128,222, 60, 98,
    190,224,  2, 92,223,129, 99, 61,124, 34,192,158, 29, 67,161,255,
     70, 24,250,164, 39,121,155,197,132,218, 56,102,229,187, 89,  7,
    219,133,103, 57,186,228,  6, 88, 25, 71,165,251,120, 38,196,154,
    101, 59,217,135,  4, 90,184,230,167,249, 27, 69,198,152,122, 36,
    248,166, 68, 26,153,199, 37,123, 58,100,134,216, 91,  5,231,185,
    140,210, 48,110,237,179, 81, 15, 78, 16,242,172, 47,113,147,205,
     17, 79,173,243,112, 46,204,146,211,141,111, 49,178,236, 14, 80,
    175,241, 19, 77,206,144,114, 44,109, 51,209,143, 12, 82,176,238,
     50,108,142,208, 83, 13,239,177,240,174, 76, 18,145,207, 45,115,
    202,148,118, 40,171,245, 23, 73,  8, 86,180,234,105, 55,213,139,
     87,  9,235,181, 54,104,138,212,149,203, 41,119,244,170, 72, 22,
    233,183, 85, 11,136,214, 52,106, 43,117,151,201, 74, 20,246,168,
    116, 42,200,150, 21, 75,169,247,182,232, 10, 84,215,137,107, 53};

//
// Compute a Dallas Semiconductor 8 bit CRC. These show up in the ROM
// and the registers.  (note: this might better be done without to
// table, it would probably be smaller and certainly fast enough
// compared to all those delayMicrosecond() calls.  But I got
// confused, so I use this table from the examples.)
//
uint8_t OneWireSlave::crc8(char addr[], uint8_t len) {
  uint8_t crc = 0;

  while (len--) {
    crc = pgm_read_byte(dscrc_table + (crc ^ *addr++));
  }
  return crc;
}
#else
//
// Compute a Dallas Semiconductor 8 bit CRC directly.
//
uint8_t OneWireSlave::crc8(char addr[], uint8_t len) {
  uint8_t crc = 0;
  
  while (len--) {
    uint8_t inbyte = *addr++;
    for (uint8_t i = 8; i; i--) {
      uint8_t mix = (crc ^ inbyte) & 0x01;
      crc >>= 1;
      if (mix) crc ^= 0x8C;
        inbyte >>= 1;
    }
  }
  return crc;
}
#endif

#endif
