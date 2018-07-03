#include "TinyWire/TinyWireS/TinyWireS.h"

#define I2C_SLAVE_ADDRESS 0x04 // 7-bit Address of this device

// default buffer size (copying from example code)
#ifndef TWI_RX_BUFFER_SIZE
#define TWI_RX_BUFFER_SIZE ( 16 )
#endif

#include <avr/interrupt.h>
#define INTERRUPTPIN PCINT4 // this is PB4 per the schematic

#define NUM_CHANNELS 8

// I2C registers (1 for each PWM channel)
volatile uint8_t i2c_regs[] =
{
  0,
  0, // channel 1
  0,
  0, // channel 2
  0,
  0, // channel 3
  0,
  0, // channel 4
  0,
  0, // channel 5
  0,
  0, // channel 6
  0,
  0, // channel 7
  0,
  0   //channel 8
};

// Tracks the current register pointer position
volatile byte reg_position;
const byte reg_size = sizeof(i2c_regs);

int8_t channel_counter;
uint16_t pulse_capt[NUM_CHANNELS];

/**
 * Process a single PPM pulse
 * 
 * Code adapted from AP_HAL_Linux/RCInput.cpp
 */
void process_ppmsum_pulse(uint16_t width_usec)
{
  if(width_usec >= 2700) {
    // a long pulse indicates the end of a frame.  Rest the
    // channel counter so next pulse is channel 0
    if(channel_counter >= NUM_CHANNELS) {
      for (uint8_t i=0; i<channel_counter; i++) {
        i2c_regs[2*i] = (uint8_t) (pulse_capt[i] >> 8);
        i2c_regs[2*i+1] = (uint8_t) (pulse_capt[i] & 0xff);
      }
    }
    channel_counter = 0;
    return;
  }
  if (channel_counter == -1) {
    // we are not synchronised
    return;
  }

  // read the value
  pulse_capt[channel_counter] = width_usec;
  channel_counter += 1;

  // if we reached the maximum supported channels then
  // mark as unsynchronised, so we wait for a wide pulse
  if (channel_counter >= NUM_CHANNELS) {
    for (uint8_t i=0; i<channel_counter; i++) {
      i2c_regs[2*i] = (uint8_t) (pulse_capt[i] >> 8);
      i2c_regs[2*i+1] = (uint8_t) (pulse_capt[i] & 0xff);
    }
    channel_counter = -1;
  }
}

uint16_t last_time_us = 0;

ISR(PCINT0_vect)
{
  if(digitalRead(4) == HIGH) {
    uint32_t now = micros();
    uint32_t delta_time_us = now - last_time_us;
    last_time_us = now;
    process_ppmsum_pulse(delta_time_us);
  }
}

/**
 * This is called for each read request we receive, never put more than one byte of data (with TinyWireS.send) to the 
 * send-buffer when using this callback
 */
void requestEvent()
{  
    // we have a special feature that will send back all the registers
    if(reg_position > reg_size) 
    {
      for(uint8_t i=0; i<reg_size; i++) {
        TinyWireS.send(i2c_regs[i]);
      }
    } else 
    {
      TinyWireS.send(i2c_regs[reg_position]);
      // Increment the reg position on each read, and loop back to zero
      reg_position++;
    }
    if (reg_position >= reg_size)
    {
        reg_position = 0;
    }
}

/**
 * The I2C data received -handler
 *
 * This needs to complete before the next incoming transaction (start, data, restart/stop) on the bus does
 * so be quick, set flags for long running tasks to be called from the mainloop instead of running them directly,
 */
void receiveEvent(uint8_t howMany)
{
    if (howMany < 1)
    {
        // Sanity-check
        return;
    }
    if (howMany > TWI_RX_BUFFER_SIZE)
    {
        // Also insane number
        return;
    }

    reg_position = TinyWireS.receive();
    howMany--;
    if (!howMany)
    {
        // This write was only to set the buffer for next read
        return;
    }
    while(howMany--)
    {
        i2c_regs[reg_position] = TinyWireS.receive();
        reg_position++;
        if (reg_position >= reg_size)
        {
            reg_position = 0;
        }
    }
}

void setup() {
  // put your setup code here, to run once:
  cli();

  PCMSK |= (1 << INTERRUPTPIN);// tell pin change mask to listen to pin3 /pb4
  GIMSK |= (1 << PCIE);  // enable PCINT interrupt in the general interrupt mask

  TinyWireS.begin(I2C_SLAVE_ADDRESS);
  TinyWireS.onReceive(receiveEvent);
  TinyWireS.onRequest(requestEvent);

  // PIN 4 is PPM input
  pinMode(4, INPUT);
  sei();
}

void loop() {
  // put your main code here, to run repeatedly:
  TinyWireS_stop_check();

}
