#include <avr/wdt.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <Arduino.h>
#include "tpms_silencer.h"

/*
Each bit/symbol: 0/1 is 100us long. sent at a rate of 10kHz
144 symbols sent in each TPMS packet: taking 14.4ms (+ 200us warmup)
All Packets are sent 40ms apart, then we deep-sleep.
If PERIODIC_TX is defined then wakeup after 7s * wakeuplimit (which is potentially variable)
BACKOFF will double the retransmit period each time until MAX_LIMIT.
*/

#define PIN_EN          PA0  // PA0, Pin 13, Arduino 10
#define PIN_FSK         PA1 // PA1, Pin 12, Arduino 9
#define PIN_ASK         PA2 // PA2, Pin 11, Arduino 8
#define PIN_BUTTON      PA7 // PA7, Pin 6, PCINT7
#define INTERRUPT_PIN   PCINT7 // interrupt for PA7

#define PERIODIC_TX // periodic retransmissions. disable for button-press tx only
//#define BACKOFF    // double transmit period everytime? disable to have a fixed period retrans.

#define ENHIGH (bitSet(PORTA, 0))
#define ENLOW (bitClear(PORTA, 0))

#define ASKHIGH (bitSet(PORTA, 2))
#define ASKLOW (bitClear(PORTA, 2))

#define FSKHIGH (bitSet(PORTA, 1))
#define FSKLOW (bitClear(PORTA, 1))
#define FSKTOGGLE (PORTA = PORTA ^ _BV(1))


#define PACKET_DELAY   40 // Milliseconds inter-packet period
#ifdef BACKOFF
#define LIMIT_START     1 // (re-transmit intervals: 14s, 28s, 56s, ..., MAX_LIMIT/7)
//#define MAX_LIMIT      80 // 80 max limit ~ 10mins.
#define MAX_LIMIT     160 // 160 max limit ~ 20 mins.
//#define MAX_LIMIT  0xffff // uint16_t max limit ~ 5 days.
#else
//#define LIMIT_START     8 //   8*(7s) = ~60s wakeups to transmit
#define LIMIT_START     120 // 120*(7s) = ~15mins wakeups to transmit
#endif


// Use the compact encoding reported by rtl_433 -vv  <-f 315M -R 110>
// could halve this by storing the Differential-Manchester decoded values...
// Or even better generate packets on the fly, based on id, pressure/temp, etc.
#define NO_PACKETS      1
#define PACKET_LEN      2
#define PACKETSIZE    (PACKET_LEN*8)
const unsigned char packets[NO_PACKETS][PACKET_LEN] = { 
  // packet 0
  { B10000110, B11110000}};

volatile unsigned char packet=0; // which packet is being sent 0 - NO_PACKETS-1
volatile unsigned int currentBit; // which bit in that packet is next

volatile bool transmitting = false; // indicates if transmitting a packet

volatile unsigned int wakeupCounter = 0; // how many 8s watchdog timeouts since last transmit
volatile unsigned int wakeuplimit = LIMIT_START;  // How many 8-second wakeups before transmit again


// setup functions

// 10khz Interrupt for an 8MHz clock
void setupInterrupt1()
{
  noInterrupts();
  // Clear registers
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;

  // 10000 Hz (8000000/((99+1)*8))
  OCR1A = 99;
  // CTC
  TCCR1B |= (1 << WGM12);
  // Prescaler 1
  // TCCR1B |= (1 << CS11);
  // Output Compare Match A Interrupt Enable
  TIMSK1 |= (1 << OCIE1A);

  // button interrupt
  PCMSK0 |= (1 << INTERRUPT_PIN);
  GIMSK |= (1 << PCIE0 );

  interrupts();
}

// 10khz Interrupt for an 8MHz clock
void setupInterrupt8()
{
  noInterrupts();
  // Clear registers
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;

  // 10000 Hz (8000000/((99+1)*8))
  OCR1A = 99;
  // CTC
  TCCR1B |= (1 << WGM12);
  // Prescaler 8
  TCCR1B |= (1 << CS11);
  // Output Compare Match A Interrupt Enable
  TIMSK1 |= (1 << OCIE1A);

  // button interrupt
  PCMSK0 |= (1 << INTERRUPT_PIN);
  GIMSK |= (1 << PCIE0 );

  interrupts();
}

// 10khz Interrupt for a 16MHz clock
void setupInterrupt16()
{
  noInterrupts();
  // Clear registers
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;

  // 10000 Hz (16000000/((24+1)*64))
  OCR1A = 24;
  // CTC
  TCCR1B |= (1 << WGM12);
  // Prescaler 64
  TCCR1B |= (1 << CS11) | (1 << CS10);
  // Output Compare Match A Interrupt Enable
  TIMSK1 |= (1 << OCIE1A);

  // button interrupt
  PCMSK0 |= (1 << INTERRUPT_PIN);
  GIMSK |= (1 << PCIE0 );

  interrupts();
}

void setup()
{
  for (byte i = 0; i < 13; i++)
    pinMode(i, INPUT);

  // disable ADC + other stuff that uses power
  ADCSRA = 0;
  power_spi_disable();
  power_usart0_disable();
  power_usart1_disable();
  power_timer2_disable();
  power_twi_disable();
  // find more in power.h

  pinMode(PIN_EN, OUTPUT);
  pinMode(PIN_FSK, OUTPUT);
  pinMode(PIN_ASK, OUTPUT);
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  
#ifdef PERIODIC_TX
  /* Clear the reset flags. */
  MCUSR = 0;

  /* set watchdog interrupt with prescalers */
  WDTCSR = 1<<WDP0 | 1<<WDP3 | 1<<WDIE | 1<<WDE; /* 8.0 seconds */ 
#endif

#if F_CPU == 16000000L
  setupInterrupt16();
#elif F_CPU == 8000000L
  setupInterrupt8();
#elif F_CPU == 1000000L
  setupInterrupt1();
#else
#error CPU is not set to 16MHz or 8MHz!
#endif
  disableTX();

  wakeupCounter = wakeuplimit; // force an immediate transmit
}

// runtime functions

// 100 usec timer for bit tx
ISR(TIMER1_COMPA_vect)
{
  if (transmitting)
  {
    if (currentBit >= PACKETSIZE)
    {
      disableTX();
      return;
    }

    // read off bits in bigendian order
    if ( ( packets[packet][currentBit/8] & ( 0x80 >> (currentBit % 8) ) ) )
      ASKLOW;
    else
      ASKHIGH;
    
    currentBit++;
  }
}

#ifdef PERIODIC_TX
// watchdog timer is setup by setupInterrupt8(). kicks roughly 7-8s at 3v.
ISR(WDT_vect)
{
  // reset WDIE to ensure next watchdog uses the interrupt too (dont reboot!)
  WDTCSR |= bit(WDIE);
  wakeupCounter++;
}
#endif

// button interrupt is setup by setupInterrupt8()
ISR(PCINT0_vect)
{
  // could use a +300ms debounce here, but its not critcal if we transmit packets twice...
  if( digitalRead(PIN_BUTTON) == LOW ) {
    // button press. wakeup, transmit and set initial retransmit period
    wakeuplimit = LIMIT_START;
    wakeupCounter = wakeuplimit;
  //} else {
    // button release
  }
}

// sleep until an WDT/button interrupt fires
void sleepyTime()
{
  // Just in case
  disableTX();

  set_sleep_mode(SLEEP_MODE_PWR_DOWN);

  sleep_enable();
  sleep_mode();
  sleep_disable();
}

// queue up and prepare to send a packet
void sendPacket(unsigned int which)
{
  packet = min( which, NO_PACKETS-1 );
  currentBit = 0;

  enableTX();
}

// warm-up transmitter and enable bit-period interrupt
void enableTX()
{
  FSKHIGH;
  ASKHIGH;
  ENHIGH;

  delayMicroseconds(200); // 200uS+ to wake up (similar to captured tpms)
  transmitting = true;
  power_timer1_enable();
}

// disable transmitter and disable bit-period interrupt
void disableTX()
{
  ENLOW;
  FSKHIGH;
  transmitting = false;
  power_timer1_disable();
}

void loop()
{
  if (wakeupCounter >= wakeuplimit)
  {
    for (int i=0; i<NO_PACKETS; i++) {
      sendPacket(i);
      delay(PACKET_DELAY);
    }

    // reset to wait for next tx
    wakeupCounter = 0;

#ifdef BACKOFF
    // double the wakeuplimit each time to back-off and save battery
    wakeuplimit *= 2;
#ifdef MAX_LIMIT
    // but only up to this limit:
    wakeuplimit = min(wakeuplimit, MAX_LIMIT);
#endif
#endif
  }

  sleepyTime();
}
