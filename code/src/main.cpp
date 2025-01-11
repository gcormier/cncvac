#include <avr/wdt.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <Arduino.h>
#include "tpms_silencer.h"

#define PIN_EN PA0     // PA0, Pin 13, Arduino 10
#define PIN_FSK PA1    // PA1, Pin 12, Arduino 9
#define PIN_ASK PA2    // PA2, Pin 11, Arduino 8
#define PIN_BUTTON PA7 // PA7, Pin 6, PCINT7
#define PIN_EXT_TRIG PA3

#define INTERRUPT_PIN PCINT7  // interrupt for PA7
#define INTERRUPT_PIN2 PCINT3 // interrupt for PA3
// #define BACKOFF    // double transmit period everytime? disable to have a fixed period retrans.

#define ENHIGH (bitSet(PORTA, PIN_EN))
#define ENLOW (bitClear(PORTA, PIN_EN))

#define ASKHIGH (bitSet(PORTA, PIN_ASK))
#define ASKLOW (bitClear(PORTA, PIN_ASK))
#define ASKTOGGLE (PORTA = PORTA ^ _BV(PIN_ASK))

#define FSKHIGH (bitSet(PORTA, PIN_FSK))
#define FSKLOW (bitClear(PORTA, PIN_FSK))
#define FSKTOGGLE (PORTA = PORTA ^ _BV(PIN_FSK))

#define PACKET_DELAY 100 // Milliseconds inter-packet period
#define LIMIT_START 120  // 120*(7s) = ~15mins wakeups to transmit

#define SHORTPACKETUS 350  // Short packet duration in uS
#define LONGPACKETUS 1000   // Long packet duration in uS
#define BREAKPACKETUS 2000 // Break between packets in uS
#define TWEAK 20           // Typically this will be your period (eg 50kHz interrupt, 20uS)
                           // but you can fine tune as needed

#define PACKET_RETRANSMIT 20 // How many times to resend the same packet?
#define NO_PACKETS 1
#define PACKET_LEN 2
// #define PACKETSIZE (PACKET_LEN * 8)
#define PACKETSIZE 13 // We only actually want 13 bits here

// on is
// 0b 10000100 0b11111000
// off is
// 0b 10000100 0b01110000

const unsigned char packets[NO_PACKETS][PACKET_LEN] = {
    // packet 0
    {B10000100, B11111000}};

volatile unsigned char packet = 0; // which packet is being sent 0 - NO_PACKETS-1
volatile unsigned int currentBit;  // which bit in that packet is next

volatile bool transmitting = false; // indicates if transmitting a packet

volatile unsigned int wakeupCounter = 0;         // how many 8s watchdog timeouts since last transmit
volatile unsigned int wakeuplimit = LIMIT_START; // How many 8-second wakeups before transmit again

volatile unsigned int tickCounter = 0; // count the number of interrupts, so we can determine how long to pulse for
volatile unsigned int sendLow = 0;     // Tell us to send the low signal
volatile unsigned int transmitCount;   // Countdown our transmit resends
// setup functions

// 50khz Interrupt for an 4MHz clock
// 50khz = 20uS
void setupInterrupt4()
{
  noInterrupts();
  // Clear registers
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;

  OCR1A = 79;

  // CTC
  TCCR1B |= (1 << WGM12);

  // Prescaler 1
  TCCR1B |= (1 << CS00);

  // Output Compare Match A Interrupt Enable
  TIMSK1 |= (1 << OCIE1A);

  // button interrupt
  PCMSK0 |= (1 << INTERRUPT_PIN);

  // Opto interrupt
  PCMSK0 |= (1 << INTERRUPT_PIN2);
  GIMSK |= (1 << PCIE0);

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
  GIMSK |= (1 << PCIE0);

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
  GIMSK |= (1 << PCIE0);

  interrupts();
}

void setup()
{
  // We have an 8MHz, and we have CKDIV8 fuse set, so we are running at 1MHz
  // This is to allow a safe startup with unknown power conditions. Then, we immediately
  // switch the scaler to 4MHz for a bit better performance, but we can still operate
  // down to 1.8V (datasheet page 238)
  cli();
  CLKPR = (1 << CLKPS0); // Set prescaler to divide by 2 (8 MHz / 2 = 4 MHz)
  sei();                 // Re-enable interrupts
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

#if F_CPU == 16000000L
  setupInterrupt16();
#elif F_CPU == 8000000L
  setupInterrupt8();
#elif F_CPU == 4000000L
  setupInterrupt4();
#else
#error CPU is not set to 16MHz or 8MHz!
#endif
  disableTX();

  // wakeupCounter = wakeuplimit; // force an immediate transmit
}

// runtime functions

// 100 usec timer for bit tx
ISR(TIMER1_COMPA_vect)
{
  if (transmitting)
  {
    if (tickCounter > 0) // If we need to keep this signal high for longer, keep going
    {
      tickCounter--;
      return;
    }
    else if (tickCounter == 0 && sendLow == 1)
    {
      sendLow = 0;
      ASKLOW;
      tickCounter = SHORTPACKETUS / TWEAK;
      return;
    }

    if (currentBit >= PACKETSIZE) // end of packet
    {
      disableTX();
      return;
    }

    // read off bits in bigendian order
    if ((packets[packet][currentBit / 8] & (0x80 >> (currentBit % 8))))
    {
      // We need to send a 1 which means be high for LONGPACKETUS, then be low for SHORTPACKETUS
      ASKHIGH;
      tickCounter = LONGPACKETUS / TWEAK; // We will lose some precision here, but it's close enough
      // tickCounter = 30;
      sendLow = 1;
    }
    else
    {
      ASKHIGH;
      tickCounter = SHORTPACKETUS / TWEAK; // We will lose some precision here, but it's close enough
      // tickCounter = 4;
      sendLow = 1;
    }

    currentBit++;
  }
}

// button interrupt is setup by setupInterrupt8()
ISR(PCINT0_vect)
{
  // could use a +300ms debounce here, but its not critcal if we transmit packets twice...
  if (digitalRead(PIN_BUTTON) == LOW)
  {
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
  packet = min(which, NO_PACKETS - 1);
  currentBit = 0;

  enableTX();
}

// warm-up transmitter and enable bit-period interrupt
void enableTX()
{
  FSKLOW;
  ASKLOW;
  ENHIGH;

  delayMicroseconds(50); // warm-up time
  transmitting = true;
  power_timer1_enable();
}

// disable transmitter and disable bit-period interrupt
void disableTX()
{
  FSKLOW;
  ENLOW;
  ASKLOW;
  transmitting = false;
  power_timer1_disable();
}

void loop()
{
  if (wakeupCounter >= wakeuplimit)
  {
    for (transmitCount = 0; transmitCount < PACKET_RETRANSMIT; transmitCount++)
    {
      for (int i = 0; i < NO_PACKETS; i++)
      {
        sendPacket(i);
        // delay(100);
      }
      delay(PACKET_DELAY);
    }
    // reset to wait for next tx
    wakeupCounter = 0;
  }

  sleepyTime();
}
