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

#define ENHIGH (bitSet(PORTA, PIN_EN))
#define ENLOW (bitClear(PORTA, PIN_EN))

#define ASKHIGH (bitSet(PORTA, PIN_ASK))
#define ASKLOW (bitClear(PORTA, PIN_ASK))

#define FSKHIGH (bitSet(PORTA, PIN_FSK))
#define FSKLOW (bitClear(PORTA, PIN_FSK))

#define PACKET_DELAY 100 // Milliseconds inter-packet period
#define LIMIT_START 120  // 120*(7s) = ~15mins wakeups to transmit

#define SHORTPACKETUS 410  // Short packet duration in uS
#define LONGPACKETUS 860   // Long packet duration in uS
#define TOTALPACKETUS 1250 // Total packet duration in uS for fix-bit width PWM

#define WEIRDLONGPACKETUS 1220 // Fixed preamble duration in uS
#define WEIRDSHORTPACKETUS 820 // Longer lows during system address
#define BREAKPACKETUS 2000 // Break between packets in uS
#define TWEAK 23           // Typically this will be your period (eg 50kHz interrupt, 20uS)
                           // but you can fine tune as needed

#define PACKET_RETRANSMIT 24 // How many times to resend the same packet?
#define PACKET_SIZE 13 // We only actually want 13 bits here

/*
             xxxx xx98 7654 3210
Bit position 1234 5678 1234 5678
             IIII IISS CTTT P000
*/

#define BIT_SYS1   9
#define BIT_SYS0   8
#define BIT_ONOFF  7
#define BIT_TOOL2  6
#define BIT_TOOL1  5
#define BIT_TOOL0  4
#define BIT_PARITY 3
#define FIXED      33792  // This is the fixed preamble

// on is
// IIII IISS CTTT P 000
// 1000 0100 1111 1 000
// off is
// 0b 10000100 0b0111 0 000

// This is where you define your on/off packet. You really need to only change the TOOL bits and the SYS bits.
// Even number of 1's? Set the bit parity flag.
const unsigned short onPacket = FIXED | (1 << BIT_ONOFF) | (1 << BIT_TOOL2) | (1 << BIT_TOOL1) | (1 << BIT_TOOL0) | (1 << BIT_PARITY);
const unsigned short offPacket = FIXED | (1 << BIT_TOOL2) | (1 << BIT_TOOL1) | (1 << BIT_TOOL0);

volatile unsigned short currentPacket = onPacket;
volatile unsigned int currentBit;  // which bit in that packet is next

volatile bool transmitting = false; // indicates if transmitting a packet

volatile unsigned int wakeupCounter = 0;         // how many 8s watchdog timeouts since last transmit
volatile unsigned int wakeuplimit = LIMIT_START; // How many 8-second wakeups before transmit again

volatile unsigned int tickCounter = 0; // count the number of interrupts, so we can determine how long to pulse for
volatile unsigned int sendLow = 0;     // Tell us to send the low signal
volatile unsigned int transmitCount;   // Countdown our transmit resends
volatile unsigned int previousBit = 0;

// setup functions

void calculateParity(unsigned short data)
{
  bool parityVal = false;
  for (int bit = 0; bit < 12; bit++)
  {
    // Calculate the parity of the first 12 bits in data, and store it in the 13th bit
    if (data & (1 << bit))
      parityVal = !parityVal;
  }
  data |= parityVal << 12;
}

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
      if (currentBit < 6)  // Preamble
        tickCounter = SHORTPACKETUS / TWEAK;
      else // We are done the preamble, go to fixed width
      {
        previousBit = currentPacket & (0x8000 >> (currentBit - 1));
        // If the previous bit (the one we are currently transmitting) is a 1, we need to send a shorter low pulse then a 0 bit.
        if (previousBit)
          tickCounter = (TOTALPACKETUS - LONGPACKETUS) / TWEAK;
        else
          tickCounter = (TOTALPACKETUS - SHORTPACKETUS) / TWEAK;
      }
        
      return;
    }

    if (currentBit >= PACKET_SIZE) // end of packet
    {
      disableTX();
      return;
    }

    // read off bits in bigendian order
    if (currentPacket & (0x8000 >> currentBit))
    {
      // We need to send a 1 which means be high for LONGPACKETUS, then be low for SHORTPACKETUS
      ASKHIGH;
      // If we are doing the preamble, then we use the weird longer length
      if (currentBit < 6)
        tickCounter = WEIRDLONGPACKETUS / TWEAK;
      else
        tickCounter = LONGPACKETUS / TWEAK;

      sendLow = 1;
    }
    else
    {
      ASKHIGH;
      tickCounter = SHORTPACKETUS / TWEAK;
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
void sendPacket(unsigned short whichPacket)
{
  currentPacket = whichPacket;
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
      sendPacket(currentPacket);
      delay(PACKET_DELAY);
    }
    // reset to wait for next tx
    wakeupCounter = 0;

  if (currentPacket == onPacket)
    currentPacket = offPacket;
  else
    currentPacket = onPacket;

  }


  sleepyTime();
}
