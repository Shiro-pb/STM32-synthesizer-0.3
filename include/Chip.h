#ifndef CHIP_H
#define CHIP_H

/*-----------------------------------------------------------------------------*/
/*--------------------          includes           ----------------------------*/
/*-----------------------------------------------------------------------------*/

#include "stdint.h"
#include "stdlib.h"
#include "stdbool.h"
#include "stm32f1xx_hal.h"

/*-----------------------------------------------------------------------------*/
/*--------------------          defines            ----------------------------*/
/*-----------------------------------------------------------------------------*/

#define PI 3.141592
#define sampleTime 25000 //29997 -> 0x960
#define channelNumber 3

/*-----------------------------------------------------------------------------*/
/*--------------------          structs             ---------------------------*/
/*-----------------------------------------------------------------------------*/

typedef struct {
    GPIO_TypeDef * port;
    uint16_t pin;
} GPIOPIN;

typedef struct {
  unsigned int count;
  unsigned int atackCount;
  unsigned int decayCount;
  unsigned int releaseCount;
  

} EnvelopeTimers;

typedef struct {
  uint8_t noteValue; // to store the note that is playing
  uint8_t noteVelocity;//to store the velocity in which the key was pressed
  uint16_t channelVolume;// the current volume of the channel 
  uint8_t gate; // key is pressed / logic for drum envelope
  double phase; // used to handle the wave synthesis
  int envelopeState; // stores the state of the envelope generator (0-atack/1-decay/2-sustain/3-release)
  EnvelopeTimers envelopeTimers;//parameters for the envelope
  //bool busy; 
} MIDIChannel;

typedef struct {
  bool recOn;
  bool holdOn;
  bool arpeggioOn; 
  double arpSpeed;
  double seqSpeed;
  int mode;
  bool modeTrackNow;
  bool modeTrackOld;
  unsigned int atackValue;
  unsigned int decayValue;
  unsigned int releaseValue;
} SynthState;

typedef struct {
  bool event1;
  uint8_t noteValue1;
  uint8_t gate1;
  bool event2;
  uint8_t noteValue2;
  uint8_t gate2;
} EventMatrix;

/*-----------------------------------------------------------------------------*/
/*--------------------          variables           ---------------------------*/
/*-----------------------------------------------------------------------------*/

//DAC pins
GPIOPIN outputPin0;
GPIOPIN outputPin1;
GPIOPIN outputPin2;
GPIOPIN outputPin3;
GPIOPIN outputPin4;
GPIOPIN outputPin5;
GPIOPIN outputPin6;
GPIOPIN outputPin7;
GPIOPIN outputPin8;
GPIOPIN outputPin9;
GPIOPIN outputPin10;
GPIOPIN outputPin11;

//input pins
GPIOPIN vibratoPin;
GPIOPIN burstPin;
GPIOPIN arpeggioPin;
GPIOPIN modePin;
GPIOPIN loopPin;
GPIOPIN dutyCyclePin;
GPIOPIN pot2Pin;

//display pins
GPIOPIN dispPin0;
GPIOPIN dispPin1;
GPIOPIN dispPin2;
GPIOPIN dispPin3;
GPIOPIN dispPin4;
GPIOPIN dispPin5;
GPIOPIN dispPin6;

//tables


/*-----------------------------------------------------------------------------*/
/*--------------------          prototypes          ---------------------------*/
/*-----------------------------------------------------------------------------*/

uint16_t square_wave(uint8_t peakValue, double frequency, double * phase, double dutyCycle);
void sendOutput(uint16_t audioIn);
uint16_t triangle_wave(uint8_t peakValue, double frequency, double * phase);
void DACOUTInit(void);
void inputPinsInit(void);
void displayInit(void);
void chipophoneInit();
uint16_t noise_generator ();
uint8_t gaussNoise();
uint16_t midiChannelOutputCompute(MIDIChannel * midiChannel, SynthState inputState);
double applyEnvelope( MIDIChannel * inputChannel, SynthState synthState);
double drumEnvelope(MIDIChannel * inputChannel, int atackValue, int decayValue, int releaseValue);
uint16_t drumGenerator(MIDIChannel * inputChannel, SynthState synthState);
void synthAtt(SynthState * inputState);
uint16_t analogReadADC1();
void sequencerCompute(void);
void sequencerStart(void);
void sequencerInsertEvent(void);

#endif