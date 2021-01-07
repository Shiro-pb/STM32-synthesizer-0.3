#include "Chip.h"
#include "stdint.h"
#include "stdlib.h"
#include "stdbool.h"
#include "stm32f1xx_hal.h"
#include "math.h"


double noteTable[] = {32.703, 34.648, 36.708, 38.891, 41.203, 43.654, 46.249, 48.999, 41.913, //9
 55, 58.270, 61.735, 65.406, 69.296, 73.416, 77.782, 82.407, 87.307, 92.499, 97.999, //20
  103.83, 110.0, 116.54, 123.47, 130.81, 138.59, 146.83, 155.56, 164.81, 174.61, 185.0, //31
   220.0, 207.65, 246.94, 261.63, 277.18, 293.67, 311.13, 329.63, 349.23, 369.99, 392, 415.3, 440, 466.16, //45
493.88, 523.25, 554.37, 587.33, 622.25, 659.26, 698.46, 739.99, 783.99, 830.61, 880, 932.33, 987.77, //58
1046.5, 1108.7, 1174.7, 1244.5, 1318.5, 1396.9, 1480.0, 1568.0, 1661.2, 1760.0, 1864.7, 1975.5, 2093.0, //71
 2217.5, 2349.3, 2489.0, 2637.0, 2793.0, 2960.0, 3136.0, 3322.0, 3520.0, 3729.3, 3951.1}; //82

float atackTable[] = { 0.06, 0.12, 0.18, 0.24, 0.30, 0.36, 0.42, 0.48, 0.54, 0.60, 0.66, 0.72, 0.78, 0.84, 0.90, 0.96, 1.02, 1.08, 1.14, 1.20, 1.26, 1.32, 1.38, 1.42};
float decayTable[] = { 1.38, 1.32, 1.26, 1.14, 1.08, 1.02};
float releaseTable[] = {1, 0.96, 0.90, 0.84, 0.78, 0.72, 0.66, 0.60, 0.54, 0.48, 0.42, 0.36, 0.30, 0.24, 0.18, 0.12, 0.06};

extern double squarePhase;

//double * noteTable2 = (double*)malloc(15);
//initialization functions
void DACOUTInit(void)
{
  outputPin0.port = GPIOA;
  outputPin0.pin = GPIO_PIN_4;

  outputPin1.port = GPIOA;
  outputPin1.pin = GPIO_PIN_5;

  outputPin2.port = GPIOA;
  outputPin2.pin = GPIO_PIN_6;

  outputPin3.port = GPIOA;
  outputPin3.pin = GPIO_PIN_7;

  outputPin4.port = GPIOB;
  outputPin4.pin = GPIO_PIN_0;

  outputPin5.port = GPIOB;
  outputPin5.pin = GPIO_PIN_1;

  outputPin6.port = GPIOB;
  outputPin6.pin = GPIO_PIN_10;

  outputPin7.port = GPIOB;
  outputPin7.pin = GPIO_PIN_11;

  outputPin8.port = GPIOB;
  outputPin8.pin = GPIO_PIN_12;

  outputPin9.port = GPIOB;
  outputPin9.pin = GPIO_PIN_13;

  outputPin10.port = GPIOB;
  outputPin10.pin = GPIO_PIN_14;

  outputPin11.port = GPIOB;
  outputPin11.pin = GPIO_PIN_15;

}

void inputPinsInit(void)
{
  vibratoPin.port = GPIOA;
  vibratoPin.pin = GPIO_PIN_9;

  burstPin.port = GPIOA;
  burstPin.pin = GPIO_PIN_8;

  arpeggioPin.port = GPIOC;
  arpeggioPin.pin = GPIO_PIN_14;

  modePin.port = GPIOA;
  modePin.pin = GPIO_PIN_10;

  loopPin.port = GPIOA;
  loopPin.pin = GPIO_PIN_11;

  dutyCyclePin.port = GPIOA;//pot 1
  dutyCyclePin.pin = GPIO_PIN_0;// pot 1

  pot2Pin.port = GPIOA;
  pot2Pin.pin = GPIO_PIN_1;
}

void displayInit(void)
{
  dispPin0.port = GPIOA;
  dispPin0. pin = GPIO_PIN_12;

  dispPin1.port = GPIOB;
  dispPin1. pin = GPIO_PIN_4;

  dispPin2.port = GPIOB;
  dispPin2. pin = GPIO_PIN_5;

  dispPin3.port = GPIOB;
  dispPin3. pin = GPIO_PIN_6;

  dispPin4.port = GPIOB;
  dispPin4. pin = GPIO_PIN_7;

  dispPin5.port = GPIOB;
  dispPin5. pin = GPIO_PIN_8;

  dispPin6.port = GPIOB;
  dispPin6. pin = GPIO_PIN_9;
}
/*
void midiChannelInit(){
  channel1.index = 0;
  channel2.index = 1;
  channel3.index = 2;
  channel4.index = 3;
  channel5.index = 4;
}*/
void chipophoneInit(){
  DACOUTInit();
  inputPinsInit();
  displayInit();
  //midiChannelInit();
  //srand(14164);
  srand(918735);
}


//send output

void sendOutput(uint16_t audioIn)
{
    HAL_GPIO_WritePin( outputPin0.port, outputPin0.pin, (audioIn & 0x01));
    HAL_GPIO_WritePin( outputPin1.port, outputPin1.pin, ((audioIn >> 1) & 0x01));
    HAL_GPIO_WritePin( outputPin2.port, outputPin2.pin, ((audioIn >> 2) & 0x01));
    HAL_GPIO_WritePin( outputPin3.port, outputPin3.pin, ((audioIn >> 3) & 0x01));
    HAL_GPIO_WritePin( outputPin4.port, outputPin4.pin, ((audioIn >> 4) & 0x01));
    HAL_GPIO_WritePin( outputPin5.port, outputPin5.pin, ((audioIn >> 5) & 0x01));
    HAL_GPIO_WritePin( outputPin6.port, outputPin6.pin, ((audioIn >> 6) & 0x01));
    HAL_GPIO_WritePin( outputPin7.port, outputPin7.pin, ((audioIn >> 7) & 0x01));
    HAL_GPIO_WritePin( outputPin8.port, outputPin8.pin, ((audioIn >> 8) & 0x01));
    HAL_GPIO_WritePin( outputPin9.port, outputPin9.pin, ((audioIn >> 9) & 0x01));
    HAL_GPIO_WritePin( outputPin10.port, outputPin10.pin, ((audioIn >> 10) & 0x01));
    HAL_GPIO_WritePin( outputPin11.port, outputPin11.pin, ((audioIn >> 11) & 0x01));
  
}

/*
void sendOutput(uint16_t audioIn){
    (audioIn & 0x01) ? (GPIOA->BSRR = 1 << GPIO_PIN_4) : (GPIOA->BSRR = 1 << GPIO_PIN_4);
    ((audioIn >> 1) & 0x01) ? (GPIOA->BSRR = 1 << GPIO_PIN_5) : (GPIOA->BRR = 1 << GPIO_PIN_5);
    ((audioIn >> 2) & 0x01) ? (GPIOA->BSRR = 1 << GPIO_PIN_6) : (GPIOA->BRR = 1 << GPIO_PIN_6);
    ((audioIn >> 3) & 0x01) ? (GPIOA->BSRR = 1 << GPIO_PIN_7) : (GPIOA->BRR = 1 << GPIO_PIN_7);
    ((audioIn >> 4) & 0x01) ? (GPIOB->BSRR = 1 << GPIO_PIN_0) : (GPIOB->BRR = 1 << GPIO_PIN_0);
    ((audioIn >> 5) & 0x01) ? (GPIOB->BSRR = 1 << GPIO_PIN_1) : (GPIOB->BRR = 1 << GPIO_PIN_1);
    ((audioIn >> 6) & 0x01) ? (GPIOB->BSRR = 1 << GPIO_PIN_10) : (GPIOB->BRR = 1 << GPIO_PIN_10);
    ((audioIn >> 7) & 0x01) ? (GPIOB->BSRR = 1 << GPIO_PIN_11) : (GPIOB->BRR = 1 << GPIO_PIN_11);
    ((audioIn >> 8) & 0x01) ? (GPIOB->BSRR = 1 << GPIO_PIN_12) : (GPIOB->BRR = 1 << GPIO_PIN_12);
    ((audioIn >> 9) & 0x01) ? (GPIOB->BSRR = 1 << GPIO_PIN_13) : (GPIOB->BRR = 1 << GPIO_PIN_13);
    ((audioIn >> 10) & 0x01) ? (GPIOB->BSRR = 1 << GPIO_PIN_14) : (GPIOB->BRR = 1 << GPIO_PIN_14);
    ((audioIn >> 11) & 0x01) ? (GPIOB->BSRR = 1 << GPIO_PIN_15) : (GPIOB->BRR = 1 << GPIO_PIN_15);
}
*/
//Wave Gen functions
uint16_t square_wave(uint8_t peakValue, double frequency, double * phase, double dutyCycle)
{
  uint16_t outputValue = 0;
  if(*phase < ((2*PI)*dutyCycle))
  {
    outputValue = peakValue;
  } else
  {
    outputValue = 1;
  }
  *phase = *phase + ((2*PI*frequency)/ sampleTime);
  
  if(*phase > (2*PI))
  {
    *phase = 0;
  }
  return outputValue;
  
}

uint16_t triangle_wave(uint8_t peakValue, double frequency, double * phase)
{
  uint16_t outputValue = 0;
  if (*phase < PI){
    outputValue = -peakValue + ((2 * peakValue / PI) * *phase);
  } else {
    outputValue = 3*peakValue - ((2 * peakValue / PI) * *phase);
  }

  *phase = *phase + ((2 * PI * frequency) / sampleTime);

  if (*phase > (2 * PI)) {
    *phase = 0;
  }
  
  return outputValue;
}

uint16_t saw_wave(uint8_t peakValue, double frequency, double * phase)
{
  uint16_t outputValue = 0;

  outputValue = -peakValue + ((2 * peakValue / PI) * *phase);


  *phase = *phase + ((2 * PI * frequency) / sampleTime);

  if (*phase > (2 * PI)) {
    *phase = 0;
  }
  
  return outputValue;
}

uint16_t noise_generator ()
{
  return (uint8_t)(rand() % 255);
  //return (uint8_t)(rand() % 17);
}

uint16_t drumGenerator(MIDIChannel * inputChannel, SynthState synthState)
{
  //uint8_t frequency, double * phase, double dutyCycle, int * envelopeState, unsigned int * count, 
  //unsigned int
  // * atackCount, unsigned int * decayCount, unsigned int * releaseCount
  uint16_t outputVolume = 0;
  switch (inputChannel->noteValue){
    case 0: // kick drum
      //outputVolume = triangle_wave(0xFF, 82.407, &inputChannel->phase);
      outputVolume = square_wave(0xFF, noteTable[7], &inputChannel->phase,0.2) * 3;//34.648
      outputVolume = outputVolume * drumEnvelope(inputChannel, 1, 20, 150);//4, 40, 200
      break;
    case 1: // snare drum
      outputVolume = noise_generator();
      outputVolume = outputVolume * drumEnvelope(inputChannel, 2 , 70, 130) * 3;
      break;
    case 2: // closed hi-hat
      outputVolume = noise_generator() * 0.3;
      outputVolume += square_wave(0xFF, noteTable[81], &inputChannel->phase, 0.8) * 0.7;//3520.0
      outputVolume = outputVolume * drumEnvelope(inputChannel, 1 , 10, 5);
      break;
    case 3: // opened hi-hat
      outputVolume = noise_generator();
      outputVolume += square_wave(0xFF, noteTable[78 + inputChannel->envelopeState], &inputChannel->phase, 0.5) * 0.5;
      outputVolume = outputVolume * drumEnvelope(inputChannel, 200 , 5, 10);
      break;
    case 4: //high tom
      outputVolume = square_wave(0xFF, noteTable[30 - (inputChannel->envelopeState * 2)], &inputChannel->phase, 0.5);//34.648
      outputVolume = outputVolume * drumEnvelope(inputChannel, 4, 40, 100);
      break;
    case 5: //low tom 
      outputVolume = square_wave(0xFF, noteTable[28 - (inputChannel->envelopeState * 2)], &inputChannel->phase, 0.5);//34.648
      outputVolume = outputVolume * drumEnvelope(inputChannel, 4, 40, 100);
      break;
    case 6: //effect 1
      outputVolume = noise_generator();
      outputVolume = outputVolume * drumEnvelope(inputChannel, 200 , 3, 200);
      break;
    case 7: //effect 2
      outputVolume = triangle_wave(0xFF, noteTable[47 + (inputChannel->envelopeState * 2)], &inputChannel->phase);
      outputVolume = outputVolume * drumEnvelope(inputChannel, 8, 40, 100);
      break;
  }
  return outputVolume;
}
 
//calculate midi channel output
uint16_t midiChannelOutputCompute(MIDIChannel * midiChannel, SynthState inputState)
{
  uint16_t outputVolume = 0;
  switch (inputState.mode){
    case 0://square wave standard
      //outputVolume = square_wave(0xFF, 880, &midiChannel->phase, 0.5);
      if(midiChannel->gate){
        //inputState.atackValue = 100;
        //inputState.decayValue = 100;
        //inputState.releaseValue = 300;
        
        outputVolume = square_wave(0xFF, noteTable[midiChannel->noteValue], &midiChannel->phase, 0.5);
        
        
      } else {
        outputVolume = 0;
      }

      //outputVolume = outputVolume * applyEnvelope(midiChannel, inputState);//parameters set at note assignment - velocity tied
      
      
      break;
    case 1://triangle wave standard
      if(midiChannel->gate){
      outputVolume = 10 * triangle_wave(0x0F, noteTable[midiChannel->noteValue], &midiChannel->phase);
      //outputVolume = applyEnvelope(midiChannel, inputState);//parameters set at note assignment - velocity tied
      } else {
        outputVolume = 0;
      }
      break;
    case 2://drums
      outputVolume = drumGenerator(midiChannel, inputState);
      break;
    case 3://noise gate
      if(midiChannel->gate){
        outputVolume = noise_generator();
      } else {
        outputVolume = 0;
      }
      break;
  }

  

  //midiChannel->channelVolume = outputVolume;
  return outputVolume;
}

double applyEnvelope(MIDIChannel * inputChannel, SynthState synthState)
{//for timer = 0, envelope length is 5.7823 mili seconds
  double outputVolume = 0;
  switch(inputChannel->envelopeState){//maybe this should start on idle mode
    case 1: //atack         
      if(inputChannel->envelopeTimers.atackCount > synthState.atackValue){
        inputChannel->envelopeTimers.atackCount = 0;
        inputChannel->envelopeTimers.count++;
      } else {
        inputChannel->envelopeTimers.atackCount++;
      }    
      if(inputChannel->envelopeTimers.count > 21){//when the true timer reaches the end, it changes the state machime and resets the true timer
        inputChannel->envelopeState = 2;
        inputChannel->envelopeTimers.count = 0;
        break;
      }
      //outputVolume = (exp((inputChannel->envelopeTimers.count * -0.022)+(0.2)));
      outputVolume = atackTable[inputChannel->envelopeTimers.count];
      if(inputChannel->gate == 0){//check for gate off. if true, go to release
        inputChannel->envelopeTimers.count = 0;
        inputChannel->envelopeState = 3;
      } 
      
      break;
    case 2: //decay
      if(inputChannel->envelopeTimers.decayCount > synthState.decayValue){
        inputChannel->envelopeTimers.decayCount = 0;
        inputChannel->envelopeTimers.count++;
      } else {
        inputChannel->envelopeTimers.decayCount++;
      }    
      if(inputChannel->envelopeTimers.count > 6){//when the true timer reaches the end, it changes the state machime and resets the true timer
        inputChannel->envelopeState = 3;
        inputChannel->envelopeTimers.count = 0;
        break;
      }
      //outputVolume = (uint16_t)(log10(inputChannel->envelopeTimers.count * (0.023)+(9.99)));
      outputVolume = decayTable[inputChannel->envelopeTimers.count];
      if(inputChannel->gate == 0){//check for gate off. if true, go to release
        inputChannel->envelopeState = 3;
        inputChannel->envelopeTimers.count = 0;
      } 

      break;
    case 3: //sustain
      outputVolume = 1;
      if(inputChannel->gate == 0){
        inputChannel->envelopeState = 4;
      }
      
      
      break;
    case 4: //release
    if(inputChannel->envelopeTimers.releaseCount > synthState.releaseValue){
        inputChannel->envelopeTimers.releaseCount = 0;
        inputChannel->envelopeTimers.count++;
      } else {
        inputChannel->envelopeTimers.releaseCount++;
      }    
      if(inputChannel->envelopeTimers.count > 17){//when the true timer reaches the end, it changes the state machime and resets the true timer
        inputChannel->envelopeState = 0;
        inputChannel->envelopeTimers.count = 0;
      }
      //outputVolume = (uint16_t)(log10(inputChannel->envelopeTimers.count * (0.035294)+(1)));
      outputVolume = releaseTable[inputChannel->envelopeTimers.count];
      break;
    case 0: //idle state
      outputVolume = 0;
      //reset the counters
      inputChannel->envelopeTimers.atackCount = 0;
      inputChannel->envelopeTimers.decayCount = 0;
      inputChannel->envelopeTimers.releaseCount = 0;
      //check for new key press to restart cycle
      if(inputChannel->gate == 1){
        inputChannel->envelopeState = 1;//atack
      }
  }
  return outputVolume;
  
}

double drumEnvelope(MIDIChannel * inputChannel, int atackValue, int decayValue, int releaseValue)
{//for timer = 0, envelope length is 5.7823 mili seconds
  double outputVolume = 0;
  switch(inputChannel->envelopeState){//maybe this should start on idle mode
    case 1: //atack         
      if(inputChannel->envelopeTimers.atackCount > atackValue){
        inputChannel->envelopeTimers.atackCount = 0;
        inputChannel->envelopeTimers.count++;
      } else {
        inputChannel->envelopeTimers.atackCount++;
      }    
      if(inputChannel->envelopeTimers.count > 23){//when the true timer reaches the end, it changes the state machime and resets the true timer
        inputChannel->envelopeState = 2;
        inputChannel->envelopeTimers.count = 0;
        break;
      }
      //outputVolume = (exp((inputChannel->envelopeTimers.count * -0.022)+(0.2)));
      outputVolume = atackTable[inputChannel->envelopeTimers.count];
      
      break;
    case 2: //decay
      if(inputChannel->envelopeTimers.decayCount > decayValue){
        inputChannel->envelopeTimers.decayCount = 0;
        inputChannel->envelopeTimers.count++;
      } else {
        inputChannel->envelopeTimers.decayCount++;
      }    
      if(inputChannel->envelopeTimers.count > 7){//when the true timer reaches the end, it changes the state machime and resets the true timer
        inputChannel->envelopeState = 3;
        inputChannel->envelopeTimers.count = 0;
        break;
      }
      //outputVolume = (uint16_t)(log10(inputChannel->envelopeTimers.count * (0.023)+(9.99)));
      outputVolume = decayTable[inputChannel->envelopeTimers.count];


      break;
    case 3: //release
    if(inputChannel->envelopeTimers.releaseCount > releaseValue){
        inputChannel->envelopeTimers.releaseCount = 0;
        inputChannel->envelopeTimers.count++;
      } else {
        inputChannel->envelopeTimers.releaseCount++;
      }    
      if(inputChannel->envelopeTimers.count > 18){//when the true timer reaches the end, it changes the state machime and resets the true timer
        inputChannel->envelopeState = 4;
        inputChannel->envelopeTimers.count = 0;
      }
      //outputVolume = (uint16_t)(log10(inputChannel->envelopeTimers.count * (0.035294)+(1)));
      outputVolume = releaseTable[inputChannel->envelopeTimers.count];
      break;
    case 4: //wait
      if(inputChannel->gate == 0){
        inputChannel->envelopeState = 0;
      }
      break;
    case 0: //idle state
      outputVolume = 0;
      //reset the counters
      inputChannel->envelopeTimers.atackCount = 0;
      inputChannel->envelopeTimers.decayCount = 0;
      inputChannel->envelopeTimers.releaseCount = 0;
      //check for new key press to restart cycle
      if(inputChannel->gate == 1){
        inputChannel->envelopeState = 1;//atack
      }
  }
  return outputVolume;
  
}
//input handling
void synthAtt(SynthState * inputState)//all the switch and button inputs are set to pull up
{
  //buttons
  if(HAL_GPIO_ReadPin(modePin.port, modePin.pin) == GPIO_PIN_RESET){//updates current pin state
    inputState->modeTrackNow = true;
  } else {
    inputState->modeTrackNow = false;
  }

  if(inputState->modeTrackNow == true && inputState->modeTrackOld == false){//compares with last pin state to catch the positive edge
    inputState->mode++;
    if(inputState->mode > 3){//only 4 modes available at the moment
      inputState->mode = 0;
    }
    switch(inputState->mode){
      case 0:
        
        break;
      case 1:
        break;
    }
  }
  
  //loop implementation later

  //switches
  if(HAL_GPIO_ReadPin(vibratoPin.port, vibratoPin.pin) == GPIO_PIN_RESET){
    inputState->recOn = true;
  } else {
    inputState->recOn = false;
  }

  if(HAL_GPIO_ReadPin(burstPin.port, burstPin.pin) == GPIO_PIN_RESET){
    inputState->holdOn = true;
  } else {
    inputState->holdOn = false;
  } 

  if(HAL_GPIO_ReadPin(arpeggioPin.port, arpeggioPin.pin) == GPIO_PIN_RESET){
    inputState->arpeggioOn = true;
  } else {
    inputState->arpeggioOn = false;
  } 

  //pots
  //inputState->arpSpeed =  (analogReadADC1() - 0) * (1 - 0) / (4096 - 0) + 1;
  inputState->arpSpeed = analogReadADC1();
  


  inputState->modeTrackOld = inputState->modeTrackNow;//makes old and new pin states the same
}

//sequencer stuff
void sequencerStart(void){
  //initialize the matrix
  //initialize the timer
}
void sequencerCompute(void){
  //advance the sequencer position
  //reset the timer
  //execute the commands in the new position
}
void sequencerInsertEvent(void){
  //take midi conmand and place it in the current position of the sequencer in one of the channels
}

//misc
uint16_t analogReadADC1()
{
  ADC1->CR2 |= ADC_CR2_SWSTART;
  while (!(ADC1->SR & ADC_SR_EOC));//1 for conversion completed
  return ADC1->DR;
}

/*
80,note,velocity - Note Off
90,note,velocity - Note On, 60=mid.C
A0,note,velocity - Note Aftertouch, 60=mid.C
B0,control,setting - Control Change
C0,program - Program Change
D0,note,velocity - Channel After-touch
E0,bend.lo,bend.hi - Pitch Wheel Change
*/