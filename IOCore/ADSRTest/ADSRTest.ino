#include <Wire.h>
#include "wiring_private.h"
#include "SAMD51_InterruptTimer.h"
#include "samd51_adc.h"
#include "adsr_class.h"

#define MAX(a,b) ((a) > (b) ? a : b)
#define MIN(a,b) ((a) < (b) ? a : b)
#define RANGE(min,v,max)	  MIN(MAX(v,min),max) 
#define MAP_RANGE(v,a,b,mi,ma) RANGE(mi,map(v,a,b,mi,ma),ma)


// A Jack+Poti
// D Jack+Poti
// S Jack+Poti
// R Jack+Poti
// Coeff Poti klein
// 1. Gate Jack+Button
// 2. Gate Jack+Button
// Loop Button
// Mode: ADSR / 2 * AD 
// OUT 1 Jack (DAC)
// OUT 2 Jack (DAC)

// Frontpannel
// 8 Jacks
// 5 Potis
// 3 Button


SAMD51_ADC adc51;
ZM_ADSR myADSR;

// 
// Timing Parameter
// 
// wir machen alles in double statt float, damit ist zwar der FPU Vorteil durch m4 hin,
// aber eine höhere Auflösung bei etwas geringer SR gibt bessere Ergebnisse
// Ein mix aus Float/Double hatte Noise Seiteneffekte die nicht weiter verfolgt wurden
//

const double    samplerate = 41666.0;  // Steuert die HK Zeit auf 1Sek bei SampleRate * 10 => 10Sekunden
const double    scale_4 = 1.0/4096.0;  // Das Brauchen wir zum scale von ADC Integers -> 0.00 ... 1.00
const uint32_t  tc_usec_timer = 32;
//
//  OUTPUT AUDIO sample Data
//
uint16_t DACValue0 = 0;
uint16_t DACValue1 = 0;
bool     noise_led=false;

//
//  OUTPUT AUDIO sample
//
void outputSample() {
  DAC->DATA[0].reg = DACValue0;
  DAC->DATA[1].reg = DACValue1;
  if(noise_led)  PORT->Group[PORTB].OUTCLR.reg = 1ul << 0;  else   PORT->Group[PORTB].OUTSET.reg = 1ul << 0;    // LED
  if(noise_led)  PORT->Group[PORTA].OUTCLR.reg = 1ul << 22; else   PORT->Group[PORTA].OUTSET.reg = 1ul << 22;   // SQUARE OUT NOISE
}


void renderAudio(){

  noise_led=!noise_led;

  bool mygate = true;
  if (  (PORT->Group[PORTB].IN.reg & (1ul << 16))  &&      // Trigger from Button
          (PORT->Group[PORTB].IN.reg & (1ul << 23))     ) {  // Trigger from jack
    mygate=false;
  }

  static bool zm_gate_before=false;
  if(mygate != zm_gate_before)
    myADSR.setNewGateState(mygate); // Note On/Off
  zm_gate_before = mygate;

  
  //  bool loop_retrigger=false;
  if(!(PORT->Group[PORTA].IN.reg & (1ul << 20))) { // PA20 button A/B Bank
      if(myADSR.getState()==ZM_ADSR::env_idle){
        myADSR.setNewGateState(true);
      }
      if(myADSR.getState()==ZM_ADSR::env_sustain){
        myADSR.setNewGateState(false);
      }
  }

  uint16_t rt_env_output = myADSR.process() * 2047.0 + 2047.0; // scale and convert to unipolar
  DACValue1 = rt_env_output;
  DACValue0 = rt_env_output;
}

void loop() { // never reached
    while(true);
}

void loop2() {

  //PORT->Group[PORTA].OUTCLR.reg = 1ul << 22; 

  outputSample(); // wird immer zuerst ausgegeben um jitter durch laufzeitunterschiede zu minimieren daher ein sample delay
  renderAudio();  // die eigenliche Sampleerzeugung

  // Ab hier wir lesen abwechselnd ADC´s und berechnen neue Timing Werte
  static uint16_t noise_pitch_jack     = 2048;
  static uint16_t noise_pitch_poti     = 2048;
  static uint16_t sample_pitch_poti    = 2048;
  static uint16_t sample_pitch_jack    = 2048; 
  static uint16_t sqr_pitch_poti       = 2048; 
  static uint16_t sqr_pitch_jack       = 2048; 
  static uint16_t spread_adc           = 2048; 
  static uint16_t cv_sample_select_adc = 2048; 
  static uint16_t prg8_smpl_select_adc = 2048; 
  static uint16_t ratchet_adc          = 2048; 


  static uint16_t adc_state_machine = 0;
  adc_state_machine++;
  adc_state_machine&=0x000f;

  switch(adc_state_machine) {
    case 0: 
      adc51.startReadAnalog(PB01,ADC_Channel13,false);      // Buchse #2 (erste Digitale) Signal: Digital_Noise_Pitch normalized 12v
      break;
    case 1: 
      noise_pitch_jack = adc51.readLastValue();
      adc51.startReadAnalog(PB02,ADC_Channel14,false);      // Poti #2  Signal: Manual_Digital_Noise_Pitch
      break;
    case 2:  
      noise_pitch_poti = adc51.readLastValue();
      adc51.startReadAnalog(PA07,ADC_Channel7,false); 
      break;
    case 3:  
      sample_pitch_poti = adc51.readLastValue();
      myADSR.setAttackRate((double)sample_pitch_poti * (double)sample_pitch_poti * scale_4 * scale_4 * samplerate);
      adc51.startReadAnalog(PA06,ADC_Channel8,true);  
      break;
    case 4:  
      sample_pitch_jack = adc51.readLastValue();
      adc51.startReadAnalog(PB03,ADC_Channel15,false);  // Manual_6XSqr_Pitch Poti
      break;
    case 5:  
      sqr_pitch_poti = adc51.readLastValue();
      myADSR.setSustainLevel((double)sqr_pitch_poti * scale_4 );
      adc51.startReadAnalog(PB04,ADC_Channel6,true);    // CV_6XSqr_Pitch Poti
      break;
    case 6:  
      sqr_pitch_jack = adc51.readLastValue();
      adc51.startReadAnalog(PB05,ADC_Channel7,true);
      break;
    case 7:  
      spread_adc = adc51.readLastValue();
      myADSR.setReleaseRate((double)spread_adc * (double)spread_adc * scale_4 * scale_4 *  samplerate);
      adc51.startReadAnalog(PB09,ADC_Channel3,false);
      break;
    case 8:  
      cv_sample_select_adc = adc51.readLastValue();
      adc51.startReadAnalog(PA06,ADC_Channel6,false);
      break;
    case 9:  
      prg8_smpl_select_adc = adc51.readLastValue();
      myADSR.setTargetRatioAll( (double)prg8_smpl_select_adc * scale_4 );
      adc51.startReadAnalog(PB07,ADC_Channel9,true);
      break;
    case 10:
      ratchet_adc = adc51.readLastValue();
      myADSR.setDecayRate( (double)ratchet_adc * (double)ratchet_adc * scale_4 * scale_4 * samplerate );
      break;
    case 11:
      break;
    case 12:
      break;
    case 13:
      break;
    case 14:
      break;
    case 15:
      break;
    }

   // DAC->DATA[1].reg=1000;
}

void setup() {
  //put your setup code here, to run once:
  // First try remove DC from Ouputs  
  pinMode(PA23,OUTPUT); // LED
  pinMode(PB00,OUTPUT); // LED
  pinMode(PB31,OUTPUT); // LED

  pinMode(PA13,INPUT); digitalWrite(PA13,false);
  pinMode(PA14,INPUT); digitalWrite(PA14,false);
  pinMode(PA15,INPUT); digitalWrite(PA15,false);
  pinMode(PA16,INPUT); digitalWrite(PA16,false);
  pinMode(PA17,INPUT); digitalWrite(PA17,false);
  pinMode(PA18,INPUT); digitalWrite(PA18,false);

  pinMode(PA19,OUTPUT); // SQR1
  pinMode(PA12,OUTPUT); // SQR2
  pinMode(PB15,OUTPUT); // SQR1
  pinMode(PB14,OUTPUT); // SQR1
  pinMode(PB13,OUTPUT); // SQR1
  pinMode(PB12,OUTPUT); // SQR1

  pinMode(PA22,OUTPUT); // PWM OUT

  pinMode(PB01,INPUT_PULLUP); // SQR1
  pinMode(PA21,INPUT_PULLUP); // SQR1
  pinMode(PB17,INPUT_PULLUP); // 8Bit or sample Button
  pinMode(PB16,INPUT_PULLUP); // Manual Trigger
  pinMode(PB23,INPUT_PULLUP); // Manual Trigger

  dacInit();
  DAC->DATA[0].reg = 2048;   // ca 1.5V
  DAC->DATA[1].reg = 2048;
  while (DAC->SYNCBUSY.bit.DATA0);

  // Timing SampleRate
  adc51.createADCMap(50000*2); // 50Khz but pitch down by * 2
  
  /*
  while(1){
    while(SysTick->VAL & 0xff); // Keine gute Idee da wir ja nicht jeden systick finden, evtl unten noch Bitmask setzen ?!?!?!
    loop2();
  } */
  TC.startTimer(tc_usec_timer, loop2); // 32 usec , wenn hier zu schnell jitter + ADC Lesefehler

  //for(;;) loop2();  // Keine Gute idee, da jitter durch Laufzeitunterschiede!

}