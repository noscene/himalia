#include <Wire.h>
#include "wiring_private.h"
#include "SAMD51_InterruptTimer.h"
#include "samd51_adc.h"
#include "adsr_class.h"
#include "lfo_wave.h"

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
// Loop Button via A/B Button
// Freece ADC Values with 8Bit Button
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
// wir machen alles in double statt float, damit ist zwar der FPU Vorteil durch m4 weg,
// aber eine höhere Auflösung bei etwas geringer SR gibt bessere Ergebnisse
// Optinal nur calcCoef auf float Basis für schnellere SampleRates
// Ein mix aus Float/Double hatte Noise Seiteneffekte die nicht weiter verfolgt wurden: vermutlich schräges verhalten durch Laufzeitunterschiede bei zu kurzen tc_usec_timer
// Stichwort: Float denormalized Performance
//

// TODO: noch ADC Bereiche mit MAP_RANGE Calibrieren
// TODO: noch CV Spannung aufaddieren und LUT auf 8192 setzen
// min 80uS ADSR Cooef kann mit Double Arbeiten 12.5Khz
// min 60uS ADSR Cooef mit float ca 16.6KHz
// 
const double    scale_4           = 1.0/4096.0;   // Das Brauchen wir zum scale von ADC Integers -> 0.00 ... 1.00
const uint32_t  tc_usec_timer     = 60;     // 38 == 26.kKhz | 40 == 25KH | 80 == 12.5Khz Achtung darunter Probleme Attack/Sustain Reading from ADC´s !!!!
const double    samplerate        = 1.0 / (double)tc_usec_timer * 1000000.0;  // Steuert die HK Zeit auf 1Sek bei SampleRate * 10 => 10Sekunden
const double    max_attack_time   = samplerate  * 1;  // Time Range for ADSR
const double    max_release_time  = samplerate * 10;  // Time Range for ADSR
double lut[4096]; // lookupTable adc -> double Value 1/4096 ^ 3

// Use this to figure out best tc_usec_timer value!!!!
// #define DEBUG_TIMING_BY_SQR_OUT 1


//
// LFO
//
float thea_sample       = 0.0f;
float inc_sample        = 0.01f;
uint16_t sample_offset  = 0;


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

  #ifndef DEBUG_TIMING_BY_SQR_OUT  
    if(noise_led)  PORT->Group[PORTA].OUTSET.reg = 1ul << 22; else PORT->Group[PORTA].OUTCLR.reg = 1ul << 22;
  #endif

}


void renderAudio() {

  // Gate Status checken
  bool mygate = true;
  if (  (PORT->Group[PORTB].IN.reg & (1ul << 16))  &&      // Trigger from Button
        (PORT->Group[PORTB].IN.reg & (1ul << 23))     ) {  // Trigger from jack
    mygate=false; // no gate from Button or Jack
  }

  // if change gate...handle in adsr
  static bool zm_gate_before=false;
  if(mygate != zm_gate_before)
    myADSR.setNewGateState(mygate); // Note On/Off
  zm_gate_before = mygate;
  
  //  AutoLoop ?
  if(!(PORT->Group[PORTA].IN.reg & (1ul << 20))) { // PA20 button A/B Bank
      if(myADSR.getState()==ZM_ADSR::env_idle){
        myADSR.setNewGateState(true);
        thea_sample=0.0f;
      }
      if(myADSR.getState()==ZM_ADSR::env_sustain){
        myADSR.setNewGateState(false);
      }
  }

  // LED On when in AttackPhase
  if(myADSR.getState()==ZM_ADSR::env_attack){
    noise_led=false;  // LED ON
  }else{
    noise_led=true;
  }

  // Scale to Integer and mark for Output
  uint16_t rt_env_output = myADSR.process() * 2047.0 + 2047.0; // scale and convert to unipolar
  // DACValue1 = rt_env_output;
  DACValue1 = rt_env_output;


  // LFO

  // Forward Button + Trigger to LED
  static uint32_t leftSamples = 1000000;
  static bool     previous_trigger = false;
  if (  (PORT->Group[PORTB].IN.reg & (1ul << 16))  &&      // Trigger from Button
        (PORT->Group[PORTB].IN.reg & (1ul << 23))     ) {  // Trigger from jack
    PORT->Group[PORTB].OUTCLR.reg = 1ul << 31;             // trigger fire!
    previous_trigger=false;
  } else {
    PORT->Group[PORTB].OUTSET.reg = 1ul << 31;
    if(!previous_trigger){          // check for edge
      leftSamples = 1000000; // ratchet_counts;
      thea_sample=0.0f; // retrigger 
    }
    previous_trigger=true;
  }


  if(leftSamples>0){
    thea_sample+=inc_sample;
    if(thea_sample>1.0f){
      thea_sample=0.0f;
      leftSamples--;
    }

    float sample_raw_len =  4095.0;

    DACValue0 = lfo_wave[sample_offset + (uint32_t)(sample_raw_len * thea_sample)];  // extend to 32 Bit
  }


}

void loop() { // never reached
    while(true);
}



void loop2() {

  #ifdef DEBUG_TIMING_BY_SQR_OUT
  PORT->Group[PORTA].OUTSET.reg = 1ul << 22;
  #endif


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

  // Freese ADC READING!!!!
  bool is_8bitchipmode = PORT->Group[PORTB].IN.reg & (1ul << 17);  
  if(!is_8bitchipmode){
      adc_state_machine=14;
  }

  float pitch_lfo;

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
      sample_offset = noise_pitch_poti*6;
      adc51.startReadAnalog(PA07,ADC_Channel7,false); 
      break;
    case 3:  
      sample_pitch_poti = adc51.readLastValue();
      myADSR.setAttackRate(lut[sample_pitch_poti] *  max_attack_time );
      pitch_lfo = 4096 - sample_pitch_poti;
      inc_sample = 1.0 / (float) pitch_lfo * 1.0 / (float) pitch_lfo * 250.0;
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
      myADSR.setReleaseRate( lut[spread_adc] *  max_release_time);
      adc51.startReadAnalog(PB09,ADC_Channel3,false);
      break;
    case 8:  
      cv_sample_select_adc = adc51.readLastValue();
      adc51.startReadAnalog(PA06,ADC_Channel6,false);
      break;
    case 9:  
      prg8_smpl_select_adc = adc51.readLastValue();
      myADSR.setTargetRatioAll( lut[prg8_smpl_select_adc] );
      adc51.startReadAnalog(PB07,ADC_Channel9,true);
      break;
    case 10:
      ratchet_adc = adc51.readLastValue();
      myADSR.setDecayRate(lut[ratchet_adc] * max_release_time );
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

   #ifdef DEBUG_TIMING_BY_SQR_OUT
   PORT->Group[PORTA].OUTCLR.reg = 1ul << 22; 
   #endif
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

  for(int i = 0 ; i < 4096; i++) {
    double scale = (double) i *  1.0/4096.0;
    lut[i] = scale * scale * scale ;
  }


  // Timing SampleRate
  adc51.createADCMap(50000*2); // 50Khz but pitch down by * 2
  
  /*
  while(1){
    while(SysTick->VAL & 0xff); // Keine gute Idee da wir ja nicht jeden systick finden, evtl unten noch Bitmask setzen ?!?!?!
    loop2();
  } */

  // Achtung! diese Variante erzeugt Doubletten, also Runtime checken
  TC.startTimer(tc_usec_timer, loop2); // 32 usec , wenn hier zu schnell jitter + ADC Lesefehler

  //for(;;) loop2();  // Keine Gute idee, da jitter durch Laufzeitunterschiede!

}