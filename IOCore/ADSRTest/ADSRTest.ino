#include <Wire.h>
#include "wiring_private.h"
#include "SAMD51_InterruptTimer.h"
#include "samd51_adc.h"

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
const float samplerate = 48000000.0 / 4.0 / 125.0;
const float reciprocal_sr = 1.0 / samplerate;

//float pitches[1024];
//uint8_t sines[256];


//
//  OUTPUT AUDIO sample Data
//

uint16_t DACValue0 = 0;
uint16_t DACValue1 = 0;
bool     noise_led=false;

//
//  OUTPUT AUDIO sample
//
void outputSample(){

  if(noise_led)  PORT->Group[PORTB].OUTCLR.reg = 1ul << 0;  else   PORT->Group[PORTB].OUTSET.reg = 1ul << 0;    // LED
  if(noise_led)  PORT->Group[PORTA].OUTCLR.reg = 1ul << 22; else   PORT->Group[PORTA].OUTSET.reg = 1ul << 22;   // SQUARE OUT NOISE


  DAC->DATA[0].reg = DACValue0;
  DAC->DATA[1].reg = DACValue1;
}

float calcCoef(float rate, float targetRatio) {
    return exp(-log((1.0 + targetRatio) / targetRatio) / rate);
}





typedef enum  {
    env_idle = 0,
    env_attack,
    env_decay,
    env_sustain,
    env_release
} envState;
envState        rt_state;
float           rt_env_output;
bool            is_trigger;

float zm__trigger_level = 0.0001;

float zm__attack=0.005;
float zm__decay=0.02;
float zm__release=0.05;
float zm__sustain=0.5;
float zm__coef1=1.3;
float zm__coef2=0.0001;
float zm__time_base = 0.2;
bool  zm__retrigger = true;

//
//  AUDIO RENDERER callback for a single sample
//


        float attackRate;
        float decayRate;
        float releaseRate;
        float attackCoef;
        float decayCoef;
        float releaseCoef;
        float sustainLevel;
        float attackBase;
        float decayBase;
        float releaseBase;

void renderAudio() {

  noise_led=!noise_led;
  //DACValue1++;
  //DACValue0++;

  //
  // ADSR Stuff
  //

        
        /*
        if (zm__coef1 < 0.0000001)
            zm__coef1 = 0.0000001;  // -180 dB
        targetRatioA = zm__coef1;
        
        if (zm__coef2 < 0.000000001)
            zm__coef2 = 0.000000001;  // -180 dB
        targetRatioDR = zm__coef2;
        */
        
        static uint16_t coeff_state_machine = 0;
        coeff_state_machine++;
        coeff_state_machine&=0x0003;

        switch(coeff_state_machine) {
          case 0:
            attackRate = zm__attack * samplerate*zm__time_base;
            attackCoef = calcCoef(attackRate, zm__coef1);
            attackBase = (1.0 + zm__coef1) * (1.0 - attackCoef);
            break;
          case 1:
            decayRate = zm__decay * samplerate*zm__time_base;
            decayCoef = calcCoef(decayRate, zm__coef2);
            decayBase = (sustainLevel - zm__coef2) * (1.0 - decayCoef);
            break;
          case 2:
            attackRate = zm__attack * samplerate*zm__time_base;
            attackCoef = calcCoef(attackRate, zm__coef1);
            attackBase = (1.0 + zm__coef1) * (1.0 - attackCoef);
            break;
          case 3:
            releaseRate = zm__release * samplerate*zm__time_base;
            releaseCoef = calcCoef(releaseRate, zm__coef2);
            releaseBase = -zm__coef2 * (1.0 - releaseCoef);
            break;
        }


        sustainLevel = zm__sustain;


        bool zm__gate = true;
        if (  (PORT->Group[PORTB].IN.reg & (1ul << 16))  &&      // Trigger from Button
                (PORT->Group[PORTB].IN.reg & (1ul << 23))     ) {  // Trigger from jack
          zm__gate=false;
        }

        // https://github.com/fdeste/ADSR/blob/master/ADSR.c#L116

            switch (rt_state) {
                case env_idle:
                    // printf("i");
                    if(zm__retrigger || zm__gate){
                        rt_state = env_attack;
                    }
                    break;
                case env_attack:
                    // printf("A");
                    rt_env_output = attackBase + rt_env_output * attackCoef;
                    if (rt_env_output >= 1.0) {
                        rt_env_output = 1.0;
                        rt_state = env_decay;
                    }
                    break;
                case env_decay:
                    // printf("D");
                    rt_env_output = decayBase + rt_env_output * decayCoef;
                    if (rt_env_output <= sustainLevel) {
                        rt_env_output = sustainLevel;
                        rt_state = env_sustain;
                    }
                    if(zm__gate) {
                       rt_state = env_attack;
                    }
                    break;
                case env_sustain:
                    // printf("S");
                    if(zm__retrigger || !zm__gate){
                        rt_state = env_release;
                    }else{
                        rt_env_output = zm__sustain;
                    }
                    break;
                case env_release:
                    // printf("R");
                    rt_env_output = releaseBase + rt_env_output * releaseCoef;
                    if (rt_env_output <= 0.0) {
                        rt_env_output = 0.0;
                        rt_state = env_idle;

                    }
                    if(zm__gate) {
                       rt_state = env_attack;
                    }
            }

            DACValue1 = rt_env_output * 2047 + 2047;
            DACValue0 = rt_env_output * 2047 + 2047;

}



void loop() {
}

void loop2() {

  //PORT->Group[PORTA].OUTCLR.reg = 1ul << 22; 
  outputSample();
  renderAudio();

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
      adc51.startReadAnalog(PA06,ADC_Channel8,true);  
      break;
    case 4:  
      sample_pitch_jack = adc51.readLastValue();
      adc51.startReadAnalog(PB03,ADC_Channel15,false);  // Manual_6XSqr_Pitch Poti
      break;
    case 5:  
      sqr_pitch_poti = adc51.readLastValue();
      adc51.startReadAnalog(PB04,ADC_Channel6,true);    // CV_6XSqr_Pitch Poti
      break;
    case 6:  
      sqr_pitch_jack = adc51.readLastValue();
      adc51.startReadAnalog(PB05,ADC_Channel7,true);
      break;
    case 7:  
      spread_adc = adc51.readLastValue();
      adc51.startReadAnalog(PB09,ADC_Channel3,false);
      break;
    case 8:  
      cv_sample_select_adc = adc51.readLastValue();
      adc51.startReadAnalog(PA06,ADC_Channel6,false);
      break;
    case 9:  
      prg8_smpl_select_adc = adc51.readLastValue();
      adc51.startReadAnalog(PB07,ADC_Channel9,true);
      break;
    case 10:
      ratchet_adc = adc51.readLastValue();
      break;
    }
  
  
  
  
  uint32_t noise_pitch_sum = ((noise_pitch_poti + noise_pitch_jack) );
  if(noise_pitch_jack>4000){  // if is connected
     noise_pitch_sum = noise_pitch_poti;
  }

  
  if(noise_pitch_sum> 4095) noise_pitch_sum= 4095;
  //inc_noise = adc51.adcToInc[noise_pitch_sum] * 4.0f;
  //if(inc_noise> 0.99)inc_noise=0.99;

  // -----------------------------------------------------------------
  // SAMPLE Speed
  uint16_t sample_pitch_sum =  sample_pitch_poti;
  if(sample_pitch_jack<4095){  // if is connected
     sample_pitch_sum += sample_pitch_jack;
  }
  if(sample_pitch_sum > 4095){
    sample_pitch_sum = 4095;
  }

  const float scale_4 = 1.0/4096.0;
  zm__attack = (float)sample_pitch_sum * scale_4 ;
  zm__attack = zm__attack * zm__attack * zm__attack;

  // float clk_sample_f = pitches[(sample_pitch_sum >> 2) & 0x03ff];  // Limit 1024 array size
  //inc_sample = clk_sample_f ;
  //if(inc_sample>1.0f) inc_sample=1.0f;
  //if(inc_sample<0.000001f) inc_sample=0.000001f;  

  // 8Bit ChipMusic Speed
  //inc_8bit =  inc_sample;

  // -----------------------------------------------------------------
  // SQR Speed
  uint16_t sqr_pitch_sum = ((sqr_pitch_poti + sqr_pitch_jack)  );
  if(sqr_pitch_jack > 4000)
    sqr_pitch_sum = sqr_pitch_poti;

  if(sqr_pitch_sum>4000) sqr_pitch_sum=4000;

  zm__sustain = (float)sqr_pitch_sum * scale_4;

  zm__decay = (float)ratchet_adc * scale_4 ;
  zm__decay = zm__decay * zm__decay * zm__decay;

  zm__release = (float)spread_adc * scale_4 ;
  zm__release = zm__release * zm__release * zm__release;

  zm__coef1 = (float)prg8_smpl_select_adc * scale_4 ;
  zm__coef1 = zm__coef1 * zm__coef1 * zm__coef1;
  zm__coef2 = zm__coef1;

  if(!(PORT->Group[PORTA].IN.reg & (1ul << 20))) // PA20 button A/B Bank
    zm__retrigger=true;
  else{
    zm__retrigger=false;
  }


  // float inc_sqr = adc51.adcToInc[sqr_pitch_sum];
  
/*
  // Bank Switch
  uint16_t spread_bank_offset=0;
  if(!(PORT->Group[PORTA].IN.reg & (1ul << 21))) // PA21 button A/B Bank
    spread_bank_offset=16;

  // prg spread
  uint16_t spread   = MAP_RANGE(spread_adc,250,3650, 0, 15) + spread_bank_offset;
  */

  
/*
  if(cv_sample_select_adc < 4000 ) // if jack is conneced (no normalized)
    prg8_smpl_select_adc+=cv_sample_select_adc;
  int16_t prg8_smpl_select    = MAP_RANGE(prg8_smpl_select_adc,210,3650, 0, 15);
  uint16_t smpl_bank_offset=0;
*/


  


  // uint16_t ratchet_adc_select   = MAP_RANGE(ratchet_adc,250,3650, 0, 15);



  // PORT->Group[PORTA].OUTSET.reg = 1ul << 22;   // SQUARE OUT NOISE


}





void setup() {
  //put your setup code here, to run once:

  // First try remove DC from Ouputs  
  pinMode(PA23,OUTPUT); // LED
  pinMode(PB00,OUTPUT); // LED
  pinMode(PB31,OUTPUT); // LED

/*
  // gen sin Table
  for(uint16_t i = 0 ; i < 256 ; i ++){
    float phase = (float)i / 256.0f * PI * 2.0f;
    sines[i] = (sin(phase) + 1.0) * 127.0f; 
  }

  // gen Table
  for(uint16_t i = 0 ; i < 1024 ; i ++){
    float clk_tempo_f = (float)i / 256.0f - 6.0f;
    pitches[i] = pow(12,clk_tempo_f);
  }
  */

  //Serial.begin(115200);
  // while ( !Serial ) delay(10);   // wait for native usb
  //Serial.println("Himalia");

  /*
  flash.begin();
  Serial.println("Adafruit Serial Flash Info example");
  Serial.print("JEDEC ID: "); Serial.println(flash.getJEDECID(), HEX);
  Serial.print("Flash size: "); Serial.println(flash.size());
  */

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
  
  TC.startTimer(16, loop2); // 20 usec = 50Khz
  //for(;;) loop2();

}