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



// Array mit Attack Times, der letzte pro Zeile muss immer Null Sein!!! also max 15 Schläge
float array_claps [16][16] = {
  {0.1, 0.1, 0.0, 0.0, 0.0, 0.0, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.0, },
  {0.2, 0.1, 0.1, 0.0, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.0, },
  {0.1, 0.1, 0.1, 0.1, 0.1, 0.0, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.0, },
  {0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.0, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.0, },
  {0.1, 0.1, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.0, },
  {0.2, 0.1, 0.1, 0.0, 0.1, 0.1, 0.1, 0.1, 0.0, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.0, },
  {0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.0, 0.1, 0.1, 0.1, 0.1, 0.1, 0.0, },
  {0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.0, 0.1, 0.1, 0.1, 0.1, 0.0, },
  {0.1, 0.1, 0.0, 0.0, 0.0, 0.0, 0.1, 0.1, 0.1, 0.1, 0.1, 0.0, 0.1, 0.1, 0.1, 0.0, },
  {0.2, 0.1, 0.1, 0.0, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.0, 0.1, 0.1, 0.0, },
  {0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.0, 0.1, 0.0, },
  {0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.0, 0.0, },
  {0.1, 0.1, 0.0, 0.0, 0.0, 0.0, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.0, },
  {0.2, 0.1, 0.1, 0.0, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.0, },
  {0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.0, },
  {0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.0, }
};


uint16_t wave_tables_lfo[4096 * 8];




// need to Test .... TODO: call this from setup function and uncomment return in Line 101
void fixBOD33(){
  // A magic number in the unused area of the user page indicates that
  // the device is already updated with the current configuration.
  static const uint32_t magic = 0xa5f12945;
  const uint32_t *page        = (uint32_t *)NVMCTRL_USER;
  if (page[8] == magic)
    return;

  union {
    uint32_t data[128];
    uint8_t bytes[512];
  };

  // void V2Memory::Flash::UserPage::read(uint32_t data[128]) {
  memcpy(data, (const void *)NVMCTRL_USER, 512);

  Serial.println("current FUSES");
  Serial.println( data[0],HEX);
  Serial.println( data[1],HEX);
  Serial.println( data[2],HEX);
  Serial.println( data[3],HEX);
  Serial.println( data[4],HEX);
  Serial.println( data[5],HEX);
  Serial.println( data[6],HEX);
  Serial.println( data[7],HEX);
  Serial.println( data[8],HEX);



  // Ignore all current values; fix the fallout caused by the broken uf2
  // bootloader, which has erased the devices's factory calibration. Try to
  // restore it with known values
  //
  // User Page Dump (Intel Hex) of pristine SAMD51G19A:
  // :0200000400807A
  // :1040000039929AFE80FFECAEFFFFFFFFFFFFFFFF3C
  // :1040100010408000FFFFFFFFFFFFFFFFFFFFFFFFDC
  if (data[4] == 0xffffffff) {
    memset(bytes, 0xff, 512);
    data[0] = 0xfe9a9239; // 0xF69A9239
    data[1] = 0xaeecff80;
    data[4] = 0x00804010;
  }


  // Protect the bootloader area.
  data[0] = (data[0] & ~NVMCTRL_FUSES_BOOTPROT_Msk) | NVMCTRL_FUSES_BOOTPROT(13);

  // Enable the Brown-Out Detector
  data[0] &= ~FUSES_BOD33_DIS_Msk;

  // Set EEPROM size (4 Kb)
  data[1] = (data[1] & ~NVMCTRL_FUSES_SEESBLK_Msk) | NVMCTRL_FUSES_SEESBLK(1);
  data[1] = (data[1] & ~NVMCTRL_FUSES_SEEPSZ_Msk) | NVMCTRL_FUSES_SEEPSZ(3);

  // Add our magic, it will skip this configuration at startup.
  data[8] = magic;

  Serial.println("New FUSES");
  Serial.println( data[0],HEX); Serial.println( data[1],HEX); Serial.println( data[2],HEX); Serial.println( data[3],HEX);
  Serial.println( data[4],HEX); Serial.println( data[5],HEX); Serial.println( data[6],HEX); Serial.println( data[7],HEX);
  Serial.println( data[8],HEX);
  
  
  // uncomment to Write Fuses
  // return;


  // V2Memory::Flash::UserPage::write(data);
  while (NVMCTRL->STATUS.bit.READY == 0)
    ;

  // Manual write
  NVMCTRL->CTRLA.bit.WMODE = NVMCTRL_CTRLA_WMODE_MAN;

  // Erase page
  NVMCTRL->ADDR.reg  = (uint32_t)NVMCTRL_USER;
  NVMCTRL->CTRLB.reg = NVMCTRL_CTRLB_CMDEX_KEY | NVMCTRL_CTRLB_CMD_EP;
  while (NVMCTRL->STATUS.bit.READY == 0)
    ;

  // Page buffer clear
  NVMCTRL->ADDR.reg  = (uint32_t)NVMCTRL_USER;
  NVMCTRL->CTRLB.reg = NVMCTRL_CTRLB_CMDEX_KEY | NVMCTRL_CTRLB_CMD_PBC;
  while (NVMCTRL->STATUS.bit.READY == 0)
    ;

  // Write page
  uint32_t *addr = (uint32_t *)NVMCTRL_USER;
  for (uint8_t i = 0; i < 128; i += 4) {
    addr[i + 0] = data[i + 0];
    addr[i + 1] = data[i + 1];
    addr[i + 2] = data[i + 2];
    addr[i + 3] = data[i + 3];

    // Write quad word (128 bits)
    NVMCTRL->ADDR.reg  = (uint32_t)(addr + i);
    NVMCTRL->CTRLB.reg = NVMCTRL_CTRLB_CMDEX_KEY | NVMCTRL_CTRLB_CMD_WQW;
    while (NVMCTRL->STATUS.bit.READY == 0)
      ;
  }
  // Reboot to enable the new settings.
  delay(100);
  NVIC_SystemReset();

}

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
const uint32_t  tc_usec_timer     = 80;     // 38 == 26.kKhz | 40 == 25KH | 80 == 12.5Khz Achtung darunter Probleme Attack/Sustain Reading from ADC´s !!!!
const double    samplerate        = 1.0 / (double)tc_usec_timer * 1000000.0;  // Steuert die HK Zeit auf 1Sek bei SampleRate * 10 => 10Sekunden
const double    max_attack_time   = samplerate  * 10;  // Time Range for ADSR
const double    max_release_time  = samplerate * 20;  // Time Range for ADSR
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
// bool     noise_led=false;

//
//  OUTPUT AUDIO sample
//
void outputSample() {
  DAC->DATA[0].reg = DACValue0;
  DAC->DATA[1].reg = DACValue1;
  /*
  if(noise_led)  PORT->Group[PORTB].OUTCLR.reg = 1ul << 0;  else   PORT->Group[PORTB].OUTSET.reg = 1ul << 0;    // LED

  #ifndef DEBUG_TIMING_BY_SQR_OUT  
    if(noise_led)  PORT->Group[PORTA].OUTSET.reg = 1ul << 22; else PORT->Group[PORTA].OUTCLR.reg = 1ul << 22;
  #endif
  */
}


void renderAudio() {

  // Gate Status checken
  bool mygate = false;
  if ( ! (PORT->Group[PORTA].IN.reg & (1ul << 00))  ) {  // Trigger from Button
      mygate=true; // no gate from Button or Jack
  }
  if ( (PORT->Group[PORTA].IN.reg & (1ul << 18))   ) {  // Trigger from jack
      mygate=true; // no gate from Button or Jack
  }  

  if(mygate)      PORT->Group[PORTB].OUTSET.reg = 1ul << 31;
  else            PORT->Group[PORTB].OUTCLR.reg = 1ul << 31;


  // if change gate...handle in adsr
  static bool zm_gate_before=false;
  if(mygate != zm_gate_before)
    myADSR.setNewGateState(mygate); // Note On/Off
  zm_gate_before = mygate;
  
  //  AutoLoop ?
  if(!(PORT->Group[PORTA].IN.reg & (1ul << 21))) { // PA21 ADSR Loop
      if(myADSR.getState()==ZM_ADSR::env_idle){
        myADSR.setNewGateState(true);
        // thea_sample=0.0f;
      }
      if(myADSR.getState()==ZM_ADSR::env_sustain){
        myADSR.setNewGateState(false);
      }
  }

  /*
  // LED On when in AttackPhase
  if(myADSR.getState()==ZM_ADSR::env_attack){
    noise_led=false;  // LED ON
  }else{
    noise_led=true;
  }
  */

  // Scale to Integer and mark for Output
  uint16_t rt_env_output = myADSR.process() * 4095.0; // scale and convert to unipolar
  // DACValue1 = rt_env_output;
  DACValue1 = rt_env_output;


  // LFO

  // Forward Button + Trigger to LED
  static uint32_t leftSamples = 1000000;
  static bool     previous_trigger = false;
  if (  (PORT->Group[PORTB].IN.reg & (1ul << 16))  &&      // Trigger from Button
        (PORT->Group[PORTB].IN.reg & (1ul << 23))     ) {  // Trigger from jack
    // PORT->Group[PORTB].OUTCLR.reg = 1ul << 31;             // trigger fire!
    previous_trigger=false;
  } else {
    // PORT->Group[PORTB].OUTSET.reg = 1ul << 31;
    if(!previous_trigger){          // check for edge
      leftSamples = 1000000; // ratchet_counts;
      thea_sample=0.0f; // retrigger 
    }
    previous_trigger=true;
  }


  if(leftSamples>0){
    thea_sample+=inc_sample;
    if(thea_sample>1.0f){
      thea_sample-=1.0f;
      leftSamples--;
    }

    if(thea_sample<0.2f){
        PORT->Group[PORTB].OUTSET.reg = 1ul << 30;
    }else  {
        PORT->Group[PORTB].OUTCLR.reg = 1ul << 30;
    }         


    const float sample_raw_len =  4095.0;

    uint32_t sample_idx = sample_offset + (uint32_t)(sample_raw_len * thea_sample);
    DACValue0 = wave_tables_lfo[sample_idx & 0x7fff];  // extend to 32 Bit
  }


}

void loop() { // never reached
    while(true);
}



void loop2() {

  #ifdef DEBUG_TIMING_BY_SQR_OUT
  PORT->Group[PORTA].OUTSET.reg = 1ul << 17;  // end of attack
  #endif


  outputSample(); // wird immer zuerst ausgegeben um jitter durch laufzeitunterschiede zu minimieren daher ein sample delay
  renderAudio();  // die eigenliche Sampleerzeugung

  // Ab hier wir lesen abwechselnd ADC´s und berechnen neue Timing Werte
  static uint16_t noise_pitch_jack          = 2048;
  static uint16_t lfo_wave_endlesspoti1     = 2048;
  static uint16_t lfo_wave_endlesspoti2     = 2048;
  static uint16_t lfo_wave_cv               = 2048;

  static uint16_t lfo_speed_poti       = 2048;
  static uint16_t lfo_speed_cv         = 2048;


  static uint16_t adsr_all_ratio       = 2048; 
  static uint16_t ratchet_adc          = 2048; 
  static uint16_t adsr_cv_attack       = 2048;
  static uint16_t adsr_cv_decay        = 2048;
  static uint16_t adsr_cv_sustain      = 2048;
  static uint16_t adsr_cv_release      = 2048;  
  static uint16_t adsr_slider_attack   = 2048; 
  static uint16_t adsr_slider_decay    = 2048; 
  static uint16_t adsr_slider_sustain  = 2048; 
  static uint16_t adsr_slider_release  = 2048; 

  static uint16_t adc_state_machine = 0;
  adc_state_machine++;
  adc_state_machine&=0x000f;

  // Freese ADC READING!!!! for Debug jitter only!
  bool is_AB_switch = PORT->Group[PORTB].IN.reg & (1ul << 17);  // A/B Knopf
  if(!is_AB_switch){ // in B modus ???
      adc_state_machine=20;
  }

  float pitch_lfo;

  switch(adc_state_machine) {
    case 0: 
      adsr_cv_release = adc51.readLastValue();
      // adc51.startReadAnalog(PB01,ADC_Channel13,false);      // Buchse #2 (erste Digitale) Signal: Digital_Noise_Pitch normalized 12v
      adc51.startReadAnalog(PA07,ADC_Channel7,false);      // Poti LFO Speed
      break;
    case 1: 
      lfo_speed_poti = adc51.readLastValue();

      pitch_lfo = 6000 - (lfo_speed_poti + lfo_speed_cv);
      pitch_lfo = RANGE(0,pitch_lfo,4096);
      inc_sample = 1.0 / (float) pitch_lfo * 50.0;
      inc_sample = inc_sample * inc_sample  * inc_sample ;
      if(inc_sample>0.02) inc_sample= 0.02;
      // inc_sample=0.01;
      adc51.startReadAnalog(PB02,ADC_Channel14,false);      // LFO Wave Endless Poti1
      break;
    case 2:  
      lfo_wave_endlesspoti1 = adc51.readLastValue();
      sample_offset = (RANGE(0,lfo_wave_endlesspoti2 + lfo_wave_cv - 2000,4096)) * 7;
      // adc51.startReadAnalog(PA07,ADC_Channel7,false); 
      adc51.startReadAnalog(PB06,ADC_Channel8,true);      // SliderPoti Attack
      break;
    case 3:  
      adsr_slider_attack = adc51.readLastValue();
      myADSR.setAttackRate((lut[adsr_slider_attack] + lut[adsr_cv_attack] ) *  max_attack_time );
      // adc51.startReadAnalog(PA06,ADC_Channel6,true);  // LFO Speed CV
      break;
    case 4:  
      // sample_pitch_jack = adc51.readLastValue();
      // adc51.startReadAnalog(PB03,ADC_Channel15,false);  // Manual_6XSqr_Pitch Poti
      adc51.startReadAnalog(PB08,ADC_Channel2,false);  // Sustain Slider
      break;
    case 5:  
      adsr_slider_sustain = adc51.readLastValue();
      myADSR.setSustainLevel((double)adsr_slider_sustain * scale_4  + (double)adsr_cv_sustain * scale_4 );
      adc51.startReadAnalog(PA06,ADC_Channel6,false);  
      break;
    case 6:  
      lfo_speed_cv = adc51.readLastValue();
      adc51.startReadAnalog(PB09,ADC_Channel3,false); // Release Slider
      break;
    case 7:  
      adsr_slider_release = adc51.readLastValue();
      myADSR.setReleaseRate( (lut[adsr_slider_release] + lut[adsr_cv_release]) *  max_release_time);
      // adc51.startReadAnalog(PB09,ADC_Channel3,false);
      break;
    case 8:  
      // cv_sample_select_adc = adc51.readLastValue();
      // PORT->Group[PORTB].OUTSET.reg = 1ul << 12;          // Setup Mux to read PotiExpAdsr
      adc51.startReadAnalog(PB00,ADC_Channel12,false);   // PortMuliplexer  PB12: LOW:  TriggerMode   HIGH: ADSR_Ratio
      break;
    case 9:  
      adsr_all_ratio = adc51.readLastValue();
      //adsr_all_ratio = 1000;
      myADSR.setTargetRatioAll( lut[adsr_all_ratio] );
      adc51.startReadAnalog(PB07,ADC_Channel9,true);    // Decay Slider
      break;
    case 10:
      adsr_slider_decay = adc51.readLastValue();
      myADSR.setDecayRate((lut[adsr_slider_decay] + lut[adsr_cv_decay] ) * max_release_time );
      adc51.startReadAnalog(PA08,ADC_Channel8,false); // CV IN AdsrAttack
      break;
    case 11:
      adsr_cv_attack = adc51.readLastValue();
      adc51.startReadAnalog(PA09,ADC_Channel9,false); // adsr decay cv
      break;
    case 12:
      adsr_cv_decay = adc51.readLastValue();
      adc51.startReadAnalog(PA10,ADC_Channel10,false); // adsr sustain cv
      break;
    case 13:
      adsr_cv_sustain = adc51.readLastValue();
      adc51.startReadAnalog(PB01,ADC_Channel13,false); // lfo endletPoti 2
      break;
    case 14:
      lfo_wave_endlesspoti2 = adc51.readLastValue();
      adc51.startReadAnalog(PB04,ADC_Channel6,true); // lfo endletPoti 2
      break;
    case 15:
      lfo_wave_cv = adc51.readLastValue();
      adc51.startReadAnalog(PA11,ADC_Channel11,false); // adsr relase cv
      break;
    }

   #ifdef DEBUG_TIMING_BY_SQR_OUT
   PORT->Group[PORTA].OUTCLR.reg = 1ul << 17; 
   #endif
}

void setup() {
  //put your setup code here, to run once:
  // First try remove DC from Ouputs  

  // fixBOD33();

  pinMode(PA23,OUTPUT); // LED Bootloader
  pinMode(PB30,OUTPUT); // LED Trigger IN LFO
  pinMode(PB31,OUTPUT); // LED ADSR GateIn

  pinMode(PA17,OUTPUT); // end OF AttackPhase
  pinMode(PA16,OUTPUT); // end OF Decay


  pinMode(PA13,INPUT); digitalWrite(PA13,false);
  pinMode(PA14,INPUT); digitalWrite(PA14,false);
  pinMode(PA15,INPUT); digitalWrite(PA15,false);
  // pinMode(PA16,INPUT); digitalWrite(PA16,false);
  // pinMode(PA17,INPUT); digitalWrite(PA17,false);
  pinMode(PA18,INPUT); digitalWrite(PA18,false);


  pinMode(PB12,OUTPUT);  // Multiplexer
  digitalWrite(PB12,true);

  pinMode(PA00,INPUT);  // Button (Encoder)
  pinMode(PA18,INPUT);  // Jack 

  // PA20 adsr Ratched Switch
  // PA21 adsr Loop Switch
  // PB17 adsr A/B Switch


  // PB05 adsr Poti Multiplayer


/*
  pinMode(PA19,OUTPUT); // SQR1
  pinMode(PA12,OUTPUT); // SQR2
  pinMode(PB15,OUTPUT); // SQR1
  pinMode(PB14,OUTPUT); // SQR1
  pinMode(PB13,OUTPUT); // SQR1
  pinMode(PB12,OUTPUT); // SQR1

  pinMode(PA22,OUTPUT); // PWM OUT
*/
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
  
  // Create LFO WaveTable
  uint16_t iw = 0;
  for(int i = 0 ; i < 4096; i++)     wave_tables_lfo[0x7fff & iw++]=4095;
  for(int i = 0 ; i < 4096; i++)     wave_tables_lfo[0x7fff & iw++]=0;
  for(int i = 0 ; i < 4096; i++)     wave_tables_lfo[0x7fff & iw++]=i;
  for(int i = 0 ; i < 4096; i++)     wave_tables_lfo[0x7fff & iw++]=i;

  for(int i = 0 ; i < 4096; i++)     wave_tables_lfo[0x7fff & iw++]=(cosf((float) i / 2048.0 * PI) + 1.0) * 2046.0;
  for(int i = 0 ; i < 4096; i++)     wave_tables_lfo[0x7fff & iw++]=(cosf((float) i / 2048.0 * PI) + 1.0) * 2046.0;

  for(int i = 0 ; i < 4096; i++)     wave_tables_lfo[0x7fff & iw++]=0;
  for(int i = 0 ; i < 4096; i++)     wave_tables_lfo[0x7fff & iw++]=4095;


  /*
  while(1){
    while(SysTick->VAL & 0xff); // Keine gute Idee da wir ja nicht jeden systick finden, evtl unten noch Bitmask setzen ?!?!?!
    loop2();
  } */

  // Achtung! diese Variante erzeugt Doubletten, also Runtime checken
  TC.startTimer(tc_usec_timer, loop2); // 32 usec , wenn hier zu schnell jitter + ADC Lesefehler

  //for(;;) loop2();  // Keine Gute idee, da jitter durch Laufzeitunterschiede!

}