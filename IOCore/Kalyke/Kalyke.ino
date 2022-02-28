#include <Wire.h>
#include "wiring_private.h"
#include "SAMD51_InterruptTimer.h"
#include "samd51_adc.h"
#include "adsr_class.h"
// #include "lfo_wave.h"
// Test Uwe
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



// Wenn ratched in der ADSR Aktiv ist wird eine zusätzliche MSEG Hülkurve beim Attack gestartet. Die Attackzeit ist Fix auf einen kurzen Wert eingestellt siehe array_claps_attacks
// Jede MSEG Phase wird durch abwechselnde poitive und negative increments geformt. Bei überschreitung von 1.0 oder unterschreitung von 0.0 
// erfolgt ein step zum nächsten wert. ist der Wert 0 wird die Hüllkurve gestoppt und im ausgang auf 1.0 gesetzt
// ADSR und MSEG werden durch einen VCA verbunden.
const float array_claps [16][16] ={ 
    {0.8, -0.010, 0.4, -0.0, 0.12, -0.004, 0.12, -0.00, 0.06, -0.003, 0.12, -0.00, 0.10, -0.004, 0.004, -0.0, },
    {0.8, -0.004, 0.4, -0.004, 0.8, -0.003, 0.4, -0.00, 0.06, -0.003, 0.12, 0.00, 0.12, -0.004, 0.004, 0.0, },
    {0.6, -0.003, 0.4, -0.003, 0.5, -0.003, 0.50, -0.003, 0.5, -0.004, 0.42, -0.00, 0.5, -0.004, 0.5, -0.000, },
    {0.8, -0.002, 0.6, -0.003, 0.6, -0.002, 0.5, -0.003, 0.6, -0.002, 0.12, -0.003, 0.12, -0.002, 0.12, 0.000, },
    {0.6, -0.0015, 0.3, -0.0016, 0.5, -0.0016, 0.20, -0.0015, 0.5, -0.0012, 0.12,-0.0015, 0.6, -0.0010, 0.14, -0.0, }, 
    {0.6, -0.0012, 0.3, -0.0013, 0.5, -0.0012, 0.20, -0.0013, 0.5, -0.0011, 0.12,-0.0013, 0.6, -0.0012, 0.14, -0.0,}, 
    {0.6, -0.0010, 0.3, -0.0009, 0.5, -0.0010, 0.2, -0.0009, 0.5, -0.0010, 0.3, -0.0009, 0.5, -0.0010, 0.4, -0.00, }, 
    {0.6,-0.00095, 0.3,-0.00085, 0.5,-0.00095, 0.2,-0.00085, 0.4,-0.00095, 0.3,-0.00085, 0.5,-0.00095, 0.4,-0.0,},
    {0.6, -0.0009, 0.3, -0.0008, 0.5, -0.0009, 0.2, -0.0008, 0.4, -0.0009, 0.3, -0.0008, 0.5, -0.0009, 0.4, -0.00, }, 
    {0.6,-0.00085, 0.3, -0.0008, 0.5, -0.00085, 0.2, -0.0008, 0.4, -0.00085, 0.3,-0.0008, 0.5,-0.00085, 0.4,-0.00,}, 
    {0.6, -0.0008, 0.3, -0.00075, 0.5, -0.0009, 0.2, -0.0008, 0.4, -0.0009, 0.3, -0.0008, 0.5, -0.0009, 0.4, -0.00, }, 
    {0.6,-0.0007, 0.3,-0.00075, 0.5, -0.0007, 0.2, -0.00075, 0.4,-0.0007, 0.3, -0.00075, 0.5, -0.0007, 0.4, -0.00,},
    {0.6, -0.0006, 0.3, -0.00065, 0.5, -0.0006, 0.2, -0.0006, 0.4, -0.0006, 0.3, -0.0007, 0.5, -0.0006, 0.4, -0.00, },
    {0.6, -0.0004, 0.3, -0.0005, 0.5, -0.0004, 0.2, -0.0005, 0.4, -0.0004, 0.3, -0.0006, 0.5, -0.0005, 0.4, -0.00, },
    {0.004, -0.004, 0.004, -0.004, 0.004, -0.004, 0.004, -0.004, 0.003, -0.003, 0.003, -0.003, 0.002, -0.002, 0.002, -0.000, },
    {0.003, -0.002, 0.002, -0.002, 0.003, -0.003, 0.002, -0.001, 0.003, -0.003, 0.003, -0.003, 0.002, -0.002, 0.002, -0.000,} 
};

const float array_claps_attacks [16] = { 
                                          0.00003, 0.00003, 0.00003, 0.00003, 
                                          0.00002, 0.00002, 0.00004, 0.00002,
                                          0.00001, 0.00002, 0.00005, 0.00002,
                                          0.00005, 0.0001, 0.001, 0.002, };

uint16_t wave_tables_lfo[4096 * 8];

float lfo_inc_tab[4096];


SAMD51_ADC adc51;
ZM_ADSR myADSR;
LFSR lsfr1(0xA1e);
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
const double    scale_4           = 1.0/4096.0;           // Das Brauchen wir zum scale von ADC Integers -> 0.00 ... 1.00
const uint32_t  tc_usec_timer     = 80;                   // 38 == 26.kKhz | 40 == 25KH | 80 == 12.5Khz Achtung darunter Probleme Attack/Sustain Reading from ADC´s !!!!
const double    samplerate        = 1.0 / (double)tc_usec_timer * 1000000.0;  // Steuert die HK Zeit auf 1Sek bei SampleRate * 10 => 10Sekunden
const double    max_attack_time   = samplerate  * 10;     // Time Range for ADSR
const double    max_release_time  = samplerate * 20;      // Time Range for ADSR
double          lut[4096];                                // lookupTable adc -> double Value 1/4096 ^ 3
uint8_t         adsr_ratched_countdown = 0;
uint8_t         adsr_ratched_prg = 0 ;                    // 0..15
bool            adsr_ratched_mode;
// Use this to figure out best tc_usec_timer value!!!!
// #define DEBUG_TIMING_BY_SQR_OUT 1


//
// LFO
//
float thea_sample       = 0.0f;
float inc_sample        = 0.12f;
uint16_t sample_offset  = 0;
// uint16_t test_ratched   = 0;
uint16_t lfo_ratchet_count = 0;


float mod_adsr2lfo = 0;
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

  if(mygate)      PORT->Group[PORTB].OUTSET.reg = 1ul << 31;  // GATE IN LED
  else            PORT->Group[PORTB].OUTCLR.reg = 1ul << 31;  // GATE IN LED

  adsr_ratched_mode = false;

  // if change gate...handle in adsr
  static bool zm_gate_before=false;
  if(mygate != zm_gate_before) {
      myADSR.setNewGateState(mygate); // Note On/Off
      if(mygate) { // Gate wurde ausgelöst per knopf oder Gate Input
        adsr_ratched_countdown=16;
      }
  }
  zm_gate_before = mygate;
  
  //  AutoLoop ?
  if(!(PORT->Group[PORTA].IN.reg & (1ul << 21))) { // PA21 ADSR Loop switch
      if(myADSR.getState()==ZM_ADSR::env_idle){
        myADSR.setNewGateState(true);
        adsr_ratched_countdown = 16;
        // thea_sample=0.0f;
      }
      if(myADSR.getState()==ZM_ADSR::env_sustain){
        myADSR.setNewGateState(false);
      }
      
  }
  if(!(PORT->Group[PORTA].IN.reg & (1ul << 20))) { // PA20 ADSR ratched Mode
    adsr_ratched_mode = true;
  }

  static float adsr_ratched_thea = 0;
  if(adsr_ratched_countdown>0){
    float adsr_ratched_lfo_inc = array_claps[adsr_ratched_prg][16-adsr_ratched_countdown];
    if(adsr_ratched_lfo_inc==0)
      adsr_ratched_countdown=0;

    adsr_ratched_thea+=adsr_ratched_lfo_inc;
    if(adsr_ratched_thea > 1.0){
      adsr_ratched_countdown--;
      adsr_ratched_thea=1.0;
    }
    if(adsr_ratched_thea < 0.0){
      adsr_ratched_countdown--;
      adsr_ratched_thea=0.0;
    }
  }else{
    adsr_ratched_thea=1.0;
  }

  if(!adsr_ratched_mode){
    adsr_ratched_thea=1.0;
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
  uint16_t rt_env_output = myADSR.process() * adsr_ratched_thea * 4095.0; // scale and convert to unipolar
  // DACValue1 = rt_env_output;
  DACValue1 = rt_env_output;


  // LFO

  // Forward Button + Trigger to LED
  static uint32_t leftSamples = 1;
  static bool     previous_trigger = false;
  if (  !(PORT->Group[PORTA].IN.reg & (1ul << 19))  &&      // PA19 Trig_In_LFO (JACK) 
        (PORT->Group[PORTA].IN.reg & (1ul << 00))     ) {  // PA00 Manual_Trig_SW (Trigger Endlesspoti)
    previous_trigger=false;
  } else {
    if(!previous_trigger){          // check for edge
      leftSamples = lfo_ratchet_count + 1; // ratchet_counts; Da gleich die phase resettet wird brauchen wir minimum 2
      thea_sample=0.99999f; // retrigger 
    }
    previous_trigger=true;
  }

  static uint16_t sample_and_hold_value = 2048;
 
  if(lfo_ratchet_count>14)  leftSamples = 10; // LFO Unendlich laufen lassen

  if(leftSamples>0){
    thea_sample+=inc_sample;
    if(thea_sample>1.0f){
      thea_sample-=1.0f;
      leftSamples--;
      sample_and_hold_value = lsfr1.next()/16;
    }

    if(thea_sample<0.2f){
        PORT->Group[PORTB].OUTSET.reg = 1ul << 30;
    }else  {
        PORT->Group[PORTB].OUTCLR.reg = 1ul << 30;
    }         


    const float sample_raw_len =  4095.0;

    uint32_t sample_idx = sample_offset + (uint32_t)(sample_raw_len * thea_sample);
    DACValue0 = wave_tables_lfo[sample_idx & 0x7fff];  // extend to 32 Bit
  }else{
    DACValue0=0;
  }
  
  lsfr1.next();

  if(!(PORT->Group[PORTB].IN.reg & (1ul << 16))) { // PB16 S/H früher S/F
    // leftSamples = 1; // LFO dauerhaft laufen lassen und neue S/H Values generieren
    DACValue0=sample_and_hold_value;   // Keinesample_and_hold_value << 4 ; // + */ test_ratched;
  }


  //now mod LFO
  
  float vca_adsr = mod_adsr2lfo / 4096.0;
  float vca_lfo = 1.0 - vca_adsr;
  float adsr_anteil = (float) DACValue1 / 4096.0 * vca_adsr;
  DACValue0 = 4095 - ( (float)DACValue0 * vca_lfo + (float)DACValue0 * adsr_anteil );
  //DACValue0 = mod_adsr2lfo;
}

void loop() { // never reached
    while(true);
}



void loop2() {

  #ifdef DEBUG_TIMING_BY_SQR_OUT
  PORT->Group[PORTA].OUTSET.reg = 1ul << 17;  // // EA_Trig EndOfAttack
  #endif


  outputSample(); // wird immer zuerst ausgegeben um jitter durch laufzeitunterschiede zu minimieren daher ein sample delay
  renderAudio();  // die eigenliche Sampleerzeugung

  // Ab hier wir lesen abwechselnd ADC´s und berechnen neue Timing Werte
  // static uint16_t noise_pitch_jack          = 2048;
  static uint16_t lfo_wave_endlesspoti1     = 2048;   // not use
  static uint16_t lfo_wave_endlesspoti2     = 2048;
  static uint16_t lfo_wave_cv               = 2048;

  static uint16_t lfo_speed_poti       = 2048;
  static uint16_t lfo_speed_cv         = 2048;
  
  static uint16_t lfo_ratchet_poti     = 2048; 
  static uint16_t lfo_decay_drywet     = 2048; 

  
  
  static uint16_t adsr_all_ratio       = 2048; 
  
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
  float inc_sample_temp;
  const float inc_sample_case = 1.0 / 20000.0;

  switch(adc_state_machine) {
    case 0: 
      adsr_cv_release = adc51.readLastValue();
      adc51.startReadAnalog(PA07,ADC_Channel7,false);      // Poti LFO Speed
      break;
    case 1: 
      lfo_speed_poti = adc51.readLastValue();
      pitch_lfo = (lfo_speed_poti + lfo_speed_cv / 2.0  - 1536); // CV Input scheint auf Mitte Normalisiert
      pitch_lfo = RANGE(0,pitch_lfo,4095) ;

      // inc_sample =  4.0f * powf(2.0f,( pitch_lfo - 2048.0f ) * 0.004 ) / samplerate;
      inc_sample =lfo_inc_tab[(uint16_t)pitch_lfo];
      

      if(inc_sample>0.2) inc_sample= 0.2;                 // ca 2500 Hz Limit LFO Speed
      if(inc_sample<0)   inc_sample = 0.0000001;
      adc51.startReadAnalog(PB02,ADC_Channel14,false);      // LFO Wave Endless Poti1
      break;
    case 2:  
      lfo_wave_endlesspoti1 = adc51.readLastValue();
      adc51.startReadAnalog(PB06,ADC_Channel8,true);      // SliderPoti Attack
      break;
    case 3:  
      adsr_slider_attack = adc51.readLastValue();
      adsr_ratched_prg = MAP_RANGE(adsr_slider_attack,180,3800, 0, 15);
      if (adsr_ratched_mode)
        myADSR.setAttackRate( array_claps_attacks[adsr_ratched_prg] *  max_attack_time );
      else
        myADSR.setAttackRate((lut[adsr_slider_attack] + lut[adsr_cv_attack] ) *  max_attack_time );
      PORT->Group[PORTB].OUTCLR.reg = 1ul << 12;      // switch AnalogMUX an KALYKE
      adc51.startReadAnalog(PB00,ADC_Channel12,false);  // TriggerMode==
      break;
    case 4:  
      lfo_ratchet_poti = adc51.readLastValue();
      lfo_ratchet_count = MAP_RANGE(lfo_ratchet_poti,350,3650, 1, 15);
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
      // adc51.startReadAnalog(PB05,ADC_Channel7,true);      // Kalyke MiniPoti entfällt, erstmal provisorisch nutzen
      adc51.startReadAnalog(PB03,ADC_Channel15,false);  // Kalyke Fade IN:  TODO: reagiert nicht
      break;
    case 8:  
      lfo_decay_drywet = adc51.readLastValue();   // Hier steht der Value vom mini poti rechts an den jacks der später entfällt
      mod_adsr2lfo = MAP_RANGE(lfo_decay_drywet,250,3650, 0, 4095);
      // test_ratched = lfo_decay_drywet;
      PORT->Group[PORTB].OUTSET.reg = 1ul << 12;          // Setup Mux to read PotiExpAdsr
      adc51.startReadAnalog(PB00,ADC_Channel12,false);   // PortMuliplexer  PB12: LOW:  TriggerMode   HIGH: ADSR_Ratio
      break;
    case 9:  
      adsr_all_ratio = adc51.readLastValue();
      myADSR.setTargetRatioAll( lut[adsr_all_ratio] );
      adc51.startReadAnalog(PB07,ADC_Channel9,true);    // Decay Slider
      break;
    case 10:
      adsr_slider_decay = adc51.readLastValue();
      //if (!adsr_ratched_mode)
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
//      adc51.startReadAnalog(PB01,ADC_Channel13,false); // lfo endletPoti 2
      adc51.startReadAnalog(PB05,ADC_Channel7,true);      // Kalyke MiniPoti entfällt, erstmal provisorisch nutzen
      break;
    case 14:
      lfo_wave_endlesspoti2 = adc51.readLastValue();
      sample_offset = (RANGE(0,lfo_wave_endlesspoti2 + lfo_wave_cv - 2000,4096)) * 7;      
      adc51.startReadAnalog(PB04,ADC_Channel6,true); // lfo endletPoti 2
      break;
    case 15:
      lfo_wave_cv = adc51.readLastValue();
      adc51.startReadAnalog(PA11,ADC_Channel11,false); // adsr relase cv
      break;
    }

   #ifdef DEBUG_TIMING_BY_SQR_OUT
   PORT->Group[PORTA].OUTCLR.reg = 1ul << 17;   // EA_Trig EndOfAttack
   #endif
}

void setup() {
  //put your setup code here, to run once:
  // First try remove DC from Ouputs  

  // fixBOD33();

  pinMode(PA23,OUTPUT); // LED Bootloader
  pinMode(PB30,OUTPUT); // LED Trigger IN LFO
  pinMode(PB31,OUTPUT); // LED ADSR GateIn

  pinMode(PA17,OUTPUT); // EA_Trig end OF AttackPhase
  pinMode(PA16,OUTPUT); // ED_Trig end OF Decay


  pinMode(PA13,INPUT); digitalWrite(PA13,false);
  pinMode(PA14,INPUT); digitalWrite(PA14,false);
  pinMode(PA15,INPUT); digitalWrite(PA15,false);


  pinMode(PB12,OUTPUT);       // Multiplexer ADC ADSR_Curv/Trig_Mode
  digitalWrite(PB12,true);

  pinMode(PA00,INPUT);        // Manual_Trig_SW Button (Encoder)
  pinMode(PA18,INPUT);        // Gate_In_ADSR  Jack
  pinMode(PB16,INPUT_PULLUP); // Slow_Fast_Tempo_SW S/F Modue Button nun S/H
  pinMode(PA20,INPUT_PULLUP); // Ratched_SW 


  // configure Cap Filters
  pinMode(PB14,INPUT); // LFO_FILTER_CAP1
  pinMode(PB15,INPUT); // LFO_FILTER_CAP2
  pinMode(PA12,INPUT); // LFO_FILTER_CAP3
  digitalWrite(PB14,false);
  digitalWrite(PB15,false);
  digitalWrite(PA12,false);


  pinMode(PA13,INPUT); // ADSR_FILTER_CAP1
  pinMode(PA14,INPUT); // ADSR_FILTER_CAP2
  pinMode(PA15,INPUT); // ADSR_FILTER_CAP3
  digitalWrite(PA13,false);
  digitalWrite(PA14,false);
  digitalWrite(PA15,false);


  // PA20 adsr Ratched Switch
  // PA21 adsr Loop Switch
  // PB17 adsr A/B Switch
  // KALYKE Pin PB23 DC_Trim_SW Taster Rückseite neben Reset 
  // 


  // PB05 adsr Poti Multiplayer
  pinMode(PA19,INPUT);          // PA19 Trig_In_LFO from JACK
  pinMode(PB01,INPUT);          // Wave_Select_Manual_2 (endlespoti)
  pinMode(PB02,INPUT);          // Wave_Select_Manual_1 (endlespoti not used)
  pinMode(PA21,INPUT_PULLUP);   // Loop_SW ADSR
  pinMode(PB17,INPUT_PULLUP);   // AB Switch Neu: Freeze
  pinMode(PB23,INPUT_PULLUP);   // Manual Trigger

  dacInit();
  DAC->DATA[0].reg = 2048;        // ca 1.5V PA05
  DAC->DATA[1].reg = 2048;        // ca 1.5V PA02
  while (DAC->SYNCBUSY.bit.DATA0);

  for(int i = 0 ; i < 4096; i++) {
    double scale = (double) i *  1.0/4096.0;
    lut[i] = scale * scale * scale ;

    // create inc lfo table at base 4Hz in Center
    // 4HZ Base Freq * pow(2,   (-8 Oktaven.... +8 Oktaven)) / samplerate
    lfo_inc_tab[i] =  4.0f * powf(2.0f,( (float)i - 2048.0f ) * 0.004 ) / samplerate;
  }


  // Timing SampleRate
  adc51.createADCMap(50000*2); // 50Khz but pitch down by * 2
  
  // Create LFO WaveTable
  uint16_t iw = 0;
  // SQR
  for(int i = 0 ; i < 4096; i++)     wave_tables_lfo[0x7fff & iw++]=4095;
  for(int i = 0 ; i < 4096; i++)     wave_tables_lfo[0x7fff & iw++]=0;
  // COS
  for(int i = 0 ; i < 4096; i++)     wave_tables_lfo[0x7fff & iw++]=(sinf((float) i / 2048.0 * PI) + 1.0) * 2046.0;
  for(int i = 0 ; i < 4096; i++)     wave_tables_lfo[0x7fff & iw++]=(sinf((float) i / 2048.0 * PI) + 1.0) * 2046.0;
  // SAW
  for(int i = 0 ; i < 4096; i++)     wave_tables_lfo[0x7fff & iw++]=i;
  for(int i = 0 ; i < 4096; i++)     wave_tables_lfo[0x7fff & iw++]=i;
  // TRI (halbe Phasen)
  for(int i = 0 ; i < 4096; i+=2)    wave_tables_lfo[0x7fff & iw++]=4096 - i;
  for(int i = 0 ; i < 4096; i+=2)    wave_tables_lfo[0x7fff & iw++]=i;
  for(int i = 0 ; i < 4096; i+=2)    wave_tables_lfo[0x7fff & iw++]=4096 - i;
  for(int i = 0 ; i < 4096; i+=2)    wave_tables_lfo[0x7fff & iw++]=i;
 
  // Enable Random Generator
  MCLK->APBCMASK.reg |= MCLK_APBCMASK_TRNG;
  //Enable TRNG
  TRNG->CTRLA.reg |= TRNG_CTRLA_ENABLE;
  //Wait for the TRNG to contain a valid data
  while((TRNG->INTFLAG.reg & TRNG_INTFLAG_DATARDY) == 0) {}

  //Get the 32-bit random value
  // value = TRNG->DATA.reg;

  /*
  while(1){
    while(SysTick->VAL & 0xff); // Keine gute Idee da wir ja nicht jeden systick finden, evtl unten noch Bitmask setzen ?!?!?!
    loop2();
  } */

  // Achtung! diese Variante erzeugt Doubletten, also Runtime checken
  TC.startTimer(tc_usec_timer, loop2); // 32 usec , wenn hier zu schnell jitter + ADC Lesefehler

  //for(;;) loop2();  // Keine Gute idee, da jitter durch Laufzeitunterschiede!

}
