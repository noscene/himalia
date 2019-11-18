
#include <Wire.h>
#include "wiring_private.h"
#include "Adafruit_ZeroTimer.h"
#include "SdFat.h"
#include "Adafruit_SPIFlash.h"

#include "samd51_adc.h"


// const frequency = Math.pow(2, (m - 69) / 12) * 440;
// incrementer unipolar = 1/SR * f
// incrementer bipolar = 2/SR * f 


#include "s0.h"
#include "s1.h"
#include "s2.h"
#include "s3.h"
#include "s4.h"
#include "s5.h"
#include "s6.h"
#include "s7.h"
#include "s8.h"
#include "s9.h"
#include "s10.h"
#include "s11.h"
#include "s12.h"
#include "s13.h"
#include "s14.h"
#include "s15.h"


/*
PA22 RandomOut    PWM Out ?
PA02 SQROut       DAC0
PA05 SampleOut    DAC1

PA19 SQ1 OUT  PA18 FilterCap1
PA12 SQ2 OUT  PA17 FilterCap2
PB15 SQ3 OUT  PA16 FilterCap3
PB14 SQ4 OUT  PA15 FilterCap4
PB13 SQ5 OUT  PA14 FilterCap5
PB12 SQ6 OUT  PA13 FilterCap6

PB08 SQR Tune ADC
PB09 SQR Spread ADC
PB07 Ratchet ADC
PB03 ClockSpeed ADC
PB06 Sample TUNE ADC
PB05 Sample Change ADC 

PA20 Taster A/B 
PA21 Taster LPF 
PB17 Taster BIT 
PB16 Taster Trigger

PB01 TriggerInput SamplePlay
PB02 TriggerInput Clk IN
PB04 Clk Input gesteckt 

PB00 LED CLK 
PB31 LED SampleChange 

PA23 LED SMD Inline (BootLoader)
*/
Adafruit_ZeroTimer zt4 = Adafruit_ZeroTimer(4);

void TC4_Handler(){
  Adafruit_ZeroTimer::timerHandler(4);
}

/*
Adafruit_FlashTransport_QSPI flashTransport(PIN_QSPI_SCK, PIN_QSPI_CS, PIN_QSPI_IO0, PIN_QSPI_IO1, PIN_QSPI_IO2, PIN_QSPI_IO3);
Adafruit_SPIFlash flash(&flashTransport);

void dump_sector(uint32_t sector) {
  uint8_t buf[512];
  flash.readBuffer(sector*512, buf, 512);
  for(uint32_t row=0; row<32; row++)  {
    if ( row == 0 ) Serial.print("0");
    if ( row < 16 ) Serial.print("0");
    Serial.print(row*16, HEX);
    Serial.print(" : ");
    for(uint32_t col=0; col<16; col++)    {
      uint8_t val = buf[row*16 + col];
      if ( val < 16 ) Serial.print("0");
      Serial.print(val, HEX);
      Serial.print(" ");
    }
    Serial.println();
  }
}
*/




SAMD51_ADC adc51;
// SAMPLE RATE = 48MHz TC / 4 TC_CLOCK_PRESCALER_DIV4 / setPeriodMatch=125  = 96Khz 
const float samplerate = 48000000.0 / 4.0 / 125.0;
const float reciprocal_sr = 1.0 / samplerate;




float pitches[1024];
uint8_t sines[256];

void setup() {
  //put your setup code here, to run once:

  // First try remove DC from Ouputs  
  dacInit();
  DAC->DATA[0].reg = 2048;   // ca 1.5V
  DAC->DATA[1].reg = 2048;
  while (DAC->SYNCBUSY.bit.DATA0);

  // gen sin Table
  for(uint16_t i = 0 ; i < 256 ; i ++){
    float phase = (float)i / 256.0f * PI * 2.0f;
    sines[i] = (sin(phase) + 1.0) * 127.0f; 
  }

  // gen Table
  for(uint16_t i = 0 ; i < 1024 ; i ++){
    float clk_tempo_f = (float)i / 128.0f - 4.0f;
    pitches[i] = pow(12,clk_tempo_f);
  }

  Serial.begin(115200);
  // while ( !Serial ) delay(10);   // wait for native usb
  Serial.println("Himalia");

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


  adc51.createADCMap();

  pinMode(PA23,OUTPUT); // LED
  pinMode(PB00,OUTPUT); // LED
  pinMode(PB31,OUTPUT); // LED

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
  

  // create Time for AudioSamples
  zt4.configure(TC_CLOCK_PRESCALER_DIV4, // prescaler
                TC_COUNTER_SIZE_8BIT,   // bit width of timer/counter
                TC_WAVE_GENERATION_MATCH_PWM  // match style
                );

  zt4.setPeriodMatch(125, 250, 0); // 1 match, channel 0

  

  zt4.setCallback(true, TC_CALLBACK_CC_CHANNEL0, renderAudio);  // set DAC in the callback
  zt4.enable(true);
}

//
//  NoiseGenerator
//
class LFSR {
  private:
  uint16_t reg;
  public:
  LFSR(uint16_t seed) : reg(seed) {}
  uint16_t next() {
    uint8_t b = ((reg >> 0) ^ (reg >> 1) ^ (reg >> 3) ^ (reg >> 12)) & 1;
    reg = (reg >> 1) | (b << 15);
    return reg;
  }
};
LFSR lsfr1(0xA1e);


// 8-Bit OSC Stuff
#define OSC8BIT_PRG_COUNT 16
typedef unsigned char (*zm8BitPrg) (uint16_t t);
// https://git.metanohi.name/bytebeat.git/raw/c2f559b6efac4b03a0233e5797437af30601c170/clive.c
static unsigned char render_prg0(uint16_t t){ return  t; }
static unsigned char render_prg1(uint16_t t){ return  sines[(t>>8)]; }
static unsigned char render_prg2(uint16_t t){ return  t*(42&t>>10); }     // viznut
static unsigned char render_prg3(uint16_t t){ return  (t*5&t>>7)|(t*3&t>>10); }

static unsigned char render_prg4(uint16_t t){ return  t|t%255|t%257; } 
static unsigned char render_prg5(uint16_t t){ return  t>>6&1?t>>5:-t>>4; }
static unsigned char render_prg6(uint16_t t){ return  t*(t>>9|t>>13)&16; }
static unsigned char render_prg7(uint16_t t){ return  t*(((t>>9)^((t>>9)-1)^1)%13); }

static unsigned char render_prg8(uint16_t t){  return  t*(t>>8*((t>>15)|(t>>8))&(20|(t>>19)*5>>t|(t>>3))); }
static unsigned char render_prg9(uint16_t t){  return  t*(t>>((t>>9)|(t>>8))&(63&(t>>4))); }
static unsigned char render_prg10(uint16_t t){ return (t>>6|t|t>>(t>>16))*10+((t>>11)&7); }
static unsigned char render_prg11(uint16_t t){ return (t|(t>>9|t>>7))*t&(t>>11|t>>9); }

static unsigned char render_prg12(uint16_t t){ return t*(((t>>12)|(t>>8))&(63&(t>>4))); }
static unsigned char render_prg13(uint16_t t){ return t*(t^t+(t>>15|1)^(t-1280^t)>>10); }
static unsigned char render_prg14(uint16_t t){ return (t&t%255)-(t*3&t>>13&t>>6); }
static unsigned char render_prg15(uint16_t t){ return (t+(t>>2)|(t>>5))+(t>>3)|((t>>13)|(t>>7)|(t>>11)); }

const zm8BitPrg prgList[OSC8BIT_PRG_COUNT] = {  render_prg0, render_prg1, render_prg2, render_prg3,
                                                render_prg4, render_prg5, render_prg6, render_prg7,
                                                render_prg8, render_prg9, render_prg10,render_prg11,
                                                render_prg12,render_prg13,render_prg14,render_prg15
                                                };




//
//  AUDIO RENDERER
//
volatile float thea[6]     = { 0.0  , 0.0    , 0.0    , 0.0    , 0.0    , 0.0   };
volatile float thea_inc[6] = { 0.001, 0.00101, 0.00102, 0.00105, 0.00107, 0.00109};
volatile float sq_TRS[6]   = { 0.0  ,0.0     ,0.0     ,0.0     ,0.0     ,0.0};
volatile float flt_TRS[6]   = { 0.0  ,0.0     ,0.0     ,0.0     ,0.0     ,0.0};
volatile float spreads[6]  = { 0.0001f, 0.0001002f, 0.0001003f,0.0001007f,0.0001011f,0.0001017f};

float thea_noise  = 0.0f;
volatile float inc_noise   = 0.001f;

volatile float inc_8bit   = 0.001f;

float thea_sample  = 0.0f;
volatile float inc_sample   = 0.001f;

int prg8=0;
uint16_t samplePrg;
bool is_8bitchipmode = false;

void renderAudio() {
//  PORT->Group[PORTA].OUTSET.reg = 1ul << 22;

  // Noise / S/H Output
  thea_noise+=inc_noise;
  if(thea_noise>1.0f){
    thea_noise-=2.0f;
    // if(!DAC->SYNCBUSY.bit.DATA0)    
    DAC->DATA[0].reg = lsfr1.next();
    static bool noise_led=false;
    noise_led=!noise_led;
    if(noise_led)  PORT->Group[PORTB].OUTCLR.reg = 1ul << 0; else   PORT->Group[PORTB].OUTSET.reg = 1ul << 0;     // LED
    if(noise_led)  PORT->Group[PORTA].OUTCLR.reg = 1ul << 22; else   PORT->Group[PORTA].OUTSET.reg = 1ul << 22;   // SQUARE OUT NOISE
  }


  // SuperSQUARE Ramps
  bool sqr_pins[6];
  bool flt_pins[6];
  for(int i = 0 ; i < 6 ; i++){
    thea[i]+=thea_inc[i];
    if(thea[i]>1.0f) thea[i]-=2.0f;
    sqr_pins[i] = (thea[i]>sq_TRS[i]); 
    flt_pins[i] = (thea[i]>flt_TRS[i]); 
  }
  // Pin assign signals
  if(sqr_pins[0])  PORT->Group[PORTA].OUTCLR.reg = 1ul << 19; else   PORT->Group[PORTA].OUTSET.reg = 1ul << 19;
  if(sqr_pins[1])  PORT->Group[PORTA].OUTCLR.reg = 1ul << 12; else   PORT->Group[PORTA].OUTSET.reg = 1ul << 12;
  if(sqr_pins[2])  PORT->Group[PORTB].OUTCLR.reg = 1ul << 15; else   PORT->Group[PORTB].OUTSET.reg = 1ul << 15;
  if(sqr_pins[3])  PORT->Group[PORTB].OUTCLR.reg = 1ul << 14; else   PORT->Group[PORTB].OUTSET.reg = 1ul << 14;
  if(sqr_pins[4])  PORT->Group[PORTB].OUTCLR.reg = 1ul << 13; else   PORT->Group[PORTB].OUTSET.reg = 1ul << 13;
  if(sqr_pins[5])  PORT->Group[PORTB].OUTCLR.reg = 1ul << 12; else   PORT->Group[PORTB].OUTSET.reg = 1ul << 12;

  if(flt_pins[0])  PORT->Group[PORTA].DIRSET.reg = 1ul << 13; else   PORT->Group[PORTA].DIRCLR.reg = 1ul << 13;
  if(flt_pins[1])  PORT->Group[PORTA].DIRSET.reg = 1ul << 14; else   PORT->Group[PORTA].DIRCLR.reg = 1ul << 14;
  if(flt_pins[2])  PORT->Group[PORTA].DIRSET.reg = 1ul << 15; else   PORT->Group[PORTA].DIRCLR.reg = 1ul << 15;
  if(flt_pins[3])  PORT->Group[PORTA].DIRSET.reg = 1ul << 16; else   PORT->Group[PORTA].DIRCLR.reg = 1ul << 16;
  if(flt_pins[4])  PORT->Group[PORTA].DIRSET.reg = 1ul << 17; else   PORT->Group[PORTA].DIRCLR.reg = 1ul << 17;
  if(flt_pins[5])  PORT->Group[PORTA].DIRSET.reg = 1ul << 18; else   PORT->Group[PORTA].DIRCLR.reg = 1ul << 18;





  if(is_8bitchipmode){
    // 8 Bit OSC
    static float t=0;
    zm8BitPrg  callBackPrg = prgList[prg8];
    t+=inc_8bit * 1024.0f;
    if(t>65535.0f) t=0.0f;
    // if(!DAC->SYNCBUSY.bit.DATA1)
    DAC->DATA[1].reg = callBackPrg((uint16_t)t) << 4;
  }else{
    // Sample Ouput
    // sox /PRJ/test1.aif  --bits 16 --encoding unsigned-integer --endian little -c 1 t1.raw
    // xxd -i t1.raw > /PRJ/IOCore/HimaliaSketch/t1.h 
    // sed -i -r 's/unsigned/const unsigned/g' /PRJ/IOCore/HimaliaSketch/t1.h 
    const float sample_mul[16] = {  (float)s0_raw_len / 2.0f , (float)s1_raw_len / 2.0f , (float)s2_raw_len / 2.0f , (float)s3_raw_len / 2.0f,
                                    (float)s4_raw_len / 2.0f , (float)s5_raw_len / 2.0f , (float)s6_raw_len / 2.0f , (float)s7_raw_len / 2.0f,
                                    (float)s8_raw_len / 2.0f , (float)s9_raw_len / 2.0f , (float)s10_raw_len / 2.0f, (float)s11_raw_len / 2.0f,
                                    (float)s12_raw_len / 2.0f, (float)s13_raw_len / 2.0f, (float)s14_raw_len / 2.0f, (float)s15_raw_len / 2.0f  } ; // 2 bytes  as one sample -> safe as float
    
    const uint16_t * samples [16] = { (uint16_t*)&s0_raw,  (uint16_t*)&s1_raw, (uint16_t*)&s2_raw,  (uint16_t*)&s3_raw,
                                      (uint16_t*)&s4_raw,  (uint16_t*)&s5_raw, (uint16_t*)&s6_raw,  (uint16_t*)&s7_raw,
                                      (uint16_t*)&s8_raw,  (uint16_t*)&s9_raw, (uint16_t*)&s10_raw, (uint16_t*)&s11_raw,
                                      (uint16_t*)&s12_raw, (uint16_t*)&s13_raw,(uint16_t*)&s14_raw, (uint16_t*)&s15_raw };
   

    thea_sample+=inc_sample;
    if(thea_sample>1.0f){
      thea_sample=0.0f;
    }
    uint16_t sample_h = samples[samplePrg][(uint32_t)(sample_mul[samplePrg] * thea_sample)];  // extend to 32 Bit
    DAC->DATA[1].reg = (uint16_t)sample_h >> 4;
  }




  // PORT->Group[PORTA].OUTCLR.reg = 1ul << 22;
}





void loop() {
  // S/H Speed
  uint16_t clk_noise = adc51.readAnalog(PB03,ADC_Channel15,false);   // analogRead(PB03);            // read pitch

  // Serial.println(clk_noise,DEC);
  // delay(100);


  // float clk_noise_f = pitches[clk_noise & 0x03ff];  // Limit 4096 array size
  inc_noise = adc51.adcToInc[clk_noise & 0x0fff ];
  // inc_noise = 0.01f * clk_noise_f;
  // if(inc_noise>2.0f) inc_noise=1.9f;
  // if(inc_noise<0.00001f) inc_noise=0.00001f;

  // SAMPLE Speed
  uint16_t clk_sample = adc51.readAnalog(PB06,ADC_Channel8,true);  
  float clk_sample_f = pitches[(clk_sample >> 2) & 0x03ff];  // Limit 1024 array size
  inc_sample = clk_sample_f * 0.00001f;
  if(inc_sample>1.0f) inc_sample=1.0f;
  if(inc_sample<0.000001f) inc_sample=0.000001f;

  // 8Bit ChipMusic Speed
  inc_8bit = 0.0001f * clk_sample_f;

  // SQR Speed
  uint16_t clk_tempo = adc51.readAnalog(PB08,ADC_Channel2,false);  // analogRead(PB08);            // read pitch
  float inc_sqr = adc51.adcToInc[clk_tempo & 0x0fff ];
  thea_inc[0]=  spreads[0] * inc_sqr;
  thea_inc[1]=  spreads[1] * inc_sqr;
  thea_inc[2]=  spreads[2] * inc_sqr;
  thea_inc[3]=  spreads[3] * inc_sqr;
  thea_inc[4]=  spreads[4] * inc_sqr;
  thea_inc[5]=  spreads[5] * inc_sqr;




  // SQR Programm
  // 0x01fc...0x02f4  dec: 508...756
  uint16_t spread_adc = adc51.readAnalog(PB09,ADC_Channel3,false);
  uint16_t spread = ((spread_adc >> 2)  - 512 ) >> 4 ;
  // Serial.println(spread_adc,HEX);
 
  if(spread > 30 ) spread = 0;
  if(spread > 15 ) spread = 15;
  // Mute as PRG 0 !!! -> easy to sequence
  // 4 Bit Noise as PRG 15 ????

  switch(spread){
    case 0: // all Off
      flt_TRS[0]= -3.0f;      flt_TRS[1]= 3.0f;        flt_TRS[2]= -3.0f;       flt_TRS[3]= 3.0f;       flt_TRS[4]= -3.0f;        flt_TRS[5]= 3.0f;
      sq_TRS[0]= -3.0f;      sq_TRS[1]= 3.0f;        sq_TRS[2]= -3.0f;       sq_TRS[3]= 3.0f;       sq_TRS[4]= -3.0f;        sq_TRS[5]= 3.0f;
      spreads[0]= 1.001f;   spreads[1]= 1.001f;    spreads[2]= 1.0001f;    spreads[3]= 1.0001f;   spreads[4]= 1.0001f;     spreads[5]= 1.0001f;
      thea[1] = thea[0]; // sync phases
      break;    
    case 1:
      flt_TRS[0]= -3.0f;      flt_TRS[1]= 3.0f;        flt_TRS[2]= -3.0f;       flt_TRS[3]= 3.0f;       flt_TRS[4]= -3.0f;        flt_TRS[5]= 3.0f;
      sq_TRS[0]= 0.2f;      sq_TRS[1]= -0.3f;        sq_TRS[2]= -3.0f;       sq_TRS[3]= 3.0f;       sq_TRS[4]= -3.0f;        sq_TRS[5]= 3.0f;
      spreads[0]= 1.001f;   spreads[1]= 1.001f;    spreads[2]= 1.0001f;    spreads[3]= 1.0001f;   spreads[4]= 1.0001f;     spreads[5]= 1.0001f;
                            thea[1] = thea[0]; // sync phases
      break;
    case 2:
      flt_TRS[0]= 0.2f;      flt_TRS[1]= 0.8f;        flt_TRS[2]= 0.9f;       flt_TRS[3]= 0.4f;       flt_TRS[4]= -3.0f;        flt_TRS[5]= 3.0f;
      sq_TRS[0]= 0.2f;       sq_TRS[1]= -0.2f;        sq_TRS[2]= 0.4f;        sq_TRS[3]= -0.4f;        sq_TRS[4]= 0.5f;       sq_TRS[5]= 0.6f;
      spreads[0]= 1.0000f;   spreads[1]= 1.0000f;    spreads[2]= 1.0000f;    spreads[3]= 1.0000f;    spreads[4]= 1.0000f;    spreads[5]= 1.0000f;
                             thea[1] = thea[0];     thea[2] = thea[0];      thea[3] = thea[0];       thea[4] = thea[0];      thea[5] = thea[0]; 
      break;
    case 3:
      flt_TRS[0]= -3.0f;      flt_TRS[1]= 3.0f;        flt_TRS[2]= -3.0f;       flt_TRS[3]= 3.0f;       flt_TRS[4]= -3.0f;        flt_TRS[5]= 3.0f;
      sq_TRS[0]= 0.0f;       sq_TRS[1]= 0.0f;        sq_TRS[2]= 0.0f;        sq_TRS[3]= 0.0f;        sq_TRS[4]= -3.0f;       sq_TRS[5]= 3.0f;
      spreads[0]= 1.001f;   spreads[1]= 1.0015f;   spreads[2]= 1.002f;       spreads[3]= 1.003f;    spreads[4]= 1.004f;    spreads[5]= 1.005f;
      break;
    case 4:
      flt_TRS[0]= -3.0f;      flt_TRS[1]= 3.0f;        flt_TRS[2]= -3.0f;       flt_TRS[3]= 3.0f;       flt_TRS[4]= -3.0f;        flt_TRS[5]= 3.0f;
      sq_TRS[0]= 0.0f;       sq_TRS[1]= 0.0f;        sq_TRS[2]= 0.0f;        sq_TRS[3]= 0.0f;        sq_TRS[4]= 0.0f;        sq_TRS[5]= 0.0f;
      spreads[0]= 1.0f;      spreads[1]= 1.0f;       spreads[2]= 0.416666f;  spreads[3]= 0.416666f;  spreads[4]= 0.583333f;    spreads[5]= 0.583333f;
      break;
    case 5:
      flt_TRS[0]= -3.0f;      flt_TRS[1]= 3.0f;        flt_TRS[2]= -3.0f;       flt_TRS[3]= 3.0f;       flt_TRS[4]= -3.0f;        flt_TRS[5]= 3.0f;
      sq_TRS[0]= 0.0f;       sq_TRS[1]= 0.0f;        sq_TRS[2]= 0.0f;        sq_TRS[3]= 0.0f;        sq_TRS[4]= 0.0f;        sq_TRS[5]= 0.0f;
      spreads[0]= 1.0001f;   spreads[1]= 1.0001007f; spreads[2]= 1.0001013f; spreads[3]= 1.0001025f; spreads[4]= 1.0001047f; spreads[5]= 1.0001093f;
      break;
    case 6:
      flt_TRS[0]= -3.0f;      flt_TRS[1]= 3.0f;        flt_TRS[2]= -3.0f;       flt_TRS[3]= 3.0f;       flt_TRS[4]= -3.0f;        flt_TRS[5]= 3.0f;
      sq_TRS[0]= 0.0f;       sq_TRS[1]= 0.0f;        sq_TRS[2]= 0.0f;        sq_TRS[3]= 0.0f;        sq_TRS[4]= 0.0f;        sq_TRS[5]= 0.0f;
      spreads[0]= 1.0001f;   spreads[1]= 1.0001013f; spreads[2]= 1.0001023f; spreads[3]= 1.0001043f; spreads[4]= 1.0001072f; spreads[5]= 1.0001151f;
      break;      
    case 7:
      flt_TRS[0]= -3.0f;      flt_TRS[1]= 3.0f;        flt_TRS[2]= -3.0f;       flt_TRS[3]= 3.0f;       flt_TRS[4]= -3.0f;        flt_TRS[5]= 3.0f;
      sq_TRS[0]= 0.0f;       sq_TRS[1]= 0.0f;        sq_TRS[2]= 0.0f;        sq_TRS[3]= 0.0f;        sq_TRS[4]= 0.9f;        sq_TRS[5]= 0.1f;
      spreads[0]= 1.001f;    spreads[1]= 1.002f;     spreads[2]= 1.001f * 0.66666f;    spreads[3]= 1.002f*0.66666f; spreads[4]= 1.004f; spreads[5]= 1.008f;
      break;  
    case 8:
      flt_TRS[0]= -3.0f;      flt_TRS[1]= 3.0f;        flt_TRS[2]= -3.0f;       flt_TRS[3]= 3.0f;       flt_TRS[4]= -3.0f;        flt_TRS[5]= 3.0f;
      sq_TRS[0]= 0.0f;       sq_TRS[1]= 0.0f;        sq_TRS[2]= 0.0f;        sq_TRS[3]= 0.0f;        sq_TRS[4]= 0.9f;        sq_TRS[5]= 0.1f;
      spreads[0]= 1.001f;    spreads[1]= 1.002f;     spreads[2]= 1.001f * 0.66666f;    spreads[3]= 1.002f*0.66666f; spreads[4]= 1.004f * 0.66666f * 2.0f; spreads[5]= 1.008f  * 0.66666f * 2.0f;
      break;  
    case 9:
      flt_TRS[0]= -3.0f;      flt_TRS[1]= 3.0f;        flt_TRS[2]= -3.0f;       flt_TRS[3]= 3.0f;       flt_TRS[4]= -3.0f;        flt_TRS[5]= 3.0f;
      sq_TRS[0]= 0.2f;       sq_TRS[1]= -0.2f;        sq_TRS[2]= 0.8f;        sq_TRS[3]= 0.2f;        sq_TRS[4]= 0.9f;        sq_TRS[5]= 0.1f;
      spreads[0]= 1.001f;    spreads[1]= 1.002f;     spreads[2]= 1.001f * 0.66666f;    spreads[3]= 1.002f*0.66666f; spreads[4]= 1.004f * 0.66666f * 2.0f; spreads[5]= 1.008f  * 0.66666f * 2.0f;
      break;  
    case 10:
      flt_TRS[0]= -3.0f;      flt_TRS[1]= 3.0f;        flt_TRS[2]= -3.0f;       flt_TRS[3]= 3.0f;       flt_TRS[4]= -3.0f;        flt_TRS[5]= 3.0f;
      sq_TRS[0]= 0.0f;       sq_TRS[1]= 0.0f;        sq_TRS[2]= 0.0f;        sq_TRS[3]= 0.0f;        sq_TRS[4]= 0.9f;        sq_TRS[5]= 0.1f;
      spreads[0]= 1.001f;    spreads[1]= 1.002f;     spreads[2]= 1.001f * 0.66666f;    spreads[3]= 1.002f*0.66666f; spreads[4]= 1.004f; spreads[5]= 1.008f;
      break;  
    case 11:
      flt_TRS[0]= -3.0f;      flt_TRS[1]= 3.0f;        flt_TRS[2]= -3.0f;       flt_TRS[3]= 3.0f;       flt_TRS[4]= -3.0f;        flt_TRS[5]= 3.0f;
      sq_TRS[0]= 0.0f;       sq_TRS[1]= 0.0f;        sq_TRS[2]= 0.0f;        sq_TRS[3]= 0.0f;        sq_TRS[4]= 0.9f;        sq_TRS[5]= 0.1f;
      spreads[0]= 1.001f;    spreads[1]= 1.002f;     spreads[2]= 1.001f * 0.66666f;    spreads[3]= 1.002f*0.66666f; spreads[4]= 1.004f; spreads[5]= 1.008f;
      break;  
    case 12:
      flt_TRS[0]= -3.0f;      flt_TRS[1]= 3.0f;        flt_TRS[2]= -3.0f;       flt_TRS[3]= 3.0f;       flt_TRS[4]= -3.0f;        flt_TRS[5]= 3.0f;
      sq_TRS[0]= 0.0f;       sq_TRS[1]= 0.0f;        sq_TRS[2]= 0.0f;        sq_TRS[3]= 0.0f;        sq_TRS[4]= 0.9f;        sq_TRS[5]= 0.1f;
      spreads[0]= 1.001f;    spreads[1]= 1.002f;     spreads[2]= 1.001f * 0.66666f;    spreads[3]= 1.002f*0.66666f; spreads[4]= 1.004f; spreads[5]= 1.008f;
      break;  
    case 13:
      flt_TRS[0]= -3.0f;      flt_TRS[1]= 3.0f;        flt_TRS[2]= -3.0f;       flt_TRS[3]= 3.0f;       flt_TRS[4]= -3.0f;        flt_TRS[5]= 3.0f;
      sq_TRS[0]= 0.0f;       sq_TRS[1]= 0.0f;        sq_TRS[2]= 0.0f;        sq_TRS[3]= 0.0f;        sq_TRS[4]= 0.9f;        sq_TRS[5]= 0.1f;
      spreads[0]= 1.001f;    spreads[1]= 1.002f;     spreads[2]= 1.001f * 0.66666f;    spreads[3]= 1.002f*0.66666f; spreads[4]= 1.004f; spreads[5]= 1.008f;
      break;  
    case 14:
      flt_TRS[0]= -3.0f;      flt_TRS[1]= 3.0f;        flt_TRS[2]= -3.0f;       flt_TRS[3]= 3.0f;       flt_TRS[4]= -3.0f;        flt_TRS[5]= 3.0f;
      sq_TRS[0]= 0.9f;       sq_TRS[1]= 0.1f;        sq_TRS[2]= 0.9f;        sq_TRS[3]= 0.1f;        sq_TRS[4]= 0.9f;        sq_TRS[5]= 0.1f;
      spreads[0]= 1.001f;    spreads[1]= 1.002f;     spreads[2]= 1.001f * 0.66666f;    spreads[3]= 1.002f*0.66666f; spreads[4]= 1.004f; spreads[5]= 1.008f;
      break;  
    case 15:
      flt_TRS[0]= -3.0f;      flt_TRS[1]= 3.0f;        flt_TRS[2]= -3.0f;       flt_TRS[3]= 3.0f;       flt_TRS[4]= -3.0f;        flt_TRS[5]= 3.0f;
      sq_TRS[0]= 0.0f;       sq_TRS[1]= 0.0f;        sq_TRS[2]= 0.0f;        sq_TRS[3]= 0.0f;        sq_TRS[4]= 0.9f;        sq_TRS[5]= 0.1f;
      spreads[0]= 1.001f;    spreads[1]= 1.002f;     spreads[2]= 1.001f * 0.66666f;    spreads[3]= 1.002f*0.66666f; spreads[4]= 1.004f; spreads[5]= 1.008f;
      break;  
  }




  is_8bitchipmode = digitalRead(PB17);
  uint16_t prg8_smpl_select_adc = adc51.readAnalog(PB05,ADC_Channel7,true);
  int16_t prg8_smpl_select = (prg8_smpl_select_adc - 2048 )  >> 5 ;
  if(prg8_smpl_select > 30 ) prg8_smpl_select = 0;
  if(prg8_smpl_select > 15 ) prg8_smpl_select = 15;
  if(prg8_smpl_select < 0 )  prg8_smpl_select = 0;
  // Serial.println(prg8_smpl_select,HEX); delay(100);
  prg8 = prg8_smpl_select;
  samplePrg=prg8_smpl_select; 








  // TODO: listen Serial or Midi for WaveUpload


  // float clk_tempo_f = (float)clk_tempo / 128.0f - 4.0f;
  // clk_tempo_f = clk_tempo_f * clk_tempo_f;
  // Compute a Lookup Table
  // http://teropa.info/blog/2016/08/10/frequency-and-pitch.html
  // clk_tempo_f = pow(12,clk_tempo_f);


  //delayMicroseconds(1);

  // LED from CLK Input (SampleTrigger)


  // digitalWrite(PB31,digitalRead(PB01));
  // faster way forward digital Pin input to output
  if ( (PORT->Group[PORTB].IN.reg & (1ul << 1)) != 0 ){
    PORT->Group[PORTB].OUTSET.reg = 1ul << 31;
  }else{
    PORT->Group[PORTB].OUTCLR.reg = 1ul << 31;
  }

/*
  // LPF Button
  if(!digitalRead(PA21)){
    // https://www.avrfreaks.net/forum/samd21-how-enable-pullups-inputs
    // PORT->Group[PORTA].DIRSET.reg = 1ul << 19; // SET OUTPUT
    // PORT->Group[PORTA].DIRCLR.reg = 1ul << 19; // SET INPUT
    // PORT->Group[port].PINCFG[pin].reg |= PORT_PINCFG_PULLEN); for later setup pull resistors

    // pinMode(PA13,INPUT);     pinMode(PA14,INPUT); 
    // pinMode(PA15,OUTPUT);    pinMode(PA16,OUTPUT); 
    // pinMode(PA17,OUTPUT);    pinMode(PA18,OUTPUT); 

    PORT->Group[PORTA].DIRSET.reg = 1ul << 13;
    PORT->Group[PORTA].DIRSET.reg = 1ul << 14;
    PORT->Group[PORTA].DIRSET.reg = 1ul << 15;
    PORT->Group[PORTA].DIRSET.reg = 1ul << 16;
    PORT->Group[PORTA].DIRSET.reg = 1ul << 17;
    PORT->Group[PORTA].DIRSET.reg = 1ul << 18;

  }else{  // Filter Off
    PORT->Group[PORTA].DIRCLR.reg = 1ul << 13;
    PORT->Group[PORTA].DIRCLR.reg = 1ul << 14;
    PORT->Group[PORTA].DIRCLR.reg = 1ul << 15;
    PORT->Group[PORTA].DIRCLR.reg = 1ul << 16;
    PORT->Group[PORTA].DIRCLR.reg = 1ul << 17;
    PORT->Group[PORTA].DIRCLR.reg = 1ul << 18;

    //  pinMode(PA13,INPUT);     pinMode(PA14,INPUT); 
   // pinMode(PA15,INPUT);     pinMode(PA16,INPUT); 
   // pinMode(PA17,INPUT);     pinMode(PA18,INPUT); 
   
  }
*/
  // LED2
  // digitalWrite(PB00,false);
  // digitalWrite(PB00,true);

}
