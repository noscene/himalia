
#include <Wire.h>
#include "wiring_private.h" // pinPeripheral() function
#include "Adafruit_ZeroTimer.h"
#include "SdFat.h"
#include "Adafruit_SPIFlash.h"

//#include "snare_verb01.h"
//#include "techno_06.h"
#include "aif16_2.h"
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


float pitches[1024];



void setup() {
  //put your setup code here, to run once:
  Serial.begin(115200);
  // while ( !Serial ) delay(10);   // wait for native usb
  Serial.println("Himalia");

  flash.begin();
  
  Serial.println("Adafruit Serial Flash Info example");
  Serial.print("JEDEC ID: "); Serial.println(flash.getJEDECID(), HEX);
  Serial.print("Flash size: "); Serial.println(flash.size());

  pinMode(PA13,OUTPUT); digitalWrite(PA13,false);
  pinMode(PA14,OUTPUT); digitalWrite(PA14,false);
  pinMode(PA15,OUTPUT); digitalWrite(PA15,false);
  pinMode(PA16,OUTPUT); digitalWrite(PA16,false);
  pinMode(PA17,OUTPUT); digitalWrite(PA17,false);
  pinMode(PA18,OUTPUT); digitalWrite(PA18,false);




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
  


  // gen Table
  for(uint16_t i = 0 ; i < 1024 ; i ++){
    float clk_tempo_f = (float)i / 128.0f - 4.0f;
    pitches[i] = pow(12,clk_tempo_f);
  }




  dacInit();
  DAC->DATA[0].reg = 2048;   // ca 1.5V
  while (DAC->SYNCBUSY.bit.DATA0);

  // create Time for AudioSamples
  zt4.configure(TC_CLOCK_PRESCALER_DIV8, // prescaler
                TC_COUNTER_SIZE_8BIT,   // bit width of timer/counter
                TC_WAVE_GENERATION_MATCH_PWM  // match style
                );

  zt4.setPeriodMatch(150, 150, 0); // 1 match, channel 0
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



//
//  AUDIO RENDERER
//
float thea[6]     = { 0.0  , 0.0    , 0.0    , 0.0    , 0.0    , 0.0   };
float thea_inc[6] = { 0.001, 0.00101, 0.00102, 0.00105, 0.00107, 0.00109};
float sq_TRS[6]   = { 0.0  ,0.0     ,0.0     ,0.0     ,0.0     ,0.0};
float spreads[6]  = { 0.0001f, 0.0001002f, 0.0001003f,0.0001007f,0.0001011f,0.0001017f};
float thea_noise  = 0.0f;
float inc_noise  = 0.001f;

void renderAudio() {
  PORT->Group[PORTA].OUTSET.reg = 1ul << 22;
  // SuperSQUARE Ramps
  for(int i = 0 ; i < 6 ; i++){
    thea[i]+=thea_inc[i];
    if(thea[i]>1.0f) thea[i]-=2.0f;
  }
  if(thea[0]>sq_TRS[0])  PORT->Group[PORTA].OUTCLR.reg = 1ul << 19; else   PORT->Group[PORTA].OUTSET.reg = 1ul << 19;
  if(thea[1]>sq_TRS[1])  PORT->Group[PORTA].OUTCLR.reg = 1ul << 12; else   PORT->Group[PORTA].OUTSET.reg = 1ul << 12;
  if(thea[2]>sq_TRS[2])  PORT->Group[PORTB].OUTCLR.reg = 1ul << 15; else   PORT->Group[PORTB].OUTSET.reg = 1ul << 15;
  if(thea[3]>sq_TRS[3])  PORT->Group[PORTB].OUTCLR.reg = 1ul << 14; else   PORT->Group[PORTB].OUTSET.reg = 1ul << 14;
  if(thea[4]>sq_TRS[4])  PORT->Group[PORTB].OUTCLR.reg = 1ul << 13; else   PORT->Group[PORTB].OUTSET.reg = 1ul << 13;
  if(thea[5]>sq_TRS[5])  PORT->Group[PORTB].OUTCLR.reg = 1ul << 12; else   PORT->Group[PORTB].OUTSET.reg = 1ul << 12;

  thea_noise+=inc_noise;
  if(thea_noise>1.0f){
    thea_noise-=2.0f;
    DAC->DATA[0].reg = lsfr1.next();
  }


  // Noise
  // if(!DAC->SYNCBUSY.bit.DATA0)
  // if(!DAC->SYNCBUSY.bit.DATA1)
  // sox https://dsp.stackexchange.com/questions/41536/convert-16-bit-wav-file-to-12-bit-raw-audio-file


  // Byte Order is wrong!!! need insert a byte in front of header!!!

  const  uint32_t sample_len = _Users_svenbraun_Downloads_Inst_1_bip_1_aif_len / 2;
  static uint32_t sample_idx=0;
  int sample_h = ((short int*)&_Users_svenbraun_Downloads_Inst_1_bip_1_aif)[sample_idx];
  sample_h+=0x7fff;
  // uint8_t sample_l = (uint8_t)_Users_svenbraun_Downloads_Inst_1_bip_aif[sample_idx+1];
  sample_idx+=2;
  if(sample_idx>= sample_len) sample_idx=0;

  DAC->DATA[1].reg = (uint16_t)sample_h >> 4;   // 16 to 12 Bit!!!!
  // DAC->DATA[1].reg = (thea[1] + 1.0f) * 16000.0f;   // 0V

  PORT->Group[PORTA].OUTCLR.reg = 1ul << 22;
}



void loop() {


  uint16_t clk_noise = analogRead(PB03);            // read pitch
  float clk_noise_f = pitches[clk_noise & 0x03ff];  // Limit 1024 array size
  inc_noise = 0.01f * clk_noise_f;
  

  uint16_t clk_tempo = analogRead(PB08);            // read pitch
  float clk_tempo_f = pitches[clk_tempo & 0x03ff];  // Limit 1024 array size




  thea_inc[0]=  spreads[0] * clk_tempo_f;
  thea_inc[1]=  spreads[1] * clk_tempo_f;
  thea_inc[2]=  spreads[2] * clk_tempo_f;
  thea_inc[3]=  spreads[3] * clk_tempo_f;
  thea_inc[4]=  spreads[4] * clk_tempo_f;
  thea_inc[5]=  spreads[5] * clk_tempo_f;

  // 0x01fc...0x02f4  dec: 508...756
  uint16_t spread = (analogRead(PB09) - 512 ) >> 4 ;
  // Serial.println(spread,HEX);
  if(spread > 30 ) spread = 0;
  if(spread > 15 ) spread = 15;
  switch(spread){
    case 0:
      sq_TRS[0]= 0.0f;      sq_TRS[1]= 0.0f;  sq_TRS[2]= -3.0f;  sq_TRS[3]= 3.0f;  sq_TRS[4]= -3.0f;  sq_TRS[5]= 3.0f;
      spreads[0]= 0.0001f;  spreads[1]= 0.0001f; spreads[2]= 0.0001f; spreads[3]= 0.0001f; spreads[4]= 0.0001f; spreads[5]= 0.0001f;
      thea[1] = thea[0]; // sync phases
      break;
    case 1:
      sq_TRS[0]= 0.0f;      sq_TRS[1]= 0.0f;  sq_TRS[2]= 0.0f;  sq_TRS[3]= 0.0f;  sq_TRS[4]= -3.0f;  sq_TRS[5]= 3.0f;
      spreads[0]= 0.0001f;  spreads[1]= 0.0001f; spreads[2]= 0.0001f; spreads[3]= 0.0001f; spreads[4]= 0.0001f; spreads[5]= 0.0001f;
      thea[1] = thea[0];    thea[2] = thea[0];   thea[3] = thea[0]; 
      break;
    case 2:
      sq_TRS[0]= 0.0f;      sq_TRS[1]= 0.0f;  sq_TRS[2]= 0.0f;  sq_TRS[3]= 0.0f;  sq_TRS[4]= -3.0f;  sq_TRS[5]= 3.0f;
      spreads[0]= 0.0001f;  spreads[1]= 0.0001002f; spreads[2]= 0.0001005f; spreads[3]= 0.0001009f; spreads[4]= 0.0001013f; spreads[5]= 0.0001019f;
      break;
    case 3:
      sq_TRS[0]= 0.0f;      sq_TRS[1]= 0.0f;  sq_TRS[2]= 0.0f;  sq_TRS[3]= 0.0f;  sq_TRS[4]= 0.0f;  sq_TRS[5]= 0.0f;
      spreads[0]= 0.0001f;  spreads[1]= 0.0001002f; spreads[2]= 0.0001005f; spreads[3]= 0.0001009f; spreads[4]= 0.0001013f; spreads[5]= 0.0001019f;
      break;
    case 4:
      sq_TRS[0]= 0.0f;      sq_TRS[1]= 0.0f;  sq_TRS[2]= 0.0f;  sq_TRS[3]= 0.0f;  sq_TRS[4]= 0.0f;  sq_TRS[5]= 0.0f;
      spreads[0]= 0.0001f;  spreads[1]= 0.0001007f; spreads[2]= 0.0001013f; spreads[3]= 0.0001025f; spreads[4]= 0.0001047f; spreads[5]= 0.0001093f;
      break;
    case 5:
      sq_TRS[0]= 0.0f;      sq_TRS[1]= 0.0f;  sq_TRS[2]= 0.0f;  sq_TRS[3]= 0.0f;  sq_TRS[4]= 0.0f;  sq_TRS[5]= 0.0f;
      spreads[0]= 0.0001f;  spreads[1]= 0.0001013f; spreads[2]= 0.0001023f; spreads[3]= 0.0001043f; spreads[4]= 0.0001072f; spreads[5]= 0.0001151f;
      break;      
    case 6:
      sq_TRS[0]= 0.0f;      sq_TRS[1]= 0.0f;  sq_TRS[2]= 0.0f;  sq_TRS[3]= 0.0f;  sq_TRS[4]= 0.0f;  sq_TRS[5]= 0.0f;
      spreads[0]= 0.0001f;  spreads[1]= 0.0001025f; spreads[2]= 0.0001051f; spreads[3]= 0.0001103f; spreads[4]= 0.0001201f; spreads[5]= 0.0001405f;
      break;  

  }


  // TODO: listen Serial or Midi for WaveUpload


  // float clk_tempo_f = (float)clk_tempo / 128.0f - 4.0f;
  // clk_tempo_f = clk_tempo_f * clk_tempo_f;
  // Compute a Lookup Table
  // http://teropa.info/blog/2016/08/10/frequency-and-pitch.html
  // clk_tempo_f = pow(12,clk_tempo_f);



  

  //delayMicroseconds(1);

  // LED from CLK Input
  digitalWrite(PB31,digitalRead(PB01));

  // LPF Button
  if(!digitalRead(PA21)){
    PORT->Group[PORTA].DIRSET.reg = 1ul << 19;
    pinMode(PA13,OUTPUT);    pinMode(PA14,OUTPUT); 
    pinMode(PA15,OUTPUT);    pinMode(PA16,OUTPUT); 
    pinMode(PA17,OUTPUT);    pinMode(PA18,OUTPUT); 
  }else{
    pinMode(PA13,INPUT);     pinMode(PA14,INPUT); 
    pinMode(PA15,INPUT);     pinMode(PA16,INPUT); 
    pinMode(PA17,INPUT);     pinMode(PA18,INPUT); 
  }

  // LED2
  // digitalWrite(PB00,false);
  // digitalWrite(PB00,true);

}
