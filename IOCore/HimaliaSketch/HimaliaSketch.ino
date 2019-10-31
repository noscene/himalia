
#include <Wire.h>
#include "wiring_private.h" // pinPeripheral() function
#include "Adafruit_ZeroTimer.h"
#include "SdFat.h"
#include "Adafruit_SPIFlash.h"

/*


PA22 RandomOut    PWM Out ?
PA02 NoiseOut     DAC0
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


void setup() {
  //put your setup code here, to run once:
  Serial.begin(115200);
  // while ( !Serial ) delay(10);   // wait for native usb
  Serial.println("Himalia");

  flash.begin();
  
  Serial.println("Adafruit Serial Flash Info example");
  Serial.print("JEDEC ID: "); Serial.println(flash.getJEDECID(), HEX);
  Serial.print("Flash size: "); Serial.println(flash.size());



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


  dacInit();
  DAC->DATA[0].reg = 2048;   // ca 1.5V
  while (DAC->SYNCBUSY.bit.DATA0);

  // create Time for AudioSamples
  zt4.configure(TC_CLOCK_PRESCALER_DIV8, // prescaler
                TC_COUNTER_SIZE_8BIT,   // bit width of timer/counter
                TC_WAVE_GENERATION_MATCH_PWM  // match style
                );

  zt4.setPeriodMatch(150, 100, 0); // 1 match, channel 0
  zt4.setCallback(true, TC_CALLBACK_CC_CHANNEL0, renderAudio);  // set DAC in the callback
  zt4.enable(true);
}

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

float thea[6]     = { 0.0  , 0.0    , 0.0    , 0.0    , 0.0    , 0.0   };
float thea_inc[6] = { 0.001, 0.00101, 0.00102, 0.00105, 0.00107, 0.00109};
float sq_TRS[6]   = { 0.0  ,0.5     ,0.0     ,0.2     ,0.0     ,0.1};

void renderAudio() {
  PORT->Group[PORTA].OUTSET.reg = 1ul << 22;
  // SuperSQUARE Ramps
  for(int i = 0 ; i < 6 ; i++){
    thea[i]+=thea_inc[i];//  * clk_tempo_f;
    if(thea[i]>1.0) thea[i]-=2.0;
  }
  if(thea[0]>sq_TRS[0])  PORT->Group[PORTA].OUTCLR.reg = 1ul << 19; else   PORT->Group[PORTA].OUTSET.reg = 1ul << 19;
  if(thea[1]>sq_TRS[1])  PORT->Group[PORTA].OUTCLR.reg = 1ul << 12; else   PORT->Group[PORTA].OUTSET.reg = 1ul << 12;
  if(thea[2]>sq_TRS[2])  PORT->Group[PORTB].OUTCLR.reg = 1ul << 15; else   PORT->Group[PORTB].OUTSET.reg = 1ul << 15;
  if(thea[3]>sq_TRS[3])  PORT->Group[PORTB].OUTCLR.reg = 1ul << 14; else   PORT->Group[PORTB].OUTSET.reg = 1ul << 14;
  if(thea[4]>sq_TRS[4])  PORT->Group[PORTB].OUTCLR.reg = 1ul << 13; else   PORT->Group[PORTB].OUTSET.reg = 1ul << 13;
  if(thea[5]>sq_TRS[5])  PORT->Group[PORTB].OUTCLR.reg = 1ul << 12; else   PORT->Group[PORTB].OUTSET.reg = 1ul << 12;

  // Noise
  //  if(!DAC->SYNCBUSY.bit.DATA0)
  DAC->DATA[0].reg = lsfr1.next();

  //  if(!DAC->SYNCBUSY.bit.DATA1)
  DAC->DATA[1].reg = (thea[1] + 1.0) * 16000;   // 0V

  PORT->Group[PORTA].OUTCLR.reg = 1ul << 22;
}



void loop() {
  // put your main code here, to run repeatedly:
  /*
  Serial.print("Enter the sector number to dump: ");
  //while( !Serial.available() ) delay(10);
  int sector = Serial.parseInt();
  Serial.println(sector); // echo
  if ( sector < flash.size()/512 ) {
    dump_sector(sector);
  }else{
    Serial.println("Invalid sector number");
    dump_sector(sector);
  }
  Serial.println();
  delay(10); // a bit of delay

  Serial.println("erase chip.....");
  if (!flash.eraseChip()) {
    Serial.println("Failed to erase chip!");
  }

  flash.waitUntilReady();
  Serial.println("Successfully erased chip!");
*/



  int clk_tempo = analogRead(PB03);
  float clk_tempo_f = (float)clk_tempo / 500.0f;
  clk_tempo_f = clk_tempo_f * clk_tempo_f * clk_tempo_f;

  thea_inc[0]=  0.001f   * clk_tempo_f;
  thea_inc[1]=  0.00102f * clk_tempo_f;
  thea_inc[2]=  0.00103f * clk_tempo_f;
  thea_inc[3]=  0.00105f * clk_tempo_f;
  thea_inc[4]=  0.00107f * clk_tempo_f;
  thea_inc[5]=  0.00109f * clk_tempo_f;


  //delayMicroseconds(1);

  // LED from CLK Input
  digitalWrite(PB31,digitalRead(PB01));

  // LED2
  digitalWrite(PB00,false);
  digitalWrite(PB00,true);

  // int adc09 = analogRead(14);
  // int adc10 = analogRead(15);

  // Write SAW to DAC

  // while (DAC->SYNCBUSY.bit.DATA0);

}
