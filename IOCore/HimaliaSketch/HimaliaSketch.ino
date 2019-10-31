
#include <Wire.h>
#include "wiring_private.h" // pinPeripheral() function

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

void setup() {
  //put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Himalia");

  pinMode(PA23,OUTPUT); // LED
  pinMode(PB00,OUTPUT); // LED
  pinMode(PB31,OUTPUT); // LED

  pinMode(PA19,OUTPUT); // SQR1
  pinMode(PA12,OUTPUT); // SQR2
  pinMode(PB15,OUTPUT); // SQR1
  pinMode(PB14,OUTPUT); // SQR1
  pinMode(PB13,OUTPUT); // SQR1
  pinMode(PB12,OUTPUT); // SQR1





  dacInit();
  DAC->DATA[0].reg = 2048;   // ca 1.5V
  while (DAC->SYNCBUSY.bit.DATA0);


}



float thea[8]     = { 0.0  , 0.0    , 0.0    , 0.0    , 0.0    , 0.0    , 0.0    , 0.0 };
float thea_inc[8] = { 0.001, 0.00101, 0.00102, 0.00103, 0.00104, 0.00105, 0.00106, 0.00106};

void loop() {
  // put your main code here, to run repeatedly:

  // SuperSQUARE Ramps
  for(int i = 0 ; i < 8 ; i++){
    thea[i]+=thea_inc[i];
    if(thea[i]>1.0) thea[i]-=2.0;
  }

  // LED from CLK Input
  digitalWrite(PB31,digitalRead(PB02));

  // LED2
  digitalWrite(PB00,false);
  digitalWrite(PB00,true);

  // int adc09 = analogRead(14);
  // int adc10 = analogRead(15);

  // Write SAW to DAC
  DAC->DATA[0].reg = thea[1]*4096;   // 0V
  while (DAC->SYNCBUSY.bit.DATA0);

}
