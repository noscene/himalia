
#ifndef SAMD51_ADC_H
#define SAMD51_ADC_H

#include "wiring_private.h" // pinPeripheral() function

//
//  SAMD ADC Registers
//
// on Himalia it produce with ADC_REFCTRL_REFSEL_INTVCC1_Val
// 2040 .... 3080 with potis
// 2000 .... 4095 with external CV 
class SAMD51_ADC {
  private:
  uint16_t reg;
  public:

  float adcToInc[4096]; // public array for lookups
  bool last_adc = true;

  SAMD51_ADC() {
    // analogReference(AR_EXTERNAL);
    // analogReadResolution(12);
    ADC0->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTVCC1_Val;  // // 1/1.48 VDDANA = 1/1.48* 3V3 = 2.2297
    while(ADC0->SYNCBUSY.reg & ADC_SYNCBUSY_CTRLB);
    
    ADC0->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_12BIT_Val;
    while(ADC0->SYNCBUSY.reg & ADC_SYNCBUSY_CTRLB);

    // add clk stuff from https://github.com/noscene/ArduinoCore-samd/blob/master/cores/arduino/wiring.c
    GCLK->PCHCTRL[ADC1_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK1_Val | (1 << GCLK_PCHCTRL_CHEN_Pos); //use clock generator 1 (48Mhz)
    ADC1->CTRLA.bit.PRESCALER = ADC_CTRLA_PRESCALER_DIV64_Val;
    ADC1->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_12BIT_Val;
    while( ADC1->SYNCBUSY.reg & ADC_SYNCBUSY_CTRLB );  //wait for sync

    ADC0->CTRLA.bit.PRESCALER = ADC_CTRLA_PRESCALER_DIV64_Val;
    ADC0->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_12BIT_Val;
    while( ADC0->SYNCBUSY.reg & ADC_SYNCBUSY_CTRLB );  //wait for sync

    ADC0->SAMPCTRL.reg = 8;                        // sampling Time Length
    while( ADC0->SYNCBUSY.reg & ADC_SYNCBUSY_SAMPCTRL );  //wait for sync

    ADC1->SAMPCTRL.reg = 8;                        // sampling Time Length
    while( ADC1->SYNCBUSY.reg & ADC_SYNCBUSY_SAMPCTRL );  //wait for sync

    ADC1->INPUTCTRL.reg = ADC_INPUTCTRL_MUXNEG_GND;   // No Negative input (Internal Ground)
    while( ADC1->SYNCBUSY.reg & ADC_SYNCBUSY_INPUTCTRL );  //wait for sync
    // Averaging (see datasheet table in AVGCTRL register description)
    ADC1->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_1 |    // 1 sample only (no oversampling nor averaging)
                ADC_AVGCTRL_ADJRES(0x0ul);   // Adjusting result by 0
    while( ADC1->SYNCBUSY.reg & ADC_SYNCBUSY_AVGCTRL );  //wait for sync

    
    ADC1->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTVCC1_Val;
    while(ADC1->SYNCBUSY.reg & ADC_SYNCBUSY_CTRLB);

    ADC1->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_12BIT_Val;
    // while(ADC1->SYNCBUSY.reg & ADC_SYNCBUSY_CTRLB);     // HÄNGT!!!!!
  };



  uint16_t readAnalog(uint16_t pin, uint16_t channel , bool adc_alt){
    pinPeripheral(pin, PIO_ANALOG);
    if(adc_alt){
      while( ADC1->SYNCBUSY.reg & ADC_SYNCBUSY_INPUTCTRL ); //wait for sync
      ADC1->INPUTCTRL.bit.MUXPOS = channel; // Selection for the positive ADC input
      while( ADC1->SYNCBUSY.reg & ADC_SYNCBUSY_ENABLE ); //wait for sync
      ADC1->CTRLA.bit.ENABLE = 0x01;             // Enable ADC    
      while( ADC1->SYNCBUSY.reg & ADC_SYNCBUSY_ENABLE ); //wait for sync
      ADC1->SWTRIG.bit.START = 1;
      while (ADC1->INTFLAG.bit.RESRDY == 0);   // Waiting for conversion to complete
      return ADC1->RESULT.reg;
    }else{
      while( ADC0->SYNCBUSY.reg & ADC_SYNCBUSY_INPUTCTRL ); //wait for sync
      ADC0->INPUTCTRL.bit.MUXPOS = channel; // Selection for the positive ADC input
      while( ADC0->SYNCBUSY.reg & ADC_SYNCBUSY_ENABLE ); //wait for sync
      ADC0->CTRLA.bit.ENABLE = 0x01;             // Enable ADC    
      while( ADC0->SYNCBUSY.reg & ADC_SYNCBUSY_ENABLE ); //wait for sync
      ADC0->SWTRIG.bit.START = 1;
      while (ADC0->INTFLAG.bit.RESRDY == 0);   // Waiting for conversion to complete
      return ADC0->RESULT.reg;
    }
  };

  uint16_t readLastValue(){
    if(last_adc){
      //while (ADC1->INTFLAG.bit.RESRDY == 0);   // Waiting for conversion to complete
      return ADC1->RESULT.reg;

    }else{
      //while (ADC0->INTFLAG.bit.RESRDY == 0);   // Waiting for conversion to complete
      return ADC0->RESULT.reg;
    }
  }

  void startReadAnalog(uint16_t pin, uint16_t channel , bool adc_alt){
    pinPeripheral(pin, PIO_ANALOG);
    last_adc = adc_alt;
    if(adc_alt){
      while( ADC1->SYNCBUSY.reg & ADC_SYNCBUSY_INPUTCTRL ); //wait for sync
      ADC1->INPUTCTRL.bit.MUXPOS = channel; // Selection for the positive ADC input
      while( ADC1->SYNCBUSY.reg & ADC_SYNCBUSY_ENABLE ); //wait for sync
      ADC1->CTRLA.bit.ENABLE = 0x01;             // Enable ADC    
      while( ADC1->SYNCBUSY.reg & ADC_SYNCBUSY_ENABLE ); //wait for sync
      ADC1->SWTRIG.bit.START = 1;
    }else{
      while( ADC0->SYNCBUSY.reg & ADC_SYNCBUSY_INPUTCTRL ); //wait for sync
      ADC0->INPUTCTRL.bit.MUXPOS = channel; // Selection for the positive ADC input
      while( ADC0->SYNCBUSY.reg & ADC_SYNCBUSY_ENABLE ); //wait for sync
      ADC0->CTRLA.bit.ENABLE = 0x01;             // Enable ADC    
      while( ADC0->SYNCBUSY.reg & ADC_SYNCBUSY_ENABLE ); //wait for sync
      ADC0->SWTRIG.bit.START = 1;
    }
  };
  // see how to compute: https://electronics.stackexchange.com/questions/278050/making-voltmeter-accepting-bipolar-input-voltage-using-a-microcontroller
  float voltageDivider(float vIn){
    const float r1 =  22000.0f;  // 20k resitor  input resistor
    const float r2 = 150000.0f;  // 6k8 resitor  to vref
    const float r3 =  10000.0f;  // 10k resitor  to gnd
    const float vref = 3.3f;    // input ref from array
    const float dby = 1.0f / ( 1.0f / r1 + 1.0f / r2 + 1.0f / r3 );
    const float utx = vref / r2;
    return ( vIn / r1 + utx) *  dby;
  };

  // scale 12 bit adc hardware
  uint16_t ADCValueByVolt(float v){
    const float adc_ref = 3.3f;
    const float bitdeep = 4096.0f;
    return  (v / adc_ref * bitdeep);
  };

  void createADCMap(float samplingrate = 166666.0f ) {
    //const float samplingrate = 166666.0f;
    const float volt_per_octave = 1.0f;     // mode for with range
    for(float vin = -15.0f ; vin < 15.0f ; vin+=0.001f){      // lets brute force, but fast enough
      uint16_t adc_v = ADCValueByVolt(voltageDivider(vin));   // 
      if(adc_v < 4096){
        // A4 = 440Hz = 2.75V as a reference point,
        float frq =  110.0f / pow(2.0f, 2.75) * pow(2.0f, vin * volt_per_octave );  // <--- tuning stuff
        float thea_inc = 1.0f / samplingrate * frq;
        if(thea_inc>1.0f) thea_inc=1.0f;            // Limit nyquist
        adcToInc[adc_v]=thea_inc;                   // compute table ADC value to phase increment for ramp osc
        /** /
        Serial.print(vin,DEC);
        Serial.print(" -> ");
        Serial.print(adc_v);
        Serial.print(" -> ");
        Serial.print(thea_inc);
        Serial.print(" -> ");
        Serial.println(frq);
        /**/
      }
    }
  };


};


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

#endif