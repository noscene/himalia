
#ifndef SAMD51_ADC_H
#define SAMD51_ADC_H

#include "wiring_private.h" // pinPeripheral() function

//
//  SAMD ADC Registers
//
class SAMD51_ADC {
  private:
  uint16_t reg;
  public:
  SAMD51_ADC() {
    // analogReference(AR_EXTERNAL);
    // analogReadResolution(12);
    ADC0->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTVCC1_Val;
    while(ADC0->SYNCBUSY.reg & ADC_SYNCBUSY_CTRLB);
    
    ADC0->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_12BIT_Val;
    while(ADC0->SYNCBUSY.reg & ADC_SYNCBUSY_CTRLB);

    // add clk stuff from https://github.com/noscene/ArduinoCore-samd/blob/master/cores/arduino/wiring.c
    GCLK->PCHCTRL[ADC1_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK1_Val | (1 << GCLK_PCHCTRL_CHEN_Pos); //use clock generator 1 (48Mhz)
    ADC1->CTRLA.bit.PRESCALER = ADC_CTRLA_PRESCALER_DIV32_Val;
    ADC1->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_12BIT_Val;
    
    while( ADC1->SYNCBUSY.reg & ADC_SYNCBUSY_CTRLB );  //wait for sync
    ADC1->SAMPCTRL.reg = 5;                        // sampling Time Length
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
};


#endif