#include <Arduino.h>
#include <SPI.h>
#include <wiring_private.h> // pinPeripheral() function

#ifndef _ICECLASS_DOPPLER_M4_
#define _ICECLASS_DOPPLER_M4_

// give subclass chance to modify the BITSTREAM
#ifndef BITSTREAM
#define BITSTREAM snd_fpga_bin
// #include "/Users/svenbraun/Documents/GitHub/Doppler_FPGA_Firmware/doppler_simple_io.h"
#include "/PRJ/snd_fpga/snd_fpga.h"
#endif



#define SPI_FPGA_SPEED 34000000
// #define SPI_FPGA_SPEED 2000000


class ICEClass {
    
public:
    
    SPIClass * SPIfpga;
    ICEClass(){};
    
    // see Board variant.h
    uint16_t  ice_cs    = ICE_CS;
    uint16_t  ice_mosi  = ICE_MOSI;
    uint16_t  ice_miso  = ICE_MISO;
    uint16_t  ice_clk   = ICE_CLK;
    
    void initSPI(){
        pinMode (ice_cs, OUTPUT);
        pinMode (ice_clk, OUTPUT);
        pinMode (ice_mosi, OUTPUT);
        pinMode (ice_miso, INPUT_PULLUP);
        SPIfpga = new SPIClass (&sercom5,  ice_miso,  ice_clk,  ice_mosi,  SPI_PAD_0_SCK_1,  SERCOM_RX_PAD_2);
        SPIfpga->begin();
        // remux Arduino Pins for Hardware SPI
        pinPeripheral(ice_miso,   PIO_SERCOM_ALT);
        pinPeripheral(ice_mosi,   PIO_SERCOM_ALT);
        pinPeripheral(ice_clk,    PIO_SERCOM_ALT);
    };
    
     
    uint16_t sendSPI16(uint16_t  data) {
        return sendSPI((data >> 8) & 0xff , data & 0xff );
    }
    
    // send 2 bytes to FPGA
    uint16_t sendSPI(uint8_t  adr , uint8_t  txdata) {
        digitalWrite(ice_cs, LOW);
        SPIfpga->beginTransaction(SPISettings(SPI_FPGA_SPEED, MSBFIRST, SPI_MODE0));
        uint8_t  rxdata1 = SPIfpga->transfer(adr);
        uint8_t  rxdata2 = SPIfpga->transfer(txdata);
        /* SPIfpga->transfer(adr);  // only for messure / debug
        SPIfpga->transfer(txdata);
        SPIfpga->transfer(adr);
        SPIfpga->transfer(txdata);
        SPIfpga->transfer(adr);
        SPIfpga->transfer(txdata);
        SPIfpga->transfer(adr);
        SPIfpga->transfer(txdata);
        SPIfpga->endTransaction(); */
        digitalWrite(ice_cs, HIGH);
        return rxdata1 << 8 | rxdata2 & 0xff ;
    };
    

    /*
    //
    // Here we start the BIT_STREAM Stuff in BitBang Mode
    // TODO: use also hardware spi
    //
    
    void iceClock() {
        digitalWrite(ice_clk, LOW);
        digitalWrite(ice_clk, HIGH);
    };
    */
   
    void reset_inout() {
        pinMode(ICE_CLK,     INPUT_PULLUP);
        pinMode(ICE_CDONE,   INPUT_PULLUP);
        pinMode(ICE_MOSI,    INPUT_PULLUP);
        pinMode(ICE_CRESET,  OUTPUT);
        //pinMode(ICE_CS,      OUTPUT);
        digitalWrite(ICE_CS, HIGH);
    };
    
    bool upload(){
        upload(BITSTREAM,sizeof(BITSTREAM));
    }
    bool upload(const unsigned char * bitstream , const unsigned int bitstream_size){
        // ensure via are in right mode
        pinPeripheral(ICE_MISO,   PIO_DIGITAL);
        pinPeripheral(ICE_MOSI,   PIO_DIGITAL);
        pinPeripheral(ICE_CLK,    PIO_DIGITAL);
        
        pinMode(ICE_CLK,     OUTPUT);
        pinMode(ICE_MOSI,    OUTPUT);
        pinMode(ICE_CRESET,  OUTPUT);
        pinMode(ICE_CS,      OUTPUT);
        
        // enable reset
        digitalWrite(ICE_CRESET, LOW);
        // start clock high
        digitalWrite(ICE_CLK, HIGH);
        
        // select SRAM programming mode
        digitalWrite(ICE_CS, LOW);
        delay(10);
        
        // release reset
        digitalWrite(ICE_CRESET, HIGH);
        delay(100);     // TODO: check time! for Waiting FPGA is self init!

        // Begin HardwareSPI
        initSPI();
        SPIfpga->beginTransaction(SPISettings(SPI_FPGA_SPEED, MSBFIRST, SPI_MODE0));
        SPIfpga->transfer(0x00);    // 8 clocks
        //const unsigned int size = sizeof(BITSTREAM);
        for (int k = 0; k < bitstream_size; k++) {
            SPIfpga->transfer(bitstream[k]);
        }
        SPIfpga->endTransaction();
        for(int i = 0 ; i < 8 ; i++){
            SPIfpga->transfer(0x00);
        }
        // End HardwareSPI

        bool cdone_high = digitalRead(ICE_CDONE) == HIGH;
        reset_inout();
        return cdone_high;
    }
    
    
};





#endif /* _ICECLASS_DOPPLER_M4_ */
