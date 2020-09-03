/*
 * 
 * DOPPLER-Board-Layout: 
 *                                                                                    ---------------- FPGA Pins ------------------
 *                                                                                   LedR LedG LedB
 * DIL Pin 48   47   46   45   44   43   42   41   40   39   38   37   36   35   34   33   32   31   30   29   28   27   26   25
 *       |--O----O----O----O----O----O----O----O----O----O----O----O----O----O----O----O----O----O----O----O----O----O----O----O---|
 * name  | VIN  5V  3.3V PA11 PA10 PA09 PA08 PA07 PA06 PA05 PA04 PB09 PB08 PA02  GND  F41  F40  F39  F38  F37  F36  F35  F34  F32  |
 *       |                                                                                            ö  ö  ö  ö                   |
 *      |                                                                                             ö  ö  ö  ö         |BTN:S1|  |
 *     | USB                           DOPPLER: SamD51 <- SPI -> icE40        |BTN:RESET|             ö  ö  ö  ö                   |
 *      |                                                                                             ö  ö  ö  ö         |BTN:S2|  |
 *       |                                                                                                                         |
 * name  | GND PA13 PA12 PB11 PB14 PA15 PB10 PA31 PA30  RES PA19 PA20 PA21 PA22 3.3V  F11  F12  F13  F18  F19  F20  F21  F23  F25  |
 *       L--O----O----O----O----O----O----O----O----O----O----O----O----O----O----O----O----O----O----O----O----O----O----O----O---|
 * DIL Pin  1    2    3    4    5    6    7    8    9   10   11   12   13   14   15   16   17   18   19   20   21   22   23   24
 *             SCL  SDA                       SWD  SWC RES
 *             -- I2C--                       --- SWD  ---   ----- Shared  -----      ---------------- FPGA Pins ------------------
 */

// Some useful Links
// https://github.com/adafruit/Adafruit_DAP/blob/master/Adafruit_DAP.h
// https://github.com/adafruit/circuitpython/issues/894
// https://learn.adafruit.com/introducing-adafruit-itsybitsy-m4/pinouts

#include "Adafruit_DAP.h"


// theM Stuff
//#include "/Users/svenbraun/Documents/GitHub/uf2-samdx1/build_as_headers/theM_bootloader.h"
//#define binfile build_theM_bootloader_theM_v2_0_0_33_ge603fc3_bin

// doppler Stuff
//#include "/Users/svenbraun/Documents/GitHub/uf2-samdx1/build_as_headers/doppler_bootloader_mac.h"
//#define binfile build_doppler_bootloader_doppler_v2_0_0_adafruit_4_18_g29d300f_bin

//#include "/Users/svenbraun/Documents/GitHub/Adafruit_DAP/examples/DopplerBootLoaderByESP32/doppler.h"

#include "/Users/svenbraun/Documents/GitHub/uf2-samdx1/build/Himalia/himalia.h"

#define binfile bootloader_Himalia_v2_0_0_adafruit_4_22_g851f5b1_dirty_bin

// this Pins using with Node32S to flash Bootloader and Set Fuses
#define SWDIO 5
#define SWCLK 18
#define SWRST 19
#define LED_PIN 2

// 0x41004000
#define REG_NVMCTRL_CTRLB          (0x41004004)

#define BUFSIZE SAMx5_PAGE_SIZE       //don't change!

//create a DAP for programming Atmel SAMx5 devices
Adafruit_DAP_SAMx5 dap;


uint32_t rr;

// Function called when there's an SWD error
void error(const char *text) {
  Serial.println(text);
  while (1){
    digitalWrite(LED_PIN, HIGH);
    delay(700); 
    digitalWrite(LED_PIN, LOW);
    delay(50);  
   }
}


void setup() {

  delay(500);
  pinMode(LED_PIN, OUTPUT);
  
  Serial.begin(115200);
  while(!Serial) {
   digitalWrite(LED_PIN, HIGH);
    delay(100); 
    digitalWrite(LED_PIN, LOW);
    delay(200);  
  }

  dap.begin(SWCLK, SWDIO, SWRST, &error);
  
  Serial.print("Connecting...");  
  if (! dap.dap_disconnect())                      error(dap.error_message);
  
  char debuggername[100];
  if (! dap.dap_get_debugger_info(debuggername))   error(dap.error_message);
  Serial.print(debuggername); Serial.print("\n\r");
  
  if (! dap.dap_connect())                         error(dap.error_message);

 Serial.print("Connected.... \n\r");
  
  if (! dap.dap_transfer_configure(0, 128, 128))   error(dap.error_message);
 Serial.print("dap_transfer_configure.... \n\r");
  if (! dap.dap_swd_configure(0))                  error(dap.error_message);
 Serial.print("dap_swd_configure.... \n\r");

/*
  // Try to ColdConnect
  pinMode(SWRST,OUTPUT);
  pinMode(SWCLK,OUTPUT);
  digitalWrite(SWRST,LOW);
  digitalWrite(SWCLK,LOW);
  delay(1);
  digitalWrite(SWRST,HIGH);
  delay(100);
  */
  
  if (! dap.dap_reset_link())                      error(dap.error_message);
 Serial.print("dap_reset_link.... \n\r");



  

 
  if (! dap.dap_swj_clock(50))               error(dap.error_message);
 Serial.print("dap_swj_clock.... \n\r");
  dap.dap_target_prepare();
 Serial.print("dap_target_prepare.... \n\r");
  uint32_t dsu_did;
  if (! dap.select(&dsu_did)) {
    Serial.print("Unknown device found 0x"); Serial.print(dsu_did, HEX);
    error("Unknown device found");
  }




 Serial.print("Erasing... ");
  // The SAMD51s bootloader protection can be temporarily disabled through an NVM
  rr = dap.dap_read_word(REG_NVMCTRL_CTRLB);
  rr = (0xA5 << 8) | 0x1a;
  dap.dap_write_word(REG_NVMCTRL_CTRLB,rr); 
  
  dap.erase();
  Serial.println(" done.");



   Serial.print("transfer.... \n\r");

  
  for (device_t *device = dap.devices; device->dsu_did > 0; device++) {
    if (device->dsu_did == dsu_did) {
      Serial.print("Found Target: ");
      Serial.print(device->name);
      Serial.print("\tFlash size: ");
      Serial.print(device->flash_size);
      Serial.print("\tFlash pages: ");
      Serial.println(device->n_pages);
      //Serial.print("Page size: "); Serial.println(device->flash_size / device->n_pages);
    }
  }


  

  // Read CPU ID!!!

  Serial.print("\nCPU UniqueID (serial) 128bit : ");
  rr = dap.dap_read_word(0x008061FC);
  Serial.print(rr,HEX);
  rr = dap.dap_read_word(0x00806010);
  Serial.print(rr,HEX);
  rr = dap.dap_read_word(0x00806014);
  Serial.print(rr,HEX);
  rr = dap.dap_read_word(0x00806018);
  Serial.print(rr,HEX);
  Serial.print("\n");

/*
  // The SAMD51s bootloader protection can be temporarily disabled through an NVM
  rr = dap.dap_read_word(REG_NVMCTRL_CTRLB);
  rr = 0xA51a
  dap.dap_write_word(REG_NVMCTRL_CTRLB,rr); 
*/


  Serial.println("lets blink PA23 via SWD ...");
  // Blink SAMD51 LED on PA11
  #define REG_PORT_DIR0              (0x41008000)
  #define REG_PORT_OUT0              (0x41008010) 
  #define DOPPLER_LED                 23
  // Set PIN to Output
  rr = dap.dap_read_word(REG_PORT_DIR0); 
  rr |= (1<<DOPPLER_LED);
  dap.dap_write_word(REG_PORT_DIR0,rr); 

  // Blink 10 times
  for(int bl=0 ; bl < 10; bl ++){
    rr = dap.dap_read_word(REG_PORT_OUT0); 
    rr |= (1<<DOPPLER_LED);
    dap.dap_write_word(REG_PORT_OUT0,rr); 
    delay(100);
    rr &= ~(1<<DOPPLER_LED);
    dap.dap_write_word(REG_PORT_OUT0,rr); 
    delay(100);
    Serial.print(".");
  }


/*
CPU UniqueID (serial) : 81C75665337433139202020FF0E271F
..........Fuses...
 - BootloaderSize (NVM_BOOT): F
 - Bootloader Protection (NVM_LOCKS): FFFF
 - BOD33_Disable: 1
 - BOD33_Level: FF
 - BOD33_Action: 3
 - BOD33_Hysteresis: F
 - SEESBLK: F
 - SEEPSZ: 7
 - RAM_ECCDIS: 1
 - WDT_Enable: 1
 - WDT_Always_On: 1
 - WDT_Period: F
 - WDT_Window: F
 - WDT_EWOFFSET: F
 - WDT_WEN: 1


transfer.... 

Found Target: SAMD51G19A  Flash size: 524288  Flash pages: 1024

CPU UniqueID (serial) : 81C75665337433139202020FF0E271F
..........Fuses...
 - BootloaderSize (NVM_BOOT): B
 - Bootloader Protection (NVM_LOCKS): FFFF
 - BOD33_Disable: 1
 - BOD33_Level: 1C
 - BOD33_Action: 1
 - BOD33_Hysteresis: 2
 - SEESBLK: 0
 - SEEPSZ: 0
 - RAM_ECCDIS: 1
 - WDT_Enable: 0
 - WDT_Always_On: 0
 - WDT_Period: B
 - WDT_Window: B
 - WDT_EWOFFSET: B
 - WDT_WEN: 0
 done.
Erasing...  done.
Programming... 16384
16384
161Millis 
Done!


  rr = dap.dap_read_word(REG_NVMCTRL_CTRLB);
  rr = (0xA5 << 8) | 0x1a;
  dap.dap_write_word(REG_NVMCTRL_CTRLB,rr); 

*/

  // Example of how to read and set fuses
  Serial.println("Fuses...");
  dap.fuseRead(); //MUST READ FUSES BEFORE SETTING OR WRITING ANY
  //dap._USER_ROW.BOOTPROT = 0x07;

  uint32_t * um;

  um = (uint32_t*)&dap._USER_ROW;
  
  Serial.println(um[0],HEX);
  Serial.println(um[1],HEX);
  Serial.println(um[2],HEX);
  Serial.println(um[3],HEX);


//  um[0] = 0b001001000111001;
//  dap._USER_ROW.bit.NVM_BOOT = (dap._USER_ROW.bit.NVM_BOOT & 0xf0) | 0x2;

//   ItsiBitsyFuses to campare
//  um[0]=0xFE9A9239;
//  um[1]=0xAEECFF80;
//  um[2]=0xFFFFFFFF;
//  um[3]=0xFFFFFFFF;

/*
Bits 11:8 – BOOTPROT[3:0] Boot Loader Protection Size
This bitfield is loaded from the USER page during the device startup.
Defines the size of the BOOTPROT region which is protected against write or erase or Chip-Erase operations. This size is given by the following formula (15-BOOTPROT)*8KB.

Name: 
Offset: 
Reset: 
Property:  Read-Only
STATUS 0x12 0x0000
Bit 15 14 13 12 11 10 9 8
Access RRRR Reset 000x
Bit 7 6 5 4 3 2 1 0
Access RRRRRR Reset 000000
Bits 11:8 – BOOTPROT[3:0] Boot Loader Protection Size
This bitfield is loaded from the USER page during the device startup.
Defines the size of the BOOTPROT region which is protected against write or erase or Chip-Erase operations. This size is given by the following formula (15-BOOTPROT)*8KB.
Bit 5 – BPDIS Boot Loader Protection Disable 0: Boot loader protection is not discarded.
1: Boot loader protection against modify operations is discarded until CBPDIS is issued or next start-up sequence except for Chip-Erase.



*/

  um[0]=0xF69A9239;     // <- SET first byte to F6 -> xxBBBBxx 13 = 16KB Bootloader    
 //  um[0]=0xFE9A9239;     // <- SET first byte to F6 -> xxBBBBxx 15 = 0KB Bootloader    
   
//  um[1]=0xAEECFF80;
//  um[2]=0xFFFFFFFF;
//  um[3]=0xFFFFFFFF;
  
  Serial.println("edit fuses");
  
  Serial.println(um[0],HEX);
  Serial.println(um[1],HEX);
  Serial.println(um[2],HEX);
  Serial.println(um[3],HEX);

  // https://github.com/adafruit/Adafruit_DAP/blob/master/Adafruit_DAP.h
  Serial.print(" - BootloaderSize (NVM_BOOT): ");
  Serial.println(dap._USER_ROW.bit.NVM_BOOT,HEX);
  //dap._USER_ROW.bit.NVM_BOOT=0x0;     // DONT DO THAT!!! it will not set bits well
  
  Serial.print(" - Bootloader Protection (NVM_LOCKS): ");
  Serial.println(dap._USER_ROW.bit.NVM_LOCKS,HEX);
  //dap._USER_ROW.bit.NVM_LOCKS = 0xffff;   // DONT DO THAT!!! it will not set bits well

  Serial.print(" - Bootloader Protection (NVM_LOCKS): ");
  Serial.println(dap._USER_ROW.bit.User_Page,HEX);
  //dap._USER_ROW.bit.User_Page = 0xffffffff;   // DONT DO THAT!!! it will not set bits well

  
  Serial.print(" - BOD33_Disable: ");
  Serial.println(dap._USER_ROW.bit.BOD33_Disable,HEX);
  //dap._USER_ROW.bit.BOD33_Disable=0x1;    // DONT DO THAT!!! it will not set bits well

  Serial.print(" - BOD33_Level: ");
  Serial.println(dap._USER_ROW.bit.BOD33_Level,HEX);
  //dap._USER_ROW.bit.BOD33_Level=0x1c;   // DONT DO THAT!!! it will not set bits well
  
  Serial.print(" - BOD33_Action: ");
  Serial.println(dap._USER_ROW.bit.BOD33_Action,HEX);
  //dap._USER_ROW.bit.BOD33_Action=0x1;   // DONT DO THAT!!! it will not set bits well
  
  Serial.print(" - BOD33_Hysteresis: ");
  Serial.println(dap._USER_ROW.bit.BOD33_Hysteresis,HEX);
  //dap._USER_ROW.bit.BOD33_Hysteresis=0x2;   // DONT DO THAT!!! it will not set bits well

  Serial.print(" - SEESBLK: ");
  Serial.println(dap._USER_ROW.bit.SEESBLK,HEX);
  //dap._USER_ROW.bit.SEESBLK=0x0;      // DONT DO THAT!!! it will not set bits well

  Serial.print(" - SEEPSZ: ");
  Serial.println(dap._USER_ROW.bit.SEEPSZ,HEX);
  //dap._USER_ROW.bit.SEEPSZ=0x0;     // DONT DO THAT!!! it will not set bits well

  Serial.print(" - RAM_ECCDIS: ");
  Serial.println(dap._USER_ROW.bit.RAM_ECCDIS,HEX);

  Serial.print(" - WDT_Enable: ");
  Serial.println(dap._USER_ROW.bit.WDT_Enable,HEX);
  //dap._USER_ROW.bit.WDT_Enable=0x0;   // DONT DO THAT!!! it will not set bits well
  
  Serial.print(" - WDT_Always_On: ");
  Serial.println(dap._USER_ROW.bit.WDT_Always_On,HEX);
  //dap._USER_ROW.bit.WDT_Always_On=0x0;    // DONT DO THAT!!! it will not set bits well

  Serial.print(" - WDT_Period: ");
  Serial.println(dap._USER_ROW.bit.WDT_Period,HEX);
  //dap._USER_ROW.bit.WDT_Period=0xb;     // DONT DO THAT!!! it will not set bits well
  
  Serial.print(" - WDT_Window: ");
  Serial.println(dap._USER_ROW.bit.WDT_Window,HEX);
  //dap._USER_ROW.bit.WDT_Window=0xb;   // DONT DO THAT!!! it will not set bits well
  
  Serial.print(" - WDT_EWOFFSET: ");
  Serial.println(dap._USER_ROW.bit.WDT_EWOFFSET,HEX);
  //dap._USER_ROW.bit.WDT_EWOFFSET=0xb;   // DONT DO THAT!!! it will not set bits well
  
  Serial.print(" - WDT_WEN: ");
  Serial.println(dap._USER_ROW.bit.WDT_WEN,HEX);
  //dap._USER_ROW.bit.WDT_WEN=0x0;    // DONT DO THAT!!! it will not set bits well
  
  //BootloaderSize
  // dap._USER_ROW.bit.NVM_BOOT = 0x0B;  // ist default 0xf(32K oder 0K) und muss neu gesetzt werden
  // BootloaderProtection
  // dap._USER_ROW.bit.NVM_LOCKS = 0x00;
  // return;


  // Cancel all Write Stuff
  Serial.println(" Cancel set Fuse & Bootloader. Ensure what you are doing! ");
 // return;
  

  
  dap.fuseWrite();
  
  Serial.println(" done.");
 
  // The SAMD51s bootloader protection can be temporarily disabled through an NVM
  rr = dap.dap_read_word(REG_NVMCTRL_CTRLB);
  rr = (0xA5 << 8) | 0x1a;
  dap.dap_write_word(REG_NVMCTRL_CTRLB,rr); 
  delay(10);


  Serial.print("Erasing... ");
  dap.erase();
  Serial.println(" done.");
  
  Serial.print("Programming... ");

  Serial.print(sizeof(binfile));
  Serial.print("\n");
  
  unsigned long t = millis();
  uint32_t addr = dap.program_start();

  while(addr < sizeof(binfile)){
    dap.programBlock(addr, binfile + addr);
    addr += BUFSIZE;
  }
   Serial.println(addr);
  Serial.print(millis() - t);
  Serial.println("Millis \nDone!");
  dap.dap_set_clock(50);

  dap.deselect();
  dap.dap_disconnect();
}

void loop() {
  //blink led on the host to show we're done
  digitalWrite(LED_PIN, HIGH);
  delay(500); 
  digitalWrite(LED_PIN, LOW);
  delay(500);  
}
