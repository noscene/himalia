/*
  Copyright (c) 2014-2015 Arduino LLC.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef _VARIANT_DOPPLER_M4_
#define _VARIANT_DOPPLER_M4_

// The definitions here needs a SAMD core >=1.6.10
#define ARDUINO_SAMD_VARIANT_COMPLIANCE 10610

/*----------------------------------------------------------------------------
 *        Definitions
 *----------------------------------------------------------------------------*/

/** Frequency of the board main oscillator */
#define VARIANT_MAINOSC		(32768ul)

/** Master clock frequency */
#define VARIANT_MCK			  (120000000ul)

#define VARIANT_GCLK0_FREQ (120000000UL)
#define VARIANT_GCLK1_FREQ (48000000UL)
#define VARIANT_GCLK2_FREQ (100000000UL)

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "WVariant.h"

#ifdef __cplusplus
#include "SERCOM.h"
#include "Uart.h"
#endif // __cplusplus

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*----------------------------------------------------------------------------
 *        Pins
 *----------------------------------------------------------------------------*/

// Number of pins defined in PinDescription array
#define PINS_COUNT           (37u)
#define NUM_DIGITAL_PINS     (27u)
#define NUM_ANALOG_INPUTS    (10u)
#define NUM_ANALOG_OUTPUTS   (1u)
#define analogInputToDigitalPin(p)  ((p < NUM_ANALOG_INPUTS) ? (p) + PIN_A0 : -1)

#define digitalPinToPort(P)        ( &(PORT->Group[g_APinDescription[P].ulPort]) )
#define digitalPinToBitMask(P)     ( 1 << g_APinDescription[P].ulPin )
//#define analogInPinToBit(P)        ( )
#define portOutputRegister(port)   ( &(port->OUT.reg) )
#define portInputRegister(port)    ( &(port->IN.reg) )
#define portModeRegister(port)     ( &(port->DIR.reg) )
#define digitalPinHasPWM(P)        ( g_APinDescription[P].ulPWMChannel != NOT_ON_PWM || g_APinDescription[P].ulTCChannel != NOT_ON_TIMER )

/*
 * digitalPinToTimer(..) is AVR-specific and is not defined for SAMD
 * architecture. If you need to check if a pin supports PWM you must
 * use digitalPinHasPWM(..).
 *
 * https://github.com/arduino/Arduino/issues/1833
 */
// #define digitalPinToTimer(P)

#define PA15 4

#define PA12 1
    
#define PA19 38
#define PB30 39
#define PB31 40

// ADC Ports
#define PB00 41
#define PB01 42
#define PB08 43
#define PB09 44

#define PC00 45
#define PC01 46
#define PC02 47
#define PC03 48

#define PC20 49
#define PC21 50

#define PB16 51
#define PB17 52

#define PB12 53
#define PB13 54
#define PB14 55
#define PB15 56
    
#define PC10 57
#define PC11 58

#define PC12 59
#define PC13 60
#define PC14 61
#define PC15 62

#define PB04 63
#define PB05 64

#define PB18 65
#define PB19 66

#define PC16 67
#define PC17 68
#define PC18 69
#define PC19 70
#define PC06 71



#define PA22 9
#define PA23 33
#define PA06 15
#define PA07 16
#define PB02 23
#define PB03 21
#define PA14 3
#define PA10 19

// CS42448
#define CS42448_RESET PB15
#define CS42448_ADDR  0x48

    
// LEDs
#define PIN_LED_20          (20u)
#define PIN_LED              PIN_LED_20
#define LED_BUILTIN          PIN_LED_20

// FPGA internal SPI Bus
#define ICE_CLK     21     // PB03
#define ICE_MOSI    22     // PB23
#define ICE_MISO    23     // PB02
#define ICE_CS      24     // PB22
#define ICE_CDONE   25     // PB06
#define ICE_CRESET  26     // PB05
    
    
/*
 * Analog pins
 */
#define PIN_A0               (10ul)
#define PIN_A1               (PIN_A0 + 1)
#define PIN_A2               (PIN_A0 + 2)
#define PIN_A3               (PIN_A0 + 3)
#define PIN_A4               (PIN_A0 + 4)
#define PIN_A5               (PIN_A0 + 5)
#define PIN_A6               (PIN_A0 + 6)
#define PIN_A7               (PIN_A0 + 7)
#define PIN_A8               (PIN_A0 + 8)
#define PIN_A9               (PIN_A0 + 9)
#define PIN_A10              (PIN_A0 + 10)
    
#define PIN_DAC0             34
#define PIN_DAC1             35

static const uint8_t A0  = PIN_A0;
static const uint8_t A1  = PIN_A1;
static const uint8_t A2  = PIN_A2;
static const uint8_t A3  = PIN_A3;
static const uint8_t A4  = PIN_A4;
static const uint8_t A5  = PIN_A5;
static const uint8_t A6  = PIN_A6 ;
static const uint8_t A7  = PIN_A7 ;
static const uint8_t A8  = PIN_A8 ;
static const uint8_t A9  = PIN_A9 ;
static const uint8_t A10 = PIN_A10 ;

static const uint8_t DAC0 = PIN_DAC0;
static const uint8_t DAC1 = PIN_DAC1;

#define ADC_RESOLUTION		12

// Other pins
#define PIN_ATN              (26ul)
static const uint8_t ATN = PIN_ATN;

/*
 * Serial interfaces
 */

// Serial1
#define PIN_SERIAL1_RX       (0ul)
#define PIN_SERIAL1_TX       (1ul)
#define PAD_SERIAL1_RX       (SERCOM_RX_PAD_1)
#define PAD_SERIAL1_TX       (UART_TX_PAD_0)

/*
 * SPI Interfaces
 */
#define SPI_INTERFACES_COUNT     1

#define PIN_SPI_CS           (5u)                   // PB10 sercom 4.2 ALT
#define PIN_SPI_MISO         (2u)                   // PB11 sercom 4.3 ALT
#define PIN_SPI_SCK          (12u)                  // PB09 sercom 4.1 ALT
#define PIN_SPI_MOSI         (11u)                  // PB08 sercom 4.0 ALT
#define PERIPH_SPI           sercom4
#define PAD_SPI_TX           SPI_PAD_0_SCK_1
#define PAD_SPI_RX           SERCOM_RX_PAD_3        // alt

static const uint8_t SS	  = PIN_A2 ;
static const uint8_t MOSI = PIN_SPI_MOSI ;
static const uint8_t MISO = PIN_SPI_MISO ;
static const uint8_t SCK  = PIN_SPI_SCK ;


    
 /*
 * Wire Interfaces
 */
#define WIRE_INTERFACES_COUNT 1

#define PIN_WIRE_SDA         (1u) // PA12
#define PIN_WIRE_SCL         (0u) // PA13
#define PERIPH_WIRE          sercom2
#define WIRE_IT_HANDLER      SERCOM2_Handler

static const uint8_t SDA = PIN_WIRE_SDA;
static const uint8_t SCL = PIN_WIRE_SCL;

/*
 * USB
 */
#define PIN_USB_HOST_ENABLE (27ul)
#define PIN_USB_DM          (28ul)
#define PIN_USB_DP          (29ul)

/*
 * I2S Interfaces
 */
#define I2S_INTERFACES_COUNT 0

#define I2S_DEVICE          0
// no I2S on G19!

//QSPI Pins
#define PIN_QSPI_SCK	(5u)
#define PIN_QSPI_CS	    (2u)
#define PIN_QSPI_IO0	(17u)
#define PIN_QSPI_IO1	(18u)
#define PIN_QSPI_IO2	(19u)
#define PIN_QSPI_IO3	(20u)

/*
//PCC Pins
#define PIN_PCC_DEN1    (PIN_SPI_MOSI)
#define PIN_PCC_DEN2    (PIN_SPI_SCK)
#define PIN_PCC_CLK     (PIN_SPI_MISO)
#define PIN_PCC_D0      (13u)
#define PIN_PCC_D1      (12u)
#define PIN_PCC_D2      (10u)
#define PIN_PCC_D3      (11u)
#define PIN_PCC_D4      (9u)
#define PIN_PCC_D5      (8u)
#define PIN_PCC_D6      (1u)
#define PIN_PCC_D7      (0u)
#define PIN_PCC_D8      (5u)
#define PIN_PCC_D9      (6u)
*/
    
//TODO: meaningful value for this
#define VARIANT_QSPI_BAUD_DEFAULT 5000000

    
    
void dacInit();
void dacWrite(uint16_t  left ,uint16_t  right);
    
#ifdef __cplusplus
}
#endif

/*----------------------------------------------------------------------------
 *        Arduino objects - C++ only
 *----------------------------------------------------------------------------*/

#ifdef __cplusplus

/*	=========================
 *	===== SERCOM DEFINITION
 *	=========================
*/
extern SERCOM sercom0;
extern SERCOM sercom1;
extern SERCOM sercom2;
extern SERCOM sercom3;
extern SERCOM sercom4;
extern SERCOM sercom5;

extern Uart Serial1;

#endif

// These serial port names are intended to allow libraries and architecture-neutral
// sketches to automatically default to the correct port name for a particular type
// of use.  For example, a GPS module would normally connect to SERIAL_PORT_HARDWARE_OPEN,
// the first hardware serial port whose RX/TX pins are not dedicated to another use.
//
// SERIAL_PORT_MONITOR        Port which normally prints to the Arduino Serial Monitor
//
// SERIAL_PORT_USBVIRTUAL     Port which is USB virtual serial
//
// SERIAL_PORT_LINUXBRIDGE    Port which connects to a Linux system via Bridge library
//
// SERIAL_PORT_HARDWARE       Hardware serial port, physical RX & TX pins.
//
// SERIAL_PORT_HARDWARE_OPEN  Hardware serial ports which are open for use.  Their RX & TX
//                            pins are NOT connected to anything by default.
#define SERIAL_PORT_USBVIRTUAL      Serial
#define SERIAL_PORT_MONITOR         Serial
// Serial has no physical pins broken out, so it's not listed as HARDWARE port
#define SERIAL_PORT_HARDWARE        Serial1
#define SERIAL_PORT_HARDWARE_OPEN   Serial1











#endif /* _VARIANT_DOPPLER_M4_ */
