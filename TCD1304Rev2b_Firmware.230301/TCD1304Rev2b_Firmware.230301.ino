/*
  Author:   Mitchell C. Nelson
  Date:     March 1, 2024
  Contact:  drmcnelson@gmail.com

  TCD1304 rev 2 only.   This version removes support for ILX11 and earlier versions of the TCD1304
  
  This work is a derivative of the following works:
  
    RAWHIDfirmware.ino   Mitchell C. Nelson,  June 18, 2021

    TCD1304_201223.ino   Mitchell C. Nelson,  Aug 10 through Dec 23, 2020


  Permissions:

     Permission is hereby granted for non-commercial use.

     For commerical uses, please contact the author at the above email address.

  Provisos:

  1) No representation is offered or assumed of suitability for any purposee whatsoever.

  2) Use entirely at your own risk.

  3) Please cite appropriately in any work using this code.

 */

// ------------------------------------------------------------------
#define DIAGNOSTICS
//#define DIAGNOSTICS_CPU
//#define DIAGNOSTICS_IDLER
//#define DIAGNOSTICS_SYNC
//#define DIAGNOSTICS_GATE
// Stream& dataPort = SerialUSB1;
// ------------------------------------------------------------------

#include "Arduino.h"

//#include <digitalWriteFast.h>

#include <limits.h>

#include <ADC.h>
#include <ADC_util.h>

#include <EEPROM.h>

#include <LittleFS.h>

LittleFS_Program filesys;
bool filesys_ready = false;

extern float tempmonGetTemp(void);

// ADC setup
ADC *adc = new ADC();

// CPU Cycles per Usec
#define CYCLES_PER_USEC (F_CPU / 1000000)

// ------------------------------------------------------------------

#define thisMANUFACTURER_NAME {'D','R','M','C','N','E','L','S','O','N' }
#define thisMANUFACTURER_NAME_LEN 10

#define thisPRODUCT_NAME {'T','C','D','1','3','0','4','V','2','B'}
#define thisPRODUCT_NAME_LEN 10

#define thisPRODUCT_SERIAL_NUMBER { 'S','N','0','0','0','0','0','0','0','0','0','1' }
#define thisPRODUCT_SERIAL_NUMBER_LEN 12

extern "C"
{
  struct usb_string_descriptor_struct_manufacturer
  {
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint16_t wString[thisMANUFACTURER_NAME_LEN];
  };

  usb_string_descriptor_struct_manufacturer usb_string_manufacturer_name = {
    2 + thisMANUFACTURER_NAME_LEN * 2,
    3,
    thisMANUFACTURER_NAME
  };

  // -------------------------------------------------
  
  struct usb_string_descriptor_struct_product
  {
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint16_t wString[thisPRODUCT_NAME_LEN];
  };

  usb_string_descriptor_struct_product usb_string_product_name = {
    2 + thisPRODUCT_NAME_LEN * 2,
    3,
    thisPRODUCT_NAME
  };

  // -------------------------------------------------
  
  struct usb_string_descriptor_struct_serial_number
  {
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint16_t wString[thisPRODUCT_SERIAL_NUMBER_LEN];
  };

  usb_string_descriptor_struct_serial_number usb_string_serial_number =
    {
      2 + thisPRODUCT_SERIAL_NUMBER_LEN * 2, 
      3,
      thisPRODUCT_SERIAL_NUMBER
    };
}

// ------------------------------------------------------------------

const char versionstr[] = "T4LCD vers 0.3";

const char authorstr[] =  "(c) 2020-2024 by Mitchell C. Nelson, Ph.D. ";

const char sensorstr[] = "TCD1304";

// ------------------------------------------------------------------

const unsigned long fM  = 2000000;   // 2MHz Mclk frequency

const int syncPin      = 0;     // Goes low for trigger standoff (delay) period, use this for desktop N2 laser
const int busyPin      = 1;     // Gate output pin, goes high during shutter
const int interruptPin = 2;     // Trigger input pin
const int sparePin     = 3;     // Spare pin for digital output
const int fMPin        = 4;     // Mclk out 
const int fMPinMonitor = 5;     // Clock monitor
const int ICGPin       = 6;     // Integration clear gate
const int SHPin        = 7;     // Shift gate
const int analogPin    = A0;    // Analog input, Pin 14

// Fast nofrills CLOCK read
#define CLOCKREAD (CORE_PIN5_PINREG & CORE_PIN5_BITMASK)

// Fast nofrills BUSY pin set, lear, flip state
#define SETBUSYPIN (CORE_PIN1_PORTSET = CORE_PIN1_BITMASK)
#define CLEARBUSYPIN (CORE_PIN1_PORTCLEAR = CORE_PIN1_BITMASK)
#define TOGGLEBUSYPIN (CORE_PIN1_PORTTOGGLE = CORE_PIN1_BITMASK)

// Fast nofrills SYNC pin set/clear
#define SETSYNCPIN (CORE_PIN0_PORTSET = CORE_PIN0_BITMASK)
#define CLEARSYNCPIN (CORE_PIN0_PORTCLEAR = CORE_PIN0_BITMASK)
#define TOGGLESYNCPIN (CORE_PIN0_PORTTOGGLE = CORE_PIN0_BITMASK)

// Conversion to volts
#define NBITS 12
#define VFS (3.3/4.0)
#define VPERBIT (VFS/(1<<NBITS))

// Sensor data readout
#define NREADOUT 3694
#define DATASTART 12
#define DATASTOP 3680

// Size of the useful part
#define NPIXELS (DATASTOP-DATASTART)
#define NBYTES (NPIXELS*2)

// And first part of that is dark
#define NDARK 13

// Transfer time for uint16's over usb 
#define TRANSFERUSECS 7500

// Minimum inter shutter interval
#define SHUTTERMIN 5 

// Pin 13 for the LED
#define HASLED
const int led = 13;

void blink() {
  digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(100);               // wait for a second
  digitalWrite(led, LOW);    // turn the LED off by making the voltage LOW
}

/* =====================================================================================================
   Some pin functions and defaults
*/
int intedgemode = RISING;        // Default trigger mode

// busy pin
unsigned int busyPinState = LOW;

// sync pin
unsigned int syncPinState = HIGH;

// Spare pin can double as PWM
unsigned int sparePinState = LOW;
bool spare_pwm = false;

uint32_t pwm_resolution = 8;
uint32_t pwm_value = 4;
float pwm_frequency = 0.;

void sparePWM_stop() {
  pinMode(sparePin,OUTPUT);
  digitalWriteFast(sparePin, sparePinState);
  spare_pwm = false;
}

void sparePWM_start() {

  float f;
  
  f = F_CPU/(1<<pwm_resolution);
  f /= 4;

  pwm_frequency = f;

  Serial.print( "PWM resolution " );
  Serial.println( pwm_resolution );
  analogWriteResolution(pwm_resolution);  // only for the next call
	
  Serial.print( "PWM Frequency " );
  Serial.println( f );
  analogWriteFrequency(sparePin, f);


  Serial.print( "PWM value " );
  Serial.println( pwm_value );
  analogWrite(sparePin,pwm_value);
	
  spare_pwm = true;
}

void pulsePin( unsigned int pin, unsigned int usecs ) {
  digitalToggleFast(pin);
  delayMicroseconds(usecs);
  digitalToggleFast(pin);
}

/* --------------------------------------------------------
   Extension of the interrupt API, re-enable pin interrupt after dropping pending
 */
#define IMR_INDEX   5
#define ISR_INDEX   6

void resumePinInterrupts(uint8_t pin)
{
	if (pin >= CORE_NUM_DIGITAL) return;
	volatile uint32_t *gpio = portOutputRegister(pin);
	uint32_t mask = digitalPinToBitMask(pin);
        gpio[ISR_INDEX] = mask;  // clear pending
	gpio[IMR_INDEX] |= mask;  // re-enable
}

void disablePinInterrupts(uint8_t pin)
{
	if (pin >= CORE_NUM_DIGITAL) return;
	volatile uint32_t *gpio = portOutputRegister(pin);
	uint32_t mask = digitalPinToBitMask(pin);
	gpio[IMR_INDEX] &= ~mask;
}

/* ===================================================================================================
   Timer functions
*/
elapsedMicros elapsed_usecs;

elapsedMicros diagnostic_usecs;
bool diagnostics = false;

uint64_t cycles64( )
{
  static uint32_t oldCycles = ARM_DWT_CYCCNT;
  static uint32_t highDWORD = 0;

  uint32_t newCycles = ARM_DWT_CYCCNT;

  if (newCycles < oldCycles) {
    ++highDWORD;
  }
  oldCycles = newCycles;
  
  return (((uint64_t)highDWORD << 32) | newCycles);
}

/* ===================================================================================================
   External USB command buffer
*/
#define RCVLEN 256
char rcvbuffer[RCVLEN];
uint16_t nrcvbuf = 0;

#define SNDLEN 256
char sndbuffer[SNDLEN];

/* --------------------------------------------------------
   Raw data and send buffers
*/
#define NBUFFERS 16
uint16_t buffers[NBUFFERS][NREADOUT] = { { 0 } };
uint16_t buffer_index = 0;
uint16_t *bufferp = &buffers[buffer_index][0];

/* Data transfer format
 */
#define BINARY 0
#define ASCII 1
unsigned int dataformat = BINARY;
unsigned int counter = 0;

/* --------------------------------------------------------
   For signal averaging, c.f. for dark spectra
   (not implemented yet, ISR and command interface)
 */
uint32_t accumulator[NREADOUT] = { 0  };
uint accumulatorCounter = 0;
bool do_accumulation = false;

/* --------------------------------------------------------
   Data buffer selector functions
 */

inline void initbufferselect( ) {
  buffer_index = 0;
  bufferp = &buffers[0][0];
}

void selectbuffer( unsigned int iselect ) {
  buffer_index = iselect % NBUFFERS;
  bufferp = &buffers[buffer_index][0];
}

void incrementbuffer( ) {
  buffer_index = ( buffer_index + 1 ) % NBUFFERS;
  bufferp = &buffers[buffer_index][0];
}

/* =====================================================================================
   Built-in ADC readout
*/
inline int fastAnalogRead( uint8_t pin ) {
  adc->adc0->startReadFast(pin);
  while ( adc->adc0->isConverting() );
  return adc->adc0->readSingle();
}

unsigned int measureADCspeed( uint8_t pin, unsigned int knt ) {
  
  uint16_t *p16 = bufferp;
  
  unsigned int usecs;

  Serial.print( "test adc " );
  Serial.println( knt );

  elapsed_usecs = 0;

  for ( unsigned int m = 0; m < knt ; m++ ) {
    for ( int n = 0; n < NREADOUT; n++ ) {
      *p16++ = fastAnalogRead(pin);
    }
    p16 = bufferp;
  }

  usecs = elapsed_usecs;

  Serial.print( "elapsed usecs " );
  Serial.println( usecs );
  
  return usecs;
}

/* --------------------------------------------------------
   ADC single reads
   (note that the alpha0 board used the first two channels)
 */
#define NADC_CHANNELS 4
#define ADC_RESOLUTION (3.3/4096.)
uint16_t adc_data[NADC_CHANNELS] = { 0 };

// Set this to something more than 0 to turn on adc reporting.
unsigned int adc_averages = 0;

/* Send ADC readings
 */
void sendADCs( unsigned int averages ) {

  unsigned int i;
  float scalefactor = ADC_RESOLUTION/averages;
  float val;

  int n = averages;

  for( i = 0 ; i < NADC_CHANNELS; i++ ) {
    adc_data[i] = fastAnalogRead( i );
  }
  n--;
  
  while( n-- ) {
    for( i = 0 ; i < NADC_CHANNELS; i++ ) {
      adc_data[i] += fastAnalogRead( i );
    }
  }
  
  Serial.print( "ADCAVGS " );
  Serial.println( averages );
  
  Serial.print( "ADCDATA" );
  for( i = 0 ; i < NADC_CHANNELS; i++ ) {
    val = adc_data[i] * scalefactor;
    Serial.print( " " );
    Serial.print( val, 5 );
  }
  
  Serial.println( "" );
}

/* --------------------------------------------------------
   Send chip temperature
 */

// Set this to something more than 0 to turn on chip temperature reporting
unsigned int chipTemp_averages = 0;

void sendChipTemperature( unsigned int averages ){
  Serial.print( "CHIPTEMPERATURE " );
  Serial.println( tempmonGetTemp() );
}

/* ===========================================================================================
 * EEPROM support for saving identifier and coordinate mapping constants
 */
#define EEPROM_SIZE 1080

#define EEPROM_ID_ADDR 0
#define EEPROM_ID_LEN 64

#define EEPROM_COEFF_ADDR 64
#define EEPROM_NCOEFFS 4
#define EEPROM_COEFF_LEN ( EEPROM_NCOEFFS * sizeof( float) )

#define EEPROM_RESP_ADDR (EEPROM_COEFF_ADDR + EEPROM_COEFF_LEN)
#define EEPROM_NRESPS 4
#define EEPROM_RESP_LEN ( EEPROM_NRESPS * sizeof( float) )

#define EEPROM_UNITS_ADDR ( EEPROM_RESP_ADDR + EEPROM_RESP_LEN )
#define EEPROM_NUNITS 8
#define EEPROM_UNITS_LEN EEPROM_NUNITS

void eeread( unsigned int address, int nbytes, char *p ) {
  while( nbytes-- > 0  && address < EEPROM_SIZE ) {
    *p++ = EEPROM.read(address++);
  }
}

void eewrite( unsigned int address, int nbytes, char *p ) {
  while( nbytes-- > 0 && address < EEPROM_SIZE ) {
    EEPROM.write(address++, *p++);
  }
}

// -------------------------------------------------------------

void eereadUntil( unsigned int address, int nbytes, char *p ) {
  char b = 0xFF;
  while( nbytes-- > 0  && address < EEPROM_SIZE && b ) {
    b = EEPROM.read(address++);
    *p++ = b;
  }
}

void eewriteUntil( unsigned int address, int nbytes, char *p ) {
  while( nbytes-- > 0 && address < EEPROM_SIZE-1 && *p ) {
    EEPROM.write(address++, *p++);
  }
  EEPROM.write(address++, 0);
}

void eeErase( unsigned int address, int nbytes ) {
  while( nbytes-- > 0 && address < EEPROM_SIZE-1 ) {
    EEPROM.write(address++, 0xFF );
  }
}

/* -------------------------------------------------------------
   Store/read Identifier
*/
void eraseIdentifier( ) {
  eeErase( EEPROM_ID_ADDR, EEPROM_ID_LEN );
}

void readIdentifier( char *p ) {
  eereadUntil( EEPROM_ID_ADDR, EEPROM_ID_LEN, p );
}

void storeIdentifier( char *p ) {
  eewriteUntil( EEPROM_ID_ADDR, EEPROM_ID_LEN, p );
}

void printIdentifier( ) {
  readIdentifier( sndbuffer );
  if (sndbuffer[0] != 0xff ) {
    Serial.print( "Identifier: " );
    Serial.println( sndbuffer );
  }
  else {
    Serial.println( "Warning:  identifier is not set" );
  }
}

/* -------------------------------------------------------------
   Store/read Units
*/
void eraseUnits( ) {
  eeErase( EEPROM_UNITS_ADDR, EEPROM_UNITS_LEN );
}

void readUnits( char *p ) {
  eereadUntil( EEPROM_UNITS_ADDR, EEPROM_UNITS_LEN, p );
}

void storeUnits( char *p ) {
  eewriteUntil( EEPROM_UNITS_ADDR, EEPROM_UNITS_LEN, p );
}

void printUnits( ) {
  readUnits( sndbuffer );
  if (sndbuffer[0] != 0xff ) {
    Serial.print( "units: " );
    Serial.println( sndbuffer );
  }
  else {
    Serial.println( "Warning:  units is not set" );
  }
}

/* -------------------------------------------------------------
   Store/read Wavelength Coeffs
*/

void eraseCoefficients( ) {
  eeErase( EEPROM_COEFF_ADDR, EEPROM_COEFF_LEN );
}

void readCoefficients( float *vals ) {
  eeread( EEPROM_COEFF_ADDR, EEPROM_COEFF_LEN, (char *) vals );
}

void storeCoefficients( float *vals ) {
  eewrite( EEPROM_COEFF_ADDR, EEPROM_COEFF_LEN, (char *) vals );
}

void printCoefficients( ) {

  float vals[EEPROM_NCOEFFS];

  readCoefficients( vals );
  
  Serial.print( "coefficients" );
  
  for( int n = 0; n < EEPROM_NCOEFFS; n++ ) {
    sprintf( sndbuffer, " %.8g", vals[n] );
    Serial.print( sndbuffer );
  }
  Serial.println( "" );
}

/* -----------------------------------------------
   Resonse function coefficients
*/

void eraseResponse( ) {
  eeErase( EEPROM_RESP_ADDR, EEPROM_RESP_LEN );
}

void readResponse( float *vals ) {
  eeread( EEPROM_RESP_ADDR, EEPROM_RESP_LEN, (char *) vals );
}

void storeResponse( float *vals ) {
  eewrite( EEPROM_RESP_ADDR, EEPROM_RESP_LEN, (char *) vals );
}

void printResponse( ) {

  float vals[EEPROM_NRESPS];

  readResponse( vals );
  
  Serial.print( "response" );
  
  for( int n = 0; n < EEPROM_NRESPS; n++ ) {
    sprintf( sndbuffer, " %.8g", vals[n] );
    Serial.print( sndbuffer );
  }
  Serial.println( "" );
}

/* ==========================================================================
   Filesys functions
 */

bool readfromfile( char *name ) {
  File file;
  
  if (!filesys_ready) {
    Serial.println( "Error: filesys not ready" );
    return false;
  }

  if ( !(file = filesys.open( name, FILE_READ )) ) {
    Serial.println( "Error: not able to open for read" );
    return false;
  }

  file.read( (char *) &bufferp[DATASTART], NPIXELS*2 );
  file.close();
  return true;
}

bool writetofile( char *name ) {
  File file;
  
  if (!filesys_ready) {
    Serial.println( "Error: filesys not ready" );
    return false;
  }

  if ( !(file = filesys.open( name, FILE_WRITE_BEGIN )) ) {
    Serial.println( "Error: not able to open for write" );
    return false;
  }

  file.write( (char *) &bufferp[DATASTART], NPIXELS*2 );

  Serial.print( "saved to ");
  Serial.print( file.name() );
  Serial.print( " ");
  Serial.print( NPIXELS*2 );
  Serial.print( " ");
  Serial.println( file.size() );

  file.close();
  
  return true;
}

void listfiles() {
  File dir, entry;
  if (filesys_ready) {
    Serial.print("Space Used = ");
    Serial.println(filesys.usedSize());
    Serial.print("Filesystem Size = ");
    Serial.println(filesys.totalSize());
    dir = filesys.open( "/" );
    while( (entry = dir.openNextFile()) ) {
      Serial.print( entry.name() );
      Serial.print( " " );
      Serial.print( entry.size(), DEC );
      Serial.println();
      entry.close();
    }
    dir.close();
  }
  else {
    Serial.println( "Error: filesys not ready" );
  }
}

/* ==========================================================================
   Sending data and messages
 */
void sendBuffer_Formatted( ) {
  uint16_t *p16 = &bufferp[DATASTART];
  Serial.print( "DATA " );
  Serial.println( NPIXELS );
  for ( int n = 0; n < NPIXELS; n++ ) {
    Serial.println( p16[n] );
  }
  Serial.println( "END DATA" );
}

void sendBuffer_Binary( ) {
  uint16_t *p16 = &bufferp[DATASTART];
  Serial.print( "BINARY16 " );
  Serial.println( NPIXELS );
  Serial.write( (byte *) p16, NBYTES );
  Serial.println( "END DATA" );
}

void sendData( ) {

  if ( dataformat == BINARY ) {
    sendBuffer_Binary( );
  }
  else if ( dataformat == ASCII ) {
    sendBuffer_Formatted( );
  }
}

void sendTestData() {
  uint16_t *p16 = &bufferp[DATASTART];
  Serial.println( "TESTDATA" );
  for ( int n = 0; n < NPIXELS; n++ ) {
    p16[n] = n;
  }
  sendData();
}

/* ======================================================================================================
   CCD sensor read

   Assumes on entry  ICG is High,  SH is low

   The time between calls is the shutter period

   ------------------------------------------------------------------------------------------- */

// Internal shutter sequence timer, we loop for the wait
uint32_t shutter_timer_cycles_holder;
inline uint32_t shutter_timer_usecs() {
  return (ARM_DWT_CYCCNT - shutter_timer_cycles_holder)/CYCLES_PER_USEC;
}

inline void shutter_timer_start() {
  shutter_timer_cycles_holder = ARM_DWT_CYCCNT;
}

// Measure elapsed shutter duration
uint32_t frameset_elapsed_cycles_holder;
inline uint32_t frameset_elapsed_usecs() {
  return (ARM_DWT_CYCCNT - frameset_elapsed_cycles_holder)/CYCLES_PER_USEC;
}

inline void frameset_elapsed_start() {
  frameset_elapsed_cycles_holder = ARM_DWT_CYCCNT;
}

/* ------------------------------------------
   Delay function call, compatible with isr,  delay up to 10msec else use timer.
   The trick is that the timer calls the function right away, so we need to skip one
*/
void (*delayed_function_ptr)() = 0;
IntervalTimer delay_Timer;

uint32_t timertest_countdown = 0;
void timertest_isr() {
  uint32_t usecs = frameset_elapsed_usecs();

  Serial.print(timertest_countdown);
  Serial.print(" ");
  Serial.println(usecs);

  if (timertest_countdown) {
    timertest_countdown--;
  }
  else {
    delay_Timer.end();
  }
}

void timertest() {
  frameset_elapsed_start();
  timertest_countdown = 10;
  delay_Timer.begin(timertest_isr,100000);
}


void delayed_isr() {
  if (delayed_function_ptr) {
    (*delayed_function_ptr)();
    delay_Timer.end();
  }
}

inline void delay_call( void (*funct)(), uint32_t delay_usecs ) {
  if ( delay_usecs < 10000 ) {
    delayMicroseconds(delay_usecs);
    (*funct)();
  }
  else {
    delayed_function_ptr = funct;
    delay_Timer.begin(delayed_isr,delay_usecs);
  }
}

/* ------------------------------------------
   Shutter controls
*/

// Shutter and frame intervals and counts, we set other fields from these
uint32_t shutterUsecs = 10000;
uint32_t frameUsecs   = 10000;
uint32_t frameCounts  = 0;
uint32_t frameCounter  = 0;
bool is_frame_triggered = false;

// Flag for the busy bit, inhibits the idler
bool is_busy = false;

// Flag, running
bool is_running = false;

// Shutter A is just a shutter with sync and busy pin toggles
IntervalTimer shutterATimer;
uint32_t shutterA_usecs = 0;
uint32_t shutterA_counter = 0;
uint32_t shutterA_counts = 0;
bool is_shutterA_clocked = false;

// Shutter B is shutter and read, also with sync and busy pin toggles
IntervalTimer shutterBTimer;
uint32_t shutterB_usecs = 10000;
uint32_t shutterB_counter = 0;
uint32_t shutterB_counts = 0;
bool is_shutterB_clocked = false;

// Measures the actual elapsed shutter time from A
unsigned int shutter_usecs_elapsed = 0;

// option to skip frames
unsigned int shutterCounterDivider = 0;

// We can clock or trigger single or multiple frames
IntervalTimer outerTimer;
unsigned int outerUsecs = 10000;
unsigned int outerCounter = 0;
unsigned int outerCounts = 0;

bool is_outer_clocked = false;
bool is_outer_triggered = false;

// Gated frames
unsigned int gatedCounter = 0;
unsigned int gatedCounts = 0;
unsigned int is_gated = false;

// Idler
IntervalTimer idlerTimer;
uint32_t idlerUsecs = 100;
uint32_t idlerCounter = 0;
uint32_t idlerCounts = 0;
volatile bool idlerRunning = false;
volatile bool idlerAuto = true;

// Latch
bool do_latch = false;
volatile unsigned int latch_value = 0;
unsigned int latch_pixel = 0;

// Sync output
volatile bool do_sync_start = false;
volatile bool do_sync_shutter = false;
volatile bool do_sync_holdoff = false;

// Holdoff
IntervalTimer holdoff_delay_Timer;
void (*holdoff_function_ptr)() = 0;
uint32_t holdoff_usecs = 0;


/* --------------------------
   Stop everything
 */
void stop_all( ) {
  idlerTimer.end();
  idlerRunning = false;  

  detachInterrupt(digitalPinToInterrupt(interruptPin));
  is_frame_triggered = false;
  is_outer_triggered = false;
  is_gated = false;

  outerTimer.end();
  is_outer_clocked = false;

  shutterATimer.end();
  is_shutterA_clocked = false;

  shutterBTimer.end();
  is_shutterB_clocked = false;

  sparePWM_stop();

  if (is_busy) {
    digitalToggleFast(busyPin);
    busyPinState = !busyPinState;
    is_busy = false;
  }

  is_running = false;
}


/* --------------------------
   Diagnostics
 */
void dumpAll( ){
  Serial.println( "# =========================" );
  Serial.print( "#busy " ); Serial.print( is_busy );
  Serial.println( "" );
  
  Serial.print( "#sync on start " ); Serial.print( do_sync_start );
  Serial.print( " shutter " ); Serial.print( do_sync_shutter );
  Serial.print( " holdoff " ); Serial.print( do_sync_holdoff );
  Serial.println( "" );

  Serial.print( "#holdoff usecs" ); Serial.print( holdoff_usecs );
  Serial.println( "" );

  Serial.print( "#frame usecs" ); Serial.print( frameUsecs );
  Serial.print( " shutter " ); Serial.print( shutterUsecs );
  Serial.print( " counts " ); Serial.print( frameCounts);
  Serial.println( "" );

  Serial.print( "#shutterA usecs " ); Serial.print( shutterA_usecs );
  Serial.print( " counter " ); Serial.print( shutterA_counter );
  Serial.print( " counts " ); Serial.print( shutterA_counts );
  Serial.println( "" );

  Serial.print( "#shutterB usecs " ); Serial.print( shutterB_usecs );
  Serial.print( " counter " ); Serial.print( shutterB_counter );
  Serial.print( " counts " ); Serial.print( shutterB_counts );
  Serial.println( "" );  
  
  Serial.print( "#outer usecs " ); Serial.print( outerUsecs );
  Serial.print( " counter " ); Serial.print( outerCounter );
  Serial.print( " counts " ); Serial.print( outerCounts );
  Serial.print( " clocked " ); Serial.print( is_outer_clocked );
  Serial.print( " triggered " ); Serial.print( is_outer_triggered );
  Serial.println( "" );  
  
  Serial.print( "#gate counter " ); Serial.print( gatedCounter );
  Serial.print( " counts " ); Serial.print( gatedCounts );
  Serial.print( " gated " ); Serial.print( is_gated );
  Serial.println( "" );  
  
  Serial.print( "#idler Usecs " ); Serial.print( idlerUsecs );
  Serial.print( " counter " ); Serial.print( idlerCounter );
  Serial.print( " counts " ); Serial.print( idlerCounts );
  Serial.print( " running " ); Serial.print( idlerRunning );
  Serial.print( " auto " ); Serial.print( idlerAuto );
  Serial.println( "" );

  Serial.print( "#do_latch " ); Serial.print( do_latch );
  Serial.print( " latch_value " ); Serial.print( latch_value );
  Serial.print( " latch_pixel " ); Serial.print( latch_pixel );
  Serial.println( "" );

  Serial.print( "#busy state " ); Serial.print( busyPinState );
  Serial.print( " sync " ); Serial.print( syncPinState );
  if (!spare_pwm) {
    Serial.print( " spare " );
    Serial.print( sparePinState );
  }
  Serial.println( "" );
  
  if (spare_pwm) {
    Serial.print( "#pwm" );
    Serial.print( " resolution " ); Serial.print( pwm_resolution );
    Serial.print( " frequency " ); Serial.print( pwm_frequency );
    Serial.print( " value " ); Serial.print( pwm_value );
    Serial.println( "" );
  }

}

/* ----------------------------------------------------------------
   latch functions
*/

void clearLatch() {
  do_latch = false;
  latch_value = 0;
  Serial.println( "latch cleared" );      
}

bool latchupdate( ) {
  
  uint16_t *p16 = &bufferp[DATASTART];
  uint16_t uval = 0;
  uint16_t umax = 0;
  int nmax = 0;
  int n;
  

  if (latch_pixel>0) {
    uval = p16[latch_pixel];
    nmax = latch_pixel;
  }

  else {
    umax = 0;
    for ( n = 0; n < NPIXELS; n++ ) {
      uval = *p16++;
      if ( uval > umax ) {
	umax = uval;
	nmax = n;
      }
    }
  }
    
  if ( umax > latch_value) {

    latch_value = umax;
  
    Serial.print( "latched " );
    Serial.print( nmax );
    Serial.print( " " );
    Serial.println( latch_value );
    
    return true;
  }
  
  return false;
}

/* ----------------------------------------------------------------
   TCD1304 shutter and readout functions
*/

inline uint32_t ShutterSequence_Start_() {
  uint32_t utemp;
  // Shift Gate cycle - start with Integration Clear Gate -> low
  digitalWriteFast(ICGPin, LOW);      // <===

  // Need at least 100 ns from ICG low to Shift Gate high
  delayNanoseconds(100);

  // Shift Gate, need 1usec pulse
  digitalWriteFast(SHPin, HIGH);      // <***
  delayMicroseconds(1);
  digitalWriteFast(SHPin, LOW);

  // measure timing relative to the trigger
  utemp = frameset_elapsed_usecs();

  // Elapsed time since last shutter (or start), and restart
  shutter_usecs_elapsed = shutter_timer_usecs();
  shutter_timer_start();

  return utemp;
}

inline void ShutterSequence_Finish_() {
  // Integration clear gate (ICG), start readout, need 1usec from SH low
  while ( shutter_timer_usecs() < 1);
  digitalWriteFast(ICGPin, HIGH);     // <===
}

inline void ShutterPulse_() {
  digitalWriteFast(SHPin, HIGH);
  delayMicroseconds(1);
  digitalWriteFast(SHPin, LOW);
}

inline void SensorReadOutLoop_() {

  uint16_t *p16 = bufferp;
  int i;
  
  // Wait for the next clock high transition/
  while ( CLOCKREAD ) {}
  while ( !CLOCKREAD ) {}

  // Read first word
  *p16++ = fastAnalogRead(analogPin);
  
  i = 1;
  do {
    
    // It takes 1 usec to read the ADC, so at 2MHz, so we wait for 2 more clocks
    while ( CLOCKREAD ) {}
    while ( !CLOCKREAD ) {}

    while ( CLOCKREAD ) {}
    while ( !CLOCKREAD ) {}
    
    *p16++ = fastAnalogRead(analogPin);
    
    i++;

  } while ( i < NREADOUT );
 
}

/* =================================================================
 * Elements of the generic Idler
 */
inline void LCCDIdler_() {
  ShutterPulse_( );
}

void LCCDIdler() {
  if ( !is_busy ) {
    LCCDIdler_();
  }
}

inline void LCCDIdlerStart_() {
  idlerTimer.begin( LCCDIdler, idlerUsecs );
  idlerRunning = true;
}

void LCCDIdlerStart( ) {
  LCCDIdlerStop();
  if (idlerUsecs >= SHUTTERMIN) {
    LCCDIdlerStart_();
  }
}

inline void LCCDIdlerStop_() {
  idlerTimer.end();
  idlerRunning = false;
}

void LCCDIdlerStop( ) {
  if (idlerRunning) {
    LCCDIdlerStop_();
  }
}

bool quickClean(unsigned int usecs, unsigned int ncycles)
{
  if (is_busy) {
    Serial.println( (char *)"Error: Quick clean while busy." );
    return false;
  }

  if (usecs > 10000) {
    while (ncycles) {
      LCCDIdler_();
      delay(usecs/1000);
      ncycles--;
    }
  }
  else if (usecs > 0) {
    while (ncycles) {
      LCCDIdler_();
      delayMicroseconds(usecs);
      ncycles--;
    }
  }
  return true;
}


/* ----------------------------------
   Auto idler, launched by shutterB
*/
void IdlerCount_isr() {
  if ( ++idlerCounter <= idlerCounts ) {
#ifdef DIAGNOSTICS_IDLER
    uint32_t utemp = shutter_timer_usecs();
    Serial.print("idler ");
    Serial.println(utemp);
#endif
    
    ShutterPulse_( );
  }
  else {
    idlerTimer.end();
    idlerRunning = false;
  }
}

void IdlerCount_start() {
  idlerCounter = 0;
  idlerTimer.begin( IdlerCount_isr, idlerUsecs );
  idlerRunning = true;
}

/* =================================================================
 * Elements of the generic start and read
 */

/* Shutter only, pulse sync and set busy
 */
void ShutterA_isr() {

#ifdef DIAGNOSTICS_IDLER
  uint32_t utemp = shutter_timer_usecs();
  Serial.print("A top ");
  Serial.println(utemp);
#endif

  if (!shutterA_counter) {
    // busy is asserted for each frameset
    if (!is_busy) {
      digitalToggleFast(busyPin);
      busyPinState = !busyPinState;
      is_busy = true;
    }
    else {
      // This is an error condition, actually
      Serial.println("Warning: Shutter A counter 0 while already busy, continuing");
    }
  }

  /* Start the shutter sequence, we have a few usec to do stuff
   */
  ShutterSequence_Start_();
  
  // First time through
  if (!shutterA_counter) {
    // Assert Sync on start?
    if (do_sync_start || do_sync_shutter) {
      digitalToggleFast(syncPin);
      delayMicroseconds(1);
      digitalToggleFast(syncPin);
    }
  }

  // Else, assert Sync on each shutter?
  else if (do_sync_shutter) {
    digitalToggleFast(syncPin);
    delayMicroseconds(1);
    digitalToggleFast(syncPin);
  }

  /* Finish the shutter sequence
   */
  ShutterSequence_Finish_();
    
  // Stop the timer?
  if (++shutterA_counter >= shutterA_counts) {    
    if (is_shutterA_clocked) {
      shutterATimer.end();
      is_shutterA_clocked = false;
    }
  }
  
}

void shutterA_clock_start() {
  if (is_shutterA_clocked) {
    Serial.println("Error: shutterA_start while clocked");
  }

  shutterA_counter = 0;
  if (shutterA_counts > 1) {
    is_shutterA_clocked = true;
  }

  // Note we tried putting this inside the isr, timing is better like this
  if (shutterA_counts > 1) {
    is_shutterA_clocked = true;
    shutterATimer.begin( ShutterA_isr, shutterA_usecs );
  }

  // first one right now
  ShutterA_isr();

  
}

void shutterA_clock_stop() {
  if (is_shutterA_clocked) {
    shutterATimer.end();
    is_shutterA_clocked = false;
  }
}

// =======================================================

void ShutterB_isr() {

  uint32_t trigger_usecs_elapsed;

  if (!is_busy) {
    Serial.println("Warning: Shutter B while not busy, ignored");
    return;
  }

  trigger_usecs_elapsed = ShutterSequence_Start_();

#ifdef DIAGNOSTICS_IDLER  
  Serial.print("B top ");
  Serial.println(trigger_usecs_elapsed);
#endif
  
  /* Stuff inside the 1 usec from SH pin to ICG
   */
  
  // not the last
  if (++shutterB_counter < shutterB_counts) {
    // sync on back to back to back shutters
    if (do_sync_shutter) {
      digitalToggleFast(syncPin);
      //TOGGLESYNCPIN;
      delayMicroseconds(1);
      digitalToggleFast(syncPin);
      //TOGGLESYNCPIN;
    }
  } 

  // Done with Shutter b?
  else {
    // stop the clock
    if (is_shutterB_clocked) {
      shutterBTimer.end();
      is_shutterB_clocked = false;
    }
  }
  
  ShutterSequence_Finish_();

  /* --------------------------------------------------------
     Until here it was like shutter A, now we do the readout
   */
  SensorReadOutLoop_();
  
  /* -------------------------------------------------------
     Send the data?
   */
  if (shutterB_counter==1) {
    Serial.print("FRAMESET START ");
    Serial.println( outerCounter );
  }

  if ( !do_latch || latchupdate() ) {

    Serial.print( "ELAPSED " );
    Serial.println( trigger_usecs_elapsed );

    Serial.print( "COUNTER " );
    Serial.println( shutterB_counter );
      
    Serial.print( "SHUTTER " );
    Serial.println( shutter_usecs_elapsed );
      
    if (adc_averages) {
      sendADCs( adc_averages );
    }
    
    sendData( );
  }  

  if (shutterB_counter >= shutterB_counts) {
    Serial.println("FRAMESET END");

    if (outerCounter >= outerCounts) {
      Serial.println("COMPLETE");
      is_running = false;

      if (is_outer_clocked) {
	outerTimer.end();
	is_outer_clocked = false;
      }
    
      if (is_outer_triggered) {
	detachInterrupt(digitalPinToInterrupt(interruptPin));
	is_outer_triggered = false;
      }
      
    }
    
    // clear busy, safe to trigger again or whatever
    if (is_busy) {
      digitalToggleFast(busyPin);
      busyPinState = !busyPinState;
      is_busy = false;
    }
  }

  /* this launches the idler until the next shutter
   */
  else if ( idlerAuto && idlerCounts ) {
    IdlerCount_start();
  }
}

void shutterB_clock_start() {
  if (is_shutterB_clocked) {
    Serial.println("Error: shutterB_start while clocked");
  }
  shutterB_counter = 0;

  // Note we tried putting this inside the isr, timing is better like this
  if (shutterB_counts > 1) {
    is_shutterB_clocked = true;
    shutterBTimer.begin( ShutterB_isr, shutterB_usecs );
  }
  // First one right now
  ShutterB_isr();
}

void shutterB_clock_stop() {
  if (is_shutterB_clocked) {
    shutterBTimer.end();
    is_shutterB_clocked = false;
  }
}

/* ==========================================================================
   Hold off
*/
void holdoff_delay_isr_() {
  holdoff_delay_Timer.end();
  if (holdoff_function_ptr) {
    (*holdoff_function_ptr)();
  }
}

inline void holdoff_isr( ) {
  if (do_sync_holdoff) {
    digitalToggleFast(syncPin);
    //TOGGLESYNCPIN;
    delayMicroseconds(1);
    digitalToggleFast(syncPin);
    //TOGGLESYNCPIN;
  }
  if ( holdoff_usecs < 10000 ) {
   delayMicroseconds(holdoff_usecs);
   (*holdoff_function_ptr)();
  }
  else {
    holdoff_delay_Timer.begin(holdoff_delay_isr_,holdoff_usecs);
  }
}

void set_holdoff_function( void (*funct)() ) {
  holdoff_function_ptr = funct;
}

void cancel_holdoff( ) {
  holdoff_delay_Timer.end();
}

/* =======================================================================
   Gated frames, intended to be called from interrupt on "change"
*/
bool gatedFrames_flag = true;
void gatedFrame_isr() {

  // Start the shutter interval
  if (gatedFrames_flag) {
    ShutterA_isr();
    gatedFrames_flag = false;  // next, readout
  }

  // Readout
  else {
    ShutterB_isr();
    gatedFrames_flag = true;  // next, start

    if (++gatedCounter >= gatedCounts) {
      detachInterrupt(digitalPinToInterrupt(interruptPin));
      is_gated = false;
      is_running = false;
    }    
  }  
}

void gatedFrameset_start() {

  // Note: SETS must come after FRAMES, this initiates accumulater configuration in the host
  Serial.println( "GATE START" );
  Serial.print( "FRAMES " ); Serial.println( gatedCounts );
  Serial.println( "SETS 1" );

  frameset_elapsed_start();
  
  // complete when this set is complete
  outerCounter = 1;
  outerCounts  = 1;
  
  gatedCounter = 0;
  gatedCounts = frameCounts;
  
  shutterA_counter = 0;
  shutterA_counts = gatedCounts;

  shutterB_counter = 0;
  shutterB_counts = gatedCounts;

  is_gated = true;
  is_running = true;

  gatedFrames_flag = true;
  attachInterrupt(digitalPinToInterrupt(interruptPin), gatedFrame_isr, intedgemode);  
}

void gatedFrameset_stop() {
  if (is_gated) {
    detachInterrupt(digitalPinToInterrupt(interruptPin));
    is_gated = false;
    is_running = false;
  }
}

/* =======================================================================
   Single frames, isr
*/
bool interruptbusy = false;

void triggeredSingleFrame_isr() {

  if (interruptbusy) return;
  interruptbusy = true;

  if ( frameCounter < frameCounts ) {

    Serial.print("triggeredSingleFrame counter ");
    Serial.print( frameCounter );
    Serial.print( "/" );
    Serial.println( frameCounts );

    ShutterA_isr();

    delay_call( ShutterB_isr, shutterUsecs );  

    if (++frameCounter >= frameCounts) {
      Serial.println("trigger count complete");
      if (is_frame_triggered) {
	detachInterrupt(digitalPinToInterrupt(interruptPin));
	is_frame_triggered = false;
	is_running = false;
      }
    }
    
  }
  interruptbusy = false;
}

void triggeredSingleFrame_setup() {

  outerCounter = 1;
  outerCounts = 1;

  frameCounter = 0;
  
  shutterA_counter = 0;
  shutterA_counts = frameCounts;
  shutterA_usecs = 0;
  is_shutterA_clocked = false;
  
  shutterB_counter = 0;
  shutterB_counts = frameCounts;
  shutterB_usecs = 0;
  is_shutterB_clocked = false;
}

void triggeredSingleFrame_start() {

  triggeredSingleFrame_setup();

  //Serial.println("triggeredSingleFrame_start");
  //dumpAll();
  
  frameset_elapsed_start();
  
  is_running = true;  
  is_frame_triggered = true;
  attachInterrupt(digitalPinToInterrupt(interruptPin), triggeredSingleFrame_isr, intedgemode);
}

void triggeredSingleFrame_start_holdoff() {

  triggeredSingleFrame_setup();

  frameset_elapsed_start();
  
  set_holdoff_function( triggeredSingleFrame_isr );

  is_running = true;  
  is_frame_triggered = true;
  attachInterrupt(digitalPinToInterrupt(interruptPin), holdoff_isr, intedgemode);
}

bool triggeredSingleFrame_launcher() {

  // Note: SETS must come after FRAMES, this initiates accumulater configuration in the host
  Serial.println( "TRIGGERED SINGLES START" );  
  Serial.print( "INTERVAL " );Serial.println( shutterUsecs );
  Serial.print( "FRAMES " ); Serial.println( frameCounts );
  Serial.println( "SETS 1" );
  
  if (holdoff_usecs) {
    triggeredSingleFrame_start_holdoff();
  }
  else {
    triggeredSingleFrame_start();
  }
  return true;
}

void triggeredSingleFrame_stop() {
  if (is_frame_triggered) {
    detachInterrupt(digitalPinToInterrupt(interruptPin));
    is_frame_triggered = false;
    is_running = false;
  }
}

/* =======================================================================
   Manual frames, from the command line
   Setup for this is the same as clocked frames, below
*/
inline void shutterDelay_() {
  uint32_t utemp1 = shutterUsecs/1000;
  uint32_t utemp2 = shutterUsecs%1000;

  if (utemp1) {
    delay(utemp1);
  }
  if (utemp2) {
    delayMicroseconds(utemp2);
  }
}

void manualFrames() {

  frameset_elapsed_start();
  
  outerCounter++;
  
  shutterA_counter = 0;
  shutterB_counter = 0;

  is_running = true;
  
  if (shutterA_counts == 1) {
    ShutterA_isr();
    while (shutterB_counter<shutterB_counts) {
      shutterDelay_();
      ShutterB_isr();
      if (Serial.available()) {
	Serial.println("Error: abort on command" );
	break;
      }
    }    
  }

  else {
    while (shutterB_counter<shutterB_counts) {
      ShutterA_isr();
      shutterDelay_();
      ShutterB_isr();      
      if (Serial.available()) {
	Serial.println("Error: abort on command" );
	break;
      }
    }
  }
  
  is_running = false;
}

void manualFrames_holdoff() {

  uint32_t utemp1 = holdoff_usecs/1000;
  uint32_t utemp2 = holdoff_usecs%1000;
  
  if (do_sync_holdoff) {
    digitalToggleFast(syncPin);
    //TOGGLESYNCPIN;
    delayMicroseconds(1);
    digitalToggleFast(syncPin);
    //TOGGLESYNCPIN;
  }

  if (utemp1) delay(utemp1);
  if (utemp2) delayMicroseconds(utemp2);

  if (Serial.available()) {
    Serial.println("Error: abort on command" );
  }

  else {
    manualFrames();
  }
  
}

// check parameters, setup, and run it
bool manualFrames_launcher() {

  if (clockedFrames_setup()) {

    // Note: SETS must come after FRAMES, this initiates accumulater configuration in the host
    Serial.println( "LOOP START " );
    Serial.print( "INTERVAL " ); Serial.println( shutterUsecs );
    Serial.print( "FRAMES " ); Serial.println( frameCounts );
    Serial.println( "SETS 1" );
  
    if (holdoff_usecs) {
      manualFrames_holdoff();
    }
    else {
      manualFrames();
    }
    return true;
  }
  return false;
}

/* =======================================================================
   Clocked Frame set, called from cli, or ISR, or outer clock
   Pre-requisite:  clockedFrames_setup(), see below
   Note: needs "is_running = true;" in the calling routine
*/
void clockedFrameset_start_isr() {

  frameset_elapsed_start();
  
  outerCounter++;
  
  shutterA_counter = 0;
  shutterB_counter = 0;
  
  // Shutter A starts the interval
  shutterA_clock_start();

  // Shutter B starts after shutter time
  delay_call( shutterB_clock_start, shutterUsecs );

  // Done?
  if (outerCounter>=outerCounts) {
    if (is_outer_clocked) {
      outerTimer.end();
      is_outer_clocked = false;
    }
    else if (is_outer_triggered) {
      detachInterrupt(digitalPinToInterrupt(interruptPin));
      is_outer_triggered = false;
    }
    is_running = false;
  }
}

void clockedFrameset_stop() {
  shutterA_clock_stop();
  shutterB_clock_stop();
  is_running = false;
}

/* ----------------------------------------------------------------
   Outer clocking of clocked sets
*/
void outerClock_start() {
  is_running = true;
  outerCounter = 0;
  if (outerCounts>1) {
    is_outer_clocked = true;
    outerTimer.begin( clockedFrameset_start_isr, outerUsecs );
  }
  clockedFrameset_start_isr();
}

void outerClock_start_holdoff() {
  
  set_holdoff_function( clockedFrameset_start_isr );

  is_running = true;
  outerCounter = 0;
  
  if (outerCounts>1) {
    is_outer_clocked = true;
    outerTimer.begin( holdoff_isr, outerUsecs );
  }

  holdoff_isr();
}

// check parameters, setup, and launch
bool outerClock_launcher() {

  if (clockedFrames_setup()) {

    // Note: SETS must come after FRAMES, this initiates accumulater configuration in the host
    Serial.println( "CLOCKED START" );
    Serial.print( "OUTER CLOCK " ); Serial.println( outerUsecs );
    Serial.print( "CLOCK " ); Serial.println( frameUsecs );
    Serial.print( "INTERVAL " ); Serial.println( shutterUsecs );
    Serial.print( "FRAMES " ); Serial.println( frameCounts );
    Serial.print( "SETS " ); Serial.println( outerCounts );
  
    if (holdoff_usecs) {
      outerClock_start_holdoff();
    }
    else {
      outerClock_start();
    }
    return true;
  }
  return false;
}

void outerClock_stop() {
  if (is_outer_clocked) {
    outerTimer.end();
    clockedFrameset_stop();
    is_outer_clocked = false;
    is_running = false;
  }
}

/* ----------------------------------------------------------------
   Outer trigger of clocked set
*/
void outerTrigger_start() {
  is_running = true;
  outerCounter = 0;
  is_outer_triggered = true;
  attachInterrupt(digitalPinToInterrupt(interruptPin), clockedFrameset_start_isr, intedgemode);
}

void outerTrigger_start_holdoff() {
  is_running = true;
  outerCounter = 0;
  is_outer_triggered = true;
  set_holdoff_function( clockedFrameset_start_isr );
  attachInterrupt(digitalPinToInterrupt(interruptPin), holdoff_isr, intedgemode);
}

// check parameters, setup, and launch
bool outerTrigger_launcher() {

  if (clockedFrames_setup()) {

    // Note: SETS must come after FRAMES, this initiates accumulater configuration in the host
    Serial.println( "TRIGGERED SETS START" );
    Serial.print( "CLOCK " ); Serial.println( frameUsecs );
    Serial.print( "INTERVAL " ); Serial.println( shutterUsecs );
    Serial.print( "FRAMES " ); Serial.println( frameCounts );
    Serial.print( "SETS " ); Serial.println( outerCounts );
  
    if (holdoff_usecs) {
      outerTrigger_start_holdoff();
    }
    else {
      outerTrigger_start();
    }
    return true;
  }
  return false;    
}

void outerTrigger_stop() {
  if (is_outer_triggered) {
    detachInterrupt(digitalPinToInterrupt(interruptPin));
    clockedFrameset_stop();
    is_outer_triggered = false;
    is_running = false;
  }
}

/* =======================================================================
   Setup clocked shutter parameters,
   inputs: frameCounts, shutterUsecs, frameUsecs
 */
bool clockedFrames_setup() {

  bool clocks_configured = false;
  
  idlerCounts = 0;
  
  // Shutter too short
  if (shutterUsecs < SHUTTERMIN) {
    Serial.print( "Error: shutter ");
    Serial.print( shutterUsecs );
    Serial.print( " < min " );
    Serial.println( SHUTTERMIN );
    clocks_configured = false;
  }

  // Frame less than shutter
  else if (frameUsecs < shutterUsecs) {
    Serial.print( "Error: frame ");
    Serial.print( frameUsecs );
    Serial.print( " < shutter " );
    Serial.println( shutterUsecs );
    clocks_configured = false;
  }

  // Frame equals shutter
  else if (frameUsecs == shutterUsecs) {    
    if (frameUsecs<TRANSFERUSECS) {
      Serial.print( "Error: frame ");
      Serial.print( frameUsecs );
      Serial.print( " < transfer time " );
      Serial.println( TRANSFERUSECS );
      clocks_configured = false;
    }
    else {
      // we are going to call A just once, then B until done
      shutterA_usecs  = 0;
      shutterA_counts = 1;
      shutterB_usecs  = shutterUsecs;
      shutterB_counts = frameCounts;
      clocks_configured = true;
    }
  }

  // Frame longer than shutter + transfer?
  else if (frameUsecs<(shutterUsecs+TRANSFERUSECS)) {
    Serial.print( "Error: frame ");
    Serial.print( frameUsecs );
    Serial.print( " > shutter " );
    Serial.print( shutterUsecs );
    Serial.print( " + transfer " );
    Serial.println( TRANSFERUSECS );
    clocks_configured = false;
  }

  // Else clock A and B in parallel
  else {
    shutterA_usecs = frameUsecs;
    shutterA_counts = frameCounts;
    shutterB_usecs = frameUsecs;
    shutterB_counts = frameCounts;
    clocks_configured = true;

    if (idlerAuto) {
      if (frameUsecs > (shutterUsecs+TRANSFERUSECS) + 2*idlerUsecs) {
	uint32_t freeusecs = frameUsecs - (shutterUsecs+TRANSFERUSECS);
	idlerCounts = freeusecs/idlerUsecs - 1;
      }
    }
  }

  return clocks_configured;
}

/* ==========================================================================
   Parse for the trigger edge specification (rising,falling,change)
 */

bool parseEdgeMode( char *pc ) {
  if (!strcmp(pc,"rising")) {
    intedgemode = RISING;
    return true;
  }
  if (!strcmp(pc,"falling")) {
    intedgemode = FALLING;
    return true;
  }
  if (!strcmp(pc,"change")) {
    intedgemode = CHANGE;
    return true;
  }
  return false;
}

/* ====================================================================
   Parse srings for words, numbers, etc

   Apart from wordLength(), these return a pointer to the
   next character in the string, or null if they fail
   
 */
unsigned int wordLength( char *s ) {
  char *s0 = s;
  while( *s && !isspace( *s ) ){
    s++;
  }
  return (s-s0);  
}

char *nextWord( char *s ) {
  while ( s  && *s && !isspace(*s) ) {
    s++;
  }
  while ( s  && *s && isspace(*s) ) {
    s++;
  }
  if ( s && *s ) {
    return s;
  }
  return 0;   
}

unsigned int countWords( char *s ) {
  unsigned int n = 0;
  if ( s && *s && !isspace(*s) ) {
    n++;
  }
  while( (s=nextWord(s)) ) {
    n++;
  }
  return n;
}

char *startsWith( char *s, const char *key ) {

  int n = strlen(key);

  if ( s && *s && key && *key ) {

    // skip leading spaces
    while ( *s && isspace(*s) ) s++;
      
    if ( *s && !strncmp( s, key, n ) ) {
      return s + n;
    }
    
  }
  return 0;
}

char *parseUint( char *s, unsigned int *u ) {
  unsigned long int l;
  char *p = s;
  if ( p && *p ) {
    l = strtoul( s, &p, 0 );
    if ( (p > s) && (l <= UINT_MAX) ) {
      *u = (unsigned int) l;
      return p;
    }
  }
  return 0;
}

char *parseUint32( char *s, uint32_t *u ) {
  unsigned long int l;
  char *p = s;
  if ( p && *p ) {
    l = strtoul( s, &p, 0 );
    if ( (p > s) && (l <= UINT32_MAX) ) {
      *u = (uint32_t) l;
      return p;
    }
  }
  return 0;
}

char *parseFlt( char *s, float *p ) {
  char *endptr = s;
  *p = strtof( s, &endptr );
  if (endptr > s) {
    return endptr;
  }
  return 0;
}

unsigned int parseUints( char *pc, unsigned int *p, unsigned int nmax ) {
  unsigned int n = 0;
  while( n < nmax && (pc = parseUint( pc, p ) ) ) {
    n++;
    p++;
  }
  return n;
}

unsigned int parseUint32s( char *pc, uint32_t *p, unsigned int nmax ) {
  unsigned int n = 0;
  while( n < nmax && (pc = parseUint32( pc, p ) ) ) {
    n++;
    p++;
  }
  return n;
}

unsigned int parseFlts( char *pc, float *p, unsigned int nmax ) {
  unsigned int n = 0;
  while( n < nmax && (pc = parseFlt( pc, p ) ) ) {
    n++;
    p++;
  }
  return n;
}

// parse each word as usecs, else as float seconds, convert to usecs
unsigned int parseUsecs( char *pc, uint32_t *p, unsigned int nmax ) {
  char *pc1 = pc;
  uint32_t ptemp = 0;
  float ftemp = 0;
  unsigned int n = 0;
  while( n < nmax && pc && pc[0] ) {
    pc1 = parseUint32(pc,&ptemp);
    if (pc1 && pc1[0] && isspace(pc1[0])) {
      *p++ = ptemp;
      n++;
      pc = pc1;
    }
    else if (parseFlt(pc,&ftemp)) {
      *p++ = (uint32_t) (ptemp * 1.E6);
      n++;
      pc = pc1;
    }
    else {
      break;
    }
  }
  return n;
}

  
/* ===================================================================
   Help text
 */
void help() {

  Serial.println("#Report device, version and configuration");
  Serial.println("#  version           - report software version");
  Serial.println("#  configuration     - report device configuration and data structure");
  Serial.println("#  pins              - report digital i/o functions and pin numbers");
  Serial.println("#");
  Serial.println("#Stop clocked and interrupt driven reads and the idler");
  Serial.println("#  stop              - stop clocks, triggers, gate.");
  Serial.println("#  stop idle[r]      - stop the idler.");
  Serial.println("#  stop all          - stop all, including the idler");
  Serial.println("#");
  Serial.println("#Coefficients for pixel number to wwavelength");
  Serial.println("#  store coefficients <a0> <a1> <a2> <a3> (need 4 numbers)");
  Serial.println("#  coefficients       - report");
  Serial.println("#");
  Serial.println("#  store units <string> (upto 6 characters, c.f. nm, um, mm)");
  Serial.println("#  units              - report");
  Serial.println("#");
  Serial.println("#Response function coefficients");
  Serial.println("#  store response <a0> <a1> <a2> <a3>");
  Serial.println("#  response       - report");
  Serial.println("#");
  Serial.println("#Save/recall dark and response spectra");
  Serial.println("#  save dark|resp      - save current buffer as dark or response");
  Serial.println("#  recall dark|resp    - and send it to the host");
  Serial.println("#");
  Serial.println("#  upload int|float    - followed by values one per line from the host");
  Serial.println("#");
  Serial.println("#  save filename       - save from current buffer to internal file");
  Serial.println("#  recall filename     - read from internal file to current buffer and send" );
  Serial.println("#");
  Serial.println("#Select current working buffer from the buffer ring");
  Serial.println("#  select buffer n     - select/report current buffer by number");
  Serial.println("#  send                - send contents of the current buffer");
  Serial.println("#");
  Serial.println("#Identifier string (63 bytes)");
  Serial.println("#  store identifier <identifier>");
  Serial.println("#  identifier         - list identifier string");
  Serial.println("#");
  Serial.println("#Data format");
  Serial.println("#  set ascii          - set data format to ascii");
  Serial.println("#  set binary         - set data format to binary");
  Serial.println("#  format             - report data form");
  Serial.println("#");
  Serial.println("#  send test          - send test data");
  Serial.println("#  send               - (re)send last data");
  Serial.println("#");
  Serial.println("#Microcontroller functions");
  Serial.println("#  temperature        - report microcontroller temperature");
  Serial.println("#  reboot             - reboots the entire board");
  Serial.println("#");
  Serial.println("#Read and average analog inputs");
  Serial.println("#  adcs <navgs>        - read analog inputs and report");
  Serial.println("#  set adcs <navgs>    - read ADCs at frame completion");
  Serial.println("#  set adcs off");
  Serial.println("#");
  Serial.println("#Read, manual loop, <shutter>, <frame> in usecs:");
  Serial.println("#  clock <shutter>               read one frame");    // legacy
  Serial.println("#  clock <n> <shutter>           read n frames");     // legacy
  Serial.println("#  clock <n> <shutter> <frame>   read n framns");     // legacy
  Serial.println("#");
  Serial.println("#Clock frames or sets of frames, with <shutter>, <frame>, and <outer> in usecs:");
  Serial.println("#  clock <shutter>               read one frame");    // legacy
  Serial.println("#  clock <n> <shutter>           read n frames");     // legacy
  Serial.println("#  clock <n> <shutter> <frame>   read n framns");     // legacy
  Serial.println("#  clock <m> <outer> <n> <shutter>");                 // new
  Serial.println("#  clock <m> <outer> <n> <shutter> <frame>");         // new
  Serial.println("#");
  Serial.println("#Trigger clocked frames:");
  Serial.println("#  trigger <shutter>                  trigger one frame");               // new
  Serial.println("#  trigger <n> <shutter>              trigger n times, single frames");  // legacy
  Serial.println("#  trigger <m> <n> <shutter>          trigger m times, n frames each, frame=shutter");  // legacy
  Serial.println("#  trigger <m> <n> <shutter> <frame>  trigger m sets of n frames");      // new
  Serial.println("#");
  Serial.println("#For gate frames (shutter opens and closes on change");
  Serial.println("#  gate <n>            - gate n frames, default to 1");
  Serial.println("#");	
  Serial.println("#  set trigger rising");
  Serial.println("#  set trigger falling");
  Serial.println("#  set trigger change");
  Serial.println("#");
  Serial.println("#  set trigger pullup           set trigger input to pullup");
  Serial.println("#  clear trigger pullup");
  Serial.println("#");
  Serial.println("#Latched frames, send only with increasing maximum");
  Serial.println("#  latch");
  Serial.println("#  latch <pixel>");
  Serial.println("#  clear latch");
  Serial.println("#");	
  Serial.println("#  set holdoff usecs");
  Serial.println("#  clear holdoff");
  Serial.println("#");	
  Serial.println("#Sync pin output at frame start, shutter or holdoff");	
  Serial.println("#  set sync shutter");
  Serial.println("#  set sync start");
  Serial.println("#  set sync holdoff [usecs]");
  Serial.println("#  set sync off");
  Serial.println("#  clear sync");
  Serial.println("#");
  Serial.println("#Pin control, sync and busy are toggled with the shutters");
  Serial.println("#  set spare|sync|busy hi");
  Serial.println("#  set spare|sync|busy lo");
  Serial.println("#  pulse spare|sync|busy [usecs]");
  Serial.println("#  toggle spare|sync|busy");
  Serial.println("#");
  Serial.println("#Idler");
  Serial.println("#  set idler off      - set idler off/auto/time spec");
  Serial.println("#  set idler auto");
  Serial.println("#  set idler <usecs>");
  Serial.println("#");

  Serial.println("#Pulse width modulation - spare pin");
  Serial.println("#  pwm <bits> <binary_value>");
  Serial.println("#  pwm off");
  Serial.println("#");
  
  Serial.println("#Preconfigured pins");
  Serial.print("#  Trigger(input)" );
  Serial.print( interruptPin );
  Serial.print("  Busy " );
  Serial.print( busyPin );
  Serial.print("  Sync " );
  Serial.print( syncPin );
  Serial.print("  Spare " );
  Serial.print( sparePin );
  Serial.println( "" );
}


/* ===================================================================
   The setup routine runs once when you press reset:
*/

void setup() {

  // initialize the digital pin as an output.
#ifdef HASLED
  pinMode(led, OUTPUT);
#endif

  // External Synch pins
  pinMode(interruptPin, INPUT);

  pinMode(busyPin, OUTPUT);
  digitalWriteFast(busyPin, busyPinState);

  pinMode(syncPin, OUTPUT);
  digitalWriteFast(syncPin, syncPinState);

  pinMode(sparePin, OUTPUT);
  digitalWriteFast(sparePin, sparePinState);

  pinMode(SHPin,     OUTPUT);
  digitalWriteFast(SHPin, LOW);
  
  pinMode(ICGPin,    OUTPUT);
  digitalWriteFast(ICGPin, HIGH);
  
  // Clock
  pinMode(fMPin,     OUTPUT);   
  digitalWriteFast(fMPin, LOW);

  pinMode(fMPinMonitor,     INPUT);

  
  analogWriteResolution(4);          // pwm range 4 bits, i.e. 2^4
  analogWriteFrequency(fMPin, fM);
  analogWrite(fMPin,8);              // dutycycle 50% for 2^4

  // ------------------------------
  pinMode(analogPin, INPUT);
  //  pinMode(analogPin, INPUT_PULLUP);
  adc->adc0->setReference(ADC_REFERENCE::REF_3V3); 
  adc->adc0->setAveraging(1);                 // set number of averages
  adc->adc0->setResolution(12);               // set bits of resolution
  adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED); 
  adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED); 
  adc->adc0->wait_for_cal();
  
  adc->adc0->singleMode();              // need this for the fast read

  // ------------------------------
  Serial.begin(9600);
  delay(100);

  // Patent pending and copyright notice displayed at startup
  Serial.println( versionstr );
  Serial.println( authorstr );
  //Serial.println( salesupportstr );

#ifdef DIAGNOSTICS_CPU 
  Serial.print("F_CPU: "); Serial.print(F_CPU/1e6);  Serial.println(" MHz."); 
  //Serial.print("F_BUS: "); Serial.print(F_BUS/1e6);  Serial.println(" MHz."); 
  Serial.print("ADC_F_BUS: "); Serial.print(ADC_F_BUS/1e6); Serial.println(" MHz.");
#endif

  // ------------------------------
  // File system in program RAM

  if (!filesys.begin( 1024*512 ) ) {
    Serial.println( "not able to setup filesys" );
  }
  else {
    filesys_ready = true;
    Serial.println("filesystem ready");
    listfiles();
  }
  
#ifdef HASLED
  blink();
  delay(1000);
  blink();
#endif

}

bool upload_int = false;
bool upload_float = false;
uint upload_counter = 0;

// the loop routine runs over and over again forever:
void loop() {

  uint16_t nlen = 0;
  char *pc;
  char c;
  
  unsigned int utemp = 0;
  float rtemp = 0.;

  /* ---------------------------------------------------
     Read serial input until endof line or ctl character
   */
  while ( Serial.available() ) {

    c = Serial.read();

    if ( c ) {

      // break at ctl-character or semi-colon
      if ( iscntrl( c ) || c == ';' ) {
        nlen = nrcvbuf;
        rcvbuffer[nrcvbuf] = 0;
        nrcvbuf = 0;
        break;
      }

      // skip leading spaces
      else if ( nrcvbuf || !isspace(c) ) {
        rcvbuffer[nrcvbuf++] = c;        
      }

      if ( nrcvbuf >= RCVLEN ) {
        Serial.println( (char *)"Error: buffer overflow" );
        nrcvbuf = 0;
      }
    }
    
#ifdef DIAGNOSTICS_RCV
    Serial.print( '#' );
    Serial.println( rcvbuffer );
#endif
  }

  /* ====================================================================
   * Command processing
   */

   if ( nlen > 0 ) {
    
     //blink();

     for ( int n = 0; (n < RCVLEN) && rcvbuffer[n] ; n++ ) {
       rcvbuffer[n] = tolower( rcvbuffer[n] );
     }
     pc = rcvbuffer;

     Serial.println( pc );
     
    /* -------------------------------------------------------------
       Firmware version identification
     */
    if ( (pc = startsWith( rcvbuffer, "version" )) ) {
      Serial.println( versionstr );
      Serial.println( authorstr );
      //Serial.println( salesupportstr );
    }

    else if ( startsWith( rcvbuffer, "reboot") ) {
      _reboot_Teensyduino_();
    }

    else if ( (pc = startsWith( rcvbuffer, "set diagnostics" )) ) {
      diagnostics = true;
    }
    else if ( (pc = startsWith( rcvbuffer, "clear diagnostics" )) ) {
      diagnostics = false;
    }

    else if ( (pc = startsWith( rcvbuffer, "help" )) ) {
      help();
    }

    else if ( (pc = startsWith( rcvbuffer, "test timer" )) ) {
      timertest();
    }
    /* -----------------------------------------------------------
       Stop command
    */
    else if ( (pc = startsWith( rcvbuffer, "stop" )) ) {

      stop_all();
      
    }

    else if ( (pc = startsWith( rcvbuffer, "dump" )) ) {
      dumpAll();
    }

    /* -----------------------------------------------------------
       Set digital output pin
     */
    
    else if ( ( (pc = startsWith( rcvbuffer, "set spare" )) ||
		(pc = startsWith( rcvbuffer, "toggle spare" )) )
	      && spare_pwm ) {
      Serial.println( "Error: spare pin in use for pwm." );
    }
    
    else if ( (pc = startsWith( rcvbuffer, "set spare hi" )) ) {
      digitalWriteFast(sparePin, HIGH);
      sparePinState = HIGH;
    }      
    
    else if ( (pc = startsWith( rcvbuffer, "set spare lo" )) ) {
      digitalWriteFast(sparePin, LOW);
      sparePinState = LOW;
    }      
    
    else if ( (pc = startsWith( rcvbuffer, "pulse spare" )) ) {
      
      utemp = 1;
      parseUint( pc, &utemp );

      pulsePin( sparePin, utemp );     
    }

    else if ( (pc = startsWith( rcvbuffer, "toggle spare" )) ) {
      digitalToggleFast(sparePin);
      sparePinState = !sparePinState;
    }

    else if ( is_running ) {
      Serial.println( "Error: is running, send the stop command" );
    }

    /* -----------------------------------------------------------
       Set busy pin (we toggle it in the shutter routines)
     */
    
    else if ( (pc = startsWith( rcvbuffer, "set busy hi" )) ) {
      digitalWriteFast(busyPin, HIGH);
      busyPinState = HIGH;
    }      
    
    else if ( (pc = startsWith( rcvbuffer, "set busy lo" )) ) {
      digitalWriteFast(busyPin, LOW);
      busyPinState = LOW;
    }      
    
    else if ( (pc = startsWith( rcvbuffer, "pulse busy" )) ) {
      
      utemp = 1;
      parseUint( pc, &utemp );

      pulsePin( busyPin, utemp );     
    }

    else if ( (pc = startsWith( rcvbuffer, "toggle busy" )) ) {
      digitalToggleFast(busyPin);
      busyPinState = !busyPinState;
    }

    /* -----------------------------------------------------------
       Set sync pin (we toggle it in the shutter routines)
     */
    
    else if ( (pc = startsWith( rcvbuffer, "set sync hi" )) ) {
      digitalWriteFast(syncPin, HIGH);
      syncPinState = HIGH;
    }      
    
    else if ( (pc = startsWith( rcvbuffer, "set sync lo" )) ) {
      digitalWriteFast(syncPin, LOW);
      syncPinState = LOW;
    }      
    
    else if ( (pc = startsWith( rcvbuffer, "pulse sync" )) ) {
      
      utemp = 1;
      parseUint( pc, &utemp );

      pulsePin( syncPin, utemp );     
    }

    else if ( (pc = startsWith( rcvbuffer, "toggle sync" )) ) {
      digitalToggleFast(syncPin);
      syncPinState = !syncPinState;
    }

    /* =================================================================
       Check if active, stop required for further commands
    */
    else if (is_busy) {
      Serial.println( "Error: device active, need stop before other commands" );
    }
    
    /* =================================================================
       The host program queries this to setup its buffers
    */    
    else if ( (pc = startsWith( rcvbuffer, "configuration" )) ) {

      /*
      sprintf( sndbuffer, "PIXELS %u DARK %u VPERBIT %.8f",
	       NPIXELS, NDARK, 3.3/4096 );
      Serial.println( sndbuffer );
      */
      
      sprintf( sndbuffer, "PIXELS %u DARK %u BITS %u VFS %f",
	       NPIXELS, NDARK, NBITS, VFS );
      Serial.print( sndbuffer );

      Serial.print( " SENSOR " );
      Serial.println( sensorstr );
      
#ifdef DIAGNOSTICS_CPU
      Serial.print("F_CPU: ");
      Serial.print(F_CPU/1e6);
      Serial.println(" MHz."); 

      Serial.print("ADC_F_BUS: ");
      Serial.print(ADC_F_BUS/1e6);
      Serial.println(" MHz.");
#endif
      
    }

    else if ( (pc = startsWith( rcvbuffer, "pins" )) ) {

      Serial.println("#Pins");
      Serial.print("#  Trigger(input)" );
      Serial.print( interruptPin );
      Serial.print("  Busy " );
      Serial.print( busyPin );
      Serial.print("  Sync " );
      Serial.print( syncPin );
      Serial.print("  Spare " );
      Serial.print( sparePin );
      Serial.println( "" );

    }
    
    /* -----------------------------------------------------------
       User store/print coefficients
    */
    else if ( (pc = startsWith( rcvbuffer, "store coefficients" )) || (pc = startsWith( rcvbuffer, "set coefficients" )) ) {
      float vals[EEPROM_NCOEFFS] = {0};
      if (parseFlts( pc, vals, EEPROM_NCOEFFS)) {
	storeCoefficients(vals);
      }      
      printCoefficients( );      
    }

    else if ( (pc = startsWith( rcvbuffer, "coefficients" )) ) {
      printCoefficients( );      
    }
    
    else if ( (pc = startsWith( rcvbuffer, "erase coefficients" )) ) {
      eraseCoefficients();
    }
    
    /* -----------------------------------------------------------
       User store/print response
    */
    else if ( (pc = startsWith( rcvbuffer, "store response" )) || (pc = startsWith( rcvbuffer, "set response" )) ) {
      float vals[EEPROM_NRESPS] = {0};
      if (parseFlts(pc, vals, EEPROM_NRESPS)) {
	storeResponse(vals);
      }      
      printResponse( );      
    }

    else if ( (pc = startsWith( rcvbuffer, "response" )) ) {
      printResponse( );      
    }
    
    else if ( (pc = startsWith( rcvbuffer, "erase response" )) ) {
      eraseResponse();
    }
    
    /* -----------------------------------------------------------
       User store/print identifier
    */

    else if ( (pc = startsWith( rcvbuffer, "store identifier" )) || (pc = startsWith( rcvbuffer, "set identifier" )) ) {
      while( *pc && isspace(*pc) ) pc++;
      storeIdentifier( pc );      
      printIdentifier( );      
    }
    
    else if ( (pc = startsWith( rcvbuffer, "identifier" )) ) {
      printIdentifier( );      
    }
    
    else if ( (pc = startsWith( rcvbuffer, "erase identifier" )) ) {
      eraseIdentifier();
    }
    
    /* -----------------------------------------------------------
       User store/print units
    */

    else if ( (pc = startsWith( rcvbuffer, "store units" )) || (pc = startsWith( rcvbuffer, "set units" )) ) {
      while( *pc && isspace(*pc) ) pc++;
      storeUnits( pc );      
      printUnits( );      
    }
    
    else if ( (pc = startsWith( rcvbuffer, "units" )) ) {
      printUnits( );      
    }
    
    else if ( (pc = startsWith( rcvbuffer, "erase units" )) ) {
      eraseUnits();
    }
    
    /* -----------------------------------------------------------
       Data output format
     */
    
    else if ( (pc = startsWith( rcvbuffer, "set ascii" )) || (pc = startsWith( rcvbuffer, "set formatted" )) ) {
      dataformat = ASCII;
      Serial.println( "Set format to ascii" );
    }

    else if ( (pc = startsWith( rcvbuffer, "set binary" )) ) {
      dataformat = BINARY;
      Serial.println( "Set format to binary" );
    }
    
    else if ( (pc = startsWith( rcvbuffer, "format" )) ) {
      if ( dataformat == BINARY ) {
	Serial.println( "format binary" );
      }
      else if ( dataformat == ASCII ) {
	Serial.println( "format ascii" );
      }
      else {
	Serial.println( "format unknown" );
      }
    }
    
    /* =================================================================
     * Support for dark and response spectra
     */
    
     else if (upload_int && upload_counter < NPIXELS && parseUint( pc, &utemp ) ) {
       if ( utemp < (1<<NBITS) ) {
	 bufferp[DATASTART+upload_counter] = utemp;
	 upload_counter++;
	 if (upload_counter == NPIXELS) {
	   upload_int = false;
	 }
       }
       else {
	 Serial.println( "Error: upload with value out of range." );
       }
     }

     else if (upload_float && upload_counter < NPIXELS && parseFlt( pc, &rtemp ) ) {
       if ( rtemp >= 0.&& rtemp < VFS ) {
	 bufferp[DATASTART+upload_counter] = (uint16_t) ( (1<<NBITS) * (rtemp/VFS) );
	 upload_counter++;
	 if (upload_counter == NPIXELS) {
	   upload_float = false;
	 }
       }
       else {
	 Serial.println( "Error: upload with value out of range." );
       }
     }
     
     else if ( (pc = startsWith( rcvbuffer, "upload int" )) ) {
       memset( &bufferp[NREADOUT], 0, NREADOUT * 2 );
       upload_int = true;
       upload_counter = 0;
     }
     
     else if ( (pc = startsWith( rcvbuffer, "upload float" )) ) {
       memset( &bufferp[NREADOUT], 0, NREADOUT * 2 );
       upload_float = true;
       upload_counter = 0;
     }
     
     else if ( (pc = startsWith( rcvbuffer, "save dark" )) ) {
       writetofile( (char *) "dark.txt" );
     }

     else if ( (pc = startsWith( rcvbuffer, "recall dark" )) ) {
       if ( readfromfile( (char *) "dark.txt" ) ) {
	 Serial.println( "DARK" );
	 sendData();
       }
     }
    
     else if ( (pc = startsWith( rcvbuffer, "save resp" )) ) {
       writetofile( (char *) "resp.txt" );
     }
    
     else if ( (pc = startsWith( rcvbuffer, "recall resp" )) ) {
       if ( readfromfile( (char *) "resp.txt" ) ) {
	 Serial.println( "RESPONSE" );
	 sendData();
       }
     }

     else if ( (pc = startsWith( rcvbuffer, "save" )) ) {
       while( pc && isspace(*pc) ) pc++;
       if ( pc && *pc ) {
	 writetofile( (char *) pc );
       }
     }
    
     else if ( (pc = startsWith( rcvbuffer, "recall" )) ) {
       while( pc && isspace(*pc) ) pc++;
       if ( readfromfile( (char *) pc ) ) {
	 Serial.print( "RECALLED " );
	 Serial.println( pc );
	 sendData();
       }
     }
    
     else if ( (pc = startsWith( rcvbuffer, "quickformat" )) ) {
       filesys.quickFormat();
       listfiles();
     }
    
     else if ( (pc = startsWith( rcvbuffer, "list files" )) ) {
       listfiles();
     }
    
     else if ( (pc = startsWith( rcvbuffer, "remove" )) ) {
       while( pc && isspace(*pc) ) pc++;
       filesys.remove(pc);
       listfiles();
     }
    
     else if ( (pc = startsWith( rcvbuffer, "send test" )) ) {    
      sendTestData( );
     }

     else if ( (pc = startsWith( rcvbuffer, "send" )) ) {    
      sendData( );
     }

     else if ( (pc = startsWith( rcvbuffer, "select buffer" )) ) {
    
       if ( (pc = parseUint( pc, &utemp )) ){
	 selectbuffer( utemp );
       }
       
       Serial.print( "using buffer " );
       Serial.print( buffer_index );
       Serial.print( " / " );
       Serial.println( NBUFFERS );
    
     }
    
    /* =================================================================
       Idler - clocks the shift gate
     */

    else if ( (pc = startsWith( rcvbuffer, "set idler off")) ) {
      Serial.println( "setting idler off" );
      LCCDIdlerStop();
      idlerAuto = false;
    }
    
    else if ( (pc = startsWith( rcvbuffer, "set idler auto")) ) {
      Serial.println( "setting idler auto" );
      idlerAuto = true;      
    }
    
    else if ( (pc = startsWith( rcvbuffer, "set idler")) && (pc = parseUint32( pc, &idlerUsecs )) ) {

      Serial.print( "setting idler " );
      Serial.println( idlerUsecs );	
      LCCDIdlerStart();
      
    }

    /* -----------------------------------------------------------
       ADC reporting
    */
    else if ( (pc = startsWith( rcvbuffer, "set adcs off")) ) {
      Serial.println( "setting adc reporting off" );
      adc_averages = 0;
    }
    
    else if ( (pc = startsWith( rcvbuffer, "set adcs")) && (pc = parseUint( pc, &adc_averages )) ) {
      Serial.print( "setting adc reporting, averaging " );
      Serial.println( adc_averages );
    }
    
    else if ( (pc = startsWith( rcvbuffer, "adcs")) || (pc = startsWith( rcvbuffer, "read analog inputs")) ) {
      utemp = 1;
      parseUint( pc, &utemp );
      if (utemp) {
	sendADCs( utemp );
      }
    }

    /* ------------------------------------------------------------
       Temperature reporting
     */
    else if ( (pc = startsWith( rcvbuffer, "set temperature off")) ) {
      Serial.println( "setting chip temperature reporting off" );
      chipTemp_averages = 0;
    }
    
    else if ( (pc = startsWith( rcvbuffer, "set temperature on")) ) {
      Serial.println( "setting chip temperature reporting on" );
      chipTemp_averages = 1;
    }
    
    else if ( !chipTemp_averages && (pc = startsWith( rcvbuffer, "temperature" )) ) {
      Serial.print( "CHIPTEMPERATURE " );
      Serial.println( tempmonGetTemp() );
    }
    
    /* -----------------------------------------------------------
       Manual reads
    */
    else if ( (pc = startsWith( rcvbuffer, "read")) ||
	      (pc = startsWith( rcvbuffer, "read sensor")) ) {


      unsigned int nargs = 0;

      uint32_t uvals[5] = { 0 };

      // Complete in one set
      outerCounter = 0;
      outerCounts = 1;
      outerUsecs  = 0;
      
      // Now we parse and launch
      nargs = parseUint32s( pc, uvals, 3 );

      if (nargs==0) {
	frameCounts = 1;
      }

      else if (nargs==1) {
	frameCounts = 1;
	shutterUsecs = uvals[0];
	frameUsecs = shutterUsecs;
      }
      else if (nargs==2) {
	frameCounts = uvals[0];
	shutterUsecs = uvals[1];
	frameUsecs = shutterUsecs;
      }
      else if (nargs==3) {
	frameCounts = uvals[0];
	shutterUsecs = uvals[1];
	frameUsecs = uvals[2];
      }

      if (!idlerRunning) {
	quickClean(100, 3);
      }
      
      manualFrames_launcher();
    }
    
    /* -----------------------------------------------------------
       Clocked reads
    */
    else if ( (pc = startsWith( rcvbuffer, "clock")) ||
	      (pc = startsWith( rcvbuffer, "clock sensor")) ) {

      unsigned int nargs = 0;
      uint32_t uvals[5] = { 0 };

      // Default, complete in one set
      outerCounter = 0;
      outerCounts  = 1;
      outerUsecs   = 0;

      // Now we parse and launch
      nargs = parseUint32s( pc, uvals, 5 );

      // One frame, just do it
      if (nargs==0) {
	frameCounts = 1;
	manualFrames_launcher();
      }

      // Setup the megillah
      else {
	if (nargs==1) {
	  frameCounts = 1;
	  shutterUsecs = uvals[0];
	  frameUsecs = shutterUsecs;
	}
	else if (nargs==2) {
	  frameCounts = uvals[0];
	  shutterUsecs = uvals[1];
	  frameUsecs = shutterUsecs;
	}
	else if (nargs==3) {
	  frameCounts = uvals[0];
	  shutterUsecs = uvals[1];
	  frameUsecs = uvals[2];
	}
	// Here we have the outer clock specified
	else if (nargs==4) {
	  outerCounts = uvals[0];
	  outerUsecs  = uvals[1];
	  frameCounts = uvals[2];
	  shutterUsecs = uvals[3];
	  frameUsecs = shutterUsecs;
	}
	else if (nargs==5) {
	  outerCounts = uvals[0];
	  outerUsecs  = uvals[1];
	  frameCounts = uvals[2];
	  shutterUsecs = uvals[3];
	  frameUsecs = uvals[4];
	}

	if (!idlerRunning) {
	  quickClean(100, 3);
	}
	
	outerClock_launcher();
      }

    }
    
    /* -----------------------------------------------------------
       Triggered clocked reads
    */
        
    else if ( (pc = startsWith( rcvbuffer, "trigger")) ) {

      unsigned int nargs = 0;
      uint32_t uvals[4] = { 0 };

      // Default, complete in one set
      outerCounter = 0;
      outerCounts  = 1;
      outerUsecs   = 0;

      // parse and launch
      nargs = parseUint32s( pc, uvals,  4 );

      if (nargs < 4 && !idlerRunning) {
	quickClean(100, 3);
      }
	
      // one frame, use previous shutter setting
      if (nargs==0) {
	frameCounts = 1;
	triggeredSingleFrame_launcher();
      }
      // one frame with shutter specified
      else if (nargs==1) {
	frameCounts  = 1;
	shutterUsecs = uvals[0];
	frameUsecs   = shutterUsecs;
	triggeredSingleFrame_launcher();
      }
      // set, trigger each frame
      else if (nargs==2) {
	frameCounts  = uvals[0];
	shutterUsecs = uvals[1];
	frameUsecs   = shutterUsecs;
	triggeredSingleFrame_launcher();
      }
      // trigger clocked frame sets
      else if (nargs==3) {
	outerCounts  = uvals[0];
	frameCounts  = uvals[1];
	shutterUsecs = uvals[2];
	frameUsecs   = shutterUsecs;
	outerTrigger_launcher();
      }
      // as above, with shutter < frame period
      else if (nargs==4) {
	outerCounts  = uvals[0];
	frameCounts  = uvals[1];
	shutterUsecs = uvals[2];
	frameUsecs   = uvals[3];
	outerTrigger_launcher();
      }
      else {
	Serial.println( (char *)"Error: trigger command with too many parameters");
      }

    }
    
    /* -----------------------------------------------------------
       Gated read
    */
    else if ( (pc = startsWith( rcvbuffer, "gate")) ) {

      unsigned int u1;

      // Default, complete in one set
      outerCounter = 0;
      outerCounts  = 1;
      outerUsecs   = 0;

      if (!idlerRunning) {
	quickClean(100, 3);
      }
      
      if ( parseUint( pc, &u1 ) ) {

	frameCounts = u1;
	gatedFrameset_start();
	
      }
      else {
	Serial.println( "Error: syntax:  gate nframes" );
      }
    }

    /* -----------------------------------------------------------
       Trigger edge selection
    */
    
    else if ( (pc = startsWith( rcvbuffer, "set trigger rising")) ) {
      intedgemode = RISING;
    }

    else if ( (pc = startsWith( rcvbuffer, "set trigger falling")) ) {
      intedgemode = FALLING;
    }
    
    else if ( (pc = startsWith( rcvbuffer, "set trigger change")) ) {
      intedgemode = CHANGE;
    }

    else if ( (pc = startsWith( rcvbuffer, "set trigger pullup")) ) {
      pinMode(interruptPin, INPUT_PULLUP);
    }
    else if ( (pc = startsWith( rcvbuffer, "set trigger nopullup")) ||
	      (pc = startsWith( rcvbuffer, "clear trigger pullup"))
	      ) {
      pinMode(interruptPin, INPUT);
    }

    /* ===============================================================
       Set latch
    */
    else if ( (pc = startsWith( rcvbuffer, "clear latch")) ) {
      do_latch = false;
      latch_value = 0;
      Serial.println( "latch cleared" );      
    }
    
    else if ( (pc = startsWith( rcvbuffer, "latch")) ) {

      unsigned int u1 = 0;

      // specify pixel
      if ( parseUint( pc, &u1 )  ) {
	if ( u1 < NPIXELS ) {
	  latch_value = 0;
	  latch_pixel = u1;
	  do_latch = true;
	  
	  Serial.print( "latch on pixel " );
	  Serial.println( latch_pixel );
	}
	else {
	  Serial.println( "Error: latch pixel out of range." );
	}
      }

      // latch anything
      else {
	  latch_value = 0;
	  latch_pixel = 0;
	  do_latch = true;
	  
	  Serial.println( "latch on max" );
      }
      
    }    
    
    /* ===============================================================
       Holdoff
    */
    else if ( (pc = startsWith( rcvbuffer, "set holdoff")) ) {
      parseUint32( pc, &holdoff_usecs );
      Serial.print( "holdoff " );
      Serial.println( holdoff_usecs );
    }

    else if ( (pc = startsWith( rcvbuffer, "clear holdoff")) ) {
      holdoff_usecs = 0;
      if (do_sync_holdoff) {
	do_sync_start = true;
	do_sync_holdoff = false;	
      }
      Serial.print( "holdoff " );
      Serial.println( holdoff_usecs );
    }
    
    /* ===============================================================
       Sync pin
    */
    else if ( (pc = startsWith( rcvbuffer, "set sync off")) ||
	      (pc = startsWith( rcvbuffer, "clear sync")) ) {
      Serial.println( "setting sync off" );
      do_sync_start = false;
      do_sync_shutter = false;
      do_sync_holdoff = false;
    }
    
    else if ( (pc = startsWith( rcvbuffer, "set sync shutter")) ) {
      Serial.println( "setting sync shutter" );
      do_sync_start = false;
      do_sync_shutter = true;
      do_sync_holdoff = false;
    }
    
    else if ( (pc = startsWith( rcvbuffer, "set sync start")) ) {
      Serial.println( "setting sync start" );
      do_sync_start = true;
      do_sync_shutter = false;
      do_sync_holdoff = false;
    }
    
    else if ( (pc = startsWith( rcvbuffer, "set sync holdoff")) ) {
      parseUint32( pc, &holdoff_usecs );
      do_sync_start = true;
      do_sync_shutter = false;
      do_sync_holdoff = true;
      Serial.print( "setting sync holdoff " );
      Serial.print( do_sync_holdoff );
      Serial.print( " usecs " );
      Serial.println( holdoff_usecs );
    }

    else if ( (pc = startsWith( rcvbuffer, "set sync hi")) ) {
      Serial.println( "setting sync high" );
      digitalWriteFast(syncPin, HIGH);
    }
    
    else if ( (pc = startsWith( rcvbuffer, "set sync lo")) ) {
      Serial.println( "setting sync high" );
      digitalWriteFast(syncPin, LOW);
    }
    
   else if ( (pc = startsWith( rcvbuffer, "pulse sync" )) ) {
     utemp = 1;
     parseUint( pc, &utemp );
     digitalToggleFast(syncPin);
     delayMicroseconds(utemp);
     digitalToggleFast(syncPin);
   }      

   else if ( (pc = startsWith( rcvbuffer, "toggle sync" )) ) {
     digitalToggleFast(syncPin);
   }      
     
    /* -----------------------------------------------------------
       Test the ADC, benchmark
    */
    else if ( (pc = startsWith( rcvbuffer, "test adc")) ) {
      unsigned int usecs;
     
      utemp = 1;
     
      parseUint( pc, &utemp );
     
      Serial.print( "command: test adc " );
      Serial.println( utemp );
       
      usecs = measureADCspeed( analogPin, utemp );

      Serial.print( "elapsed time " );
      Serial.println( usecs );
    }

    /* -----------------------------------------------------------
       PWM
    */
    else if ( (pc = startsWith( rcvbuffer, "pwm off" )) ) {
      sparePWM_stop();
    }

    else if ( (pc = startsWith( rcvbuffer, "pwm" )) ) {

      unsigned int u1, u2;

      if ( (pc = parseUint( pc, &u1 ) ) && (pc = parseUint( pc, &u2 ) )
	   && u1 > 1 && u1 < 16 && u2 < (unsigned int) (1<<u1) ) {

	pwm_resolution = u1;
	pwm_value = u2;

	sparePWM_start();
      }
      else {
	Serial.println( "Error: syntax:  need pwm off or <bits> <value>" );
      }
      

    }
    /* -----------------------------------------------------------
       Diagnostics, send selected buffer
     */
    
    else if ( (pc = startsWith( rcvbuffer, "send" )) ) {
    
      unsigned int usecs;
          
      elapsed_usecs = 0;
     
      sendData( );
     
      usecs = elapsed_usecs;
     
      Serial.print( "elapsed usecs " );
      Serial.println( usecs );    
    }

    /*
    else if ( (pc = startsWith( rcvbuffer, "test clock")) ) {

      (pc = parseUint( pc, &test_u1 ) ) && (pc = parseUint( pc, &test_u2 ) );
      
      testclock();
    }
    */
    
    else {
      Serial.print( "Error: unknown command //" );
      Serial.print( (char *) rcvbuffer );
      Serial.println( "//" );
      
    }

   // Indicate that we processed this message
   nlen = 0;

   Serial.println( "DONE" );
  }
  
}
