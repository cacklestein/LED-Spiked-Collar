/*-------------------------------------------------------------------------
  Arduino library to control a wide variety of WS2811- and WS2812-based RGB
  LED devices such as Adafruit FLORA RGB Smart Pixels and NeoPixel strips.
  Currently handles 400 and 800 KHz bitstreams on 8, 12 and 16 MHz ATmega
  MCUs, with LEDs wired for RGB or GRB color order.  8 MHz MCUs provide
  output on PORTB and PORTD, while 16 MHz chips can handle most output pins
  (possible exception with upper PORT registers on the Arduino Mega).

  Written by Phil Burgess / Paint Your Dragon for Adafruit Industries,
  contributions by PJRC and other members of the open source community.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing products
  from Adafruit!

  -------------------------------------------------------------------------
  This file is part of the Adafruit NeoPixel library.

  NeoPixel is free software: you can redistribute it and/or modify
  it under the terms of the GNU Lesser General Public License as
  published by the Free Software Foundation, either version 3 of
  the License, or (at your option) any later version.

  NeoPixel is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with NeoPixel.  If not, see
  <http://www.gnu.org/licenses/>.
  -------------------------------------------------------------------------*/

#include "Adafruit_NeoPixel.h"

Adafruit_NeoPixel::Adafruit_NeoPixel(uint16_t n, uint8_t p, uint8_t t, uint16_t m)
  : numLEDs(n)
  , numBytes(n * 3)
  , maxCurrent(m)
  , pin(p)
  , pixels(NULL)
  , type(t)
  , brightness(NEO_MAX_BRIGHTNESS)
  , endTime(0)
#ifdef __AVR__
  ,port(portOutputRegister(digitalPinToPort(p))),
   pinMask(digitalPinToBitMask(p))
#endif
{
  if((pixels = (uint8_t *)malloc(numBytes))) {
    memset(pixels, 0, numBytes);
  }
  if(t & NEO_GRB) { // GRB vs RGB; might add others if needed
    rOffset = 1;
    gOffset = 0;
    bOffset = 2;
  } else if (t & NEO_BRG) {
    rOffset = 1;
    gOffset = 2;
    bOffset = 0;
  } else {
    rOffset = 0;
    gOffset = 1;
    bOffset = 2;
  }
  
}

Adafruit_NeoPixel::~Adafruit_NeoPixel() {
  if(pixels) free(pixels);
  pinMode(pin, INPUT);
}

void Adafruit_NeoPixel::begin(void) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
}

uint16_t Adafruit_NeoPixel::calculateCurrent(bool scale) {
    // The current consumption of a single WS2812B has been measured
    // to be I(mA) = 1 + R/16 + G/16 + B/16
    uint16_t current = 0;
    uint8_t* ptr = pixels;
    for (int i = 0; i < numBytes; ++i) {
        current += *(ptr++);
    }
    current >>= 4;

    if (scale) {
        current *= brightness;
        current >>= 5;
    }

    return current + numLEDs;
}

void Adafruit_NeoPixel::adjustBrightnessForMaxCurrent(void) {
    if (maxCurrent) {
        uint16_t current = calculateCurrent(false);
        if (current > maxCurrent) {
            setBrightness(maxCurrent * NEO_MAX_BRIGHTNESS / current);
        }
    }
}

void Adafruit_NeoPixel::show(void) {

  if(!pixels) return;

  if (maxCurrent) adjustBrightnessForMaxCurrent();  

  // Data latch = 50+ microsecond pause in the output stream.  Rather than
  // put a delay at the end of the function, the ending time is noted and
  // the function will simply hold off (if needed) on issuing the
  // subsequent round of data until the latch time has elapsed.  This
  // allows the mainline code to start generating the next frame of data
  // rather than stalling for the latch.
  while(!canShow());
  // endTime is a private member (rather than global var) so that mutliple
  // instances on different pins can be quickly issued in succession (each
  // instance doesn't delay the next).

  // In order to make this code runtime-configurable to work with any pin,
  // SBI/CBI instructions are eschewed in favor of full PORT writes via the
  // OUT or ST instructions.  It relies on two facts: that peripheral
  // functions (such as PWM) take precedence on output pins, so our PORT-
  // wide writes won't interfere, and that interrupts are globally disabled
  // while data is being issued to the LEDs, so no other code will be
  // accessing the PORT.  The code takes an initial 'snapshot' of the PORT
  // state, computes 'pin high' and 'pin low' values, and writes these back
  // to the PORT register as needed.

  noInterrupts(); // Need 100% focus on instruction timing

#ifdef __AVR__
  volatile uint16_t
    i   = numBytes; // Loop counter
  volatile uint8_t
   *ptr = pixels,   // Pointer to next byte
    b,              // Current byte (scaled) value
    hi,             // PORT w/output bit set high
    lo,             // PORT w/output bit set low
    multa,          // Next byte (unscaled) value
    multb,          // Scale value
    multc;          // Next byte (scaled) value

    multa = *ptr++;
    multb = brightness;
    // Perform scaling for the first byte here
    multc = (uint8_t)((((uint16_t)multa) * multb) >> 5);

  // Hand-tuned assembly code issues data to the LED drivers at a specific
  // rate.  There's separate code for different CPU speeds (8, 12, 16 MHz)
  // for both the WS2811 (400 KHz) and WS2812 (800 KHz) drivers.  The
  // datastream timing for the LED drivers allows a little wiggle room each
  // way (listed in the datasheets), so the conditions for compiling each
  // case are set up for a range of frequencies rather than just the exact
  // 8, 12 or 16 MHz values, permitting use with some close-but-not-spot-on
  // devices (e.g. 16.5 MHz DigiSpark).  The ranges were arrived at based
  // on the datasheet figures and have not been extensively tested outside
  // the canonical 8/12/16 MHz speeds; there's no guarantee these will work
  // close to the extremes (or possibly they could be pushed further).
  // Keep in mind only one CPU speed case actually gets compiled; the
  // resulting program isn't as massive as it might look from source here.

// 8 MHz(ish) AVR ---------------------------------------------------------
#if (F_CPU >= 7400000UL) && (F_CPU <= 9500000UL)

#ifdef NEO_KHZ400
  if((type & NEO_SPDMASK) == NEO_KHZ800) { // 800 KHz bitstream
#endif

    volatile uint8_t n1, n2 = 0;  // First, next bits out

    // Squeezing an 800 KHz stream out of an 8 MHz chip requires code
    // specific to each PORT register.  At present this is only written
    // to work with pins on PORTD or PORTB, the most likely use case --
    // this covers all the pins on the Adafruit Flora and the bulk of
    // digital pins on the Arduino Pro 8 MHz (keep in mind, this code
    // doesn't even get compiled for 16 MHz boards like the Uno, Mega,
    // Leonardo, etc., so don't bother extending this out of hand).
    // Additional PORTs could be added if you really need them, just
    // duplicate the else and loop and change the PORT.  Each add'l
    // PORT will require about 150(ish) bytes of program space.

    // 10 instruction clocks per bit: HHxxxxxLLL
    // OUT instructions:              ^ ^    ^   (T=0,2,7)

      hi = PORTB |  pinMask;
      lo = PORTB & ~pinMask;
      n1 = lo;
      if (multc & 0x80) n1 = hi;

      asm volatile(
       "headB:"                   "\n\t" // Clk  Pseudocode
        // Bit 7:
        "out  %[port] , %[hi]"    "\n\t" // 1    PORT = hi
        "mov  %[n2]   , %[lo]"    "\n\t" // 1    n2   = lo
        "out  %[port] , %[n1]"    "\n\t" // 1    PORT = n1
        "mov  %[byte] , %[multc]" "\n\t" // 1    b = multc
        "clr  %[multc]"           "\n\t" // 1    multc = 0
        "sbrc %[byte] , 6"        "\n\t" // 1-2  if(b & 0x40)
         "mov %[n2]   , %[hi]"    "\n\t" // 0-1   n2 = hi
        "out  %[port] , %[lo]"    "\n\t" // 1    PORT = lo
        "ld   %[multa], %a[ptr]+" "\n\t" // 2    multa = *ptr++
        // Bit 6:
        "out  %[port] , %[hi]"    "\n\t" // 1    PORT = hi
        "mov  %[n1]   , %[lo]"    "\n\t" // 1    n1   = lo
        "out  %[port] , %[n2]"    "\n\t" // 1    PORT = n2
        "sbrc %[multb], 5"        "\n\t" // 1-2  if(multb & 0x20)
         "mov  %[multc], %[multa]""\n\t" // 0-1   multc = multa
        "sbrc %[byte] , 5"        "\n\t" // 1-2  if(b & 0x20)
         "mov %[n1]   , %[hi]"    "\n\t" // 0-1   n1 = hi
        "out  %[port] , %[lo]"    "\n\t" // 1    PORT = lo
        "lsr  %[multa]"           "\n\t" // 1    multa >>= 1
        "nop"                     "\n\t" // 1    nop
        // Bit 5:
        "out  %[port] , %[hi]"    "\n\t" // 1    PORT = hi
        "mov  %[n2]   , %[lo]"    "\n\t" // 1    n2   = lo
        "out  %[port] , %[n1]"    "\n\t" // 1    PORT = n1
        "sbrc %[multb], 4"        "\n\t" // 1-2  if(multb & 0x10)
         "adc  %[multc], %[multa]""\n\t" // 0-1   multc += multa (+ carry)
        "sbrc %[byte] , 4"        "\n\t" // 1-2  if(b & 0x10)
         "mov %[n2]   , %[hi]"    "\n\t" // 0-1   n2 = hi
        "out  %[port] , %[lo]"    "\n\t" // 1    PORT = lo
        "rjmp bit4B"              "\n\t" // 2    jump bit4B
       "bit1B:"                   "\n\t"
        // Bit 1:
        "out  %[port] , %[hi]"    "\n\t" // 1    PORT = hi
        "mov  %[n2]   , %[lo]"    "\n\t" // 1    n2   = lo
        "out  %[port] , %[n1]"    "\n\t" // 1    PORT = n1
        "lsr  %[multa]"           "\n\t" // 1    multa >>= 1
        "nop"                     "\n\t" // 1    nop
        "sbrc %[byte] , 0"        "\n\t" // 1-2  if(b & 0x01)
         "mov %[n2]   , %[hi]"    "\n\t" // 0-1   n2 = hi
        "out  %[port] , %[lo]"    "\n\t" // 1    PORT = lo
        "sbrc %[multb], 0"        "\n\t" // 1-2  if(multb & 0x01)
         "adc %[multc], %[multa]" "\n\t" // 0-1   multc += multa (+ carry)
        // Bit 0:
        "out  %[port] , %[hi]"    "\n\t" // 1    PORT = hi
        "mov  %[n1]   , %[lo]"    "\n\t" // 1    n1   = lo
        "out  %[port] , %[n2]"    "\n\t" // 1    PORT = n2
        "sbiw %[count], 1"        "\n\t" // 2    i-- (don't act on Z flag yet)
        "sbrc %[multc] , 7"       "\n\t" // 1-2  if(multc & 0x80)
         "mov %[n1]   , %[hi]"    "\n\t" // 0-1   n1 = hi
        "out  %[port] , %[lo]"    "\n\t" // 1    PORT = lo
        "brne headB"              "\n\t" // 2    while(i) (Z flag set above)
        "rjmp continueB"          "\n\t"
       "bit4B:"                   "\n\t"
        // Bit 4:
        "out  %[port] , %[hi]"    "\n\t" // 1    PORT = hi
        "mov  %[n1]   , %[lo]"    "\n\t" // 1    n1   = lo
        "out  %[port] , %[n2]"    "\n\t" // 1    PORT = n2
        "lsr  %[multa]"           "\n\t" // 1    multa >>= 1
        "mov  %[n2]   , %[lo]"    "\n\t" // 1    n2   = lo
        "sbrc %[byte] , 3"        "\n\t" // 1-2  if(b & 0x08)
         "mov %[n1]   , %[hi]"    "\n\t" // 0-1   n1 = hi
        "out  %[port] , %[lo]"    "\n\t" // 1    PORT = lo
        "sbrc %[multb], 3"        "\n\t" // 1-2  if(multb & 0x08)
         "adc %[multc], %[multa]" "\n\t" // 0-1   multc += multa (+ carry)
        // Bit 3:
        "out  %[port] , %[hi]"    "\n\t" // 1    PORT = hi
        "lsr  %[multa]"           "\n\t" // 1    multa >>= 1
        "out  %[port] , %[n1]"    "\n\t" // 1    PORT = n1
        "sbrc %[multb], 2"        "\n\t" // 1-2  if(multb & 0x04)
         "adc %[multc], %[multa]" "\n\t" // 0-1   multc += multa (+ carry)
        "sbrc %[byte] , 2"        "\n\t" // 1-2  if(b & 0x04)
         "mov %[n2]   , %[hi]"    "\n\t" // 0-1   n2 = hi
        "out  %[port] , %[lo]"    "\n\t" // 1    PORT = lo
        "lsr  %[multa]"           "\n\t" // 1    multa >>= 1
        "mov  %[n1]   , %[lo]"    "\n\t" // 1    n1   = lo 
        // Bit 2:
        "out  %[port] , %[hi]"    "\n\t" // 1    PORT = hi
        "mov  %[n1]   , %[lo]"    "\n\t" // 1    n1   = lo
        "out  %[port] , %[n2]"    "\n\t" // 1    PORT = n2
        "sbrc %[multb], 1"        "\n\t" // 1-2  if(multb & 0x02)
         "adc %[multc], %[multa]" "\n\t" // 0-1   multc += multa (+ carry)
        "sbrc %[byte] , 1"        "\n\t" // 1-2  if(b & 0x02)
         "mov %[n1]   , %[hi]"    "\n\t" // 0-1   n1 = hi
        "out  %[port] , %[lo]"    "\n\t" // 1    PORT = lo
        "rjmp bit1B"              "\n\t" // 2    jump bit1B
       "continueB:"               "\n"
      : [byte] "+r" (b),
        [n1] "+r" (n1),
        [n2] "+r" (n2),
        [count] "+w" (i),
        [multa] "+r" (multa),
        [multb] "+r" (multb),
        [multc] "+r" (multc)
      : [port] "I" (_SFR_IO_ADDR(PORTB)),
        [ptr] "e" (ptr),
        [hi] "r" (hi),
        [lo] "r" (lo));

#else
 #error "CPU SPEED NOT SUPPORTED"
#endif

#endif // end Architecture select

  interrupts();
  endTime = micros(); // Save EOD time for latch on next call
}

// Set the output pin number
void Adafruit_NeoPixel::setPin(uint8_t p) {
  pinMode(pin, INPUT);
  pin = p;
  pinMode(p, OUTPUT);
  digitalWrite(p, LOW);
#ifdef __AVR__
  port    = portOutputRegister(digitalPinToPort(p));
  pinMask = digitalPinToBitMask(p);
#endif
}

// Set pixel color from separate R,G,B components:
void Adafruit_NeoPixel::setPixelColor(
 uint16_t n, uint8_t r, uint8_t g, uint8_t b) {
  if(n < numLEDs) {
/* Note: No longer scaling here. Data gets scaled as it is written to the LED(s)
    if(brightness) { // See notes in setBrightness()
      r = (r * brightness) >> 8;
      g = (g * brightness) >> 8;
      b = (b * brightness) >> 8;
    }
*/
    uint8_t *p = &pixels[n * 3];
    p[rOffset] = r;
    p[gOffset] = g;
    p[bOffset] = b;
  }
}

// Query color from previously-set pixel (returns packed 32-bit RGB value)
uint32_t Adafruit_NeoPixel::getPixelColor(uint16_t n) const {
  if(n >= numLEDs) {
    // Out of bounds, return no color.
    return 0;
  }
  uint8_t *p = &pixels[n * 3];
  uint32_t c = ((uint32_t)p[rOffset] << 16) |
               ((uint32_t)p[gOffset] <<  8) |
                (uint32_t)p[bOffset];
/* Note: No longer scaling here. Data gets scaled as it is written to the LED(s)
  // Adjust this back up to the true color, as setting a pixel color will
  // scale it back down again.
  if(brightness) { // See notes in setBrightness()
    //Cast the color to a byte array
    uint8_t * c_ptr =reinterpret_cast<uint8_t*>(&c);
    c_ptr[0] = (c_ptr[0] << 8)/brightness;
    c_ptr[1] = (c_ptr[1] << 8)/brightness;
    c_ptr[2] = (c_ptr[2] << 8)/brightness;
  }
*/
  return c; // Pixel # is out of bounds
}

// Returns pointer to pixels[] array.  Pixel data is stored in device-
// native format and is not translated here.  Application will need to be
// aware whether pixels are RGB vs. GRB and handle colors appropriately.
uint8_t *Adafruit_NeoPixel::getPixels(void) const {
  return pixels;
}

uint16_t Adafruit_NeoPixel::numPixels(void) const {
  return numLEDs;
}

/*
// Adjust output brightness; 0=darkest (off), 255=brightest.  This does
// NOT immediately affect what's currently displayed on the LEDs.  The
// next call to show() will refresh the LEDs at this level.  However,
// this process is potentially "lossy," especially when increasing
// brightness.  The tight timing in the WS2811/WS2812 code means there
// aren't enough free cycles to perform this scaling on the fly as data
// is issued.  So we make a pass through the existing color data in RAM
// and scale it (subsequent graphics commands also work at this
// brightness level).  If there's a significant step up in brightness,
// the limited number of steps (quantization) in the old data will be
// quite visible in the re-scaled version.  For a non-destructive
// change, you'll need to re-render the full strip data.  C'est la vie.
void Adafruit_NeoPixel::setBrightness(uint8_t b) {
  // Stored brightness value is different than what's passed.
  // This simplifies the actual scaling math later, allowing a fast
  // 8x8-bit multiply and taking the MSB.  'brightness' is a uint8_t,
  // adding 1 here may (intentionally) roll over...so 0 = max brightness
  // (color values are interpreted literally; no scaling), 1 = min
  // brightness (off), 255 = just below max brightness.
  uint8_t newBrightness = b + 1;
  if(newBrightness != brightness) { // Compare against prior value
    // Brightness has changed -- re-scale existing data in RAM
    uint8_t  c,
            *ptr           = pixels,
             oldBrightness = brightness - 1; // De-wrap old brightness value
    uint16_t scale;
    if(oldBrightness == 0) scale = 0; // Avoid /0
    else if(b == 255) scale = 65535 / oldBrightness;
    else scale = (((uint16_t)newBrightness << 8) - 1) / oldBrightness;
    for(uint16_t i=0; i<numBytes; i++) {
      c      = *ptr;
      *ptr++ = (c * scale) >> 8;
    }
    brightness = newBrightness;
  }
}
*/

// Adjust the brightness setting between 0 and NEO_MAX_BRIGHTNESS,
// which corresponds to 0.0 to 1.0 scaling. This scaling is done on the fly
// as the data is written out to the LED(s), so color data in the pixel buffer
// are not lossy.
void Adafruit_NeoPixel::setBrightness(uint8_t b) {
    brightness = (b >= NEO_MAX_BRIGHTNESS) ? NEO_MAX_BRIGHTNESS : b;
}

//Return the brightness value
uint8_t Adafruit_NeoPixel::getBrightness(void) const {
  return brightness;
}

void Adafruit_NeoPixel::clear() {
  memset(pixels, 0, numBytes);
}

void Adafruit_NeoPixel::blendPixelColor(uint16_t n, uint8_t r, uint8_t g,
  uint8_t b, uint16_t blend) {
  if(n < numLEDs) {
    blend = (blend > NEO_MAX_BLEND) ? NEO_MAX_BLEND : blend;
    uint16_t orig = NEO_MAX_BLEND - blend;
    uint8_t *p = &pixels[n * 3];
    p[rOffset] = ((uint16_t)p[rOffset] * orig + (uint16_t)r * blend) >> 8;
    p[gOffset] = ((uint16_t)p[gOffset] * orig + (uint16_t)g * blend) >> 8;
    p[bOffset] = ((uint16_t)p[bOffset] * orig + (uint16_t)b * blend) >> 8;
  }
}

void Adafruit_NeoPixel::fade(uint8_t r, uint8_t g, uint8_t b, uint16_t blend) {
  for (uint16_t i = 0; i < numLEDs; ++i) {
    blendPixelColor(i, r, g, b, blend);
  }
}
