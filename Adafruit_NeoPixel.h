/*--------------------------------------------------------------------
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
  --------------------------------------------------------------------*/

#ifndef ADAFRUIT_NEOPIXEL_H
#define ADAFRUIT_NEOPIXEL_H

#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
 #include <pins_arduino.h>
#endif

// 'type' flags for LED pixels (third parameter to constructor):
#define NEO_RGB     0x00 // Wired for RGB data order
#define NEO_GRB     0x01 // Wired for GRB data order
#define NEO_BRG     0x04
  
#define NEO_COLMASK 0x01
#define NEO_KHZ800  0x02 // 800 KHz datastream
#define NEO_SPDMASK 0x02
// Trinket flash space is tight, v1 NeoPixels aren't handled by default.
// Remove the ifndef/endif to add support -- but code will be bigger.
// Conversely, can comment out the #defines to save space on other MCUs.
#ifndef __AVR_ATtiny85__
#define NEO_KHZ400  0x00 // 400 KHz datastream
#endif

#define NEO_MAX_BRIGHTNESS 0x20
#define NEO_MAX_BLEND 0x100

class Adafruit_NeoPixel {

 public:

  // Constructor: number of LEDs, pin number, LED type
  Adafruit_NeoPixel(uint16_t n, uint8_t p=6, uint8_t t=NEO_GRB + NEO_KHZ800,
    uint16_t m = 0);
  ~Adafruit_NeoPixel();

  void
    begin(void),
    show(),
    setPin(uint8_t p),
    setPixelColor(uint16_t n, uint8_t r, uint8_t g, uint8_t b),
    setBrightness(uint8_t),
    blendPixelColor(uint16_t n, uint8_t r, uint8_t g, uint8_t b, uint16_t blend),
    clear(),
    fade(uint8_t r, uint8_t g, uint8_t b, uint16_t blend),
    adjustBrightnessForMaxCurrent();
  uint8_t
   *getPixels(void) const,
    getBrightness(void) const;
  uint16_t
    numPixels(void) const,
    calculateCurrent(bool scale = false);
  uint32_t
    getPixelColor(uint16_t n) const;
  inline bool
    canShow(void) { return (micros() - endTime) >= 50L; }

 private:

  const uint16_t
    numLEDs,       // Number of RGB LEDs in strip
    numBytes,      // Size of 'pixels' buffer below
    maxCurrent;    // Maximum current in mA
  uint8_t
    pin,           // Output pin number
    brightness,    // Brightness scale (0 - 32 corresponds to 0.0 - 1.0 scale)
   *pixels,        // Holds LED color values (3 bytes each)
    rOffset,       // Index of red byte within each 3-byte pixel
    gOffset,       // Index of green byte
    bOffset;       // Index of blue byte
  const uint8_t
    type;          // Pixel flags (400 vs 800 KHz, RGB vs GRB color)
  uint32_t
    endTime;       // Latch timing reference
#ifdef __AVR__
  const volatile uint8_t
    *port;         // Output PORT register
  uint8_t
    pinMask;       // Output PORT bitmask
#endif

};

#endif // ADAFRUIT_NEOPIXEL_H

