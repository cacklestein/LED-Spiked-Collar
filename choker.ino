//  Choker
//  Controls a NeoPixel (WS2812B) spiked collar to provind several light
//  pattern modes on an ATtiny85.
//  
//  Support for optional ADXL345 accelerometer board will be added in a future update.
//
//  19 April 2015
//  by Joshua Brooks
//
#include "Adafruit_NeoPixel.h" // Requires modified version with on-the-fly brightness scaling

// Constants
//
#define NUMROWS 3
#define NUMCOLUMNS 23
#define CENTERROW ((NUMROWS - 1)/2)
#define CENTERCOLUMN ((NUMCOLUMNS - 1)/2)
#define NUMPIXELS (NUMROWS * NUMCOLUMNS)
#define PIN_WS2812B 1
#define PIN_LED 4
#define PIN_SWITCH 3

#define MAX_PATTERN 16
#define AUTO_CYCLE_TIMEOUT 2400

// Global variables
//
Adafruit_NeoPixel pixels(NUMPIXELS, PIN_WS2812B, NEO_GRB + NEO_KHZ800, 200);

// The setup function runs once after power-up.
//
void setup() {
  // initialize digital pin 3 as input with pullup
  pinMode(PIN_SWITCH, INPUT_PULLUP);
  // initialize digital pin 4 as an output.
  pinMode(PIN_LED, OUTPUT);
  pixels.begin();
  
  // Note: total max power is NUMPIXELS * (1 + 48mA * Brightness / 255)
  // For 69 LEDs:
  //   Brightness              Max power(mA)
  //   -------------------------------------
  //   NEO_MAX_BRIGHTNESS / 32      173
  //   NEO_MAX_BRIGHTNESS / 16      277
  //   NEO_MAX_BRIGHTNESS / 8       485
  //   NEO_MAX_BRIGHTNESS / 4       900
  //   NEO_MAX_BRIGHTNESS / 2      1731
  //   NEO_MAX_BRIGHTNESS          3381
  pixels.setBrightness(NEO_MAX_BRIGHTNESS);
  
  // Blank out the pixels
  pixels.clear();
  pixels.show();
  
  // Blink the on-board LED a few times before starting up
  //
  for (int i = 0; i < 8; ++i) {
    digitalWrite(PIN_LED, i & 1 ? LOW : HIGH);
    delay(100);
  }
}

#define RAINBOW_SIZE 16
#define RGB8(r,g,b) ((r & 0xE0) | ((g & 0xE0) >> 3) | (b >> 6))
const unsigned char rainbow[RAINBOW_SIZE] = {
  RGB8(242, 109, 125),
  RGB8(242, 101, 34),
  RGB8(247, 148, 29),
  RGB8(255, 242, 0),
  RGB8(141, 198, 63),
  RGB8(57, 181, 74),
  RGB8(0, 166, 81),
  RGB8(0, 169, 157),
  RGB8(0, 174, 239),
  RGB8(0, 114, 88),
  RGB8(0, 84, 166),
  RGB8(49, 49, 146),
  RGB8(46, 49, 146),
  RGB8(146, 39, 146),
  RGB8(236, 0, 140),
  RGB8(237, 20, 91)
};


// HSV to RGB colors
// hue: 0-359, sat: 0-255, val (lightness): 0-255
// adapted from http://funkboxing.com/wordpress/?p=1366
void HSVtoRGB(int hue, int sat, int val, uint8_t * colors) {
    int base;
    
    // Scale hue 0-383 range to avoid division
    // This makes:
    //     60 become 0x40
    //   / 60 become >> 6
    //   % 60 become & 0x3F
    hue *= 17;
    hue >>= 4;
    
    if (sat == 0) { // Achromatic color (gray).
        colors[0] = val;
        colors[1] = val;
        colors[2] = val;
    } else {
        base = ((255 - sat) * val) >> 8;
        switch (hue >> 6) { // orig: hue / 60
        case 0:
            colors[0] = val;
            colors[1] = (((val - base) * hue) >> 6) + base;
            colors[2] = base;
            break;
        case 1:
            colors[0] = (((val - base) * (0x40 - (hue & 0x3F))) >> 6) + base;
            colors[1] = val;
            colors[2] = base;
            break;
        case 2:
            colors[0] = base;
            colors[1] = val;
            colors[2] = (((val - base) * (hue & 0x3F)) >> 6) + base;
            break;
        case 3:
            colors[0] = base;
            colors[1] = (((val - base) * (0x40 - (hue & 0x3F))) >> 6) + base;
            colors[2] = val;
            break;
        case 4:
            colors[0] = (((val - base) * (hue & 0x3F)) >> 6) + base;
            colors[1] = base;
            colors[2] = val;
            break;
        case 5:
            colors[0] = val;
            colors[1] = base;
            colors[2] = (((val - base) * (0x40 - (hue & 0x3F))) >> 6) + base;
            break;
        }
    }
}


void rainbowStripes(uint8_t frame) {
  unsigned char dim;
  unsigned char index;
  for (int col = 0, i = 0; col < NUMCOLUMNS; ++col) {
    for(int row = 0; row < NUMROWS; ++row, ++i) {
      if (row == CENTERROW) {
        index = (frame + col) >> 1;
        dim = 0;
      } else {
        index = (frame - col + 5) >> 2;
        dim = 3;
      }
      uint8_t rgb8 = rainbow[index & (RAINBOW_SIZE - 1)];
      pixels.setPixelColor(i,
        (rgb8 & 0xE0) >> dim,
        ((rgb8 << 3) & 0xE0) >> dim,
        (rgb8 << 6) >> dim);
    }
  }
}

// Red/Green/Blue disco mode
uint8_t disco(uint8_t frame) {
  frame >>= 2;
  uint8_t frameRed = frame;
  uint8_t frameGreen = frame + 10;
  uint8_t frameBlue = frame - 10;
  frameRed = ((frameRed & 0x10) ? frameRed ^ 0x0f : frameRed) & 0x0f;
  frameGreen = ((frameGreen & 0x10) ? frameGreen ^ 0x0f : frameGreen) & 0x0f;
  frameBlue = ((frameBlue & 0x10) ? frameBlue ^ 0x0f : frameBlue) & 0x0f;
  uint8_t r, g, b;
  for (int col = 0; col < NUMCOLUMNS; ++col) {
    int distFromCenter = (col > CENTERCOLUMN) ? col - CENTERCOLUMN : CENTERCOLUMN - col;
    r = (distFromCenter == frameRed) ? 0xFF : 0x00;
    g = (distFromCenter == frameGreen) ? 0xFF : 0x00;
    b = (distFromCenter == frameBlue) ? 0xFF : 0x00;
    pixels.blendPixelColor(col * 3, r >> 2, g >> 2, b >> 2, 32);
    pixels.blendPixelColor(col * 3 + 1, r, g, b, 32);
    pixels.blendPixelColor(col * 3 + 2, r >> 2, g >> 2, b >> 2, 32);
  }
  return 25; // 25 milliseconds = 40 frames/sec
}

// Rainbow Cylon
uint8_t rainbowCylon(int frame) {
  uint8_t r, g, b, rgb8;
  rgb8 = rainbow[(frame >> 5) & (RAINBOW_SIZE - 1)];
  r = (rgb8 & 0xE0);
  g = ((rgb8 << 3) & 0xE0);
  b = (rgb8 << 6);
  
  pixels.fade(r >> 4, g >> 4, b >> 4, 64);
  if (frame & 0x20) frame ^= 0xff;
  frame &= 0x1F;
  if (frame >= 4) {
    frame -= 4;
    if (frame < NUMCOLUMNS) {
      pixels.setPixelColor(NUMROWS * frame + CENTERROW, r, g, b);
    }
  }
  return 25; // 25 milliseconds = 40 frames/sec
}

// White max current test
void autoDimmingTest(uint8_t frame) {
  uint8_t intensity;
  frame >>= 3;
  frame &= 0xff;
  frame = ((frame & 0x10) ? frame ^ 0x0f : frame) & 0x0f;
  for (int col = 0; col < NUMCOLUMNS; ++col) {
    int distFromCenter = (col > CENTERCOLUMN) ? col - CENTERCOLUMN : CENTERCOLUMN - col;
    intensity = (distFromCenter < frame) ? 0xFF : 0x00;
    pixels.blendPixelColor(col * 3, intensity, intensity, intensity, 64);
    pixels.blendPixelColor(col * 3 + 1, intensity, intensity, intensity, 64);
    pixels.blendPixelColor(col * 3 + 2, intensity, intensity, intensity, 64);
  }
}

// Flashlight
void flashlight(uint8_t frame) {
  for (uint8_t col = 0, i = 0; col < NUMCOLUMNS; ++col) {
    uint8_t intensity = (col == CENTERCOLUMN || col == CENTERCOLUMN - 1 || col == CENTERCOLUMN + 1) ? 0xFF : 0x00;
    for (uint8_t row = 0; row < NUMROWS; ++row, ++i) {
      pixels.setPixelColor(i, intensity, intensity, intensity);
    }
  }
}

// bling
uint8_t bling(uint16_t frame) {
  uint8_t color[3];

  pixels.fade(0, 0, 0, 32); // Fade already lit pixels to black
  HSVtoRGB(random(0, 360), 128, 255, color);
  pixels.setPixelColor(random(0, pixels.numPixels()), color[0], color[1], color[2]);
  return 5;
}

// waves
uint8_t waves(uint16_t index) {
  float angleBase = (float)index * 1.;
  uint8_t colorA[3];
  uint8_t colorB[3];
  HSVtoRGB((index << 2) % 360, 255, 32, colorA);
  HSVtoRGB((index >> 1) % 360, 255, 32, colorB);

  pixels.fade(colorA[0], colorA[1], colorA[2], 256);
  for (uint8_t col = 0; col < NUMCOLUMNS; ++col) {
    uint16_t level = (sin(angleBase + .5 * (float)col) + 1.) * 384;
    pixels.blendPixelColor(col * 3, colorB[0], colorB[1], colorB[2], (level >= 256) ? 256 : level);
    pixels.blendPixelColor(col * 3 + 1, colorB[0], colorB[1], colorB[2], (level >= 512) ? 256 : (level < 256) ? 0 : level - 256);
    pixels.blendPixelColor(col * 3 + 2, colorB[0], colorB[1], colorB[2], (level < 512) ? 0 : level - 512);
    if (level < 256) pixels.blendPixelColor(col * 3, 32, 32, 32, 256 - level);
    else if (level < 384) pixels.blendPixelColor(col * 3 + 1, 32, 32, 32, level - 128);
    else if (level < 512) pixels.blendPixelColor(col * 3 + 1, 32, 32, 32, 512 - level);
    else pixels.blendPixelColor(col * 3 + 2, 32, 32, 32, level - 512);
  }
  
  return 50;
}

uint8_t tracer(uint16_t index) {
  static float y = -1;
  static float x = CENTERCOLUMN;
  static float gravityX = CENTERCOLUMN;
  static float vx = 0;
  static float vy = 0;
  
  // Calculate position based on a loose spring model with velocity damping in the x - direction
  vx *= .99;
  vx += .005 * (gravityX - x);
  vy += -.03 * y;
  x += vx;
  y += vy;
  
  uint8_t px = (x < 0.) ? 0 : (x >= NUMCOLUMNS) ? NUMCOLUMNS - 1 : x;
  uint8_t py = (y < -.33) ? 0 : (y > .33) ? 2 : 1;
  
  uint8_t color[3];
  HSVtoRGB(index % 360, 255, 255, color);
  pixels.fade(0, 0, 0, 48);
  pixels.setPixelColor(px * NUMROWS + py, color[0], color[1], color[2]);
  
  if (!(index & 0xFF)) {
    gravityX = 2 + (float)random(0, NUMCOLUMNS - 4);
  }
  
  return 10;
}

// The loop function runs over and over again forever
//
void loop() {
  static int index = 0;
  static unsigned char buttonDebounce = 0;
  static unsigned char pattern = 0;
  static int autoCycleTimeout = AUTO_CYCLE_TIMEOUT;
  uint8_t msDelay = 25; // milliseconds, default delay
  
  if (pixels.getPixels() == NULL) {
    // Error! Flash the LED
    digitalWrite(PIN_LED, (index & 1) ? LOW : HIGH);   // Toggle the LED
  } else {
    switch (pattern) {
      case 7:
        msDelay = rainbowCylon(index);
        break;
      case 8:
        msDelay = tracer(index);
        break;
//      case 9:
//        msDelay = waves(index);
//        break;
      case 10:
        msDelay = bling(index);
        break;
//      case 11:
//        flashlight(index);
//        break;
//      case 12:
//        autoDimmingTest(index);
//        break;
      case 13:
        msDelay = disco(index);
        break;
//      case 14:
//        rainbowStripes(color);
//        break;
//      case 15:
//        pixels.clear();
//        break;
      default:
        pattern++;
    }
    if (digitalRead(PIN_SWITCH) == LOW) {
      if (buttonDebounce < 4) {
        if (++buttonDebounce == 4) {
          // Reset the auto cycle timeout
          autoCycleTimeout = AUTO_CYCLE_TIMEOUT;
          // Change to a new pattern
          ++pattern;
        }
      }
    } else {
      buttonDebounce = 0;
    }
    
    if (--autoCycleTimeout <= 0) {
      // No change in pattern mode has occurred within the default timeout value.
      // Time to change mode.
      autoCycleTimeout = AUTO_CYCLE_TIMEOUT;
      ++pattern;
    }
    
    if (pattern >= MAX_PATTERN) {
      pattern = 0;
    }
    pixels.show(); // This sends the updated pixel color to the hardware.
  }
  ++index;
  
  digitalWrite(PIN_LED, ((index & 0x7F) != 0) ? LOW : HIGH);   // Toggle the LED
  delay(msDelay);
}
