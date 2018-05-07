/**
 * Affectron by Savage Internet
 * HUMAN, I STILL CAN'T SEE: Code 0
 * 
 * In this step, you set up the NeoPixel Ring 16.
 * 
 * This sketch requires the Adafruit NeoPixel library, which you can install
 * by going to Sketch > Include Library > Manage Libraries..., then searching
 * for "adafruit neopixel".
 * 
 * Make sure you install the library called "Adafruit NeoPixel", and NOT the
 * DMA one!
 */

#include <Adafruit_NeoPixel.h>

// NEOPIXELS

/*
 * We're using a 16-LED ring, so let's set that up.  We can also use this value below to
 * set colors for all the lights.
 */
#define NEOPIXEL_NUM_PIXELS 16

/*
 * Pin for the NeoPixel Ring.  This pin controls color and brightness for the NeoPixels.
 * See https://learn.adafruit.com/adafruit-neopixel-uberguide/arduino-library-use for
 * more details.
 */
#define NEOPIXEL_PIN 4

/*
 * Initialize the strip.
 */
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NEOPIXEL_NUM_PIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

// SETUP AND LOOP

void setup() {
  Serial.begin(9600);

  /*
   * NeoPixel brightness ranges from 0-255.  255 is *really* bright, though - you can
   * often use a much lower value, like we do here!
   * 
   * Due to a fancy thing called "gamma", 24 here is actually about 40% as bright as
   * 255.  See this link for more details:
   * 
   * https://gamedevelopment.tutsplus.com/articles/gamma-correction-and-why-it-matters--gamedev-14466
   */
  strip.setBrightness(24);
  strip.begin();
}

void loop() {
    
  /*
   * TODO 0: make all pixels show a color of your choice!  Google actually has a
   * great color picker:
   * 
   * https://www.google.ca/search?q=color+picker
   * 
   * Note the part that starts with "rgb(": the three numbers in parentheses are
   * the red, green, and blue values in that order.
   */
  for (int i = 0; i < 4; i++) {
    /*
     * setPixelColor() takes four arguments: which LED to light up, followed by
     * the red, green, and blue values in that order.
     */
    strip.setPixelColor(i, 255, 0, 0);
  }
  strip.show();
}
