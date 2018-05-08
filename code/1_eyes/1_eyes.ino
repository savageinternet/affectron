/**
 * Affectron by Savage Internet
 * HUMAN, I STILL CAN'T SEE
 * 
 * In this step, you set up the NeoPixel Ring 16, then combine it with the HC-SR04
 * ultrasonic range sensor from the last step.  Finally, we do AWESOME MAGIC with
 * the lights!
 * 
 * This step requires the Adafruit NeoPixel library, which you can install
 * by going to Sketch > Include Library > Manage Libraries..., then searching
 * for "adafruit neopixel".
 * 
 * Make sure you install the library called "Adafruit NeoPixel", and NOT the
 * DMA one!
 */

#include <Adafruit_NeoPixel.h>

// ULTRASONIC RANGE SENSOR

/*
 * TODO 1.1a: copy the #define statements for TRIG_PIN, ECHO_PIN, and
 * ECHO_TIMEOUT_US here.
 * 
 * #define statements should almost always be placed above setup().
 */

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

// GLOBAL VARIABLES

/*
 * TODO 1.3a: declare the variable "startedSeeingHumanAt" here.
 */

// SETUP AND LOOP

void setup() {
  Serial.begin(9600);

  /*
   * TODO 1.1b: copy the pinMode() calls for the Trig and Echo pins here.
   * 
   * It's important that your program only have one setup() function and one loop() function.
   * Otherwise, Arduino gets confused as to which one it should use!  This is why you can't
   * just copy-paste everything.
   */

  /*
   * NeoPixel brightness ranges from 0-255.  255 is *really* bright, though - you can
   * often use a much lower value, like we do here!
   * 
   * Note that setBrightness() is intended to be called only once in setup().  Do NOT use
   * this to perform animations.  See the library documentation for more details:
   * 
   * https://learn.adafruit.com/adafruit-neopixel-uberguide/arduino-library-use
   * 
   * Due to a fancy thing called "gamma", 24 here is actually about 40% as bright as
   * 255.  See this link for more details:
   * 
   * https://gamedevelopment.tutsplus.com/articles/gamma-correction-and-why-it-matters--gamedev-14466
   */
  strip.setBrightness(24);
  strip.begin();
}

/*
 * TODO 1.2a: copy the helper functions from the Human Explanation Document here.
 */

/*
 * TODO 1.2b: move the code that measures distance from loop() into measureDistance(), then
 * move the code that shows the lights from loop() into displayColors().  Finally, call
 * both functions from loop(), as shown in the Human Explanation Document.
 * 
 * Organizing your code into smaller functions helps you write larger, more complicated programs.
 * Trust us, you'll thank yourself later.
 */

/*
 * TODO 1.3b: use millis() inside displayColors() to mark when a human came nearby.
 */

/*
 * TODO 1.3c: use a "for" loop inside displayColors() to show how long the human has been nearby
 * by lighting up more or fewer LEDs.
 */

void loop() {
  /*
   * TODO 1.1c: copy all your loop() code from the 0_ears sketch here.
   */
  
  /*
   * TODO 1.0: make all pixels show a color of your choice!  See the examples below
   * for some ideas.
   * 
   * Google actually has a great color picker:
   * 
   * https://www.google.ca/search?q=color+picker
   * 
   * Note the part that starts with "rgb(": the three numbers in parentheses are
   * the red, green, and blue values in that order.
   */
   
  // YOU CAN GET RID OF EVERYTHING BELOW HERE

  /*
   * setPixelColor() takes four arguments: which LED to light up, followed by
   * the red, green, and blue values in that order.
   * 
   * Here we use setPixelColor() to turn LED 0 red.
   */
  strip.setPixelColor(0, 255, 0, 0);

  /*
   * "for" loops are a fundamental part of many programming languages: they let
   * you do something over and over again.
   * 
   * Here we use a "for" loop to turn LEDs 1-3 yellow.
   */
  for (int i = 1; i <= 3; i = i + 1) {
    strip.setPixelColor(i, 255, 255, 0);
  }

  /*
   * Another "for" loop - these are quite versatile!
   */
  for (int i = 4; i < 12; i = i + 1) {
    int red = 0;
    int green = 255 - (i - 4) * 32;
    int blue = (i - 4) * 32;
    strip.setPixelColor(i, red, green, blue);
  }

  /*
   * We *have* to call this to update the LEDs.
   */
  strip.show();

  delay(100);
}
