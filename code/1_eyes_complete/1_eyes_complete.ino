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

#define TRIG_PIN 2
#define ECHO_PIN 3
#define ECHO_TIMEOUT_US 40000

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

unsigned long startedSeeingHumanAt = 0;

// SETUP AND LOOP

void setup() {
  Serial.begin(9600);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

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

double measureDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, ECHO_TIMEOUT_US);
  if (duration == 0) {
    duration = ECHO_TIMEOUT_US;
  }

  double distance = 0.01715 * duration;
  Serial.print(distance);
  Serial.println(" cm");

  return distance;
}

void displayColors(boolean humanNearby) {
  if (!humanNearby) {
    startedSeeingHumanAt = 0;
  } else if (startedSeeingHumanAt == 0) {
    startedSeeingHumanAt = millis();
  }

  unsigned long timeNearbyHuman = millis() - startedSeeingHumanAt;
  int pixelsToLight = 0;
  if (startedSeeingHumanAt > 0) {
    pixelsToLight = timeNearbyHuman / 100;
  }
  
  for (int i = 0; i < 16; i = i + 1) {
    if (i < pixelsToLight) {
      strip.setPixelColor(i, 0, 0, 255);
    } else {
      strip.setPixelColor(i, 0, 0, 0);
    }
  }
  strip.show();
}

void loop() {
  double distanceToHuman = measureDistance();
  boolean humanNearby = (distanceToHuman <= 25);
  displayColors(humanNearby);
  delay(100);
}
