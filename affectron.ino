#include <Adafruit_NeoPixel.h>

// ULTRASONIC RANGE SENSOR

/*
 * Pins for the ultrasonic range sensor.  These are labelled "Trig" and "Echo"
 * on the board; Trig sends an ultrasonic pulse, Echo listens for the pulse to
 * bounce back.
 */
#define TRIG_PIN 2
#define ECHO_PIN 3

/*
 * When reading distance from the ultrasonic range sensor, this is the maximum
 * time in microseconds we'll wait to register the bounce.
 * 
 * According to https://www.electroschematics.com/wp-content/uploads/2013/07/HC-SR04-datasheet-version-2.pdf,
 * the Echo pin caps out at 38ms = 38000us, so we'll round that up a bit.
 */
#define ECHO_TIMEOUT_US 40000

// NEOPIXELS

/*
 * We're using a 16-LED ring, so let's set that up.  We'll also need this value below to
 * set colors for all the lights.
 */
#define NEOPIXEL_NUM_PIXELS 16

/*
 * Pin for the NeoPixel Ring.  This pin controls color and brightness for the NeoPixels.
 * See https://learn.adafruit.com/adafruit-neopixel-uberguide/arduino-library-use for
 * more details.
 */
#define NEOPIXEL_PIN 4

Adafruit_NeoPixel strip = Adafruit_NeoPixel(NEOPIXEL_NUM_PIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

// HUG MECHANICS

enum HugState {
  CHARGE,
  ACTIVE,
  COOLDOWN
};

#define HUG_CHARGE_MS 1500
#define HUG_DISCHARGE_MS 3000
#define HUG_ACTIVE_MS 3000
#define HUG_COOLDOWN_MS 6000

#define HUG_THRESHOLD 30

/*
 * This is the target length of time loop() should take to run, in milliseconds.
 * 
 * LOOP_CHARGE_MS should be at least as large as ECHO_TIMEOUT_US / 1000, as it can take up to that
 * much time to read a pulse off.
 * 
 * LOOP_MS can be shorter than LOOP_CHARGE_MS by ECHO_TIMEOUT_US / 1000.
 */
#define LOOP_CHARGE_MS 50
#define LOOP_MS 10

double distance = 0.0;
double power = 0.0;
HugState state = CHARGE;
long activeTwinkle[NEOPIXEL_NUM_PIXELS];
long startOfLastLoop = 0l;

char buf[80];

void setup() {
  Serial.begin(9600);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  strip.setBrightness(24);
  strip.begin();
  strip.show();

  for (int i = 0; i < NEOPIXEL_NUM_PIXELS; i++) {
    activeTwinkle[i] = random(900, 1200);
  }
}

/*
 * Read distance from the ultrasonic range sensor, in centimetres.
 */
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
  double distance = duration / 58.3;
  return constrain(distance, 0, 200.0);
}

long getLoopMs() {
  if (state == CHARGE) {
    return LOOP_CHARGE_MS;
  }
  return LOOP_MS;
}

void updateHugPowerAndState() {
  long loopMs = getLoopMs();
  if (state == CHARGE) {
    distance = measureDistance();
    if (distance <= 0 || distance > HUG_THRESHOLD) {
      power -= 1.0 * loopMs / HUG_DISCHARGE_MS;
    } else {
      power += 1.0 * loopMs / HUG_CHARGE_MS;
    }
    power = constrain(power, 0, 1);
    if (power == 1) {
      state = ACTIVE;
    }
  } else if (state == ACTIVE) {
    power -= 1.0 * loopMs / HUG_ACTIVE_MS;
    power = constrain(power, 0, 1);
    if (power == 0) {
      power = 1;
      state = COOLDOWN;
    }
  } else { // state == COOLDOWN
    power -= 1.0 * loopMs / HUG_COOLDOWN_MS;
    power = constrain(power, 0, 1);
    if (power == 0) {
      state = CHARGE;
    }
  }
}

void displayPowerMeter(int r, int g, int b) {
  double k = power * NEOPIXEL_NUM_PIXELS;
  for (int i = 0; i < NEOPIXEL_NUM_PIXELS; i++) {
    if (i < k - 1) {
      strip.setPixelColor(i, r, g, b);
    } else if (i < k) {
      double f = k - i;
      strip.setPixelColor(i, f * r, f * g, f * b);
    } else {
      strip.setPixelColor(i, 0, 0, 0);
    }
  }
}

void displayActiveAnimation() {
  long now = millis();
  for (int i = 0; i < NEOPIXEL_NUM_PIXELS; i++) {
    
    float theta = 2.0 * M_PI * now / activeTwinkle[i];
    double f = (sin(theta) + 1) / 2;
    int r = 255;
    int g = f * 105;
    int b = f * 180;
    strip.setPixelColor(i, r, g, b);
  }
}

void displayHugPowerAndState() {
  if (state == CHARGE) {
    displayPowerMeter(0, 255 * power, 255 * (1 - power));
  } else if (state == ACTIVE) {
    displayActiveAnimation();
  } else { // state == COOLDOWN
    displayPowerMeter(255 * (0.75 + 0.25 * power), 0, 0);
  }
  strip.show();
}

long endOfLoopDelay() {
  long loopMs = getLoopMs();
  long elapsed = millis() - startOfLastLoop;
  long delayMs = loopMs - elapsed;
  if (delayMs > 0) {
    delay(delayMs);
  }
  startOfLastLoop = millis();
  return elapsed;
}

void loop() {
  updateHugPowerAndState();
  displayHugPowerAndState();
  endOfLoopDelay();
  //sprintf(buf, "%d %d %d %ld", (int) distance, (int) (power * 100), state, elapsed);
  //Serial.println(buf);
}
