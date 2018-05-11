/**
 * Affectron by Savage Internet
 * 
 * In this step, we show off some advanced programming techniques for robotics.  Don't
 * worry if you don't understand them fully at first: you can do plenty with what you've
 * learned so far!
 * 
 * Whether you're curious to learn more or just want to see Affectron do some more cool
 * stuff, give this sketch a whirl.
 */

#include <Adafruit_NeoPixel.h>
#include <Servo.h>

// ULTRASONIC RANGE SENSOR

#define TRIG_PIN 2
#define ECHO_PIN 3
#define ECHO_TIMEOUT_US 40000

// NEOPIXELS

#define NEOPIXEL_NUM_PIXELS 16
#define NEOPIXEL_PIN 4
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NEOPIXEL_NUM_PIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

// SERVOS

#define SERVO_LEFT_PIN A0
#define SERVO_LEFT_MIN 850
#define SERVO_LEFT_MAX 1400

#define SERVO_RIGHT_PIN A1
#define SERVO_RIGHT_MIN 850
#define SERVO_RIGHT_MAX 1500

Servo servoLeft;
Servo servoRight;

// STATES

enum AffectronState {
  SEEKING_HUMAN,
  HUMAN_NEARBY,
  DEPLOYING_SERVICES,
  COOLDOWN
};

#define HUMAN_NEARBY_MS 1600
#define DEPLOYING_SERVICES_MS 6400
#define COOLDOWN_MS 6400

// GLOBAL VARIABLES

unsigned long enteredStateAt = 0;
unsigned long startedLoopAt = 0;

long twinklePeriods[NEOPIXEL_NUM_PIXELS];

double F_MIN = 1 / (1 + exp(4));
double F_MAX = 1 / (1 + exp(-4));

double distance = 0.0;

AffectronState state = SEEKING_HUMAN;

// SETUP AND LOOP

void setup() {
  Serial.begin(9600);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  strip.setBrightness(24);
  strip.begin();
  servoLeft.attach(SERVO_LEFT_PIN, SERVO_LEFT_MIN, SERVO_LEFT_MAX);
  servoRight.attach(SERVO_RIGHT_PIN, SERVO_RIGHT_MIN, SERVO_RIGHT_MAX);

  for (int i = 0; i < NEOPIXEL_NUM_PIXELS; i++) {
    twinklePeriods[i] = random(900, 1200);
  }
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

  double d = 0.01715 * duration;
  if (d < 400) {
    distance += (d - distance) * 0.5;
  }
  return distance;
}

void displaySeekingSpinner(int pixelToLight) {
  for (int i = 0; i < NEOPIXEL_NUM_PIXELS; i++) {
    int di = pixelToLight - i;
    if (di < 0) {
      di += NEOPIXEL_NUM_PIXELS;
    }
    double f = 1.0 * di / NEOPIXEL_NUM_PIXELS;
    f = pow(1.0 - f, 2.6);
    int b = 255 * f;
    strip.setPixelColor(i, 0, 0, b);
  }
}

void displayHumanNearbyAnimation(int pixelsToLight) {
  double f = 1.0 * pixelsToLight / NEOPIXEL_NUM_PIXELS;
  f = pow(f, 2.6);
  for (int i = 0; i < NEOPIXEL_NUM_PIXELS; i++) {
    int g = 64 + 192 * f;
    int b = 192 - 192 * f;
    if (i < pixelsToLight) {
      strip.setPixelColor(i, 0, g, b);
    } else {
      strip.setPixelColor(i, 0, 0, 0);
    }
  }
}

void displayTwinkleAnimation() {
  long now = millis();
  for (int i = 0; i < NEOPIXEL_NUM_PIXELS; i++) {
    float theta = 2.0 * M_PI * now / twinklePeriods[i];
    double f = (sin(theta) + 1) / 2;
    int r = 255;
    int g = 105 * f;
    int b = 180 * f;
    strip.setPixelColor(i, r, g, b);
  }
}

void displayCooldownAnimation(unsigned long timeInState) {
  double k = 1.0 * timeInState / 400;
  for (int i = 0; i < 16; i++) {
    if (i < k - 1) {
      strip.setPixelColor(i, 0, 0, 0);
    } else if (i < k) {
      double f = 1.0 - (k - i);
      int r = 255 * f;
      strip.setPixelColor(i, r, 0, 0);
    } else {
      strip.setPixelColor(i, 255, 0, 0);
    }
  }
}

void displayColors() {
  unsigned long now = millis();
  unsigned long timeInState = now - enteredStateAt;
  if (state == SEEKING_HUMAN) {
    int pixelToLight = timeInState / 100;
    pixelToLight = pixelToLight % NEOPIXEL_NUM_PIXELS;
    displaySeekingSpinner(pixelToLight);
  } else if (state == HUMAN_NEARBY) {
    int pixelsToLight = timeInState / 100;
    pixelsToLight = constrain(pixelsToLight, 0, 16);
    displayHumanNearbyAnimation(pixelsToLight);
  } else if (state == DEPLOYING_SERVICES) {
    displayTwinkleAnimation();
  } else if (state == COOLDOWN) {
    displayCooldownAnimation(timeInState);
  }
  strip.show();
}

void moveServo(unsigned long timeInState) {
  double t;
  if (timeInState <= 2800) {
    t = 1.0 * timeInState / 2800;
  } else if (timeInState <= 3600) {
    t = 1.0;
  } else {
    t = 1.0 * (6400 - timeInState) / 2800;
  }
  double f = 1 / (1 + exp(-8 * (t - 0.5)));
  f = (f - F_MIN) / (F_MAX - F_MIN);
  
  int servoLeftAngle = (int) (90.5 + 90 * f);
  servoLeftAngle = constrain(servoLeftAngle, 90, 180);
  servoLeft.write(servoLeftAngle);
  
  int servoRightAngle = (int) (90.5 - 90 * f);
  servoRightAngle = constrain(servoRightAngle, 0, 90);
  servoRight.write(servoRightAngle);
}

void resetServo() {
  servoLeft.write(90);
  servoRight.write(90);
}

void updateState() {
  unsigned long now = millis();
  unsigned long timeInState = now - enteredStateAt;
  
  if (state == SEEKING_HUMAN) {
    double distanceToHuman = measureDistance();
    boolean humanNearby = (distanceToHuman <= 25);
    if (humanNearby) {
      state = HUMAN_NEARBY;
      enteredStateAt = now;
    }
  } else if (state == HUMAN_NEARBY) {
    double distanceToHuman = measureDistance();
    boolean humanNearby = (distanceToHuman <= 25);
    if (!humanNearby) {
      state = SEEKING_HUMAN;
      enteredStateAt = now;
    } else if (timeInState >= HUMAN_NEARBY_MS) {
      state = DEPLOYING_SERVICES;
      enteredStateAt = now;
    }
  } else if (state == DEPLOYING_SERVICES) {
    moveServo(timeInState);
    if (timeInState >= DEPLOYING_SERVICES_MS) {
      state = COOLDOWN;
      enteredStateAt = now;
    }
  } else if (state == COOLDOWN) {
    resetServo();
    if (timeInState >= COOLDOWN_MS) {
      state = SEEKING_HUMAN;
      enteredStateAt = now;
    }
  }
}

long getLoopMs() {
  if (state == SEEKING_HUMAN || state == HUMAN_NEARBY) {
    return 15;
  }
  return 50;
}

void endOfLoopDelay() {
  long loopMs = getLoopMs();
  unsigned long shouldEndLoopAt = startedLoopAt + loopMs;
  unsigned long now = millis();
  if (now < shouldEndLoopAt) {
    unsigned long toDelay = shouldEndLoopAt - now;
    delay(toDelay);
  }
}

void loop() {
  startedLoopAt = millis();
  updateState();
  displayColors();
  endOfLoopDelay();
}
