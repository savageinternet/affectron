/**
 * Affectron by Savage Internet
 * HUMAN, BRAIN ME
 * 
 * In this step, you'll give the Affectron three operating states: DETECT_HUMAN,
 * DEPLOYING_SERVICE, and COOLDOWN.  In DETECT_HUMAN state, the Affectron uses the
 * HC-SR04 ultrasonic range sensor to, well, detect a human.  In DEPLOYING_SERVICE
 * state, the Affectron uses its lights and servo to respond to the human's presence.
 * In COOLDOWN state, the Affectron rests a bit so it can get ready to detect humans
 * again.
 * 
 * To keep track of these states, we'll use a "state machine".  State machines are
 * a common programming technique in robotics: they let us keep track of which state
 * a robot is in, how it moves from one state to another, and what it does in each
 * state.
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

#define SERVO_PIN A0
#define SERVO_PWM_MIN 850
#define SERVO_PWM_MAX 1400

Servo servo;

// STATES

/*
 * This is an "enum" describing the possible states Affectron can be in.
 * 
 * We'll start with DETECT_HUMAN and DEPLOYING_SERVICES first, then add COOLDOWN
 * later.
 */
enum AffectronState {
  DETECT_HUMAN,
  DEPLOYING_SERVICES
};

// GLOBAL VARIABLES

unsigned long startedSeeingHumanAt = 0;
int pixelsToLight = 0;

/*
 * Affectron starts in the DETECT_HUMAN state.
 */
AffectronState state = DETECT_HUMAN;

// SETUP AND LOOP

void setup() {
  Serial.begin(9600);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  strip.setBrightness(24);
  strip.begin();
  servo.attach(SERVO_PIN, SERVO_PWM_MIN, SERVO_PWM_MAX);
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
  if (startedSeeingHumanAt > 0) {
    pixelsToLight = timeNearbyHuman / 100;
  } else {
    pixelsToLight = 0;
  }
  pixelsToLight = constrain(pixelsToLight, 0, 16);
  
  for (int i = 0; i < 16; i = i + 1) {
    if (i < pixelsToLight) {
      strip.setPixelColor(i, 0, 0, 255);
    } else {
      strip.setPixelColor(i, 0, 0, 0);
    }
  }
  strip.show();
}

void moveServo() {
  int angle = map(pixelsToLight, 0, 16, 0, 180);
  servo.write(angle);
}

void updateState() {
  if (state == DETECT_HUMAN) {
    
  } else if (state == 
}

void loop() {
  moveServo();
  double distanceToHuman = measureDistance();
  boolean humanNearby = (distanceToHuman <= 25);
  displayColors(humanNearby);
  delay(100);
}
