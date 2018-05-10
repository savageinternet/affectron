/**
 * Affectron by Savage Internet
 * HUMAN, BRAIN ME
 * 
 * In this step, you'll give the Affectron four operating states: SEEKING_HUMAN,
 * HUMAN_NEARBY, DEPLOYING_SERVICE, and COOLDOWN.
 * 
 * In the SEEKING_HUMAN state, the Affectron uses the HC-SR04 ultrasonic range sensor
 * to find a human.
 * 
 * When a human is found, Affectron enters the HUMAN_NEARBY state, where it uses the
 * HC-SR04 to check whether the human remains nearby.
 * 
 * If the human remains nearby long enough, Affectron enters the DEPLOYING_SERVICE
 * state, where it uses its lights and servo to respond to the human's presence.
 * 
 * After it finishes the light-and-servo show, Affectron enters COOLDOWN state,
 * where it rests a bit so it can get ready to detect humans again.
 * 
 * Finally, after a few seconds, Affectron returns to the SEEKING_HUMAN state.
 * 
 * To keep track of these states, we'll use a "state machine".  State machines are
 * a common programming technique in robotics: they let us keep track of which state
 * a robot is in, how it moves from one state to another, and what it does in each
 * state.
 * 
 * There's a *lot* of programming in this step.  If you get stuck, you can always
 * sneak a peek at the "3_brain_complete" sketch!  Try to get as far as you can
 * first, though.
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
 * This is an "enum" describing the possible states Affectron can be in.  Enums are useful when you have
 * a limited set of possible values for variables, and you want to give them all human-readable names.
 * 
 * Here you see all four states.
 */
enum AffectronState {
  SEEKING_HUMAN,
  HUMAN_NEARBY,
  DEPLOYING_SERVICES,
  COOLDOWN
};

/*
 * Time a human must remain nearby to receive services from Affectron.  If the human leaves before
 * this many milliseconds have elapsed, Affectron reverts to the SEEKING_HUMAN state.  Otherwise,
 * it proceeds to the DEPLOYING_SERVICES state.
 */
#define HUMAN_NEARBY_MS 1600

/*
 * Time it takes to deploy services.  After this many milliseconds have elapsed, Affectron enters
 * the COOLDOWN state - see the state diagram in the Human Explanation Document for more details.
 */
#define DEPLOYING_SERVICES_MS 3200

/*
 * Time it takes to cooldown.  After this many milliseconds have elapsed, Affectron returns to the
 * SEEKING_HUMAN state.
 */
#define COOLDOWN_MS 6400

// GLOBAL VARIABLES

unsigned long enteredStateAt = 0;
unsigned long startedLoopAt = 0;

/*
 * These control the servo angle and direction.  When the servo is waving, we'll 
 */
int servoAngle = 0;
int servoAngleChange = 10;

/*
 * Affectron starts in the SEEKING_HUMAN state.
 */
AffectronState state = SEEKING_HUMAN;

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

void displayColors() {
  unsigned long now = millis();
  unsigned long timeInState = now - enteredStateAt;
  if (state == SEEKING_HUMAN) {
    int pixelToLight = timeInState / 100;
    pixelToLight = pixelToLight % 16;
    for (int i = 0; i < 16; i = i + 1) {
      if (i == pixelToLight) {
        strip.setPixelColor(i, 0, 0, 255);
      } else {
        strip.setPixelColor(i, 0, 0, 0);
      }
    }
  } else if (state == HUMAN_NEARBY) {
    int pixelsToLight = timeInState / 100;
    pixelsToLight = constrain(pixelsToLight, 0, 16);
    for (int i = 0; i < 16; i = i + 1) {
      if (i < pixelsToLight) {
        strip.setPixelColor(i, 0, 0, 255);
      } else {
        strip.setPixelColor(i, 0, 0, 0);
      }
    }
  } else if (state == DEPLOYING_SERVICES) {
    for (int i = 0; i < 16; i = i + 1) {
      strip.setPixelColor(i, 0, 255, 0);
    }
  } else if (state == COOLDOWN) {
    int pixelsToHide = timeInState / 400;
    pixelsToHide = constrain(pixelsToHide, 0, 16);
    for (int i = 0; i < 16; i = i + 1) {
      if (i >= pixelsToHide) {
        strip.setPixelColor(i, 255, 0, 0);
      } else {
        strip.setPixelColor(i, 0, 0, 0);
      }
    }
  }
  strip.show();
}

void moveServo() {
  servoAngle = servoAngle + servoAngleChange * 10;
  servoAngle = constrain(servoAngle, 0, 180);
  if (servoAngle == 0 || servoAngle == 180) {
    servoAngleChange = -servoAngleChange;
  }
  servo.write(servoAngle);
}

void resetServo() {
  servoAngle = 0;
  servoAngleChange = 10;
  servo.write(0);
}

void updateState() {
  if (state == SEEKING_HUMAN) {
    double distanceToHuman = measureDistance();
    boolean humanNearby = (distanceToHuman <= 25);

    if (humanNearby) {
      state = HUMAN_NEARBY;
      enteredStateAt = millis();
    }
  } else if (state == HUMAN_NEARBY) {
    double distanceToHuman = measureDistance();
    boolean humanNearby = (distanceToHuman <= 25);

    if (!humanNearby) {
      state = SEEKING_HUMAN;
      enteredStateAt = millis();
    }
  } else if (state == DEPLOYING_SERVICES) {
    moveServo();

    unsigned long now = millis();
    if (now - enteredStateAt >= DEPLOYING_SERVICES_MS) {
      state = COOLDOWN;
      enteredStateAt = now;
    }
  } else if (state == COOLDOWN) {
    resetServo();
    
    unsigned long now = millis();
    if (now - enteredStateAt >= COOLDOWN_MS) {
      state = SEEKING_HUMAN;
      enteredStateAt = now;
    }
  }
}

void endOfLoopDelay() {
  unsigned long shouldEndLoopAt = startedLoopAt + 50;
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
