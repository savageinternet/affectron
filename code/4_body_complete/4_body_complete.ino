/**
 * Affectron by Savage Internet
 * HUMAN, I HAVE NO BODY
 * 
 * In this step, you get a bit of a break: we've extended the code from "3_brain_complete"
 * to include two servos, one for each arm.  You just have to upload the code, assemble
 * the robot, and accept its wonderful services!
 * 
 * As usual, we've commented things of note in the code.  Once you've played around with
 * Affectron for a bit, take a look at "5_body_advanced" - there's even more goodies in there!
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

/*
 * Like most humanoid robots, Affectron has two arms - so we have to set up two
 * servos here.  Each servo has its own pin, its own PWM minimum and maximum lengths...
 */

#define SERVO_LEFT_PIN A0
#define SERVO_LEFT_MIN 850
#define SERVO_LEFT_MAX 1400

#define SERVO_RIGHT_PIN A1
#define SERVO_RIGHT_MIN 850
#define SERVO_RIGHT_MAX 1500

/*
 * ...and its own Servo variable.
 */
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

/*
 * Aren't services more satisfying when they take longer?  But of course!
 * We've doubled the amount of service Affectron deploys here - no need to
 * thank us.
 */
#define DEPLOYING_SERVICES_MS 6400

#define COOLDOWN_MS 6400

// GLOBAL VARIABLES

unsigned long enteredStateAt = 0;
unsigned long startedLoopAt = 0;

/*
 * Both servos now start at 90 degrees.  We also got rid of our "servoAngleChange"
 * variable - we're now handling this a slightly different way right in moveServo().
 */
int servoLeftAngle = 90;
int servoRightAngle = 90;

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
      if (i < pixelsToHide) {
        strip.setPixelColor(i, 0, 0, 0);
      } else {
        strip.setPixelColor(i, 255, 0, 0);
      }
    }
  }
  strip.show();
}

/*
 * moveServo() gets a slight upgrade: we pass in the "timeInState" parameter so
 * that we can more precisely control what happens at each phase of the service
 * deployment cycle.j
 * 
 * (And yes, we *could* change our state machine to break DEPLOYING_SERVICES into
 * several sub-states, each controlling one phase of service deployment.  There's
 * usually more than one way to solve a problem in programming / robotics!)
 */
void moveServo(unsigned long timeInState) {
  if (timeInState < 1600) {
    /*
     * First phase of service deployment.  Note that the left arm moves from
     * 90 to 180 degrees, while the right arm moves from 90 to 0 degrees.  The
     * servos mirror each other, so they must move in opposite directions.
     * 
     * Note also that, since we don't have our servo change variables anymore, we
     * put the angle change values right in here.  We've slowed this down quite
     * a bit to account for the shorter range of motion and longer service
     * deployment time!
     */
    servoLeftAngle = servoLeftAngle + 3;
    servoLeftAngle = constrain(servoLeftAngle, 90, 180);
    servoLeft.write(servoLeftAngle);
    
    servoRightAngle = servoRightAngle - 3;
    servoRightAngle = constrain(servoRightAngle, 0, 90);
    servoRight.write(servoRightAngle);
  } else if (timeInState <= 4800) {
    /*
     * Second phase: maintain position.
     */
    servoLeft.write(180);
    servoRight.write(0);
  } else if (timeInState > 4800) {
    /*
     * Final phase: move both servos back to 90 degrees.
     */
    servoLeftAngle = servoLeftAngle - 3;
    servoLeftAngle = constrain(servoLeftAngle, 90, 180);
    servoLeft.write(servoLeftAngle);
    
    servoRightAngle = servoRightAngle + 3;
    servoRightAngle = constrain(servoRightAngle, 0, 90);
    servoRight.write(servoRightAngle);
  }
}

void resetServo() {
  /*
   * Again, make sure everything gets reset in resetServo(), including our global
   * variables.
   */
  servoLeftAngle = 90;
  servoLeft.write(servoLeftAngle);
  
  servoRightAngle = 90;
  servoRight.write(servoRightAngle);
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
    /*
     * Remember how "timeInState" is available for use in all of our states?
     * Here we pass it into moveServo(); see that function for more details
     * on how we're using it to control the servos.
     */
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
