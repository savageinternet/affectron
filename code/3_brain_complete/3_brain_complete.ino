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
 * These control the servo angle and direction.  When the servo is waving, we'll change the angle
 * by "servoAngleChange" each time moveServo() is called.
 * 
 * To change the rate at which the servo moves, try changing "servoAngleChange" to any value between
 * 1 and 30.  (The servo isn't fast enough to handle higher values.)
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
  /*
   * Several of the states need these two pieces of information, so we set them up top
   * to make them available to every branch of the "if" statement.
   */
  unsigned long now = millis();
  unsigned long timeInState = now - enteredStateAt;
  if (state == SEEKING_HUMAN) {
    int pixelToLight = timeInState / 100;
    /*
     * The "remainder" operator in action!  This divides "pixelToLight" by 16, then
     * takes the remainder so that we can get a valid LED number.
     */
    pixelToLight = pixelToLight % 16;
    /*
     * Even though we're only turning one light on, we still need the "for" loop to
     * set all the other lights off.
     */
    for (int i = 0; i < 16; i = i + 1) {
      if (i == pixelToLight) {
        strip.setPixelColor(i, 0, 0, 255);
      } else {
        strip.setPixelColor(i, 0, 0, 0);
      }
    }
  } else if (state == HUMAN_NEARBY) {
    /*
     * This next part is similar to the previous code.
     */
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
    /*
     * Nothing fancy here: setting all the pixels to the same color is relatively
     * straightforward.
     */
    for (int i = 0; i < 16; i = i + 1) {
      strip.setPixelColor(i, 0, 255, 0);
    }
  } else if (state == COOLDOWN) {
    /*
     * Similar to HUMAN_NEARBY, but:
     * 
     * - COOLDOWN lasts 6.4s, or 400ms per pixel, so we divide by 400 instead of 100;
     * - we're turning pixels off over time, so we flip the on / off states in our
     *   "if" statement.
     */
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
  /*
   * Finally, all changes to the lights require a call to strip.show(), so we do that
   * outside the "if" statement at the bottom.
   */
  strip.show();
}

void moveServo() {
  /*
   * Now we can see what this means: we're adding "servoAngleChange" to "servoAngle"
   * each time moveServo() is called.  Since "servoAngleChange" is set to 10, we'll move
   * by 10 degrees every 50ms.
   */
  servoAngle = servoAngle + servoAngleChange;
  servoAngle = constrain(servoAngle, 0, 180);
  if (servoAngle == 0 || servoAngle == 180) {
    /*
     * When we reach either end of the servo range, we switch direction.  This is an
     * easy way to cover both ends of the range at once, but you could equally well
     * split it into two separate cases.
     */
    servoAngleChange = -servoAngleChange;
  }
  servo.write(servoAngle);
}

void resetServo() {
  /*
   * Note that we reset the values of our global servo variables to their initial
   * values.  This is good practice, especially if you want your robots to have predictable
   * behavior as humans keep interacting with them.
   */
  servoAngle = 0;
  servoAngleChange = 10;
  servo.write(0);
}

void updateState() {
  /*
   * Again, we get these two pieces of information outside the "if" statement.
   */
  unsigned long now = millis();
  unsigned long timeInState = now - enteredStateAt;
  /*
   * updateState() is the heart of our state machine: a giant "if" statement whose branches
   * describe what happens in each state and when to move between states.  The global variable
   * "state" keeps track of the current state.
   */
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

    /*
     * This is the most complicated transition, as Affectron can move to two possible
     * states depending on what's going on.
     */
    if (!humanNearby) {
      state = SEEKING_HUMAN;
      enteredStateAt = now;
    } else if (timeInState >= HUMAN_NEARBY_MS) {
      /*
       * Here's an example of a time-triggered transition: we move to the next state once
       * a certain amount of time has elapsed.  We reuse this idea in DEPLOYING_SERVICES
       * and COOLDOWN below.
       */
      state = DEPLOYING_SERVICES;
      enteredStateAt = now;
    }
  } else if (state == DEPLOYING_SERVICES) {
    moveServo();

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
    /*
     * Note that we only perform the subtraction when we're sure the result will be
     * positive! 
     * 
     * If you try to give an "unsigned long" variable a negative value, it will
     * "wrap around" and instead be set to a *really* high positive value.
     */
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
