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
 * We'll start with these first three states, then eventually add the COOLDOWN state once the first
 * three are working.
 */
enum AffectronState {
  SEEKING_HUMAN,
  HUMAN_NEARBY,
  DEPLOYING_SERVICES
  /*
   * TODO 3.2a: add the COOLDOWN state here.
   */
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
 * TODO 3.2b: add a #define statement for COOLDOWN_MS here, and make it equal 6400 (6.4s = 6400ms).
 */

// GLOBAL VARIABLES

/*
 * TODO 3.0a: rename "startedSeeingHumanAt" to "enteredStateAt".  We're going to use
 * this variable for all Affectron states, so we'll give it a more generic name.
 * 
 * You'll have to rename this everywhere in the entire file.  That *would* be tedious, but we
 * have a computer and can use find-replace instead (Edit > Find...)!
 */
unsigned long startedSeeingHumanAt = 0;
int pixelsToLight = 0;

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
   * TODO 3.1: this one's a bit more open-ended.  Change displayColors() to have the
   * following behavior depending on state:
   * 
   * - SEEKING_HUMAN: light one blue pixel that goes around and around the ring, moving
   *   to the next pixel every 100ms.
   * - HUMAN_NEARBY: as before, light up more and more pixels as the human stays nearby.
   * - DEPLOYING_SERVICE: light all pixels green.
   * 
   * See updateState() for an example of how to do different things based on the current
   * state.  Feel free to introduce more functions if you need to!
   * 
   * For SEEKING_HUMAN, think about how we used millis() and "pixelsToLight" to light
   * several lights.  You might also find the "remainder" operator useful:
   * 
   * https://www.arduino.cc/reference/en/language/structure/arithmetic-operators/remainder/
   */

  /*
   * TODO 3.2e: add the COOLDOWN state to displayColors(), and give it this behavior:
   * 
   * - COOLDOWN: start with all pixels lit red, then light fewer and fewer pixels as
   *   Affectron rests.
   * 
   * This behavior is very similar to our HUMAN_NEARBY behavior.
   */
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
  /*
   * TODO 3.4a: make the robot wave back and forth in moveServo().  You will likely have
   * to introduce two global variables for this: "servoAngle" and "servoAngleChange".
   * As the names suggest, "servoAngle" keeps track of the current servo angle, while
   * "servoAngleChange" tracks how much to move the servo each time we call moveServo()
   * (and which direction to move it in).
   */
  int angle = map(pixelsToLight, 0, 16, 0, 180);
  servo.write(angle);
}

void resetServo() {
  /*
   * TODO 3.4b: make the robot return its arm to angle 0 in resetServo(),
   * and call resetServo() from the COOLDOWN state in updateState() below.
   * 
   * Make sure you set any global variables you used for moveServo() back to
   * their initial state, too!
   */
}

void updateState() {
  if (state == SEEKING_HUMAN) {
    double distanceToHuman = measureDistance();
    boolean humanNearby = (distanceToHuman <= 25);

    /*
     * TODO 3.0b: if a human is nearby, enter the HUMAN_NEARBY state.  In this case, we should also
     * set "enteredStateAt" to the current time using millis(); see displayColors() above for an
     * example.
     */
  } else if (state == HUMAN_NEARBY) {
    double distanceToHuman = measureDistance();
    boolean humanNearby = (distanceToHuman <= 25);

    /*
     * TODO 3.0c: if a human is not nearby, return to the SEEKING_HUMAN state.  Again, set
     * "enteredStateAt" to the current time using millis() in this case.
     * 
     * Otherwise, check whether we've spent HUMAN_NEARBY_MS milliseconds in the HUMAN_NEARBY
     * state; if so, enter the DEPLOYING_SERVICES state.
     */
  } else if (state == DEPLOYING_SERVICES) {
    moveServo();

    /*
     * TODO 3.0d: if we've spent at least DEPLOYING_SERVICES_MS milliseconds in the
     * DEPLOYING_SERVICES state, enter the SEEKING_HUMAN state.  (And - you guessed
     * it - set "enteredStateAt"!  We have to do this every time we change state.)
     */

    /*
     * TODO 3.2c: if we've spent DEPLOYING_SERVICES_MS milliseconds in the DEPLOYING_SERVICES
     * state, we should now enter the COOLDOWN state instead.
     */
  }
  /*
   * TODO 3.2d: add the COOLDOWN state to updateState(), and implement its behavior.  Use the
   * other states as examples if you get stuck!
   */
}

void endOfLoopDelay() {
  /*
   * TODO 3.3: another slightly more open-ended task.  Robots often work best when they respond
   * to humans on a regular schedule, so we want to make sure every call to loop() takes roughly
   * 50ms.
   * 
   * However, delay(50) isn't quite good enough, since we don't know how long the other stuff in
   * loop() took!  We need to figure out how much time to wait to make up 50ms in total, then
   * wait that long.  (Also, if we already took more than 50ms, we shouldn't wait at all.)
   * 
   * You'll need a global variable, say "startedLoopAt", and you'll need to use millis() to keep
   * track of timing.  Also, one word of warning about "unsigned long" variables: they cannot
   * take negative values.
   * 
   * Good luck!
   */
  delay(100);
}

void loop() {
  updateState();
  displayColors();
  endOfLoopDelay();
}
