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

/*
 * These parameters are used in smoothTweenParameter() below.
 */
#define T_K 8.0
#define T_MIN  (1 / (1 + exp(T_K / 2)))
#define T_MAX  (1 / (1 + exp(-T_K / 2)))

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
#define DEPLOYING_SERVICES_PAUSE_MS 800
#define DEPLOYING_SERVICES_MOVE_MS  ((DEPLOYING_SERVICES_MS - DEPLOYING_SERVICES_PAUSE_MS) / 2)
#define COOLDOWN_MS 6400

// GLOBAL VARIABLES

unsigned long enteredStateAt = 0;
unsigned long startedLoopAt = 0;

long twinklePeriods[NEOPIXEL_NUM_PIXELS];

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
  /*
   * Basic signal processing!  We'll ignore readings above 400 cm...
   */
  if (d < 400) {
    /*
     * ...and use a fun little technique called "rolling averages" on the rest of the readings.
     * The ultrasonic reading can sometimes be a bit jumpy.  By using a rolling average, we
     * smooth out some of these jumps.
     * 
     * 0.3 here is a "sensitivity parameter".  Higher values make the rolling average more
     * responsive to changes, while lower values smooth out changes over time.  Try changing
     * this to different values between 0 and 1 and see how the sensor responds differently
     * to nearby objects!
     */
    distance += (d - distance) * 0.3;
  }
  return distance;
}

/*
 * Now for some fun with lights.  We've given the different states a bit of personality;
 * see the comments on each to understand this code a bit better.
 * 
 * Where possible, these functions use NEOPIXEL_NUM_PIXELS instead of 16.  Programmers
 * usually prefer to use variables instead of hardcoded numbers - here, this makes it
 * easier to use the same code if you later want to swap the NeoPixel Ring out for a
 * different NeoPixel LED module.
 */

/*
 * Spinner animation for the SEEKING_HUMAN state.
 */
void displaySeekingSpinner(int pixelToLight) {
  for (int i = 0; i < NEOPIXEL_NUM_PIXELS; i++) {
    /*
     * This next bit creates the "trailing fade-away" effect.  "pixelToLight" is the
     * brightest LED, and every LED behind it is a bit dimmer.
     * 
     * To achieve that, we need to figure out how far behind "pixelToLight" the
     * current LED is.
     */
    int di = pixelToLight - i;
    if (di < 0) {
      di += NEOPIXEL_NUM_PIXELS;
    }
    /*
     * Next, we use this formula to generate a brightness factor between 0 and 1.
     * For "pixelToLight", "di" is 0, and so this would return 1.  For the LED
     * immediately in front of "pixelToLight", "di" is 15, and so this would return
     * 1 / 16.
     */
    double f = 1.0 * (NEOPIXEL_NUM_PIXELS - di) / NEOPIXEL_NUM_PIXELS;
    /*
     * Finally, we apply "gamma correction" to make it look like the color drops off
     * smoothly.  If you've ever played around with gamma settings on your computer
     * monitor - well, that's what the 2.6 value does here!  Try changing it to
     * different values between 1 and 4 and see the effect.
     */
    f = pow(f, 2.6);
    
    int b = 255 * f;
    strip.setPixelColor(i, 0, 0, b);
  }
}

/*
 * Charging animation for the HUMAN_NEARBY state.
 */
void displayHumanNearbyAnimation(int pixelsToLight) {
  /*
   * Very similar to displaySeekingSpinner(), but now we just change the color
   * of all the lights at once.  This causes the color to slowly shift from
   * blue to green as it charges!
   */
  double f = 1.0 * pixelsToLight / NEOPIXEL_NUM_PIXELS;
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

/*
 * Twinkling animation for the DEPLOYING_SERVICES state.
 */
void displayTwinkleAnimation() {
  long now = millis();
  for (int i = 0; i < NEOPIXEL_NUM_PIXELS; i++) {
    /*
     * Up in setup(), we initialized "twinklePeriods" to some random values between
     * 900 and 1200 - these are sine wave period lengths, in milliseconds.  We use
     * these to create a twinkling effect.
     */
    float theta = 2.0 * M_PI * now / twinklePeriods[i];
    /*
     * sin() takes a value between -1 and 1; this just maps that to a value between
     * 0 and 1, which we then use as a color factor.
     */
    double f = (sin(theta) + 1) / 2;
    /*
     * Move from red to pink and back.
     */
    int r = 255;
    int g = 105 * f;
    int b = 180 * f;
    strip.setPixelColor(i, r, g, b);
  }
}

/*
 * Discharging animation for the COOLDOWN state.
 */
void displayCooldownAnimation(unsigned long timeInState) {
  /*
   * Very similar to displayHumanNearbyAnimation()...
   */
  double k = 1.0 * timeInState * NEOPIXEL_NUM_PIXELS / DEPLOYING_SERVICES_MS;
  for (int i = 0; i < NEOPIXEL_NUM_PIXELS; i++) {
    if (i < k - 1) {
      strip.setPixelColor(i, 0, 0, 0);
    } else if (i < k) {
      /*
       * ...but with an added bonus: we fade each light out!
       */
      double f = 1.0 - (k - i);
      /*
       * There's that gamma correction again.
       */
      f = pow(f, 2.6);
      int r = 255 * f;
      strip.setPixelColor(i, r, 0, 0);
    } else {
      strip.setPixelColor(i, 255, 0, 0);
    }
  }
}

/*
 * Now, we stitch all this together by loading the correct animation for the
 * current state.
 */
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

/*
 * Animation, generally, is the art of moving something - cartoon characters,
 * pieces of a video game world, robot parts, etc. - according to a schedule.
 * Congratulations: you're an animator now!
 * 
 * In this case, we're moving a servo back and forth between a start angle and
 * an end angle.  Animators call these important positions "keyframes".
 * 
 * "Tweening", then, describes what happens between the keyframes.  Note the
 * three parameters to servoTween().  The last two, "angleStart" and
 * "angleEnd", are our keyframes.
 * 
 * What about "t", though?  "t" is a special "tweening parameter" that takes
 * values between 0 and 1.  When "t" is 0, the servo should be at "angleStart".
 * When "t" is 1, the servo should be at "angleEnd".
 * 
 * We'll use tweening parameters in the next couple of functions as well.
 */
int servoTween(double t, int angleStart, int angleEnd) {
  int angle = (int) ((1 - t) * angleStart + t * angleEnd + 0.5);
  if (angleStart < angleEnd) {
    return constrain(angle, angleStart, angleEnd);
  } else {
    return constrain(angle, angleEnd, angleStart);
  }
}

/*
 * So how do we get the value of "t"?  Enter tweenParameter().
 * 
 * We're moving the servos according to a schedule, right?  Well, here's the
 * schedule: at the start of DEPLOYING_SERVICES, "t" is 0.  It then moves to
 * 1, pauses for a bit, and returns to 0.
 * 
 * The beauty of this setup is that we can change the exact amount of time
 * it takes easily - no problem!  This will figure out the tweening parameter,
 * and servoTween() will use that to figure out the right angle.
 * 
 * This is why tweening parameters are really, really useful: they help us
 * define reusable animations.  If we want to use the same tweening parameter
 * to, say, control the NeoPixel light color, we can do that!  All you have
 * to do is write a function, similar to servoTween(), that describes how
 * tweening from one color to the next works.
 * 
 * (Hint: you can do basically what servoTween() does for the red, green,
 * and blue values separately.)
 * 
 */
double tweenParameter(unsigned long timeInState) {
  if (timeInState <= DEPLOYING_SERVICES_MOVE_MS) {
    /*
     * Move "t" from 0 to 1 gradually over the first "movementTime"
     * milliseconds.
     */
    return 1.0 * timeInState / DEPLOYING_SERVICES_MOVE_MS;
  } else if (timeInState <= DEPLOYING_SERVICES_MS - DEPLOYING_SERVICES_MOVE_MS) {
    /*
     * Pause "t" at 1 for 800 ms.
     */
    return 1.0;
  } else {
    /*
     * Move "t" from 1 back to 0 gradually over the last "movementTime"
     * milliseconds.
     */
    return 1.0 * (DEPLOYING_SERVICES_MS - timeInState) / DEPLOYING_SERVICES_MOVE_MS;
  }
}

/*
 * This is where it gets crazy.  Our tweening parameter is a value between 0 and 1,
 * right?  Well, smoothTweenParameter() is a function that takes a value between 0 and
 * 1 and returns another value between 0 and 1.  In other words, it's a function that
 * modifies animations!
 * 
 * It's not just any old function, though - this is the logistic function:
 * 
 * https://en.wikipedia.org/wiki/Logistic_function
 * 
 * Ignoring all the math, look at the curve on the Wikipedia page: this function
 * creates a nice smooth path that starts slow, speeds up in the middle, and slows down
 * again at the end.  We can use this to give our servos a more "lifelike" movement
 * pattern.
 */
double smoothTweenParameter(double t) {
  t = 1 / (1 + exp(-T_K * (t - 0.5)));

  /*
   * Normally the logistic function doesn't reach 0 or 1 until negative or positive 
   * infinity.  We don't have an infinite amount of time to wait for our servos, so we
   * instead pick a smaller piece of the function and just "stretch" it out to
   * return values between 0 and 1.
   */
  return (t - T_MIN) / (T_MAX - T_MIN);
}

void moveServo(unsigned long timeInState) {
  /*
   * See the notes on these functions above for more details.
   */
  double t = tweenParameter(timeInState);
  t = smoothTweenParameter(t);

  int servoLeftAngle = servoTween(t, 90, 180);
  servoLeft.write(servoLeftAngle);

  int servoRightAngle = servoTween(t, 90, 0);
  servoRight.write(servoRightAngle);
}

void resetServo() {
  servoLeft.write(90);
  servoRight.write(90);
}

#define HUMAN_NEARBY_CM 25

void updateState() {
  unsigned long now = millis();
  unsigned long timeInState = now - enteredStateAt;
  
  if (state == SEEKING_HUMAN) {
    double distanceToHuman = measureDistance();
    /*
     * We've given our 25 cm threshold a #define statement, so that we
     * can easily change the threshold later.
     */
    boolean humanNearby = (distanceToHuman <= HUMAN_NEARBY_CM);
    if (humanNearby) {
      state = HUMAN_NEARBY;
      enteredStateAt = now;
    }
  } else if (state == HUMAN_NEARBY) {
    double distanceToHuman = measureDistance();
    boolean humanNearby = (distanceToHuman <= HUMAN_NEARBY_CM);
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
    return 50;
  }
  return 15;
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
