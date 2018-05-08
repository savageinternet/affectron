/**
 * Affectron by Savage Internet
 * HUMAN, I <arms?>
 * 
 * In this step, you attach an SG90 servo, get it to move, and then make it respond
 * along with the lights to readings from the range sensor.
 * 
 * Most motors just rotate continuously; servos instead move to a specific angle
 * and stay there.
 * 
 * The Servo library is already installed by default, so you don't need to install
 * anything new to get that working.
 */

#include <Adafruit_NeoPixel.h>
#include <Servo.h>

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

#define NEOPIXEL_NUM_PIXELS 16
#define NEOPIXEL_PIN 4
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NEOPIXEL_NUM_PIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

// SERVOS

/*
 * Pin for our first servo!  This pin controls the angle that the servo moves to; see the
 * next comment for more details.
 */
#define SERVO_PIN A0

/*
 * The Servo library uses Pulse Width Modulation (PWM) to control the servo, which involves
 * alternating quickly between LOW and HIGH states in short "pulses".  See
 * https://learn.sparkfun.com/tutorials/pulse-width-modulation for more details on PWM!
 * 
 * For servos, the PWM "cycle" is 20ms long, and the "duty cycle" (or length of the HIGH
 * part of the pulse) varies depending on what angle you want to move the servo to.  We've
 * determined the values SERVO_PWM_MIN, SERVO_PWM_MAX below by experimentation - so a
 * duty cycle of 850us every 20ms moves the servo to one end of its range, and a
 * duty cycle of 1400us every 20ms moves the servo to the other end.
 * 
 * See http://akizukidenshi.com/download/ds/towerpro/SG90.pdf for more details on this
 * type of motor.
 */
#define SERVO_PWM_MIN 850
#define SERVO_PWM_MAX 1400

Servo servo;

// GLOBAL VARIABLES

unsigned long startedSeeingHumanAt = 0;
int pixelsToLight = 0;

// SETUP AND LOOP

void setup() {
  Serial.begin(9600);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  strip.setBrightness(24);
  strip.begin();
  
  /*
   * Attach the servo to the servo pin.  This sets pinMode() properly, and it
   * tells the Servo library how to map the PWM duty cycle to servo angles.
   */
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

void loop() {
  moveServo();
  double distanceToHuman = measureDistance();
  boolean humanNearby = (distanceToHuman <= 25);
  displayColors(humanNearby);
  delay(100);
}
