/**
 * Affectron by Savage Internet
 * HUMAN, WHERE ARE YOU?
 * 
 * In this step, you set up the HC-SR04, an ultrasonic range sensor.
 * 
 * If you're curious to know more about the HC-SR04, you can always check its datasheet!
 * Every electronic component we'll be working with has a datasheet, which details exactly
 * how it works.  The HC-SR04 datasheet is available here:
 * 
 * https://www.electroschematics.com/wp-content/uploads/2013/07/HC-SR04-datasheet-version-2.pdf
 */

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
 * According to the datasheet, the Echo pin will be reset after 38ms = 38000us if
 * no object is detected.  Let's round that up to 40ms = 40000us.
 */
#define ECHO_TIMEOUT_US 40000

// SETUP AND LOOP

void setup() {
  /*
   * Serial.begin() lets us send data to the Serial Monitor.  This is useful for
   * double-checking that things make sense.
   */
  Serial.begin(9600);
  
  /*
   * We'll be writing to the Trig pin and reading from the Echo pin.  Anything we
   * write to is an OUTPUT; anything we read from is an INPUT.
   */
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  /*
   * TODO 0.1a: make the onboard LED blink if an object is closer than 10 cm.  See
   * the Blink example (File > Examples > 01.Basics > Blink) if you get stuck!
   */
}

void loop() {
  /*
   * To use the HC-SR04, we have to first send out a pulse.  We do this by setting the
   * Trig pin high for a *very* short period of time.
   * 
   * This is an ultrasonic range sensor, so the pulse is actually a high-frequency sound
   * wave!  This will be important down below.
   */
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  /*
   * Once we send out the pulse, we wait for it to bounce off the nearest object.
   * When the HC-SR04 sends out the pulse, it sets the Echo pin HIGH, then sets the
   * Echo pin LOW again once it detects that the pulse has returned.
   * 
   * pulseIn() times this HIGH-LOW cycle, and returns the time it takes in microseconds.
   * (That's why we used microseconds for ECHO_TIMEOUT_US!)
   * 
   * You can read more about pulseIn() in the Arduino reference:
   * 
   * https://www.arduino.cc/reference/en/language/functions/advanced-io/pulsein/
   */
  long duration = pulseIn(ECHO_PIN, HIGH, ECHO_TIMEOUT_US);
  if (duration == 0) {
    /*
     * pulseIn() returns 0 if it doesn't receive a pulse before the timeout.
     */
    duration = ECHO_TIMEOUT_US;
  }

  /*
   * TODO 0.0: we need to do some PHYSICS MATH to convert our duration into distance.  See
   * the Human Explanation Document for details, then fix the distance calculation
   * below!
   */
  double distance = duration;
  Serial.print(distance);
  Serial.println(" cm???");

  /*
   * TODO 0.1b: make the onboard LED blink if an object is closer than 25 cm.  See
   * the Blink example (File > Examples > 01.Basics > Blink) if you get stuck!
   */
  
  delay(100);
}

