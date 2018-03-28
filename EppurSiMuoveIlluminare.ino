/*
This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <ST_HW_HC_SR04.h>

//Enables debug output
#define DEBUG
//#define LIGHT_TEST
//#define MOTOR_TEST

// Define the robot you're currently working with
// See markings on the inside of the base circle

// Mama!
//#define I

// Works, but had two busted Arduinos already, and has a cracked top
// Also now light sensors are dead (1023 all the time)
//BN: Unknown board
//VID: 1A86
//PID: 7523
//SN: Upload any sketch to obtain it
//#define II

// Works
//BN: Arduino/Genuino Uno
//VID: 2341
//PID: 0243
//SN: 75533353837351104041
//#define III

// Works
#define IIII

// Motors don't work... best to disable them
// It looks like IIII, but look closer... 2cm off there is another I
//#define IIIII
//#define DisableMotors

//
// Utils
int readAnalogPin(int pin) {
  // Read twice to avoid problems
  analogRead(pin);
  int value = analogRead(pin);
  #ifdef DEBUG
  Serial.println(value);
  #endif
  return value;
}

bool randomBool() {
  return random(1) > .5;
}

int avg4(int a, int b, int c, int d) {
  return (a + b + c + d)/4;
}
int min3(int a, int b, int c) {
  return min(a, min(b, c));
}
int min4(int a, int b, int c, int d) {
  return min(a, min3(b, c, d));
}
int min4Index(int a, int b, int c, int d) {
  int minValue = min4(a, b, c, d);
  if (minValue == a) return 0;
  if (minValue == b) return 1;
  if (minValue == c) return 2;
  return 3;
}
int max3(int a, int b, int c) {
  return max(a, max(b, c));
}
int max4(int a, int b, int c, int d) {
  return max(a, max3(b, c, d));
}
int max4Index(int a, int b, int c, int d) {
  int maxValue = max4(a, b, c, d);
  if (maxValue == a) return 0;
  if (maxValue == b) return 1;
  if (maxValue == c) return 2;
  return 3;
}

// Clockwise
const int
  FRONT = 0,
  RIGHT = 1,
  BACK  = 2,
  LEFT  = 3;
struct Quad {
  int frontPin, rightPin, backPin, leftPin;
  int front, right, back, left;
  int min, minIndex, max, maxIndex, avg;
};
void QuadCalcStats(struct Quad* q) {
  q->min      = min4(q->front, q->right, q->back, q->left);
  q->max      = max4(q->front, q->right, q->back, q->left);
  q->minIndex = min4Index(q->front, q->right, q->back, q->left);
  q->maxIndex = max4Index(q->front, q->right, q->back, q->left);
  q->avg      = avg4(q->front, q->right, q->back, q->left);
}

// Global counter
unsigned long cnt = 0;


//
// Photoresistors
// https://hairyfotr.psywerx.org/share/2017-ZoranStebri/orientation.png
// Orange, Blue, Blue/White, Orange/White wires
// front, right, back, left
Quad photo = {
  // Pins
  #ifdef I
  A0,A1,A2,A3,
  #endif
  #ifdef II
  A0,A3,A2,A1,
  #endif
  #ifdef III
  A3,A2,A0,A1,
  #endif
  #ifdef IIII
  A1,A0,A3,A2,
  #endif
  #ifdef IIIII
  A3,A2,A0,A1,
  #endif
  0,0,0,0,     // Values
  0,0,0,0,0    // Calculated values
};
Quad lastPhoto = photo;
bool photoChanged = false;

void handlePhotoResistors() {
  lastPhoto = photo;

  #ifdef DEBUG
  Serial.println("---");
  #endif
  // Read values
  photo.front = readAnalogPin(photo.frontPin);
  photo.right = readAnalogPin(photo.rightPin);
  photo.back  = readAnalogPin(photo.backPin);
  photo.left  = readAnalogPin(photo.leftPin);

  QuadCalcStats(&photo);

  photoChanged = abs(lastPhoto.max - photo.max) > 25;
}


//
// Sonars
ST_HW_HC_SR04
  //(TRIG, ECHO)
  #ifdef I
  sonarFront(2, 3), sonarRight(A4, A5), sonarLeft(4, 5), sonarBack(6, 7);
  #endif
  #ifdef II
  sonarFront(2, 3), sonarRight(A4, A5), sonarLeft(4, 5), sonarBack(6, 7);
  #endif
  #ifdef III
  sonarFront(A4, A5), sonarRight(6, 7), sonarBack(4, 5), sonarLeft(2, 3);
  #endif
  #ifdef IIII
  sonarFront(6, 7), sonarRight(A4, A5), sonarBack(4, 5), sonarLeft(2, 3);
  #endif
  #ifdef IIIII
  sonarFront(A4, A5), sonarRight(6, 7), sonarBack(4, 5), sonarLeft(2, 3);
  #endif
Quad sonar = {
  0,0,0,0,  // Pins (... but not used here)
  0,0,0,0,  // Values
  0,0,0,0,0 // Calculated values
};
Quad lastSonar = sonar;
bool sonarChanged = false;

int readSonar(ST_HW_HC_SR04 sonar, int lastDistance) {
  // Delay to eliminate echoes and the accidentally synced
  delay(random(20, 40));
  int distance = sonar.getHitTime() / 29;
  if (distance <= 0) {
    delay(10);
    distance = sonar.getHitTime() / 29;
  }
  if (distance <= 0) {
    delay(10);
    distance = sonar.getHitTime() / 29;
  }
  if (distance <= 0 && lastDistance > 0 && lastDistance < 400) {
    // Maybe it was right, but slowly go towards 400 in case it's bad
    distance = (lastDistance + 401)/2;
  }
  if (distance <= 0 || distance > 400) {
    distance = 400;
  }
  #ifdef DEBUG
  Serial.println(distance);
  #endif

  return distance;
}

void handleSonars() {
  lastSonar = sonar;

  #ifdef DEBUG
  Serial.println("+++");
  #endif
  // Read values
  sonar.front = readSonar(sonarFront, sonar.front);
  sonar.right = readSonar(sonarRight, sonar.right);
  sonar.back  = readSonar(sonarBack, sonar.back);
  sonar.left  = readSonar(sonarLeft, sonar.left);

  QuadCalcStats(&sonar);

  sonarChanged = abs(lastSonar.max - sonar.max) > 15;
}


//
// LED
// must be a PWM pin
// using two pins because we fried some :)
int ledPin1 = 9;
int ledPin2 = 10;
int led = 0;

#ifdef I
int blinkySteps = 25;
int blinky = blinkySteps;
#endif

void controlLed(int led) {
  analogWrite(ledPin1, led);
  analogWrite(ledPin2, led);
}

void handleLed() {
  // Some have 22kOhm, and some 33kOhm dividing resistors
  #ifdef I
  int newLed = constrain(map(photo.max, 250,700, 0,255), 0, 255);
  #endif
  #ifdef II
  int newLed = constrain(map(photo.max, 520,800, 0,200), 0, 255);
  #endif
  #ifdef III
  int newLed = constrain(map(photo.max, 380,700, 0,200), 0, 255);
  #endif
  #ifdef IIII
  int newLed = constrain(map(photo.max, 520,800, 0,200), 0, 255);
  #endif
  #ifdef IIIII
  int newLed = constrain(map(photo.max, 520,800, 0,200), 0, 255);
  #endif

  // Slow accumulation / falldown of LED values
  int weight = 20;
  led = (led*weight + newLed)/(weight+1);

  #ifdef I
    //Mama

    // Ideas:
    /// Mama blinks and attracts younglings.
    /// Would need history and pattern detection (nyquist-shannon = freq < 2x detection rate)
    /// Or simply: count the number of changes in the last 50 measurements

    if (blinky == 0 && random(100) > 98) {
      blinky = blinkySteps;
    }
    if (blinky > 0) {
      blinky--;
      controlLed((blinky % 2) ? 255 : 0);
      if (blinky == 0) {
        led = 0;
      }
    } else {
      controlLed(led/2);
    }
  #else
    // The others
    controlLed(led);
  #endif
}


//
// Motor
// see https://electronics.stackexchange.com/a/179128/155849 for basic wiring
const int
  motorRightPin = 12,
  motorLeftPin1 = 11,
  motorLeftPin2 = 13; // Also fried a few pins...
const int
  STOP       = 0,
  GO_LEFT    = 1,
  GO_RIGHT   = 2,
  GO_FORWARD = 3;
int motorAction = STOP;
int keepCountDown = 0;

bool motorOn = false;
unsigned long
  motorOnTime = millis(), motorOnDuration = 0,
  motorStopTime = millis(), motorStopDuration = 0;

void controlMotors(int action) {
  digitalWrite(motorLeftPin1, (action & 1) ? HIGH : LOW);
  digitalWrite(motorLeftPin2, (action & 1) ? HIGH : LOW);
  digitalWrite(motorRightPin, (action & 2) ? HIGH : LOW);
}

void handleMotors() {
  int quiteClose = 150;
  int quiteBright = 850;

  bool tooClose = sonar.min <= 70;
  bool tooBright = photo.max >= 930;

  bool forwardIsNoGood = sonar.front < quiteClose || sonar.minIndex == FRONT || sonar.minIndex != FRONT;

  if (tooClose || tooBright) {
    motorAction = STOP;
    keepCountDown = 7;
  } else if (keepCountDown > 0) {
    // Keep on doing the previous action
    keepCountDown--;
  } else if (forwardIsNoGood) {
    bool sonarValuesClose = abs(sonar.left - sonar.right) < 30 || sonar.left == 400 || sonar.right == 400;
    if (sonarValuesClose) {
      if (photo.minIndex == BACK && sonar.max == sonar.back) {
        // Rotate towards current back position
        int randomDirection = randomBool() ? GO_LEFT : GO_RIGHT;
        motorAction = randomDirection;
        keepCountDown = 7;
      } else {
        bool photoValuesClose = abs(photo.left - photo.right) < 35;
        bool nothingAhead = sonar.front > quiteClose;
        if (photoValuesClose && nothingAhead) {
          motorAction = GO_FORWARD;
          keepCountDown = 3;
        } else {
          motorAction = (photo.left < photo.right) ? GO_LEFT : GO_RIGHT;
        }
      }
    } else {
      motorAction = (sonar.left > sonar.right) ? GO_LEFT : GO_RIGHT;
    }
  } else {
    motorAction = GO_FORWARD;
  }

  // Only move motors some of the time
  unsigned long now = millis();
  if (motorOn && now-motorOnTime > motorOnDuration) {
    motorOn = false;
    motorStopTime = now;
    // How long it's going to be stopped
    motorStopDuration = random(15000, 90000);
    #ifdef II
    // II had a few problems with vibration (killed an arduino, and cracked the top)
    // ... so we move it a bit less
    motorStopDuration = random(30000, 190000);
    #endif
  } else if (!motorOn && now-motorStopTime > motorStopDuration) {
    motorOn = true;
    motorOnTime = now;
    // How long it's going to run
    motorOnDuration = random(8000, 15000);
    #ifdef II
    motorOnDuration = random(7000, 14000);
    #endif
  }

  // If there is something happening, shorten the time to start motors
  if (photoChanged && sonarChanged && now-motorStopTime < motorStopDuration && motorStopDuration > 1000) {
    motorStopDuration -= 1000;
  }

  controlMotors(motorOn ? motorAction : STOP);
}


/////////////////
/// MAIN PART ///
/////////////////

void setup() {
  Serial.begin(9600);

  // Init photoresistors
  pinMode(photo.frontPin, INPUT);
  pinMode(photo.rightPin, INPUT);
  pinMode(photo.backPin,  INPUT);
  pinMode(photo.leftPin,  INPUT);

  // Init sonars
  // default: 5000us, which is only ~75cm
  // Pins are initialized by the libraries
  int timeout = 30000;
  sonarFront.setTimeout(timeout);
  sonarLeft.setTimeout(timeout);
  sonarRight.setTimeout(timeout);
  sonarBack.setTimeout(timeout);

  // Init Motors
  pinMode(motorLeftPin1, OUTPUT);
  pinMode(motorLeftPin2, OUTPUT);
  pinMode(motorRightPin, OUTPUT);

  // Init LED
  pinMode(ledPin1, OUTPUT);
  pinMode(ledPin2, OUTPUT);
}

void loop() {
  delay(100);
  cnt++;

  #ifdef LIGHT_TEST
    Serial.println("Light");
    analogWrite(ledPin1, 255);
    analogWrite(ledPin2, 255);

    delay(2000);
    Serial.println("No Light");
    //digitalWrite(13, LOW);
    analogWrite(ledPin1, 0);
    analogWrite(ledPin2, 0);
  #endif

  #ifdef MOTOR_TEST
    Serial.println("Left");
    digitalWrite(motorLeftPin1, HIGH);
    digitalWrite(motorLeftPin2, HIGH);
    delay(2000);
    digitalWrite(motorLeftPin1, LOW);
    digitalWrite(motorLeftPin2, LOW);
    digitalWrite(motorRightPin, LOW);

    delay(500);

    Serial.println("Right");
    digitalWrite(motorRightPin, HIGH);
    delay(2000);
    digitalWrite(motorLeftPin1, LOW);
    digitalWrite(motorLeftPin2, LOW);
    digitalWrite(motorRightPin, LOW);

    delay(500);

    Serial.println("Both");
    digitalWrite(motorLeftPin1, HIGH);
    digitalWrite(motorLeftPin2, HIGH);
    digitalWrite(motorRightPin, HIGH);
    delay(2000);
    Serial.println("None");
    digitalWrite(motorLeftPin1, LOW);
    digitalWrite(motorLeftPin2, LOW);
    digitalWrite(motorRightPin, LOW);

    delay(500);
  #endif

  // Read inputs
  handlePhotoResistors();
  handleSonars();

  // Control outputs
  #ifndef DisableLED
  handleLed();
  #endif
  #ifndef DisableMotors
  handleMotors();
  #endif
}

