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
//#define DEBUG
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
//#define IIII

// It looks like IIII, but look closer... 2cm off there is another I
// Motor pins are White=+, Brown=Gnd
#define IIIII

// Clockwise
const int
  FRONT = 0,
  RIGHT = 1,
  BACK  = 2,
  LEFT  = 3;
String dirToString(int dir) {
  switch(dir) {
    case FRONT: return "Front";
    case RIGHT: return "Right";
    case BACK:  return "Back";
    case LEFT:  return "Left";
  }
}

bool mamaCalling = false;
int mamaDir = FRONT;
float mamaAngle = 0;
bool happening = false;
bool motorSoundOn = false;
int lastHappening = 0;

//
// Utils
void handleEvents(unsigned long duration) {
  unsigned long start = millis();
  unsigned long now = start;
  do {
    handleSound();
    handleLDRs();
  } while (millis() - start < duration);
}
void sleep(unsigned long duration) {
  handleEvents(duration);
}

int bell(int range, int n) {
  int out = 0;
  for (int i = 0; i < n; i++) {
    out += random(range);
  }
  return out/n;
}

int k = 0;
bool kInc = true;
unsigned long last = 0;
unsigned long lastSoundDuration = 0;
int soundPin = 8;
int lastReason = 0;

// Sound generator
bool soundEnabled = false;
void handleSound() {
  int newReason = 0; {
    if (mamaCalling) newReason = 1; else  
    if (happening) newReason = 2; else
    if (motorSoundOn) newReason = 3;
    else newReason = 4;
  }
  /*if (!soundEnabled) {
    noTone(soundPin);
    return;
  }*/
  if (millis() - last < lastSoundDuration && last != 0 && newReason == lastReason) return;
  lastReason = newReason;

  int duration = bell(6, 7);
  if (mamaCalling) {
    lastSoundDuration = duration+100+random(500)+k/10;
  } else if (happening) {
    lastSoundDuration = duration+300+random(700)+k/10;
  } else if (motorSoundOn) {
    lastSoundDuration = duration+500+random(900)+k;
  } else {
    lastSoundDuration = duration+2000+random(8000)+k;
  }
  
  k += kInc ? random(5) : -random(5);
  int base = 3000;
  int freq = base+k*25+bell(13000, 10);
  if (freq < 10000) {
    tone(soundPin, freq, duration);
  } else {
    noTone(soundPin);
  }
  last = millis();
  if (k > 200) {
    kInc = false;
  } else if (k < 0) {
    kInc = true;
  }
}

int readAnalogPin(int pin) {
  analogRead(pin);
  // Read twice to avoid problems
  int value = (analogRead(pin)+analogRead(pin))/2;
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
int secondMax4Index(int a, int b, int c, int d) {
  int maxInd = max4Index(a,b,c,d);
  int aa = (maxInd == 0) ? b : a;
  int bb = (maxInd == 1) ? c : b;
  int cc = (maxInd == 2) ? d : c;
  int dd = (maxInd == 3) ? a : d;
  return max4Index(aa,bb,cc,dd);
}
int max2Index(int a, int b) {
  if (a >= b) return 0; else return 1;
}

struct Quad {
  int frontPin, rightPin, backPin, leftPin;
  int front, right, back, left;
  int xFront, xRight, xBack, xLeft;
  int min, minIndex, max, maxIndex, avg, diff, diffDir1, diffDir2;
};
void QuadCalcStats(struct Quad* q) {
  q->min      = min4(q->front, q->right, q->back, q->left);
  q->max      = max4(q->front, q->right, q->back, q->left);
  q->minIndex = min4Index(q->front, q->right, q->back, q->left);
  q->maxIndex = max4Index(q->front, q->right, q->back, q->left);
  q->avg      = avg4(q->front, q->right, q->back, q->left);
  //q->diff     = max4(abs(q->front - q->xFront), abs(q->right - q->xRight), abs(q->back - q->xBack), abs(q->left - q->xLeft));
  //q->diff     = q->front-q->xFront + q->right-q->xRight + q->back-q->xBack + q->left-q->xLeft;
  int frontDiff = abs(q->front-q->xFront);
  int rightDiff = abs(q->right-q->xRight);
  int backDiff  = abs(q->back-q->xBack);
  int leftDiff  = abs(q->left-q->xLeft);
  q->diff = frontDiff + rightDiff + backDiff + leftDiff;
  q->diffDir1 = max4Index(frontDiff, rightDiff, backDiff, leftDiff);
  q->diffDir2 = secondMax4Index(frontDiff, rightDiff, backDiff, leftDiff);
  q->xFront   = q->front;
  q->xRight   = q->right;
  q->xBack    = q->back;
  q->xLeft    = q->left;
}

// Global counter
unsigned long cnt = 0;


//
// Photoresistors (LDR07)
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
  A0,A1,A3,A2,
  #endif
  0,0,0,0,     // Values
  0,0,0,0,     // ExValues
  0,0,0,0,0,0,0,0  // Calculated values
};
Quad lastPhoto = photo;
bool photoChanged = false;

int diffDirs[4];
int diffCounter = 0;
int diffSum = 0;
float xDiffAvg = 0, xxDiffAvg = 0;
#define diffLimit 50
unsigned long lastMamaDetection = 0;

void detectMamasCalls() {
  int diff = photo.diff > 3 ? 1 : 0;
  if (diff != 0) {
    diffDirs[photo.diffDir1] += 2;
    diffDirs[photo.diffDir2] += 1;
  }
  diffSum += diff;
  diffCounter++;

  #define borderThresh 0.35
  #define fullThresh 0.45
  
  if (diffCounter > diffLimit) {
    float diffAvg = diffSum / (float)diffLimit;
    //Serial.println("Diff: " + String(diffAvg));
    if (mamaCalling && diffAvg < borderThresh) {
      mamaCalling = false;
      soundEnabled = false;
      xxDiffAvg = 0;
      xDiffAvg = 0;
      diffSum = 0;
    } else if (diffAvg > fullThresh && xDiffAvg > fullThresh && xxDiffAvg > borderThresh && (diffAvg+xDiffAvg+xxDiffAvg)/3 > fullThresh) {
      mamaCalling = true;
      soundEnabled = true;
      mamaDir = max4Index(diffDirs[0], diffDirs[1], diffDirs[2], diffDirs[3]);
      mamaAngle = mamaDir*90;
      // Starting angle
      float leftDiffDirs  = diffDirs[(mamaDir+1)%4];
      float rightDiffDirs = diffDirs[(mamaDir+3)%4];
      float secondDiffDirs = max(leftDiffDirs, rightDiffDirs);
      // Exact angle
      float diffFreq1 = diffDirs[mamaDir];
      float diffFreq2 = secondDiffDirs;
      float diffFreqSum = diffFreq1+diffFreq2;
      float mamaAngleQuadrant = rightDiffDirs > leftDiffDirs ? -90 : 90;
      float mamaAnglePercent = (((diffFreq1 > diffFreq2) ? diffFreq2 : diffFreq1)/diffFreqSum);
      mamaAngle +=  mamaAngleQuadrant * mamaAnglePercent;
      if (mamaAngle < 0) mamaAngle += 360;
      if (mamaAngle > 360) mamaAngle -= 360;
      
      Serial.println(diffAvg);
      Serial.println(diffDirs[0]);
      Serial.println(diffDirs[1]);
      Serial.println(diffDirs[2]);
      Serial.println(diffDirs[3]);
      Serial.println(mamaAngle);
      Serial.println("Mama detected in direction: " + dirToString(mamaDir));
      diffDirs[0] = 0;
      diffDirs[1] = 0;
      diffDirs[2] = 0;
      diffDirs[3] = 0;
    }
    xxDiffAvg = xDiffAvg;
    xDiffAvg = diffAvg;
    diffSum = 0;
    diffCounter = 0;
  }
}

unsigned long lastLDRTime = 0;
void handleLDRs() {
  unsigned long now = millis();
  if (now-lastLDRTime < 20) return;
  lastLDRTime = now;

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

  detectMamasCalls();
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
  0,0,0,0,  // ExValues
  0,0,0,0,  // Values
  0,0,0,0,0,0,0,0 // Calculated values
};
Quad lastSonar = sonar;
bool sonarChanged = false;

int readSonar(ST_HW_HC_SR04 sonar, int lastDistance) {
  // Delay to eliminate echoes and accidentally synced sonars
  sleep(random(10, 15));
  int distance = sonar.getHitTime() / 29;
  if (distance <= 0) {
    sleep(random(10, 15));
    distance = sonar.getHitTime() / 29;
  }
  if (distance <= 0) {
    sleep(random(10, 15));
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

    if (blinky == 0 && random(100) > 8) {//98
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
  motorSoundOn = (action != STOP);
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


//////////////////
/// SETUP PART ///
//////////////////

void setup() {
  Serial.begin(115200);

  // Init LDRs
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

/////////////////
/// LOOP PART ///
/////////////////

void loop() {
    #define MOTOR_TEST
    #ifdef MOTOR_TEST
      controlMotors(GO_FORWARD);
      sleep(10000);

      controlMotors(STOP);
      sleep(10000);

      /*controlMotors(GO_LEFT);
      delay(10000);
      
      controlMotors(STOP);
      delay(15000);
      
      controlMotors(GO_FORWARD);
      delay(10000);
      
      controlMotors(STOP);
      delay(15000);

      controlMotors(GO_RIGHT);
      delay(10000);
      
      controlMotors(STOP);
      delay(15000);
      return;*/
    #endif

  //#define LIGHT_TEST
  #ifdef LIGHT_TEST
    Serial.println("Light");
    //digitalWrite(13, HIGH);
    analogWrite(ledPin1, 255);
    analogWrite(ledPin2, 255);

    delay(100);
    Serial.println("No Light");
    //digitalWrite(13, LOW);
    analogWrite(ledPin1, 0);
    analogWrite(ledPin2, 0);
    delay(100);
    if (cnt % 50 == 0) {
      delay(10000);
    }
    cnt++;
    return;
  #endif

  handleEvents(100);
  cnt++;
  handleSonars();
  handleEvents(10);

  // Control outputs
  #ifndef DisableLED
  handleLed();
  handleEvents(10);
  #endif
  #ifndef DisableMotors
  handleMotors();
  #endif
}

