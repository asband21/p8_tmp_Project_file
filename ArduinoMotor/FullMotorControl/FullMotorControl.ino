#include <DS1804.h>

//USING DIGITAL POTENTIOMETER:
#define BWDMAXSPEEDPWM 13                 
#define FWDMAXSPEEDPWM 97                 

#define TOPTOBOTFWDCUT 62  //ON TO OFF FORWARD CUT OUT
#define BOTTOTOPFWDCUT 63 //OFF TO ON FORWARD CUT OUT
#define TOPTOBOTBWDCUT 24  //OFF TO ON BWD CUT OUT
#define BOTTOTOPBWDCUT 25//ON TO OFF BWD CUT OUT
#define RELAYDELAY 500
#define STOPPWM 45
//digital potentiometer pins
#define CSPIN 53
#define INCPIN 49
#define UDPIN 51

DS1804 digipot = DS1804(CSPIN, INCPIN, UDPIN, DS1804_HUNDRED); //Digital potentiometer for front motor

unsigned long rateLimit = 1000;	// time in ms required between changes in digi pot
long lastLockedAt = -1*rateLimit;
byte lastWiperPosition;
bool digipotNeedsUpdate = true;
byte digipotUpdatePosition = 45;

// END OF DIGITAL POTENTIOMETER

/*
//Using RC filter  1.3kohm and 220uF
#define BWDMAXSPEEDPWM 46                  //REVERSE RANGE 79-46=33
#define FWDMAXSPEEDPWM 255                 // FORWARD RANGE 255-158=97

#define TOPTOBOTFWDCUT 158 //ON TO OFF FORWARD CUT OUT
#define BOTTOTOPFWDCUT 168 //167   //OFF TO ON FORWARD CUT OUT
#define TOPTOBOTBWDCUT 77  //78 //OFF TO ON BWD CUT OUT
#define BOTTOTOPBWDCUT 79 //ON TO OFF BWD CUT OUT
#define RELAYDELAY 500
#define STOPPWM 128

#define FWDOFFSETCORRECT 0.000001 //DISABLED

// END OF RC FILTER
*/

#define RMOTORPROP 13  // PWM for Motor 1       
#define LMOTORPROP 8  // PWM for Motor 2
#define SPEED_PIN_ESCON_R  11 //Pin for angle velocity right motor
#define SPEED_PIN_ESCON_L  12 //Pin for angle velocity
#define ENCODER_A_PIN_R 20  // Channel A left motor
#define ENCODER_B_PIN_R 21  // Channel B left motor
#define ENCODER_A_PIN_L 24  // Channel A right motor
#define ENCODER_B_PIN_L 25  // Channel B right motor

#define CW_PIN_R 32 // Clockwise motor pin right motor
#define CCW_PIN_R 33 // Counter-clockwise motor pin right motor

#define CW_PIN_L 34 // Clockwise motor pin left motor
#define CCW_PIN_L 35 // Counter-clockwise motor pin left motor

#define ALLOWEDERRORINNER 0.05
#define ALLOWEDERROROUTER 10 

//ESCON angular velocity PWMS
#define MAXSPEEDPWM 229
#define MIDSPEEDPWM 140
#define MINSPEEDPWM 26


bool trackingEnabled = false;
volatile bool updateGoals = false;  // NEW ANGLE FLAG


volatile long positionCountR = 0;
volatile long positionCountL = 0;

const float ppr = 5173;  // Pulses per revolution of encoder 10347og    2587
float goalAngleR = 0;
float goalAngleL = 0;
const float Kp = 1.3;  // Proportional gain for speed control

long int errorEncoderTrackingOuter = ppr * ALLOWEDERROROUTER / 360;
long int errorEncoderTrackingINNER = ppr * ALLOWEDERRORINNER / 360;

  long int goalPositionCountR=0;
  long int goalPositionCountL=0;


  int loope = 0;

  char commandString[50];
  int angleR=0;
  int angleL=0;
  char directionF='f';
  int throttleR=0;
  int throttleL=0;
  int throttleF=0;

void setup() {
  Serial.begin(115200);

  digipot.setToZero();
  digipot.setWiperPosition(STOPPWM);

  //Right motor
  pinMode(RMOTORPROP, OUTPUT);
  pinMode(ENCODER_A_PIN_R, INPUT);
  pinMode(ENCODER_B_PIN_R, INPUT);
  pinMode(SPEED_PIN_ESCON_R, OUTPUT);

  analogWrite(RMOTORPROP, 0);

  //Left motor
  pinMode(LMOTORPROP, OUTPUT);
  pinMode(ENCODER_A_PIN_L, INPUT);
  pinMode(ENCODER_B_PIN_L, INPUT);
  pinMode(SPEED_PIN_ESCON_L, OUTPUT);

  analogWrite(LMOTORPROP, 0);

  pinMode(CCW_PIN_R, OUTPUT);
  pinMode(CW_PIN_R, OUTPUT);
  pinMode(CCW_PIN_L, OUTPUT);
  pinMode(CW_PIN_L, OUTPUT);

  analogWrite(SPEED_PIN_ESCON_R, MINSPEEDPWM);
  analogWrite(SPEED_PIN_ESCON_L, MINSPEEDPWM);

  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN_R), updatePositionR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN_L), updatePositionL, RISING);

//digipot
 lastWiperPosition = digipot.getWiperPosition();


// TESTER INTERRUPT
  pinMode(19, INPUT_PULLUP); 
  attachInterrupt(digitalPinToInterrupt(19), updateGoalAngles, FALLING);
  randomSeed(analogRead(0));  // Seed for randomness (only for testing)

}

void loop() {

  if ( digipot.isLocked() ) {
    if (millis() - lastLockedAt > rateLimit) {
      digipot.unlock();
      if(digipotNeedsUpdate)
        setWiperPositionRateLimited(digipotUpdatePosition);
    }
  }

  // CHECK SERIAL HERE INSTEAD OF FLAG
  if (updateGoals) {
    int result = sscanf(commandString, "%c %d %d %d %d %d", 
                        &directionF, 
                        &throttleF, 
                        &angleR, 
                        &throttleR, 
                        &angleL, 
                        &throttleL);
      

    //SHOULD WE DISABLE PROP SPEED WHILE ROTATING?
    if(result == 6){
      Serial.println(commandString);
      setPropSpeedFront(directionF, throttleF);
      setPropSpeedRear('r', throttleR); 
      //setPropSpeedRear('r', 100); //remove this one after testing, just avoiding noise on the lab
      setPropSpeedRear('l', throttleL);
      goalAngleR=wrapAngle(angleR);
      goalAngleL=wrapAngle(angleL);
      goalPositionCountR = -(goalAngleR * ppr / 360);
      goalPositionCountL = -(goalAngleL * ppr / 360);
      updateGoals=false;
    }
    else
      Serial.println("Serial fail");
  }
  int errorR = goalPositionCountR - positionCountR;
  int errorL = goalPositionCountL - positionCountL;

if(loope==100){
  Serial.println(":::");
  Serial.println(errorR*360/ppr);
  Serial.println(goalPositionCountR*360/ppr);
  Serial.println(positionCountR*360/ppr);

  // Serial.println(errorR);
  // Serial.println(goalPositionCountR);
  // Serial.println(positionCountR);
  loope=0;
  }

  // CHECK FRONT MOTOR ROTATION
  // PUT A RATE LIMIT AND A FLAG TO KNOW IF THE LAST VALUE HAS BEEN CHANGED OR NOT MAYBE RATE LIMIT OF 1 SECOND
  // DIGI POTENTIOMETER KINDA SUCKS
  // MIGHT STILL GO TO LOW PASS FILTER


  // Right Motor Control
  if (abs(errorR) > errorEncoderTrackingINNER) { //WE CAN MAKE A FLAG TO ONLY GO HERE IF WE GET OUTSID THE ALLOWEDERROROUTER
    controlMotor(errorR, CW_PIN_R, CCW_PIN_R, SPEED_PIN_ESCON_R);
  } else {
    stopMotor(CW_PIN_R, CCW_PIN_R, SPEED_PIN_ESCON_R);
  }
  // Left Motor Control
  if (abs(errorL) > errorEncoderTrackingINNER) {
    controlMotor(errorL, CW_PIN_L, CCW_PIN_L, SPEED_PIN_ESCON_L);
  } else {
    stopMotor(CW_PIN_L, CCW_PIN_L, SPEED_PIN_ESCON_L);
  }
  loope++;
  delay(10);  // Small delay to stabilize
}

bool setWiperPositionRateLimited(byte position) {
  if (digipot.isLocked()) {
    digipotNeedsUpdate = true;
    digipotUpdatePosition = position;
    return false;  // Do nothing if locked
  }
  // Unlock and set the position
  digipot.unlock();
  digipot.setWiperPosition(position);
  digipot.lock();
  // Update the lock timestamp
  digipotNeedsUpdate = false;
  lastLockedAt = millis();
  return true;
}


void setPropSpeedFront(char direction, int throttle){
  if(throttle>100)
    throttle=100;
  if(throttle<0)
    throttle=0;

  if(throttle==0 || direction=='s') //STOP
    stopFront();
  else{
    if(direction=='b')//backwards
      movebwd(throttle);
    if(direction=='f')//forward
      movefwd(throttle);
  }
}

void stopFront(){
  setWiperPositionRateLimited(STOPPWM);
}

void movefwd(int throttle){
  int digi_value= map(throttle, 0, 100, TOPTOBOTFWDCUT, FWDMAXSPEEDPWM);
  if(digi_value<BOTTOTOPFWDCUT){
    if(setWiperPositionRateLimited(BOTTOTOPFWDCUT)){
      delay(RELAYDELAY);
      digipot.setWiperPosition(digi_value);
    }
  }
    setWiperPositionRateLimited(digi_value);
}

void movebwd(int throttle){
  int digi_value= map(throttle, 0, 100, TOPTOBOTBWDCUT, BWDMAXSPEEDPWM);
  if(digi_value>BOTTOTOPBWDCUT){
    if(setWiperPositionRateLimited(BOTTOTOPBWDCUT)){
      delay(RELAYDELAY);
      digipot.setWiperPosition(digi_value);
    }
  }
  setWiperPositionRateLimited(digi_value);
}

/*
void movefwd(int throttle){
  uint8_t pwm_value= map(throttle, 0, 100, TOPTOBOTFWDCUT, FWDMAXSPEEDPWM);
  if(pwm_value<BOTTOTOPFWDCUT){
    analogWrite(pwmPin, BOTTOTOPFWDCUT);
    delay(RELAYDELAY);
  }
  analogWrite(pwmPin, pwm_value);
    Serial.println("Forward:");
    Serial.println(pwm_value);
}

void movebwd(int rpm){
  uint8_t pwm_value= map(rpm, 0, -MAXROTATIONBWD, TOPTOBOTBWDCUT, BWDMAXSPEEDPWM);
  if(pwm_value>BOTTOTOPBWDCUT){
    analogWrite(pwmPin, BOTTOTOPBWDCUT);
    delay(RELAYDELAY);
  }
  analogWrite(pwmPin, pwm_value);
    Serial.println("Backward:");
    Serial.println(pwm_value);
}

*/

void setPropSpeedRear(char motor,int throttle){
  if(throttle>100)
    throttle=100;
  if(throttle<0)
    throttle=0;

  int pwm = map(throttle, 0, 100, 0, 255);
  if(motor=='r')
    analogWrite(RMOTORPROP, pwm);
  if(motor=='l')
    analogWrite(LMOTORPROP, pwm);
}

float getDeltaRotation(float currentAngle, float goalAngle) { //Chatgpt did this one but if you want you can debug yours
  float delta = goalAngle - currentAngle;
  if (delta > 180) {
    delta -= 360; 
  } else if (delta < -180) {
    delta += 360;  
  }
  return delta;
}

void updatePositionR() {
  int bState = digitalRead(ENCODER_B_PIN_R);
    if (bState == LOW){
      positionCountR++; // Clockwise
    } else {
      positionCountR--; // Counter-clockwise
    }
}

void updatePositionL() {
  int bState = digitalRead(ENCODER_B_PIN_L);
    if (bState == LOW){
      positionCountL++; // Clockwise
    } else {
      positionCountL--; // Counter-clockwise
    }
}

void updateGoalAngles() {
  char directionmotf;

  int t1, t2, tf;
  int a1, a2;
  a1= random(0,360);
  a2=random(0,360);
  t1= random(0,101);
  t2= random(0,101);
  tf= random(0,101);
  // Serial.println(tf);
  // Serial.println(a1);
  // Serial.println(t1);
  // Serial.println(a2);
  // Serial.println(t2);
  if(random(0,2)==1)
    directionmotf='f';
  else 
    directionmotf='b';//                                     throttle Front   ang r right  thr r right   ang r LEFt       thr r left
  //sprintf(commandString, "%c %d %d %d %d %d", directionmotf, random(0,101), random(0,360), random(0,101), random(0,360), random(0,101));
  sprintf(commandString, "%c %d %d %d %d %d", directionmotf, tf, a1, t1, a2, t2);
  updateGoals = true;
  //Serial.println(commandString);
}

void controlMotor(float error, int cwPin, int ccwPin, int speedPin) {
  int pwmSpeed = constrain(map(abs(error) * Kp, 0, ppr/2, MINSPEEDPWM, MAXSPEEDPWM), MINSPEEDPWM, MAXSPEEDPWM);

  if (error > 0) {  // Clockwise
    analogWrite(speedPin, pwmSpeed);
    digitalWrite(ccwPin, LOW);
    digitalWrite(cwPin, HIGH);
  } else {          // Counter-clockwise
    analogWrite(speedPin, pwmSpeed);
    digitalWrite(cwPin, LOW);
    digitalWrite(ccwPin, HIGH);
  }
}

void stopMotor(int cwPin, int ccwPin, int speedPin) {
  digitalWrite(cwPin, LOW);
  digitalWrite(ccwPin, LOW);
  analogWrite(speedPin, MINSPEEDPWM);  // Minimum speed or stop
}

float wrapAngle(float angle) {
  angle = fmod(angle + 180, 360);  // Wrap within [0, 360)
  if (angle < 0) angle += 360;     // Ensure positive result
  return angle - 180;              // Shift to [-180, 180]
}

void returnToHome(){
  trackingEnabled = false; 
  //gotoAngle(0);

}





void gotoAngle(float goalAngle, volatile long &positionCount, int cwPin, int ccwPin, int speedPin) {
  long int goalPositionCount= -(goalAngle*ppr/360);
  analogWrite(speedPin, MIDSPEEDPWM);
  while (!(positionCount > goalPositionCount - errorEncoderTrackingINNER  &&
           positionCount <= goalPositionCount + errorEncoderTrackingINNER)) {
    if (positionCount < goalPositionCount) {
      digitalWrite(cwPin, LOW);
      digitalWrite(ccwPin, HIGH);
    } else if (positionCount > goalPositionCount) {
      digitalWrite(ccwPin, LOW);
      digitalWrite(cwPin, HIGH);
    }
  }
  digitalWrite(cwPin, LOW);
  digitalWrite(ccwPin, LOW);
}

void refTrackingLoop() {

   goalPositionCountR = -(goalAngleR * ppr / 360);
   goalPositionCountL = -(goalAngleL * ppr / 360);

  bool skipR= false;
  bool skipL= false;
  gotoAngle(goalAngleR, positionCountR, CW_PIN_R, CCW_PIN_R, SPEED_PIN_ESCON_R);  
  gotoAngle(goalAngleL, positionCountL, CW_PIN_L, CCW_PIN_L, SPEED_PIN_ESCON_L);  
  while (trackingEnabled) {  // Stay in loop as long as tracking is enabled
    // Check for stop command to exit tracking loop
    if (Serial.available() > 0) {
      String input = Serial.readStringUntil('\n');
      input.trim();
      if (input == "stop") {
        trackingEnabled = false;
        break;
      }
    }

    if (updateGoals) {

    goalPositionCountR = -(goalAngleR * ppr / 360);
    goalPositionCountL = -(goalAngleL * ppr / 360);
    updateGoals = false;

    }

    int errorR = goalPositionCountR - positionCountR;
    int errorL = goalPositionCountL - positionCountL;

    if (abs(errorR) <= errorEncoderTrackingOuter) {
      digitalWrite(CW_PIN_R, LOW);
      digitalWrite(CCW_PIN_R, LOW);
      analogWrite(SPEED_PIN_ESCON_R, MINSPEEDPWM);  
      skipR = true;     
    }
    if (abs(errorL) <= errorEncoderTrackingOuter) {
      digitalWrite(CW_PIN_L, LOW);
      digitalWrite(CCW_PIN_L, LOW);
      analogWrite(SPEED_PIN_ESCON_L, MINSPEEDPWM);  
      skipL = true;     
    }

    // P controll
    int pwmSpeedR = constrain(map(abs(errorR) * Kp, 0, 180, MINSPEEDPWM, MAXSPEEDPWM), MINSPEEDPWM, MAXSPEEDPWM); 

    if (errorR < 0) {  // Move clockwise
      analogWrite(SPEED_PIN_ESCON_R, pwmSpeedR);
      digitalWrite(CCW_PIN_R, LOW);
      digitalWrite(CW_PIN_R, HIGH);
    } else {           // Move counter-clockwise
      analogWrite(SPEED_PIN_ESCON_R, pwmSpeedR);
      digitalWrite(CW_PIN_R, LOW);
      digitalWrite(CCW_PIN_R, HIGH);
    }

    int pwmSpeedL = constrain(map(abs(errorL) * Kp, 0, 180, MINSPEEDPWM, MAXSPEEDPWM), MINSPEEDPWM, MAXSPEEDPWM); 

    if (errorL < 0) {  // Move clockwise
      analogWrite(SPEED_PIN_ESCON_L, pwmSpeedL);
      digitalWrite(CCW_PIN_L, LOW);
      digitalWrite(CW_PIN_L, HIGH);
    } else {           // Move counter-clockwise
      analogWrite(SPEED_PIN_ESCON_L, pwmSpeedL);
      digitalWrite(CW_PIN_L, LOW);
      digitalWrite(CCW_PIN_L, HIGH);
    }
    skipR = false;
    skipL = false;
    delay(10);
  }
  //Stop tracking
  digitalWrite(CW_PIN_R, LOW);
  digitalWrite(CCW_PIN_R, LOW);
  digitalWrite(CW_PIN_L, LOW);
  digitalWrite(CCW_PIN_L, LOW);
  analogWrite(SPEED_PIN_ESCON_R, MINSPEEDPWM); 
  analogWrite(SPEED_PIN_ESCON_L, MINSPEEDPWM); 
}
