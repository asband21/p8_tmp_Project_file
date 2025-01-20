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
#define SPEED_PIN_ESCON_R  12 //Pin for angle velocity right motor
#define SPEED_PIN_ESCON_L  11 //Pin for angle velocity
#define ENCODER_A_PIN_R 18  // Channel A left motor
#define ENCODER_B_PIN_R 19  // Channel B left motor
#define ENCODER_A_PIN_L 20  // Channel A right motor
#define ENCODER_B_PIN_L 21  // Channel B right motor

#define CW_PIN_R 24 // Clockwise motor pin right motor
#define CCW_PIN_R 25 // Counter-clockwise motor pin right motor

#define CW_PIN_L 22 // Clockwise motor pin left motor
#define CCW_PIN_L 23 // Counter-clockwise motor pin left motor

#define ALLOWEDERRORINNER 0.05
#define ALLOWEDERROROUTER 10 

//ESCON angular velocity PWMS
#define MAXSPEEDPWM 229
#define MIDSPEEDPWM 140
#define MINSPEEDPWM 26

volatile long positionCountR = 0;
volatile long positionCountL = 0;

const float ppr = 5173;  // Pulses per revolution of encoder 10347og  
float goalAngleR = 0;
float goalAngleL = 0;
const float Kp = 10;  // Proportional gain for ang velocity control

long int errorEncoderTrackingOuter = ppr * ALLOWEDERROROUTER / 360;
long int errorEncoderTrackingINNER = ppr * ALLOWEDERRORINNER / 360;

long int goalPositionCountR=0;
long int goalPositionCountL=0;

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

}

void loop() {

  static unsigned long lastTime = 0;
  unsigned long currentTime = millis();


  if ( digipot.isLocked() ) {
    if (millis() - lastLockedAt > rateLimit) {
      digipot.unlock();
      if(digipotNeedsUpdate)
        setWiperPositionRateLimited(digipotUpdatePosition);
    }
  }

//    if (currentTime - lastTime >= 10) {  // Log every 10 ms //FOR ANG RESPONSE TESTING
//    float angle = (positionCountR * 360.0) / ppr;  // Convert count to angle
//    Serial.print(currentTime);  // Time in milliseconds
//    Serial.print(",");
//    Serial.println(angle);  // Angle in degrees
//    lastTime = currentTime;
//    }

  if (Serial.available() > 0) {
    String inputString = Serial.readStringUntil('\n');
    inputString.trim(); 
    //string in the format: "f 50 90 60 180 70"
    int result = sscanf(inputString.c_str(), "%c %d %d %d %d %d", 
                        &directionF, 
                        &throttleF, 
                        &angleR, 
                        &throttleR, 
                        &angleL, 
                        &throttleL);

    //SHOULD WE DISABLE PROP SPEED WHILE ROTATING?
    if(result == 6){
      setPropSpeedFront(directionF, throttleF);
      setPropSpeedRear('r', throttleR); 
      setPropSpeedRear('l', throttleL);
      goalAngleR=wrapAngle(angleR);
      goalAngleL=wrapAngle(angleL);
      goalPositionCountR = (goalAngleR * ppr / 360);
      goalPositionCountL = (goalAngleL * ppr / 360);
    }
  }
  int errorR = goalPositionCountR - positionCountR;
  int errorL = goalPositionCountL - positionCountL;


  // Right Motor Control
  if (abs(errorR) > errorEncoderTrackingINNER) {
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
  delay(10); 
}


bool setWiperPositionRateLimited(byte position) {
  if (digipot.isLocked()) {
    digipotNeedsUpdate = true;
    digipotUpdatePosition = position;
    return false;  // Do nothing if locked
  }
  // Unlock and change digipot valuue
  digipot.unlock();
  digipot.setWiperPosition(position);
  digipot.lock();
  // Update the locktime
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

//PWM VARIANT OF FUNCTIONS
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
//PWM END

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

float getDeltaRotation(float currentAngle, float goalAngle) {
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


void controlMotor(float error, int cwPin, int ccwPin, int speedPin) {
  int pwmSpeed = constrain(map(abs(error) * Kp, 0, ppr, MINSPEEDPWM, MAXSPEEDPWM), MINSPEEDPWM, MAXSPEEDPWM);

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
  setPropSpeedRear('r', 0);
  setPropSpeedRear('l', 0);
  gotoAngle(0, positionCountR, CW_PIN_R, CCW_PIN_R, SPEED_PIN_ESCON_R);  
  gotoAngle(0, positionCountL, CW_PIN_L, CCW_PIN_L, SPEED_PIN_ESCON_L);
  setPropSpeedFront('s', 0);

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
