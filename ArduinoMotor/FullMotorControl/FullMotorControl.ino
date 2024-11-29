#include <DS1804.h>

#define BWDMAXSPEEDPWM 13                 
#define FWDMAXSPEEDPWM 97                 

#define TOPTOBOTFWDCUT 62  //ON TO OFF FORWARD CUT OUT
#define BOTTOTOPFWDCUT 63 //OFF TO ON FORWARD CUT OUT
#define TOPTOBOTBWDCUT 24  //OFF TO ON BWD CUT OUT
#define BOTTOTOPBWDCUT 25//ON TO OFF BWD CUT OUT
#define RELAYDELAY 500
#define STOPPWM 45




#define RMOTORPROP 13  // PWM for Motor 1       
#define LMOTORPROP 8  // PWM for Motor 2
#define SPEED_PIN_ESCON_R  11 //Pin for angle velocity right motor
#define SPEED_PIN_ESCON_L  12 //Pin for angle velocity
#define ENCODER_A_PIN_R 2  // Channel A left motor
#define ENCODER_B_PIN_R 3  // Channel B left motor
#define ENCODER_A_PIN_L 2  // Channel A right motor
#define ENCODER_B_PIN_L 3  // Channel B right motor

#define CW_PIN_R 22 // Clockwise motor pin right motor
#define CCW_PIN_R 23 // Counter-clockwise motor pin right motor

#define CW_PIN_L 24 // Clockwise motor pin left motor
#define CCW_PIN_L 25 // Counter-clockwise motor pin left motor

#define ALLOWEDERRORINNER 0.05
#define ALLOWEDERROROUTER 10 

//ESCON angular velocity PWMS
#define MAXSPEEDPWM 229
#define MIDSPEEDPWM 140
#define MINSPEEDPWM 26

//digital potentiometer pins
#define CSPIN 49
#define INCPIN 51
#define UDPIN 53

DS1804 digipot = DS1804(CSPIN, INCPIN, UDPIN, DS1804_HUNDRED); //Digital potentiometer for front motor

bool trackingEnabled = false;



volatile long positionCountR = 0;
volatile long positionCountL = 0;
int lastAStateR = LOW;
int lastAStateL = LOW;
const float ppr = 2587;  // Pulses per revolution of encoder 10347og
float goalAngleR = 0;
float goalAngleL = 0;
const float Kp = 0.5;  // Proportional gain for speed control


void setup() {
  Serial.begin(9600);

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


  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN_R), updatePositionR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN_L), updatePositionL, CHANGE);
}

void loop() {
  //digipot.setWiperPosition(i);
}


void movefwd(int throttle){
   if(throttle>100)
     throttle=100;
  int digi_value= map(throttle, 0, 100, TOPTOBOTFWDCUT, FWDMAXSPEEDPWM);
  if(digi_value<BOTTOTOPFWDCUT){
    digipot.setWiperPosition(BOTTOTOPFWDCUT);
    delay(RELAYDELAY);
  }
  digipot.setWiperPosition(digi_value);
    Serial.println("Forward:");//DELETE THIS
    Serial.println(digi_value);
}

void movebwd(int throttle){
  int digi_value= map(throttle, 0, 100, TOPTOBOTBWDCUT, BWDMAXSPEEDPWM);
  if(digi_value>BOTTOTOPBWDCUT){
   digipot.setWiperPosition(BOTTOTOPBWDCUT);
    delay(RELAYDELAY);
  }
  digipot.setWiperPosition(digi_value);
  Serial.println("Backward:"); //DELETE THIS
  Serial.println(digi_value);
}

void setPropSpeedRear(char motor,int throttle){
  int pwm = map(throttle, 0, 100, 0, 255);
  if(motor=='r')
    analogWrite(RMOTORPROP, pwm);
  if(motor=='l')
    analogWrite(LMOTORPROP, pwm);
}

void setPropSpeedFront(char direction, int throttle){
  if(throttle==0 || direction=='s') //STOP
    digipot.setWiperPosition(STOPPWM);
  else{
    if(direction=='b')//backwards
      movebwd(throttle);
    if(direction=='f')//forward
      movefwd(throttle);
  }
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
  int aState = digitalRead(ENCODER_A_PIN_R);
  int bState = digitalRead(ENCODER_B_PIN_R);

  if (aState != lastAStateR) {
    if (bState == LOW) {
      positionCountR--;
    } else {
      positionCountR++;
    }
  }
  lastAStateR = aState;
}

void updatePositionL() {
  int aState = digitalRead(ENCODER_A_PIN_L);
  int bState = digitalRead(ENCODER_B_PIN_L);

  if (aState != lastAStateL) {
    if (bState == LOW) {
      positionCountL--;
    } else {
      positionCountL++;
    }
  }
  lastAStateL = aState;
}

void refTracking(float goalAngle ) {
  long int allowedTrackingError= ppr*ALLOWEDERRORINNER/360;
  long int goalPositionCount= -(goalAngle*ppr/360);
  analogWrite(SPEED_PIN, MIDSPEEDPWM);
  while (!(positionCount > goalPositionCount - allowedTrackingError && positionCount <= goalPositionCount + allowedTrackingError)) { 
    if (positionCount < goalPositionCount) {
      digitalWrite(CW_PIN, LOW);
      digitalWrite(CCW_PIN, HIGH);
    }
    if (positionCount > goalPositionCount) {
      digitalWrite(CCW_PIN, LOW);
      digitalWrite(CW_PIN, HIGH);
    }
    //delay(10);
  }
  digitalWrite(CW_PIN, LOW);
  digitalWrite(CCW_PIN, LOW);
}

void refTrackingLoop(float goalAngle) {
  long int allowedTrackingErrorLoop = ppr * ALLOWEDERROROUTER / 360;
  long int goalPositionCount = -(goalAngle * ppr / 360);
  refTracking(goalAngle);  // Initial move to goal
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

    // Calculate error
    int error = goalPositionCount - positionCount;
    
    // If within allowed error range, stop
    if (abs(error) <= allowedTrackingErrorLoop) {
      digitalWrite(CW_PIN, LOW);
      digitalWrite(CCW_PIN, LOW);
      analogWrite(SPEED_PIN, MINSPEEDPWM);  
      delay(100); 
      continue;    
    }

    // P controll
    int pwmSpeed = constrain(map(abs(error) * Kp, 0, 180, MINSPEEDPWM, MAXSPEEDPWM), MINSPEEDPWM, MAXSPEEDPWM); 

    if (error < 0) {  // Move clockwise
      analogWrite(SPEED_PIN, pwmSpeed);
      digitalWrite(CCW_PIN, LOW);
      digitalWrite(CW_PIN, HIGH);
    } else {           // Move counter-clockwise
      analogWrite(SPEED_PIN, pwmSpeed);
      digitalWrite(CW_PIN, LOW);
      digitalWrite(CCW_PIN, HIGH);
    }
    delay(10);
  }

  digitalWrite(CW_PIN, LOW);
  digitalWrite(CCW_PIN, LOW);
  analogWrite(SPEED_PIN, MINSPEEDPWM);  // Set speed to 0
}

float wrapAngle(float angle) {
  angle = fmod(angle + 180, 360);  // Wrap within [0, 360)
  if (angle < 0) angle += 360;     // Ensure positive result
  return angle - 180;              // Shift to [-180, 180]
}

void returnToHome(){
  trackingEnabled = false; 
  refTracking(0);

}

