
#define IN1 13  // PWM for Motor 1       
#define IN2 12  // Direction for Motor 1
#define IN3 8  // PWM for Motor 2
#define IN4 10  // Direction for Motor 2
#define SPEED_PIN  11 //Pin for angle velocity
#define ENCODER_A_PIN 2  // Channel A
#define ENCODER_B_PIN 3  // Channel B
#define CW_PIN 5 // Clockwise motor pin
#define CCW_PIN 6 // Counter-clockwise motor pin

#define ALLOWEDERRORINNER 0.05
#define ALLOWEDERROROUTER 10 

//Angular velocity PWMS
#define MAXSPEEDPWM 229
#define MIDSPEEDPWM 140
#define MINSPEEDPWM 26

int pwmValue1 = 0;  // PWM value for Motor 1
int pwmValue2 = 0;  // PWM value for Motor 2

bool inMenu = true;  //Menu flag
bool inManualMode = false;  //Manual mode flag
bool inAutoMode = false;   //auto mode flag
bool trackingEnabled = false;

volatile long positionCount = 0;
int lastAState = LOW;
const float ppr = 2587;  // Pulses per revolution of encoder 10347og

float goalAngle = 0;
const float Kp = 0.5;  // Proportional gain for speed control


void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(SPEED_PIN, OUTPUT);
  
  Serial.begin(9600);
  analogWrite(IN1, 0);
  analogWrite(IN3, 0);
  digitalWrite(IN2, LOW);
  digitalWrite(IN4, LOW);

  pinMode(ENCODER_A_PIN, INPUT);
  pinMode(ENCODER_B_PIN, INPUT);
  pinMode(CCW_PIN, OUTPUT);
  pinMode(CW_PIN, OUTPUT);
  digitalWrite(CW_PIN, LOW);
  digitalWrite(CCW_PIN, LOW);
  analogWrite(SPEED_PIN, MINSPEEDPWM);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), updatePosition, RISING);

  printMenu();
}

void loop() {
  if (inMenu && Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input == "m") {
      inManualMode = true;
      inAutoMode = false;
      Serial.println("Manual mode selected. Enter motor number and PWM value (e.g., '1 127'). Type 'exit' to return to menu.");
      manualMode();
    } else if (input == "a") {
      inAutoMode = true;
      inManualMode = false;
      Serial.println("Automatic mode selected. Type 'exit' to return to menu.");
    } else if (input == "r") {
        inManualMode = false;
        inAutoMode = false;
        Serial.println("Enter an angle for refTracking (e.g., 45):");
        while (Serial.available() == 0); 
        String angleInput = Serial.readStringUntil('\n');
        angleInput.trim();
        float angleInp = angleInput.toFloat();
        float currentAngle = -(positionCount * 360/ppr);
        goalAngle = wrapAngle(angleInp);
        float deltaRot= getDeltaRotation(currentAngle, goalAngle);
        Serial.print("Moving to angle: ");
        Serial.println(goalAngle);
        Serial.print("Counter: ");
        Serial.println(positionCount);
        Serial.print("Current angle: ");
        Serial.println(currentAngle);
        Serial.print("Delta rot: ");
        Serial.println(deltaRot);
        trackingEnabled = true;
        refTrackingLoop(goalAngle);
        currentAngle = -(positionCount * 360/ppr);
        Serial.print("Final angle: ");
        Serial.println(currentAngle);
     } else if (input == "exit") {
        inManualMode = false;
        inAutoMode = false;
        inMenu = true;
        returnToHome();
        printMenu();
    } else {
      Serial.println("Invalid input. Please select 'm' for manual or 'a' for automatic.");
    }
  }

  if (inAutoMode) {
    // PWM ramp-up and down in opposite phases for motors
    for (int pwmValue = 0; pwmValue <= 255; pwmValue += 5) {
      analogWrite(IN1, pwmValue);          // Motor 1 increasing
      analogWrite(IN3, 255 - pwmValue);    // Motor 2 decreasing
      delay(50);
    }
    for (int pwmValue = 255; pwmValue >= 0; pwmValue -= 5) {
      analogWrite(IN1, pwmValue);          // Motor 1 decreasing
      analogWrite(IN3, 255 - pwmValue);    // Motor 2 increasing
      delay(50);
    }
    // Check for "exit" command to leave auto mode
    if (Serial.available() > 0) {
      String input = Serial.readStringUntil('\n');
      input.trim();
      if (input == "exit") {
        inAutoMode = false;
        inMenu = true;
        printMenu();
      }
    }
  }
}

void manualMode() {
  while (inManualMode) {
    if (Serial.available() > 0) {
      String input = Serial.readStringUntil('\n');
      input.trim();
      
      if (input == "exit") {
        inManualMode = false;
        inMenu = true;
        printMenu();
      } else {
        int motorNumber = input.substring(0, 1).toInt();  // Motor number (1 or 2)
        int pwmValue = input.substring(2).toInt();        // PWM value (0-255)

        if ((motorNumber == 1 || motorNumber == 2) && pwmValue >= 0 && pwmValue <= 255) {
          if (motorNumber == 1) {
            pwmValue1 = pwmValue;
            analogWrite(IN1, pwmValue1);  // Set PWM for Motor 1
            Serial.print("Motor 1 speed set to: ");
            Serial.println(pwmValue1);
          } else if (motorNumber == 2) {
            pwmValue2 = pwmValue;
            analogWrite(IN3, pwmValue2);  // Set PWM for Motor 2
            Serial.print("Motor 2 speed set to: ");
            Serial.println(pwmValue2);
          }
        } else {
          Serial.println("Invalid input. Enter '1 <value>' for Motor 1 or '2 <value>' for Motor 2, or type 'exit' to return to menu.");
        }
      }
    }
  }
}

void printMenu() {
  Serial.println("====== Motor Control Menu ======");
  Serial.println("'m' for manual mode.");
  Serial.println("'a' for automatic mode.");
  Serial.println("'r' to input an angle for refTracking.");
  Serial.println("'exit' to return to this menu.");
  inMenu = true;
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


void updatePosition() {
  int aState = digitalRead(ENCODER_A_PIN);
  int bState = digitalRead(ENCODER_B_PIN);

  if (aState != lastAState) {
    if (bState == LOW){
      positionCount--; 
    } else {
      positionCount++;
    }
  }
  lastAState = aState;
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
  digitalWrite(CW_PIN, LOW);
  digitalWrite(CCW_PIN, LOW);
  analogWrite(IN1, 0);
  analogWrite(IN3, 0);
  analogWrite(SPEED_PIN, MINSPEEDPWM);
}
