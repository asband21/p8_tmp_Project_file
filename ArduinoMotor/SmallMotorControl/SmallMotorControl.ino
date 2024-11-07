const int IN1 = 13;  // PWM for Motor 1        //BEING USED FOR STOP PIN
const int IN2 = 12;  // Direction for Motor 1
const int IN3 = 8;  // PWM for Motor 2
const int IN4 = 10;  // Direction for Motor 2

#define ENCODER_A_PIN 2  // Channel A
#define ENCODER_B_PIN 3  // Channel B
#define STOP_PIN 4 // Stop pin for testing 
#define CW_PIN 5 // Clockwise motor pin
#define CCW_PIN 6 // Counter-clockwise motor pin
#define ALLOWEDERROR 0.05

int pwmValue1 = 0;  // PWM value for Motor 1
int pwmValue2 = 0;  // PWM value for Motor 2
bool inMenu = true;
bool inManualMode = false;
bool inAutoMode = false;



volatile long positionCount = 0;
int lastAState = LOW;
const float ppr = 10347;  // Pulses per revolution of your encoder
float degreesPerPulse = (float)ppr / 360;
float goalAngle = 0;


void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  Serial.begin(9600);
  analogWrite(IN1, 0);
  analogWrite(IN3, 0);
  digitalWrite(IN2, LOW);
  digitalWrite(IN4, LOW);

  pinMode(ENCODER_A_PIN, INPUT);
  pinMode(ENCODER_B_PIN, INPUT);
  pinMode(CCW_PIN, OUTPUT);
  pinMode(CW_PIN, OUTPUT);
  pinMode(STOP_PIN, OUTPUT);
  digitalWrite(CW_PIN, LOW);
  digitalWrite(CCW_PIN, LOW);
  digitalWrite(STOP_PIN, LOW);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), updatePosition, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B_PIN), updatePosition, CHANGE);

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
        refTracking(goalAngle );
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
    if (aState == HIGH && bState == LOW || aState == LOW && bState == HIGH){
      positionCount--; 
    } else {
      positionCount++;
    }
  }
  lastAState = aState;
}


void refTracking(float goalAngle ) {
  // long int goalPosition = positionCount + deltaRotation*ppr/360;
  long int allowedTrackingError= ppr*ALLOWEDERROR/360;
  long int goalPositionCount= -(goalAngle*ppr/360);
  while (!(positionCount > goalPositionCount - allowedTrackingError && positionCount <= goalPositionCount + allowedTrackingError)) { 
    if (positionCount < goalPositionCount) {
      digitalWrite(STOP_PIN, LOW);
      digitalWrite(CW_PIN, LOW);
      digitalWrite(CCW_PIN, HIGH);
    }
    if (positionCount > goalPositionCount) {
      digitalWrite(STOP_PIN, LOW);
      digitalWrite(CCW_PIN, LOW);
      digitalWrite(CW_PIN, HIGH);
    }
    //delay(10);
  }
  digitalWrite(CW_PIN, LOW);
  digitalWrite(CCW_PIN, LOW);
  digitalWrite(STOP_PIN, HIGH);
}

float wrapAngle(float angle) {
  angle = fmod(angle + 180, 360);  // Wrap within [0, 360)
  if (angle < 0) angle += 360;     // Ensure positive result
  return angle - 180;              // Shift to [-180, 180]
}

void returnToHome(){
  refTracking(0);
  digitalWrite(CW_PIN, LOW);
  digitalWrite(CCW_PIN, LOW);
  digitalWrite(STOP_PIN, LOW);
  analogWrite(IN1, 0);
  analogWrite(IN3, 0);
  //positionCount=0;
}
