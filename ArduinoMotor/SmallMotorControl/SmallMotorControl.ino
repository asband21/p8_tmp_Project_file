const int IN1 = 4;  // PWM for Motor 1
const int IN2 = 10;  // Direction for Motor 1
const int IN3 = 13;  // PWM for Motor 2
const int IN4 = 12;  // Direction for Motor 2

int pwmValue1 = 0;  // PWM value for Motor 1
int pwmValue2 = 0;  // PWM value for Motor 2
bool inMenu = true;
bool inManualMode = false;
bool inAutoMode = false;

void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  Serial.begin(9600);
  analogWrite(IN1, 0);
  analogWrite(IN3, 0);
  digitalWrite(IN2, LOW);
  digitalWrite(IN4, HIGH);
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
    } else if (input == "exit") {
      inManualMode = false;
      inAutoMode = false;
      inMenu = true;
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
  Serial.println("Type 'm' for manual mode.");
  Serial.println("Type 'a' for automatic mode.");
  Serial.println("Type 'exit' to return to this menu.");
  inMenu = true;
}
