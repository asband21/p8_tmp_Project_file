const int IN1 = 13;  // PWM 900Hz
const int IN2 = 10; // DIGITAL OUTPUT

int pwmValue = 0;

void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  Serial.begin(9600);
  analogWrite(IN1, 0);  // START AT ZERO
  digitalWrite(IN2, LOW);
  Serial.println("Enter a PWM value between 0 and 255 to control motor speed:");
}

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    int pwmValue = input.toInt();
    if (pwmValue >= 0 && pwmValue <= 255) {
      analogWrite(IN1, pwmValue);  // Output PWM signal to control motor speed
      digitalWrite(IN2, LOW);      // Set motor direction
      Serial.print("Motor speed set to: ");
      Serial.println(pwmValue);
    } else {
      Serial.println("Please enter a valid PWM value (0-255).");
    }
  }
  // analogWrite(IN1, 255);  // JUST FOR UNCONNECTED PURPOSES ONLY
  // digitalWrite(IN2, LOW); 
}
