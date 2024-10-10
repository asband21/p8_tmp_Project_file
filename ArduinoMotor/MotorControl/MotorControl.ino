int pwmPin = 2;  // Choose a PWM-capable pin (9 in this case)

void setup() {
  // Start serial communication
  Serial.begin(9600);
  
  // Set PWM pin as output
  pinMode(pwmPin, OUTPUT);
  
  // Print instructions
  Serial.println("Type a value between 0 and 255 to set the PWM signal.");
}

void loop() {
  // Check if any serial data is available
  if (Serial.available() > 0) {
    // Read the input as a string
    String input = Serial.readStringUntil('\n');
    
    // Convert the input to an integer
    int pwmValue = input.toInt();
    
    // Ensure the value is between 0 and 255
    if (pwmValue >= 0 && pwmValue <= 255) {
      // Set the PWM output to the typed value
      analogWrite(pwmPin, pwmValue);
      
      // Provide feedback to the user
      Serial.print("Setting PWM to: ");
      Serial.println(pwmValue);
    } else {
      Serial.println("Please enter a value between 0 and 255.");
    }
  }
}
