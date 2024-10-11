int pwmPin = 2;



void setup() {
  Serial.begin(9600);
  pinMode(pwmPin, OUTPUT);
  analogWrite(pwmPin, 128);
  Serial.println("Type a value between 0 to 1960");
}

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    int rotation = input.toInt();
    
    movefwd(rotation);
    
    
    
    // if (pwmValue >= 0 && pwmValue <= 255) {
    //   analogWrite(pwmPin, pwmValue);
    //   Serial.print("Setting PWM to: ");
    //   Serial.println(pwmValue);
    // } else {
    //   Serial.println("Wrong value");
    // }
  }
}

void movefwd(int rpm){
  uint8_t pwm_value= map(rpm, 0, 1963, 159, 255);
  if(pwm_value<168){
    analogWrite(pwmPin, 168);
    //delay
  }
  analogWrite(pwmPin, pwm_value);
}

void movebwd(int rpm){
  uint8_t pwm_value= map(rpm, 0, 1972, 78, 46);
  if(pwm_value>80){
    analogWrite(pwmPin, 80);
    //delay
  }
  analogWrite(pwmPin, pwm_value);
}
// // // // // 159 Top to bottom forward cutout
// // // // // 167 Bottom to top forward cutout


// 78 Top to bottom backward cutout
// 80 Bottom to top Backward cutout

//255 TOP MAX 795
//46 BOTTOM MAX SPEED 799
