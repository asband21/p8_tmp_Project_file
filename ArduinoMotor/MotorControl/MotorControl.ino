
//255 TOP MAX 795
//46 BOTTOM MAX SPEED 799

#define TOPTOBOTFWDCUT 159
#define BOTTOTOPFWDCUT 159
#define TOPTOBOTBWDCUT 78
#define BOTTOTOPBWDCUT 80

#define STOPPWM 128

#define MAXROTATIONFWD 1963
#define MAXROTATIONBWD -1972
#define pwmPin 2


void setup() {
  Serial.begin(9600);
  pinMode(pwmPin, OUTPUT);
  analogWrite(pwmPin, STOPPWM);
  Serial.println("Type a value between -1972 to 1963");
}

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    int rotation = input.toInt();
    if( rotation>=MAXROTATIONBWD && rotation<= MAXROTATIONFWD ){
      setRotation(rotation);
    }
    else
       Serial.println("Wrong value");
  }
}

void movefwd(int rpm){
  uint8_t pwm_value= map(rpm, 0, 1963, 159, 255);
  if(pwm_value<168){
    analogWrite(pwmPin, 168);
    delay(10);
  }
  analogWrite(pwmPin, pwm_value);
    Serial.println("Forward:");
    Serial.println(pwm_value);
}

void movebwd(int rpm){
  uint8_t pwm_value= map(rpm, 0, 1972, 78, 46);
  if(pwm_value>80){
    analogWrite(pwmPin, 80);
    delay(10);
  }
  analogWrite(pwmPin, pwm_value);
    Serial.println("Backward:");
    Serial.println(pwm_value);
}


void setRotation(int rpm){
  if(rpm<0){
    movebwd(-rpm);
  }
  if(rpm>0){
    movefwd(rpm);
  }
  if(rpm==0){
    analogWrite(pwmPin, 128);
  }
}



