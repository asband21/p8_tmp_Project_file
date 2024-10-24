//SWagged out  1.3kohm and 220uF RC FILTER
#define BWDMAXSPEEDPWM 46
#define FWDMAXSPEEDPWM 255

#define TOPTOBOTFWDCUT 158 //ON TO OFF FORWARD CUT OUT
#define BOTTOTOPFWDCUT 168 //167   //OFF TO ON FORWARD CUT OUT
#define TOPTOBOTBWDCUT 77  //78 //OFF TO ON BWD CUT OUT
#define BOTTOTOPBWDCUT 79 //ON TO OFF BWD CUT OUT

#define STOPPWM 128

#define MAXROTATIONFWD 1970
#define MAXROTATIONBWD -1975
#define pwmPin 2

#define RELAYDELAY 500

#define FWDOFFSETCORRECT 0.000001 //DISABLED


void setup() {
  Serial.begin(9600);
  pinMode(pwmPin, OUTPUT);
  analogWrite(pwmPin, STOPPWM);
  Serial.print("Type a value between");
  Serial.print(MAXROTATIONBWD);
  Serial.print(" to ");
  Serial.println(MAXROTATIONFWD);
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
  rpm=rpm+rpm*FWDOFFSETCORRECT;
   if(rpm>MAXROTATIONFWD)
     rpm=MAXROTATIONFWD;
  uint8_t pwm_value= map(rpm, 0, MAXROTATIONFWD, TOPTOBOTFWDCUT, FWDMAXSPEEDPWM);
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


void setRotation(int rpm){
  if(rpm<0){
    movebwd(-rpm);
  }
  if(rpm>0){
    movefwd(rpm);
  }
  if(rpm==0){
    analogWrite(pwmPin, STOPPWM);
    Serial.println("Stop");
  }
}



