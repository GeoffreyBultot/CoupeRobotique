#define RR_DIR A3 
#define RR_ENA A2 
#define RL_DIR A0
#define RL_ENA A1

#define FR_DIR 4 
#define FR_ENA 5 
#define FL_DIR 7 
#define FL_ENA 6 
#define PWM_PIN 3


#define C_N_MOTORS 4

//int motors_PWM_Pins[C_N_MOTORS]  = {FL_STEP,FR_STEP,RL_STEP,RR_STEP};
int motors_ENA_Pins[C_N_MOTORS]  = {FL_ENA,FR_ENA,RL_ENA,RR_ENA};
int motors_DIR_Pins[C_N_MOTORS]  = {FL_DIR,FR_DIR,RL_DIR,RR_DIR};

void setup() {
  Serial.begin(115200);
  for(int i = 0;i<C_N_MOTORS;i++){
    pinMode(motors_ENA_Pins[i], OUTPUT);
    pinMode(motors_DIR_Pins[i], OUTPUT);
    digitalWrite(motors_ENA_Pins[i],LOW);
    digitalWrite(motors_DIR_Pins[i],LOW);
  }
  //nouveau shield
  pinMode(PWM_PIN,OUTPUT);
  analogWrite(PWM_PIN,127);

  //---------------------------------------------- Set PWM frequency for D3 & D11 ------------------------------
  
  //TCCR2B = TCCR2B & B11111000 | B00000001;    // set timer 2 divisor to     1 for PWM frequency of 31372.55 Hz
  //TCCR2B = TCCR2B & B11111000 | B00000010;    // set timer 2 divisor to     8 for PWM frequency of  3921.16 Hz
  //TCCR2B = TCCR2B & B11111000 | B00000011;    // set timer 2 divisor to    32 for PWM frequency of   980.39 Hz
  TCCR2B = TCCR2B & B11111000 | B00000100;    // set timer 2 divisor to    64 for PWM frequency of   490.20 Hz (The DEFAULT)
  //TCCR2B = TCCR2B & B11111000 | B00000101;    // set timer 2 divisor to   128 for PWM frequency of   245.10 Hz
  //TCCR2B = TCCR2B & B11111000 | B00000110;    // set timer 2 divisor to   256 for PWM frequency of   122.55 Hz
  //TCCR2B = TCCR2B & B11111000 | B00000111;    // set timer 2 divisor to  1024 for PWM frequency of    30.64 Hz
}

void printBinary(byte inByte)
{
  for (int b = 7; b >= 0; b--)
  {
    Serial.print(bitRead(inByte, b));
  }
  Serial.println();
}

String msg;
void readSerialPort() {
  msg = "";
  if (Serial.available()) {
    delay(10);
    while (Serial.available() > 0) {
      msg += (char)Serial.read();
    }
    Serial.flush();
  }
  //Serial.println(msg);
}


byte motorsRegister = 0x00;
void loop() {
  if(Serial.available()){
    motorsRegister = Serial.read();
    Serial.flush();
  }
  setMotor(motorsRegister);
}

void setMotor(byte motorsRegister){
  for(int i = 0; i<C_N_MOTORS;i++){
    bool direction = (motorsRegister>> (2*i))&0x01;
    bool stateMotor = !((motorsRegister>>((2*i)+1))&0x01);
    digitalWrite(motors_DIR_Pins[i],!direction);
    digitalWrite(motors_ENA_Pins[i],stateMotor);
  }
}
