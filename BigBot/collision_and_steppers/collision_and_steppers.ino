

/*STEPPER DEFINES*/
#define RR_DIR A3 
#define RR_ENA A2 
#define RL_DIR A0
#define RL_ENA A1

#define FR_DIR 4 
#define FR_ENA 5 
#define FL_DIR 7 
#define FL_ENA 6 
#define PWM_PIN 3
#define C_BLOCKED_MOTORS_DATA 0b01010101 //Quand on veut bloquer les moteur
#define C_N_MOTORS 4

/*Collision defines*/
#define C_N_SENSORS         4
#define C_PIN_SENSORS_COMM  2
#define C_PIN_SENSOR_LEFT   3
#define C_PIN_SENSOR_RIGHT  4
#define C_PIN_SENSOR_FRONT  5
#define C_PIN_SENSOR_BACK   6

#define C_HALF_V_SOUND      0.017
#define C_MAXIMUM_RANGE     30.0

#define C_FREQUENCY_REFRESH       10.0
#define C_TIME_BETWEEN_2_SENSORS  (1/C_FREQUENCY_REFRESH)/(float)C_N_SENSORS
#define C_TIME_REFRESH_SENSORS_D  0.01


/*globals stepper variables*/
int motors_ENA_Pins[C_N_MOTORS]  = {FL_ENA,FR_ENA,RL_ENA,RR_ENA};
int motors_DIR_Pins[C_N_MOTORS]  = {FL_DIR,FR_DIR,RL_DIR,RR_DIR};


/*globals collision variables*/

volatile byte current_sensor = 0;  
long t_last_refresh = 0;
unsigned char uc_byteToSend = 0x00;
volatile int duration = 0;
volatile const int tbi_Echo_Pins[C_N_SENSORS] = {C_PIN_SENSOR_LEFT,C_PIN_SENSOR_RIGHT,C_PIN_SENSOR_FRONT,C_PIN_SENSOR_BACK};

/*functions stepper declarations*/ 
void setMotor(byte motorsRegister){
  if(motorsRegister == C_BLOCKED_MOTORS_DATA){
    analogWrite(PWM_PIN,0);
    for(int i = 0; i<C_N_MOTORS;i++){
      digitalWrite(motors_ENA_Pins[i],LOW);
    }
  }else{
    analogWrite(PWM_PIN,127);
    for(int i = 0; i<C_N_MOTORS;i++){
      bool direction = (motorsRegister>> (2*i))&0x01;
      bool stateMotor = !((motorsRegister>>((2*i)+1))&0x01);
      digitalWrite(motors_DIR_Pins[i],!direction);
      digitalWrite(motors_ENA_Pins[i],stateMotor);
    }
  }
}



  //---------------------------------------------- Set PWM frequency for D3 & D11 ------------------------------

  //TCCR2B = TCCR2B & B11111000 | B00000001;    // set timer 2 divisor to     1 for PWM frequency of 31372.55 Hz
  //TCCR2B = TCCR2B & B11111000 | B00000010;    // set timer 2 divisor to     8 for PWM frequency of  3921.16 Hz
  //TCCR2B = TCCR2B & B11111000 | B00000011;    // set timer 2 divisor to    32 for PWM frequency of   980.39 Hz
  //TCCR2B = TCCR2B & B11111000 | B00000100;    // set timer 2 divisor to    64 for PWM frequency of   490.20 Hz (The DEFAULT)
  //TCCR2B = TCCR2B & B11111000 | B00000101;    // set timer 2 divisor to   128 for PWM frequency of   245.10 Hz
  //TCCR2B = TCCR2B & B11111000 | B00000110;    // set timer 2 divisor to   256 for PWM frequency of   122.55 Hz
  //TCCR2B = TCCR2B & B11111000 | B00000111;    // set timer 2 divisor to  1024 for PWM frequency of    30.64 Hz


void setSpeed(unsigned char divider){
  unsigned char mask = B00000110;
  switch(divider){
    case 1://31372
      mask = B00000001;
      break; 
    case 8: //3921
      mask = B00000010;
      break;
    case 32: //980
      mask = B00000011;
      break;
    case 64://490
      mask = B00000100;
      break;
    case 128://245
      mask = B00000101;
      break; 
    case 256://122
      mask = B00000110;
      break; 
    case 1024://30
      mask = B00000111;
      break; 
  }
  TCCR2B = TCCR2B & B11111000 | mask;
  //TCCR2B = reg;
}

/*functions collision declarations*/ 

float read_distance(byte sensor_trig_pin, byte sensor_echo_pin){
  long duration; // Duration used to calculate distance
  digitalWrite(sensor_trig_pin, LOW); 
  delayMicroseconds(2); 
  digitalWrite(sensor_trig_pin, HIGH);
  delayMicroseconds(10); 
  digitalWrite(sensor_trig_pin, LOW);
  duration = pulseIn(sensor_echo_pin, HIGH);
  return (float)duration*C_HALF_V_SOUND;
}


void setup() {
  Serial.begin(115200);
  int i = 0;
  /*STEPPERS PINS*/
  for(i = 0;i<C_N_MOTORS;i++){
    pinMode(motors_ENA_Pins[i], OUTPUT);
    pinMode(motors_DIR_Pins[i], OUTPUT);
    digitalWrite(motors_ENA_Pins[i],HIGH);
    digitalWrite(motors_DIR_Pins[i],LOW);
  }
  /*ECHOS COLLISIONS*/
  for(i=0;i<C_N_SENSORS;i++){
    pinMode(tbi_Echo_Pins[i],INPUT);
  }
  // PIN TRIG COLLISION
  pinMode(C_PIN_SENSORS_COMM, OUTPUT);
  //PIN PWM STEPPER
  pinMode(PWM_PIN,OUTPUT);
  analogWrite(PWM_PIN,127);
  TCCR2B = TCCR2B & B11111000 | B00000100;    // set timer 2 divisor to    64 for PWM frequency of   490.20 Hz (The DEFAULT)
  setMotor(0x00);
}


byte motorsRegister = 0x00;
byte motorsSpeed = 0x00;
int bytesToReceive = 0;
int positionRegister = 0;
#define C_LEN_BUFF_RECEPT 4
#define C_IDX_START_BYTE  0
#define C_IDX_MOTOR_REG   1
#define C_IDX_MOTOR_SPEED 2
#define C_IDX_STOP_BYTE   3
byte circ_Buffer[C_LEN_BUFF_RECEPT];

void loop() {
  bytesToReceive = Serial.available();
  if(Serial.available()){
    for(int i = 0; i < bytesToReceive;i++){
      circ_Buffer[positionRegister]=Serial.read();
      //Serial.println(circ_Buffer[positionRegister]);//circ_Buffer[positionRegister]);
      positionRegister = (positionRegister+1)%4;
      //Serial.println("g recu les bytes");

    }
    //Serial.flush();
  }
  for(int i = 0; i < C_LEN_BUFF_RECEPT;i++){
    if((circ_Buffer[i+C_IDX_START_BYTE]==123) && (circ_Buffer[ (i+C_IDX_STOP_BYTE)%C_LEN_BUFF_RECEPT]==253)){
      motorsRegister=circ_Buffer[ (i+C_IDX_MOTOR_REG)%C_LEN_BUFF_RECEPT];
      motorsSpeed=circ_Buffer[ (i+C_IDX_MOTOR_SPEED)%C_LEN_BUFF_RECEPT];
      //Serial.println("Set");
      setMotor(motorsRegister);
      setSpeed(motorsSpeed);
      break;
    }
  }
  //Serial.flush();

  if(millis()-t_last_refresh > C_TIME_BETWEEN_2_SENSORS*1000){
    t_last_refresh = millis();
    current_sensor = (++current_sensor)%C_N_SENSORS;
    duration = 0;// read_distance(C_PIN_SENSORS_COMM, tbi_Echo_Pins[current_sensor]);
    if(duration <= C_MAXIMUM_RANGE){
        uc_byteToSend |= 1<<current_sensor;
    }else{
        uc_byteToSend &=~ 1<<current_sensor;
    }
    Serial.write(uc_byteToSend);
  }
}








  //---------------------------------------------- Set PWM frequency for D3 & D11 ------------------------------

  //TCCR2B = TCCR2B & B11111000 | B00000001;    // set timer 2 divisor to     1 for PWM frequency of 31372.55 Hz
  //TCCR2B = TCCR2B & B11111000 | B00000010;    // set timer 2 divisor to     8 for PWM frequency of  3921.16 Hz
  //TCCR2B = TCCR2B & B11111000 | B00000011;    // set timer 2 divisor to    32 for PWM frequency of   980.39 Hz
  //TCCR2B = TCCR2B & B11111000 | B00000100;    // set timer 2 divisor to    64 for PWM frequency of   490.20 Hz (The DEFAULT)
  //TCCR2B = TCCR2B & B11111000 | B00000101;    // set timer 2 divisor to   128 for PWM frequency of   245.10 Hz
  //TCCR2B = TCCR2B & B11111000 | B00000110;    // set timer 2 divisor to   256 for PWM frequency of   122.55 Hz
  //TCCR2B = TCCR2B & B11111000 | B00000111;    // set timer 2 divisor to  1024 for PWM frequency of    30.64 Hz