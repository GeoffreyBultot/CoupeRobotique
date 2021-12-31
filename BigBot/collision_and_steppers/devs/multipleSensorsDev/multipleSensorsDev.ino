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
volatile int duration = 0;
volatile const int tbi_Echo_Pins[C_N_SENSORS] = {C_PIN_SENSOR_LEFT,C_PIN_SENSOR_RIGHT,C_PIN_SENSOR_FRONT,C_PIN_SENSOR_BACK};

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

volatile byte current_sensor = 0;  
long t1 = 0;
long t_last_refresh = 0;
unsigned char uc_byteToSend = 0x00;

void setup()
{
  int i=0;
  for(i=0;i<C_N_SENSORS;i++){
    pinMode(tbi_Echo_Pins[i],INPUT);
  }
  pinMode(C_PIN_SENSORS_COMM, OUTPUT);
  Serial.begin(115200);
}



void loop()
{
  /*if(Serial.available()){
    int motorsRegister = Serial.read();
  }*/
  if(millis()-t_last_refresh > C_TIME_BETWEEN_2_SENSORS*1000){
    t_last_refresh = millis();
    current_sensor = (++current_sensor)%C_N_SENSORS;
    duration = read_distance(C_PIN_SENSORS_COMM, tbi_Echo_Pins[current_sensor]);
    if(duration <= C_MAXIMUM_RANGE){
        uc_byteToSend |= 1<<current_sensor;
    }else{
        uc_byteToSend &=~ 1<<current_sensor;
    }
    Serial.write(uc_byteToSend);
  }
}





/*
// initialize Timer1
  cli();          // disable global interrupts
  TCCR1A = 0;     // set entire TCCR1A register to 0
  TCCR1B = 0;     // set entire TCCR1B register to 0
  // set compare match register to desired timer count, corresponds to 1 second
  OCR1A = 1561;   // Timer 1 Output Compare Register A
  TCCR1B |= (1 << WGM12);// turn on CTC mode on Timer1
  TCCR1B |= (1 << CS10);// Set CS10 and CS12 bits for 1024 prescaler
  TCCR1B |= (1 << CS12);
  TIMSK1 |= (1 << OCIE1A);// enable timer compare interrupt:
  sei();    // enable global interrupts
*/




