#include <Servo.h>
#include "TFT_eSPI.h" 
#include <SPI.h>


#define ST77XX_BLACK TFT_BLACK
#define ST77XX_WHITE TFT_WHITE
#define ST77XX_RED TFT_RED
#define ST77XX_GREEN TFT_GREEN
#define ST77XX_BLUE TFT_BLUE
#define ST77XX_CYAN TFT_CYAN
#define ST77XX_MAGENTA TFT_MAGENTA
#define ST77XX_YELLOW TFT_YELLOW
#define ST77XX_ORANGE TFT_ORANGE

TFT_eSPI tft_ = TFT_eSPI();
TFT_eSprite sprite_ = TFT_eSprite(&tft_);
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
#define _servo1 12
#define _servo2 13
#define _servo3 14
#define _servo4 15



#define motor1A  0
#define motor1B  1
#define motor2A  2
#define motor2B  3
#define motor3A  10
#define motor3B  11
#define motor4A  8
#define motor4B  9

int _sensorPins[20];
int _NumofSensor = 0;
int _min_sensor_values[20];
int _max_sensor_values[20];
int _lastPosition = 0;
int _Sensitive  = 20;
int stateOfRunPID = 0;
float  errors = 0, output = 0, integral = 0, derivative = 0, previous_error = 0;
uint8_t FrontLineColor = 0;
uint8_t BackLineColor = 0;


void puppybot_setup() {
  analogWriteResolution(10);
  analogWriteRange(1023);
  tft_.init();
  tft_.setRotation(1);
  tft_.fillScreen(TFT_BLACK);

}
int ADC(int analog_CH) {
  int val = 0;
  if (analog_CH < 8 ) {
    pinMode(22, OUTPUT);
    pinMode(23, OUTPUT);
    pinMode(24, OUTPUT);
    int controlPin[] = {22, 23, 24};
    int muxChannel[8][3] = {{0, 1, 0}, {1, 0, 0}, {0, 0, 0}, {1, 1, 0}, {0, 0, 1}, {0, 1, 1}, {1, 1, 1}, {1, 0, 1}};
    digitalWrite(22, muxChannel[analog_CH][0]);
    digitalWrite(23, muxChannel[analog_CH][1]);
    digitalWrite(24, muxChannel[analog_CH][2]);
    val = analogRead(26);
  }
  else if (analog_CH >= 8 && analog_CH < 11 ) {
    val = analogRead(19 + analog_CH);
  }
  return val;
}

int IN(int _pins) {
  if (_pins == 1) {_pins = 25;}
  else if(_pins >=2 && _pins <=4){_pins = 25 +_pins;}
  else{return 0;}
  pinMode(_pins, INPUT); 
  return digitalRead(_pins);
}
void OUT(int _pins,uint8_t _Status){
  if (_pins == 1) {_pins = 25;}
  else if(_pins >=2 && _pins <=4){_pins = 25 +_pins;}
  pinMode(_pins, OUTPUT); 
  digitalWrite(_pins,_Status);
}
void buzzer(int freq, int timr_delay) {
  pinMode(7, OUTPUT);
  tone(7, freq);
  delay(timr_delay);
  tone(7, 0);
}
void printText(uint8_t x,uint8_t y,String text,uint8_t size,uint16_t  color){
	tft_.setCursor(x, y);
	tft_.setTextSize(size);
  tft_.setTextColor(color);
  tft_.setTextWrap(true);
  tft_.println(text);
}
void printText(uint8_t x,uint8_t y,String text,uint8_t size,uint16_t  color1,uint16_t  color2){
  tft_.setCursor(x, y);
  tft_.setTextSize(size);
  tft_.setTextColor(color1,color2);
  tft_.setTextWrap(true);
  tft_.println(text);
}
void wait_SW1() {
  int state_waitSW1 = 0;
  pinMode(6, INPUT_PULLUP);
  tft_.setTextSize(2);
  tft_.setTextColor(TFT_WHITE, TFT_BLACK);
  tft_.fillScreen(TFT_BLACK);
  do {
    
    tft_.setTextColor(TFT_WHITE, TFT_BLACK);
    tft_.drawString("0="+String(ADC(0)),0,0);
    tft_.drawString("1="+String(ADC(1)),80,0);
    tft_.drawString("2="+String(ADC(2)),0,17);
    tft_.drawString("3="+String(ADC(3)),80,17);
    tft_.drawString("4="+String(ADC(4)),0,34);
    tft_.drawString("5="+String(ADC(5)),80,34);
    tft_.drawString("6="+String(ADC(6)),0,51);
    tft_.drawString("7="+String(ADC(7)),80,51);
    tft_.drawString("8="+String(ADC(8)),0,68);
    tft_.drawString("9="+String(ADC(9)),80,68);
    tft_.drawString("10="+String(ADC(10)),0,85);
    if(state_waitSW1 == 0){
      state_waitSW1 = 1;
      tft_.setTextColor(TFT_RED, TFT_BLUE);
      tft_.drawString("  SW1 Press  ",0,105);
    }
    else
    {
      state_waitSW1 = 0;
      tft_.setTextColor(TFT_GREEN, TFT_YELLOW);
      tft_.drawString("  SW1 Press  ",0,105);
    }
  	
  	delay(50);
  } while (digitalRead(6) == 1);
  tft_.fillScreen(ST77XX_BLACK);
  buzzer(500,100);
}

void motor(int pin, int speed_Motor) {
  if (speed_Motor > 100)speed_Motor = 100;
  if (speed_Motor < -100)speed_Motor = -100;
  if (pin == 1) {
    if (speed_Motor < 0) {
      // Serial.println(speed_Motor);
      speed_Motor = abs(speed_Motor) * 10.23;
      analogWrite(motor1B, 1023);
      analogWrite(motor1A, 1023-abs(speed_Motor));

    }
    else {
      speed_Motor = abs(speed_Motor) * 10.23;
      analogWrite(motor1A, 1023);
      analogWrite(motor1B, 1023- abs(speed_Motor));
    }
  }
  else if (pin == 2) {
    if (speed_Motor < 0) {
      speed_Motor = abs(speed_Motor) * 10.23;
      analogWrite(motor2B, 1023);
      analogWrite(motor2A, 1023-abs(speed_Motor));
    }
    else {
      speed_Motor = abs(speed_Motor) * 10.23;
      analogWrite(motor2A, 1023);
      analogWrite(motor2B, 1023-abs(speed_Motor));
    }
  }
  else if (pin == 3) {
    if (speed_Motor < 0) {
      speed_Motor = abs(speed_Motor) * 10.23;
      analogWrite(motor3B, 1023);
      analogWrite(motor3A, 1023-abs(speed_Motor));
    }
    else {
      speed_Motor = abs(speed_Motor) * 10.23;
      analogWrite(motor3A, 1023);
      analogWrite(motor3B, 1023-abs(speed_Motor));
    }
  }
  else if (pin == 4) {
    if (speed_Motor < 0) {
      speed_Motor = abs(speed_Motor) * 10.23;
      analogWrite(motor4B, 1023);
      analogWrite(motor4A, 1023-abs(speed_Motor));
    }
    else {
      speed_Motor = abs(speed_Motor) * 10.23;
      analogWrite(motor4A, 1023);
      analogWrite(motor4B, 1023-abs(speed_Motor));
    }
  }
}
void motor_control(uint8_t state , int _speed) {
  switch (state) {
    case 0: {
        motor(1, _speed);
        motor(2, _speed);
      } break;
    case 1: {
        motor(1, -_speed);
        motor(2, -_speed);
      } break;
    case 2: {
        motor(1, _speed);
        motor(2, 0);
      } break;
    case 3: {
        motor(1, 0);
        motor(2, _speed);
      } break;
    case 4: {
        motor(1, -_speed);
        motor(2, _speed);
      } break;
    case 5: {
        motor(1, _speed);
        motor(2, -_speed);
      } break;
    case 6: {
        motor(1, _speed);
      } break;
    case 7: {
        motor(2, _speed);
      } break;
    case 8: {
        motor(1, -_speed);
      } break;
    case 9: {
        motor(2, -_speed);
      } break;
  }
}
void ao(){
	analogWrite(motor1B, 1023);
	analogWrite(motor1A, 1023);
	analogWrite(motor2B, 1023);
	analogWrite(motor2A, 1023);
	analogWrite(motor3B, 1023);
	analogWrite(motor3A, 1023);
	analogWrite(motor4B, 1023);
	analogWrite(motor4A, 1023);
}
void aoS(int speed_break){
  speed_break = constrain(speed_break, 0, 100);
  speed_break = speed_break * 10.23;
  analogWrite(motor1B, speed_break);
  analogWrite(motor1A, speed_break);
  analogWrite(motor2B, speed_break);
  analogWrite(motor2A, speed_break);
}
void motorStop(int motor_ch){
  	if(motor_ch == 0){
	  analogWrite(motor1B, 0);
	  analogWrite(motor1A, 0);
	  analogWrite(motor2B, 0);
	  analogWrite(motor2A, 0);
	  analogWrite(motor3B, 0);
	  analogWrite(motor3A, 0);
	  analogWrite(motor4B, 0);
	  analogWrite(motor4A, 0);
	}
	else if(motor_ch == 1 ){
	  analogWrite(motor1B, 0);
	  analogWrite(motor1A, 0);
	}
	else if(motor_ch == 2 ){
	  analogWrite(motor2B, 0);
	  analogWrite(motor2A, 0);
	}
	else if(motor_ch == 3 ){
	  analogWrite(motor3B, 0);
	  analogWrite(motor3A, 0);
	}
	else if(motor_ch == 4 ){
	  analogWrite(motor4B, 0);
	  analogWrite(motor4A, 0);
	}
	else{
	  analogWrite(motor1B, 0);
	  analogWrite(motor1A, 0);
	  analogWrite(motor2B, 0);
	  analogWrite(motor2A, 0);
	  analogWrite(motor3B, 0);
	  analogWrite(motor3A, 0);
	  analogWrite(motor4B, 0);
	  analogWrite(motor4A, 0);
	}
}
void motorBreak(){
    analogWrite(motor1B, 1023);
    analogWrite(motor1A, 1023);
    analogWrite(motor2B, 1023);
    analogWrite(motor2A, 1023);
    analogWrite(motor3B, 1023);
    analogWrite(motor3A, 1023);
    analogWrite(motor4B, 1023);
    analogWrite(motor4A, 1023);
}
void motorBreak(int motor_ch){
  
	if(motor_ch == 0){
	  analogWrite(motor1B, 1023);
	  analogWrite(motor1A, 1023);
	  analogWrite(motor2B, 1023);
	  analogWrite(motor2A, 1023);
	  analogWrite(motor3B, 1023);
	  analogWrite(motor3A, 1023);
	  analogWrite(motor4B, 1023);
	  analogWrite(motor4A, 1023);

	}
	else if(motor_ch == 1 ){
	  analogWrite(motor1B, 1023);
	  analogWrite(motor1A, 1023);
	}
	else if(motor_ch == 2 ){
	  analogWrite(motor2B, 1023);
	  analogWrite(motor2A, 1023);
	}
	else if(motor_ch == 3 ){
	  analogWrite(motor3B, 1023);
	  analogWrite(motor3A, 1023);
	}
	else if(motor_ch == 4 ){
	  analogWrite(motor4B, 1023);
	  analogWrite(motor4A, 1023);
	}
	else{
	  analogWrite(motor1B, 1023);
	  analogWrite(motor1A, 1023);
	  analogWrite(motor2B, 1023);
	  analogWrite(motor2A, 1023);
	  analogWrite(motor3B, 1023);
	  analogWrite(motor3A, 1023);
	  analogWrite(motor4B, 1023);
	  analogWrite(motor4A, 1023);
	}
}
void fd(int speed_Motor){
	motor(1,speed_Motor);
	motor(2,speed_Motor);
	motor(3,speed_Motor);
	motor(4,speed_Motor);
}
void fd2(int speed_MotorA,int speed_MotorB){
  motor(1,speed_MotorA);
  motor(2,speed_MotorB);
}
void bk(int speed_Motor){
	motor(1,-speed_Motor);
	motor(2,-speed_Motor);
	motor(3,-speed_Motor);
	motor(4,-speed_Motor);
}
void bk2(int speed_MotorA,int speed_MotorB){
  motor(1,-speed_MotorA);
  motor(2,-speed_MotorB);
}
void tl(int speed_Motor){
	motor(1,0);
	motor(2,speed_Motor);
}
void tr(int speed_Motor){
	motor(1,speed_Motor);
	motor(2,0);
}
void sl(int speed_Motor){
	motor(1,-speed_Motor);
	motor(2,speed_Motor);
}
void sr(int speed_Motor){
	motor(1,speed_Motor);
	motor(2,-speed_Motor);
}

void servoRun(uint8_t servo_ch, int16_t angle) {

  if (servo_ch == 1)
  {
    if(angle == -1){servo1.detach();}
    servo1.attach(_servo1,300,2500,angle);
  }
  if (servo_ch == 2)
  {
    if(angle == -1){servo2.detach();}
    servo2.attach(_servo2,300,2500,angle);
  }
  if (servo_ch == 3)
  {
    if(angle == -1){servo3.detach();}
    servo3.attach(_servo3,300,2500,angle);
  }
  if (servo_ch == 4)
  {
    if(angle == -1){servo4.detach();}
    servo4.attach(_servo4,300,2500,angle);
  }
}
int ultrasonic(uint8_t Echo_pin , uint8_t Trig_pin) {
  int ECHO = Echo_pin;
  int TRIG = Trig_pin;
  pinMode(ECHO, INPUT);
  pinMode(TRIG, OUTPUT);
  long duration = 0;
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  duration += pulseIn(ECHO, HIGH);

  // Calculating the distance
  return (duration) * 0.034 / 2;
}



//*********************************************************  TC01Sensor **********************************************************************************************
//*********************************************************  TC01Sensor **********************************************************************************************
//*********************************************************  TC01Sensor **********************************************************************************************
//*********************************************************  TC01Sensor **********************************************************************************************

void setSensorPins(const int * _pins, int _NumofSensor_)
{
  _NumofSensor = _NumofSensor_;
  // _sensorPins = (int *)realloc(_sensorPins, sizeof(int) * _NumofSensor_);
  // _min_sensor_values = (int *)realloc(_min_sensor_values, sizeof(int) * _NumofSensor_);
  // _max_sensor_values = (int *)realloc(_max_sensor_values, sizeof(int) * _NumofSensor_);
  for (uint8_t i = 0; i < _NumofSensor_; i++)
  {
    _sensorPins[i] = _pins[i];
    _min_sensor_values[i] = 1023;
    _max_sensor_values[i] = 0;
  }

}
void setSensorMin(const int * _MinSensor)
{
  for (uint8_t i = 0; i < _NumofSensor; i++)
  {
    _min_sensor_values[i] = _MinSensor[i];
  }
}
void setSensorMax(const int * _MaxSensor)
{
  for (uint8_t i = 0; i < _NumofSensor; i++)
  {
    _max_sensor_values[i] = _MaxSensor[i];
  }
}
void setSensitive(const uint16_t  _SensorSensitive)
{
  _Sensitive = _SensorSensitive;
}
void setFrontLineColor(const uint16_t  _setFrontLineColor)     // if Value = 1 is BlackLine ,value = 0 is WhiteLine
{
  FrontLineColor = _setFrontLineColor;
}
int refSensor(int ch){
  return ( _max_sensor_values[ch] + _min_sensor_values[ch] ) / 2 ;
}

int readSensorMinValue(uint8_t _Pin) {
  return _min_sensor_values[_Pin];
}
int readSensorMaxValue(uint8_t _Pin) {
  return _max_sensor_values[_Pin];
}
int ReadLightSensor(int analog_CH) {
  int value = 0;

  if(FrontLineColor == 0)value= map(ADC(_sensorPins[analog_CH]), _min_sensor_values[analog_CH], _max_sensor_values[analog_CH], 100, 0);
  else if (FrontLineColor == 1) value= map(ADC(_sensorPins[analog_CH]), _min_sensor_values[analog_CH], _max_sensor_values[analog_CH], 0, 100);
  if(value < 0)value = 0;
  else if(value >100)value = 100;
  return value;
}
void showGraph(){
  int state_waitSW1 = 0;
  pinMode(6, INPUT_PULLUP);
  tft_.fillScreen(ST77XX_BLACK);

  do {
    //tft_.fillRect(20,0,140,128,TFT_BLACK);
    for(int i = 0;i<_NumofSensor;i++){
      tft_.setTextColor(TFT_WHITE, TFT_BLACK);
      tft_.setTextSize(1);
      tft_.drawString("A"+String(i)+"=",0,10*i);
      tft_.fillRect(20,10*i,100,5,TFT_BLACK);
      tft_.fillRect(20,10*i,ReadLightSensor(i),5,TFT_ORANGE);
      tft_.drawString(String(ReadLightSensor(i))+"  ",130,10*i);
    }
    tft_.setTextSize(2);

    static unsigned long lastTimeUpdateBackground = 0;
    static bool flagBackground = false;

    if(millis()-lastTimeUpdateBackground >= 100){
      lastTimeUpdateBackground = millis();
      flagBackground =! flagBackground;
      tft_.setTextColor(flagBackground?TFT_RED:TFT_GREEN, flagBackground?TFT_BLUE:TFT_YELLOW);
      tft_.drawString("  SW1 Press  ",0,115);
    }
    delay(50);
  } while (digitalRead(6) == 1);
  tft_.fillScreen(ST77XX_BLACK);
  buzzer(500,100);
 }
void setCalibrate(int cal_round) {
  tft_.fillScreen(ST77XX_BLACK);
  if(_NumofSensor <= 0){
    printText(0,20,"  No Sensors ",2,ST77XX_WHITE);
    printText(0,50,"   Defined   ",2,ST77XX_WHITE);
    while(1){

    }
    //printText(0,70,"No Sensors Defined",2,ST77XX_WHITE);
  }
  printText(0,10,"  Calibrate  ",2,ST77XX_WHITE);
  printText(0,50,"   Sensor  ",2,ST77XX_WHITE);

  tft_.setTextColor(TFT_WHITE, TFT_BLACK);
  tft_.setTextSize(2);
      
  for (int round_count = 0; round_count < cal_round; round_count ++ ) {
    tft_.drawString("Count="+String(cal_round-round_count)+"   ",0,90);

    for (uint8_t i = 0; i < _NumofSensor; i++)
    {
      if (ADC(_sensorPins[i]) > _max_sensor_values[i] || _max_sensor_values[i] > 1023 ) {
        _max_sensor_values[i]  = ADC(_sensorPins[i]);
        if (_max_sensor_values[i] > 1023 )_max_sensor_values[i] = 1023;
      }
    }
    for (uint8_t i = 0; i < _NumofSensor; i++)
    {
      if (ADC(_sensorPins[i]) < _min_sensor_values[i] || _min_sensor_values[i] == 0) {
        _min_sensor_values[i] = ADC(_sensorPins[i]);
        if (_min_sensor_values[i] < 0) _min_sensor_values[i] = 0;
      }
    }
  }
  for (uint8_t i = 0; i < _NumofSensor; i++)
  {
    _max_sensor_values[i] =  _max_sensor_values[i];
    _min_sensor_values[i] = _min_sensor_values[i];
  }
  tft_.fillScreen(ST77XX_BLACK);
  tft_.setTextSize(1);
  tft_.setTextColor(ST77XX_WHITE);
  if(_NumofSensor >0){tft_.setCursor(0, 0);tft_.print("A0 >> Min="+String(readSensorMinValue(0))+"  Max="+String(readSensorMaxValue(0)));}
  if(_NumofSensor >1){tft_.setCursor(0, 10);tft_.print("A1 >> Min="+String(readSensorMinValue(1))+"  Max="+String(readSensorMaxValue(1)));}
  if(_NumofSensor >2){tft_.setCursor(0, 20);tft_.print("A2 >> Min="+String(readSensorMinValue(2))+"  Max="+String(readSensorMaxValue(2)));}
  if(_NumofSensor >3){tft_.setCursor(0, 30);tft_.print("A3 >> Min="+String(readSensorMinValue(3))+"  Max="+String(readSensorMaxValue(3)));}
  if(_NumofSensor >4){tft_.setCursor(0, 40);tft_.print("A4 >> Min="+String(readSensorMinValue(4))+"  Max="+String(readSensorMaxValue(4)));}
  if(_NumofSensor >5){tft_.setCursor(0, 50);tft_.print("A5 >> Min="+String(readSensorMinValue(5))+"  Max="+String(readSensorMaxValue(5)));}
  if(_NumofSensor >6){tft_.setCursor(0, 60);tft_.print("A6 >> Min="+String(readSensorMinValue(6))+"  Max="+String(readSensorMaxValue(6)));}
  if(_NumofSensor >7){tft_.setCursor(0, 70);tft_.print("A7 >> Min="+String(readSensorMinValue(7))+"  Max="+String(readSensorMaxValue(7)));}
  if(_NumofSensor >8){tft_.setCursor(0, 80);tft_.print("A8 >> Min="+String(readSensorMinValue(8))+"  Max="+String(readSensorMaxValue(8)));}
  if(_NumofSensor >9){tft_.setCursor(0, 90);tft_.print("A9 >> Min="+String(readSensorMinValue(9))+"  Max="+String(readSensorMaxValue(9)));}
  if(_NumofSensor >10){tft_.setCursor(0, 100);tft_.print("A10 >> Min="+String(readSensorMinValue(10))+"  Max="+String(readSensorMaxValue(10)));}

}


int readline()
{
  bool onLine = false;
  long avg = 0;
  long sum = 0;
  for (uint8_t i = 0; i < _NumofSensor; i++)
  {
    long value = ReadLightSensor(i);
    // long value =  0 ;
    // if( FrontLineColor == 0)value = map(ADC(_sensorPins[i]), _min_sensor_values[i], _max_sensor_values[i], 1000, 0);
    // else value = map(ADC(_sensorPins[i]), _min_sensor_values[i], _max_sensor_values[i], 0, 1000);
    // if(value < 0)value = 0;
    if (value > _Sensitive) {
      onLine = true;
    }
    if (value > 5)
    {
      avg += (long)value * (i * 100);
      sum += value;
    }
  }
  if (!onLine)
  {
    if (_lastPosition < (_NumofSensor - 1) * 100 / 2)
    {
      return 0;
    }
    else
    {
      return (_NumofSensor - 1) * 100;
    }
  }
  _lastPosition = avg / sum;
  return _lastPosition;
}

void lineFollow_PID(int RUN_PID_speed , float RUN_PID_KP, float RUN_PID_KI, float RUN_PID_KD) {

  int speed_PID = RUN_PID_speed;
  int present_position = readline();
  int setpoint = ((_NumofSensor - 1) * 100) / 2;
  errors = present_position - setpoint;
  if (errors == 0) integral = 0;
  integral = integral + errors ;
  derivative = (errors - previous_error) ;
  output = RUN_PID_KP * errors  + RUN_PID_KI * integral + RUN_PID_KD * derivative;
  //int max_output = RUN_PID_speed;
  // if (output > max_output)output = max_output;
  // else if (output < -max_output)output = -max_output;

  int motorL = constrain(RUN_PID_speed + output, -RUN_PID_speed, RUN_PID_speed);
  int motorR = constrain(RUN_PID_speed - output, -RUN_PID_speed, RUN_PID_speed);
  // if(m1Speed < 0 )m1Speed = 0;
  // if(m2Speed < 0 )m2Speed = 0;

  motor(1,motorL);
  motor(2,motorR);
  previous_error = errors;
  delay(1);

}
void run_PID(int RUN_PID_speed , int RUN_PID_Mspeed, float RUN_PID_KP, float RUN_PID_KD) {
  int speed_PID = RUN_PID_speed;
  int present_position = readline();
  int setpoint = ((_NumofSensor - 1) * 100) / 2;
  errors = present_position - setpoint;
  integral = integral + errors ;
  derivative = (errors - previous_error) ;
  output = RUN_PID_KP * errors  + RUN_PID_KD * derivative;
    previous_error = errors;
  int max_output = RUN_PID_Mspeed;
  if (output > max_output)output = max_output;
  else if (output < -max_output)output = -max_output;
  int m1Speed = speed_PID + output ;
  int m2Speed = speed_PID - output;
  if(m1Speed < 0 )m1Speed = 0;
  if(m2Speed < 0 )m2Speed = 0;

  motor(1,m1Speed);
  motor(2,m2Speed);
  delay(1);
    previous_error = errors;

}
//*********************************************************  TC01Sensor **********************************************************************************************
//*********************************************************  TC01Sensor **********************************************************************************************
//*********************************************************  TC01Sensor **********************************************************************************************

int _sensorPins_B[20];
int _NumofSensor_B = 0;
int _min_sensor_values_B[20];
int _max_sensor_values_B[20];
int _lastPosition_B = 0;
int _Sensitive_B  = 20;
int stateOfRunPID_B = 0;
float  errors_B = 0, output_B = 0, integral_B = 0, derivative_B = 0, previous_error_B = 0;
void setSensorPins_B(const int * _pins, int _NumofSensor_)
{
  _NumofSensor_B = _NumofSensor_;
  // _sensorPins = (int *)realloc(_sensorPins, sizeof(int) * _NumofSensor_);
  // _min_sensor_values = (int *)realloc(_min_sensor_values, sizeof(int) * _NumofSensor_);
  // _max_sensor_values = (int *)realloc(_max_sensor_values, sizeof(int) * _NumofSensor_);
  for (uint8_t i = 0; i < _NumofSensor_B; i++)
  {
    _sensorPins_B[i] = _pins[i];
    _min_sensor_values_B[i] = 1023;
    _max_sensor_values_B[i] = 0;
  }

}
void setSensorMin_B(const int * _MinSensor)
{
  for (uint8_t i = 0; i < _NumofSensor_B; i++)
  {
    _min_sensor_values_B[i] = _MinSensor[i];
  }
}
void setSensorMax_B(const int * _MaxSensor)
{
  for (uint8_t i = 0; i < _NumofSensor_B; i++)
  {
    _max_sensor_values_B[i] = _MaxSensor[i];
  }
}
void setBackLineColor(const uint16_t  setBackLineColor)     // if Value = 1 is BlackLine ,value = 0 is WhiteLine
{
  BackLineColor = setBackLineColor;
}
int refSensor_B(int ch){
  return ( _max_sensor_values_B[ch] + _min_sensor_values_B[ch] ) / 2 ;
}
int ReadLightSensor_B(int analog_CH) {
  int value = 0;

  if(BackLineColor == 0)value= map(ADC(_sensorPins_B[analog_CH]), _min_sensor_values_B[analog_CH], _max_sensor_values_B[analog_CH], 100, 0);
  else if (BackLineColor == 1) value= map(ADC(_sensorPins_B[analog_CH]), _min_sensor_values_B[analog_CH], _max_sensor_values_B[analog_CH], 0, 100);
  if(value < 0)value = 0;
  else if(value >100)value = 100;
  return value;
}
int readline_B()
{
  bool onLine = false;
  long avg = 0;
  long sum = 0;
  for (uint8_t i = 0; i < _NumofSensor_B; i++)
  {
    long value =  ReadLightSensor_B(i);
    // if( BlackLineColor == 0)value = map(ADC(_sensorPins_B[i]), _min_sensor_values_B[i], _max_sensor_values_B[i], 100, 0);
    // else value = map(ADC(_sensorPins_B[i]), _min_sensor_values_B[i], _max_sensor_values_B[i], 0, 100);
    if (value > _Sensitive_B) {
      onLine = true;
    }
    if (value > 5)
    {
      avg += (long)value * (i * 100);
      sum += value;
    }
  }
  if (!onLine)
  {
    if (_lastPosition_B < (_NumofSensor_B - 1) * 100 / 2)
    {
      return 0;
    }
    else
    {
      return (_NumofSensor_B - 1) * 100;
    }
  }
  _lastPosition_B = avg / sum;
  return _lastPosition_B;
}
void run_PID_B(int RUN_PID_speed , int RUN_PID_Mspeed, float RUN_PID_KP, float RUN_PID_KD) {
  int speed_PID = RUN_PID_speed;
  int present_position = readline_B();
  int setpoint = ((_NumofSensor_B - 1) * 100) / 2;
  errors = present_position - setpoint;
  integral = integral + errors ;
  derivative = (errors - previous_error) ;
  output = RUN_PID_KP * errors  + RUN_PID_KD * derivative;
    previous_error = errors;
  int max_output = RUN_PID_Mspeed;
  if (output > max_output)output = max_output;
  else if (output < -max_output)output = -max_output;
  int m1Speed = speed_PID - output ;
  int m2Speed = speed_PID + output;
  if(m1Speed < 0 )m1Speed = 0;
  if(m2Speed < 0 )m2Speed = 0;

  motor(1,-m1Speed);
  motor(2,-m2Speed);
  delay(1);
    previous_error_B = errors;

}
void setCalibrate_B(int cal_round) {
  tft_.fillScreen(ST77XX_BLACK);
  if(_NumofSensor_B <= 0){
    printText(0,20,"  No Sensors ",2,ST77XX_WHITE);
    printText(0,50,"   Defined   ",2,ST77XX_WHITE);
    while(1){

    }
    //printText(0,70,"No Sensors Defined",2,ST77XX_WHITE);
  }
  printText(0,10," Calibrate_B ",2,ST77XX_WHITE);
  printText(0,50,"   Sensor  ",2,ST77XX_WHITE);

  tft_.setTextColor(TFT_WHITE, TFT_BLACK);
  tft_.setTextSize(2);
      
  for (int round_count = 0; round_count < cal_round; round_count ++ ) {
    tft_.drawString("Count="+String(cal_round-round_count)+"   ",0,90);

    for (uint8_t i = 0; i < _NumofSensor_B; i++)
    {
      if (ADC(_sensorPins_B[i]) > _max_sensor_values_B[i] || _max_sensor_values_B[i] > 1023 ) {
        _max_sensor_values_B[i]  = ADC(_sensorPins_B[i]);
        if (_max_sensor_values_B[i] > 1023 )_max_sensor_values_B[i] = 1023;
      }
    }
    for (uint8_t i = 0; i < _NumofSensor_B; i++)
    {
      if (ADC(_sensorPins_B[i]) < _min_sensor_values_B[i] || _min_sensor_values_B[i] == 0) {
        _min_sensor_values_B[i] = ADC(_sensorPins_B[i]);
        if (_min_sensor_values_B[i] < 0) _min_sensor_values_B[i] = 0;
      }
    }
  }
  for (uint8_t i = 0; i < _NumofSensor_B; i++)
  {
    _max_sensor_values_B[i] =  _max_sensor_values_B[i];
    _min_sensor_values_B[i] = _min_sensor_values_B[i];
  }
  tft_.fillScreen(ST77XX_BLACK);
  tft_.setTextSize(1);
  tft_.setTextColor(ST77XX_WHITE);
  if(_NumofSensor_B >0){tft_.setCursor(0, 0);tft_.print("A0 >> Min="+String(readSensorMinValue(0))+"  Max="+String(readSensorMaxValue(0)));}
  if(_NumofSensor_B >1){tft_.setCursor(0, 10);tft_.print("A1 >> Min="+String(readSensorMinValue(1))+"  Max="+String(readSensorMaxValue(1)));}
  if(_NumofSensor_B >2){tft_.setCursor(0, 20);tft_.print("A2 >> Min="+String(readSensorMinValue(2))+"  Max="+String(readSensorMaxValue(2)));}
  if(_NumofSensor_B >3){tft_.setCursor(0, 30);tft_.print("A3 >> Min="+String(readSensorMinValue(3))+"  Max="+String(readSensorMaxValue(3)));}
  if(_NumofSensor_B >4){tft_.setCursor(0, 40);tft_.print("A4 >> Min="+String(readSensorMinValue(4))+"  Max="+String(readSensorMaxValue(4)));}
  if(_NumofSensor_B >5){tft_.setCursor(0, 50);tft_.print("A5 >> Min="+String(readSensorMinValue(5))+"  Max="+String(readSensorMaxValue(5)));}
  if(_NumofSensor_B >6){tft_.setCursor(0, 60);tft_.print("A6 >> Min="+String(readSensorMinValue(6))+"  Max="+String(readSensorMaxValue(6)));}
  if(_NumofSensor_B >7){tft_.setCursor(0, 70);tft_.print("A7 >> Min="+String(readSensorMinValue(7))+"  Max="+String(readSensorMaxValue(7)));}
  if(_NumofSensor_B >8){tft_.setCursor(0, 80);tft_.print("A8 >> Min="+String(readSensorMinValue(8))+"  Max="+String(readSensorMaxValue(8)));}
  if(_NumofSensor_B >9){tft_.setCursor(0, 90);tft_.print("A9 >> Min="+String(readSensorMinValue(9))+"  Max="+String(readSensorMaxValue(9)));}
  if(_NumofSensor_B >10){tft_.setCursor(0, 100);tft_.print("A10 >> Min="+String(readSensorMinValue(10))+"  Max="+String(readSensorMaxValue(10)));}
}
