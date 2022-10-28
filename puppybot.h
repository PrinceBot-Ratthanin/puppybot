#include <Servo.h>
#include "extend_Adafruit_GFX.h"
#include "extend_Adafruit_ST7735.h"
//#include "TC01Sensor.h"
#include <SPI.h>

#define TFT_CS        17
#define TFT_RST        21
#define TFT_DC         20


Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);
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

void puppybot_setup() {
  analogWriteResolution(10);
  analogWriteRange(1023);
  tft.initR(INITR_BLACKTAB);
  tft.setRotation(1);
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
	tft.setCursor(x, y);
	tft.setTextSize(size);
    tft.setTextColor(color);
    tft.setTextWrap(true);
    tft.print(text);
}
void wait_SW1() {
  pinMode(6, INPUT_PULLUP);
  tft.fillScreen(ST77XX_BLACK);
  do {
  	tft.fillRect(22, 0 , 50, 105, ST77XX_BLACK);
  	tft.fillRect(102, 0 , 50, 105, ST77XX_BLACK);
  	//printText(0,0," Please press     SW1",2,ST77XX_WHITE);
  	printText(0,0,"0="+String(ADC(0)),2,ST77XX_WHITE);
  	printText(80,0,"1="+String(ADC(1)),2,ST77XX_WHITE);
  	printText(0,17,"2="+String(ADC(2)),2,ST77XX_WHITE);
  	printText(80,17,"3="+String(ADC(3)),2,ST77XX_WHITE);
  	printText(0,34,"4="+String(ADC(4)),2,ST77XX_WHITE);
  	printText(80,34,"5="+String(ADC(5)),2,ST77XX_WHITE);
  	printText(0,51,"6="+String(ADC(6)),2,ST77XX_WHITE);
  	printText(80,51,"7="+String(ADC(7)),2,ST77XX_WHITE);
  	printText(0,68,"8="+String(ADC(8)),2,ST77XX_WHITE);
  	printText(80,68,"9="+String(ADC(9)),2,ST77XX_WHITE);
  	printText(0,85,"10="+String(ADC(10)),2,ST77XX_WHITE);
  	printText(80,85,"Analog",2,ST77XX_YELLOW);
  	printText(0,105,"  SW1 Press  ",2,ST77XX_WHITE);
  	delay(50);
  } while (digitalRead(6) == 1);
  tft.fillScreen(ST77XX_BLACK);
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
	analogWrite(motor1B, 0);
	analogWrite(motor1A, 0);
	analogWrite(motor2B, 0);
	analogWrite(motor2A, 0);
}
void motorStop(int motor_ch){
  	if(motor_ch < 1){
	  analogWrite(motor1B, 0);
	  analogWrite(motor1A, 0);
	  analogWrite(motor2B, 0);
	  analogWrite(motor2A, 0);
	}
	else if(motor_ch < 2 ){
	  analogWrite(motor1B, 0);
	  analogWrite(motor1A, 0);
	}
	else if(motor_ch < 3 ){
	  analogWrite(motor2B, 0);
	  analogWrite(motor2A, 0);
	}
}
void motorBreak(int motor_ch){
  if(motor_ch < 1){
	  analogWrite(motor1B, 1023);
	  analogWrite(motor1A, 1023);
	  analogWrite(motor2B, 1023);
	  analogWrite(motor2A, 1023);
	}
	else if(motor_ch < 2 ){
	  analogWrite(motor1B, 1023);
	  analogWrite(motor1A, 1023);
	}
	else if(motor_ch < 3 ){
	  analogWrite(motor2B, 1023);
	  analogWrite(motor2A, 1023);
	}
}
void fd(int speed_Motor){
	motor(1,speed_Motor);
	motor(2,speed_Motor);
}
void bk(int speed_Motor){
	motor(1,-speed_Motor);
	motor(2,-speed_Motor);
}
void tl(int speed_Motor){
	motor(2,speed_Motor);
}
void tr(int speed_Motor){
	motor(1,speed_Motor);
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
int _sensorPins[10];
int _NumofSensor = 0;
int _min_sensor_values[10];
int _max_sensor_values[10];
int _lastPosition = 0;
int _Sensitive  = 30;
int stateOfRunPID = 0;
float  errors = 0, output = 0, integral = 0, derivative = 0, previous_error = 0;
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
int refSensor(int ch){
  return ( _max_sensor_values[ch] + _min_sensor_values[ch] ) / 2 ;
}

int readSensorMinValue(uint8_t _Pin) {
  return _min_sensor_values[_Pin];
}
int readSensorMaxValue(uint8_t _Pin) {
  return _max_sensor_values[_Pin];
}
void setCalibrate(int cal_round) {
  tft.fillScreen(ST77XX_BLACK);
  printText(0,20,"  Calibrate  ",2,ST77XX_WHITE);
  printText(0,70," Sensor  ",3,ST77XX_WHITE);

  for (int round_count = 0; round_count < cal_round; round_count ++ ) {

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
    delay(1);
  }
  for (uint8_t i = 0; i < _NumofSensor; i++)
  {
    _max_sensor_values[i] =  _max_sensor_values[i] + 30;
    _min_sensor_values[i] = _min_sensor_values[i] - 30;
  }
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_WHITE);
  if(_NumofSensor >0){tft.setCursor(0, 0);tft.print("A0 >> Min="+String(readSensorMinValue(0))+"  Max="+String(readSensorMaxValue(0)));}
  if(_NumofSensor >1){tft.setCursor(0, 10);tft.print("A1 >> Min="+String(readSensorMinValue(1))+"  Max="+String(readSensorMaxValue(1)));}
  if(_NumofSensor >2){tft.setCursor(0, 20);tft.print("A2 >> Min="+String(readSensorMinValue(2))+"  Max="+String(readSensorMaxValue(2)));}
  if(_NumofSensor >3){tft.setCursor(0, 30);tft.print("A3 >> Min="+String(readSensorMinValue(3))+"  Max="+String(readSensorMaxValue(3)));}
  if(_NumofSensor >4){tft.setCursor(0, 40);tft.print("A4 >> Min="+String(readSensorMinValue(4))+"  Max="+String(readSensorMaxValue(4)));}
  if(_NumofSensor >5){tft.setCursor(0, 50);tft.print("A5 >> Min="+String(readSensorMinValue(5))+"  Max="+String(readSensorMaxValue(5)));}
  if(_NumofSensor >6){tft.setCursor(0, 60);tft.print("A6 >> Min="+String(readSensorMinValue(6))+"  Max="+String(readSensorMaxValue(6)));}
  if(_NumofSensor >7){tft.setCursor(0, 70);tft.print("A7 >> Min="+String(readSensorMinValue(7))+"  Max="+String(readSensorMaxValue(7)));}
  if(_NumofSensor >8){tft.setCursor(0, 80);tft.print("A8 >> Min="+String(readSensorMinValue(8))+"  Max="+String(readSensorMaxValue(8)));}
  if(_NumofSensor >9){tft.setCursor(0, 90);tft.print("A9 >> Min="+String(readSensorMinValue(9))+"  Max="+String(readSensorMaxValue(9)));}
  if(_NumofSensor >10){tft.setCursor(0, 100);tft.print("A10 >> Min="+String(readSensorMinValue(10))+"  Max="+String(readSensorMaxValue(10)));}
}

int readline()
{
  bool onLine = false;
  long avg = 0;
  long sum = 0;
  for (uint8_t i = 0; i < _NumofSensor; i++)
  {
    long value = map(ADC(_sensorPins[i]), _min_sensor_values[i], _max_sensor_values[i], 100, 0);
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

int _sensorPins_B[10];
int _NumofSensor_B = 0;
int _min_sensor_values_B[10];
int _max_sensor_values_B[10];
int _lastPosition_B = 0;
int _Sensitive_B  = 30;
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
int refSensor_B(int ch){
  return ( _max_sensor_values_B[ch] + _min_sensor_values_B[ch] ) / 2 ;
}
int readline_B()
{
  bool onLine = false;
  long avg = 0;
  long sum = 0;
  for (uint8_t i = 0; i < _NumofSensor_B; i++)
  {
    long value = map(ADC(_sensorPins_B[i]), _min_sensor_values_B[i], _max_sensor_values_B[i], 100, 0);
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