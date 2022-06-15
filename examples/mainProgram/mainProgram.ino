#include "puppybot.h"

void setup() {
  poppybot_setup();
  servoRun(1,90);
  servoRun(2,90);
  wait_SW1();
  setSensorPins((const int[]) {0, 1, 2 , 3, 4}, 5);
  //setSensorMax((const int[]){1000, 1000,1000,1000,1000});
  //setSensorMin((const int[]){100, 100,100,100,100});
  setCalibrate(2000);
  buzzer(800,500);
  buzzer(500,100);
}

void loop() {

//if(ADC(0) < refSensor(0) && ADC(4) < refSensor(4)){
//  motorStop();
//  servoRun(2,60);
//  delay(1000000);
//}
  run_PID(25,35,0.5,10);

}
