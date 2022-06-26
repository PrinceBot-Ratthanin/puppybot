#include "puppybot.h"
int degree_servo1 = 90;
int degree_servo2 = 90;
void followLine_Slow() {
  for (int i = 0; i < 150; i++) {
    run_PID(25, 35, 0.1, 2);
    delay(1);
  }
  do {
    run_PID(50, 60, 0.2, 2);
  } while ((ADC(0) > refSensor(0) || ADC(1) > refSensor(1))   && (ADC(3) > refSensor(3) || ADC(4) > refSensor(4)) ) ;
  motor(1, 25); motor(2, 25); delay(200);
  motorBreak(0);
}
void followLine_Fast() {
  for (int i = 0; i < 50; i++) {
    run_PID(100, 100, 0.5, 15);
    delay(1);
  }
  do {
    run_PID(100, 100, 0.5, 15);
  } while ((ADC(0) > refSensor(0) || ADC(1) > refSensor(1))   && (ADC(3) > refSensor(3) || ADC(4) > refSensor(4)) ) ;
  //motor(1, 50); motor(2, 50); delay(100);
  //motorStop(0);
}
void followLine() {
  for (int i = 0; i < 100; i++) {
    run_PID(50, 60, 0.1, 2);
    delay(1);
  }
  do {
    run_PID(50, 60, 0.2, 2);
  } while ((ADC(0) > refSensor(0) || ADC(1) > refSensor(1))   && (ADC(3) > refSensor(3) || ADC(4) > refSensor(4)) ) ;
  motor(1, 50); motor(2, 50); delay(100);
  motorStop(0);
}
void followLine_to_can() {
  for (int i = 0; i < 150; i++) {
    run_PID(20, 30, 0.1, 0);
    delay(1);
  }
  do {
    run_PID(30, 60, 0.2, 2);
  } while (ADC(6) < 580 ) ;
  motorBreak(0);
}
void followLine_B() {
  for (int i = 0; i < 100; i++) {
    run_PID_B(40, 45, 0.1, 10);
    delay(1);
  }
  do {
    run_PID_B(40, 45, 0.1, 10);
  } while (ADC(10) > 600 || ADC(9) > 600   || (ADC(8) > 600  )) ;
  motor(1, 50); motor(2, 50); delay(100);
  motorStop(0);
}
void turnLeft(int speed_motor, int time_delay) {
  motor(1, -speed_motor); motor(2, speed_motor);
  delay(time_delay);
  while (ADC(1) > refSensor(1)) {
    motor(1, -speed_motor / 2); motor(2, speed_motor / 2);
  }
  motorBreak(0); delay(50);
}
void turnRight(int speed_motor, int time_delay) {
  motor(1, speed_motor); motor(2, -speed_motor);
  delay(time_delay);
  while (ADC(3) > refSensor(3)) {
    motor(1, speed_motor / 2); motor(2, -speed_motor / 2);
  }
  motorBreak(0); 
}
void uTurn(int speed_motor, int time_delay) {
  motor(1, speed_motor); motor(2, -speed_motor);
  delay(time_delay);
  while (ADC(4) > refSensor(4)) {
    motor(1, speed_motor / 2); motor(2, -speed_motor / 2);
  }
  motorBreak(0); 
}
void vang(){
  for(int i = degree_servo1 ; i > 0 ; i--){
    servoRun(1,i);
    delay(3);
  }
  degree_servo1 = 0;
}
void yok(){
  for(int i = degree_servo1 ; i < 90 ; i++){
    servoRun(1,i);
    delay(3);
  }
  degree_servo1 = 90;
}
void setup() {
  puppybot_setup();
  buzzer(200, 100);
  buzzer(400, 100);
  Serial.begin(115200);
  servoRun(1, degree_servo1);
  servoRun(2, degree_servo2);
  wait_SW1();
  setSensorPins((const int[]) {0, 1, 2 , 3, 4}, 5);
  setSensorMax((const int[]) {850, 860, 935, 851, 960});
  setSensorMin((const int[]) {100, 100, 100, 100, 100});

  setSensorPins_B((const int[]) {8, 9, 10}, 3);
  setSensorMax_B((const int[]) {850, 860, 935});
  setSensorMin_B((const int[]) {100, 100, 100});

  //setCalibrate(2000);
  buzzer(800, 500);
  buzzer(500, 100);

  

    motor(1,50);motor(2,50);delay(200);

    followLine_Fast();
    followLine_Fast();
    followLine_Fast();
    followLine_Fast();
    followLine();
    motorBreak(0);
    turnLeft(40,450);

    followLine_Fast();
    followLine_Fast();
    followLine_Fast();
    followLine();
    motorBreak(0);
    turnLeft(40,450);
   
    followLine();
    turnLeft(40,200);
    vang();
    followLine_to_can();
    servoRun(2,22);
    delay(400);
    yok();
    delay(200);
    turnLeft(40,250);
    followLine();
    motor(1,25);motor(2,25);delay(50);motorBreak(0);
    turnRight(40,250);
    followLine_Slow();
    motorBreak(0);
    turnLeft(40,100);
    followLine_Slow();
    motor(1, 25); motor(2, 25); delay(100);
    turnRight(40,100);
    for (int i = 0; i < 100; i++) {
      run_PID(30, 60, 0.2, 5);
      delay(1);
    }
    do {
      run_PID(30, 60, 0.2, 2);
    } while ((ADC(0) > refSensor(0) || ADC(1) > refSensor(1))   && (ADC(3) > refSensor(3) || ADC(4) > refSensor(4)) ) ;
    motorBreak(0);
    vang();
    servoRun(2,90);
    delay(400);
    yok();
    delay(500);
    
  

  


}

void loop() {
//  motor(1,100);motor(2,100);delay(1000);
//  motor(1,-100);motor(2,-100);delay(1000);
//  vang();
//  servoRun(2,22);
//  delay(400);
//  yok();
//  delay(1000);
//  vang();
//  servoRun(2,90);
//  delay(400);
//  yok();
//  delay(500);
  
//  yok();
//  delay(2000);
//  vang();
//  delay(2000);

  //run_PID_B(40, 60, 0.2, 5);
//    followLine();
//    followLine_B();
//    followLine();
//    motor(1,50);motor(2,50);delay(50);
//    turnLeft(50,100);
//    turnLeft(40,100);
//    followLine();
//    followLine();
//    followLine_B();
//    followLine();
//    motor(1,50);motor(2,50);delay(50);
//    turnRight(50,100);
//    turnRight(40,100);
//    followLine();

  //  FollowLine();
  //  turnLeft(30,10);
  //  turnLeft(30,0);
  //  //uTurn(35,0);
  //  FollowLine();
  //  turnRight(35,0);
  //  turnRight(35,0);
  //  FollowLine();

  //if(ADC(0) < refSensor(0) && ADC(4) < refSensor(4)){
  //  motorStop();
  //  servoRun(2,60);
  //  delay(1000000);
  //}
  //run_PID(25,35,0.2,5);

}
