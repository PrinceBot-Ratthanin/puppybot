#include "puppybot.h"
void followLine() {
  for (int i = 0; i < 100; i++) {
    run_PID(50, 60, 0.2, 5);
    delay(1);
  }
  do {
    run_PID(50, 60, 0.2, 5);
  } while ((ADC(0) > refSensor(0) || ADC(1) > refSensor(1))   && (ADC(3) > refSensor(3) || ADC(4) > refSensor(4)) ) ;
  motor(1, 25); motor(2, 25); delay(100);
  motorStop(0);
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
  motorBreak(0); delay(50);
}
void uTurn(int speed_motor, int time_delay) {
  motor(1, speed_motor); motor(2, -speed_motor);
  delay(time_delay);
  while (ADC(4) > refSensor(4)) {
    motor(1, speed_motor / 2); motor(2, -speed_motor / 2);
  }
  motorBreak(0); delay(50);
}
void setup() {
  poppybot_setup();
  servoRun(1, 90);
  servoRun(2, 90);
  wait_SW1();
  setSensorPins((const int[]) {
    0, 1, 2 , 3, 4
  }, 5);
  setSensorMax((const int[]) {
    850, 860, 935, 851, 960
  });
  setSensorMin((const int[]) {
    100, 100, 100, 100, 100
  });

  setSensorPins_B((const int[]) {
    8, 9, 10
  }, 3);
  setSensorMax_B((const int[]) {
    850, 860, 935
  });
  setSensorMin_B((const int[]) {
    100, 100, 100
  });

  //setCalibrate(2000);
  buzzer(800, 500);
  buzzer(500, 100);

    motor(1,50);motor(2,50);delay(200);
    followLine();
    turnLeft(50,100);


}

void loop() {

  //run_PID_B(40, 60, 0.2, 5);
    followLine();
    followLine_B();
    followLine();
    motor(1,50);motor(2,50);delay(50);
    turnLeft(50,100);
    turnLeft(40,100);
    followLine();
    followLine();
    followLine_B();
    followLine();
    motor(1,50);motor(2,50);delay(50);
    turnRight(50,100);
    turnRight(40,100);
    followLine();

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
