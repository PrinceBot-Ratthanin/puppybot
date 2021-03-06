#include <puppybot.h>    //ประกาศชุดคำสั่งสำหรับบอร์ด PuppyBot
//อย่าลืมประกาศทุกครั้ง หากไม่ประกาศโปรแกรมจะทำงานไม่ได้

void setup() {
  puppybot_setup();       //เรียกใช้งานฟังก์ชั้นต่าง
  setSensorPins((const int[]) {     //เลือกเซ็นเซอร์สำหรับการตามเส้น
    0, 1, 2 , 3
  }, 4);
  setSensorMax((const int[]) {      //ตัวเลขที่หุ่นยนต์อ่านได้จาก พื้นสีขาว ของเซ็นเซอร์ตัวนั้นๆ
    1000, 1000, 1000, 1000
  });
  setSensorMin((const int[]) {      //ตัวเลขที่หุ่นยนต์อ่านได้จาก เส้นสีดำ ของเซ็นเซอร์ตัวนั้นๆ
    200, 200, 200, 200
  });
  wait_SW1();                       //รอจนกว่าจะกดสวิทช์ SW1 เพื่อทำงาน
  fd(20);                         //หุ่นยนต์เคลื่อนที่ไปด้านหน้าด้วยความเร็ว 20 %
  delay(200);                     //เป็นระยะเวลา 0.2 วินาที

}

void loop() {
  run_PID(40, 50, 0.2, 0);        //วิ่งตามเส้นด้วยการใช้ระบบควบคุมแบบ PD
}
