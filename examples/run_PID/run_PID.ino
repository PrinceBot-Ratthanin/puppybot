#include <puppybot.h>    //ประกาศชุดคำสั่งสำหรับบอร์ด PuppyBot
//อย่าลืมประกาศทุกครั้ง หากไม่ประกาศโปรแกรมจะทำงานไม่ได้

/*               ตัวอย่างคำสั่งสำหรับ PID เบื้องต้น
setSensorPins((const int[]) { 0, 1, 2 , 3,4 }, 5);    //เลือกเซ็นเซอร์สำหรับการตามเส้น
			   การเรียงตำแหน่ง     จำนวนเซนเซอร์
setSensorMax((const int[]) { 1000, 1000, 1000, 1000 , 1000});     //ค่าที่อ่านได้สูงสุด
                            เรียงตาม การเรียงตำแหน่งเซ็นเซอร์
setSensorMin((const int[]) { 200, 200, 200, 200, 200  });        // ค่าที่อ่านได้น้อยที่สุด
			       เรียงตาม การเรียงตำแหน่งเซ็นเซอร์
setCalibrate(2000);        	// การหาค่าแสงอัตโนมัติ  ใช้การคำนวน 2000 รอบ
showGraph(); 	                	//  เมื่อหาค่าสูงสุด ต่ำสุดได้แล้ว สามารถคำนวณและโชว์กราฟ
readline()					//  ฟังก์ชั่น หรือตัวแปร  อ่านตำแหน่งของเซ็นเซอร์ 
lineFollow_PID(30,0.1,0,2);    //วิ่งตามเส้นแบบมีถอยหลังช่วย 
run_PID(20, 50, 0.1, 2);          // วิ่งตามเส้นแบบไม่มีถอยหลังช่วย 
Read_sumValue_sensor();     // คำสั่งอ่านค่า % เซนเซอร์รวมกัน ทั้งหมด
Run_PID_until_frontSensor(30,0.1,2,200);   // วิ่งตามเส้นจนกว่า Sum > 200

*/

void setup() {
  puppybot_setup();       //เรียกใช้งานฟังก์ชั้นต่าง
  setSensorPins((const int[]) {     //เลือกเซ็นเซอร์สำหรับการตามเส้น
    0, 1, 2 , 3,4
  }, 5);
  setSensorMax((const int[]) {      //ตัวเลขที่หุ่นยนต์อ่านได้จาก พื้นสีขาว ของเซ็นเซอร์ตัวนั้นๆ
    900, 900, 900, 900, 900
  });
  setSensorMin((const int[]) {      //ตัวเลขที่หุ่นยนต์อ่านได้จาก เส้นสีดำ ของเซ็นเซอร์ตัวนั้นๆ
    200, 200, 200, 200, 200
  });

  // หรือไม่อยากใส่ตัวเลขเองสามารถใช้คำสั่ง หาค่าเองได้ 

    //setCalibrate(2000);        	// การหาค่าแสงอัตโนมัติ  ใช้การคำนวน 2000 รอบ
    //showGraph();


  wait_SW1();                       //รอจนกว่าจะกดสวิทช์ SW1 เพื่อทำงาน
  fd(20);                         //หุ่นยนต์เคลื่อนที่ไปด้านหน้าด้วยความเร็ว 20 %
  delay(200);                     //เป็นระยะเวลา 0.2 วินาที

    //Run_PID_until_frontSensor(30,0.1,2,200);        // วิ่งตามเส้นจนกว่า Sum > 200

}

void loop() {
  run_PID(40, 50, 0.2, 0);        //วิ่งตามเส้นด้วยการใช้ระบบควบคุมแบบ PD
   //   run_PID(ความเร็วปกติ, ความเร็วสูงสุด, ค่าคงที่ Kp, ค่าคงที่ Kd);
  //lineFollow_PID(30,0.1,0,2);   //คำสั่งวิ่งตามเส้นสามารถให้ล้อหมุนกลับหลังได้ ทำให้จับเส้นได้ดีกว่า แต่ปรับจูนยากกว่า
  //lineFollow_PID(ความเร็วปกติ,  ค่าคงที่ Kp, ค่าคงที่ Ki, ค่าคงที่ Kd);   แนะนำค่า Ki = 0
}
