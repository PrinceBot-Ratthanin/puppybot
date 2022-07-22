

//คำสั่งสำหรับการใช้งานหุ่นยนต์


// puppybot_setup();    คือคำสั่งสำหรับการเริ่มต้นการใช้งานของบอร์ด วางอยู่ฟังก์ชั้น Setup เป็นคำสั่งแรกสุด

// wait_SW1();      คือฟังก์ชั่นสำหรับ การรอจนกว่าจะกด สวิตช์ที่ 1 หรือ SW1

// ADC( ช่องที่เลือกอ่าน );          คือคำสั่งสำหรับการอ่านข้อมูลแบบ Analog โดยสามารถระบุช่องที่อ่านได้ 
// ตัวอย่างการใช้งาน ADC(1);   อ่านข้อมูล Analog ที่ช่อง A1

// buzzer( ความถี่ , ระยะเวลา );  คือคำสั่งสำหรับการสั่งงาน ลำโพง Buzzer  โดยสามารถระบุ ความถี่และระยะเวลาได้
// ตัวอย่างการใช้งาน buzzer(500,1000);     สั่งงาน Buzzer ด้วยความถี่ 500 เป็นระยะเวลา  1 วินาที 

// motor( ช่องมอเตอร์ , ความเร็วมอเตอร์ );    คำสั่งสำหรับการสั่งงานมอเตอร์ให้ทำงาน โดยสามารถเลือก ช่อง ความเร็ว และทิศทาง
// ตัวอย่างการใช้งาน motor(1,100);      สั่งงานมอเตอร์ช่อง 1  ให้เคลื่อนที่ ไปด้านหน้า โดยใช้ความเร็ว  100 %
// ตัวอย่างการใช้งาน motor(2,-50);      สั่งงานมอเตอร์ช่อง 2  ให้เคลื่อนที่ กลับหลัง  โดยใช้ความเร็ว  50 %

// printText(0,Y,"Hello world!",2,ST77XX_WHITE);  คำสั่งแสดงผล Hellow ไปยังหน้าจอที่ตำแหน่ง X = 0 , Y = 10
                                           // โดยให้ตัวหนังสือมีขนาด  2  และตัวหนังสิอสี ขาว

// motorStop(0);           คำสั่งสำหรับหยุดหยุดการเคลื่อนที่ของมอเตอร์ แบบ หยุดจ่ายไฟให้มอเตอร์ 
//                          0 คือ หยุดทั้งสองมอเตอร์ 
//                          1 คือ หยุดมอเตอร์ตั้วที่  1  
//                          2 คือ หยุดมอเตอร์ตั้วที่  2  

// motorBreak(0);          คำสั่งสำหรับการหยุดการเคลื่อนที่ของมอเตอร์ แบบ  เบรคมอเตอร์ (จะหยุดได้แม่นยำกว่า)
//                          0 คือ หยุดทั้งสองมอเตอร์ 
//                          1 คือ หยุดมอเตอร์ตั้วที่  1  
//                          2 คือ หยุดมอเตอร์ตั้วที่  2  

// servoRun( ช่องของเซอร์โว  ,  ตำแหน่ง หน่วยองศา );    คือคำสั่งสำหรับการสั่งงาน Servo motor ให้ทำงานไปยังตำแหน่งที่กำหนด
//ตัวอย่างการใช้งาน servoRun( 1 ,90)    คือ ให้เซอร์โวมอเตอร์ตัวที่ 1 หมุนไปตำแหน่งที่ 90 องศา 

// setSensorPins(ตำแหน่งของเซ็นเซอร์ , จำนวนการเชื่อมต่อเซ็นเซอร์ );  //ตำสั่งสำหรับการเลือกการใช้งาน เซ็นเซอร์สำหรับการวิ่งตามเส้นแบบ PID
// ตัวอย่างการใช้งาน setSensorPins((const int[]) {0, 1, 2 , 3, 4}, 5);  
//    คือการกำหนดการเชื่อมต่อเซ็นเซอร์ซ้ายสุดต่อทช่อง A0 ไปจนถึง A4 รวมจำนวนเซ็นเซอร์แล้วเท่ากับ  5 ตัว 

// setSensorMin ( ค่าของเซ็นเซอร์ที่อ่านได้จำนวนน้อยที่สุด );   คำสั่งสำหรับการตั้งค่า ข้อมูลที่อ่านได้จากเซ็นเซอร์ น้อยที่สุด
// ตัวอย่างการใช้งาน setSensorMin((const int[]){100, 100,100,100,100});    กำหนดให้ทุกเซ็นเซอร์มีค่า น้อยที่สุด เท่ากับ 100

// setSensorMin ( ค่าของเซ็นเซอร์ที่อ่านได้จำนวนน้อยที่สุด );   คำสั่งสำหรับการตั้งค่า ข้อมูลที่อ่านได้จากเซ็นเซอร์ มากที่สุด
// ตัวอย่างการใช้งาน setSensorMax((const int[]){1000, 1000,1000,1000,1000});    กำหนดให้ทุกเซ็นเซอร์มีค่า มากที่สุด เท่ากับ 1000

// setCalibrate();      คือคำสั่งสำหรับการอ่านข้อมูลที่ได้จากเซ็นเซอร์เอง และสามารถระบุ ค่ามากสุดและค่าน้อยสุดของแต่ละเซ็นเซอร์

// readline();          คือคำสั่งสำหรับการอ่าน ต่ำแหน่งของเส้นที่เซ็นเซอร์ของเราอ่านข้อมูลได้ เมื่อเส้นอยู่ซ้ายมือแสดงว่าคำแหน่งก็จะมีค่าน้อยๆ 
                       //เมื่อเส้นอยู่ตรงการของหุ่นยนต์ ค่าที่อ่านได้มีค่าเท่ากับ 0 
                       //เมื่อเส้นอยู่ด้านขวามือ แสดงว่าค่าที่อ่านได้จะมีค่ามากกว่า 0 

// run_PID(25,35,0.5,10);   ตำสั่งสำหรับการให้หุ่นยนต์วิ่งตามเส้นโดยใช้สมการ PID
                            // เราสามารถกำหนด ความเร็ว และค่าคงที่ KP และ KD
// ตัวอย่างการใช้งาน run_PID(25,35,0.5,10);  =  วิ่งตามเส้น ด้วยความเร็วเริ่มต้น 25 และมากสุดที่ 35  โดยมีค่า KP = 0.5 KD = 1 
