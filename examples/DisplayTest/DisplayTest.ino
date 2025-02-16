#include <puppybot.h>

//tft_.fillScreen(ST77XX_BLACK);                          //สำหรับการเปลี่ยนสีหน้าจอ ทั้งจอ
//printText(x,y,"Hello"),2,ST77XX_WHITE);                 //คำสั่งแสดงผลตัวอักษร
//printNumber(0,20,ADC(0),2,ST77XX_WHITE,ST77XX_BLACK);   //คำสั่งแสดงผลตัวเลข (มีการป้องกันตัวหนังสือแสดงทับกัน)
//tft_.fillRect(x, y, width, height, ST77XX_WHITE);        //คำสั่ง วาดรูป สี่เหลี่ยม แบบทึบ
//tft_.fillCircle(x, y, radius, color);                     //คำสั่ง วาดรูป วงกลม
//tft_.drawRect(x,y, width, height, color2);                //คำสั่ง วาดรูป สี่เหลี่ยม แบบโปร่งตรงกลาง
//tft_.drawCircle(x,y, radius, color2);                     //คำสั่ง วาดรูป วงกลม แบบโปร่งตรงกลาง
//tft_.drawLine(x,y, x, y, color2);                         //คำสั่งสำหรับการ สร้าง เส้น 
                                                  //โดยกำหนดจุดเริ่มต้นและจุดสิ้นสุด

void setup() {
 puppybot_setup();
 wait_SW1();
 printText(0,0,"Hello world",2,ST77XX_WHITE);
 delay(2000);
 buzzer(800,100);
 wait_SW1();
 
}

void loop() {
  
  
  //วิธีที่ 1 การแสดงผลตัวเลขที่มีการวนซ้้ำ  เราจำเป็นจะต้องเคลียหน้าจอใหม่ ด้วยวิธีการ เทสีพื้นหลังแล้วแสดงผลใหม่ทับลงไป 
  tft_.fillScreen(ST77XX_BLACK);
  printText(0,0,"ADC_0 = ",2,ST77XX_WHITE);
  printText(90,20,String(ADC(0)),2,ST77XX_WHITE);

  //วิธีที่ 2 การแสดงผลตัวเลขซ้ำๆ  โดยไม่มีการ เคลียหน้าจอ แต่เราจำเป็นจะต้องพื้นหลังเฉพาะตัวอักษรเท่านั้น 
  // printText(0,0,"ADC_0 = ",2,ST77XX_WHITE);
  // printNumber(0,20,ADC(0),2,ST77XX_WHITE,ST77XX_BLACK);


  delay(50);
  
}
