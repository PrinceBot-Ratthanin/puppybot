#include <puppybot.h>
//tft.fillScreen(ST77XX_BLACK);                     //สำหรับการเปลี่ยนสีหน้าจอ ทั้งจอ
//printText(x,y,"Hellow"),2,ST77XX_WHITE);          //คำสั่งแสดงผลตัวอักษร
//tft.fillRect(x, y, width, height, ST77XX_WHITE);  //คำสั่ง วาดรูป สี่เหลี่ยม แบบทึบ
//tft.fillCircle(x, y, radius, color);              //คำสั่ง วาดรูป วงกลม
//tft.drawRect(x,y, width, height, color2);         //คำสั่ง วาดรูป สี่เหลี่ยม แบบโปร่งตรงกลาง
//tft.drawCircle(x,y, radius, color2);              //คำสั่ง วาดรูป วงกลม แบบโปร่งตรงกลาง
//tft.drawLine(x,y, x, y, color2);                  //คำสั่งสำหรับการ สร้าง เส้น 
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
  tft.fillScreen(ST77XX_BLACK);
  printText(0,0,"ADC_0 = ",2,ST77XX_WHITE);
  printText(90,20,String(ADC(0)),2,ST77XX_WHITE);
  delay(50);
  
}
