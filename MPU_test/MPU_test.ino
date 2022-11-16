/* Get tilt angles on X and Y, and rotation angle on Z
 * Angles are given in degrees, displays on SSD1306 OLED
 * 
 * License: MIT
 */
#include <Wire.h>
#include <MPU6050_light.h>

MPU6050 mpu(Wire);
unsigned long timer = 0;

void setup() {
  Serial.begin(19200);                           // Ensure serial monitor set to this value also    
                       
  Wire.begin();
  mpu.begin();
  mpu.calcGyroOffsets();                          // This does the calibration
}

void loop() {
  mpu.update();  
  if((millis()-timer)>10)                         // print data every 10ms
  {                                           
    Serial.print("\n P : ");
    Serial.print(mpu.getAngleX());
    Serial.print("\n R : ");
    Serial.print(mpu.getAngleY());
    Serial.print("\n Y : ");
    Serial.print(mpu.getAngleZ());
    timer = millis();  
  }
}
