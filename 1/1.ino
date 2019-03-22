#include <Wire.h>
#include "ICM20948.h"

const int I2C_speed = 100000    ;

void setup() {
  Serial.begin(115200);  // start serial for output
  Wire.begin();
  Wire.setClock(I2C_speed);
}

void printValue(Sensor& sensor)
{
    Serial.println("");
    Serial.print("Device address:");
    Serial.println(sensor.addr,HEX);
    Serial.print("accel/g:");
    Serial.print("x:");
    Serial.print(sensor.accel_x_g);
    Serial.print(",y:");
    Serial.print(sensor.accel_y_g);
    Serial.print(",z:");
    Serial.print(sensor.accel_z_g);
    Serial.println("");

 
    Serial.print("gyro: /dps");
    Serial.print("x:");
    Serial.print(sensor.gyro_x_dps);
    Serial.print(",y:");
    Serial.print(sensor.gyro_y_dps);
    Serial.print(",z:");
    Serial.print(sensor.gyro_z_dps);
    Serial.println("");

    Serial.print("temp:");
    Serial.print(sensor.temp_c);
    Serial.println("");
   
    Serial.print("mag: /uT");
    Serial.print("x:");
    Serial.print(sensor.mag_x_tesla);
    Serial.print(",y:");
    Serial.print(sensor.mag_y_tesla);
    Serial.print(",z:");
    Serial.print(sensor.mag_z_tesla);
    Serial.println("");
    
}

void loop() {
  Sensor sensor1 = Sensor(0x68);
  Sensor sensor2 = Sensor(0x69);
  
  while(true)
  {
     sensor1.readAll();
     printValue(sensor1);
     //sensor2.readAll();
     delay(1000);

  }

}