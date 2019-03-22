#include <Wire.h>
#include "ICM20948.h"

const int I2C_speed = 100000    ;

Sensor* sensor1;
Sensor* sensor2;
void setup() {
  Serial.begin(9600);  // start serial for output
  Wire.begin();
  Wire.setClock(I2C_speed);
  sensor1 = new Sensor(0x68);
  sensor2 = new Sensor(0x69);
}


void loop() {
 sensor1->readAll();
 sensor2->readAll();
  delay(1000);

}
