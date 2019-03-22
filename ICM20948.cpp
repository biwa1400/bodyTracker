#include <Wire.h>
#include<Arduino.h>
#include "ICM20948.h"

const uint8_t I2C_addr_ec = 0x0c; // I2C address for AK09916
const uint8_t EC_REG_ADDR_start = 0x11; // I2C address for AK09916


Sensor::Sensor(uint8_t addr){
  I2C_addr = addr;
  init_sen();
}


void Sensor::printAddr(){
 // Serial.println(this->I2C_addr,HEX);
}

void Sensor::init_sen(){
      changeBank(0);
      // set up
      writeReg_single(this->I2C_addr,0x06,0x01); 
      // sample rate 
      writeReg_single(this->I2C_addr,0x05,0x40);
      //I2C_MST_EN
      writeReg_single(this->I2C_addr,0x03,(1<<5)); 
     changeBank(3);
     configAK09916();
     changeBank(0);
  }


 void Sensor::changeBank(uint8_t bankNum){
  writeReg_single(this->I2C_addr,0x7f,bankNum<<4);
 }

  void  Sensor::readAll(){
    uint8_t addr = 0x2d;
    int bytes_num = 20;
    uint8_t buf[bytes_num];

     readReg_burst(this->I2C_addr,addr,bytes_num,buf);

    short accel_x= (buf[0]<<8)+buf[1];
    short accel_y= (buf[2]<<8)+buf[3];
    short accel_z= (buf[4]<<8)+buf[5];

    const float accel_sen = 16.384*1000;
    accel_x_g = accel_x/accel_sen;
    accel_y_g = accel_y/accel_sen;
    accel_z_g = accel_z/accel_sen;

    short gyro_x= (buf[6]<<8)+buf[7];
    short gyro_y= (buf[8]<<8)+buf[9];
    short gyro_z= (buf[10]<<8)+buf[11];

    const float gyro_sen = 131;
    
    //float gyro_x_dps = gyro_x/gyro_sen;
    gyro_x_dps = gyro_x/gyro_sen;
    gyro_y_dps = gyro_y/gyro_sen;
    gyro_z_dps = gyro_z/gyro_sen;

  


    int temp= (buf[12]<<8)+buf[13];
    temp_c = temp/333.87+21;

    short mag_x= (buf[15]<<8)+buf[14];
    short mag_y= (buf[17]<<8)+buf[16];
    short mag_z= (buf[19]<<8)+buf[18];

    
    const float mag_unit = 0.15;
    mag_x_tesla = mag_x*mag_unit;
    mag_y_tesla = mag_y*mag_unit;
    mag_z_tesla = mag_z*mag_unit;

    addr = this->I2C_addr;
/*
    Serial.print("Device address:");
    Serial.println(this->I2C_addr,HEX);
    Serial.print("accel/g:");
    Serial.print("x:");
    Serial.print(accel_x_g);
    Serial.print(",y:");
    Serial.print(accel_y_g);
    Serial.print(",z:");
    Serial.print(accel_z_g);
    Serial.println("");

    Serial.print("gyro: /dps");
    Serial.print("x:");
    Serial.print(gyro_x_dps);
    Serial.print(",y:");
    Serial.print(gyro_y_dps);
    Serial.print(",z:");
    Serial.print(gyro_z_dps);
    Serial.println("");
   
    Serial.print("temp:");
    Serial.print(temp_c);
    Serial.println("");

    Serial.print("mag: /uT");
    Serial.print("x:");
          
    Serial.print(mag_x_tesla);
    Serial.print(",y:");
    Serial.print(mag_y_tesla);
  
    Serial.print(",z:");
    Serial.print(mag_z_tesla);
    Serial.println("");
    */

//delay(1000);
 //mag_z_tesla *= 1;
}


// change to Back 3 before calling it
void Sensor::configAK09916(){
  // set master sample rate ODR
        writeReg_single(this->I2C_addr,0x00,0x0a); 
  //  Operation mode setting,  Continuous measurement mode 4

      ext_writeReg(1,0x31,0x08);
      ext_readSetting(0,0x11,6,true);

  }

void Sensor::ext_writeReg(uint8_t ext_num, uint8_t addrs,uint8_t values){
     // physic addr     
     uint8_t value = I2C_addr_ec;
     writeReg_single(this->I2C_addr,0x03+(ext_num*4),value);    
     
     //  register addr
     writeReg_single(this->I2C_addr,0x04+(ext_num*4),addrs); 

     // output value
     writeReg_single(this->I2C_addr,0x06+(ext_num*4),values);  

     // enable
     writeReg_single(this->I2C_addr,0x05+(ext_num*4),0x81);  
  }


  void Sensor::ext_readSetting(uint8_t ext_num,uint8_t addr,uint8_t number,bool isRegAddr){
     
     // physic addr
     uint8_t  value = I2C_addr_ec|0x80;
     writeReg_single(this->I2C_addr,0x03+(ext_num*4),value); 
        
     //  register addr
     writeReg_single(this->I2C_addr,0x04+(ext_num*4),addr); 
     
     // enable , bytes
     if (isRegAddr == true)
     {
      value = 0x80|number;
      }
      else
      {
        value = 0x80|number|0x20;
       }
     writeReg_single(this->I2C_addr,0x05+(ext_num*4),value); 
  
  }



  



void readReg_burst(uint8_t dev_addr,uint8_t reg_addr,int bytes_num,uint8_t* value_buf){
   Wire.beginTransmission(dev_addr); // transmit to device #8
   Wire.write(reg_addr);              // sends one byte
   Wire.endTransmission();
   Wire.requestFrom(dev_addr,bytes_num); // transmit to device #8

   for (int i;i<bytes_num;i++){
       while (!Wire.available()) {}// slave may send less than requested
           value_buf[i] =(uint8_t)Wire.read(); // receive a byte as character
            //Serial.println(i,HEX);
            //Serial.println(value_buf[i],HEX);
    }
  //Wire.endTransmission();
}

uint8_t readReg_single(uint8_t dev_addr,uint8_t reg_addr){
     uint8_t result;
     
   Wire.beginTransmission(dev_addr); // transmit to device #8
   Wire.write(reg_addr);              // sends one byte
   Wire.endTransmission();
   Wire.requestFrom(dev_addr,1); // transmit to device #8

   while (Wire.available()) { // slave may send less than requested
    result = Wire.read(); // receive a byte as character
  }
  Wire.endTransmission();
  return result;
}


void writeReg_single(uint8_t dev_addr,uint8_t reg_addr,uint8_t value){
   Wire.beginTransmission(dev_addr); // transmit to device #8
   Wire.write(reg_addr);              // sends one byte
   Wire.write(value);              // sends one byte
   Wire.endTransmission();

   //Wire.endTransmission();
}

/*
void burstReadExample(){
  uint8_t addr = 0x2d;
  int bytes_num = 6;
  uint8_t buf[bytes_num];
 
  readReg_burst(addr,bytes_num,buf);
  
  for (int i=0;i<bytes_num;i++){
      Serial.print((uint8_t)buf[i],HEX);
      Serial.print(",");
    }
    Serial.println("");
}
*/

/*
void singleReadExample(){
    uint8_t value =  readReg_single(0x01);
    Serial.println((uint8_t)value,HEX);
}

void singleWriteExample(){
      writeReg_single(0x07,0x00);
    uint8_t value =  readReg_single(0x07);
    Serial.println((uint8_t)value,HEX);
 }


 
void whoAmI(){
   Wire.beginTransmission(I2C_addr); // transmit to device #8
   Wire.write(0x0);              // sends one byte
   Wire.endTransmission();
   Wire.requestFrom(I2C_addr,1); // transmit to device #8
   
   while (Wire.available()) { // slave may send less than requested
    char c = Wire.read(); // receive a byte as character
    Serial.print((uint8_t)c,HEX);         // print the character
  }
  Wire.endTransmission();
}







 void readGYRO(){
    uint8_t addr = 0x33;
    int bytes_num = 6;
    uint8_t buf[bytes_num];

     readReg_burst(addr,bytes_num,buf);

    int x= (buf[0]<<8)+buf[1];
    int y= (buf[2]<<8)+buf[3];
    int z= (buf[4]<<8)+buf[5];

    Serial.print("GYRO:");
    Serial.print("x:");
    Serial.print(x);
    Serial.print(",y:");
    Serial.print(y);
    Serial.print(",z:");
    Serial.print(z);
    Serial.println("");
   
  }


   void readACCEL(){
    uint8_t addr = 0x2d;
    int bytes_num = 6;
    uint8_t buf[bytes_num];
    
  readReg_burst(addr,bytes_num,buf);


    int x= (buf[0]<<8)+buf[1];
    int y= (buf[2]<<8)+buf[3];
    int z= (buf[4]<<8)+buf[5];


    Serial.print("ACCEL:");
    Serial.print("x:");
    Serial.print(x);
    Serial.print(",y:");
    Serial.print(y);
    Serial.print(",z:");
    Serial.print(z);
    Serial.println("");

  }

 void readMagn(){
    uint8_t addr = 0x3b;
    int bytes_num = 6;
    uint8_t buf[bytes_num];
    
  readReg_burst(addr,bytes_num,buf);

    int x= (buf[0]<<8)+buf[1];
    int y= (buf[2]<<8)+buf[3];
    int z= (buf[4]<<8)+buf[5];
    Serial.println((uint8_t)buf[0],HEX);
    Serial.println((uint8_t)buf[1],HEX);

    Serial.print("MAGN:");
    Serial.print("x:");
    Serial.print(x);
    Serial.print(",y:");
    Serial.print(y);
    Serial.print(",z:");
    Serial.print(z);
    Serial.println("");

   
  }


*/