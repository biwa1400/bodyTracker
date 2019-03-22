#include <Wire.h>



class Sensor
{
  private:
    uint8_t I2C_addr;
  public:
    Sensor(uint8_t addr);
    void printAddr();
    void init_sen();
    void changeBank(uint8_t bankNum);
    void configAK09916();
    void ext_writeReg(uint8_t ext_num, uint8_t addrs,uint8_t values);
    void ext_readSetting(uint8_t ext_num,uint8_t addr,uint8_t number,bool isRegAddr);
    void readAll();
    
    float accel_x_g;
    float accel_y_g;
    float accel_z_g;

    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;

    float mag_x_tesla;
    float mag_y_tesla;
    float mag_z_tesla;

    float temp_c;

    uint8_t addr;
};

void readReg_burst(uint8_t dev_addr,uint8_t reg_addr,int bytes_num,uint8_t* value_buf);
uint8_t readReg_single(uint8_t dev_addr,uint8_t reg_addr);
void writeReg_single(uint8_t dev_addr,uint8_t reg_addr,uint8_t value);