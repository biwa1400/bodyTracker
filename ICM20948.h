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
};

void readReg_burst(uint8_t dev_addr,uint8_t reg_addr,int bytes_num,uint8_t* value_buf);
uint8_t readReg_single(uint8_t dev_addr,uint8_t reg_addr);
void writeReg_single(uint8_t dev_addr,uint8_t reg_addr,uint8_t value);
