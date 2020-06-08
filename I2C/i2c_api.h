#include <cstddef>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <cstdlib>
#include <cstdio>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <stdint.h>
#include <i2c/smbus.h>


class I2C_Object
{

public:
    unsigned char I2CBus;
    int I2CFileDescriptor;
    int I2CAddress;

    int error;
    
    I2C_Object(int address);
  
    bool openI2C();
    void closeI2C();
  
    int I2Cwrite(int writeValue);
  
    void begin();
    void end();
    
    size_t write(uint8_t c);
};
