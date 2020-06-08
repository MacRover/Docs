#include "i2c_api.h"


I2C_Object::I2C_Object(int address)
{
    I2CBus = 0; // default I2C 0
    I2CAddress = address;
}


bool I2C_Object::openI2C()
{
    char fileNameBuffer[32];

    // Create the path to the correct i2c bus
    sprintf(fileNameBuffer, "/dev/i2c-%d", I2CBus);
    I2CFileDescriptor = open(fileNameBuffer, O_RDWR);

    // Check if the file was correcty opened
    if(I2CFileDescriptor < 0){
	error = errno;
	return false;
    }

    // Define the file as I2C_SLAVE and check if the address works
    if(ioctl(I2CFileDescriptor, I2C_SLAVE_FORCE, I2CAddress) < 0)
    {
	error = errno;
	return false;
    }

    return true;
    
}


void I2C_Object::closeI2C()
{
    if(I2CFileDescriptor > 0) {
	close(I2CFileDescriptor);
	I2CFileDescriptor = -1;
    }
}

int I2C_Object::I2Cwrite(int writeValue) {
    // printf("Wrote: 0x%02X  \n",writeValue) ;
    int toReturn = i2c_smbus_write_byte(I2CFileDescriptor, writeValue);
    if (toReturn < 0) {
        printf("HT16K33 Write error: %d",errno) ;
        error = errno ;
        toReturn = -1 ;
    }
    return toReturn ;

}
