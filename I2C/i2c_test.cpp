#include <i2c_api.h>

int main()
{
    I2C_Object p(39); // 0x27 = 39 dec

    bool status = p.openI2C();

    unsigned char arr[10] = {'a','b','c','d','e','f','g','h','i','j'};
    
    for( auto &item : arr)
	int write = p.I2Cwrite(item);

    p.closeI2C();
}
