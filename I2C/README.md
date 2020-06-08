# I2C Code Example

## i2c_api.h

There are three types of specific i2c headers we need to include

```
#include <linux/i2c-dev.h>
```

This is the in-built i2c header provided in the linux kernel. Then we have the i2c header from i2c dev library


```
#include <i2c/smbus.h>
```

Lastly we need to define how we define how we handle input and output

```
#include <sys/ioctl.h>
```

The other headers are normal C headers that are not specific to I2C


## i2c_api.cpp

We first need to open the i2c I/O file

```
    sprintf(fileNameBuffer, "/dev/i2c-%d", I2CBus);
    I2CFileDescriptor = open(fileNameBuffer, O_RDWR);
```

We next define that we are treating the io as a slave with the I2C Address


```
ioctl(I2CFileDescriptor, I2C_SLAVE_FORCE, I2CAddress)
```

To write to the i2c bus we can use the `i2c_smbus_write_*` 

```
int toReturn = i2c_smbus_write_byte(I2CFileDescriptor, writeValue);
```

If we get an error then we are going to return `-1`


