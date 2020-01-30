#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/*
''' 
-------------------
Written by ECE team
-------------------

This code sets a pins state to be enabled or disabled, the output of the pin is still low.
Manufacturing samples for the software team so they can write their own library in C
'''
*/
#define gpio_dir "/sys/class/gpio/"
#define gpio_setter "sys/class/gpio/gpio"

enum state {off = 0, on = 1};
enum output {high = 1, low = 0};

void setPin(int pin, state isOn)
{
  if (isOn)
    {
      FILE *fd = fopen(gpio_dir "export", "a");
      fprintf(fd, "%d", pin);
      fclose(fd);
    }
  else
    {
      FILE *fd = fopen(gpio_dir "unexport", "a");
      fprintf(fd, "%d", pin);
      fclose(fd);
    }
}

void outputPin(int pin, output setHigh)
{
  char dir[100] = "sys/class/gpio/gpio";
  char buffer[5];

  if (pin >= 10000)
    printf("Warning: Buffer Overflow -- Check pin number\n");
  
  if(setHigh)
    {
      sprintf(buffer, "%d", pin);
      strcat(dir, buffer);
      
      FILE *fd = fopen(dir , "a");
      fprintf(fd, "%d", high);
      fclose(fd);
    }
  else
    {
      sprintf(buffer, "%d", pin);
      strcat(dir, buffer);
      
      FILE *fd = fopen(dir, "a");
      fprintf(fd, "%d", low);
      fclose(fd);
    }  
}

int main(int argc, char *argv[])
{
  if (argc <= 2) {
    printf("Please read the code\n");
    return 0;
  }
  if (!strcmp(argv[1], "export")) {
    int pin = atoi(argv[2]);
    
    setPin(pin, on);
  }
  if (!strcmp(argv[1], "unexport")) {
    int pin = atoi(argv[2]);
    setPin(pin, off);
  }

  if (!strcmp(argv[1], "high")){
    int pin = atoi(argv[2]);
    outputPin(pin, high);
    }

  if (!strcmp(argv[1], "low")){
    int pin = atoi(argv[2]);
    outputPin(pin, low);
    }
  
}


