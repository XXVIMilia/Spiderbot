#include <stdio.h>
#include <stdint.h>
#include "helper.h"
#include "metal/i2c.h"
#include "metal/timer.h"
#include "metal/rtc.h"

struct metal_i2c *i2c;
uint8_t bufWrite[24];
uint8_t bufRead[9];

inline uint64_t get_cycles(void)
{
  return *(volatile uint64_t *)(CLINT_CTRL_ADDR + CLINT_MTIME);
}

void delay(int msec)
{
  uint64_t tend; 
  tend = get_cycles() + msec * 32768 / 1000;
  while (get_cycles() < tend) {}; 
}

void delayMetal(int msec){
  unsigned long long curTime;
  u_int64_t hz;
  metal_timer_get_timebase_frequency(0,&hz);
  metal_timer_get_cyclecount(0, &curTime);
  unsigned long long waitTime = curTime + hz*(0.001 * msec);
  while(curTime < waitTime){};

}

void delay_usec(int usec)
{
  uint64_t tend; 
  tend = get_cycles() + (uint64_t)usec * 32768 / 1000000;
  while (get_cycles() < tend) {}; 
}



//A function used to quickly map [-45,45] to [155,355]
int map(int angle,int lowIn, int highIn, int lowOut, int highOut){
  int mapped = lowOut + (((float)highOut-lowOut)/((float)highIn-lowIn))*(angle-lowIn);
  return mapped;
}

//The entire setup sequence
void set_up_I2C(){
    uint8_t oldMode;
    uint8_t newMode;
    _Bool success;


    bufWrite[0] = PCA9685_MODE1;
    bufWrite[1] = MODE1_RESTART;
    printf("%d\n",bufWrite[0]);
    
    i2c = metal_i2c_get_device(0);

    if(i2c == NULL){
        printf("Connection Unsuccessful\n");
    }
    else{
        printf("Connection Successful\n");
    }
    
    //Setup Sequence
    metal_i2c_init(i2c,I2C_BAUDRATE,METAL_I2C_MASTER);
    success = metal_i2c_write(i2c,PCA9685_I2C_ADDRESS,2,bufWrite,METAL_I2C_STOP_DISABLE);//reset
    delay(100);
    printf("resetting PCA9685 control 1\n");

    //Initial Read of control 1
    bufWrite[0] = PCA9685_MODE1;//Address
    success = metal_i2c_transfer(i2c,PCA9685_I2C_ADDRESS,bufWrite,1,bufRead,1);//initial read
    printf("Read success: %d and control value is: %d\n", success, bufWrite[0]);
    
    //Configuring PCA9685 
    oldMode = bufRead[0];
    newMode = (oldMode & ~MODE1_RESTART) | MODE1_SLEEP;
    printf("sleep setting is %d\n", newMode);
    bufWrite[0] = PCA9685_MODE1;//address
    bufWrite[1] = newMode;//writing to register
    success = metal_i2c_write(i2c,PCA9685_I2C_ADDRESS,2,bufWrite,METAL_I2C_STOP_DISABLE);//sleep
    bufWrite[0] = PCA9685_PRESCALE;//Setting PWM prescale
    bufWrite[1] = 0x79;
    success = metal_i2c_write(i2c,PCA9685_I2C_ADDRESS,2,bufWrite,METAL_I2C_STOP_DISABLE);//sets prescale
    bufWrite[0] = PCA9685_MODE1;
    bufWrite[1] = 0x01 | MODE1_AI | MODE1_RESTART;
    printf("on setting is %d\n", bufWrite[1]);
    success = metal_i2c_write(i2c,PCA9685_I2C_ADDRESS,2,bufWrite,METAL_I2C_STOP_DISABLE);//awake
    delay(100);
    printf("Setting the control register\n");
    bufWrite[0] = PCA9685_MODE1;
    success = metal_i2c_transfer(i2c,PCA9685_I2C_ADDRESS,bufWrite,1,bufRead,1);//initial read
    printf("Set register is %d\n",bufRead[0]);

} 

//simple function used to break a 16 bit number into two 8 bit numbers
void breakup(int bigNum, uint8_t* low, uint8_t* high){
    uint16_t test1 = bigNum & 0xff;
    uint16_t test2 = (bigNum >> 8) & 0xff;
    uint16_t test3 = (test2 << 8) | test1;
    *low = test1;
    *high = test2;
    printf("low 8 bit is %u high 8 bit is %u whole is %d\n",test1,test2,test3);
}

void setServo(int angle,int servoNum){
    int toBreak = map(angle,0,180,SERVOMIN,SERVOMAX);
    bufWrite[0] = PCA9685_LED0_ON_L+(4*servoNum);//<-the plus 4 moves it to the second LED444444 
    bufWrite[1] = 0;//PCA9685_LED1_ON_L;
    bufWrite[2] = 0;//PCA9685_LED1_ON_H;
    breakup(toBreak,&bufWrite[3],&bufWrite[4]);//PCA9685_LED1_OFF_L is 3 and PCA9685_LED1_OFF_H is 4;
    metal_i2c_transfer(i2c,PCA9685_I2C_ADDRESS,bufWrite,5,bufRead,1);      
}

void getIMUData(){
  //Return Something? Or just print for now
}