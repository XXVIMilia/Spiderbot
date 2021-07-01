#ifndef __HELPER_H__
#define __HELPER_H__

/******************************************************************************
 *   generic definitions
 *******************************************************************************/
#define ON                  1
#define OFF                 0
#define OUTPUT              1
#define INPUT               0

#define CLINT_CTRL_ADDR     0x02000000  // CLINT block base address
#define CLINT_MTIME         0xbff8      // timer register



#define UART_ADDR(devid) (UART0_CTRL_ADDR + devid * 0x10000)
/***********************************************************
 *  Modifications for physical final project
 * ***********************************************************/
//High 5 board info
#define I2C_CTRL_ADDR       0x10016000 //Address of I2C control instance


//Setup for PCA9685
#define PCA9685_I2C_ADDRESS 0x40
#define PCA9685_MODE1 0x00      /**< Mode Register 1 */
#define PCA9685_LED0_ON_L 0x06  /**< LED0 on tick, low byte*/
#define PCA9685_PRESCALE 0xFE     /**< Prescaler for PWM output frequency */

// MODE1 bits
#define MODE1_SLEEP 0x10   /**< Low power mode. Oscillator off */
#define MODE1_AI 0x20      /**< Auto-Increment enabled */
#define MODE1_RESTART 0x80 /**< Restart enabled */
#define FREQUENCY_OSCILLATOR 25000000 /**< Int. osc. frequency in datasheet */

//Servo
#define SERVOMIN  155 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  355 // This is the 'maximum' pulse length count (out of 4096)

#define MIN_PULSE_WIDTH       800
#define MAX_PULSE_WIDTH       2200
#define DEFAULT_PULSE_WIDTH   1500
#define FREQUENCY             50



#define I2C_BAUDRATE        100000


//Setup for BNO055 (Absolute Orientation)
#define BNO055_I2C_ADDRESS 0x28
#define QUA_W_LSB_ADDRESS 0x20 // W LSB -> MSB -> X LSB -> MSB -> Y LSB -> MSB -> Z LSB - > MSB (1 write, 8 reads)
#define BNO055_OPR_MODE_ADDRESS 0x3D

//Operation mode data
#define CONFIG_MODE 0x00
#define BNO055_NDOF 0x0C




void set_up_I2C();
void breakup();
void toEuler();
/* do this in c
Vector<3> toEuler() const {
    Vector<3> ret;
    double sqw = _w * _w;
    double sqx = _x * _x;
    double sqy = _y * _y;
    double sqz = _z * _z;

    ret.x() = atan2(2.0 * (_x * _y + _z * _w), (sqx - sqy - sqz + sqw));
    ret.y() = asin(-2.0 * (_x * _z - _y * _w) / (sqx + sqy + sqz + sqw));
    ret.z() = atan2(2.0 * (_y * _z + _x * _w), (-sqx - sqy + sqz + sqw));

    return ret;
  }


*/




void clearBuffers();


/******************************************************************************
 *   eecs388 library api (similar to Arduino)
 *******************************************************************************/

void delay(int msec);
void delay_usec(int usec);
int map(int angle,int lowIn, int highIn, int lowOut, int highOut);
void setServo(int angle,int servoNum);
void getIMUData();
#endif // __HELPER_H__
