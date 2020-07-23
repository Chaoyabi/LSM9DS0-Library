/******************************************************************************
LSM9DS0.h
Definning every register in the LSM9DS0 (both the Gyro and Accel/Magnetometer registers).
Development environment specifics:
    IDE: Mbed online compiler
    Hardware Platform: STM32 NUCLEO-f446re
******************************************************************************/
#ifndef __LSM9DS0_H__
#define __LSM9DS0_H__

#include "mbed.h"

////////////////////////////
// LSM9DS0 Gyro Registers //
////////////////////////////
#define WHO_AM_I_G          0x0F
#define CTRL_REG1_G         0x20
#define CTRL_REG2_G         0x21
#define CTRL_REG3_G         0x22
#define CTRL_REG4_G         0x23
#define CTRL_REG5_G         0x24
#define REFERENCE_G         0x25
#define STATUS_REG_G        0x27
#define OUT_X_L_G           0x28
#define OUT_X_H_G           0x29
#define OUT_Y_L_G           0x2A
#define OUT_Y_H_G           0x2B
#define OUT_Z_L_G           0x2C
#define OUT_Z_H_G           0x2D
#define FIFO_CTRL_REG_G     0x2E
#define FIFO_SRC_REG_G      0x2F
#define INT1_CFG_G          0x30
#define INT1_SRC_G          0x31
#define INT1_THS_XH_G       0x32
#define INT1_THS_XL_G       0x33
#define INT1_THS_YH_G       0x34
#define INT1_THS_YL_G       0x35
#define INT1_THS_ZH_G       0x36
#define INT1_THS_ZL_G       0x37
#define INT1_DURATION_G     0x38

//////////////////////////////////////////
// LSM9DS0 Accel/Magneto (XM) Registers //
//////////////////////////////////////////
#define OUT_TEMP_L_XM       0x05
#define OUT_TEMP_H_XM       0x06
#define STATUS_REG_M        0x07
#define OUT_X_L_M           0x08
#define OUT_X_H_M           0x09
#define OUT_Y_L_M           0x0A
#define OUT_Y_H_M           0x0B
#define OUT_Z_L_M           0x0C
#define OUT_Z_H_M           0x0D
#define WHO_AM_I_XM         0x0F
#define INT_CTRL_REG_M      0x12
#define INT_SRC_REG_M       0x13
#define INT_THS_L_M         0x14
#define INT_THS_H_M         0x15
#define OFFSET_X_L_M        0x16
#define OFFSET_X_H_M        0x17
#define OFFSET_Y_L_M        0x18
#define OFFSET_Y_H_M        0x19
#define OFFSET_Z_L_M        0x1A
#define OFFSET_Z_H_M        0x1B
#define REFERENCE_X         0x1C
#define REFERENCE_Y         0x1D
#define REFERENCE_Z         0x1E
#define CTRL_REG0_XM        0x1F
#define CTRL_REG1_XM        0x20
#define CTRL_REG2_XM        0x21
#define CTRL_REG3_XM        0x22
#define CTRL_REG4_XM        0x23
#define CTRL_REG5_XM        0x24
#define CTRL_REG6_XM        0x25
#define CTRL_REG7_XM        0x26
#define STATUS_REG_A        0x27
#define OUT_X_L_A           0x28
#define OUT_X_H_A           0x29
#define OUT_Y_L_A           0x2A
#define OUT_Y_H_A           0x2B
#define OUT_Z_L_A           0x2C
#define OUT_Z_H_A           0x2D
#define FIFO_CTRL_REG       0x2E
#define FIFO_SRC_REG        0x2F
#define INT_GEN_1_REG       0x30
#define INT_GEN_1_SRC       0x31
#define INT_GEN_1_THS       0x32
#define INT_GEN_1_DURATION  0x33
#define INT_GEN_2_REG       0x34
#define INT_GEN_2_SRC       0x35
#define INT_GEN_2_THS       0x36
#define INT_GEN_2_DURATION  0x37
#define CLICK_CFG           0x38
#define CLICK_SRC           0x39
#define CLICK_THS           0x3A
#define TIME_LIMIT          0x3B
#define TIME_LATENCY        0x3C
#define TIME_WINDOW         0x3D
#define ACT_THS             0x3E
#define ACT_DUR             0x3F



class LSM9DS0
{
public:
    // gyro_scale defines the possible full-scale ranges of the gyroscope:
    enum gyro_scale
    {
        G_SCALE_245DPS = 0x0,  // 00:  245  degrees per second
        G_SCALE_500DPS = 0x1,  // 01:  500  dps
        G_SCALE_2000DPS = 0x2,  // 10:  2000 dps
    };
    // accel_scale defines all possible FSR's of the accelerometer:
    enum accel_scale
    {
        A_SCALE_2G = 0x0, // 000:  2g
        A_SCALE_4G = 0x1, // 001:  4g
        A_SCALE_6G = 0x2, // 010:  6g
        A_SCALE_8G = 0x3, // 011:  8g
        A_SCALE_16G = 0x4, // 100:  16g
    };
    // gyro_odr defines all possible data rate/bandwidth combos of the gyro:
    enum gyro_odr
    {                           // ODR (Hz) --- Cutoff
        G_ODR_95_BW_125 = 0x0,  //    95         12.5
        G_ODR_95_BW_25 = 0x1,   //    95         25
        // 0x2 and 0x3 define the same data rate and bandwidth
        G_ODR_190_BW_125 = 0x4, //   190         12.5
        G_ODR_190_BW_25 = 0x5,  //   190         25
        G_ODR_190_BW_50 = 0x6,  //   190         50
        G_ODR_190_BW_70 = 0x7,  //   190         70
        G_ODR_380_BW_20 = 0x8,  //   380         20
        G_ODR_380_BW_25 = 0x9,  //   380         25
        G_ODR_380_BW_50 = 0xA,  //   380         50
        G_ODR_380_BW_100 = 0xB, //   380        100
        G_ODR_760_BW_30 = 0xC,  //   760         30
        G_ODR_760_BW_35 = 0xD,  //   760         35
        G_ODR_760_BW_50 = 0xE,  //   760         50
        G_ODR_760_BW_100 = 0xF, //   760        100
    };
    // accel_oder defines all possible output data rates of the accelerometer:
    enum accel_odr
    {
        A_POWER_DOWN = 0x00,     // Power-down mode (0x0)
        A_ODR_3125   = 0x01,     //    3.125 Hz (0x1)
        A_ODR_625    = 0x02,     //    6.25  Hz (0x2)
        A_ODR_125    = 0x03,     //   12.5   Hz (0x3)
        A_ODR_25     = 0x04,     //   25     Hz (0x4)
        A_ODR_50     = 0x05,     //   50     Hz (0x5)
        A_ODR_100    = 0x06,     //  100     Hz (0x6)
        A_ODR_200    = 0x07,     //  200     Hz (0x7)
        A_ODR_400    = 0x08,     //  400     Hz (0x8)
        A_ODR_800    = 0x09,     //  800     Hz (0x9)
        A_ODR_1600   = 0x0A,     // 1600     Hz (0xA)
    };

    // accel_abw defines all possible anti-aliasing filter rates of the accelerometer:
    enum accel_abw
    {
        A_ABW_773 = 0x0,      // 773 Hz (0x0)
        A_ABW_194 = 0x1,      // 194 Hz (0x1)
        A_ABW_362 = 0x2,      // 362 Hz (0x2)
        A_ABW_50  = 0x3,      //  50 Hz (0x3)
    };


    // gRes, aRes, and mRes store the current resolution for each sensor. 
    // Units of these values would be DPS (or g's or Gs's) per ADC tick.
    // This value is calculated as (sensor scale) / (2^15).
    float gRes, aRes;

    // We'll store the gyro, accel, and magnetometer readings in a series of
    // public class variables. Each sensor gets three variables -- one for each
    // axis. Call readGyro(), readAccel(), and readMag() first, before using
    // these variables!
    // These values are the RAW signed 16-bit readings from the sensors.
    int16_t gx, gy, gz; // x, y, and z axis readings of the gyroscope
    int16_t ax, ay, az; // x, y, and z axis readings of the accelerometer
    
    int16_t gyroOffset[3];
    int16_t accelOffset[3];
    void setGyroOffset(int16_t, int16_t, int16_t);
    void setAccelOffset(int16_t, int16_t, int16_t);

    // LSM9DS0 -- LSM9DS0 class constructor
    // The constructor will set up a handful of private variables, and set the
    // communication mode as well.
    // Input:
    //  - gAddr  = this is the cs pin of the gyro (CSG)
    //  - xmAddr = this is the cs pin of the accel/mag (CSXM)
    LSM9DS0(PinName mosi, PinName miso, PinName sclk, PinName gAddr, PinName xmAddr);

    // begin() -- Initialize the gyro, accelerometer, and magnetometer.
    // This will set up the scale and output rate of each sensor. It'll also
    // "turn on" every sensor and every axis of every sensor.
    // Input:
    //  - gScl = The scale of the gyroscope. This should be a gyro_scale value.
    //  - aScl = The scale of the accelerometer. Should be a accel_scale value.
    //  - gODR = Output data rate of the gyroscope. gyro_odr value.
    //  - aODR = Output data rate of the accelerometer. accel_odr value.
    // Output: The function will return an unsigned 16-bit value. The most-sig
    //      bytes of the output are the WHO_AM_I reading of the accel. The
    //      least significant two bytes are the WHO_AM_I reading of the gyro.
    // All parameters have a defaulted value, so you can call just "begin()".
    // Default values are FSR's of:  2000DPS, 8g, 8Gs; ODRs of 760 Hz for 
    // gyro, 800 Hz for accelerometer, 100 Hz for magnetometer.
    // Use the return value of this function to verify communication.
    void begin(gyro_scale gScl = G_SCALE_2000DPS, accel_scale aScl = A_SCALE_8G, gyro_odr gODR = G_ODR_760_BW_100, accel_odr aODR = A_ODR_800);

    // readGyro() -- Read the gyroscope output registers.
    // This function will read all six gyroscope output registers.
    // The readings are stored in the class' gx, gy, and gz variables. Read
    // those _after_ calling readGyro().
    void readGyro(float* gyr);
    void readGyroValue(float* gyr);

    // readAccel() -- Read the accelerometer output registers.
    // This function will read all six accelerometer output registers.
    // The readings are stored in the class' ax, ay, and az variables. Read
    // those _after_ calling readAccel().
    void readAccel(float* acc);
    void readAccelValue(float* acc);

    // setGyroScale() -- Set the full-scale range of the gyroscope.
    // This function can be called to set the scale of the gyroscope to 
    // 245, 500, or 200 degrees per second.
    // Input:
    //  - gScl = The desired gyroscope scale. Must be one of three possible
    //      values from the gyro_scale enum.
    void setGyroScale(gyro_scale gScl);

    // setAccelScale() -- Set the full-scale range of the accelerometer.
    // This function can be called to set the scale of the accelerometer to
    // 2, 4, 6, 8, or 16 g's.
    // Input:
    //  - aScl = The desired accelerometer scale. Must be one of five possible
    //      values from the accel_scale enum.
    void setAccelScale(accel_scale aScl);

    // setGyroODR() -- Set the output data rate and bandwidth of the gyroscope
    // Input:
    //  - gRate = The desired output rate and cutoff frequency of the gyro.
    //      Must be a value from the gyro_odr enum (check above, there're 14).
    void setGyroODR(gyro_odr gRate);

    // setAccelODR() -- Set the output data rate of the accelerometer
    // Input:
    //  - aRate = The desired output rate of the accel.
    //      Must be a value from the accel_odr enum (check above, there're 11).
    void setAccelODR(accel_odr aRate);

    // setAccelABW() -- Set the anti-aliasing filter rate of the accelerometer
    // Input:
    //  - abwRate = The desired anti-aliasing filter rate of the accel.
    //      Must be a value from the accel_abw enum (check above, there're 4).
    void setAccelABW(accel_abw abwRate);

private:
    SPI* spi_;
    DigitalOut* csG_;
    DigitalOut* csXM_;

    // initSPI() -- Initialize the SPI hardware.
    // This function will setup all SPI pins and related hardware.
    void initSPI();
    
    // gScale, aScale, and mScale store the current scale range for each 
    // sensor. Should be updated whenever that value changes.
    gyro_scale gScale;
    accel_scale aScale;

    // calcgRes() -- Calculate the resolution of the gyroscope.
    // This function will set the value of the gRes variable. gScale must
    // be set prior to calling this function.
    void calcgRes();

    // calcaRes() -- Calculate the resolution of the accelerometer.
    // This function will set the value of the aRes variable. aScale must
    // be set prior to calling this function.
    void calcaRes();
    
    // initGyro() -- Sets up the gyroscope to begin reading.
    // This function steps through all five gyroscope control registers.
    // Upon exit, the following parameters will be set:
    //  - CTRL_REG1_G = 0x0F: Normal operation mode, all axes enabled. 
    //      95 Hz ODR, 12.5 Hz cutoff frequency.
    //  - CTRL_REG2_G = 0x00: HPF set to normal mode, cutoff frequency
    //      set to 7.2 Hz (depends on ODR).
    //  - CTRL_REG3_G = 0x88: Interrupt enabled on INT_G (set to push-pull and
    //      active high). Data-ready output enabled on DRDY_G.
    //  - CTRL_REG4_G = 0x00: Continuous update mode. Data LSB stored in lower
    //      address. Scale set to 245 DPS. SPI mode set to 4-wire.
    //  - CTRL_REG5_G = 0x00: FIFO disabled. HPF disabled.
    void initGyro();

    // initAccel() -- Sets up the accelerometer to begin reading.
    // This function steps through all accelerometer related control registers.
    // Upon exit these registers will be set as:
    //  - CTRL_REG0_XM = 0x00: FIFO disabled. HPF bypassed. Normal mode.
    //  - CTRL_REG1_XM = 0x57: 100 Hz data rate. Continuous update.
    //      all axes enabled.
    //  - CTRL_REG2_XM = 0x00:  2g scale. 773 Hz anti-alias filter BW.
    //  - CTRL_REG3_XM = 0x04: Accel data ready signal on INT1_XM pin.
    void initAccel();

    // gReadByte() -- Reads a byte from a specified gyroscope register.
    // Input:
    //  - subAddress = Register to be read from.
    // Output:
    //  - An 8-bit value read from the requested address.
    uint8_t gReadByte(uint8_t subAddress);///////////////////////////////////////////

    // gWriteByte() -- Write a byte to a register in the gyroscope.
    // Input:
    //  - subAddress = Register to be written to.
    //  - data = data to be written to the register.
    void gWriteByte(uint8_t subAddress, uint8_t data);

    // xmReadByte() -- Read a byte from a register in the accel/mag sensor
    // Input:
    //  - subAddress = Register to be read from.
    // Output:
    //  - An 8-bit value read from the requested register.
    uint8_t xmReadByte(uint8_t subAddress);//////////////////////////////////////

    // xmWriteByte() -- Write a byte to a register in the accel/mag sensor.
    // Input:
    //  - subAddress = Register to be written to.
    //  - data = data to be written to the register.
    void xmWriteByte(uint8_t subAddress, uint8_t data);
};

#endif // _LSM9DS0_H //