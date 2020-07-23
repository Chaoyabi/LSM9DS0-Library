#include "LSM9DS0.h"
#include "mbed.h"

//SPI spi(D4,D5,D3);
//**************************************************************************
//  LSM9DS0 functions.
//
//    where gAddr and xmAddr are addresses for chip select pin
//    number for SPI_MODE
//
//  For SPI, construct LSM9DS0 myIMU(D4, D5, D3, D6, D9);
//=================================

unsigned char temp[6];

LSM9DS0::LSM9DS0(PinName mosi, PinName miso, PinName sclk, PinName gAddr, PinName xmAddr)
{
    spi_ = new SPI(mosi, miso, sclk);
    csG_ = new DigitalOut(gAddr);
    csXM_ = new DigitalOut(xmAddr);
}

void LSM9DS0::begin(gyro_scale gScl, accel_scale aScl, gyro_odr gODR, accel_odr aODR)
{
    // Store the given scales in class variables. These scale variables
    // are used throughout to calculate the actual g's, DPS,and Gs's.
    gScale = gScl;
    aScale = aScl;

    // Once we have the scale values, we can calculate the resolution
    // of each sensor. That's what these functions are for. One for each sensor
    calcgRes(); // Calculate DPS / ADC tick, stored in gRes variable
    calcaRes(); // Calculate g / ADC tick, stored in aRes variable

    gReadByte(WHO_AM_I_G);      // Read the gyro WHO_AM_I
    xmReadByte(WHO_AM_I_XM);   // Read the accel/mag WHO_AM_I

    // Gyro initialization stuff:
    initGyro(); // This will "turn on" the gyro. Setting up interrupts, etc.
    setGyroODR(gODR); // Set the gyro output data rate and bandwidth.
    setGyroScale(gScale); // Set the gyro range

    // Accelerometer initialization stuff:
    initAccel(); // "Turn on" all axes of the accel. Set up interrupts, etc.
    setAccelODR(aODR); // Set the accel data rate.
    setAccelScale(aScale); // Set the accel range.

    setGyroOffset(0, 0, 0);
    setAccelOffset(0, 0, 0);
}

void LSM9DS0::initGyro()
{
    /* CTRL_REG1_G sets output data rate, bandwidth, power-down and enables
    Bits[7:0]: DR1 DR0 BW1 BW0 PD Zen Xen Yen
    DR[1:0] - Output data rate selection
        00=95Hz, 01=190Hz, 10=380Hz, 11=760Hz
    BW[1:0] - Bandwidth selection (sets cutoff frequency)
         Value depends on ODR. See datasheet table 21.
    PD - Power down enable (0=power down mode, 1=normal or sleep mode)
    Zen, Xen, Yen - Axis enable (o=disabled, 1=enabled) */
    gWriteByte(CTRL_REG1_G, 0xFF); // Normal mode, enable all axes

    /* CTRL_REG2_G sets up the HPF
    Bits[7:0]: 0 0 HPM1 HPM0 HPCF3 HPCF2 HPCF1 HPCF0
    HPM[1:0] - High pass filter mode selection
        00=normal (reset reading HP_RESET_FILTER, 01=ref signal for filtering,
        10=normal, 11=autoreset on interrupt
    HPCF[3:0] - High pass filter cutoff frequency
        Value depends on data rate. See datasheet table 26.
    */
    gWriteByte(CTRL_REG2_G, 0x09); // Normal mode, high cutoff frequency

    /* CTRL_REG3_G sets up interrupt and DRDY_G pins
    Bits[7:0]: I1_IINT1 I1_BOOT H_LACTIVE PP_OD I2_DRDY I2_WTM I2_ORUN I2_EMPTY
    I1_INT1 - Interrupt enable on INT_G pin (0=disable, 1=enable)
    I1_BOOT - Boot status available on INT_G (0=disable, 1=enable)
    H_LACTIVE - Interrupt active configuration on INT_G (0:high, 1:low)
    PP_OD - Push-pull/open-drain (0=push-pull, 1=open-drain)
    I2_DRDY - Data ready on DRDY_G (0=disable, 1=enable)
    I2_WTM - FIFO watermark interrupt on DRDY_G (0=disable 1=enable)
    I2_ORUN - FIFO overrun interrupt on DRDY_G (0=disable 1=enable)
    I2_EMPTY - FIFO empty interrupt on DRDY_G (0=disable 1=enable) */
    // Int1 enabled (pp, active low), data read on DRDY_G:
    gWriteByte(CTRL_REG3_G, 0x00);

    /* CTRL_REG4_G sets the scale, update mode
    Bits[7:0] - BDU BLE FS1 FS0 - ST1 ST0 SIM
    BDU - Block data update (0=continuous, 1=output not updated until read
    BLE - Big/little endian (0=data LSB @ lower address, 1=LSB @ higher add)
    FS[1:0] - Full-scale selection
        00=245dps, 01=500dps, 10=2000dps, 11=2000dps
    ST[1:0] - Self-test enable
        00=disabled, 01=st 0 (x+, y-, z-), 10=undefined, 11=st 1 (x-, y+, z+)
    SIM - SPI serial interface mode select
        0=4 wire, 1=3 wire */
    gWriteByte(CTRL_REG4_G, 0x20); // Set scale to 2000 dps

    /* CTRL_REG5_G sets up the FIFO, HPF, and INT1
    Bits[7:0] - BOOT FIFO_EN - HPen INT1_Sel1 INT1_Sel0 Out_Sel1 Out_Sel0
    BOOT - Reboot memory content (0=normal, 1=reboot)
    FIFO_EN - FIFO enable (0=disable, 1=enable)
    HPen - HPF enable (0=disable, 1=enable)
    INT1_Sel[1:0] - Int 1 selection configuration
    Out_Sel[1:0] - Out selection configuration */
    gWriteByte(CTRL_REG5_G, 0x00);
}

void LSM9DS0::initAccel()
{
    /* CTRL_REG0_XM (0x1F) (Default value: 0x00)
    Bits (7-0): BOOT FIFO_EN WTM_EN 0 0 HP_CLICK HPIS1 HPIS2s
    BOOT - Reboot memory content (0: normal, 1: reboot)
    FIFO_EN - FIFO enable (0: disable, 1: enable)
    WTM_EN - FIFO watermark enable (0: disable, 1: enable)
    HP_CLICK - HPF enabled for click (0: filter bypassed, 1: enabled)
    HPIS1 - HPF enabled for interrupt generator 1 (0: bypassed, 1: enabled)
    HPIS2 - HPF enabled for interrupt generator 2 (0: bypassed, 1 enabled)   */
    xmWriteByte(CTRL_REG0_XM, 0x00);

    /* CTRL_REG1_XM (0x20) (Default value: 0x07)
    Bits (7-0): AODR3 AODR2 AODR1 AODR0 BDU AZEN AYEN AXEN
    AODR[3:0] - select the acceleration data rate:
        0000=power down, 0001=3.125Hz, 0010=6.25Hz, 0011=12.5Hz,
        0100=25Hz, 0101=50Hz, 0110=100Hz, 0111=200Hz, 1000=400Hz,
        1001=800Hz, 1010=1600Hz, (remaining combinations undefined).
    BDU - block data update for accel AND mag
        0: Continuous update
        1: Output registers aren't updated until MSB and LSB have been read.
    AZEN, AYEN, and AXEN - Acceleration x/y/z-axis enabled.
        0: Axis disabled, 1: Axis enabled                                    */
    xmWriteByte(CTRL_REG1_XM, 0x97); // 100Hz data rate, x/y/z all enabled

    //Serial.println(xmReadByte(CTRL_REG1_XM));
    /* CTRL_REG2_XM (0x21) (Default value: 0x00)
    Bits (7-0): ABW1 ABW0 AFS2 AFS1 AFS0 AST1 AST0 SIM
    ABW[1:0] - Accelerometer anti-alias filter bandwidth
        00=773Hz, 01=194Hz, 10=362Hz, 11=50Hz
    AFS[2:0] - Accel full-scale selection
        000=+/-2g, 001=+/-4g, 010=+/-6g, 011=+/-8g, 100=+/-16g
    AST[1:0] - Accel self-test enable
        00=normal (no self-test), 01=positive st, 10=negative st, 11=not allowed
    SIM - SPI mode selection
        0=4-wire, 1=3-wire                                                   */
    xmWriteByte(CTRL_REG2_XM, 0xD8); // Set scale to 2g

    /* CTRL_REG3_XM is used to set interrupt generators on INT1_XM
    Bits (7-0): P1_BOOT P1_TAP P1_INT1 P1_INT2 P1_INTM P1_DRDYA P1_DRDYM P1_EMPTY
    */
    // Accelerometer data ready on INT1_XM (0x04)
    xmWriteByte(CTRL_REG3_XM, 0x00);
}

//**********************
//  Gyro section
//**********************
void LSM9DS0::readGyro(float* gyr)
{
    csG_->write(0);
    spi_->write(0xC0 | OUT_X_L_G);
    for (int i = 0; i < 6; i++)
    {
        temp[i] = spi_->write(0x00); // Read into destination array
    }
    csG_->write(1);

    gx = (temp[1] << 8) | temp[0]; // Store x-axis values into gx
    gy = (temp[3] << 8) | temp[2]; // Store y-axis values into gy
    gz = (temp[5] << 8) | temp[4]; // Store z-axis values into gz

    gyr[0] = float(gx - gyroOffset[0]);
    gyr[1] = float(gy - gyroOffset[1]);
    gyr[2] = float(gz - gyroOffset[2]);
}

void LSM9DS0::readGyroValue(float* gyr)
{
    csG_->write(0);
    spi_->write(0xC0 | OUT_X_L_G);
    for (int i = 0; i < 6; i++)
    {
        temp[i] = spi_->write(0x00); // Read into destination array
    }
    csG_->write(1);

    gx = (temp[1] << 8) | temp[0]; // Store x-axis values into gx
    gy = (temp[3] << 8) | temp[2]; // Store y-axis values into gy
    gz = (temp[5] << 8) | temp[4]; // Store z-axis values into gz

    gyr[0] = float(gx - gyroOffset[0]) * gRes;
    gyr[1] = float(gy - gyroOffset[1]) * gRes;
    gyr[2] = float(gz - gyroOffset[2]) * gRes;
}

void LSM9DS0::setGyroOffset(int16_t _gx, int16_t _gy, int16_t _gz)
{
    gyroOffset[0] = _gx;
    gyroOffset[1] = _gy;
    gyroOffset[2] = _gz;
}

//**********************
//  Accel section
//**********************
void LSM9DS0::readAccel(float* acc)
{
    csXM_->write(0);
    spi_->write(0xC0 | OUT_X_L_A);
    for (int i = 0; i < 6; i++)
    {
        temp[i] = spi_->write(0x00);
    }
    csXM_->write(1);

    ax = (temp[1] << 8) | temp[0];
    ay = (temp[3] << 8) | temp[2];
    az = (temp[5] << 8) | temp[4];

    acc[0] = float(ax - accelOffset[0]); // Store x-axis values into ax
    acc[1] = float(ay - accelOffset[1]); // Store y-axis values into ay
    acc[2] = float(az - accelOffset[2]); // Store z-axis values into az
}

void LSM9DS0::readAccelValue(float* acc)
{
    csXM_->write(0);
    spi_->write(0xC0 | OUT_X_L_A);
    for (int i = 0; i < 6; i++)
    {
        temp[i] = spi_->write(0x00);
    }
    csXM_->write(1);

    ax = (temp[1] << 8) | temp[0];
    ay = (temp[3] << 8) | temp[2];
    az = (temp[5] << 8) | temp[4];

    acc[0] = float(ax - accelOffset[0]) * aRes; // Store x-axis values into ax
    acc[1] = float(ay - accelOffset[1]) * aRes; // Store y-axis values into ay
    acc[2] = float(az - accelOffset[2]) * aRes; // Store z-axis values into az
}

void LSM9DS0::setAccelOffset(int16_t _ax, int16_t _ay, int16_t _az)
{
    accelOffset[0] = _ax;
    accelOffset[1] = _ay;
    accelOffset[2] = _az;
}

void LSM9DS0::setGyroScale(gyro_scale gScl)
{
    // We need to preserve the other bytes in CTRL_REG4_G. So, first read it:
    uint8_t temp = gReadByte(CTRL_REG4_G);
    // Then mask out the gyro scale bits:
    temp &= 0xFF ^ (0x3 << 4);
    // Then shift in our new scale bits:
    temp |= gScl << 4;
    // And write the new register value back into CTRL_REG4_G:
    gWriteByte(CTRL_REG4_G, temp);

    // We've updated the sensor, but we also need to update our class variables
    // First update gScale:
    gScale = gScl;
    // Then calculate a new gRes, which relies on gScale being set correctly:
    calcgRes();
}

void LSM9DS0::setAccelScale(accel_scale aScl)
{
    // We need to preserve the other bytes in CTRL_REG2_XM. So, first read it:
    uint8_t temp = xmReadByte(CTRL_REG2_XM);
    // Then mask out the accel scale bits:
    temp &= 0xFF ^ (0x7 << 3);
    // Then shift in our new scale bits:
    temp |= aScl << 3;
    // And write the new register value back into CTRL_REG2_XM:
    xmWriteByte(CTRL_REG2_XM, temp);

    // We've updated the sensor, but we also need to update our class variables
    // First update aScale:
    aScale = aScl;
    // Then calculate a new aRes, which relies on aScale being set correctly:
    calcaRes();
}

void LSM9DS0::setGyroODR(gyro_odr gRate)
{
    // We need to preserve the other bytes in CTRL_REG1_G. So, first read it:
    uint8_t temp = gReadByte(CTRL_REG1_G);
    // Then mask out the gyro ODR bits:
    temp &= 0xFF ^ (0xF << 4);
    // Then shift in our new ODR bits:
    temp |= (gRate << 4);
    // And write the new register value back into CTRL_REG1_G:
    gWriteByte(CTRL_REG1_G, temp);
}
void LSM9DS0::setAccelODR(accel_odr aRate)
{
    csXM_->write(0);
    spi_->write(0x80 | CTRL_REG1_XM);
    temp[0] = spi_->write(0x00);
    csXM_->write(1);

    // Then mask out the accel ABW bits:
    temp[0] &= 0xFF ^ (0xF << 4);
    // Then shift in our new ODR bits:
    temp[0] |= (aRate << 4);
    // And write the new register value back into CTRL_REG2_XM:
    csXM_->write(0);

    spi_->write(CTRL_REG1_XM & 0x3F); // Send Address
    spi_->write(temp[0]); // Send data

    csXM_->write(1);
}
void LSM9DS0::setAccelABW(accel_abw abwRate)
{
    csXM_->write(0);
    spi_->write(0x80 | CTRL_REG2_XM);
    temp[0] = spi_->write(0x00);
    csXM_->write(1);

    // Then mask out the accel ABW bits:
    temp[0] &= 0xFF ^ (0x3 << 6);
    // Then shift in our new ODR bits:
    temp[0] |= (abwRate << 6);
    // And write the new register value back into CTRL_REG2_XM:
    csXM_->write(0);

    spi_->write(CTRL_REG2_XM & 0x3F); // Send Address
    spi_->write(temp[0]); // Send data

    csXM_->write(1);
}

void LSM9DS0::calcgRes()
{
    // Possible gyro scales (and their register bit settings) are:
    // 245 DPS (00), 500 DPS (01), 2000 DPS (10). Here's a bit of an algorithm
    // to calculate DPS/(ADC tick) based on that 2-bit value:
    switch (gScale)
    {
    case G_SCALE_245DPS:
        gRes = 245.0f / 32768.0f;
        break;
    case G_SCALE_500DPS:
        gRes = 500.0f / 32768.0f;
        break;
    case G_SCALE_2000DPS:
        gRes = 2000.0f / 32768.0f;
        break;
    }
}

void LSM9DS0::calcaRes()
{
    // Possible accelerometer scales (and their register bit settings) are:
    // 2 g (000), 4g (001), 6g (010), 8g (011), 16g (100). Here's a bit of an 
    // algorithm to calculate g/(ADC tick) based on that 3-bit value:
    switch (aScale)
    {
    case A_SCALE_2G:
        aRes = 2.0f / 32768.0f;
        break;
    case A_SCALE_4G:
        aRes = 4.0f / 32768.0f;
        break;
    case A_SCALE_6G:
        aRes = 6.0f / 32768.0f;
        break;
    case A_SCALE_8G:
        aRes = 8.0f / 32768.0f;
        break;
    case A_SCALE_16G:
        aRes = 16.0f / 32768.0f;
        break;
    }
}

void LSM9DS0::gWriteByte(uint8_t subAddress, uint8_t data)
{
    csG_->write(0);

    spi_->write(subAddress & 0x3F); // Send Address
    spi_->write(data); // Send data

    csG_->write(1);
}

void LSM9DS0::xmWriteByte(uint8_t subAddress, uint8_t data)
{
    csXM_->write(0);

    spi_->write(subAddress & 0x3F); // Send Address
    spi_->write(data); // Send data

    csXM_->write(1);
}

uint8_t LSM9DS0::gReadByte(uint8_t subAddress)
{
    csG_->write(0);
    spi_->write(0x80 | subAddress);
    temp[0] = spi_->write(0x00);
    csG_->write(1);

    return temp[0];
}

uint8_t LSM9DS0::xmReadByte(uint8_t subAddress)
{
    csXM_->write(0);
    spi_->write(0x80 | subAddress);
    temp[0] = spi_->write(0x00);
    csXM_->write(1);

    return temp[0];
}

void LSM9DS0::initSPI()
{
    csG_->write(1);
    csXM_->write(1);

    // Maximum SPI frequency is 10MHz:
    spi_->format(8, 3);
    spi_->frequency(1000000);
}

