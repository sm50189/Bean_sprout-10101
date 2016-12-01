#ifndef SELFMPU_H
#define SELFMPU_H

#include "mbed.h"
#include "math.h"

#define AK8963_ADDRESS   0x0C<<1
#define WHO_AM_I_AK8963  0x00 // should return 0x48
#define INFO             0x01
#define AK8963_ST1       0x02  // data ready status bit 0
#define AK8963_XOUT_L    0x03  // data
#define AK8963_XOUT_H    0x04
#define AK8963_YOUT_L    0x05
#define AK8963_YOUT_H    0x06
#define AK8963_ZOUT_L    0x07
#define AK8963_ZOUT_H    0x08
#define AK8963_ST2       0x09  // Data overflow bit 3 and data read error status bit 2
#define AK8963_CNTL      0x0A  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define AK8963_ASTC      0x0C  // Self test control
#define AK8963_I2CDIS    0x0F  // I2C disable
#define AK8963_ASAX      0x10  // Fuse ROM x-axis sensitivity adjustment value
#define AK8963_ASAY      0x11  // Fuse ROM y-axis sensitivity adjustment value
#define AK8963_ASAZ      0x12  // Fuse ROM z-axis sensitivity adjustment value

#define SELF_TEST_X_GYRO 0x00
#define SELF_TEST_Y_GYRO 0x01
#define SELF_TEST_Z_GYRO 0x02

/*#define X_FINE_GAIN      0x03 // [7:0] fine gain
#define Y_FINE_GAIN      0x04
#define Z_FINE_GAIN      0x05
#define XA_OFFSET_H      0x06 // User-defined trim values for accelerometer
#define XA_OFFSET_L_TC   0x07
#define YA_OFFSET_H      0x08
#define YA_OFFSET_L_TC   0x09
#define ZA_OFFSET_H      0x0A
#define ZA_OFFSET_L_TC   0x0B */

#define SELF_TEST_X_ACCEL 0x0D
#define SELF_TEST_Y_ACCEL 0x0E
#define SELF_TEST_Z_ACCEL 0x0F

#define SELF_TEST_A      0x10

#define XG_OFFSET_H      0x13  // User-defined trim values for gyroscope
#define XG_OFFSET_L      0x14
#define YG_OFFSET_H      0x15
#define YG_OFFSET_L      0x16
#define ZG_OFFSET_H      0x17
#define ZG_OFFSET_L      0x18
#define SMPLRT_DIV       0x19
#define CONFIG           0x1A
#define GYRO_CONFIG      0x1B
#define ACCEL_CONFIG     0x1C
#define ACCEL_CONFIG2    0x1D
#define LP_ACCEL_ODR     0x1E
#define WOM_THR          0x1F

#define MOT_DUR          0x20  // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
#define ZMOT_THR         0x21  // Zero-motion detection threshold bits [7:0]
#define ZRMOT_DUR        0x22  // Duration counter threshold for zero motion interrupt generation, 16 Hz rate, LSB = 64 ms

#define FIFO_EN          0x23
#define I2C_MST_CTRL     0x24
#define I2C_SLV0_ADDR    0x25
#define I2C_SLV0_REG     0x26
#define I2C_SLV0_CTRL    0x27
#define I2C_SLV1_ADDR    0x28
#define I2C_SLV1_REG     0x29
#define I2C_SLV1_CTRL    0x2A
#define I2C_SLV2_ADDR    0x2B
#define I2C_SLV2_REG     0x2C
#define I2C_SLV2_CTRL    0x2D
#define I2C_SLV3_ADDR    0x2E
#define I2C_SLV3_REG     0x2F
#define I2C_SLV3_CTRL    0x30
#define I2C_SLV4_ADDR    0x31
#define I2C_SLV4_REG     0x32
#define I2C_SLV4_DO      0x33
#define I2C_SLV4_CTRL    0x34
#define I2C_SLV4_DI      0x35
#define I2C_MST_STATUS   0x36
#define INT_PIN_CFG      0x37
#define INT_ENABLE       0x38
#define DMP_INT_STATUS   0x39  // Check DMP interrupt
#define INT_STATUS       0x3A
#define ACCEL_XOUT_H     0x3B
#define ACCEL_XOUT_L     0x3C
#define ACCEL_YOUT_H     0x3D
#define ACCEL_YOUT_L     0x3E
#define ACCEL_ZOUT_H     0x3F
#define ACCEL_ZOUT_L     0x40
#define TEMP_OUT_H       0x41
#define TEMP_OUT_L       0x42
#define GYRO_XOUT_H      0x43
#define GYRO_XOUT_L      0x44
#define GYRO_YOUT_H      0x45
#define GYRO_YOUT_L      0x46
#define GYRO_ZOUT_H      0x47
#define GYRO_ZOUT_L      0x48
#define EXT_SENS_DATA_00 0x49
#define EXT_SENS_DATA_01 0x4A
#define EXT_SENS_DATA_02 0x4B
#define EXT_SENS_DATA_03 0x4C
#define EXT_SENS_DATA_04 0x4D
#define EXT_SENS_DATA_05 0x4E
#define EXT_SENS_DATA_06 0x4F
#define EXT_SENS_DATA_07 0x50
#define EXT_SENS_DATA_08 0x51
#define EXT_SENS_DATA_09 0x52
#define EXT_SENS_DATA_10 0x53
#define EXT_SENS_DATA_11 0x54
#define EXT_SENS_DATA_12 0x55
#define EXT_SENS_DATA_13 0x56
#define EXT_SENS_DATA_14 0x57
#define EXT_SENS_DATA_15 0x58
#define EXT_SENS_DATA_16 0x59
#define EXT_SENS_DATA_17 0x5A
#define EXT_SENS_DATA_18 0x5B
#define EXT_SENS_DATA_19 0x5C
#define EXT_SENS_DATA_20 0x5D
#define EXT_SENS_DATA_21 0x5E
#define EXT_SENS_DATA_22 0x5F
#define EXT_SENS_DATA_23 0x60
#define MOT_DETECT_STATUS 0x61
#define I2C_SLV0_DO      0x63
#define I2C_SLV1_DO      0x64
#define I2C_SLV2_DO      0x65
#define I2C_SLV3_DO      0x66
#define I2C_MST_DELAY_CTRL 0x67
#define SIGNAL_PATH_RESET  0x68
#define MOT_DETECT_CTRL  0x69
#define USER_CTRL        0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define PWR_MGMT_1       0x6B // Device defaults to the SLEEP mode
#define PWR_MGMT_2       0x6C
#define DMP_BANK         0x6D  // Activates a specific bank in the DMP
#define DMP_RW_PNT       0x6E  // Set read/write pointer to a specific start address in specified DMP bank
#define DMP_REG          0x6F  // Register in DMP from which to read or to which to write
#define DMP_REG_1        0x70
#define DMP_REG_2        0x71
#define FIFO_COUNTH      0x72
#define FIFO_COUNTL      0x73
#define FIFO_R_W         0x74
#define WHO_AM_I_MPU9250 0x75 // Should return 0x71
#define XA_OFFSET_H      0x77
#define XA_OFFSET_L      0x78
#define YA_OFFSET_H      0x7A
#define YA_OFFSET_L      0x7B
#define ZA_OFFSET_H      0x7D
#define ZA_OFFSET_L      0x7E

uint8_t MPU_ADDRESS = 0x68<<1;
// #define MPU_ADDRESS 0x68<<1
// #define MPU_ADDRESS 0x69<<1

#define    GYRO_FULL_SCALE_250_DPS    0x00
#define    GYRO_FULL_SCALE_500_DPS    0x08
#define    GYRO_FULL_SCALE_1000_DPS   0x10
#define    GYRO_FULL_SCALE_2000_DPS   0x18

#define    ACC_FULL_SCALE_2_G        0x00
#define    ACC_FULL_SCALE_4_G        0x08
#define    ACC_FULL_SCALE_8_G        0x10
#define    ACC_FULL_SCALE_16_G       0x18

uint8_t Gscale = GYRO_FULL_SCALE_250_DPS;
uint8_t Ascale = ACC_FULL_SCALE_2_G;

I2C i2c(I2C_SDA,I2C_SCL);
// I2C i2c(A4,A5);








class MPU9555{
    protected:

    public:

      void writeByte(uint8_t address, uint8_t subdata, uint8_t data) {
        char data2send[2];
        data2send[0] = subdata;
        data2send[1] = data;
        i2c.write(address,data2send,2,0);
      }

      uint8_t readByte(uint8_t address, uint8_t subdata) {
        char data2send[1];
        data2send[0] = subdata;
        char data[1];
        i2c.write(address,data2send,1,1);
        i2c.read(address,data,1,0);
        return data[0];
      }

      void readManyByte(uint8_t address, uint8_t subdata, uint8_t NByte, uint8_t* output){
        char data[14];
        char data2send[1];
        data2send[0] = subdata;
        i2c.write(address,data2send,1,1);
        i2c.read(address,data,NByte,0);
        for (int i = 0; i < NByte; i++) {
          output[i] = data[i];
        }
      }


      void setGrange(){
        writeByte(MPU_ADDRESS,GYRO_CONFIG,Gscale<<3);
      }

      void setArange() {
        writeByte(MPU_ADDRESS,ACCEL_CONFIG,Ascale<<3);
      }

      float getGres(){
        switch (Gscale) {
          case GYRO_FULL_SCALE_250_DPS:
            return (float)500/(float)65535;
          case GYRO_FULL_SCALE_500_DPS:
            return (float)1000/(float)65535;
          case GYRO_FULL_SCALE_1000_DPS:
            return (float)2000/(float)65535;
          case GYRO_FULL_SCALE_2000_DPS:
            return (float)4000/(float)65535;
        }
      }

      float getAres(){
        switch (Ascale) {
          case ACC_FULL_SCALE_2_G:
            return (float)4/(float)65535;
          case ACC_FULL_SCALE_4_G:
            return (float)8/(float)65535;
          case ACC_FULL_SCALE_8_G:
            return (float)16/(float)65535;
          case ACC_FULL_SCALE_16_G:
            return (float)32/(float)65535;
        }
      }

      void readATandG(int16_t* output){
        uint8_t data[14];
        readManyByte(MPU_ADDRESS,ACCEL_XOUT_H,14,&data[0]);
        output[0] = (((int16_t)data[0]<<8)|data[1]);
        output[1] = (((int16_t)data[2]<<8)|data[3]);
        output[2] = (((int16_t)data[4]<<8)|data[5]);
        output[3] = (((int16_t)data[6]<<8)|data[7]);
        output[4] = (((int16_t)data[8]<<8)|data[9]);
        output[5] = (((int16_t)data[10]<<8)|data[11]);
        output[6] = (((int16_t)data[12]<<8)|data[13]);
      }

      void resetMPU(){
        writeByte(MPU_ADDRESS,PWR_MGMT_1,0x80);
        wait(0.1);
      }

      void initial_MPU(){
        writeByte(MPU_ADDRESS,PWR_MGMT_1,0x00);
        wait(0.1);

        writeByte(MPU_ADDRESS,PWR_MGMT_1,0x01);

        writeByte(MPU_ADDRESS,CONFIG,0x03);

        writeByte(MPU_ADDRESS,SMPLRT_DIV,0x04);

        uint8_t c = readByte(MPU_ADDRESS,GYRO_CONFIG);
        writeByte(MPU_ADDRESS,GYRO_CONFIG, c& ~0xE0);
        writeByte(MPU_ADDRESS,GYRO_CONFIG, c& ~0x18);
        writeByte(MPU_ADDRESS,GYRO_CONFIG, c|Gscale<<3);

        c = readByte(MPU_ADDRESS,ACCEL_CONFIG);
        writeByte(MPU_ADDRESS,ACCEL_CONFIG, c&~0xE0);
        writeByte(MPU_ADDRESS,ACCEL_CONFIG, c&~0x18);
        writeByte(MPU_ADDRESS,ACCEL_CONFIG, c|Ascale<<3);

        c = readByte(MPU_ADDRESS,ACCEL_CONFIG2);
        writeByte(MPU_ADDRESS,ACCEL_CONFIG2, c& ~0x0F);
        writeByte(MPU_ADDRESS,ACCEL_CONFIG2, c| 0x03);

        writeByte(MPU_ADDRESS,INT_PIN_CFG, 0x22);
        writeByte(MPU_ADDRESS,INT_ENABLE, 0x01);

      }

      void calibrateMPU(float* output1, float* output2){
        uint8_t data[12];
        uint16_t i,packetCount,fifoCount;
        int32_t Gbias[3]={0,0,0} , Abias[3] = {0,0,0};

        writeByte(MPU_ADDRESS,CONFIG,0x80);
        wait(0.1);

        writeByte(MPU_ADDRESS,PWR_MGMT_1,0x01);
        writeByte(MPU_ADDRESS,PWR_MGMT_1,0x00);
        wait(0.2);

        writeByte(MPU_ADDRESS, INT_ENABLE, 0x00);   // Disable all interrupts
        writeByte(MPU_ADDRESS, FIFO_EN, 0x00);      // Disable FIFO
        writeByte(MPU_ADDRESS, PWR_MGMT_1, 0x00);   // Turn on internal clock source
        writeByte(MPU_ADDRESS, I2C_MST_CTRL, 0x00); // Disable I2C master
        writeByte(MPU_ADDRESS, USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
        writeByte(MPU_ADDRESS, USER_CTRL, 0x0C);    // Reset FIFO and DMP
        wait(0.015);

        writeByte(MPU_ADDRESS,CONFIG,0x01);
        writeByte(MPU_ADDRESS,SMPLRT_DIV,0x00);
        writeByte(MPU_ADDRESS,GYRO_CONFIG,0x00);
        writeByte(MPU_ADDRESS,ACCEL_CONFIG,0x00);

        uint16_t Gsensitive = 65535/500;
        uint16_t Asensitive = 65535/4;

        writeByte(MPU_ADDRESS, USER_CTRL, 0x40);   // Enable FIFO
        writeByte(MPU_ADDRESS, FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO (max size 512 bytes in MPU-9250)
        wait(0.04); // accumulate 40 samples in 80 milliseconds = 480 bytes

        writeByte(MPU_ADDRESS, FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
        readManyByte(MPU_ADDRESS, FIFO_COUNTH, 2, data); // read FIFO sample count
        fifoCount = ((uint16_t)data[0] << 8) | data[1];
        packetCount = fifoCount/12;// How many sets of full gyro and accelerometer data for averaging

        for ( i = 0; i < packetCount; i++) {
          readManyByte(MPU_ADDRESS,FIFO_R_W,12,&data[0]);

          int16_t Atemp[3]={0,0,0},Gtemp[3]={0,0,0};

          Atemp[0] = (int16_t)(((int16_t)data[0]<<8|data[1]));
          Atemp[1] = (int16_t)(((int16_t)data[2]<<8|data[3]));
          Atemp[2] = (int16_t)(((int16_t)data[4]<<8|data[5]));
          Gtemp[0] = (int16_t)(((int16_t)data[6]<<8|data[7]));
          Gtemp[1] = (int16_t)(((int16_t)data[8]<<8|data[9]));
          Gtemp[2] = (int16_t)(((int16_t)data[10]<<8|data[11]));

          Abias[0] += (int32_t)Atemp[0];
          Abias[1] += (int32_t)Atemp[1];
          Abias[2] += (int32_t)Atemp[2];
          Gbias[0] += (int32_t)Gtemp[0];
          Gbias[1] += (int32_t)Gtemp[1];
          Gbias[2] += (int32_t)Gtemp[2];
        }
        Abias[0] /= (int32_t)Asensitive;
        Abias[1] /= (int32_t)Asensitive;
        Abias[2] /= (int32_t)Asensitive;
        Gbias[0] /= (int32_t)Gsensitive;
        Gbias[1] /= (int32_t)Gsensitive;
        Gbias[2] /= (int32_t)Gsensitive;

        if(Abias[2]>=0L){
          Abias[2] -= (int32_t)Asensitive;
        }
        else{
          Abias[2] += (int32_t)Asensitive;
        }

        output1[0] = (float)Gbias[0]/(float)Gsensitive;
        output1[1] = (float)Gbias[1]/(float)Gsensitive;
        output1[2] = (float)Gbias[2]/(float)Gsensitive;
        output2[0] = (float)Abias[0]/(float)Asensitive;
        output2[1] = (float)Abias[1]/(float)Asensitive;
        output2[2] = (float)Abias[2]/(float)Asensitive;


      }

};
#endif
