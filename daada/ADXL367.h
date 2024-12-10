/*
 Arduino Library for Analog Devices ADXL367 - Micropower 3-axis accelerometer
 go to http://www.analog.com/ADXL367 for datasheet
 
 Created by Pablo del Corro <pablo.delcorro@analog.com>
 */  

#include "Arduino.h"

#ifndef ADXL367_h
#define ADXL367_h



/******************************************************************************/
/********************************* ADXL367 ************************************/
/******************************************************************************/

/* ADXL367 communication commands */
#define ADXL367_WRITE_REG               0x0A
#define ADXL367_READ_REG                0x0B
#define ADXL367_READ_FIFO              0x0D

/* Registers */
#define ADXL367_REG_DEVID_AD            0x00
#define ADXL367_REG_DEVID_MST           0x01
#define ADXL367_REG_PARTID              0x02
#define ADXL367_REG_REVID               0x03
#define ADXL367_REG_SERIAL_NUMBER_R3  0x04
#define ADXL367_REG_SERIAL_NUMBER_R2  0x05
#define ADXL367_REG_SERIAL_NUMBER_R1  0x06
#define ADXL367_REG_SERIAL_NUMBER_R0  0x07
#define ADXL367_REG_XDATA               0x08
#define ADXL367_REG_YDATA               0x09
#define ADXL367_REG_ZDATA               0x0A
#define ADXL367_REG_STATUS              0x0B
#define ADXL367_REG_FIFO_L              0x0C
#define ADXL367_REG_FIFO_H              0x0D
#define ADXL367_REG_XDATA_H             0x0E
#define ADXL367_REG_XDATA_L             0x0F
#define ADXL367_REG_YDATA_H             0x10
#define ADXL367_REG_YDATA_L             0x11
#define ADXL367_REG_ZDATA_H             0x12
#define ADXL367_REG_ZDATA_L             0x13
#define ADXL367_REG_TEMP_H              0x14
#define ADXL367_REG_TEMP_L              0x15
#define ADXL367_REG_EX_ADC_H      0x16
#define ADXL367_REG_EX_ADC_L      0x17
#define ADXL367_REG_I2C_FIFO_DATA   0x18
#define ADXL367_REG_SOFT_RESET          0x1F
#define ADXL367_REG_THRESH_ACT_H        0x20
#define ADXL367_REG_THRESH_ACT_L        0x21
#define ADXL367_REG_TIME_ACT            0x22
#define ADXL367_REG_THRESH_INACT_H      0x23
#define ADXL367_REG_THRESH_INACT_L      0x24
#define ADXL367_REG_TIME_INACT_H        0x25
#define ADXL367_REG_TIME_INACT_L        0x26
#define ADXL367_REG_ACT_INACT_CTL       0x27
#define ADXL367_REG_FIFO_CTL            0x28
#define ADXL367_REG_FIFO_SAMPLES        0x29
#define ADXL367_REG_INTMAP1_LOWER   0x2A
#define ADXL367_REG_INTMAP2_LOWER   0x2B
#define ADXL367_REG_FILTER_CTL          0x2C
#define ADXL367_REG_POWER_CTL           0x2D
#define ADXL367_REG_SELF_TEST           0x2E
#define ADXL367_REG_TAP_THRESH      0x2F
#define ADXL367_REG_TAP_DUR       0x30
#define ADXL367_REG_TAP_LATENT      0x31
#define ADXL367_REG_TAP_WINDOW      0x32
#define ADXL367_REG_X_OFFSET      0x33
#define ADXL367_REG_Y_OFFSET      0x34
#define ADXL367_REG_Z_OFFSET      0x35
#define ADXL367_REG_X_SEN       0x36
#define ADXL367_REG_Y_SEN       0x37
#define ADXL367_REG_Z_SEN       0x38
#define ADXL367_REG_TIMER_CTL     0x39
#define ADXL367_REG_INTMAP1_UPPER   0x3A
#define ADXL367_REG_INTMAP2_UPPER   0x3B
#define ADXL367_REG_ADC_CTL       0x3C
#define ADXL367_REG_TEMP_CTL      0x3D
#define ADXL367_REG_TEMP_ADC_OVER_THRSH_H 0x3E
#define ADXL367_REG_TEMP_ADC_OVER_THRSH_L 0x3F
#define ADXL367_REG_TEMP_ADC_UNDER_THRSH_H  0x40
#define ADXL367_REG_TEMP_ADC_UNDER_THRSH_L  0x41
#define ADXL367_REG_TEMP_ADC_TIMER    0x42
#define ADXL367_REG_AXIS_MASK     0x43
#define ADXL367_REG_STATUS_COPY     0x44
#define ADXL367_REG_STATUS2       0x45

/* ADXL367_REG_STATUS definitions */
#define ADXL367_STATUS_ERR_USER_REGS    (1 << 7)
#define ADXL367_STATUS_AWAKE            (1 << 6)
#define ADXL367_STATUS_INACT            (1 << 5)
#define ADXL367_STATUS_ACT              (1 << 4)
#define ADXL367_STATUS_FIFO_OVERRUN     (1 << 3)
#define ADXL367_STATUS_FIFO_WATERMARK   (1 << 2)
#define ADXL367_STATUS_FIFO_RDY         (1 << 1)
#define ADXL367_STATUS_DATA_RDY         (1 << 0)

/* ADXL367_REG_ACT_INACT_CTL definitions */
#define ADXL367_ACT_INACT_CTL_LINKLOOP(x)   (((x) & 0x3) << 4)
#define ADXL367_ACT_INACT_CTL_INACT_REF     (0x3 << 2)
#define ADXL367_ACT_INACT_CTL_INACT_EN      (0x1 << 2)
#define ADXL367_ACT_INACT_CTL_ACT_REF       (0x3 << 0)
#define ADXL367_ACT_INACT_CTL_ACT_EN        (0x1 << 0)

/* ADXL367_ACT_INACT_CTL_LINKLOOP(x) options */
#define ADXL367_MODE_DEFAULT            0
#define ADXL367_MODE_LINK               1
#define ADXL367_MODE_LOOP               3

/* ADXL367_REG_FIFO_CTL */
#define ADXL367_FIFO_CTL_XYZ      (0x0 <<3)
#define ADXL367_FIFO_CTL_X        (0x1 <<3)
#define ADXL367_FIFO_CTL_Y        (0x2 <<3)
#define ADXL367_FIFO_CTL_Z        (0x3 <<3)
#define ADXL367_FIFO_CTL_XYZ_TEM    (0x4 <<3)
#define ADXL367_FIFO_CTL_X_TEM      (0x5 <<3)
#define ADXL367_FIFO_CTL_Y_TEM      (0x6 <<3)
#define ADXL367_FIFO_CTL_Z_TEM      (0x7 <<3)
#define ADXL367_FIFO_CTL_XYZ_ADC    (0x8 <<3)
#define ADXL367_FIFO_CTL_X_ADC      (0x9 <<3)
#define ADXL367_FIFO_CTL_Y_ADC      (0xA <<3)
#define ADXL367_FIFO_CTL_Z_ADC      (0xB <<3)
#define ADXL367_FIFO_CTL_FIFO_SAMPLE8 (1<<2)
#define ADXL367_FIFO_CTL_FIFO_MODE(x)   ((x) & 0x3)

/* ADXL367_FIFO_CTL_FIFO_MODE(x) options */
#define ADXL367_FIFO_DISABLE            0
#define ADXL367_FIFO_OLDEST_SAVED       1
#define ADXL367_FIFO_STREAM             2
#define ADXL367_FIFO_TRIGGERED          3

/* ADXL367_REG_INTMAP1_LOWER definitions*/
#define ADXL367_INTMAP1_LOWER_INT_LOW_INT1        (1 << 7)
#define ADXL367_INTMAP1_LOWER_AWAKE_INT1           (1 << 6)
#define ADXL367_INTMAP1_LOWER_INACT_INT1           (1 << 5)
#define ADXL367_INTMAP1_LOWER_ACT_INT1             (1 << 4)
#define ADXL367_INTMAP1_LOWER_FIFO_OVERRUN_INT1    (1 << 3)
#define ADXL367_INTMAP1_LOWER_FIFO_WATERMARK_INT1  (1 << 2)
#define ADXL367_INTMAP1_LOWER_FIFO_READY_INT1      (1 << 1)
#define ADXL367_INTMAP1_LOWER_DATA_READY_INT1      (1 << 0)

/* ADXL367_REG_INTMAP2_LOWER definitions */
#define ADXL367_INTMAP2_LOWER_INT_LOW_INT2         (1 << 7)
#define ADXL367_INTMAP2_LOWER_AWAKE_INT2           (1 << 6)
#define ADXL367_INTMAP2_LOWER_INACT_INT2           (1 << 5)
#define ADXL367_INTMAP2_LOWER_ACT_INT2             (1 << 4)
#define ADXL367_INTMAP2_LOWER_FIFO_OVERRUN_INT2    (1 << 3)
#define ADXL367_INTMAP2_LOWER_FIFO_WATERMARK_INT2  (1 << 2)
#define ADXL367_INTMAP2_LOWER_FIFO_READY_INT2      (1 << 1)
#define ADXL367_INTMAP2_LOWER_DATA_READY_INT2      (1 << 0)

/* ADXL367_REG_INTMAP1_UPPER definitions*/
#define ADXL367_INTMAP1_UPPER_ERR_FUSE_INT1         (1 << 7)
#define ADXL367_INTMAP1_UPPER_ERR_USER_REGS_INT1    (1 << 6)
#define ADXL367_INTMAP1_UPPER_KPALV_TIMER_INT1    (1 << 4)
#define ADXL367_INTMAP1_UPPER_TEMP_ADC_HI_INT1      (1 << 3)
#define ADXL367_INTMAP1_UPPER_TEMP_ADC_LOW_INT1   (1 << 2)
#define ADXL367_INTMAP1_UPPER_TAP_TWO_INT1        (1 << 1)
#define ADXL367_INTMAP1_UPPER_TAP_ONE_INT1          (1 << 0)

/* ADXL367_REG_INTMAP1_UPPER definitions*/
#define ADXL367_INTMAP1_UPPER_ERR_FUSE_INT2         (1 << 7)
#define ADXL367_INTMAP1_UPPER_ERR_USER_REGS_INT2    (1 << 6)
#define ADXL367_INTMAP1_UPPER_KPALV_TIMER_INT2    (1 << 4)
#define ADXL367_INTMAP1_UPPER_TEMP_ADC_HI_INT2      (1 << 3)
#define ADXL367_INTMAP1_UPPER_TEMP_ADC_LOW_INT2   (1 << 2)
#define ADXL367_INTMAP1_UPPER_TAP_TWO_INT2        (1 << 1)
#define ADXL367_INTMAP1_UPPER_TAP_ONE_INT2          (1 << 0)

/* ADXL367_REG_FILTER_CTL definitions */
#define ADXL367_FILTER_CTL_RANGE(x)     (((x) & 0x3) << 6)
#define ADXL367_FILTER_CTL_I2C_HS       (1 << 5)
#define ADXL367_FILTER_CTL_EXT_SAMPLE   (1 << 3)
#define ADXL367_FILTER_CTL_ODR(x)       ((x) & 0x7) 

/* ADXL367_FILTER_CTL_RANGE(x) options */
#define ADXL367_RANGE_2G                0 /* +/-2 g */
#define ADXL367_RANGE_4G                1 /* +/-4 g */
#define ADXL367_RANGE_8G                2 /* +/-8 g */

/* ADXL367_FILTER_CTL_ODR(x) options */
#define ADXL367_ODR_12_5_HZ             0 /* 12.5 Hz */
#define ADXL367_ODR_25_HZ               1 /* 25 Hz */
#define ADXL367_ODR_50_HZ               2 /* 50 Hz */
#define ADXL367_ODR_100_HZ              3 /* 100 Hz */
#define ADXL367_ODR_200_HZ              4 /* 200 Hz */
#define ADXL367_ODR_400_HZ              5 /* 400 Hz */

/* ADXL367_REG_POWER_CTL definitions */
#define ADXL367_POWER_CTL_EXT_CLK       (1 << 6)
#define ADXL367_POWER_CTL_LOW_NOISE(x)  (((x) & 0x3) << 4)
#define ADXL367_POWER_CTL_WAKEUP        (1 << 3)
#define ADXL367_POWER_CTL_AUTOSLEEP     (1 << 2)
#define ADXL367_POWER_CTL_MEASURE(x)    (((x) & 0x3) << 0)

/* ADXL367_POWER_CTL_LOW_NOISE(x) options */
#define ADXL367_NOISE_MODE_NORMAL       0
#define ADXL367_NOISE_MODE_LOW          1

/* ADXL367_POWER_CTL_MEASURE(x) options */
#define ADXL367_MEASURE_STANDBY         0
#define ADXL367_MEASURE_ON              2

/* ADXL367_REG_SELF_TEST */
#define ADXL367_SELF_TEST_ST_FORCE    (1 << 0)
#define ADXL367_SELF_TEST_ST_RESULT   (0 << 0)

/* ADXL367 device information */
#define ADXL367_DEVICE_AD               0xAD
#define ADXL367_DEVICE_MST              0x1D
#define ADXL367_PART_ID                 0xF7

/* ADXL367 Reset settings */
#define ADXL367_RESET_KEY               0x52
/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/

class ADXL367
{
public:

	ADXL367();
	
	//
	// Basic Device control and readback functions
	//
	void begin(int16_t chipSelectPin = 10,int CLK_freq = 5000000); 		
	
	//  Low-level SPI control, to simplify overall coding
  void SPI_get_register_value(uint8_t* read_data, 
        uint8_t  register_address,uint8_t  bytes_number);
  void SPI_set_register_value(uint16_t register_value,uint8_t register_address,
        uint8_t bytes_number);
  void SPI_get_fifo_data(uint8_t  *data_buffer, uint16_t bytes_number);
  void SPI_software_reset();
  void SPI_set_power_mode(uint8_t pwr_mode);
  uint8_t SPI_set_range(uint8_t g_range);
  void SPI_set_output_rate(uint8_t out_rate);
  void SPI_get_xyz(int16_t* x,int16_t* y,int16_t* z);
  void SPI_get_g_xyz(float* x,float* y,float* z, uint8_t selected_range);
  float SPI_read_temperature();
  void SPI_fifo_setup(uint8_t  mode,uint16_t water_mark_lvl,bool  en_temp_read);
  int twos_complement(unsigned int val,unsigned char bits);
  

private:

	
};

#endif
