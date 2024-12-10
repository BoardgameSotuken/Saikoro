/*
 Arduino Library for Analog Devices ADXL367 - Micropower 3-axis accelerometer
 go to http://www.analog.com/ADXL367 for datasheet
 
 Created by Pablo del Corro <pablo.delcorro@analog.com>
 */ 
/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <Arduino.h>
#include "ADXL367.h"
#include <SPI.h>

/******************************************************************************/
/************************ Functions Definitions *******************************/
/******************************************************************************/
int16_t slaveSelectPin = 10;//SPI CS pin 

ADXL367::ADXL367() {
}

/***************************************************************************//**
 * @brief Initializes SPI communications with ADXL367 and performs a SW reset.
 *
 * @param chipSelectPin     - SPI Chip Select pin configuration.
 * @param CLK_freq          - SPI clock frequency in Hz.
 *
 * @return None.
*******************************************************************************/
void ADXL367::begin(int16_t chipSelectPin,int CLK_freq) {
	slaveSelectPin = chipSelectPin;
	pinMode(slaveSelectPin, OUTPUT);
	SPI.begin();
	//SPI.setDataMode(SPI_MODE0);	//CPHA = CPOL = 0    MODE = 0
	
	//for Arduino nano 33 BLE
	SPI.beginTransaction(SPISettings(CLK_freq, MSBFIRST, SPI_MODE0));//CPHA = CPOL = 0    MODE = 0
	delay(100);
  
  SPI_software_reset();
	delay(1000);
  uint8_t reg_value = 0;
  SPI_get_register_value(&reg_value,0x00,1);
  if(reg_value == 0xAD){
  Serial.println("Device Connected");
  }else{
    Serial.println("Failed to connect");
    Serial.println(reg_value);
  }
  //delay(1000);
 }
/***************************************************************************//**
 * @brief Performs a burst read of a specified number of registers.
 *
 * @param read_data        - The read values are stored in this buffer.
 * @param register_address - The start address of the burst read.
 * @param bytes_number     - Number of bytes to read.
 *
 * @return None.
*******************************************************************************/
void ADXL367::SPI_get_register_value(
        uint8_t* read_data,
        uint8_t  register_address,
        uint8_t  bytes_number)
{
  uint8_t data_buffer[32];
  uint8_t index = 0;

  data_buffer[0] = ADXL367_READ_REG;
  data_buffer[1] = register_address;
  for(index = 0; index < bytes_number; index++)
    data_buffer[index + 2] = read_data[index];
  digitalWrite(slaveSelectPin, LOW);
  SPI.transfer(&data_buffer, bytes_number + 2);
  digitalWrite(slaveSelectPin, HIGH);
  for(index = 0; index < bytes_number; index++)
    read_data[index] = data_buffer[index + 2];
}

/***************************************************************************//**
 * @brief Writes data into a register.
 *
 * @param register_value   - Data value to write.
 * @param register_address - Address of the register.
 * @param bytes_number     - Number of bytes. Accepted values: 0 - 1.
 *
 * @return None.
*******************************************************************************/
void ADXL367::SPI_set_register_value(
        uint16_t register_value,
        uint8_t register_address,
        uint8_t bytes_number)
{
  uint8_t data_buffer[4] = {0, 0, 0, 0};

  data_buffer[0] = ADXL367_WRITE_REG;
  data_buffer[1] = register_address;
  data_buffer[2] = (register_value & 0x00FF);
  data_buffer[3] = (register_value >> 8);
  digitalWrite(slaveSelectPin, LOW);
  SPI.transfer(&data_buffer, bytes_number + 2);
  digitalWrite(slaveSelectPin, HIGH);
}

/***************************************************************************//**
 * @brief Reads multiple bytes from the device's FIFO buffer.
 *
 * @param data_buffer       - Stores the read bytes.
 * @param bytes_number - Number of bytes to read.
 *
 * @return None.
*******************************************************************************/
void ADXL367::SPI_get_fifo_data(
          uint8_t  *data_buffer,
          uint16_t bytes_number)
{
  uint8_t spi_buffer[1024];

  uint16_t index = 0;

  spi_buffer[0] = ADXL367_READ_FIFO;
  for(index = 0; index < bytes_number; index++)
    spi_buffer[index + 1] = data_buffer[index];
  digitalWrite(slaveSelectPin, LOW);
  SPI.transfer(&spi_buffer, bytes_number + 1);
  digitalWrite(slaveSelectPin, HIGH);
  for(index = 0; index < bytes_number; index++)
    data_buffer[index] = spi_buffer[index + 1];
}

/***************************************************************************//**
 * @brief Resets the device via SPI communication bus.
 *
 * @return None.
*******************************************************************************/
void ADXL367::SPI_software_reset()
{
  SPI_set_register_value(
           ADXL367_RESET_KEY,
           ADXL367_REG_SOFT_RESET,
           1);
}
/***************************************************************************//**
 * @brief Places the device into standby/measure mode.
 *
 * @param pwr_mode - Power mode.
 *                   Example: 0 - standby mode.
 *                            1 - measure mode.
 *
 * @return None.
*******************************************************************************/
void ADXL367::SPI_set_power_mode(uint8_t pwr_mode)
{
  uint8_t old_power_ctl = 0;
  uint8_t new_power_ctl = 0;

  SPI_get_register_value(&old_power_ctl,
           ADXL367_REG_POWER_CTL,
           1);
  new_power_ctl = old_power_ctl & ~ADXL367_POWER_CTL_MEASURE(0x3);
  new_power_ctl = new_power_ctl |
      (pwr_mode * ADXL367_POWER_CTL_MEASURE(ADXL367_MEASURE_ON));
  SPI_set_register_value(new_power_ctl,
           ADXL367_REG_POWER_CTL,
           1);
}

/***************************************************************************//**
 * @brief Selects the measurement range.
 *
 * @param g_range - Range option.
 *                  Example: ADXL367_RANGE_2G  -  +-2 g
 *                           ADXL367_RANGE_4G  -  +-4 g
 *                           ADXL367_RANGE_8G  -  +-8 g
 *
 * @return selected_range.
*******************************************************************************/
uint8_t ADXL367::SPI_set_range(uint8_t g_range)
{
  uint8_t old_filter_ctl = 0;
  uint8_t new_filter_ctl = 0;
  uint8_t selected_range = 0;

  SPI_get_register_value(&old_filter_ctl,
           ADXL367_REG_FILTER_CTL,
           1);
  new_filter_ctl = old_filter_ctl & ~ADXL367_FILTER_CTL_RANGE(0x3);
  new_filter_ctl = new_filter_ctl | ADXL367_FILTER_CTL_RANGE(g_range);
  SPI_set_register_value(new_filter_ctl,
           ADXL367_REG_FILTER_CTL,
           1);
  selected_range = (1 << g_range) * 2;
  return selected_range; 
}

/***************************************************************************//**
 * @brief Selects the Output Data Rate of the device.
 *
 * @param out_rate - Output Data Rate option.
 *                   Example: ADXL367_ODR_12_5_HZ  -  12.5Hz
 *                            ADXL367_ODR_25_HZ    -  25Hz
 *                            ADXL367_ODR_50_HZ    -  50Hz
 *                            ADXL367_ODR_100_HZ   -  100Hz
 *                            ADXL367_ODR_200_HZ   -  200Hz
 *                            ADXL367_ODR_400_HZ   -  400Hz
 *
 * @return None.
*******************************************************************************/
void ADXL367::SPI_set_output_rate(uint8_t out_rate)
{
  uint8_t old_filter_ctl = 0;
  uint8_t new_filter_ctl = 0;

  SPI_get_register_value(&old_filter_ctl,
           ADXL367_REG_FILTER_CTL,
           1);
  new_filter_ctl = old_filter_ctl & ~ADXL367_FILTER_CTL_ODR(0x7);
  new_filter_ctl = new_filter_ctl | ADXL367_FILTER_CTL_ODR(out_rate);
  SPI_set_register_value(new_filter_ctl,
           ADXL367_REG_FILTER_CTL,
           1);
}

/***************************************************************************//**
 * @brief Reads the 3-axis raw data from the accelerometer.
 *
 * @param x   - Stores the X-axis data(as two's complement).
 * @param y   - Stores the Y-axis data(as two's complement).
 * @param z   - Stores the Z-axis data(as two's complement).
 *
 * @return None.
*******************************************************************************/
void ADXL367::SPI_get_xyz(int16_t* x,int16_t* y,int16_t* z)
{
  uint8_t xyz_values[6] = {0, 0, 0, 0, 0, 0};

  SPI_get_register_value(xyz_values,
           ADXL367_REG_XDATA_H,
           6);
//  *x = ((int16_t)xyz_values[0] << 6) | (xyz_values[1]>>2);
//  *x = twos_complement(*x,14);
//  *y = ((int16_t)xyz_values[2] << 6) | (xyz_values[3]>>2);
//  *y = twos_complement(*y,14);
//  *z = ((int16_t)xyz_values[4] << 6) | (xyz_values[5]>>2);
//  *z = twos_complement(*z,14);
  int16_t tmpX, tmpY, tmpZ;
  tmpX = ((int16_t)xyz_values[0]) << 6;
  tmpY = ((int16_t)xyz_values[2]) << 6;
  tmpZ = ((int16_t)xyz_values[4]) << 6;
  *x = tmpX | (xyz_values[1]>>2);
  *y = tmpY | (xyz_values[3]>>2);
  *z = tmpZ | (xyz_values[5]>>2);

//  *x = ((int16_t)xyz_values[0] << 6) | (xyz_values[1]>>2);
//  *y = ((int16_t)xyz_values[2] << 6) | (xyz_values[3]>>2);
//  *z = ((int16_t)xyz_values[4] << 6) | (xyz_values[5]>>2);
}

/***************************************************************************//**
 * @brief Reads the 3-axis raw data from the accelerometer and converts it to g.
 *
 * @param dev - The device structure.
 * @param x   - Stores the X-axis data.
 * @param y   - Stores the Y-axis data.
 * @param z   - Stores the Z-axis data.
 *
 * @return None.
*******************************************************************************/
void ADXL367::SPI_get_g_xyz(float* x,float* y,float* z, uint8_t selected_range)
{
  uint8_t xyz_values[6] = {0, 0, 0, 0, 0, 0};

  SPI_get_register_value(xyz_values,
           ADXL367_REG_XDATA_H,
           6);
  *x = ((int16_t)xyz_values[0] << 6) | (xyz_values[1]>>2);
  *x = twos_complement(*x,14);
  *x /= (1000 * (selected_range / 2));
  *y = ((int16_t)xyz_values[2] << 6) | (xyz_values[3]>>2);
  *y = twos_complement(*y,14);
  *y /= (1000 * (selected_range / 2));
  *z = ((int16_t)xyz_values[4] << 6) | (xyz_values[5]>>2);
  *z = twos_complement(*z,14);
  *z /= (1000 * (selected_range / 2));
}

/***************************************************************************//**
 * @brief Reads the temperature of the device.
 *
 * @param dev - The device structure.
 *
 * @return temp_celsius - The value of the temperature(degree s Celsius).
*******************************************************************************/
float ADXL367::SPI_read_temperature()
{
  uint8_t raw_temp_data[2] = {0, 0};
  int16_t signed_temp = 0;
  float temp_celsius = 0;

  SPI_get_register_value(raw_temp_data,
           ADXL367_REG_TEMP_H,
           2);
  signed_temp = (int16_t)(raw_temp_data[0] << 6) | (raw_temp_data[1]>>2);
  signed_temp = twos_complement(signed_temp,14);
  temp_celsius = (float)signed_temp * 0.0185;

  return temp_celsius;
}

/***************************************************************************//**
 * @brief Configures the FIFO feature.
 *
 * @param dev          - The device structure.
 * @param mode         - Mode selection.
 *                       Example: ADXL367_FIFO_DISABLE      -  FIFO is disabled.
 *                                ADXL367_FIFO_OLDEST_SAVED -  Oldest saved mode.
 *                                ADXL367_FIFO_STREAM       -  Stream mode.
 *                                ADXL367_FIFO_TRIGGERED    -  Triggered mode.
 * @param water_mark_lvl - Specifies the number of samples to store in the FIFO.
 * @param en_temp_read   - Store Temperature Data to FIFO.
 *                         Example: 1 - temperature data is stored in the FIFO
 *                                      together with x-, y- and x-axis data.
 *                                  0 - temperature data is skipped.
 *
 * @return None.
*******************************************************************************/
void ADXL367::SPI_fifo_setup(uint8_t  mode,
      uint16_t water_mark_lvl,
      bool  en_temp_read)
{
  uint8_t write_val = 0;
  if(en_temp_read){
    write_val = ADXL367_FIFO_CTL_FIFO_MODE(mode)|ADXL367_FIFO_CTL_XYZ_TEM;
  }else{
    write_val = ADXL367_FIFO_CTL_FIFO_MODE(mode)|ADXL367_FIFO_CTL_XYZ;
  }
  SPI_set_register_value(write_val,
           ADXL367_REG_FIFO_CTL,
           1);
  if(water_mark_lvl > 0xFF){
    SPI_set_register_value(ADXL367_FIFO_CTL_FIFO_SAMPLE8,
           ADXL367_REG_FIFO_CTL,
           1);
    SPI_set_register_value((water_mark_lvl & 0xFF),
           ADXL367_REG_FIFO_SAMPLES,
           1);       
  }else{
  SPI_set_register_value(water_mark_lvl,
           ADXL367_REG_FIFO_SAMPLES,
           1);
  }
}

/***************************************************************************//**
 * @brief covert to twos complement.
 *
 * @param val         - unsigned input 
 * @param bits        - word size.
 * 
 * @return value      - twos complement of val.
*******************************************************************************/
int ADXL367::twos_complement(unsigned int val,unsigned char bits)
{
    int value  = (int)val;
    if (value & (1 << (bits-1))){
       value -= 1 << bits;
    }
    return value;
}
