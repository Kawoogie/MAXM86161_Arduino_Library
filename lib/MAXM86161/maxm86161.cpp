/*
    maxm86161.cpp - Library for running the MAXM86161 sensor from an Arduino.
    Created by Lee Sikstrom 16 July 2025.
    Updated by Lee Sikstrom 16 July 2025.
    Released into the public domain.
*/


#include "Arduino.h"
#include "maxm86161.h"



// Masks and shift value for getting value out of FIFO data.
#define MAXM86161_REG_FIFO_DATA_MASK  0x7FFFF
#define MAXM86161_REG_FIFO_RES        19
#define MAXM86161_REG_FIFO_TAG_MASK   0x1F



/**
 * @brief Construct a new MAXM86161::MAXM86161 object
 *
 */
MAXM86161::MAXM86161(void)
{
}

/**
 * @brief Destroy the MAXM86161::MAXM86161 object
 *
 */
MAXM86161::~MAXM86161()
{
}

/*!
 *    @brief  Sets up the hardware and initializes I2C
 *    @param  interrupt
 *            The pin to use for the interrupt from the sensor.
 *    @param  gpio
 *            The pin used for the gpio connection to the sensor.
 *    @param  wirePort
 *            The Wire object to be used for I2C connections.
 *    @param  i2cSpeed
 *            The speed to set the I2C bus to. Defaults to 400k baud.
 *    @param  i2c_addr
 *            The I2C address to be used.
 *    @return True if initialization was successful, otherwise false.
 */
bool MAXM86161::begin(int interrupt, int gpio, TwoWire &wirePort, uint32_t i2cSpeed, uint8_t i2c_addr)
{
    return false;
}
