/*
    maxm86161.cpp - Library for running the MAXM86161 sensor from an Arduino.
    Created by Lee Sikstrom 16 July 2025.
    Updated by Lee Sikstrom 16 July 2025.
    Released into the public domain.
*/

#include "Arduino.h"
#include <Wire.h>
#include "maxm86161.h"




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
MAXM86161::~MAXM86161(void)
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
bool MAXM86161::begin(int interrupt, int gpio, TwoWire *wire, uint32_t i2cSpeed, uint8_t i2c_addr, uint32_t device_id)
{
    // Save the pin values for the interrupt and gpio
    _interrupt = interrupt;
    _gpio = gpio;
    
    // Check if a device is defined and remove it
    if (i2c_dev) {
        delete i2c_dev; // remove old interface
    }

    // Define a new I2C device
    i2c_dev = new Adafruit_I2CDevice(i2c_addr, wire);

    // Start the device and return a fault if it doesn't start
    if (!i2c_dev->begin()) {
        return false;
    }

    // Set the desired I2C speed
    bool error = i2c_dev->setSpeed(i2cSpeed);
    if (!error){
        return false;
    }

    // return true;
    bool isInit;
    isInit = _init(device_id);
    return isInit;
}

/*!  @brief Read from a register on the device
 *   @param address Register address to read from
 *   @param value Value from the register
 *   @returns True if data read is successful
 */
bool MAXM86161::data_from_reg(int address, uint8_t &value)
{
    bool error;

    Adafruit_BusIO_Register data = Adafruit_BusIO_Register(i2c_dev, address);

    error = data.read(&value);

    return error;
}

/*!  @brief Write to a register on the device
 *   @param address Register address to write data
 *   @param value Data to write
 *   @returns True if data write is successful
 */
bool MAXM86161::write_to_reg(int address, uint8_t value)
{
    bool error;

    Adafruit_BusIO_Register data = Adafruit_BusIO_Register(i2c_dev, address);

    error = data.write(value, 1);

    return error;
}

/*!  @brief Reset the sensor
 *   @returns True if reset command sent successfully
 */
bool MAXM86161::reset(void)
{
    bool error;
    uint8_t reg_val[1];
    error = sensor.data_from_reg(MAXM86161_SYSTEM_CONTROL, *reg_val);
    if (!error){
        return false;
    }
    reg_val[0] = reg_val[0] + 1;
    error = sensor.write_to_reg(MAXM86161_SYSTEM_CONTROL, reg_val[0]);
    if (!error){
        return false;
    }

    return true;
}

/*!  @brief Shutdown the sensor in a low power mode
 *   @returns True if reset command sent successfully
 */
bool MAXM86161::shutdown(void)
{
    bool error;
    uint8_t reg_val[1];
    error = sensor.data_from_reg(MAXM86161_SYSTEM_CONTROL, *reg_val);
    if (!error){
        return false;
    }
    reg_val[0] = reg_val[0] + 2;
    error = sensor.write_to_reg(MAXM86161_SYSTEM_CONTROL, reg_val[0]);
    if (!error){
        return false;
    }

    return true;
}

/*!  @brief Initializer for post i2c/spi init
 *   @param id ID for the sensor
 *   @returns True if chip identified and initialized
 */
bool MAXM86161::_init(uint32_t id)
{
    bool error;
    uint8_t buffer[8];

    // Read the part ID from the device
    error = data_from_reg(MAXM86161_PART_ID, *buffer);
    if (!error){
        return false;
    }

    // Check if the part ID is correct
    if (buffer[0] == MAXM86161_ID){
        return true;
    }
    else {
        return false;
    }
}
