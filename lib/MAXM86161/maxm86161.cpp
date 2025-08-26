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
 *            The pin used to interrupt the sensor.
 *    @param  gpio
 *            The pin used for the GPIO for the sensor.
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
    _interrupt = interrupt;
    _gpio = gpio;
    // Set the temperature reading calibration values
    _temp_cal_a = 1.0f;
    _temp_cal_b = 0.0f;
    bool error = false;
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
    error = i2c_dev->setSpeed(i2cSpeed);
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


/*!  @brief Set the starting parameters for the sensor
 *   @param integration_time Pulse width and integration time setting. Valid range: 0-3.
 *   @param sample_rate Samples per second setting value. Valid range: 0-19.
 *   @param averaging Number of samples to average. Valid range: 0-7.
 *   @param led_current LED drive current for all three LEDs. Valid range: 0-255.
 *   @returns Returns true if successful
 */
bool MAXM86161::startup(uint8_t integrate_time, uint8_t sample_rate, uint8_t averaging, uint8_t led_current)
{
    bool error;
    uint8_t reg_val[2];

    // Reset the sensor
    error = reset();
    if (!error){
        return false;
    }

    delay(5);

    // Shut down the sensor
    error = shutdown();
    if (!error){
        return false;
    }

    delay(5);

    // Clear the interrupts
    error = clear_interrupt();
    if (!error){
        return false;
    }

    // Clear interrupt bank 2
    error = data_from_reg(MAXM86161_INTERRUPT_STATUS_2, *reg_val);
    if (!error){
        return false;
    }

    // Set the integration time with max ADC range and with no ALC and ADD
    reg_val[0] = 0b00001100 + integrate_time;
    error = write_to_reg(MAXM86161_PPG_CONFIG_1, reg_val[0]);
    if (!error){
        return false;
    }

    // Set sample rate and averaging
    reg_val[0] = (sample_rate << 2) + averaging;
    error = write_to_reg(MAXM86161_PPG_CONFIG_2, reg_val[0]);
    if (!error){
        return false;
    }

    // Set LED Settling, digital filter, burst rate, burst enable
    reg_val[0] = 0x40;
    error = write_to_reg(MAXM86161_PPG_CONFIG_3, reg_val[0]);
    if (!error){
        return false;
    }

    // Set photodiode bias
    error = set_photodiode_bias(1);
    if (!error){
        return false;
    }

    // Set LED driver range to the maximum
    reg_val[0] = 3;
    error = set_led_driver_range(reg_val[0]);
    if (!error){
        return false;
    }

    // Set LED current
    error = set_led_current(led_current);
    if (!error){
        return false;
    }

    // Enable FIFO rollover when full
    reg_val[0] = 0b0001'1000;  // Clear interrupt on FIFO read, repeat interrupt once full condition is reached
    error = write_to_reg(MAXM86161_FIFO_CONFIG_2, reg_val[0]);
    if (!error){
        return false;
    }

    // Enable data ready interrupt
    error = data_ready_interrupt_enable(true);
    if (!error){
        return false;
    }

    // Set FIFO full to 15 empty spaces left
    reg_val[0] = 0xF;
    error = write_to_reg(MAXM86161_FIFO_CONFIG_1, reg_val[0]);
    if (!error){
        return false;
    }

    // Set LED exposure time slots
    // LED 2 to time slot 1, LED 3 to time slot 2
    reg_val[0] = 0x12;
    error = write_to_reg(MAMX86161_LED_SEQUENCE_1, reg_val[0]);
    if (!error){
        return false;
    }

    // Set LED exposure time slots continued
    reg_val[0] = 0x93;
    error = write_to_reg(MAMX86161_LED_SEQUENCE_2, reg_val[0]);
    if (!error){
        return false;
    }

    // Set LED exposure time slots continued again
    reg_val[0] = 0x00;
    error = write_to_reg(MAMX86161_LED_SEQUENCE_3, reg_val[0]);
    if (!error){
        return false;
    }

    // Enable low power mode
    error = enable_low_power_mode();
    if (!error){
        return false;
    }

    // Shut down until the program starts
    error = shutdown();
    if (!error){
        return false;
    }

    // Clear the flags
    error = clear_interrupt();

    return error;
}


/*!  @brief Start taking optical data with the sensor
 *   @returns True if no errors
 */
bool MAXM86161::start_sensor(void)
{
    bool error;
    uint8_t reg_val[1];

    // Get the data from the register
    error = data_from_reg(MAXM86161_SYSTEM_CONTROL, *reg_val);
    if (!error){
        return false;
    }
    // Write the start bit to the register
    reg_val[0] = reg_val[0] & ~(1 << MAXM86161_SHUTDOWN_SHIFT);
    error = write_to_reg(MAXM86161_SYSTEM_CONTROL, reg_val[0]);

    return error;
}

/*!  @brief Set the calibration values used for the package temperature values
 *   @param a slope of the calibration line
 *   @param b offset of the calibration line
 *   @returns void
 */
void MAXM86161::set_temp_cal(float a, float b)
{
    // Set the temperature reading calibration values
    _temp_cal_a = a;
    _temp_cal_b = b;

    return;
}


/*!  @brief Enable or disable the interrupt when data data is ready to read
 *   @param status True to enable the interrupt, false to disable
 *   @returns False if there is an error
 */
bool MAXM86161::data_ready_interrupt_enable(bool status)
{
    bool error;
    uint8_t reg_val[1];

    // Get the current status of the registry. Return 0 if error.
    error = data_from_reg(MAXM86161_INTERRUPT_ENABLE_1, *reg_val);
    if (!error){
        return false;
    }

    // Clear the bit if status is 0
    if (!status){
        reg_val[0] = reg_val[0] & ~(1 << MAXM86161_DATA_RDY_EN_SHIFT);
    }
    // Set the bit if status is not 0.
    else{
        reg_val[0] = reg_val[0] | (1 << MAXM86161_DATA_RDY_EN_SHIFT);
    }

    error = write_to_reg(MAXM86161_INTERRUPT_ENABLE_1, reg_val[0]);

    return error;
}


/*!  @brief Enable or disable the interrupt when the pakcage temp data data is ready to read
 *   @param status True to enable the interrupt, false to disable
 *   @returns False if there is an error
 */
bool MAXM86161::temp_ready_interrupt_enable(bool status)
{
    bool error;
    uint8_t reg_val[1];

    // Get the current status of the registry. Return 0 if error.
    error = data_from_reg(MAXM86161_INTERRUPT_ENABLE_1, *reg_val);
    if (!error){
        return false;
    }

    // Clear the bit if status is 0
    if (!status){
        reg_val[0] = reg_val[0] & ~(1 << MAXM86161_DIE_TEMP_RDY_EN_SHIFT);
    }
    // Set the bit if status is not 0.
    else{
        reg_val[0] = reg_val[0] | (1 << MAXM86161_DIE_TEMP_RDY_EN_SHIFT);
    }

    error = write_to_reg(MAXM86161_INTERRUPT_ENABLE_1, reg_val[0]);

    return error;
}



bool MAXM86161::read_sensor(int &red, int &green, int &ir, int &ambient, float &temp)
{
    bool error;
    bool data_ready = false;
    bool temp_ready = false;
    uint8_t interrupt;

    error = interrupt_status(interrupt);
    if (!error){
        return false;
    }

    // Parse the interrupt data to see which flags have been tripped
    // Check the data_rdy interrupt flag
    data_ready = interrupt & (1 << MAXM86161_DATA_RDY_EN_SHIFT);
    // Check the temp_ready interrupt flag
    temp_ready = interrupt & (1 << MAXM86161_DIE_TEMP_RDY_EN_SHIFT);

    // Read the data from the tripped flags
    if (data_ready){
        error = _get_optical_data(red, green, ir, ambient);
        if (!error){
            return false;
        }
    }

    if (temp_ready){
        error = get_package_temp(temp);
        start_temp_read();
    }

    return error;
}

bool MAXM86161::read_sensor(int &red, int &green, int &ir, int &ambient)
{
    bool error;
    bool data_ready = false;
    uint8_t interrupt;

    error = interrupt_status(interrupt);
    if (!error){
        return false;
    }

    // Parse the interrupt data to see if the data_rdy interrupt flag
    data_ready = interrupt & (1 << MAXM86161_DATA_RDY_EN_SHIFT);

    // Read the data from the tripped flags
    if (data_ready){
        error = _get_optical_data(red, green, ir, ambient);
        return error;
    }

    return false;
}

/*!  @brief Read the status of the interrupt register to determine what caused the interrupt
 *   @param status Data from the interrupt register. This also clears the interrupt
 *   @returns False if there is an error reading the register
 */
bool MAXM86161::interrupt_status(uint8_t &status)
{
    bool error;

    error = data_from_reg(MAXM86161_INTERRUPT_STATUS_1, status);
    
    return error;
}


/*!  @brief Start a temperature read if the temperature measurement is idle.
 *   @returns True temperature read started
 */
bool MAXM86161::start_temp_read()
{
    bool error;
    uint8_t reg_val[1];
    // Check to see if the device is doing a temp measurement
    error = data_from_reg(MAXM86161_TEMP_EN, *reg_val);
    if (!error || reg_val[0] != 0){
        return false;
    }

    // If not doing a temp measurement, start a temp measurement
    reg_val[0] = 1;
    error = write_to_reg(MAXM86161_TEMP_EN, reg_val[0]);
    
    return error;
}


/*!  @brief Read the package temperature.
 *   @param temp_value package temperature value
 *   @returns True temperature read successful
 */
bool MAXM86161::get_package_temp(float &temp_value)
{
    bool error;
    uint8_t temp_whole;
    uint8_t temp_frac;
    float temperature;
    uint8_t reg_val[1];


    // Check if the sensor temperature measurement is idle
    error = data_from_reg(MAXM86161_TEMP_EN, *reg_val);
    if (!error & (reg_val[0] == 0)){
        return false;
    }

    // Get the whole number portion
    error = data_from_reg(MAXM86161_TEMP_INT, temp_whole);
    if (!error){
        return false;
    }
    
    // Get the fractional portion
    error = data_from_reg(MAXM86161_TEMP_FRAC, temp_frac);
    if (!error){
        return false;
    }

    // Convert the whole number portion and make a float
    temperature = float(_two_comp_to_dec(temp_whole));

    // Add the fractional component
    temperature += (float(temp_frac & MAXM86161_TEMP_FRAC_MASK) * MAXM86161_FRAC_CONVERT);

    // Output the results
    temp_value = _temperature_cal(temperature);

    return error;
}


/*!  @brief Set the LED drive current for all three LEDs
 *   @param current The value to set the registries to. Valid range: 0-255
 *   @returns True if all three LEDs are set
 */
bool MAXM86161::set_led_current(uint8_t current)
{
    bool error;
    // Write to LED 1
    error = write_to_reg(MAMX86161_LED1_PA, current);
    if (!error){
        return false;
    }
    // Write to LED 2
    error = write_to_reg(MAMX86161_LED2_PA, current);
    if (!error){
        return false;
    }
    // Write to LED 3
    error = write_to_reg(MAMX86161_LED3_PA, current);
    return error;
}


/*!  @brief Read the drive current for the LEDs
 *   @param current The value that the registery is set to.
 *   @param led The LED to read the value from. Valid values: 1, 2, or 3.
 *   @returns True if no error reading the current
 */
bool MAXM86161::read_led_current(uint8_t &current, uint8_t led)
{
    bool error;

    if (led == 1){
        error = data_from_reg(MAMX86161_LED1_PA, current);
        return error;
    }
    else if (led == 2){
        error = data_from_reg(MAMX86161_LED2_PA, current);
        return error;
    }
    else if (led == 3){
        error = data_from_reg(MAMX86161_LED3_PA, current);
        return error;
    }
    else{
    return false;
    }
}

/*!  @brief Set the sample rate for the optical signals
 *   @param sample_rate The sample rate in Hz. Valid range: 0 to 19
 *   @returns True if the sample rate is set successfully
 */
bool MAXM86161::set_data_rate(uint8_t sample_rate)
{
    bool error;
    uint8_t reg_val[1];

    // Return an error if the sample rate setting is too high
    if (sample_rate > 20){
        return false;
    }

    // Read the existing registry value
    error = data_from_reg(MAXM86161_PPG_CONFIG_2, *reg_val);
    if (!error){
        return false;
    } 

    // Append the new sample rate to the existing registry data
    reg_val[0] = ((reg_val[0] & MAXM86161_PPG_SAMPLE_RATE_MASK) | (sample_rate << MAXM86161_PPG_SAMPLE_RATE_SHIFT));

    // Write the new value to the registry
    error = write_to_reg(MAXM86161_PPG_CONFIG_2, reg_val[0]);

    return error;
}


/*!  @brief Set the ADC range for the photodiode.
 *   @param range ADC range registry value, only acceptable values: 0 to 3
 *   @returns True if set successfully
 */
bool MAXM86161::set_adc_range(uint8_t range)
{
    bool error;
    uint8_t reg_val[1];
    
    // Acceptable range values
    uint8_t values[] = {0, 1, 2, 3};
    
    // Check if the range value is acceptable
    error = _arrayIncludeElement(values, 4, range);
    if (!error){
        return false;
    }

    // Read the registry to get the value of the other bits
    error = data_from_reg(MAXM86161_PPG_CONFIG_1, *reg_val);
    if (!error){
        return false;
    }

    // Append the new sample rate to the existing registry data
    reg_val[0] = ((reg_val[0] & MAXM86161_ADC_RANGE_MASK) | (range << MAXM86161_ADC_RANGE_SHIFT));

    // Write the updated registry
    error = write_to_reg(MAXM86161_PPG_CONFIG_1, reg_val[0]);

    return error;
}


/*!  @brief Set the integration time for the photodiode.
 *   @param integration_time ADC range registry value, only acceptable values: 0 to 3
 *   @returns True if set successfully
 */
bool MAXM86161::set_integration_time(uint8_t integration_time)
{
    bool error;
    uint8_t reg_val[1];
    
    // Acceptable range values
    uint8_t values[] = {0, 1, 2, 3};
    
    // Check if the range value is acceptable
    error = _arrayIncludeElement(values, 4, integration_time);
    if (!error){
        return false;
    }

    // Read the registry to get the value of the other bits
    error = data_from_reg(MAXM86161_PPG_CONFIG_1, *reg_val);
    if (!error){
        return false;
    }

    // Append the new sample rate to the existing registry data
    reg_val[0] = ((reg_val[0] & MAXM86161_INT_TIME_MASK) | (integration_time << MAXM86161_INT_TIME_SHIFT));

    // Write the updated registry
    error = write_to_reg(MAXM86161_PPG_CONFIG_1, reg_val[0]);

    return error;
}

/*!  @brief Set the photodiode bias capacitance.
 *   @param bias bias value, only acceptable values: 1, 5, 6, 7
 *   @returns True if bias set successfully
 */
bool MAXM86161::set_photodiode_bias(uint8_t bias)
{
    bool error;
    uint8_t buffer[8];

    uint8_t values[] = {1, 5, 6, 7};
    
    // Check if the bias value is acceptable
    error = _arrayIncludeElement(values, 4, bias);
    if (!error){
        return false;
    }

    // Write the new bias value
    error = write_to_reg(MAXM86161_PHOTO_DIODE_BIAS, bias);
    if (!error){
        return false;
    }

    // Read bias to ensure that it is set correctly
    error = data_from_reg(MAXM86161_PHOTO_DIODE_BIAS, *buffer);
    if (!error){
        return false;
    }

    
    return buffer[0] == bias;
}


/*!  @brief Set the LED driver range for all three LEDs.
 *   @param range LED driver registry value, only acceptable values: 0 to 3
 *   @returns True if set successfully
 */
bool MAXM86161::set_led_driver_range(uint8_t range)
{
    bool error;
    uint8_t reg_val[1];
    
    // Acceptable range values
    uint8_t values[] = {0, 1, 2, 3};
    
    // Check if the range value is acceptable
    error = _arrayIncludeElement(values, 4, range);
    if (!error){
        return false;
    }

    // Set up the value to write
    reg_val[0] = (range << MAXM86161_LED1_RANGE_SHIFT) + (range << MAXM86161_LED2_RANGE_SHIFT) + (range << MAXM86161_LED3_RANGE_SHIFT);

    error = write_to_reg(MAXM86161_LED_RANGE_1, reg_val[0]);

    return error;
}

/*!  @brief Reset the sensor
 *   @returns True if reset command sent successfully
 */
bool MAXM86161::reset(void)
{
    bool error;
    uint8_t reg_val[1];
    error = data_from_reg(MAXM86161_SYSTEM_CONTROL, *reg_val);
    if (!error){
        return false;
    }
    reg_val[0] = reg_val[0] + 1;
    error = write_to_reg(MAXM86161_SYSTEM_CONTROL, reg_val[0]);
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
    error = data_from_reg(MAXM86161_SYSTEM_CONTROL, *reg_val);
    if (!error){
        return false;
    }
    reg_val[0] = reg_val[0] | 1 << MAXM86161_SHUTDOWN_SHIFT;
    error = write_to_reg(MAXM86161_SYSTEM_CONTROL, reg_val[0]);
    if (!error){
        return false;
    }

    return true;
}

/*!  @brief Clears the interrupts from the sensor
 *   @returns True if reset command sent successfully
 */
bool MAXM86161::clear_interrupt()
{
    bool error;
    uint8_t reg_val[1];
    
    error = data_from_reg(MAXM86161_INTERRUPT_STATUS_1, *reg_val);

    return error;
}

/*!  @brief Puts the sensor in a low power mode between samples. Only works for sample rates of 256 sps and lower.
 *   @returns True if reset command sent successfully
 */
bool MAXM86161::enable_low_power_mode()
{
        bool error;
    uint8_t reg_val[1];
    error = data_from_reg(MAXM86161_SYSTEM_CONTROL, *reg_val);
    if (!error){
        return false;
    }
    reg_val[0] = reg_val[0] | 1 << MAXM86161_LOW_POWER_SHIFT;
    error = write_to_reg(MAXM86161_SYSTEM_CONTROL, reg_val[0]);

    return error;
}

/*!  @brief Set the I2C bus to low speed
 *   @param i2cSpeed Bus speed in kHz. Default of 100,000 kHz
 *   @returns True if speed adjusted. Note: Doesn't verify actual bus speed.
 */
bool MAXM86161::set_i2c_speed_low(uint32_t i2cSpeed)
{
    // Set the desired I2C speed
    bool error = i2c_dev->setSpeed(i2cSpeed);

    return error;
}

/*!  @brief Set the I2C bus to high speed
 *   @param i2cSpeed Bus speed in kHz. Default of 400,000 kHz
 *   @returns True if speed adjusted. Note: Doesn't verify actual bus speed.
 */
bool MAXM86161::set_i2c_speed_high(uint32_t i2cSpeed)
{
    // Set the desired I2C speed
    bool error = i2c_dev->setSpeed(i2cSpeed);

    return error;
}


/*!  @brief Clear the FIFO of the device and reset the data pointers
 *   @returns True if FIFO cleared
 */
bool MAXM86161::clear_fifo()
{
    bool error;
    uint8_t reg_val[1];
    
    // Get the current status of the register
    error = data_from_reg(MAXM86161_FIFO_CONFIG_2, *reg_val);
    if (!error){
        return false;
    }

    // Set the correct bit to clear the FIFO
    reg_val[0] = reg_val[0] | 1 << MAXM86161_FIFO_CLEAR_SHIFT;

    // Write the data to the register to reset the FIFO
    error = write_to_reg(MAXM86161_FIFO_CONFIG_2, reg_val[0]);

    return error;
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


/*!  @brief Twos compliment to decimal converter
 *   @param two_comp value to convert from
 *   @returns decimal value of two_comp
 */
int MAXM86161::_two_comp_to_dec(int two_comp)
{
    // If the value is positive, return it as it
    if ((two_comp >> 7) == 0){
        return two_comp;
    }   

    // If the value is negative, invert the bits, subtract one, 
    // then invert again.
    return two_comp | ~((1 << 7) - 1);
}


/*!  @brief Apply the calibration values to the temperature reading
 *   @param temp_value raw temperature reading needing calibration
 *   @returns calibrated temperature reading
 */
float MAXM86161::_temperature_cal(float &temp_value)
{
    float temp_cal;
    
    temp_cal = (_temp_cal_a * temp_value) + _temp_cal_b;

    return temp_cal;

}

/*!  @brief Checks if element is in array
 *   @param array array of integers to check
 *   @param array_size number of elements in the array
 *   @param element value to search for in array
 *   @returns True if element is in array, false otherwise
 */
bool MAXM86161::_arrayIncludeElement(uint8_t array[], uint8_t array_size, uint8_t element) {
 for (uint8_t i = 0; i < array_size; i++) {
      if (array[i] == element) {
          return true;
      }
    }
  return false;
 }


 /*!  @brief Reads the number of samples to read in the FIFO
 *   @param samples Number of samples in the FIFO
 *   @returns False if there is an error reading the register
 */
bool MAXM86161::_samples_to_read(uint8_t &samples)
{
    bool error;

    error = data_from_reg(MAXM86161_FIFO_DATA_COUNTER, samples);

    return error;
}


 /*!  @brief Reads the sensor data from the FIFO
 *   @param red Value of the red LED channel
 *   @param green Value of the green LED channel
 *   @param ir Value of the IR LED channel
 *   @param ambient Value of the ambient light channel
 *   @returns False if there is an error reading the data
 */
 bool MAXM86161::_get_optical_data(int &red, int &green, int &ir, int &ambient)
 {
    bool error;
    uint8_t databuffer[128*3];
    uint8_t samples;
    uint32_t temp_data;
    int label[32];
    int value[32];

    // Get the number of samples to read
    error = _samples_to_read(samples);
    if (!error){
        return false;
    }
    
    // Read all of the sample data from the FIFO
    Adafruit_BusIO_Register data = Adafruit_BusIO_Register(i2c_dev, MAXM86161_FIFO_DATA);
    error = data.read(databuffer, samples);


    // If there is no error reading the data, parse it
    if (error){
        for (int i = 0; i < samples; i++){
            temp_data = 0;
            temp_data = (databuffer[i*3 + 0] << 16)
                | (databuffer[i*3 + 1] << 8)
                | databuffer[i*3 + 2];
            // Apply the appropriate mask
            value[i] = temp_data & MAXM86161_REG_FIFO_DATA_MASK;
            label[i] = (temp_data >> MAXM86161_REG_FIFO_RES) & MAXM86161_REG_FIFO_TAG_MASK;
        }

        // Sort the data to the corresponding reading label
        for (int i = 0; i < 4; i++){
            if (label[i] == 3){
                red = value[i];
            }
            else if (label[i] == 2){
                green = value[i];
            }
            else if (label[i] == 1){
                ir = value[i];
            }
            else if (label[i] == 4){
                ambient = value[i];
            }
        }

    }

    return error;
 }
