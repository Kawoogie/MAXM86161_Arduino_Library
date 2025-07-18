/*

*/
#ifndef maxm86161_h
#define maxm86161_h

#include "Arduino.h"

#include <Wire.h>


#define MAXM86161_ADDRESS   0x62  //7-bit address

#define I2C_SPEED_STANDARD        100000
#define I2C_SPEED_FAST            400000


// Masks and shift value for getting value out of FIFO data.
#define MAXM86161_REG_FIFO_DATA_MASK  0x7FFFF
#define MAXM86161_REG_FIFO_RES        19
#define MAXM86161_REG_FIFO_TAG_MASK   0x1F


class MAXM86161 {
    public:
    MAXM86161(void);
    ~MAXM86161();

    bool begin(
        int interrupt, 
        int gpio, 
        TwoWire &wirePort = Wire, 
        uint32_t i2cSpeed = I2C_SPEED_FAST, 
        uint8_t i2c_addr = MAXM86161_ADDRESS);
    
    // Functions for I2C reading and writing
    bool read_from_reg(int address);
    bool data_from_reg(int address, int&value);
    bool write_to_reg(int address, int value);

    // Sensor initialization
    bool start_sensor(void);

    // Reading data from the sensor
    bool start_temp_read();
    bool samples_to_read();
    bool get_package_temp(float &temp_value);

    // Sensor settings
    bool set_led_current();
    bool read_led_current();
    bool set_data_rate();

    bool set_integration_time();
    bool set_photodiode_bias();
    bool set_led_driver_range();

    // Sensor Operations
    bool reset();
    bool shutdown();
    bool clear_interrupt();
    bool enable_low_power_mode();
    bool set_i2c_speed_low(uint32_t i2cSpeed = I2C_SPEED_STANDARD);
    bool set_i2c_speed_high(uint32_t i2cSpeed = I2C_SPEED_FAST);

    private:
    int _two_comp_to_dec(int two_comp);
    float _temperature_cal(float &temp_value);

};

#endif
