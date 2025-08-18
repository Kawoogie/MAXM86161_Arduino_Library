/*

*/
#ifndef maxm86161_h
#define maxm86161_h

#include "Arduino.h"
#include <Adafruit_BusIO_Register.h>
#include <Adafruit_I2CDevice.h>
#include <Wire.h>


#define MAXM86161_ADDRESS             0x62  //7-bit address
#define MAXM86161_ID                  54  // Part ID value

#define MAXM86161_FRAC_CONVERT        0.0625  // For converting the package temp fractional component to decimal

#define I2C_SPEED_STANDARD            100000
#define I2C_SPEED_FAST                400000


// Registers
#define MAXM86161_INTERRUPT_STATUS_1  0x00  // Interrupt Status Registry
#define MAXM86161_PHOTO_DIODE_BIAS    0x15  // Photodiode Bias Registry
#define MAXM86161_SYSTEM_CONTROL      0x0D  // System Control Registry
#define MAXM86161_TEMP_EN             0x40  // Temperature Read Start Registry
#define MAXM86161_TEMP_INT            0x41  // Temperature integer portion registry
#define MAXM86161_TEMP_FRAC           0x42  // Temperature fraction portion registry
#define MAXM86161_PART_ID             0xFF  // Part ID Registry


// Masks and shift value for getting value out of FIFO data.
#define MAXM86161_REG_FIFO_DATA_MASK  0x7FFFF
#define MAXM86161_REG_FIFO_RES        19
#define MAXM86161_REG_FIFO_TAG_MASK   0x1F
#define MAXM86161_TEMP_FRAC_MASK      0b0000'1111


// typedef declarations


class MAXM86161 {
    public:
    MAXM86161(void);
    ~MAXM86161(void);

    bool begin(
        int interrupt, 
        int gpio, 
        TwoWire *wire = &Wire, 
        uint32_t i2cSpeed = I2C_SPEED_FAST, 
        uint8_t i2c_addr = MAXM86161_ADDRESS,
        uint32_t device_id = MAXM86161_ID);
    
    // Functions for I2C reading and writing
    bool data_from_reg(int address, uint8_t &value);
    bool write_to_reg(int address, uint8_t value);

    // Sensor initialization
    bool start_sensor(void);
    void set_temp_cal(float a, float b);

    // Reading data from the sensor
    bool samples_to_read();
    bool start_temp_read();
    bool get_package_temp(float &temp_value);

    // Sensor settings
    bool set_led_current();
    bool read_led_current();
    bool set_data_rate();

    bool set_adc_range();
    bool set_integration_time();
    bool set_photodiode_bias(uint8_t bias = 1);
    bool set_led_driver_range();

    // Sensor Operations
    bool reset();
    bool shutdown();
    bool clear_interrupt();
    bool enable_low_power_mode();
    bool set_i2c_speed_low(uint32_t i2cSpeed = I2C_SPEED_STANDARD);
    bool set_i2c_speed_high(uint32_t i2cSpeed = I2C_SPEED_FAST);

    private:
    int _interrupt;
    int _gpio;
    float _temp_cal_a;
    float _temp_cal_b;
    Adafruit_I2CDevice *i2c_dev = NULL; ///< Pointer to the I2C bus interface
    bool _init(uint32_t id);

    int _two_comp_to_dec(int two_comp);
    float _temperature_cal(float &temp_value);
    bool _arrayIncludeElement(uint8_t array[], uint8_t array_size, uint8_t element);

};

#endif
