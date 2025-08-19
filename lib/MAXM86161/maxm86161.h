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
#define MAXM86161_INTERRUPT_STATUS_2  0x01  // SHA Interrupt Status Registry
#define MAXM86161_INTERRUPT_ENABLE_1  0x02  // Interrupt Enable 1 Registry
#define MAXM86161_FIFO_CONFIG_1       0x09  // FIFO Configuration 1 Registry
#define MAXM86161_FIFO_CONFIG_2       0x0A  // FIFO Configuration 2 Registry
#define MAXM86161_SYSTEM_CONTROL      0x0D  // System Control Registry
#define MAXM86161_PPG_CONFIG_1        0x11  // PPG Configuration Registry 1
#define MAXM86161_PPG_CONFIG_2        0x12  // PPG Configuration Registry 2
#define MAXM86161_PPG_CONFIG_3        0x13  // PPG Configuration Registry 3
#define MAXM86161_PHOTO_DIODE_BIAS    0x15  // Photodiode Bias Registry
#define MAMX86161_LED_SEQUENCE_1      0x20  // LED Sequence 1 Registry
#define MAMX86161_LED_SEQUENCE_2      0x21  // LED Sequence 2 Registry
#define MAMX86161_LED_SEQUENCE_3      0x22  // LED Sequence 3 Registry
#define MAMX86161_LED1_PA             0x23  // LED Slot 1 Drive Current Registry
#define MAMX86161_LED2_PA             0x24  // LED Slot 2 Drive Current Registry
#define MAMX86161_LED3_PA             0x25  // LED Slot 3 Drive Current Registry
#define MAXM86161_LED_RANGE_1         0x2A  // LED Driver Range 1 Registry
#define MAXM86161_TEMP_EN             0x40  // Temperature Read Start Registry
#define MAXM86161_TEMP_INT            0x41  // Temperature integer portion registry
#define MAXM86161_TEMP_FRAC           0x42  // Temperature fraction portion registry
#define MAXM86161_PART_ID             0xFF  // Part ID Registry


// Masks and shift value for getting value out of FIFO data.
#define MAXM86161_REG_FIFO_DATA_MASK     0x7FFFF
#define MAXM86161_REG_FIFO_RES           19
#define MAXM86161_REG_FIFO_TAG_MASK      0x1F
#define MAXM86161_TEMP_FRAC_MASK         0b0000'1111
#define MAXM86161_DIE_TEMP_RDY_EN_SHIFT  2   // Shift for setting the Temp Interrupt Enable
#define MAXM86161_DATA_RDY_EN_SHIFT      6   // Shift for setting the Data Ready Interrupt Enable
#define MAXM86161_SHUTDOWN_SHIFT         1   // Shift for shutting down the device
#define MAXM86161_LOW_POWER_SHIFT        2   // Shift for putting the device in low power mode
#define MAXM86161_LED1_RANGE_SHIFT       0   // Shift for setting LED 1 driver range
#define MAXM86161_LED2_RANGE_SHIFT       2   // Shift for setting LED 2 driver range
#define MAXM86161_LED3_RANGE_SHIFT       4   // Shift for setting LED 3 driver range

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
    bool startup(uint8_t integrate_time = 3, uint8_t sample_rate = 0, uint8_t averaging = 0, uint8_t led_current = 0x34);
    bool start_sensor(void);
    void set_temp_cal(float a, float b);
    bool data_ready_interrupt_enable(bool status);
    bool temp_ready_interrupt_enable(bool status);

    // Reading data from the sensor
    bool interrupt_status(int status);
    bool samples_to_read();
    bool start_temp_read();
    bool get_package_temp(float &temp_value);

    // Sensor settings
    bool set_led_current(uint8_t current);
    bool read_led_current();
    bool set_data_rate(int sample_rate);

    bool set_adc_range();
    bool set_integration_time();
    bool set_photodiode_bias(uint8_t bias = 1);
    bool set_led_driver_range(uint8_t range);

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
