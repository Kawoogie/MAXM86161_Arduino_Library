/*

*/
#ifndef maxm86161_h
#define maxm86161_h

#include "Arduino.h"


class MAXM86161 {
    public:
    MAXM86161(int sda, int scl, int interrupt, int gpio);
    int read_from_reg(int address);
    int data_from_reg(int address, int&value);
    int write_to_reg(int address, int value);
    int start_sensor(void);
    int start_temp_read();
    int get_package_temp(float &temp_value);

    private:
    int _sda;
    int _scl;
    int _two_comp_to_dec(int two_comp);
    float _temperature_cal(float &temp_value);

};

#endif