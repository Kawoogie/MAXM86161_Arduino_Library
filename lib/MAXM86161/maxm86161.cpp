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


MAXM86161::MAXM86161(int sda, int scl, int interrupt, int gpio)
{
}
