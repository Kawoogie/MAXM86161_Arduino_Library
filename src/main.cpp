#include <Arduino.h>
#include "maxm86161.h"

#include <Wire.h>


#define i2c_address 92
  
MAXM86161 sensor;

// Define the interrupt pin
const byte interruptPin = D3;  // Interrupt Pin D3
volatile byte interruptFlag = LOW;

void interrupttrigger(){
  interruptFlag = HIGH;
}


// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  
  // Set up the interrupt
  pinMode(interruptPin, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(interruptPin), interrupttrigger, CHANGE);

  // Initialize the I2C connection
  Wire.begin();

  // Start Serial Communication
  Serial.begin(115200);


  while (!Serial);             // Leonardo: wait for serial monitor
}

// the loop function runs over and over again forever
void loop() {
  byte error;
  // Define the MAXM86161 device
  error = sensor.begin(2, 3);

  if (!error){
    Serial.println("Problem initializing device.");
  }
  else {
    Serial.println("MAXM86161 initialized!");
  }


  // Test bias values
  error = sensor.set_photodiode_bias(1);
  Serial.print("Photodiode bias set to ");
  Serial.print(1);
  Serial.print(" Results: ");
  Serial.println(error);
  delay(50);
  error = sensor.set_photodiode_bias(5);
  Serial.print("Photodiode bias set to ");
  Serial.print(5);
  Serial.print(" Results: ");
  Serial.println(error);
  delay(50);
  error = sensor.set_photodiode_bias(6);
  Serial.print("Photodiode bias set to ");
  Serial.print(6);
  Serial.print(" Results: ");
  Serial.println(error);
  delay(50);
  error = sensor.set_photodiode_bias(7);
  Serial.print("Photodiode bias set to ");
  Serial.print(7);
  Serial.print(" Results: ");
  Serial.println(error);
  delay(50);
  error = sensor.set_photodiode_bias(4);
  Serial.print("Photodiode bias set to ");
  Serial.print(4);
  Serial.print(" Results: ");
  Serial.println(error);
  delay(50);
  error = sensor.set_photodiode_bias(10);
  Serial.print("Photodiode bias set to ");
  Serial.print(10);
  Serial.print(" Results: ");
  Serial.println(error);
  delay(50);
  //     // Write to Register
    // error = sensor.write_to_reg(0x09, i);

  //   if (!error){
  //     Serial.println("Write Error!");
  //   }

  uint8_t fifo[1];
  // Read from register to check starting value
  error = sensor.data_from_reg(0x09, *fifo);

  if (!error){
    Serial.println("Read Error!");
  }

  Serial.print("Register Starting Value: ");
  Serial.println(fifo[0]);

  // Write to the sensor's register
  error = sensor.write_to_reg(0x09, 100);

  // Read from register to check it changed
  error = sensor.data_from_reg(0x09, *fifo);

  if (!error){
    Serial.println("Read Error!");
  }

  Serial.print("Register new value: ");
  Serial.println(fifo[0]);

  Serial.println("Resetting the device");
  sensor.reset();
  delay(100);
  // Read from register to check it is the original value
  error = sensor.data_from_reg(0x09, *fifo);

  if (!error){
    Serial.println("Read Error!");
  }

  Serial.print("Register after reset: ");
  Serial.println(fifo[0]);
  
  /*
    Setting interrupt flag testing
  */

  Serial.println();
  Serial.print("Interrupt Flag Status:");
  sensor.data_from_reg(0x02, *fifo);
  Serial.println(fifo[0], BIN);

  Serial.println("Setting Temp Flag");
  sensor.temp_ready_interrupt_enable(true);

  Serial.print("Interrupt Flag Status:");
  sensor.data_from_reg(0x02, *fifo);
  Serial.println(fifo[0], BIN);

  Serial.println("Setting Data Flag");
  sensor.data_ready_interrupt_enable(true);

  Serial.print("Interrupt Flag Status:");
  sensor.data_from_reg(0x02, *fifo);
  Serial.println(fifo[0], BIN);

  Serial.println("Clearing Flags");
  sensor.temp_ready_interrupt_enable(false);
  sensor.data_ready_interrupt_enable(false);

  Serial.println();

/*
  Device Startup Testing
*/

  Serial.println("Starting up sensor");
  error = sensor.startup();
  Serial.print("Startup Status: ");
  Serial.println(error);

/*
  Temperature Reading Testing
*/

  Serial.println();
  Serial.println("Temperature Testing");

  Serial.println("Setting Temp Flag to enable");
  sensor.temp_ready_interrupt_enable(true);

  for (int i = 0; i < 5; i++) {
    Serial.println("Starting a temperature measurement");
    sensor.start_temp_read();

    Serial.print("Waiting for flag: ");
    Serial.println(interruptFlag);
    while (!interruptFlag){
      delay(1);
    }

    Serial.print("Interrupt Flag Triggered: ");
    Serial.println(interruptFlag);
    delay(100);
    interruptFlag = LOW;

  }
    



  Serial.println();
  Serial.println();
  Serial.println();
  delay(5000);           // wait 3 seconds for next scan

  // digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
  // Serial.println("LED On");
  // delay(500);                      // wait for a second
  // digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
  // Serial.println("LED Off");
  // delay(500);                      // wait for a second
}