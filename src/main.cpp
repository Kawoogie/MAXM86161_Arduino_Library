#include <Arduino.h>
#include "maxm86161.h"

// Define the MAXM86161 device
MAXM86161 sensor;

// Define the interrupt pin
const byte interruptPin = D3;  // Interrupt Pin D3
volatile byte interruptFlag = LOW;

// Function for the interrupt to trigger the interruptFlag
void interrupttrigger(){
  interruptFlag = HIGH;
}

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  
  // Set up the interrupt
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), interrupttrigger, FALLING);
  
  // Start Serial Communication
  Serial.begin(115200);

  // Wait for the Serial connection
  while (!Serial);

  // Begin the device
  byte start_error = sensor.begin(2, 3);

  if (!start_error){
    Serial.println("Problem initializing device.");
  }
  else {
    Serial.println("MAXM86161 initialized!");
  }

  // Set up the sensor parameters
  Serial.println("Starting up sensor");
  start_error = sensor.startup();
  Serial.print("Startup Status: ");
  Serial.println(start_error);

  sensor.shutdown();
  sensor.set_data_rate(4);

}

// the loop function runs over and over again forever
void loop() {
  byte error;
  int red = -99;
  int green = -99;
  int ir = -99;
  int ambient = -99;
  float temp = -99;
    
  Serial.println();
  Serial.println("Optical and Temp Reading Test");

  // sensor.clear_interrupt();
  // interruptFlag = LOW;

  // delay(100);
  // sensor.temp_ready_interrupt_enable(false);
  // sensor.data_ready_interrupt_enable(true);

  delay(100);
  Serial.println("Red, Green, IR, Ambient, Temp");
  sensor.start_sensor();
  sensor.start_temp_read();
  for (int i = 0; i < 200; i++) {

    while (!interruptFlag){
      delay(1);
    }

    sensor.get_package_temp(temp);
    error = sensor.read_sensor(red, green, ir, ambient);

    if (!error){
      Serial.println("***** ERROR READING DATA ******");
    }

    else{
      Serial.print(red);
      Serial.print(", ");
      Serial.print(green);
      Serial.print(", ");
      Serial.print(ir);
      Serial.print(", ");
      Serial.print(ambient);
      Serial.print(", ");
      Serial.println(temp);
    }

    if (!(i % 20)){
      sensor.start_temp_read();
    }

    // Clear the interrupt flag
    interruptFlag = LOW;
    sensor.clear_interrupt();

  }

  sensor.shutdown();
  Serial.println();
  Serial.println();
  Serial.println();
  delay(3000);           // wait for next scan
                
}