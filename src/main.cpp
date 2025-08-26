#include <Arduino.h>
#include "maxm86161.h"
#include <Wire.h>

// Declare the MAXM86161 sensor object  
MAXM86161 sensor;

// Define the interrupt pin
const byte interruptPin = D3;  // Interrupt Pin D3
volatile byte interruptFlag = LOW;

// Function for the interrupt
void interrupttrigger(){
  interruptFlag = HIGH;
}


// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // Set up the interrupt
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), interrupttrigger, FALLING);

  // Initialize the I2C connection
  Wire.begin();

  // Start Serial Communication
  Serial.begin(115200);

  while (!Serial); 
}

// the loop function runs over and over again forever
void loop() {
  // Declar variables for storing data
  byte error;
  int red = -99;
  int green = -99;
  int ir = -99;
  int ambient = -99;
  float temp = -99;
  
  delay(100);
  Serial.println();
  Serial.println("***************************");
  Serial.println("      STARTING       ");
  Serial.println("***************************");
  Serial.println();

  delay(100);
  error = sensor.begin();
  
    // Define the MAXM86161 device
  if (!error){
    Serial.println("Problem initializing device.");
  }
  else {
    Serial.println("MAXM86161 initialized!");
  }

  // Set up sensor for taking data
  Serial.println("Setting up the sensor");
  error = sensor.startup();
  if (!error){
    Serial.println("Error setting up the sensor");
  }
  sensor.shutdown();
  Serial.println();
  Serial.println("Reading Optical Data and Package Temperature");

  delay(100);

  // Set the sensor data rate to 200 Hz
  sensor.set_data_rate(4);

  // Set the flags and clear the interrupts
  sensor.clear_fifo();
  // sensor.temp_ready_interrupt_enable(false);
  // sensor.data_ready_interrupt_enable(true);
  sensor.clear_interrupt();
  // interruptFlag = LOW;

  delay(100);

  // Print the data header
  Serial.println("Red, Green, IR, Ambient, Temp");
  
  // Start the sensor rading
  sensor.start_temp_read();
  sensor.start_sensor();

  // take 2000 data points
  for (int i = 0; i < 2000; i++) {

    // Wait for an interrupt to read the data
    while (!interruptFlag){
      digitalWrite(LED_BUILTIN, HIGH);
      delay(1);
    }

    digitalWrite(LED_BUILTIN, LOW);

    // Get the temperature data
    sensor.get_package_temp(temp);
    // Get the optical data
    error = sensor.read_sensor(red, green, ir, ambient);

    if (!error){
      Serial.println("***** ERROR READING DATA ******");
    }

    // Return the values if there is no error
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

    // Start a temperature read every 20 optical data points
    if (!(i % 20)){
      sensor.start_temp_read();
    }

    // Clear the interrupt flag
    interruptFlag = LOW;
    sensor.clear_interrupt();

  }

  Serial.println("Shutting Down Sensor for 5 Seconds");
  sensor.shutdown();
  Serial.println();
  Serial.println();
  Serial.println();
  delay(5000);           // wait for next scan            
}