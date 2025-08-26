#include <Arduino.h>
#include "maxm86161.h"

// Define the MAXM86161 device
MAXM86161 sensor;

// Define the interrupt pin
const byte interruptPin = D3;  // Interrupt Pin D3
volatile byte interruptFlag = LOW;

// Function for the interrupt to trigger the interruptFlag
void interrupttoggle(){
  interruptFlag = !interruptFlag;
}

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  // Turn on the LED while starting up
  digitalWrite(LED_BUILTIN, HIGH);
  
  // Set up the interrupt
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), interrupttoggle, FALLING);
  
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
  start_error = sensor.startup();
  // Set the data rate of the sensor
  sensor.set_data_rate(4);
  // Shut down the sensor and wait for the command to start reading data
  sensor.shutdown();

  // Disable interrupts from package temp ready
  sensor.temp_ready_interrupt_enable(false);

  // Shut down the LED to indicate that the sensor is ready
  digitalWrite(LED_BUILTIN, LOW);

}

// the loop function runs over and over again forever
void loop() {
  byte error;
  int red = -99;
  int green = -99;
  int ir = -99;
  int ambient = -99;
  float temp = -99;

  // Troubleshoot hangup
  sensor.clear_fifo();
  sensor.clear_interrupt();

  delay(100);
  Serial.println("Red, Green, IR, Ambient, Temp");
  sensor.start_sensor();
  sensor.start_temp_read();
  for (int i = 0; i < 200; i++) {

    // Hangup troubleshooting
    Serial.print("BEFORE Flag: ");
    Serial.print(interruptFlag);
    bool interruptpinstate = digitalRead(interruptPin);
    Serial.print(" Pin: ");
    Serial.print(interruptpinstate);


    while (!interruptFlag){
      delay(1);
    }

    Serial.print(" |  AFTER: Flag: ");
    Serial.print(interruptFlag);
    interruptpinstate = digitalRead(interruptPin);
    Serial.print(" Pin: ");
    Serial.println(interruptpinstate);

    error = sensor.get_package_temp(temp);
    
    if (!error){
      Serial.println("***** ERROR READING TEMP ******");
    }
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

  // Stop the sensor
  sensor.shutdown();

  Serial.println();
  Serial.println();
  Serial.println();
  delay(3000);           // wait for next scan
                
}