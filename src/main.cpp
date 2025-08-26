#include <Arduino.h>
#include "maxm86161.h"

// #include <Wire.h>


// #define i2c_address 92
  
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
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), interrupttrigger, FALLING);  // CHANGE, RISING, FALLING, LOW

  // Initialize the I2C connection
  // Wire.begin();

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
    LED Driver Setting Testing
  */

  // Serial.println();
  // Serial.println("LED Driver Range Setting Testing");

  // // Read the LED Driver Register
  // error = sensor.data_from_reg(0x2A, *fifo);

  // Serial.print("Starting Reg Value: ");
  // Serial.println(fifo[0], BIN);

  // // Set a new value

  // Serial.println("Setting a new value");
  // error = sensor.set_led_driver_range(2);

  // Serial.print("New Value: ");
  // error = sensor.data_from_reg(0x2A, *fifo);
  // Serial.print("New Reg Value: ");
  // Serial.println(fifo[0], BIN);

  // Serial.println("Setting a new value");
  // error = sensor.set_led_driver_range(3);

  // Serial.print("New Value: ");
  // error = sensor.data_from_reg(0x2A, *fifo);
  // Serial.print("New Reg Value: ");
  // Serial.println(fifo[0], BIN);
  // Serial.println();


  /*
    Setting interrupt flag testing
  */

  Serial.println();
  // Serial.print("Interrupt Flag Status:");
  // sensor.data_from_reg(0x02, *fifo);
  // Serial.println(fifo[0], BIN);

  // Serial.println("Setting Temp Flag");
  // sensor.temp_ready_interrupt_enable(true);

  // Serial.print("Interrupt Flag Status:");
  // sensor.data_from_reg(0x02, *fifo);
  // Serial.println(fifo[0], BIN);

  // Serial.println("Setting Data Flag");
  // sensor.data_ready_interrupt_enable(true);

  // Serial.print("Interrupt Flag Status:");
  // sensor.data_from_reg(0x02, *fifo);
  // Serial.println(fifo[0], BIN);

  // Serial.println("Clearing Flags");
  // sensor.temp_ready_interrupt_enable(false);
  // sensor.data_ready_interrupt_enable(false);

  // Serial.println();

/*
  Device Startup Testing
*/

  Serial.println("Starting up sensor");
  error = sensor.startup();
  Serial.print("Startup Status: ");
  Serial.println(error);



/*
  Starting the optical sensors Testing
*/

  Serial.println("Starting LEDs");
  error = sensor.start_sensor();
  Serial.print("Start Status: ");
  Serial.println(error);

/*
  Data rate setting testing
*/
  // Serial.println("Setting a fast data rate");
  // sensor.set_data_rate(2);
  // delay(3000);
  // Serial.println("Setting low speed data rate");
  // sensor.set_data_rate(0x0A);
  // delay(3000);


/*
  Optical Interrupt Reset testing
*/
  // Serial.println("Optical Interrupt Reset Testing");
  // Serial.println("Shutting sensor down and clearing FIFO and interrupts");
  // sensor.shutdown();
  
  // Serial.print("Starting Interrupt Status: ");
  // sensor.interrupt_status(*fifo);
  // Serial.println(fifo[0], BIN);

  // sensor.clear_fifo();
  // sensor.clear_interrupt();
  // interruptFlag = LOW;

  // Serial.print("Starting Interrupt Status after clearning: ");
  // sensor.interrupt_status(*fifo);
  // Serial.println(fifo[0], BIN);

  // Serial.println("Turning Interrupts on");
  // // Turn interrupts on
  // sensor.data_ready_interrupt_enable(true);
  // sensor.temp_ready_interrupt_enable(true);
  // error = sensor.data_from_reg(0x02, *fifo);
  // Serial.print("Reg 0x02: ");
  // Serial.println(fifo[0], BIN);
  
  // delay(100);
  // Serial.println("Turning interrupts off");
  // // Turn interrupts off
  // sensor.data_ready_interrupt_enable(false);
  // sensor.temp_ready_interrupt_enable(false);
  // error = sensor.data_from_reg(0x02, *fifo);
  // Serial.print("Reg 0x02: ");
  // Serial.println(fifo[0], BIN);

  // Serial.println("Turning optical interrupt on");
  // sensor.data_ready_interrupt_enable(true);
  // error = sensor.data_from_reg(0x02, *fifo);
  // Serial.print("Reg 0x02: ");
  // Serial.println(fifo[0], BIN);

  // Serial.println("Starting Interrupt Status");
  // Serial.print("  Interrupt Flag: ");
  // Serial.println(interruptFlag);
  // Serial.print("  Interrupt Status: ");
  // sensor.interrupt_status(*fifo);
  // Serial.println(fifo[0], BIN);

  // Serial.println("Starting Sensor");
  // sensor.start_sensor();
  // delay(100);
  // sensor.interrupt_status(*fifo);
  // delay(100);
  // sensor.shutdown();

  // Serial.println("After Interrupt Status");
  // Serial.print("  Interrupt Flag: ");
  // Serial.println(interruptFlag);
  // Serial.print("  Interrupt Status: ");
  // Serial.println(fifo[0], BIN);

  // Serial.print("Check that shutdown cleared the interrupts:");
  // sensor.interrupt_status(*fifo);
  // Serial.println(fifo[0], BIN);

/*
  Optical Data Reading Testing
*/
  
  Serial.println();
  Serial.println("Reading Optical Data Test");
  int red = -99;
  int green = -99;
  int ir = -99;
  int ambient = -99;
  float temp = -99;

  sensor.shutdown();
  sensor.clear_fifo();
  delay(1000);
  sensor.temp_ready_interrupt_enable(false);
  sensor.data_ready_interrupt_enable(true);
  sensor.clear_interrupt();
  interruptFlag = LOW;
  sensor.start_sensor();
  
  for (int i = 0; i < 20; i++){
    Serial.print("Interrupt Flag at start: ");
    Serial.println(interruptFlag);

    while (!interruptFlag){
      delay(1);
    }

    Serial.print("   Interrupt Flag Triggered: ");
    Serial.println(interruptFlag);

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
      Serial.println(ambient);
    }
    // Clear the interrupt flag
    interruptFlag = LOW;
    // Clear the sensor interrupt
    sensor.clear_interrupt(); // Note: This is needed. Sensor hangs up eventually
  }



/*
  Temperature Reading Testing
*/

  sensor.data_ready_interrupt_enable(false);
  sensor.clear_interrupt();


  Serial.println();
  Serial.println("Temperature Testing");

  Serial.println("   Setting Temp Flag to enable");
  sensor.temp_ready_interrupt_enable(true);
  Serial.print("Interrupt Flag at start: ");
  Serial.println(interruptFlag);

  if (interruptFlag){
    interruptFlag = LOW;
  }

  sensor.clear_interrupt();

  for (int i = 0; i < 3; i++) {
    Serial.println("   Starting a temperature measurement");
    sensor.start_temp_read();

    Serial.print("   Waiting for flag: ");
    Serial.println(interruptFlag);
    while (!interruptFlag){
      delay(1);
    }

    Serial.print("   Interrupt Flag Triggered: ");
    Serial.println(interruptFlag);
    float package_temp = -99;

    error = sensor.get_package_temp(package_temp);
    
    if (error){
      Serial.print("Temperature: ");
      Serial.println(package_temp);
    }
    else {
      Serial.println("   Error reading the temperature");
  
    }
    // Reset the flags
    interruptFlag = LOW;
    // sensor.clear_interrupt();

  }
    
  /*
    Optical and Temperature reading test
  */


  Serial.println();
  Serial.println("Optical and Temp Reading Test");

  sensor.shutdown();
  sensor.set_data_rate(4);
  sensor.clear_fifo();
  red = -99;
  green = -99;
  ir = -99;
  ambient = -99;
  delay(1000);
  sensor.temp_ready_interrupt_enable(false);
  sensor.data_ready_interrupt_enable(true);
  sensor.clear_interrupt();
  interruptFlag = LOW;
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

    // Start a temp read, if so needed
    // sensor.get_package_temp(temp);

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
  delay(5000);           // wait for next scan
                
}