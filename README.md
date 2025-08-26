<!-- Improved compatibility of back to top link: See: https://github.com/othneildrew/Best-README-Template/pull/73 -->
<a name="readme-top"></a>
<!-- -->



<!-- PROJECT SHIELDS -->
<!--
*** I'm using markdown "reference style" links for readability.
*** Reference links are enclosed in brackets [ ] instead of parentheses ( ).
*** See the bottom of this document for the declaration of the reference variables
*** for contributors-url, forks-url, etc. This is an optional, concise syntax you may use.
*** https://www.markdownguide.org/basic-syntax/#reference-style-links
-->
[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![MIT License][license-shield]][license-url]
[![LinkedIn][linkedin-shield]][linkedin-url]



<!-- PROJECT LOGO -->
<br />
<div align="center">
  <a href="https://github.com/Kawoogie/MAXM86161_Arduino_Library">
    <img src="images/logo.png" alt="Logo" width="80" height="80">
  </a>

<h3 align="center">MAXM86161 Arduino Library.</h3>

  <p align="center">
    This library runs the MAXM86161 optical sensor using an Arduino. The MAXM86161 is used for pulse oximetry.
    <br />
    <a href="https://github.com/Kawoogie/MAXM86161_Arduino_Library"><strong>Explore the docs »</strong></a>
    <br />
    <br />
    <a href="https://github.com/Kawoogie/MAXM86161_Arduino_Library">View Demo</a>
    ·
    <a href="https://github.com/Kawoogie/MAXM86161_Arduino_Library/issues">Report Bug</a>
    ·
    <a href="https://github.com/Kawoogie/MAXM86161_Arduino_Library/issues">Request Feature</a>
  </p>
</div>



<!-- TABLE OF CONTENTS -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#about-the-project">About The Project</a>
      <ul>
        </ul>
    </li>
    <li>
      <a href="#getting-started">Getting Started</a>
      <ul>
        <li><a href="#prerequisites">Prerequisites</a></li>
        <li><a href="#installation">Installation</a></li>
      </ul>
    </li>
    <li><a href="#usage">Usage</a></li>
    <li><a href="#roadmap">Roadmap</a></li>
    <li><a href="#contributing">Contributing</a></li>
    <li><a href="#license">License</a></li>
    <li><a href="#contact">Contact</a></li>    
  </ol>
</details>



<!-- ABOUT THE PROJECT -->
## About The Project
This library is written to drive the MAXM86161 pulse oximeter sensor with an Arduino. The library is written to get the raw optical data from the sensor for the computation of heart rate and blood oxygenation.

- The `maxm86161.h` and `maxm86161.cpp` files contain the split library code with detailed Doxygen-style comments. 
- The `README.md` file provides a description of the library, installation instructions, usage example, API documentation, and license information, formatted for posting on GitHub.
<p align="right">(<a href="#readme-top">back to top</a>)</p>




<!-- GETTING STARTED -->
## Getting Started

The sensor's manufacture website: https://www.analog.com/en/products/maxm86161.html

To use this library, include the `maxm86161.h` and `maxm86161.cpp` files in your Arduino project. This is easiest done importing as a library.



### Prerequisites
This code is tested on an Arduino Nano 33 BLE Sense Rev 2
  

### Installation
This code can be downloaded to the local Arduino library folder. 

If using Platform IO, it can be imported by adding  
```
lib_deps = https://github.com/Kawoogie/MAXM86161_Arduino_Library.git
```
to the platformio.ini file.


<p align="right">(<a href="#readme-top">back to top</a>)</p>

  


<!-- USAGE EXAMPLES -->
## Usage
Here's an example of how to use the MAXM86161 library to get raw optical data using the chip’s built in interrupts. The code collects 2000 optical readings, then waits 5 seconds with the MAXM86161 shutdown before repeating another 2000 readings. The package temperature is read and output as well.

### Example 

```
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
  // Turn on the LED while starting up
  digitalWrite(LED_BUILTIN, HIGH);
  
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
  start_error = sensor.startup();
  // Set the data rate of the sensor
  sensor.set_data_rate(4);
  // Shut down the sensor and wait for the command to start reading data
  sensor.shutdown();

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
```
Note: if using something other than the Arduino IDE, include the Arduino library at the top of the code as well
```
#include <Arduino.h>
```

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- ROADMAP -->
## Roadmap

See the [open issues](https://github.com/Kawoogie/MAXM86161_Arduino_Library/issues) for a full list of proposed features (and known issues).

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- CONTRIBUTING -->
## Contributing

Contributions are what make the open source community such an amazing place to learn, inspire, and create. Any contributions you make are **greatly appreciated**.

If you have a suggestion that would make this better, please fork the repo and create a pull request. You can also simply open an issue with the tag "enhancement".
Don't forget to give the project a star! Thanks again!

1. Fork the Project
2. Create your Feature Branch (`git checkout -b feature/AmazingFeature`)
3. Commit your Changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the Branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- LICENSE -->
## License

Distributed under the GPL-3.0 License. See `LICENSE.txt` for more information.

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- CONTACT -->
## Contact
Project Link: [https://github.com/Kawoogie/MAXM86161_Arduino_Library](https://github.com/Kawoogie/MAXM86161_Arduino_Library)

<p align="right">(<a href="#readme-top">back to top</a>)</p>






<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[contributors-shield]: https://img.shields.io/github/contributors/Kawoogie/MAXM86161_Arduino_Library.svg?style=for-the-badge
[contributors-url]: https://github.com/Kawoogie/MAXM86161_Arduino_Library/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/Kawoogie/MAXM86161_Arduino_Library.svg?style=for-the-badge
[forks-url]: https://github.com/Kawoogie/MAXM86161_Arduino_Library/network/members
[stars-shield]: https://img.shields.io/github/stars/Kawoogie/MAXM86161_Arduino_Library.svg?style=for-the-badge
[stars-url]: https://github.com/Kawoogie/MAXM86161_Arduino_Library/stargazers
[issues-shield]: https://img.shields.io/github/issues/Kawoogie/MAXM86161_Arduino_Library.svg?style=for-the-badge
[issues-url]: https://github.com/Kawoogie/MAXM86161_Arduino_Library/issues
[license-shield]: https://img.shields.io/github/license/Kawoogie/MAXM86161_Arduino_Library.svg?style=for-the-badge
[license-url]: https://github.com/Kawoogie/MAXM86161_Arduino_Library/blob/master/LICENSE.txt
[linkedin-shield]: https://img.shields.io/badge/-LinkedIn-black.svg?style=for-the-badge&logo=linkedin&colorB=555
[linkedin-url]: https://linkedin.com/in/lee-sikstrom-a6472a113
[product-screenshot]: images/screenshot.png
[Next.js]: https://img.shields.io/badge/next.js-000000?style=for-the-badge&logo=nextdotjs&logoColor=white
[Next-url]: https://nextjs.org/
[React.js]: https://img.shields.io/badge/React-20232A?style=for-the-badge&logo=react&logoColor=61DAFB
[React-url]: https://reactjs.org/
[Vue.js]: https://img.shields.io/badge/Vue.js-35495E?style=for-the-badge&logo=vuedotjs&logoColor=4FC08D
[Vue-url]: https://vuejs.org/
[Angular.io]: https://img.shields.io/badge/Angular-DD0031?style=for-the-badge&logo=angular&logoColor=white
[Angular-url]: https://angular.io/
[Svelte.dev]: https://img.shields.io/badge/Svelte-4A4A55?style=for-the-badge&logo=svelte&logoColor=FF3E00
[Svelte-url]: https://svelte.dev/
[Laravel.com]: https://img.shields.io/badge/Laravel-FF2D20?style=for-the-badge&logo=laravel&logoColor=white
[Laravel-url]: https://laravel.com
[Bootstrap.com]: https://img.shields.io/badge/Bootstrap-563D7C?style=for-the-badge&logo=bootstrap&logoColor=white
[Bootstrap-url]: https://getbootstrap.com
[JQuery.com]: https://img.shields.io/badge/jQuery-0769AD?style=for-the-badge&logo=jquery&logoColor=white
[JQuery-url]: https://jquery.com
