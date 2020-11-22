# Raspberry and ATmega328P UART communication

This is a school IoT courses project work, which demonstrates UART communication between Raspberry Pi and ATmega328P microcontroller
using a LCD screen, temperature sensor and a LED.

## Parts used
- ATmega328P microcontroller (Arduino Uno)
- TMP36 temperature sensor
- LCD 16x2 display

## How it works
This ATmega328P side of the project is written in C and Atmel Studio 7 was used as the development environment.
LCD display, LED and a temperature sensor are connected to the Arduino Uno and temperature value is read through the ADC converter
and it is displayed on the LCD. Arduino Uno is also connected to a Raspberry Pi via UART and the connected LED brightness can be
controlled from the Raspberry by sending either "+" or "-". LED brightness is then controlled using PWM. The idea of using a temperature
sensor and PWM controlled LED was to demonstrate how, for example a cooling machine could be operated similarly through UART from Raspberry.
