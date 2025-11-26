#include <Arduino.h>
#ifndef PINS_SETUP_HPP
    // input pins for IR sensors
    #define I_IR0 A4
    #define I_IR1 A5
    #define I_IR2 A6
    #define I_IR3 A0
    #define I_IR4 A1

    #define I_BLACK HIGH
    #define I_WHITE LOW
    
    // infrared receiver pin
    #define IR_PIN 2

    // L298N pins
    #define ENA 5
    #define IN1 4
    #define IN2 3
    #define ENB 6
    #define IN3 7
    #define IN4 8

#endif // PINS_SETUP_HPP