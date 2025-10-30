#include "Arduino.h"
#include "line_follower.hpp"
#include "pin_setup.hpp"

LineFollower line_flwr;

void setup() {
    Serial.begin(115200);
    Serial.println("Hello, line follower!\n");
    // line_flwr.calibrate_sensor();
}

void loop() {
    line_flwr.follow_line();
    // int sensorValue = digitalRead(I_IR3);

    // if (sensorValue == HIGH) {
    //     Serial.println("black");
    //     Serial.println(sensorValue);
    // } else {
    //     Serial.println("white");
    //     Serial.println(sensorValue);
    // }
    delay(500);
}