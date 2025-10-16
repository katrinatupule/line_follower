#include "Arduino.h"
#include "line_follower.hpp"

LineFollower line_flwr;

void setup() {
    Serial.begin(9600);
    Serial.println("Hello, line follower!\n");
    // line_flwr.calibrate_sensor();
}

void loop() {
    line_flwr.follow_line();
    delay(500);
}