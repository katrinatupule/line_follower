#include "Arduino.h"
#include "line_follower.hpp"

void setup() {
    // pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(9600);

}

void loop(LineFollower &line_flwr) {
    // digitalWrite(LED_BUILTIN, HIGH);
    // delay(500);
    // digitalWrite(LED_BUILTIN, LOW);
    // delay(500);

    line_flwr.follow_line();
}

int main(int args, char *argv[]) {
    Serial.println("Hello, line follower!\n");

    LineFollower line_flwr;
    // line_flwr.calibrate_sensor();
    loop(line_flwr);

    return 0;
}