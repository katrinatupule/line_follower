#include "Arduino.h"
#include "line_follower.hpp"
#include "pin_setup.hpp"
#include <IRremote.h>

LineFollower line_flwr;

// State machine
bool running = false;

void setup() {
  Serial.begin(115200);
  IrReceiver.begin(IR_PIN);

  line_flwr.stopMotors();
  Serial.println("READY — Press 1 = Start cycle, 2 = Stop");
}

void loop() {

    // IR RECEIVE
    if (IrReceiver.decode()) {
        uint8_t key = IrReceiver.decodedIRData.command;
        Serial.print("Key: "); Serial.println(key, HEX);

        if (key == 0x0C) {  // Button "1"
            running = true;
            line_flwr.timer = millis();
            Serial.println("Start moving");
        }

        if (key == 0x18) {  // Button "2"
            running = false;
            line_flwr.stopMotors();
            Serial.println("STOP ALL");
        }

        IrReceiver.resume();
    }
    
    // Run line follower logic
    if (running) {
        line_flwr.follow_line();
    }

    // Serial runtime commands (non-blocking)
    if (Serial.available()) {
        char c = Serial.read();
        if (c == 's') { // toggle steer inversion
            line_flwr.toggleInvertSteer();
        } else if (c == 'm') { // toggle motor inversion
            line_flwr.toggleInvertMotors();
        } else if (c == 'p') { // print status
            line_flwr.printStatus();
        }
    }
}