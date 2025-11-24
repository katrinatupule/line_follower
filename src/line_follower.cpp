#include "Arduino.h"
#include "line_follower.hpp"
#include "pin_setup.hpp"

/*
TODO:

- build robot
    - select motors
        - test existing motors; identify shortcommings
        - test motor control

    - select motor driver
        - test existing, if available
    - select sensors
        - test existing
        - compare alternatives
        - define vague solutions with each sensor; identify shortcommings and advantages
        - implement sensor reading

    - select microcontroller
        - define requirements (pins, performance, voltage, current, power consumption, size, weight)
        - compare alternatives
    - select power supply
        - define requirements (voltage, current, capacity, weight, size)
        - compare alternatives
    - design circuit
    - order parts (if necessary)
    - wire robot
        - map pins
    - build robot chassis
        - design alternatives
        - define advantages and shortcommings
        - order/print parts
        - build (and rebuild/adjust) chassis
 - implement line following algorithm
    - implement robot driving (forward, backward, stop)
        - implement speed control (slow, fast, accelerate, decelerate) based on sensor input and steering
        - PID speed control (if needed)
    - implemet steering (left, right, straight)
        - implement steering control based on sensor input
        - test different steering strategies
            - turn with one wheel active
            - turn with both wheels active (different speeds, directions)
            - turn with additional motor (if available, if needed)
            - PID steering control (if needed)
    - implement decision making based on sensor input
        - test different sensor configurations (if possible)
        - implement two sensor setup
        - implement four sensor setup
        - implement more complex sensor setup (if possible)

*/

LineFollower::LineFollower() {
    now = millis();
    motor_phase = 0;
    
    slow_speed_right = 100;
    slow_speed_left = 95;
    
    last_throttle = 0.0;
    last_steer = 0.0;
    new_input = false;
    digital = false;
    sensor_input_count = 3;
    sensor_pin_nrs = new int[sensor_input_count] {I_IR2, I_IR1, I_IR0}; // left to right
    last_sensor_input = new float[sensor_input_count] {0.0, 0.0, 0.0};

    for (int i=0; i<sensor_input_count; i++) {
        pinMode(sensor_pin_nrs[i], INPUT);
    }

    // left motor
    pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
    // right motor
    pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
}

void LineFollower::forward() {
    // left
    Serial.println("forward left");
    digitalWrite(IN1, HIGH); 
    digitalWrite(IN2, LOW); 
    analogWrite(ENA, slow_speed_right);

    // right
    Serial.println("forward right");
    digitalWrite(IN3, HIGH); 
    digitalWrite(IN4, LOW); 
    analogWrite(ENB, slow_speed_left);
}

void LineFollower::backward() {
    // left
    Serial.println("backward left");
    digitalWrite(IN1, LOW); 
    digitalWrite(IN2, HIGH); 
    analogWrite(ENA, slow_speed_right);
    
    // right
    Serial.println("backward right");
    digitalWrite(IN3, LOW); 
    digitalWrite(IN4, HIGH); 
    analogWrite(ENB, slow_speed_left);
}

void LineFollower::left() {
    // left motor
    digitalWrite(IN1, LOW); 
    digitalWrite(IN2, HIGH); 
    analogWrite(ENA, slow_speed_right);
    
    // right motor
    digitalWrite(IN3, HIGH); 
    digitalWrite(IN4, LOW); 
    analogWrite(ENB, slow_speed_left);
}

void LineFollower::right() {
    // left motor
    digitalWrite(IN1, HIGH); 
    digitalWrite(IN2, LOW); 
    analogWrite(ENA, slow_speed_right);
    
    // right motor
    digitalWrite(IN3, LOW); 
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, slow_speed_left);
}


void LineFollower::stopMotors() {
  digitalWrite(IN1, LOW); 
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); 
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0); 
  analogWrite(ENB, 0);
}

void LineFollower::test_motors() {
    now = millis();

    switch (motor_phase) {
        case 0: forward(); if (now - timer > 2000) { motor_phase = 1; timer = now; } break;
        case 1: stopMotors(); if (now - timer > 1000) { motor_phase = 2; timer = now; } break;
        case 2: backward(); if (now - timer > 2000) { motor_phase = 3; timer = now; } break;
        case 3: stopMotors(); if (now - timer > 1000) { motor_phase = 0; timer = now; } break;
    }
}


void LineFollower::calibrate_sensor() {
    Serial.println("calibrate sensor");
}

/*
Write to last_sensor_input
Update new_input flag
*/
void LineFollower::read_sensor_data() {
    Serial.println("read sensor data");
    for (int i=0; i<sensor_input_count; i++) {
        if (digital){
            last_sensor_input[i] = digitalRead(sensor_pin_nrs[i]);
        } else {
            float val = analogRead(sensor_pin_nrs[i]);
            if (val > 500) {
                last_sensor_input[i] = I_WHITE;
            } else {
                last_sensor_input[i] = I_BLACK;

            }
        }
        Serial.println(i);
        Serial.println(last_sensor_input[i]);
    }

    new_input = true;

}

/*
Calculate next steer action based on current sensor readings
*/
void LineFollower::calculate_steer() {
    Serial.println("calculate steer");
    if (!new_input) {
        return;
    }

    if (last_sensor_input[0] == I_WHITE) {
        if (last_sensor_input[1] == I_WHITE) {
            // both sensors on white -> go straight
            last_steer = 0.0;
        } else {
            // right sensor on black, left on white -> turn right
            last_steer = 1.0;
        }
    } else {
        if (last_sensor_input[1] == I_WHITE) {
            // left sensor on white, right on black -> turn right
            last_steer = -1.0;
        } else {
             // both sensors on black == crossroad -> go straight
            last_steer = 0.0;
        }
    }
}
/*
Calculate steer from 3 IR sensor inputs
*/
void LineFollower::calculate_steer3() {
    Serial.println("calculate steer");
    if (!new_input) {
        return;
    }

    if (last_sensor_input[1] == I_BLACK) {
        if (last_sensor_input[0] == I_WHITE) {
            if (last_sensor_input[2] == I_WHITE) {
                last_steer = 0.0;
                return;
            } else {
                // both center and right sensor on black, slight turn right
                last_steer = 0.5;
                return;
            }
        } else {
            if (last_sensor_input[2] == I_WHITE) {
                // both center and left sensor on black, slight turn left
                last_steer = -0.5;
                return;
            } else {
                // at cross-road -> go straight
                last_steer = 0.0;
                return;
            }
        }
    } else {
        if (last_sensor_input[0] == I_BLACK) {
            // left senor on black -> steer right
            last_steer = 1.0;
            return;
        }
        if (last_sensor_input[2] == I_BLACK) {
            // right senor on black -> steer left
            last_steer = -1.0;
            return;
        }
        Serial.println("off-course: all white");
    }

}

/*
Calculate next torque action based on current sensor readings
*/
void LineFollower::calculate_throttle() {
    Serial.println("calculate throttle");

}

/*
send action to speed motors
*/
void LineFollower::control_motors() {
    Serial.println("control speed");
    if (!new_input) {
        return;
    }

    // TODO: send last_throttle to motor driver
    if (last_steer == 0.0) {
        // go forward
        last_throttle = 1.0;
        Serial.println("same speed; go straight");
        forward();
    } else {
        stopMotors();
        // turn -> slow down
        last_throttle = 0.5;
        Serial.println("slow down");
        
        if (last_steer > 0.0) {
            // turn right
            Serial.println("turn right");
            right();
        } else {
            // turn left
            Serial.println("turn left");
            left();
        }
    }
}

/*
send action to speed motors
*/
void LineFollower::motor_control_speed() {
    Serial.println("control speed");
    if (!new_input) {
        return;
    }

    if (last_steer == 0.0) {
        // go forward
        last_throttle = 1.0;
        Serial.println("same speed");
    } else {
        // turn -> slow down
        last_throttle = 0.5;
        Serial.println("slow down");
    }
}

/*
send action to steer motors
*/
void LineFollower::motor_control_steer() {
    Serial.println("control steer");
    if (!new_input) {
        return;
    }

    if (last_steer == 0.0) {
        // go straight
        Serial.println("go straight");
    } else if (last_steer > 0.0) {
        // turn right
        Serial.println("turn right");
    } else {
        // turn left
        Serial.println("turn left");
    }

    // TODO: send last_steer to motor driver

}

/*
Call all functions needed for following line
*/
void LineFollower::follow_line() {
    Serial.println("run control loop\n");
    read_sensor_data();
    calculate_steer3();
    // calculate_throttle();
    control_motors();
    // delay(5);
    // stopMotors();
    // delay(50);

    new_input = false;
    
}