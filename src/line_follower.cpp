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
    last_throttle = 0.0;
    last_steer = 0.0;
    new_input = false;
    sensor_input_count = 2;
    sensor_pin_nrs = new int[sensor_input_count] {I_IR0, I_IR1};
    last_sensor_input = new float[sensor_input_count] {0.0, 0.0};

    for (int i=0; i<sensor_input_count; i++) {
        pinMode(sensor_pin_nrs[i], INPUT);
    }

    steer_pin_nr = O_STEER;
    pinMode(steer_pin_nr, OUTPUT);
    throttle_pin_nrs = new uint8_t[1] {O_THROTTLE};
    for (int i=0; i<1; i++)
        pinMode(throttle_pin_nrs[i], OUTPUT);
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
        last_sensor_input[i] = digitalRead(sensor_pin_nrs[i]);
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
    } else {
        // turn -> slow down
        last_throttle = 0.5;
        Serial.println("slow down");
        
        if (last_steer > 0.0) {
            // turn right
            Serial.println("turn right");
            // TODO: throttle = 1 to right motor, <=0.5 to left motor
        } else {
            // turn left
            Serial.println("turn left");
            // TODO: throttle = 1 to left motor, <=0.5 to right motor
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

    // TODO: send last_throttle to motor driver

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
    calculate_steer();
    calculate_throttle();
    control_motors();
    // motor_control_speed();
    // motor_control_steer();

    new_input = false;
    
}