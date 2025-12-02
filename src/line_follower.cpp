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
    slow_speed_left = 100;

    white_th = 0;
    black_th = 800;
    
    last_throttle = 0.0;
    last_steer = 0.0;
    new_input = false;
    digital = false;

    off_course = false;

    sensor_input_count = 5;
    sensor_pin_nrs = new int[sensor_input_count] {I_IR0, I_IR1, I_IR2, I_IR3, I_IR4}; // left to right
    last_sensor_input = new float[sensor_input_count] {0.0, 0.0, 0.0, 0.0, 0.0};

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
    digitalWrite(IN1, HIGH); 
    digitalWrite(IN2, LOW); 
    analogWrite(ENA, slow_speed_right);

    // right
    digitalWrite(IN3, HIGH); 
    digitalWrite(IN4, LOW); 
    analogWrite(ENB, slow_speed_left);
}

void LineFollower::backward() {
    // left
    digitalWrite(IN1, LOW); 
    digitalWrite(IN2, HIGH); 
    analogWrite(ENA, slow_speed_right);
    
    // right
    digitalWrite(IN3, LOW); 
    digitalWrite(IN4, HIGH); 
    analogWrite(ENB, slow_speed_left);
}

void LineFollower::left() {
    // left motor
    digitalWrite(IN1, LOW); 
    digitalWrite(IN2, HIGH); 
    analogWrite(ENA, last_throttle * slow_speed_right);
    
    // right motor
    digitalWrite(IN3, HIGH); 
    digitalWrite(IN4, LOW); 
    analogWrite(ENB, slow_speed_right);
}

void LineFollower::right() {
    // left motor
    digitalWrite(IN1, HIGH); 
    digitalWrite(IN2, LOW); 
    analogWrite(ENA, slow_speed_right);
    
    // right motor
    digitalWrite(IN3, LOW); 
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, last_throttle * slow_speed_left);
}


void LineFollower:: back_left() {
    // left motor
    digitalWrite(IN1, HIGH); 
    digitalWrite(IN2, LOW); 
    analogWrite(ENA, last_throttle * slow_speed_right);
    
    // right motor
    digitalWrite(IN3, LOW); 
    digitalWrite(IN4, HIGH); 
    analogWrite(ENB, slow_speed_right);
}

void LineFollower::back_right() {
    // left motor
    digitalWrite(IN1, LOW); 
    digitalWrite(IN2, HIGH); 
    analogWrite(ENA, slow_speed_right);
    
    // right motor
    digitalWrite(IN3, HIGH); 
    digitalWrite(IN4, LOW);
    analogWrite(ENB, last_throttle * slow_speed_left);
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
    white_th = 0;
    black_th = 0;
    for (int i=0; i<sensor_input_count; i++) {
        float val = analogRead(sensor_pin_nrs[i]);
        if (i != 2) {
            white_th += 0.25 * val;
        } else {
            Serial.print("Sensor "); Serial.print(i); Serial.print(": "); Serial.println(val);
            black_th = 800;;
        }
    }

    Serial.print("Calibrated white th: "); Serial.println(white_th);
    Serial.print("Calibrated black th: "); Serial.println(black_th);
}

/*
Write to last_sensor_input
Update new_input flag
*/
void LineFollower::read_sensor_data() {
    // Serial.println("read sensor data");
    for (int i=0; i<sensor_input_count; i++) {
        if (digital){
            last_sensor_input[i] = digitalRead(sensor_pin_nrs[i]);
        } else {
            float val = analogRead(sensor_pin_nrs[i]);
            if (val < black_th) {
                last_sensor_input[i] = I_WHITE;
            } else {
                last_sensor_input[i] = I_BLACK;
            }
        }
    }
    new_input = true;
}

/*
Calculate next steer action based on current sensor readings
*/
void LineFollower::calculate_steer2(int id_left, int id_right) {
    // Serial.println("calculate steer");
    if (!new_input) {
        return;
    }

    if (last_sensor_input[id_left] == I_WHITE) {
        if (last_sensor_input[id_right] == I_WHITE) {
            if (off_course > 0) {
                // both sensors on white -> was off-course, try to turn back
                // last_steer = 100;
                return;
            }
            // else -> keep decision of inner sensors
        } else {
            off_course = 0;
            if (last_sensor_input[id_left+1] == I_BLACK) {
                // left sensor on white, left inner sensor on black, right sensor on black 
                // -> slight turn right, could be crossroad
                last_steer = 0.10;
                return;
            }
            // right sensor on black, left on white -> turn right
            // if outer sensor on black -> strong turn
            last_steer = 1.0;
        }
    } else {
        off_course = 0;
        if (last_sensor_input[id_right] == I_WHITE) {
            if (last_sensor_input[id_right-1] == I_BLACK) {
                // right sensor on white, right inner sensor on black, left sensor on black 
                // -> slight turn left, could be crossroad
                last_steer = -0.10;
                return;
            }
            // right sensor on white, left on black -> turn left
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
void LineFollower::calculate_steer3(int id_left, int id_center, int id_right) {
    if (!new_input) {
        return;
    }

    if (last_sensor_input[id_center] == I_BLACK) {
        off_course = 0;
        if (last_sensor_input[id_left] == I_WHITE) {
            if (last_sensor_input[id_right] == I_WHITE) {
                last_steer = 0.0;
                return;
            } else {
                // both center and right sensor on black -> turn right
                // strong turn, sensors far apart -> can't be both on line if not a turn
                last_steer = 1.0;
                return;
            }
        } else {
            if (last_sensor_input[id_right] == I_WHITE) {
                // both center and right sensor on white -> turn left
                // strong turn, sensors far apart -> can't be both on line if not a turn
                last_steer = -1.0;
                return;
            } else {
                // at cross-road -> go straight
                last_steer = 0.0;
                return;
            }
        }
    } else {
        if (last_sensor_input[id_left] == I_BLACK) {
            // left senor on black -> steer left
            if (last_steer > -0.10) {
                //start of left turn, set minimum steer
                last_steer = -0.10;
            } else if (last_steer > -1.0) {
                // increase left steer
                last_steer -= 0.05;
            }
            // else keep max left steer
            off_course = 0;
            return;
        }
        if (last_sensor_input[id_right] == I_BLACK) {
            // right senor on black -> steer right
            if (last_steer < 0.10) {
                //start of right turn, set minimum steer
                last_steer = 0.10;
            } else if (last_steer < 1.0) {
                // increase right steer
                last_steer += 0.05;
            }
            // else keep max right steer
            off_course = 0;
            return;
        }
        // Serial.println("off-course: all white");
        off_course += 1;
    }

}

void LineFollower::calculate_steer5() {
    // Serial.println("calculate steer");
    if (!new_input) {
        return;
    }
    calculate_steer3(1, 2, 3);

    if (off_course) {
        calculate_steer2(0, 4);
    }
}

// /*
// Calculate next torque action based on current sensor readings
// */
// void LineFollower::calculate_throttle() {
//     // Serial.println("calculate throttle");
//     last_throttle = abs(last_steer);
// }

/*
send action to speed motors
*/
void LineFollower::control_motors() {
    if (!new_input) {
        return;
    }

    last_throttle = abs(last_steer);

    // backtrack if off course for too long
    if (off_course > 175) {
        stopMotors();
        if (last_steer == 0.0) {
            backward();
        }

        if (last_steer > 0.0) {
            back_left();
        } else if (last_steer < 0.0) {
            back_right();
        }
        return;
    }

    // TODO: send last_throttle to motor driver
    if (last_steer == 0.0) {
        // go forward
        
        Serial.println("same speed; go straight");
        forward();
    } else {
        
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

// /*
// send action to speed motors
// */
// void LineFollower::motor_control_speed() {
//     Serial.println("control speed");
//     if (!new_input) {
//         return;
//     }

//     if (last_steer == 0.0) {
//         // go forward
//         last_throttle = 1.0;
//         // Serial.println("same speed");
//     } else {
//         // turn -> slow down
//         last_throttle = 0.5;
//         // Serial.println("slow down");
//     }
// }

// /*
// send action to steer motors
// */
// void LineFollower::motor_control_steer() {
//     Serial.println("control steer");
//     if (!new_input) {
//         return;
//     }

//     if (last_steer == 0.0) {
//         // go straight
//         Serial.println("go straight");
//     } else if (last_steer > 0.0) {
//         // turn right
//         Serial.println("turn right");
//     } else {
//         // turn left
//         Serial.println("turn left");
//     }
// }

/*
Call all functions needed for following line
*/
void LineFollower::follow_line() {
    read_sensor_data();
    calculate_steer5();
    // calculate_throttle();
    control_motors();
    // delay(5);
    // stopMotors();
    // delay(5);

    new_input = false;
    
}