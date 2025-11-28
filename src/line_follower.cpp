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
    
    slow_speed_right = 50;
    slow_speed_left = 50;
    
    last_throttle = 0.0;
    last_steer = 0.0;
    new_input = false;
    digital = false;

    off_course = false;
    invert_steer = true; // set to true if turns are mirrored

    // 5 sensors: LEFT to RIGHT
    // I_IR4(A1)=left, I_IR3(A0)=little left, I_IR2(A6)=middle, I_IR1(A5)=little right, I_IR0(A4)=right
    sensor_input_count = 5;
    sensor_pin_nrs = new int[sensor_input_count] {I_IR4, I_IR3, I_IR2, I_IR1, I_IR0};
    last_sensor_input = new float[sensor_input_count] {0.0, 0.0, 0.0, 0.0, 0.0};

    // allocate adaptive min/max arrays for each sensor
    sensor_min = new int[sensor_input_count];
    sensor_max = new int[sensor_input_count];
    for (int i=0; i<sensor_input_count; i++) {
        sensor_min[i] = 1023; // large initial min
        sensor_max[i] = 0;    // small initial max
    }

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
    analogWrite(ENA, slow_speed_left);

    // right
    digitalWrite(IN3, HIGH); 
    digitalWrite(IN4, LOW); 
    analogWrite(ENB, slow_speed_right);
}

void LineFollower::backward() {
    // left
    digitalWrite(IN1, LOW); 
    digitalWrite(IN2, HIGH); 
    analogWrite(ENA, slow_speed_left);
    
    // right
    digitalWrite(IN3, LOW); 
    digitalWrite(IN4, HIGH); 
    analogWrite(ENB, slow_speed_right);
}

void LineFollower::left() {
    // left motor
    digitalWrite(IN1, LOW); 
    digitalWrite(IN2, HIGH); 
    analogWrite(ENA, slow_speed_left);
    
    // right motor
    digitalWrite(IN3, HIGH); 
    digitalWrite(IN4, LOW); 
    analogWrite(ENB, slow_speed_right);
}

void LineFollower::right() {
    // left motor
    digitalWrite(IN1, HIGH); 
    digitalWrite(IN2, LOW); 
    analogWrite(ENA, slow_speed_left);
    
    // right motor
    digitalWrite(IN3, LOW); 
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, slow_speed_right);
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
    // Serial.println("read sensor data");
    for (int i=0; i<sensor_input_count; i++) {
        if (digital){
            last_sensor_input[i] = digitalRead(sensor_pin_nrs[i]);
        } else {
            // Diagnostic-friendly analog read
            // ADJUST black_th to match your sensor calibration
            float val = analogRead(sensor_pin_nrs[i]);
            // update adaptive min/max
            int v = (int)val;
            if (v < sensor_min[i]) sensor_min[i] = v;
            if (v > sensor_max[i]) sensor_max[i] = v;
            int thresh = (sensor_min[i] + sensor_max[i]) / 2;

            // Print raw values and current threshold for debugging
            Serial.print("S"); Serial.print(i); Serial.print(": "); Serial.print(v);
            Serial.print(" th="); Serial.print(thresh);
            if (v > thresh) {
                last_sensor_input[i] = I_BLACK;
                Serial.println(" -> BLACK");
            } else {
                last_sensor_input[i] = I_WHITE;
                Serial.println(" -> WHITE");
            }
        }

        // Serial.println(i);
        // Serial.println(last_sensor_input[i]);
    }

    new_input = true;

}

/*
Calculate next steer action based on current sensor readings
*/
void LineFollower::calculate_steer2(int id_left, int id_right) {
    Serial.println("calculate steer");
    if (!new_input) {
        return;
    }

    if (last_sensor_input[id_left] == I_WHITE) {
        if (last_sensor_input[id_right] == I_WHITE) {
            // both sensors on white -> go straight
            last_steer = 0.0;
        } else {
            // right sensor on black, left on white -> turn right
            last_steer = 1.0;
        }
    } else {
        if (last_sensor_input[id_right] == I_WHITE) {
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
void LineFollower::calculate_steer3() {
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
                last_steer = 1;
                return;
            }
        } else {
            if (last_sensor_input[2] == I_WHITE) {
                // both center and right sensor on white, slight turn left
                last_steer = -1;
                return;
            } else {
                // at cross-road -> go straight
                last_steer = 0.0;
                return;
            }
        }
    } else {
        if (last_sensor_input[0] == I_BLACK) {
            // left senor on black -> steer left
            last_steer = -0.5;
            return;
        }
        if (last_sensor_input[2] == I_BLACK) {
            // right senor on black -> steer right
            last_steer = 0.5;
            return;
        }
        // Serial.println("off-course: all white");
        off_course = true;
    }

}

/*
Calculate steer from 5 IR sensor inputs
Sensor indices: 0=LEFT, 1=little left, 2=MIDDLE, 3=little right, 4=RIGHT
*/
void LineFollower::calculate_steer5() {
    if (!new_input) {
        return;
    }

    // Check middle sensor first
    if (last_sensor_input[2] == I_BLACK) {
        // Middle on line - check if we need slight correction
        if (last_sensor_input[1] == I_BLACK && last_sensor_input[3] == I_WHITE) {
            // Middle + little left on black -> slight turn left
            last_steer = -0.5;
        } else if (last_sensor_input[3] == I_BLACK && last_sensor_input[1] == I_WHITE) {
            // Middle + little right on black -> slight turn right
            last_steer = 0.5;
        } else {
            // Centered or crossroad -> go straight
            last_steer = 0.0;
        }
        return;
    }
    
    // Middle is WHITE - check inner sensors (little left/right)
    if (last_sensor_input[1] == I_BLACK) {
        // Little left sees line -> turn left
        last_steer = -1.0;
        return;
    }
    if (last_sensor_input[3] == I_BLACK) {
        // Little right sees line -> turn right
        last_steer = 1.0;
        return;
    }
    
    // Inner sensors all WHITE - check outer sensors (hard turn needed)
    if (last_sensor_input[0] == I_BLACK) {
        // Far LEFT sees line -> hard turn left
        last_steer = -2.0;
        return;
    }
    if (last_sensor_input[4] == I_BLACK) {
        // Far RIGHT sees line -> hard turn right
        last_steer = 2.0;
        return;
    }
    
    // All sensors WHITE - off course, keep last direction
    off_course = true;
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
    if (!new_input) {
        return;
    }

    // DEBUG: print chosen steer value
    Serial.print("last_steer: "); Serial.println(last_steer);

    // Apply invert flag (useful if sensors/motors are mirrored)
    float eff_steer = invert_steer ? -last_steer : last_steer;
    Serial.print("eff_steer: "); Serial.println(eff_steer);

    if (eff_steer == 0.0) {
        // Go straight
        slow_speed_left = 60;
        slow_speed_right = 60;
        forward();
    } else if (eff_steer == 0.5) {
        // Slight right - slow down right wheel
        slow_speed_left = 60;
        slow_speed_right = 40;
        forward();
    } else if (eff_steer == -0.5) {
        // Slight left - slow down left wheel
        slow_speed_left = 40;
        slow_speed_right = 60;
        forward();
    } else if (eff_steer == 1.0) {
        // Turn right
        slow_speed_left = 55;
        slow_speed_right = 55;
        right();
    } else if (eff_steer == -1.0) {
        // Turn left
        slow_speed_left = 55;
        slow_speed_right = 55;
        left();
    } else if (eff_steer >= 2.0) {
        // HARD turn right (outer sensor)
        slow_speed_left = 65;
        slow_speed_right = 65;
        right();
    } else if (last_steer <= -2.0) {
        // HARD turn left (outer sensor)
        slow_speed_left = 65;
        slow_speed_right = 65;
        left();
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
}

/*
Call all functions needed for following line
*/
void LineFollower::follow_line() {
    read_sensor_data();
    calculate_steer5();
    control_motors();

    new_input = false;
    
    // LOOP DELAY - controls how often sensors are checked
    // Increase this if robot overshoots turns (try 20, 30, 50)
    delay(15);
}