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
    
    slow_speed_right = 130;
    slow_speed_left = 130;
    fast_speed_right = 190;
    fast_speed_left = 190;
    
    speed_right = slow_speed_left;
    speed_left = slow_speed_right;
    
    last_right_pwm = 0;
    last_left_pwm = 0;

    white_th = 0;
    black_th = 500;
    
    last_throttle = 0.0;
    last_steer = 0.0;

    Kp = 1.0;
    Kd = 0.2;

    straighten = false;

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

void LineFollower::reset_vals() {    
    
    speed_right = slow_speed_left;
    speed_left = slow_speed_right;
    
    last_right_pwm = 0;
    last_left_pwm = 0;

    white_th = 0;
    black_th = 800;
    
    last_throttle = 0.0;
    last_steer = 0.0;

    straighten = false;

    new_input = false;
    digital = false;

    off_course = false;

    sensor_input_count = 5;
    last_sensor_input = new float[sensor_input_count] {0.0, 0.0, 0.0, 0.0, 0.0}; 
}

void LineFollower::forward() {
    last_left_pwm = (last_left_pwm + fast_speed_left) / 2;
    last_right_pwm = (last_right_pwm + fast_speed_right) / 2;
    // left
    digitalWrite(IN1, HIGH); 
    digitalWrite(IN2, LOW); 
    analogWrite(ENA, last_left_pwm);

    // right
    digitalWrite(IN3, HIGH); 
    digitalWrite(IN4, LOW); 
    analogWrite(ENB, last_right_pwm);
}

void LineFollower::backward() {
    // left
    digitalWrite(IN1, LOW); 
    digitalWrite(IN2, HIGH); 
    analogWrite(ENA, last_left_pwm);
    
    // right
    digitalWrite(IN3, LOW); 
    digitalWrite(IN4, HIGH); 
    analogWrite(ENB, last_right_pwm);

    last_left_pwm = -last_left_pwm;
    last_right_pwm = -last_right_pwm;
}

void LineFollower:: back_left() {
    // last_left_pwm = int((0.2*(float)last_left_pwm + 0.8*last_throttle * (float)slow_speed_right) / 2);
    last_left_pwm = int(last_throttle * slow_speed_left);
    // left motor
    digitalWrite(IN1, HIGH); 
    digitalWrite(IN2, LOW); 
    analogWrite(ENA, last_left_pwm);
    
    // right motor
    digitalWrite(IN3, LOW); 
    digitalWrite(IN4, HIGH); 
    analogWrite(ENB, slow_speed_right);

    last_right_pwm = -slow_speed_right;
}

void LineFollower::back_right() {
    last_right_pwm = int(last_throttle * slow_speed_right);
    // left motor
    digitalWrite(IN1, LOW); 
    digitalWrite(IN2, HIGH); 
    analogWrite(ENA, slow_speed_left);
    
    // right motor
    digitalWrite(IN3, HIGH); 
    digitalWrite(IN4, LOW);
    analogWrite(ENB, last_throttle * slow_speed_right);

    last_left_pwm = -slow_speed_left;
}

void LineFollower::drive_action(float left_cmd, float right_cmd) {
    bool left_forward = (left_cmd > 0) ? true : false;
    bool right_forward = (right_cmd > 0) ? true : false;

    int left_pwm = abs(left_cmd) * speed_left;
    int right_pwm = abs(right_cmd) * speed_right;

    if (left_forward) { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); }
    else             { digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); }
    analogWrite(ENA, left_pwm);

    if (right_forward) { digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); }
    else              { digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH); }
    analogWrite(ENB, right_pwm);
    last_left_pwm = (left_forward) ? left_pwm : -left_pwm;
    last_right_pwm = (right_forward) ? right_pwm : -right_pwm;
}

void LineFollower::calculate_motor_cmd(float &left_cmd, float &right_cmd) {
    
    if (last_steer > 0.0) {
        left_cmd = 1.0;
        speed_left = fast_speed_left - 20;
        speed_right = slow_speed_right;
        
        right_cmd = 0.25 - last_steer;
    }
    
    if (last_steer < 0.0) {
        right_cmd = 1.0;
        speed_right = fast_speed_right - 20;
        speed_left = slow_speed_left;
        
        left_cmd = 0.25 + last_steer;
    }
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
            black_th = val;
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
                // continue turn after end of turn to straighten robot
                if (!straighten && abs(last_steer) > 0.2) {
                    straighten = true;
                    return;
                }

                if (straighten) {
                    last_steer *= 0.8;
                    if (abs(last_steer) < 0.05) {
                        last_steer = 0.0;
                        straighten = false;
                    }
                    return;
                }
                last_steer = 0.0;
                return;
            } else {
                // both center and right sensor on black -> turn right
                // strong turn, sensors far apart -> can't be both on line if not a turn
                last_steer = (last_steer + 1.0) / 2;
                return;
            }
        } else {
            if (last_sensor_input[id_right] == I_WHITE) {
                // both center and right sensor on white -> turn left
                // strong turn, sensors far apart -> can't be both on line if not a turn
                // last_steer = -1.0;
                last_steer = (last_steer - 1.0) / 2;
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
            if (last_steer > -0.05) {
                //start of left turn, set minimum steer
                last_steer = (last_steer - 0.05) / 2;
            } else if (last_steer > -1.0) {
                // increase left steer
                last_steer = (last_steer + last_steer - 0.15) / 2;
            }
            // else keep max left steer
            off_course = 0;
            return;
        }
        if (last_sensor_input[id_right] == I_BLACK) {
            // right senor on black -> steer right
            if (last_steer < 0.05) {
                //start of right turn, set minimum steer
                last_steer = (last_steer + 0.05) / 2;
            } else if (last_steer < 1.0) {
                // increase right steer
                last_steer = (last_steer + last_steer + 0.15) / 2;
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
    if (!new_input) {
        return;
    }
    if (is_crossroad()) {
        last_steer = 0.0;
        return;
    }

    calculate_pid_steer();
    // calculate_steer3(1, 2, 3);

    //     calculate_steer2(0, 4);
}

void LineFollower::calculate_pid_steer() {
    if (!new_input) {
        return;
    }

    float weighted_pos_sum = 0.0;
    float active_sensors = 0.0;
    float weights[5] = {-1.5, -1.0, 0.0, 1.0, 1.5};

    for (int i = 0; i < sensor_input_count; i++) {
        int black = (last_sensor_input[i] == I_BLACK) ? 1 : 0;

        if (black) {
            weighted_pos_sum += weights[i];
            active_sensors++;
        }
    }

    float curr_error = 0.0;
    if (active_sensors) {
        off_course = 0;
        curr_error = weighted_pos_sum / active_sensors;

        if (curr_error > 0) {
            last_steer = 1.0;
        } else if (curr_error < 0) {
            last_steer = -1.0;
        } else {
            last_steer = 0.0;
        }
    } else {
        off_course++;
        // if (off_course > 20) {
            if (last_steer > 0.0) {
                curr_error = -1.5;
            } else if (last_steer < 0.0) {
                curr_error = -1.5;
            } else {
                curr_error = 0; // reconsider this
            }
        // }
    }

    float P = curr_error;
    float D = curr_error - last_error;
    last_error = curr_error;

    float last_steer = Kp * P + Kd * D;

    if (last_steer > 1.0) {
        last_steer = 1.0;
    } else if (last_steer < -1.0) {
        last_steer = -1.0;
    }
}


/*
send action to speed motors
*/
void LineFollower::control_motors() {
    if (!new_input) {
        return;
    }

    last_throttle = abs(last_steer);

    // backtrack if off course for too long
    if (off_course > 60) {
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

    if (last_steer == 0.0) {
        last_throttle = 1.0;
        // go forward
        forward();
    } else {
        float left_cmd = 0.0;
        float right_cmd = 0.0;
        calculate_motor_cmd(left_cmd, right_cmd);
        drive_action(left_cmd, right_cmd);
    }
}

bool LineFollower::is_crossroad() {
    int black_det = 0;
    for (int i = 0; i < sensor_input_count; i++) {
        black_det += last_sensor_input[i];
    }

    if (black_det >= 4) {
        return true;
    }

    black_det = 0;
    black_det += last_sensor_input[1];
    black_det += last_sensor_input[2];
    black_det += last_sensor_input[3];
    if (black_det == 3) {
        return true;
    }

    if (last_steer > 0) {
        black_det = 0;
        black_det += last_sensor_input[0];
        black_det += last_sensor_input[1];
        black_det += last_sensor_input[2];
        if (black_det == 3) {
            return true;
        }
    } else if (last_steer < 0) {
        black_det = 0;
        black_det += last_sensor_input[2];
        black_det += last_sensor_input[3];
        black_det += last_sensor_input[4];
        if (black_det == 3) {
            return true;
        }
    }

    return false;
}

/*
Call all functions needed for following line
*/
void LineFollower::follow_line() {
    read_sensor_data();
    calculate_steer5();
    control_motors();

    new_input = false;
    
}