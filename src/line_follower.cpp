#include "Arduino.h"
#include "line_follower.hpp"
#include "pin_setup.hpp"

LineFollower::LineFollower() {
    now = millis();
    motor_phase = 0;
   
    // slow speed used for turns
    slow_speed_right = 40;
    slow_speed_left = 40;
    // max speed
    fast_speed_right = 100;
    fast_speed_left = 100;
    // current speed
    speed_right = slow_speed_left;
    speed_left = slow_speed_right;
    
    // target motor action
    target_left = 0.0;
    target_right = 0.0;
    // current motor action
    curr_left = 0.0;
    curr_right = 0.0;
    // smoothing parameter for calculating next motor action
    alpha = 0.8235003;

    white_th = 0;
    black_th = 500;
    
    last_steer = 0.0;

    // PID parameters
    Kp = 1.11413;
    Kd = 0.45122;

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

    curr_left = 0.0;
    curr_right = 0.0;
    target_left = 0.0;
    target_right = 0.0;
    
    last_steer = 0.0;

    straighten = false;
    new_input = false;
    off_course = 0;
}

// set motor action target intesity, direction and max speed for forward drive
void LineFollower::forward() {
    target_left = 1.0;
    target_right = 1.0;

    speed_left = fast_speed_left;
    speed_right = fast_speed_right;
}

// set motor action target intesity, direction and max speed for backward drive
void LineFollower::backward() {
    target_left = -1.0;
    target_right = -1.0;

    speed_left = slow_speed_left;
    speed_right = slow_speed_right;
}

// set motor action target intesity, direction and max speed for backward left drive
void LineFollower:: back_left() {
    target_left = -1.0;
    target_right = -1.0;

    speed_left = (int)(slow_speed_left*abs(last_steer));
    speed_right = slow_speed_right;

}

// set motor action target intesity, direction and max speed for backward right drive
void LineFollower::back_right() {
    target_left = -1.0;
    target_right = -1.0;

    speed_left = slow_speed_left;
    speed_right = (int)(slow_speed_right*abs(last_steer));

}

// calculate motor action target intesity, direction and max speed for steer actions
// based on last_steer value
void LineFollower::calculate_motor_cmd(float &left_cmd, float &right_cmd) {
    // right turns
    if (last_steer > 0.0) {
        left_cmd = 1.0;
        speed_left = fast_speed_left - 20;
        speed_right = slow_speed_right;
        right_cmd = 0.25 - last_steer;
    }
    
    // left turns
    if (last_steer < 0.0) {
        right_cmd = 1.0;
        speed_right = fast_speed_right - 20;
        speed_left = slow_speed_left;
        left_cmd = 0.25 + last_steer;
    }
}

// STOP motor action 
void LineFollower::stopMotors() {
  digitalWrite(IN1, LOW); 
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); 
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0); 
  analogWrite(ENB, 0);
}

// update motor physics based on target and current motor commands
void LineFollower::update_motor_physics() {
    // weighted average smoothing on spped fluctuations.
    curr_left = (target_left * alpha) + (curr_left * (1.0 - alpha));
    curr_right = (target_right * alpha) + (curr_right * (1.0 - alpha));

    // convert to [-255; 255]
    int left_pwm = (int)(curr_left * speed_left); 
    int right_pwm = (int)(curr_right * speed_right);

    apply_motor_action(left_pwm, right_pwm);
}

// apply provided left and right pwm to motors
void LineFollower::apply_motor_action(int left_pwm, int right_pwm) {
// --- Left Motor ---
    if (left_pwm > 0) {
        // forward action
        digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    } else if (left_pwm < 0) {
        // backward action
        digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
    } else {
        // stop
        digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
    }
    analogWrite(ENA, abs(left_pwm));

    // --- Right Motor ---
    if (right_pwm > 0) {
        // forward action
        digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
    } else if (right_pwm < 0) {
        // backward action
        digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
    } else {
        // stop
        digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
    }
    analogWrite(ENB, abs(right_pwm));
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
/*
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
*/
/*
Calculate steer from 3 IR sensor inputs
*/
/*
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
*/

void LineFollower::calculate_steer5() {
    if (!new_input) {
        return;
    }
    
    if (is_crossroad()) {
        last_steer = 0.0;
        return;
    }

    calculate_pid_steer();
}

void LineFollower::calculate_pid_steer() {
    if (!new_input) {
        return;
    }

    float weighted_pos_sum = 0.0;
    float active_sensors = 0.0;
    float weights[5] = {-1.5, -1.0, 0.0, 1.0, 1.5};

    // calculate weighted sum and count active sensors
    for (int i = 0; i < sensor_input_count; i++) {
        int black = (last_sensor_input[i] == I_BLACK) ? 1 : 0;

        if (black) {
            weighted_pos_sum += weights[i];
            active_sensors++;
        }
    }

    // calculate current error based on weighted sum and active sensor count
    float curr_error = 0.0;
    if (active_sensors) {
        off_course = 0;
        curr_error = weighted_pos_sum / active_sensors;
    } else {
        // set high current error when off track
        off_course++;
            if (last_steer > 0.0) {
                curr_error = 1.5;
            } else if (last_steer < 0.0) {
                curr_error = -1.5;
            } else {
                curr_error = 0; // reconsider this
            }
    }

    float P = curr_error;
    float D = curr_error - last_error;
    last_error = curr_error;

    // calculate steer with pid
    last_steer = Kp * P + Kd * D;

    // cut-off steer values
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

    // backtrack if off course for too long
    if (off_course > 200) {
        // stopMotors();
        // if (last_steer == 0.0) {
            backward();
        if (last_steer > 0.0) {
            back_right();
        } else if (last_steer < 0.0) {
            back_left();
        }
    } else {
        if (last_steer == 0.0) {
            // go forward
            forward();
        } else {
            calculate_motor_cmd(target_left, target_right);
        }
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
    // get new sensor input
    read_sensor_data();
    // calculate new steer based on sensor input
    calculate_steer5();
    // calculate target motor actions and speeds (pwm)
    control_motors();
    // calculate and send action to motors
    update_motor_physics();

    new_input = false;
    
}