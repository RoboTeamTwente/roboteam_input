//
// Created by roboteampc on 23-11-17.
//

#ifndef ROBOTEAM_INPUT_JOYSTICK_PROFILES_H
#define ROBOTEAM_INPUT_JOYSTICK_PROFILES_H

struct joystick_profile {
    double SMOOTH_FACTOR;
    double SPEED_MULTIPLIER;
    double SPEED_MIN;
    double SPEED_MAX;

    double ROTATION_MULTIPLIER;
    double ROTATION_MIN;
};

const struct joystick_profile profile_default {
                                // VALUE RANGE      - DESCRIPTION
    .SMOOTH_FACTOR = 0.3,       // (0, 1]           - Influences acceleration / deceleration smoothing. 0 = no movement. 1 = no smoothing
    .SPEED_MULTIPLIER = 1,      // (0, ->)          - Influences acceleration / deceleration and max speed. 0 = no movement. 1 = no influence.
    .SPEED_MIN = 0.3,           // (0, SPEED_MAX)   - Sets minimum speed. speed < SPEED_MIN = no movement
    .SPEED_MAX = 2,             // (SPEED_MIN, ->)  - Sets maximum speed. speed > SPEED_MAX = SPEED_MAX
    .ROTATION_MULTIPLIER = 4,   // (0, ->)          - Influences rotational acceleration / deceleration and max rotation speed. 0 = no movement. 1 = no influence
    .ROTATION_MIN = 1.5         // (0, ->)          - Sets minimum rotation speed. speed < ROTATION_MIN = no rotation
};

const struct joystick_profile profile_slow {
    .SMOOTH_FACTOR = 1,
    .SPEED_MULTIPLIER = 1,
    .SPEED_MIN = 0.3,
    .SPEED_MAX = 0.6,
    .ROTATION_MULTIPLIER = 3,
    .ROTATION_MIN = 1
};

const struct joystick_profile profile_quick {
    .SMOOTH_FACTOR = 0.08,
    .SPEED_MULTIPLIER = 2,
    .SPEED_MIN = 0.3,
    .SPEED_MAX = 4,
    .ROTATION_MULTIPLIER = 7,
    .ROTATION_MIN = 1.5
};

#define NUM_JOYSTICK_PROFILES 3;
joystick_profile joystick_profiles[3] = { profile_slow, profile_default, profile_quick };

#endif //ROBOTEAM_INPUT_JOYSTICK_PROFILES_H
