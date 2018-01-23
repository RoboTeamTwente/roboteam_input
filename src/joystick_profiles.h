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
    .SMOOTH_FACTOR = 0.3,
    .SPEED_MULTIPLIER = 1,
    .SPEED_MIN = 0.5,
    .SPEED_MAX = 2,
    .ROTATION_MULTIPLIER = 5,
    .ROTATION_MIN = 1.5
};

const struct joystick_profile profile_slow {
    .SMOOTH_FACTOR = 1,
    .SPEED_MULTIPLIER = 1,
    .SPEED_MIN = 0.5,
    .SPEED_MAX = 0.6,
    .ROTATION_MULTIPLIER = 3,
    .ROTATION_MIN = 1
};

const struct joystick_profile profile_quick {
    .SMOOTH_FACTOR = 0.08,
    .SPEED_MULTIPLIER = 2,
    .SPEED_MIN = 0.5,
    .SPEED_MAX = 4,
    .ROTATION_MULTIPLIER = 7,
    .ROTATION_MIN = 1.5
};

#define NUM_JOYSTICK_PROFILES 3;
joystick_profile joystick_profiles[3] = { profile_default, profile_slow, profile_quick };

#endif //ROBOTEAM_INPUT_JOYSTICK_PROFILES_H
