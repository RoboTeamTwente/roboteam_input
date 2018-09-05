//
// Created by roboteampc on 23-11-17.
//

#ifndef ROBOTEAM_INPUT_JOYSTICK_PROFILES_H
#define ROBOTEAM_INPUT_JOYSTICK_PROFILES_H

struct joystick_profile {
    double SPEED_MIN;
	double SPEED_MAX;
	std::string DESCRIPTION;

	double ROTATION_MULTIPLIER;
	double ROTATION_MIN;
};

const struct joystick_profile profile_default {
                                // VALUE RANGE      - DESCRIPTION
    .SPEED_MIN = 0.3,           // (0, SPEED_MAX)   - Sets minimum speed. speed < SPEED_MIN = no movement
    .SPEED_MAX = 2,             // (SPEED_MIN, ->)  - Sets maximum speed. speed > SPEED_MAX = SPEED_MAX
	.DESCRIPTION = "DEFAULT",
    .ROTATION_MULTIPLIER = 1,   // Mutliplier      - Sets rotation speed/multiplier for relative mode
};

const struct joystick_profile profile_slow {
    .SPEED_MIN = 0.3,
    .SPEED_MAX = 1.0,
	.DESCRIPTION = "SLOW",
    .ROTATION_MULTIPLIER = 0.5, // Mutliplier      - Sets rotation speed/multiplier for relative mode
};

const struct joystick_profile profile_quick {
    .SPEED_MIN = 0.3,
    .SPEED_MAX = 5.6,
	.DESCRIPTION = "QUICK",
    .ROTATION_MULTIPLIER = 1.5,   // Mutliplier      - Sets rotation speed/multiplier for relative mode
};

#define NUM_JOYSTICK_PROFILES 3;
joystick_profile joystick_profiles[3] = { profile_default, profile_slow, profile_quick };

#endif //ROBOTEAM_INPUT_JOYSTICK_PROFILES_H
