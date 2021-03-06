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
    .ROTATION_MULTIPLIER = 0.1,   // Mutliplier      - Sets rotation speed/multiplier for relative mode
};

const struct joystick_profile profile_children {
		// VALUE RANGE      - DESCRIPTION
		.SPEED_MIN = 0.3,           // (0, SPEED_MAX)   - Sets minimum speed. speed < SPEED_MIN = no movement
		.SPEED_MAX = 1.3,             // (SPEED_MIN, ->)  - Sets maximum speed. speed > SPEED_MAX = SPEED_MAX
		.DESCRIPTION = "CHILDREN",
		.ROTATION_MULTIPLIER = 0.1,   // Mutliplier      - Sets rotation speed/multiplier for relative mode
};

const struct joystick_profile profile_slow {
    .SPEED_MIN = 0.3,
    .SPEED_MAX = 1.0,
	.DESCRIPTION = "SLOW",
    .ROTATION_MULTIPLIER = 0.05, // Mutliplier      - Sets rotation speed/multiplier for relative mode
};

const struct joystick_profile profile_quick {
    .SPEED_MIN = 0.3,
    .SPEED_MAX = 5.6,
	.DESCRIPTION = "QUICK",
    .ROTATION_MULTIPLIER = 0.15,   // Mutliplier      - Sets rotation speed/multiplier for relative mode
};

#define NUM_JOYSTICK_PROFILES 4

#endif //ROBOTEAM_INPUT_JOYSTICK_PROFILES_H
