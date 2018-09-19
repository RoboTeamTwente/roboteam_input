/*
 * This test checks for all possible keyboard inputs if the robotcommand is correct.
 */

#include <gtest/gtest.h>
#include <keyboard/keyboard_manager.h>
#include <SDL.h>

void simulateKeyPress(KeyboardManager * keyboard, Uint32 type, SDL_Keycode key, Uint8 repeat = 0, int amountOfPresses = 1) {
    for (int i = 0; i < amountOfPresses; i++) {
        SDL_Event test_event;
        test_event.type = type;
        test_event.key.repeat = repeat;
        test_event.key.keysym.sym = key;
        keyboard->handleSDLEvents(test_event);
    }
}

roboteam_msgs::RobotCommand getCommandForKeyPress(KeyboardManager * keyboard, Uint32 type, SDL_Keycode key, Uint8 repeat = 0, int amountOfPresses = 1) {
    simulateKeyPress(keyboard, type, key, repeat, amountOfPresses);
    return keyboard->GetRobotCommand();
}

// dribbler
TEST(KeyboardTest, It_toggles_the_dribbler) {
    KeyboardManager keyboard;
    roboteam_msgs::RobotCommand cmd;

    bool prevDribblerVal = keyboard.GetRobotCommand().dribbler;

    // toggle it so it becomes different from before
    cmd = getCommandForKeyPress(&keyboard, SDL_KEYDOWN, constants::KEY_DRIBBLE);
    EXPECT_FALSE(cmd.dribbler == prevDribblerVal);

    // this keyup event should not let the test fail
    cmd = getCommandForKeyPress(&keyboard, SDL_KEYUP, constants::KEY_DRIBBLE);
    EXPECT_FALSE(cmd.dribbler == prevDribblerVal);

    // toggle again so it is equal to prevDribblerVal again
    cmd = getCommandForKeyPress(&keyboard, SDL_KEYDOWN, constants::KEY_DRIBBLE);
    EXPECT_TRUE(cmd.dribbler == prevDribblerVal);
}

TEST(KeyboardTest, It_strafes) {
    KeyboardManager keyboard;
    roboteam_msgs::RobotCommand cmd;

    // Strafe right
    cmd = getCommandForKeyPress(&keyboard, SDL_KEYDOWN, constants::KEY_STRAFE_RIGHT);
    ASSERT_LT(cmd.y_vel, 0);

    // stop strafing right
    cmd = getCommandForKeyPress(&keyboard, SDL_KEYUP, constants::KEY_STRAFE_RIGHT);
    ASSERT_EQ(cmd.y_vel, 0);

    // strafe left
    cmd = getCommandForKeyPress(&keyboard, SDL_KEYDOWN, constants::KEY_STRAFE_LEFT);
    ASSERT_GT(cmd.y_vel, 0);

    // stop strafe left
    cmd = getCommandForKeyPress(&keyboard, SDL_KEYUP, constants::KEY_STRAFE_LEFT);
    ASSERT_EQ(cmd.y_vel, 0);
}

TEST(KeyboardTest, It_kicks) {
    KeyboardManager keyboard;
    roboteam_msgs::RobotCommand cmd;

    // do kick on keypress
    cmd = getCommandForKeyPress(&keyboard, SDL_KEYDOWN, constants::KEY_KICK);
    EXPECT_TRUE(cmd.kicker && cmd.kicker_forced);

    cmd = getCommandForKeyPress(&keyboard, SDL_KEYUP, constants::KEY_KICK);
    EXPECT_TRUE(!cmd.kicker && !cmd.kicker_forced);
}

// chipper
TEST(KeyboardTest, It_chips) {
    KeyboardManager keyboard;
    roboteam_msgs::RobotCommand cmd;

    cmd = getCommandForKeyPress(&keyboard, SDL_KEYDOWN, constants::KEY_CHIP);
    ASSERT_TRUE(cmd.chipper && cmd.chipper_forced);

    cmd = getCommandForKeyPress(&keyboard, SDL_KEYUP, constants::KEY_CHIP);
    ASSERT_TRUE(!cmd.chipper && !cmd.chipper_forced);
}

TEST(KeyboardTest, It_changes_kick_and_chip_force) {
    KeyboardManager keyboard;
    roboteam_msgs::RobotCommand cmd;

    simulateKeyPress(&keyboard, SDL_KEYDOWN, constants::KEY_INCREASE_KICK, 0, 100);
    cmd = getCommandForKeyPress(&keyboard, SDL_KEYDOWN, constants::KEY_KICK);
    EXPECT_EQ(cmd.kicker_vel, roboteam_msgs::RobotCommand::MAX_KICKER_VEL);

    simulateKeyPress(&keyboard, SDL_KEYDOWN, constants::KEY_DECREASE_KICK, 0, 100);
    cmd = keyboard.GetRobotCommand();
    EXPECT_EQ(cmd.chipper_vel, constants::MIN_KICKER_VEL);
}

// Geneva turning test and boundary check
TEST(KeyboardTest, It_turns_geneva) {
    KeyboardManager keyboard;
    roboteam_msgs::RobotCommand cmd;

    int InitialGenevaState = keyboard.GetRobotCommand().geneva_state;

    // turn left
    for (int i = 1; i <= 5; i++) {
        simulateKeyPress(&keyboard, SDL_KEYUP, constants::KEY_GENEVA_LEFT); // KEYUP must not make a difference
        cmd = getCommandForKeyPress(&keyboard, SDL_KEYDOWN, constants::KEY_GENEVA_LEFT);
        EXPECT_EQ(cmd.geneva_state, std::max(InitialGenevaState - i, 1));
    }

    // turn right
    InitialGenevaState = keyboard.GetRobotCommand().geneva_state;
    for (int i = 1; i <= 5; i++) {
        simulateKeyPress(&keyboard, SDL_KEYUP, constants::KEY_GENEVA_RIGHT); // KEYUP must not make a difference
        cmd = getCommandForKeyPress(&keyboard, SDL_KEYDOWN, constants::KEY_GENEVA_RIGHT);
        EXPECT_EQ(cmd.geneva_state, std::min(InitialGenevaState + i, 5));
    }

    // turn left but with key pressed on repeat, which shouldn't do anything.
    InitialGenevaState = cmd.geneva_state;
    cmd = getCommandForKeyPress(&keyboard, SDL_KEYDOWN, constants::KEY_GENEVA_RIGHT, 1);
    EXPECT_EQ(cmd.geneva_state, InitialGenevaState);

    cmd = getCommandForKeyPress(&keyboard, SDL_KEYDOWN, constants::KEY_GENEVA_RIGHT, 1);
    EXPECT_EQ(cmd.geneva_state, InitialGenevaState);

    cmd = getCommandForKeyPress(&keyboard, SDL_KEYDOWN, constants::KEY_GENEVA_LEFT, 1);
    EXPECT_EQ(keyboard.GetRobotCommand().geneva_state, InitialGenevaState);
}


TEST(KeyboardTest, It_rotates_robot) {
    KeyboardManager keyboard;
    roboteam_msgs::RobotCommand cmd;

    float oldAngle =  keyboard.GetRobotCommand().w;
    cmd = getCommandForKeyPress(&keyboard, SDL_KEYDOWN, SDLK_LEFT);
    EXPECT_TRUE(cmd.use_angle && cmd.w > oldAngle);
    float angleDifference = cmd.w - oldAngle;

    // Test if changing angle speed does something
    oldAngle = keyboard.GetRobotCommand().w;
    cmd = getCommandForKeyPress(&keyboard, SDL_KEYDOWN, constants::KEY_INCREASE_ROTATION_SPEED);
    float newAngleDifference = cmd.w - oldAngle;
    oldAngle = cmd.w;

    EXPECT_GT(newAngleDifference, angleDifference);

    simulateKeyPress(&keyboard, SDL_KEYDOWN, constants::KEY_DECREASE_ROTATION_SPEED);
    cmd = getCommandForKeyPress(&keyboard, SDL_KEYDOWN, SDLK_LEFT);
    newAngleDifference = cmd.w - oldAngle;
    EXPECT_NEAR(newAngleDifference, angleDifference, 0.1);

    // these are some repeated keypresses
    // Test limits
    for (int i = 0; i < 200; i++) {
        cmd = getCommandForKeyPress(&keyboard, SDL_KEYDOWN, SDLK_LEFT, 1);
        EXPECT_LT(cmd.w, 16*M_PI);
        EXPECT_GT(cmd.w, -16*M_PI);
    }

    // stop rotating on keyup
    oldAngle = keyboard.GetRobotCommand().w;
    cmd = getCommandForKeyPress(&keyboard, SDL_KEYUP, SDLK_LEFT);
    EXPECT_EQ(cmd.w, oldAngle);
}

TEST(KeyboardTest, It_drives_forward_and_backward) {
    KeyboardManager keyboard;
    roboteam_msgs::RobotCommand cmd;

    // make sure we are driving (we simulate using xvel, which is up and down.)
    cmd = getCommandForKeyPress(&keyboard, SDL_KEYDOWN, SDLK_UP);
    EXPECT_GT(cmd.x_vel, 0);
    float velocityDifference = cmd.x_vel;

    // stop pressing so the velocity should be 0 again
    cmd = getCommandForKeyPress(&keyboard, SDL_KEYUP, SDLK_UP);
    EXPECT_EQ(cmd.x_vel, 0);

    // increase speed
    simulateKeyPress(&keyboard, SDL_KEYDOWN, constants::KEY_INCREASE_VEL);

    // the speed difference should be higher
    cmd = getCommandForKeyPress(&keyboard, SDL_KEYDOWN, SDLK_UP);
    EXPECT_GT(cmd.x_vel, velocityDifference);

    // test limits
    getCommandForKeyPress(&keyboard, SDL_KEYDOWN, constants::KEY_INCREASE_VEL, 0, 100);
    cmd = getCommandForKeyPress(&keyboard, SDL_KEYDOWN, SDLK_DOWN);
    EXPECT_NEAR(std::abs(cmd.x_vel), constants::MAX_ROBOT_VELOCITY, 0.5); // TODO make this step size

    simulateKeyPress(&keyboard, SDL_KEYDOWN, constants::KEY_DECREASE_VEL, 0, 100);
    cmd = getCommandForKeyPress(&keyboard, SDL_KEYDOWN, SDLK_DOWN);
    EXPECT_EQ(std::abs(cmd.x_vel), constants::MIN_ROBOT_VELOCITY);
}




