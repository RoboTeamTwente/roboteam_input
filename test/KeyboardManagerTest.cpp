#include <gtest/gtest.h>
#include <keyboard/keyboard_manager.h>
#include <SDL.h>

KeyboardManager keyboard;
roboteam_msgs::RobotCommand cmd;

void generateKeyPress(Uint32 type, SDL_Keycode key, Uint8 repeat = 0) {
    SDL_Event test_event;
    test_event.type = type;

    // please note that repeat is an optional parameter
    test_event.key.repeat = repeat;
    test_event.key.keysym.sym = key;
    keyboard.handleSDLEvents(test_event);
}

// dribbler
TEST(KeyboardTest, It_toggles_the_dribbler) {
    bool prevDribblerVal = keyboard.GetRobotCommand().dribbler;

    // toggle it so it becomes different from before
    generateKeyPress(SDL_KEYDOWN, constants::KEY_DRIBBLE);
    EXPECT_FALSE(keyboard.GetRobotCommand().dribbler == prevDribblerVal);

    // this keyup event should not let the test fail
    generateKeyPress(SDL_KEYUP, constants::KEY_DRIBBLE);
    EXPECT_FALSE(keyboard.GetRobotCommand().dribbler == prevDribblerVal);

    // toggle again so it is equal to prevDribblerVal again
    generateKeyPress(SDL_KEYDOWN, constants::KEY_DRIBBLE);
    EXPECT_TRUE(keyboard.GetRobotCommand().dribbler == prevDribblerVal);
}

TEST(KeyboardTest, It_strafes) {
    // Strafe right
    generateKeyPress(SDL_KEYDOWN, constants::KEY_STRAFE_RIGHT);
    ASSERT_LT(keyboard.GetRobotCommand().y_vel, 0);

    // stop strafing right
    generateKeyPress(SDL_KEYUP, constants::KEY_STRAFE_RIGHT);
    ASSERT_EQ(keyboard.GetRobotCommand().y_vel, 0);

    // strafe left
    generateKeyPress(SDL_KEYDOWN, constants::KEY_STRAFE_LEFT);
    ASSERT_GT(keyboard.GetRobotCommand().y_vel, 0);

    // stop strafe left
    generateKeyPress(SDL_KEYUP, constants::KEY_STRAFE_LEFT);
    ASSERT_EQ(keyboard.GetRobotCommand().y_vel, 0);
}

// kicker
TEST(KeyboardTest, It_kicks) {
    generateKeyPress(SDL_KEYDOWN, constants::KEY_KICK);
    cmd = keyboard.GetRobotCommand();
    EXPECT_TRUE(cmd.kicker && cmd.kicker_forced);

    generateKeyPress(SDL_KEYUP, constants::KEY_KICK);
    cmd = keyboard.GetRobotCommand();
    EXPECT_TRUE(!cmd.kicker && !cmd.kicker_forced);
}

// chipper
TEST(KeyboardTest, It_chips) {
    generateKeyPress(SDL_KEYDOWN, constants::KEY_CHIP);
    cmd = keyboard.GetRobotCommand();
    ASSERT_TRUE(cmd.chipper && cmd.chipper_forced);

    generateKeyPress(SDL_KEYUP, constants::KEY_CHIP);
    cmd = keyboard.GetRobotCommand();
    ASSERT_TRUE(!cmd.chipper && !cmd.chipper_forced);
}

TEST(KeyboardTest, It_changes_kick_and_chip_force) {

    for (int i = 0; i < 100; i++) {
        generateKeyPress(SDL_KEYDOWN, constants::KEY_INCREASE_KICK);
    }

    generateKeyPress(SDL_KEYDOWN, constants::KEY_KICK);
    cmd = keyboard.GetRobotCommand();
    EXPECT_EQ(cmd.kicker_vel, roboteam_msgs::RobotCommand::MAX_KICKER_VEL);

    for (int i = 0; i < 100; i++) {
        generateKeyPress(SDL_KEYDOWN, constants::KEY_DECREASE_KICK);
    }

    cmd = keyboard.GetRobotCommand();
    EXPECT_EQ(cmd.chipper_vel, constants::MIN_KICKER_VEL);
}

// Geneva turning test and boundary check
TEST(KeyboardTest, It_turns_geneva) {
    int InitialGenevaState = keyboard.GetRobotCommand().geneva_state;

    // turn left
    for (int i = 1; i <= 5; i++) {
        generateKeyPress(SDL_KEYUP, constants::KEY_GENEVA_LEFT); // KEYUP must not make a difference
        generateKeyPress(SDL_KEYDOWN, constants::KEY_GENEVA_LEFT);
        int genevaState = keyboard.GetRobotCommand().geneva_state;
        EXPECT_EQ(genevaState, std::max(InitialGenevaState - i, 1));
    }

    // turn right
    InitialGenevaState = keyboard.GetRobotCommand().geneva_state;
    for (int i = 1; i <= 5; i++) {
        generateKeyPress(SDL_KEYUP, constants::KEY_GENEVA_RIGHT); // KEYUP must not make a difference
        generateKeyPress(SDL_KEYDOWN, constants::KEY_GENEVA_RIGHT);
        int genevaState = keyboard.GetRobotCommand().geneva_state;
        EXPECT_EQ(genevaState, std::min(InitialGenevaState + i, 5));
    }

    // turn left but with key pressed on repeat, which shouldn't do anything.
    InitialGenevaState = keyboard.GetRobotCommand().geneva_state;
    generateKeyPress(SDL_KEYDOWN, constants::KEY_GENEVA_RIGHT, 1);
    EXPECT_EQ(keyboard.GetRobotCommand().geneva_state, InitialGenevaState);

    generateKeyPress(SDL_KEYDOWN, constants::KEY_GENEVA_RIGHT, 1);
    EXPECT_EQ(keyboard.GetRobotCommand().geneva_state, InitialGenevaState);

    generateKeyPress(SDL_KEYDOWN, constants::KEY_GENEVA_LEFT, 1);
    EXPECT_EQ(keyboard.GetRobotCommand().geneva_state, InitialGenevaState);
}


TEST(KeyboardTest, It_rotates_robot) {
    float oldAngle =  keyboard.GetRobotCommand().w;
    generateKeyPress(SDL_KEYDOWN, SDLK_LEFT);
    cmd = keyboard.GetRobotCommand();
    EXPECT_TRUE(cmd.use_angle && cmd.w > oldAngle);
    float angleDifference = cmd.w - oldAngle;

    // Test if changing angle speed does something
    oldAngle = keyboard.GetRobotCommand().w;
    generateKeyPress(SDL_KEYDOWN, constants::KEY_INCREASE_ROTATION_SPEED);
    cmd = keyboard.GetRobotCommand();
    float newAngleDifference = cmd.w - oldAngle;
    oldAngle = cmd.w;

    EXPECT_GT(newAngleDifference, angleDifference);

    generateKeyPress(SDL_KEYDOWN, constants::KEY_DECREASE_ROTATION_SPEED);
    generateKeyPress(SDL_KEYDOWN, SDLK_LEFT);
    cmd = keyboard.GetRobotCommand();
    newAngleDifference = cmd.w - oldAngle;
    EXPECT_NEAR(newAngleDifference, angleDifference, 0.1);

    // these are some repeated keypresses
    // Test limits
    for (int i = 0; i < 200; i++) {
        generateKeyPress(SDL_KEYDOWN, SDLK_LEFT, 1);
        EXPECT_LT(cmd.w, 32);
        EXPECT_GT(cmd.w, 0);
    }

    // stop rotating on keyup
    oldAngle =  keyboard.GetRobotCommand().w;
    generateKeyPress(SDL_KEYUP, SDLK_LEFT);
    cmd = keyboard.GetRobotCommand();
    EXPECT_EQ(cmd.w, oldAngle);
}

TEST(KeyboardTest, It_drives_forward_and_backward) {
    // make sure we are driving (we simulate using xvel, which is up and down.)
    generateKeyPress(SDL_KEYDOWN, SDLK_UP);
    cmd = keyboard.GetRobotCommand();
    EXPECT_GT(cmd.x_vel, 0);
    float velocityDifference = cmd.x_vel;

    // stop pressing so the velocity should be 0 again
    generateKeyPress(SDL_KEYUP, SDLK_UP);
    cmd = keyboard.GetRobotCommand();
    EXPECT_EQ(cmd.x_vel, 0);

    // increase speed
    generateKeyPress(SDL_KEYDOWN, constants::KEY_INCREASE_VEL);

    // the speed difference should be higher
    generateKeyPress(SDL_KEYDOWN, SDLK_UP);
    cmd = keyboard.GetRobotCommand();
    EXPECT_GT(cmd.x_vel, velocityDifference);

    // test limits
    cmd.y_vel = 0;
    for (int i = 0; i < 100; i++) {
        generateKeyPress(SDL_KEYDOWN, constants::KEY_INCREASE_VEL);
    }

    generateKeyPress(SDL_KEYDOWN, SDLK_DOWN);
    cmd = keyboard.GetRobotCommand();
    EXPECT_NEAR(std::abs(cmd.x_vel), constants::MAX_ROBOT_VELOCITY, 0.5); // TODO make this step size

    cmd.y_vel = 0;
    for (int i = 0; i < 100; i++) {
        generateKeyPress(SDL_KEYDOWN, constants::KEY_DECREASE_VEL);
    }

    generateKeyPress(SDL_KEYDOWN, SDLK_DOWN);
    cmd = keyboard.GetRobotCommand();
    EXPECT_EQ(std::abs(cmd.x_vel), constants::MIN_ROBOT_VELOCITY);
}




