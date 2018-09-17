#include <gtest/gtest.h>
#include <keyboard/keyboard_manager.h>
#include <SDL.h>

KeyboardManager keyboard;

void generateKeyPress(Uint32 type, SDL_Keycode key) {
    SDL_Event test_event;
    test_event.type = type;
    test_event.key.repeat = 0;
    test_event.key.keysym.sym = key;

    keyboard.handleSDLEvents(test_event);
}

// dribbler
TEST(KeyboardTest, It_toggles_the_dribbler) {
    bool testsPassing = true;
    bool prevDribblerVal = keyboard.GetRobotCommand().dribbler;

    // toggle it so it becomes different from before
    generateKeyPress(SDL_KEYDOWN, SDLK_SPACE);
    testsPassing = keyboard.GetRobotCommand().dribbler != prevDribblerVal;

    // this keyup event should not let the test fail
    generateKeyPress(SDL_KEYUP, SDLK_SPACE);

    // toggle again so it is equal to prevDribblerVal again
    generateKeyPress(SDL_KEYDOWN, SDLK_SPACE);
    testsPassing = testsPassing && keyboard.GetRobotCommand().dribbler == prevDribblerVal;

    ASSERT_TRUE(testsPassing);
}

// strafe right
TEST(KeyboardTest, It_strafes_right) {
    float old_y_vel = keyboard.GetRobotCommand().y_vel;
    generateKeyPress(SDL_KEYDOWN, SDLK_x);
    ASSERT_LT(keyboard.GetRobotCommand().y_vel, old_y_vel);
}

TEST(KeyboardTest, It_stops_strafing_right_on_keyrelease) {
    generateKeyPress(SDL_KEYUP, SDLK_x);
    ASSERT_EQ(keyboard.GetRobotCommand().y_vel, 0);
}

// strafe left
TEST(KeyboardTest, It_strafes_left) {
    float old_y_vel = keyboard.GetRobotCommand().y_vel;
    generateKeyPress(SDL_KEYDOWN, SDLK_z);
    ASSERT_GT(keyboard.GetRobotCommand().y_vel, old_y_vel);
}

TEST(KeyboardTest, It_stops_strafing_left_on_keyrelease) {
    generateKeyPress(SDL_KEYUP, SDLK_z);
    ASSERT_EQ(keyboard.GetRobotCommand().y_vel, 0);
}

// kicker
TEST(KeyboardTest, It_kicks) {
    generateKeyPress(SDL_KEYDOWN, SDLK_v);
    roboteam_msgs::RobotCommand cmd = keyboard.GetRobotCommand();
    ASSERT_TRUE(cmd.kicker && cmd.kicker_forced);
}

TEST(KeyboardTest, It_stops_kick_on_keyrelease) {
    generateKeyPress(SDL_KEYUP, SDLK_v);
    roboteam_msgs::RobotCommand cmd = keyboard.GetRobotCommand();
    ASSERT_TRUE(!cmd.kicker && !cmd.kicker_forced);
}

// chipper
TEST(KeyboardTest, It_chips) {
    generateKeyPress(SDL_KEYDOWN, SDLK_n);
    roboteam_msgs::RobotCommand cmd = keyboard.GetRobotCommand();
    ASSERT_TRUE(cmd.chipper && cmd.chipper_forced);
}

TEST(KeyboardTest, It_stops_chip_on_keyrelease) {
    generateKeyPress(SDL_KEYUP, SDLK_n);
    roboteam_msgs::RobotCommand cmd = keyboard.GetRobotCommand();
    ASSERT_TRUE(!cmd.chipper && !cmd.chipper_forced);
}

// Geneva turning test and boundary check
TEST(KeyboardTest, It_turns_geneva) {
    int InitialGenevaState = keyboard.GetRobotCommand().geneva_state;

    bool testsPassing = true;

    // turn left
    for (int i = 0; i < 5; i++) {
        generateKeyPress(SDL_KEYUP, SDLK_PAGEUP); // KEYUP must not make a difference
        generateKeyPress(SDL_KEYDOWN, SDLK_PAGEUP);
        int genevaState = keyboard.GetRobotCommand().geneva_state;
        testsPassing = testsPassing
                && genevaState == std::max(InitialGenevaState - 1 - i, 1);
    }

    // turn left
    for (int i = 0; i < 5; i++) {
        generateKeyPress(SDL_KEYUP, SDLK_PAGEDOWN); // KEYUP must not make a difference

        generateKeyPress(SDL_KEYDOWN, SDLK_PAGEDOWN);
        testsPassing = testsPassing
                       && keyboard.GetRobotCommand().geneva_state == std::min(InitialGenevaState - 1 + i, 5);
    }
    ASSERT_TRUE(testsPassing);

}