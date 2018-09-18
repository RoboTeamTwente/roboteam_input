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
    generateKeyPress(SDL_KEYDOWN, SDLK_SPACE);
    EXPECT_FALSE(keyboard.GetRobotCommand().dribbler == prevDribblerVal);

    // this keyup event should not let the test fail
    generateKeyPress(SDL_KEYUP, SDLK_SPACE);
    EXPECT_FALSE(keyboard.GetRobotCommand().dribbler == prevDribblerVal);

    // toggle again so it is equal to prevDribblerVal again
    generateKeyPress(SDL_KEYDOWN, SDLK_SPACE);
    EXPECT_TRUE(keyboard.GetRobotCommand().dribbler == prevDribblerVal);
}

TEST(KeyboardTest, It_strafes) {
    // Strafe right
    generateKeyPress(SDL_KEYDOWN, SDLK_x);
    ASSERT_LT(keyboard.GetRobotCommand().y_vel, 0);

    // stop strafing right
    generateKeyPress(SDL_KEYUP, SDLK_x);
    ASSERT_EQ(keyboard.GetRobotCommand().y_vel, 0);

    // strafe left
    generateKeyPress(SDL_KEYDOWN, SDLK_z);
    ASSERT_GT(keyboard.GetRobotCommand().y_vel, 0);

    // stop strafe left
    generateKeyPress(SDL_KEYUP, SDLK_z);
    ASSERT_EQ(keyboard.GetRobotCommand().y_vel, 0);
}

// kicker
TEST(KeyboardTest, It_kicks) {
    generateKeyPress(SDL_KEYDOWN, SDLK_v);
    cmd = keyboard.GetRobotCommand();
    EXPECT_TRUE(cmd.kicker && cmd.kicker_forced);

    generateKeyPress(SDL_KEYUP, SDLK_v);
    cmd = keyboard.GetRobotCommand();
    EXPECT_TRUE(!cmd.kicker && !cmd.kicker_forced);
}

// chipper
TEST(KeyboardTest, It_chips) {
    generateKeyPress(SDL_KEYDOWN, SDLK_n);
    cmd = keyboard.GetRobotCommand();
    ASSERT_TRUE(cmd.chipper && cmd.chipper_forced);

    generateKeyPress(SDL_KEYUP, SDLK_n);
    cmd = keyboard.GetRobotCommand();
    ASSERT_TRUE(!cmd.chipper && !cmd.chipper_forced);
}

// Geneva turning test and boundary check
TEST(KeyboardTest, It_turns_geneva) {
    int InitialGenevaState = keyboard.GetRobotCommand().geneva_state;

    // turn left
    for (int i = 1; i <= 5; i++) {
        generateKeyPress(SDL_KEYUP, SDLK_PAGEUP); // KEYUP must not make a difference
        generateKeyPress(SDL_KEYDOWN, SDLK_PAGEUP);
        int genevaState = keyboard.GetRobotCommand().geneva_state;
        EXPECT_EQ(genevaState, std::max(InitialGenevaState - i, 1));
    }

    // turn right
    InitialGenevaState = keyboard.GetRobotCommand().geneva_state;
    for (int i = 1; i <= 5; i++) {
        generateKeyPress(SDL_KEYUP, SDLK_PAGEDOWN); // KEYUP must not make a difference
        generateKeyPress(SDL_KEYDOWN, SDLK_PAGEDOWN);
        int genevaState = keyboard.GetRobotCommand().geneva_state;
        EXPECT_EQ(genevaState, std::min(InitialGenevaState + i, 5));
    }

    // turn left but with key pressed on repeat, which shouldn't do anything.
    InitialGenevaState = keyboard.GetRobotCommand().geneva_state;
    generateKeyPress(SDL_KEYDOWN, SDLK_PAGEDOWN, 1);
    EXPECT_EQ(keyboard.GetRobotCommand().geneva_state, InitialGenevaState);

    generateKeyPress(SDL_KEYDOWN, SDLK_PAGEDOWN, 1);
    EXPECT_EQ(keyboard.GetRobotCommand().geneva_state, InitialGenevaState);

    generateKeyPress(SDL_KEYDOWN, SDLK_PAGEUP, 1);
    EXPECT_EQ(keyboard.GetRobotCommand().geneva_state, InitialGenevaState);
}

TEST(KeyboardTest, It_rotates_robot) {
    float oldAngle =  keyboard.GetRobotCommand().w;
    generateKeyPress(SDL_KEYDOWN, SDLK_LEFT);
    cmd = keyboard.GetRobotCommand();

    EXPECT_TRUE(cmd.use_angle && cmd.w > oldAngle);

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

    // The angle should not change anymore.
    ASSERT_EQ(cmd.w, oldAngle);
}
