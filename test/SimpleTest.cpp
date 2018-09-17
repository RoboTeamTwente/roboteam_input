#include <gtest/gtest.h>
#include <keyboard/keyboard_manager.h>
#include <SDL.h>

SDL_Event evt;
KeyboardManager keyboard;

void generateKeyPress(Uint32 type, SDL_Keycode key) {
    SDL_Event test_event;
    test_event.type = type;
    test_event.key.keysym.sym = key;

    keyboard.handleSDLEvents(test_event);
}

TEST(KeyboardTest, It_toggles_the_dribbler) {
    bool testsPassing = true;
    bool prevDribblerVal = keyboard.GetRobotCommand().dribbler;

    // toggle it so it becomes different from before
    generateKeyPress(SDL_KEYDOWN, SDLK_SPACE);
    testsPassing = keyboard.GetRobotCommand().dribbler != prevDribblerVal;

    // toggle again so it is equal to prevDribblerVal again
    generateKeyPress(SDL_KEYDOWN, SDLK_SPACE);
    testsPassing = testsPassing && keyboard.GetRobotCommand().dribbler == prevDribblerVal;

    ASSERT_TRUE(testsPassing);
}

TEST(KeyboardTest, It_strafes_right) {
    float old_y_vel = keyboard.GetRobotCommand().y_vel;
    generateKeyPress(SDL_KEYDOWN, SDLK_x);
    ASSERT_LT(keyboard.GetRobotCommand().y_vel, old_y_vel);
}

TEST(KeyboardTest, It_strafes_left) {
    float old_y_vel = keyboard.GetRobotCommand().y_vel;
    generateKeyPress(SDL_KEYDOWN, SDLK_z);
    ASSERT_GT(keyboard.GetRobotCommand().y_vel, old_y_vel);
}
