/*
This class creates a manager for keyboard control of the robots.
created by mrlukasbos on 04-09-2018

Multiple parameters are distilled from the keyboard controls and converted into a robotCommand.
*/

#include "roboteam_msgs/RobotCommand.h"
#include <boost/optional.hpp>
#include "roboteam_utils/Vector2.h"
#include <iostream>
#include <SDL.h>

namespace b = boost;

class KeyboardManager {
public:
  KeyboardManager();
  void handleSDLEvents(SDL_Event const & e);
  roboteam_msgs::RobotCommand GetRobotCommand();

// TODO make these variables private and only accesible using getters/setters
  int currentGenevaState = 3;
  double currentVel = 1;
  double currentW = 2;
  int currentKick = 1;
  int currentId = 0;

private:
  // Limits
  double const MAX_VEL = 6;
  double const MAX_W = 2048.0 / 360.0 * (2 * M_PI);
  int const MIN_GENEVA_STATE = 1;
  int const MAX_GENEVA_STATE = 5;

  // robot Values
  int x_vel = 0;
  int y_vel = 0;
  double w = 18;
  bool doKick = false;
  bool enKick = false;
  bool doChip = false;
  bool doDribble = false;
  int orientationOffset = 0;

  double const STEP_VEL = 0.1;
  double const STEP_W = 0.5;

  std::vector<SDL_Keycode> const arrowKeys = {
      SDLK_UP,
      SDLK_DOWN,
      SDLK_LEFT,
      SDLK_RIGHT,
      SDLK_z,
      SDLK_x
  };

  // define more descriptive names for the keys
  static const SDL_Keycode KEY_INCREASE_VEL    = SDLK_KP_6;
  static const SDL_Keycode KEY_DECREASE_VEL    = SDLK_KP_4;
  static const SDL_Keycode KEY_INCREASE_ANGLE  = SDLK_KP_3;
  static const SDL_Keycode KEY_DECREASE_ANGLE  = SDLK_KP_1;
  static const SDL_Keycode KEY_INCREASE_KICK   = SDLK_KP_9;
  static const SDL_Keycode KEY_DECREASE_KICK   = SDLK_KP_7;
  static const SDL_Keycode KEY_STRAFE_LEFT     = SDLK_z;
  static const SDL_Keycode KEY_STRAFE_RIGHT    = SDLK_x;
  static const SDL_Keycode KEY_DRIBBLE         = SDLK_SPACE;
  static const SDL_Keycode KEY_KICK            = SDLK_v;
  static const SDL_Keycode KEY_CHIP            = SDLK_n;
  static const SDL_Keycode KEY_ROTATE_LEFT     = SDLK_PAGEUP;
  static const SDL_Keycode KEY_ROTATE_RIGHT    = SDLK_PAGEDOWN;
};
