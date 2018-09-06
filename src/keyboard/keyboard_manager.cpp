#include "keyboard_manager.h"

KeyboardManager::KeyboardManager() {}

void KeyboardManager::handleSDLEvents(SDL_Event const & e) {
  int modifierInt = 0;
  bool modifierBool = false;
  if (e.type == SDL_KEYDOWN) {
      modifierInt = 1;
      modifierBool = true;
  } else if (e.type == SDL_KEYUP) {
      modifierInt = -1;
      modifierBool = false;
  }

if ((e.type == SDL_KEYDOWN || e.type == SDL_KEYUP) && e.key.repeat == 0) {

    auto key = e.key.keysym.sym;
    switch(key) {
      case KEY_INCREASE_VEL:
        currentVel += STEP_VEL;
        break;
      case KEY_DECREASE_VEL:
        currentVel -= STEP_VEL;
        break;
      case KEY_INCREASE_ANGLE:
        currentW += STEP_W;
        break;
      case KEY_DECREASE_ANGLE:
        currentW-=STEP_W;
        break;
      case KEY_INCREASE_KICK:
        currentKick++;
        break;
      case KEY_DECREASE_KICK:
        currentKick--;
        break;
      case SDLK_UP:
        x_vel += modifierInt;
        break;
      case SDLK_DOWN:
        x_vel -= modifierInt;
        break;
      case SDLK_LEFT:
        w += modifierInt;
        break;
      case SDLK_RIGHT:
        w -= modifierInt;
        break;
      case KEY_STRAFE_LEFT:
        y_vel += modifierInt;
        break;
      case KEY_STRAFE_RIGHT:
        y_vel -= modifierInt;
        break;
      case KEY_DRIBBLE:
        doDribble = modifierBool;
        break;
      case KEY_KICK:
        doKick = modifierBool;
        break;
      case KEY_CHIP:
        doChip = modifierBool;
        break;
      case KEY_ROTATE_LEFT:
        currentGenevaState--;
        break;
      case KEY_ROTATE_RIGHT:
        currentGenevaState++;
        break;
      default: break;
    }
  }

  // limits
  if (currentVel > MAX_VEL) currentVel = MAX_VEL;
  if (currentVel < 0) currentVel = 0;
  if (currentW > MAX_W) currentW = MAX_W;
  if (currentW < 0) currentW = 0;
  if (currentKick > roboteam_msgs::RobotCommand::MAX_KICKER_VEL) currentKick = roboteam_msgs::RobotCommand::MAX_KICKER_VEL;
  if (currentKick < 0) currentKick = 0;
  if (currentGenevaState<MIN_GENEVA_STATE) currentGenevaState = MIN_GENEVA_STATE;
  if (currentGenevaState>MAX_GENEVA_STATE) currentGenevaState = MAX_GENEVA_STATE;
}

roboteam_msgs::RobotCommand KeyboardManager::GetRobotCommand() {
  roboteam_msgs::RobotCommand robotCommand;

  robotCommand.id = currentId;
  robotCommand.x_vel = x_vel * currentVel;
  robotCommand.y_vel = y_vel * currentVel;
  robotCommand.w = w * currentW;
  robotCommand.use_angle = true;

  if (doKick) {
      robotCommand.kicker = true;
      robotCommand.kicker_forced = true;
      robotCommand.kicker_vel = currentKick;
      // robotCommand.kicker_forced = true;
  } else if (doChip) {
      robotCommand.chipper = true;
      robotCommand.chipper_forced = true;
      robotCommand.chipper_vel = currentKick;
      // robotCommand.chipper_forced= true;
  }

  robotCommand.dribbler = doDribble;
  robotCommand.geneva_state = currentGenevaState;

  return robotCommand;
}
