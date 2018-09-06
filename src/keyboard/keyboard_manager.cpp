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
      case KEY_INCREASE_ROTATION_SPEED:
        rotationSpeed+=STEP_ROTATION_SPEED;
        break;
      case KEY_DECREASE_ROTATION_SPEED:
        rotationSpeed-=STEP_ROTATION_SPEED;
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
        w+=modifierInt;
        break;
      case SDLK_RIGHT:
        w-=modifierInt;
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
      case KEY_GENEVA_LEFT:
        currentGenevaState--;
        break;
      case KEY_GENEVA_RIGHT:
        currentGenevaState++;
        break;
      default: break;
    }
  }


  if (rotationSpeed > MAX_ROTATION_SPEED) rotationSpeed = MAX_ROTATION_SPEED;
  if (rotationSpeed < 0) rotationSpeed = 0;

  rotation += w * rotationSpeed; // the number is rotation speed;

  // limits
  if (currentVel > MAX_VEL) currentVel = MAX_VEL;
  if (currentVel < 0) currentVel = 0;
  if (currentAngularVelocity > MAX_W) currentAngularVelocity = MAX_W;
  if (currentAngularVelocity < 0) currentAngularVelocity = 0;
  if (currentKick > roboteam_msgs::RobotCommand::MAX_KICKER_VEL) currentKick = roboteam_msgs::RobotCommand::MAX_KICKER_VEL;
  if (currentKick < 0) currentKick = 0;
  if (currentGenevaState<MIN_GENEVA_STATE) currentGenevaState = MIN_GENEVA_STATE;
  if (currentGenevaState>MAX_GENEVA_STATE) currentGenevaState = MAX_GENEVA_STATE;

  if (16 * M_PI < rotation)
      rotation -= 32 * M_PI;       // Bring rotation into range [-16 Pi, 16 Pi]
  if (rotation < -16 * M_PI)
      rotation += 32 * M_PI;       // Bring rotation into range [-16 Pi, 16 Pi]
}

roboteam_msgs::RobotCommand KeyboardManager::GetRobotCommand() {
  roboteam_msgs::RobotCommand robotCommand;

  rtt::Vector2 driveVector;
  driveVector.x = x_vel;
  driveVector.y = y_vel;

  driveVector = driveVector.rotate(rotation/16);   // Rotate velocity according to orientation and orientation offset
  robotCommand.x_vel = currentVel * driveVector.x;  // Set x velocity
  robotCommand.y_vel = currentVel * driveVector.y;  // Set y velocity

  robotCommand.id = currentId;
  robotCommand.w = rotation;
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
