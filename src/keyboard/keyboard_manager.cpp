#include "keyboard_manager.h"

KeyboardManager::KeyboardManager() = default;

void KeyboardManager::handleSDLEvents(SDL_Event const & e) {
    bool keyPressed = e.type == SDL_KEYDOWN;
    bool keyReleased = e.type == SDL_KEYUP;
    bool notARepeatedKeyPress = e.key.repeat == 0;

    // handle events for each key
    auto key = e.key.keysym.sym;
    switch(key) {

             // increase driving speed
        case constants::KEY_INCREASE_VEL:
            if (keyPressed) currentVel += STEP_VEL;
            break;

            // decrease driving speed
        case constants::KEY_DECREASE_VEL:
            if (keyPressed) currentVel -= STEP_VEL;
            break;

            // increase rotation speed
        case constants::KEY_INCREASE_ROTATION_SPEED:
            if (keyPressed) rotationSpeed+=STEP_ROTATION_SPEED;
            break;

            // decrease rotation speed
        case constants::KEY_DECREASE_ROTATION_SPEED:
            if (keyPressed) rotationSpeed-=STEP_ROTATION_SPEED;
            break;

            // increase kickpower
        case constants::KEY_INCREASE_KICK:
            if (keyPressed) currentKick++;
            break;

            // decrease kickpower
        case constants::KEY_DECREASE_KICK:
            if (keyPressed) currentKick--;
            break;

            // drive forward
        case SDLK_UP:
            if (keyPressed) xDirection = 1;
            else if (keyReleased) xDirection = 0;
            break;

            // drive backward
        case SDLK_DOWN:
            if (keyPressed) xDirection = -1;
            else if (keyReleased) xDirection = 0;
            break;

            // rotate to the left
        case SDLK_LEFT:
            if (keyPressed) rotationDirection = 1;
            else if (keyReleased) rotationDirection = 0;
            break;

            // rotate to the right
        case SDLK_RIGHT:
            if (keyPressed) rotationDirection = -1;
            else if (keyReleased) rotationDirection = 0;
            break;

            // strafe towards left
        case constants::KEY_STRAFE_LEFT:
            if (keyPressed) yDirection = 1;
            else if (keyReleased) yDirection = 0;
            break;

            // strafe towards right
        case constants::KEY_STRAFE_RIGHT:
            if (keyPressed) yDirection = -1;
            else if (keyReleased) yDirection = 0;
            break;

            // toggle the dribbler
        case constants::KEY_DRIBBLE:
            if (keyPressed) doDribble = !doDribble;
            break;

            // kick
        case constants::KEY_KICK:
            if (keyPressed) doKick = true;
            else if (keyReleased) doKick = false;
            break;

            // chip
        case constants::KEY_CHIP:
            if (keyPressed) doChip = true;
            else if (keyReleased) doChip = false;
            break;

            // rotate the geneva to left
        case constants::KEY_GENEVA_LEFT:
            if (keyPressed && notARepeatedKeyPress) currentGenevaState --;
            break;

            // rotate the geneva to right
        case constants::KEY_GENEVA_RIGHT:
            if (keyPressed && notARepeatedKeyPress) currentGenevaState ++;
            break;
        default: break;
    }


    // limits
    if (rotationSpeed > constants::MAX_ROTATION_SPEED) rotationSpeed = constants::MAX_ROTATION_SPEED;
    if (rotationSpeed < constants::MIN_ROTATION_SPEED) rotationSpeed = constants::MIN_ROTATION_SPEED;


    if (currentVel > constants::MAX_ROBOT_VELOCITY) currentVel = constants::MAX_ROBOT_VELOCITY;
    if (currentVel < 0) currentVel = 0;
    if (currentKick > roboteam_msgs::RobotCommand::MAX_KICKER_VEL) currentKick = roboteam_msgs::RobotCommand::MAX_KICKER_VEL;
    if (currentKick < 0) currentKick = 0;
    if (currentGenevaState<constants::MIN_GENEVA_STATE) currentGenevaState = constants::MIN_GENEVA_STATE;
    if (currentGenevaState>constants::MAX_GENEVA_STATE) currentGenevaState = constants::MAX_GENEVA_STATE;


}

roboteam_msgs::RobotCommand KeyboardManager::GetRobotCommand() {
    roboteam_msgs::RobotCommand robotCommand;

    rtt::Vector2 driveVector;
    driveVector.x = xDirection;
    driveVector.y = yDirection;

    rotation += rotationDirection * rotationSpeed;

    if (16 * M_PI < rotation)
        rotation -= 32 * M_PI;       // Bring rotation into range [-16 Pi, 16 Pi]
    if (rotation < -16 * M_PI)
        rotation += 32 * M_PI;       // Bring rotation into range [-16 Pi, 16 Pi]

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
    } else if (doChip) {
        robotCommand.chipper = true;
        robotCommand.chipper_forced = true;
        robotCommand.chipper_vel = currentKick;
    }

    robotCommand.dribbler = doDribble;
    robotCommand.geneva_state = currentGenevaState;

    return robotCommand;
}

int KeyboardManager::getCurrentGenevaState() const {
    return currentGenevaState;
}

double KeyboardManager::getCurrentVel() const {
    return currentVel;
}

int KeyboardManager::getCurrentKick() const {
    return currentKick;
}

int KeyboardManager::getCurrentId() const {
    return currentId;
}

double KeyboardManager::getRotationSpeed() const {
    return rotationSpeed;
}

void KeyboardManager::setCurrentId(int currentId) {
    KeyboardManager::currentId = currentId;
}
