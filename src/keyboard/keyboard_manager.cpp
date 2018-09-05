std::vector<SDL_Keycode> const robotIDKeys = {
    SDLK_0, SDLK_1,SDLK_2,SDLK_3,SDLK_4,SDLK_5,SDLK_6,SDLK_7,SDLK_8,SDLK_9,
    SDLK_a,SDLK_b,SDLK_c,SDLK_d,SDLK_e,SDLK_f
};

std::vector<SDL_Keycode> const arrowKeys = {
    SDLK_UP,
    SDLK_DOWN,
    SDLK_LEFT,
    SDLK_RIGHT,
    SDLK_z,
    SDLK_x
};

b::optional<int> stringToID(std::string idStr) {
    if (idStr.length() < 1) return b::none;
    if (idStr[0] >= 'A' && idStr[0] <= 'F') {
        return idStr[0] - 'A' + 10;
    } else if (idStr[0] >= 'a' && idStr[0] <= 'f') {
        return idStr[0] - 'a' + 10;
    } else if (idStr[0] >= '0' && idStr[0] <= '9') {
        return idStr[0] - '0';
    } else {
        return b::none;
    }
}

double const MAX_VEL = 6;
double const MAX_W = 2048.0 / 360.0 * (2 * M_PI);
int const MIN_GENEVA_STATE = 1;
int const MAX_GENEVA_STATE = 5;

enum keyMap : SDL_Keycode = {
        { KEY_INCREASE_VEL  =   SDLK_KP_6 },
        { KEY_DECREASE_VEL  =   SDLK_KP_4 },
        { KEY_INCREASE_W  =     SDLK_KP_3 },
        { KEY_DECREASE_W  =     SDLK_KP_1 },
        { KEY_INCREASE_KICK   =  SDLK_KP_9 },
        { KEY_DECREASE_KICK   =  SDLK_KP_7 },
        { KEY_STRAFE_LEFT   =    SDLK_z },
        { KEY_DECREASE_VEL  =   SDLK_x },
        { KEY_DRIBBLE   =        SDLK_SPACE },
        { KEY_KICK  =           SDLK_v },
        { KEY_CHIP  =           SDLK_n },
        { KEY_ROTATE_LEFT   =    SDLK_PAGEUP },
        { KEY_ROTATE_RIGHT  =   SDLK_PAGEDOWN },

void handleSDLEvent(SDL_Event const & e) {
  int x_vel = 0;
  int y_vel = 0;
  int w = 0;
  bool doKick = false;
  bool enKick = false;
  bool doChip = false;
  bool doDribble = false;
  int currentGenevaState = 3;
  double currentVel = 1;
  double currentW = 5;
  int currentKick = 1;
  double const STEP_VEL = 0.1;
  double const STEP_W = 0.1;


int modifierInt = 0;
bool modifierBool = false;
if (e.type == SDL_KEYDOWN) {
    modifierInt = 1;
    modifierBool = true;
} else if (e.type == SDL_KEYUP && e.key.repeat == 0) {
    modifierInt = -1;
    modifierBool = false;
}

if (e.type == SDL_KEYDOWN || e.type == SDL_KEYUP && e.key.repeat == 0) {
    auto key = e.key.keysym.sym;
    switch(key) {
      case  keyMap::KEY_INCREASE_VEL:
        currentVel += STEP_VEL;
        break;
      case keyMap::KEY_DECREASE_VEL:
        currentVel -= STEP_VEL;
        break;
      case keyMap::KEY_INCREASE_ANGLE:
        currentW += STEP_W;
        break;
      case keyMap::KEY_DECREASE_ANGLE:
        currentW-=STEP_W;
        break;
      case keyMap::KEY_INCREASE_KICK:
        currentKick++;
        break;
      case keyMap::KEY_DECREASE_KICK:
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
      case keyMap::KEY_STRAFE_LEFT:
        y_vel += modifierInt;
        break;
      case keyMap::KEY_STRAFE_RIGHT:
        y_vel -= modifierInt;
        break;
      case keyMap::KEY_DRIBBLE:
        doDribble = modifierBool;
        break;
      case keyMap::KEY_KICK:
        doKick = modifierBool;
        break;
      case keyMap::KEY_CHIP:
        doChip = modifierBool;
        break;
      case keyMap::KEY_ROTATE_LEFT:
        currentGenevaState--;
        break;
      case keyMap::KEY_ROTATE_RIGHT;
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


roboteam_msgs::RobotCommand robotCommand;

robotCommand.id = currentID;
robotCommand.x_vel = direction.x_vel * speed.currentVel;
robotCommand.y_vel = direction.y_vel * speed.currentVel;
robotCommand.w = direction.w * speed.currentW;
robotCommand.use_angle = true;

if (direction.doKick) {
    robotCommand.kicker = true;
    robotCommand.kicker_forced = true;
    robotCommand.kicker_vel = speed.currentKick;
    // robotCommand.kicker_forced = true;
} else if (direction.doChip) {
    robotCommand.chipper = true;
    robotCommand.chipper_forced = true;
    robotCommand.chipper_vel = speed.currentKick;
    // robotCommand.chipper_forced= true;
}

robotCommand.dribbler = direction.doDribble;
robotCommand.geneva_state = direction.currentGenevaState;
}

bool isCtrlPressed(SDL_Event const e) {
    if (e.type == SDL_KEYDOWN) {
        if ((e.key.keysym.mod & KMOD_CTRL) > 0) {
            return true;
        }
    }
    return false;
}
}
