//
// This class creates a manager for keyboard control of the robots.
// created by mrlukasbos on 04-09-2018
//
// Multiple parameters are distilled from the keyboard controls and converted into a robotCommand.
//

#include "roboteam_msgs/RobotCommand.h"
#include "roboteam_utils/Vector2.h"
#include <SDL.h>
#include "../constants.h"

class KeyboardManager {
public:
    KeyboardManager();
    void handleSDLEvents(SDL_Event const & e);
    roboteam_msgs::RobotCommand GetRobotCommand();
    int getCurrentGenevaState() const;
    double getCurrentVel() const;
    int getCurrentKick() const;
    int getCurrentId() const;
    double getRotationSpeed() const;
    void setCurrentId(int currentId);

private:
    // robot Values
    double rotationSpeed = 1;
    int currentGenevaState = 3;
    double currentVel = 1;
    int currentKick = 1;
    int currentId = 0;
    int xDirection = 0;
    int yDirection = 0;
    int rotationDirection = 0;
    double w = 0;
    double rotation = 0;
    bool doKick = false;
    bool enKick = false;
    bool doChip = false;
    bool doDribble = false;
    int orientationOffset = 0;


    double const STEP_VEL = 0.1;
    double const STEP_W = 0.5;
    const double STEP_ROTATION_SPEED = 0.2;

};
