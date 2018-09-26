#include <string>
#include <cmath>
#include <ctime>
#include <boost/optional.hpp>
#include <math.h>
#include <map>
#include "diagnostic_msgs/DiagnosticArray.h"    // Used for listening to Joy on /diagnostics

#include "joystick_manager.h"

const int NUM_CONTROLLERS = 4;
const int TIMEOUT_SECONDS = 1;

int JoyEntry::intSupplier = 0;
std::array<JoyEntry, NUM_CONTROLLERS> joys;

template<typename T> T getVal(const std::vector<T> &values, int index) {
    if(index < values.size()) {
        return values[index];
    }
    return T(0);
}

void handleButtons(JoyEntry &joy, sensor_msgs::Joy const &msg, sensor_msgs::Joy const &msg_prev){

    Xbox360Controller btn;

    /* ==== DPad control is off by default. While Y is pressed it is enabled ==== */
    if(getVal(msg.buttons, xbox360mapping.at(Xbox360Controller::Y))) {

        /* === Check if profile has to be modified === */
        btn = Xbox360Controller::Start;
        if (getVal(msg.buttons, xbox360mapping.at(btn)) > 0) {                // If Start is pressed
            if (!getVal(msg_prev.buttons, xbox360mapping.at(btn)) >
                0) {      // Check whether it was not already pressed before
                joy.nextJoystickProfile();                                  // If not, switch to next profile
            }
        }

        /* === Check if control mode has to be modified === */
        btn = Xbox360Controller::Back;
        if (getVal(msg.buttons, xbox360mapping.at(btn)) > 0) {                // If Back is pressed
            if (!getVal(msg_prev.buttons, xbox360mapping.at(btn)) >
                0) {      // Check whether it was not already pressed before
                joy.switchControlMode();                                    // If not, switch to next control mode
            }
        }

        /* ==== Check if ID has to be switched lower ==== */
        btn = Xbox360Controller::DpadY;
        if (getVal(msg.axes, xbox360mapping.at(btn)) > 0) {                   // If DpadDown is pressed
            if (!getVal(msg_prev.axes, xbox360mapping.at(btn)) >
                0) {        // Check whether it was not already pressed before
                joy.setRobotID((joy.robotID + 1) % 16);                     // If not, increment id
            }
        } else if (getVal(msg.axes, xbox360mapping.at(btn)) < 0) {                   // If DpadUp is pressed
            if (getVal(msg_prev.axes, xbox360mapping.at(btn)) ==
                -0) {       // Check whether it was not already pressed before
                joy.setRobotID((joy.robotID + 15) % 16);                     // If not, decrement id
            }
        }
    }

    /* ==== Rotate kicker (Geneva Drive) if X is pressed ==== */
    if(getVal(msg.buttons, xbox360mapping.at(Xbox360Controller::X))) {
        /* ==== Rotate kicker (Geneva Drive)==== */
        btn = Xbox360Controller::DpadX;
        if(getVal(msg.axes, xbox360mapping.at(btn)) > 0){                   // If DpadLeft is pressed
            if(getVal(msg_prev.axes, xbox360mapping.at(btn)) == -0) {        // Check whether it was not already pressed before
                if(joy.genevaState < 5) {                                 // Check whether the state is not already minimal
                    joy.genevaState++;                                      // Increase geneva state by 1
                }
            }
        }else
        if(getVal(msg.axes, xbox360mapping.at(btn)) < 0){                   // If DpadLeft is pressed
            if(getVal(msg_prev.axes, xbox360mapping.at(btn)) == -0) {        // Check whether it was not already pressed before
                if(joy.genevaState > 1) {                                 // Check whether the state is not already minimal
                    joy.genevaState--;                                      // Decrease geneva state by 1
                }
            }
        }
    }

    /* ==== Set rotation offset to current rotation ==== */
    btn = Xbox360Controller::B;
    if(getVal(msg.buttons, xbox360mapping.at(btn)) > 0){                // If B is pressed
        if(!getVal(msg_prev.buttons, xbox360mapping.at(btn)) > 0){      // Check whether it was not already pressed before
            joy.orientationOffset += joy.orientation;                   // Add current orientation to orientation offset
            joy.orientation = 0;                                        // Set current orientation to 0
            ROS_INFO_STREAM(joy.input << " : orientation offset = " << (joy.orientation / (16 * M_PI)));
        }
    }
    /* ================================================== */

}

roboteam_msgs::RobotCommand makeRobotCommand(JoyEntry &joy, sensor_msgs::Joy const &msg, sensor_msgs::Joy const &msg_prev) {

    roboteam_msgs::RobotCommand command;
    command.id = joy.robotID;
    Xbox360Controller btn;

    /* ==== Driving x and y ==== */
    rtt::Vector2 driveVector;
    driveVector.x = getVal(msg.axes, xbox360mapping.at(Xbox360Controller::LeftStickY)); // Get x velocity from joystick
    driveVector.y = getVal(msg.axes, xbox360mapping.at(Xbox360Controller::LeftStickX)); // Get y velocity from joystick
    if(joy.useRelativeControl){
        driveVector = driveVector.rotate(joy.orientation / 16);                         // Rotate velocity according to orientation
    }else{
        driveVector = driveVector.rotate((joy.orientationOffset) / 16);                 // Rotate velocity according to the orientation offset
    }
    command.x_vel = joy.profile.SPEED_MAX * driveVector.x;                              // Set x velocity
    command.y_vel = joy.profile.SPEED_MAX * driveVector.y;                              // Set y velocity

    // ==== Orientation ==== //
    float orientationX = getVal(msg.axes, xbox360mapping.at(Xbox360Controller::RightStickX));
    float orientationY = getVal(msg.axes, xbox360mapping.at(Xbox360Controller::RightStickY));
    rtt::Vector2 orientation(orientationY, orientationX);

    // This checks which control mode to use (absolute (FIFA) or relative (Call of Duty))
    if(!joy.useRelativeControl) {                                               // Checks whether control mode is absolute
        if (0.9 < orientation.length())                                         // Check whether the orientation stick is pushed far enough
            joy.orientation = orientation.angle() * 16;                         // Set new orientation
        command.w = joy.orientation + joy.orientationOffset;                    // Write orientation command and adjust for offset
    } else {                                                                    // If not, control mode is relative
        if (0.9 < fabs(orientationX));
        joy.orientation += orientationX * joy.profile.ROTATION_MULTIPLIER;
        if (16 * M_PI < joy.orientation)
            joy.orientation -= 32 * M_PI;       // Bring rotation into range [-16 Pi, 16 Pi]
        if (joy.orientation < -16 * M_PI)
            joy.orientation += 32 * M_PI;       // Bring rotation into range [-16 Pi, 16 Pi]
        command.w = joy.orientation;
    }

    // Make sure the angle is within range [-16 Pi, 16 Pi]
    if(16 * M_PI < command.w) command.w -= 32 * M_PI;       // Bring rotation into range [-16 Pi, 16 Pi]
    if(command.w <-16 * M_PI) command.w += 32 * M_PI;       // Bring rotation into range [-16 Pi, 16 Pi]

    // ==== Set Kicker ====//
    btn = Xbox360Controller::RightBumper;
    if(getVal(msg.buttons, xbox360mapping.at(btn)) > 0) {            // If RightBumper is pressed
        if (!getVal(msg_prev.buttons, xbox360mapping.at(btn)) > 0) { // Check whether it was not already pressed before
            command.kicker = true;                                  // If not, activate kicker
            command.kicker_forced = true;                            // Don't wait for ball sensor, kick immediately
        }
    }

    /* ==== Set Chipper ====*/
    double RightTriggerVal = getVal(msg.axes, xbox360mapping.at(Xbox360Controller::RightTrigger));
    double RightTriggerVal_prev = getVal(msg_prev.axes, xbox360mapping.at(Xbox360Controller::RightTrigger));

    // If right trigger has not been pressed since starting the program
    if(RightTriggerVal == 0 && !std::signbit(RightTriggerVal)) {
        RightTriggerVal = 1;
    }
    if(RightTriggerVal < 0.9 && RightTriggerVal_prev > 0.9) {   // If RightBumper is pressed and was not before
        command.chipper = true;                                 // Activate chipper
        command.chipper_forced = true;							// Don't wait for ball sensor, chip immediately
    }
    /* ==================== */

    // ==== Set kicker velocity
    if(command.kicker || command.chipper) {
        command.kicker_vel = 5.0;
    }

    // ==== Set dribbler ====
    btn = Xbox360Controller::LeftBumper;
    if(getVal(msg.buttons, xbox360mapping.at(btn))){            // Check whether the Left Bumper is pressed
        if(!getVal(msg_prev.buttons, xbox360mapping.at(btn))){  // Check whether it was not already pressed before
            joy.dribblerOn = !joy.dribblerOn;                   // If not, activate dribbler
        }
    }
    command.dribbler = joy.dribblerOn;

    // ==== Set geneva drive state
    command.geneva_state = joy.genevaState;

    /// ==== Check speed boundaries ==== ///
    /* === Check x === */
    // If speed is below -SPEED_MAX
    if(command.y_vel < -joy.profile.SPEED_MAX) { command.y_vel = -joy.profile.SPEED_MAX; }
    // If speed is inbetween -SPEED_MAX and SPEED_MAX
    else if(-joy.profile.SPEED_MIN < command.y_vel && command.y_vel < joy.profile.SPEED_MIN) { command.y_vel =  0.0; }
    // If speed is above SPEED_MAX
    else if(joy.profile.SPEED_MAX < command.y_vel) { command.y_vel =  joy.profile.SPEED_MAX; }

    /* === Check y === */
    // If speed is below -SPEED_MAX
    if(command.x_vel < -joy.profile.SPEED_MAX) { command.x_vel = -joy.profile.SPEED_MAX; }
    // If speed is inbetween -SPEED_MAX and SPEED_MAX
    else if(-joy.profile.SPEED_MIN <  command.x_vel && command.x_vel < joy.profile.SPEED_MIN) { command.x_vel =  0.0; }
    // If speed is above SPEED_MAX
    else if(joy.profile.SPEED_MAX < command.x_vel) { command.x_vel =  joy.profile.SPEED_MAX; }
    /* ================================ */

    joy.msg_prev = *joy.msg;

    return command;
}

void handleDiagnostics(const diagnostic_msgs::DiagnosticArrayConstPtr& cmd){
    int size = cmd->status.size();
    for(int i = 0; i < size; i++){

        std::string target = cmd->status.at(i).values.at(0).value;
        bool isConnected = cmd->status.at(i).message == "OK";

        for (auto &joy : joys)
            if("/" + joy.input == target)
                joy.setControllerConnected(isConnected);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "joystick_controller");
    ros::NodeHandle n;

    // Publish on robotcommands
    ros::Publisher pub = n.advertise<roboteam_msgs::RobotCommand>("robotcommands", 10);

    // Listen to diagnostics
    ros::Subscriber sub = n.subscribe<diagnostic_msgs::DiagnosticArray>("diagnostics", 1, &handleDiagnostics);

    ros::Rate fps(60);

    ROS_INFO_STREAM("Initializing NUM_CONTROLLERS controller(s)");
    for (auto &joy : joys) {
        joy.init();
    }

    int tickCounter = 0;

    while (ros::ok()) {
        tickCounter++;
        ROS_INFO_STREAM_THROTTLE(5, "==========| Tick " << tickCounter << " |==========");

        // Handle subscriber callbacks
        ros::spinOnce();

        for (auto &joy : joys) {
            // If joystick message received
            if(joy.msg) {
                // HandleButtons, such as ID switching
                if (joy.msg_prev) {
                    handleButtons(joy, *joy.msg, *joy.msg_prev);

                    // Create and publish robot command
                    auto command = makeRobotCommand(joy, *joy.msg, *joy.msg_prev);
                    command.use_angle = static_cast<unsigned char>(true);
                    pub.publish(command);
                } else
                    joy.msg_prev = joy.msg;
            }
        }
        fps.sleep();
    }

    return 0;
}