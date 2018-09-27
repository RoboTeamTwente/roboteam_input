#include "joystick_manager.h"

std::array<joystick_profile, NUM_JOYSTICK_PROFILES> joystick_profiles = {profile_default, profile_children, profile_quick, profile_slow};

// ==== Initialize joystick ==== //
void JoyEntry::init() {
    ROS_INFO_STREAM(input << " connected to robot " << robotID);
    setToInput("js" + std::to_string(MY_ID));
}

// ==== Create subscriber for the joystick ==== //
void JoyEntry::setToInput(std::string newInput) {
    ROS_INFO_STREAM(input << " now listening to " << newInput);

    // Remove the most recently received message to prevent stale values.
    msg = ::boost::none;
    msg_prev = ::boost::none;

    if(newInput == input)
        return;

    input = newInput;

    // Kill the current process
    if(process) {
        process->terminate();
        process = ::boost::none;
    }

    // Kill the current subscriber
    if(subscriber) {
        subscriber->shutdown();
        subscriber = ::boost::none;
    }

    if(input == "")
        return;

    // If this ever starts throwing weird compile time errors about
    // exe_cmd_init being deleted, just go to posix/basic_cmd.hpp
    // and mark the exe_cmd_init(const ...) function as default
    // See: https://github.com/klemens-morgenstern/boost-process/issues/21

    process = ::boost::process::child(
            ::boost::process::search_path("roslaunch"),
            "roboteam_input",
            "joy_node.launch",
            "jsTarget:=" + input,
            "deadzone:=0.20",
            "nodeName:=JoyStick_" + this->input,
            ::boost::process::std_out > ::boost::process::null
    );

    // Make new subscriber
    ros::NodeHandle n;
    subscriber = n.subscribe(input, 1, &JoyEntry::receiveJoyMsg, this);
}

void JoyEntry::receiveJoyMsg(const sensor_msgs::JoyConstPtr &msg) {
    //std::cout<<"Do stuff"<<std::endl;
    //this->msg_prev = this->msg;     // Store the current message in previous message
    this->msg = *msg;               // Only store the newest messages
}

// ==== Set robot ID and initialize speed and geneva state ==== //
void JoyEntry::setRobotID(int id){
    ROS_INFO_STREAM(input << " connected to robot " << id);
    robotID = id;
    // Reset robot velocity
    speedState.x = 0;
    speedState.y = 0;
    // Reset geneva drive
    genevaState = 3;
    // Reset the orientation
    orientation = 0.0;
}

// ==== Switch to next joystick profile ==== //
void JoyEntry::nextJoystickProfile(){
    profileCounter = (profileCounter + 1) % NUM_JOYSTICK_PROFILES;
    profile = joystick_profiles[profileCounter];
    ROS_INFO_STREAM(input << " using profile " << profileCounter << " " << joystick_profiles[profileCounter].DESCRIPTION);
}

// ==== Switch control mode ==== //
void JoyEntry::switchControlMode(){
    if(useRelativeControl) {
        useRelativeControl = false;
        ROS_INFO_STREAM(input << " using control mode absolute (Fifa)");
    } else {
        useRelativeControl = true;
        ROS_INFO_STREAM(input << " using control mode relative (Call of Duty)");
    }
}

// ==== Check whether joystick is connected ==== //
void JoyEntry::setControllerConnected(bool isConnected){
    if(isConnected && !controllerConnected)
        ROS_INFO_STREAM(input << ": Controller reconnected");

    if(!isConnected && controllerConnected)
        ROS_WARN_STREAM(input << ": Controller disconnected");

    controllerConnected = isConnected;

    // Clear message if controller is disconnected, to make the robot stop
    if(!controllerConnected)
        msg = boost::none;
        msg_prev = boost::none;

}

template<typename T> T getVal(const std::vector<T> &values, int index) {
    if(index < values.size()) {
        return values[index];
    }
    return T(0);
}

// ==== Handle buttons such as control mode and ID, and geneva state ==== //
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
    if(getVal(msg.buttons, xbox360mapping.at(Xbox360Controller::X))) {      // Press X to rotate the Geneva Drive
        /* ==== Rotate kicker (Geneva Drive)==== */
        btn = Xbox360Controller::DpadX;
        if(getVal(msg.axes, xbox360mapping.at(btn)) > 0){                   // If DpadRight is pressed
            if(getVal(msg_prev.axes, xbox360mapping.at(btn)) == -0) {       // Check whether it was not already pressed before
                if(joy.genevaState < 5) {                                   // Check whether the state is not already maximum
                    joy.genevaState++;                                      // Increase geneva state by 1
                }
            }
        }else
        if(getVal(msg.axes, xbox360mapping.at(btn)) < 0){                   // If DpadLeft is pressed
            if(getVal(msg_prev.axes, xbox360mapping.at(btn)) == -0) {       // Check whether it was not already pressed before
                if(joy.genevaState > 1) {                                   // Check whether the state is not already minimal
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

// ==== Make all robot commands and return them ==== //
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

    // The RightTriggerVal is a range from -1 to 1, where -1 is fully pressed and 1 is fully unpressed
    if(RightTriggerVal <= -0.9) {
        if(RightTriggerVal_prev > -0.9) {
            std::cout<<"CHIP!"<<std::endl;
            command.chipper = true;
            command.chipper_forced = true;
        }
    }
    /* ==================== */

    // ==== Set kicker velocity
    if(command.kicker || command.chipper) {
        command.kicker_vel = 5.0;
        command.chipper_vel = 5.0;
    }

    // ==== Set dribbler ====
    btn = Xbox360Controller::LeftBumper;
    if(getVal(msg.buttons, xbox360mapping.at(btn))){            // Check whether the Left Bumper is pressed
        if(!getVal(msg_prev.buttons, xbox360mapping.at(btn))){  // Check whether it was not already pressed before
            std::cout<<"DRIBBLE!!"<<std::endl;
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

