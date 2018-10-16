#include "joystick_manager.h"

std::array<joystick_profile, NUM_JOYSTICK_PROFILES> joystick_profiles = {profile_default, profile_children, profile_quick, profile_slow};

// ==== Initialize joystick ==== //
void joySticks::init() {
    ROS_INFO_STREAM(input << " connected to robot " << robotID);
    setToInput("js" + std::to_string(MY_ID));
}

// ==== Create subscriber for the joystick ==== //
void joySticks::setToInput(std::string newInput) {
    ROS_INFO_STREAM(input << " now listening to " << newInput);

    // Remove the most recently received message to prevent stale values.
    msg = ::boost::none;
    previousMsg = ::boost::none;

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
    subscriber = n.subscribe(input, 1, &joySticks::receiveJoyMsg, this);
}

void joySticks::receiveJoyMsg(const sensor_msgs::JoyConstPtr &msg) {
    this->msg = *msg;               // Only store the newest messages
}

// ==== Set robot ID and initialize speed and geneva state ==== //
void joySticks::initializeRobot(int id){
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
void joySticks::nextJoystickProfile(){
    profileCounter = (profileCounter + 1) % NUM_JOYSTICK_PROFILES;
    profile = joystick_profiles[profileCounter];
    ROS_INFO_STREAM(input << " using profile " << profileCounter << " " << joystick_profiles[profileCounter].DESCRIPTION);
}

// ==== Switch control mode ==== //
void joySticks::switchControlMode(){
    if(useRelativeControl) {
        useRelativeControl = false;
        ROS_INFO_STREAM(input << " using control mode absolute (Fifa)");
    } else {
        useRelativeControl = true;
        ROS_INFO_STREAM(input << " using control mode relative (Call of Duty)");
    }
}

// ==== Check whether joystick is connected ==== //
void joySticks::setControllerConnected(bool isConnected){
    if(isConnected && !controllerConnected)
        ROS_INFO_STREAM(input << ": Controller reconnected");

    if(!isConnected && controllerConnected)
        ROS_WARN_STREAM(input << ": Controller disconnected");

    controllerConnected = isConnected;

    // Clear message if controller is disconnected, to make the robot stop
    if(!controllerConnected)
        msg = boost::none;
        previousMsg = boost::none;

}

void joySticks::setAutoPlay(std::string role) {
    boost::filesystem::path pathRosrun = boost::process::search_path("rosrun");
    std::vector<std::string> args;
    args.emplace_back("roboteam_tactics");
    args.emplace_back("TestX");
    args.emplace_back("rtt_jelle/" + role);
    args.emplace_back("ROBOT_ID=" + boost::lexical_cast<std::string>(this->robotID));

    processAuto = boost::process::child(pathRosrun, args);
    this->autoPlay = true;
}

void joySticks::stopAutoPlay() {
    ROS_INFO_STREAM("Joy " << this->robotID << " terminating process..");
    processAuto->terminate();
    processAuto = boost::none;
    this->autoPlay = false;
}

template<typename T> T getVal(const std::vector<T> &values, int index) {
    if(index < values.size()) {
        return values[index];
    }
    return T(0);
}

// ==== Handle buttons such as control mode and ID, and geneva state ==== //
void handleButtons(joySticks &joy, sensor_msgs::Joy const &msg, sensor_msgs::Joy const &previousMsg){

    Xbox360Controller btn;

    /* ==== DPad control is off by default. While Y is pressed it is enabled ==== */
    if(getVal(msg.buttons, xbox360mapping.at(Xbox360Controller::Y))) {

        /* === Check if profile has to be modified === */
        btn = Xbox360Controller::Start;
        if (getVal(msg.buttons, xbox360mapping.at(btn)) > 0) {
            if (getVal(previousMsg.buttons, xbox360mapping.at(btn)) <=
                0) {
                joy.nextJoystickProfile();
            }
        }

        /* === Check if control mode has to be modified === */
        btn = Xbox360Controller::Back;
        if (getVal(msg.buttons, xbox360mapping.at(btn)) > 0) {
            if (getVal(previousMsg.buttons, xbox360mapping.at(btn)) <=
                0) {      // Check whether it was not already pressed before
                joy.switchControlMode();
            }
        }

        // Check if ID has to be switched, only when auto play is off
        if(!joy.autoPlay) {
            btn = Xbox360Controller::DpadY;
            if (getVal(msg.axes, xbox360mapping.at(btn)) > 0) {
                if (getVal(previousMsg.axes, xbox360mapping.at(btn)) <=
                    0) {
                    joy.initializeRobot((joy.robotID + 1) % 16);
                }
            } else if (getVal(msg.axes, xbox360mapping.at(btn)) < 0) {
                if (getVal(previousMsg.axes, xbox360mapping.at(btn)) ==
                    -0) {
                    joy.initializeRobot((joy.robotID + 15) % 16);
                }
            }
        }
    }

    /* ==== Rotate kicker (Geneva Drive) if X is pressed ==== */
    if(getVal(msg.buttons, xbox360mapping.at(Xbox360Controller::X))) {
        /* ==== Rotate kicker (Geneva Drive)==== */
        btn = Xbox360Controller::DpadX;
        if(getVal(msg.axes, xbox360mapping.at(btn)) > 0){
            if(getVal(previousMsg.axes, xbox360mapping.at(btn)) == -0) {
                if(joy.genevaState < 5) {
                    joy.genevaState++;
                }
            }
        }else
        if(getVal(msg.axes, xbox360mapping.at(btn)) < 0){
            if(getVal(previousMsg.axes, xbox360mapping.at(btn)) == -0) {
                if(joy.genevaState > 1) {
                    joy.genevaState--;
                }
            }
        }
    }

    /* ==== Set rotation offset to current rotation ==== */
    btn = Xbox360Controller::B;
    if(getVal(msg.buttons, xbox360mapping.at(btn)) > 0){
        if(!getVal(previousMsg.buttons, xbox360mapping.at(btn)) > 0){
            joy.orientationOffset += joy.orientation;
            joy.orientation = 0;
            ROS_INFO_STREAM(joy.input << " : orientation offset = " << (joy.orientation / (16 * M_PI)));
        }
    }

    // Toggle autoPlay
    if(getVal(msg.buttons, xbox360mapping.at(Xbox360Controller::A)) > 0) {

        if (!joy.autoPlay) {
            // Check for left bumper, and toggle autoKeeper
            if (getVal(msg.buttons, xbox360mapping.at(Xbox360Controller::LeftBumper)) > 0) {
                if (getVal(previousMsg.buttons, xbox360mapping.at(Xbox360Controller::LeftBumper)) <= 0) {
                    joy.setAutoPlay("DemoKeeper");
                }
            } else

            // Check for left trigger, and toggle autoAttacker
            if (getVal(msg.buttons, xbox360mapping.at(Xbox360Controller::RightBumper)) > 0) {
                if (getVal(previousMsg.buttons, xbox360mapping.at(Xbox360Controller::RightBumper)) <= 0) {
                    joy.setAutoPlay("DemoAttacker");
                }
            }
        } else
            if(getVal(previousMsg.buttons, xbox360mapping.at(Xbox360Controller::A)) <= 0) {
                joy.stopAutoPlay();
        }
    }
}

// ==== Make all robot commands and return them ==== //
roboteam_msgs::RobotCommand makeRobotCommand(joySticks &joy, sensor_msgs::Joy const &msg, sensor_msgs::Joy const &previousMsg) {

    roboteam_msgs::RobotCommand command;
    command.id = joy.robotID;
    Xbox360Controller btn;

    /* ==== Driving x and y ==== */
    rtt::Vector2 driveVector;
    driveVector.x = getVal(msg.axes, xbox360mapping.at(Xbox360Controller::LeftStickY));
    driveVector.y = getVal(msg.axes, xbox360mapping.at(Xbox360Controller::LeftStickX));
    if(joy.useRelativeControl){
        driveVector = driveVector.rotate(joy.orientation / 16);
    }else{
        driveVector = driveVector.rotate((joy.orientationOffset) / 16);
    }
    command.x_vel = joy.profile.SPEED_MAX * driveVector.x;
    command.y_vel = joy.profile.SPEED_MAX * driveVector.y;

    // ==== Orientation ==== //
    float orientationX = getVal(msg.axes, xbox360mapping.at(Xbox360Controller::RightStickX));
    float orientationY = getVal(msg.axes, xbox360mapping.at(Xbox360Controller::RightStickY));
    rtt::Vector2 orientation(orientationY, orientationX);

    // This checks which control mode to use (absolute (FIFA) or relative (Call of Duty))
    if(!joy.useRelativeControl) {
        if (0.9 < orientation.length())
            joy.orientation = orientation.angle() * 16;
        command.w = joy.orientation + joy.orientationOffset;
    } else {
        if (0.9 < fabs(orientationX));
        joy.orientation += orientationX * joy.profile.ROTATION_MULTIPLIER;
        if (16 * M_PI < joy.orientation)
            joy.orientation -= 32 * M_PI;
        if (joy.orientation < -16 * M_PI)
            joy.orientation += 32 * M_PI;
        command.w = joy.orientation;
    }

    // Make sure the angle is within range [-16 Pi, 16 Pi]
    if(16 * M_PI < command.w) command.w -= 32 * M_PI;
    if(command.w <-16 * M_PI) command.w += 32 * M_PI;

    // ==== Set Kicker ====//
    btn = Xbox360Controller::RightBumper;
    if(getVal(msg.buttons, xbox360mapping.at(btn)) > 0) {
        if (!getVal(previousMsg.buttons, xbox360mapping.at(btn)) > 0) {
            command.kicker = true;
            command.kicker_forced = true;
        }
    }

    /* ==== Set Chipper ====*/
    double RightTriggerVal = getVal(msg.axes, xbox360mapping.at(Xbox360Controller::RightTrigger));
    double RightTriggerVal_prev = getVal(previousMsg.axes, xbox360mapping.at(Xbox360Controller::RightTrigger));

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
    if(getVal(msg.buttons, xbox360mapping.at(btn))){
        if(!getVal(previousMsg.buttons, xbox360mapping.at(btn))){
            joy.dribblerOn = !joy.dribblerOn;
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

    return command;
}

