#include "joystick_manager.h"

std::array<joystick_profile, NUM_JOYSTICK_PROFILES> joystick_profiles = {profile_default, profile_quick, profile_slow};

void JoyEntry::init() {
    ROS_INFO_STREAM(input << " connected to robot " << robotID);
    setToInput("js" + std::to_string(MY_ID));
}

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

int JoyEntry::getTimer(){
    return std::time(0) - timeLastReceived;
}

void JoyEntry::resetTimer(){
    timeLastReceived = std::time(0);
}

void JoyEntry::receiveJoyMsg(const sensor_msgs::JoyConstPtr &msg) {
    this->resetTimer();             // Reset the timer
    this->msg_prev = this->msg;     // Store the current message in previous message
    this->msg = *msg;               // Only store the newest messages
}


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

void JoyEntry::nextJoystickProfile(){
    profileCounter = (profileCounter + 1) % NUM_JOYSTICK_PROFILES;
    profile = joystick_profiles[profileCounter];
    ROS_INFO_STREAM(input << " using profile " << profileCounter << " " << joystick_profiles[profileCounter].DESCRIPTION);
}

void JoyEntry::switchControlMode(){
    if(useRelativeControl) {
        useRelativeControl = false;
        ROS_INFO_STREAM(input << " using control mode absolute (Fifa)");
    } else {
        useRelativeControl = true;
        ROS_INFO_STREAM(input << " using control mode relative (Call of Duty)");
    }
}

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
