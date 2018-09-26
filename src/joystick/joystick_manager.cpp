#include "joystick_manager.h"

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
