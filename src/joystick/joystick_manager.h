#include "roboteam_msgs/RobotCommand.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_msgs/RoleDirective.h"
#include "roboteam_msgs/World.h"
#include "roboteam_msgs/GeometryFieldSize.h"
#include "roboteam_msgs/GeometryData.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/world_analysis.h"
#include "joystick_enums.h"
#include "sensor_msgs/Joy.h"
#include <boost/process.hpp>
#include "ros/ros.h"
#include "joystick_profiles.h"
#include "diagnostic_msgs/DiagnosticArray.h"

#ifndef ROBOTEAM_JOYSTICK_MANAGER_H
#define ROBOTEAM_JOYSTICK_MANAGER_H

const int NUM_CONTROLLERS = 4;  // 4 is the max a joystick receiver can connect to
const int TIMEOUT_SECONDS = 1;

/* Maps the buttons, triggers, and sticks from the Xbox 360 controller to the messages received from joy_node */
const std::map<Xbox360Controller, int> xbox360mapping = {
        // Axes
        { Xbox360Controller::LeftStickX   , 0  },  // Drive forward / backward
        { Xbox360Controller::LeftStickY   , 1  },  // Strafe left / right
        { Xbox360Controller::LeftTrigger  , 2  },
        { Xbox360Controller::RightStickX  , 3  },  // Turn left / right
        { Xbox360Controller::RightStickY  , 4  },
        { Xbox360Controller::RightTrigger , 5  },
        { Xbox360Controller::DpadX        , 6  },  // Left : Turn Geneva Drive left / Right : Turn Geneva Drive right
        { Xbox360Controller::DpadY        , 7  },  // Up+Y : ID +=1 / Down+Y : ID -= 1

        // Buttons
        { Xbox360Controller::A, 0 },
        { Xbox360Controller::B, 1 },                // Turn counterclockwise
        { Xbox360Controller::X, 2 },                // Turn clockwise
        { Xbox360Controller::Y, 3 },                // Enables ID switching and profile switching as long as it is pressed
        { Xbox360Controller::LeftBumper   , 4  },   // Dribbler
        { Xbox360Controller::RightBumper  , 5  },   // Kicker
        { Xbox360Controller::Back         , 6  },   // +Y : Switch control mode (Fifa/CoD)
        { Xbox360Controller::Start        , 7  },   // +Y : Switch profile
        { Xbox360Controller::Guide        , 8  },
        { Xbox360Controller::LeftStick    , 9  },
        { Xbox360Controller::RightStick   , 10 }
};

// ======================================================================== //


class joySticks {
public:
    joySticks() : robotID{intSupplier}, MY_ID{intSupplier++} {
        this->profile = profile_children;
    }
    void init();
    void setToInput(std::string newInput);
    int getTimer();
    void resetTimer();
    void receiveJoyMsg(const sensor_msgs::JoyConstPtr &msg);
    void setRobotID(int id);
    void nextJoystickProfile();
    void switchControlMode();
    void setControllerConnected(bool isConnected);

    ::boost::optional<::boost::process::child> process;     // Holds the joy_node process
    ::boost::optional<ros::Subscriber> subscriber;          // Subscribes to the joy_node topic (/js0, /js1 etc etc)
    ::boost::optional<sensor_msgs::Joy> msg;                // Holds the latest message from the joy_node topic
    ::boost::optional<sensor_msgs::Joy> previousMsg;           // Holds the latest message from the joy_node topic

    std::string input;                          // js0
    int robotID;                                // 1 - 16
    const int MY_ID;                            // Holds the unique id

    joystick_profile profile;
    int profileCounter;

    // ==== Variables related to automatically running a RoleNode
    ::boost::optional<::boost::process::child> processAuto;         // Holds the joy_node auto process
    bool dribblerOn = false;

    // ==== Variables related to driving ==== //
    rtt::Vector2 speedState;                    // Holds the x-speed and y-speed of the robot.
    int genevaState;                            // Holds the state of the Geneva Drive. Range is [1,5]
    bool controllerConnected = true;            // Holds if the corresponding controller is connected
    float orientation = 0.0;                    // Holds the last orientation of the robot
    float orientationOffset = 0.0;              // Holds the orientation offset
    bool useRelativeControl = true;             // Holds the control mode (relative or absolute)
    static int intSupplier;                     // Supplies ids to new instances of joySticks
};

template<typename T> T getVal(const std::vector<T> &values, int index);
void handleButtons(joySticks &joy, sensor_msgs::Joy const &msg, sensor_msgs::Joy const &previousMsg);
roboteam_msgs::RobotCommand makeRobotCommand(joySticks &joy, sensor_msgs::Joy const &msg, sensor_msgs::Joy const &previousMsg);
void handleDiagnostics(const diagnostic_msgs::DiagnosticArrayConstPtr& cmd);

#endif  //ROBOTEAM_JOYSTICK_MANAGER_H