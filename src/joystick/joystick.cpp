#include <string>
#include <cmath>
#include <ctime>
#include <boost/optional.hpp>
#include <math.h>
#include <boost/process.hpp>
#include <map>
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "diagnostic_msgs/DiagnosticArray.h"    // Used for listening to Joy on /diagnostics
#include "roboteam_msgs/RobotCommand.h"
#include "roboteam_msgs/RoleDirective.h"
#include "roboteam_msgs/World.h"
#include "roboteam_msgs/GeometryFieldSize.h"
#include "roboteam_msgs/GeometryData.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/world_analysis.h"

#include "joystick_enums.h"
#include "joystick_profiles.h"

const int NUM_CONTROLLERS = 2;
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
        { Xbox360Controller::B, 1 },    // Turn counterclockwise
        { Xbox360Controller::X, 2 },    // Turn clockwise
        { Xbox360Controller::Y, 3 },    // Enables ID switching and profile switching as long as it is pressed
        { Xbox360Controller::LeftBumper   , 4  },  // Dribbler
        { Xbox360Controller::RightBumper  , 5  },  // Kicker
        { Xbox360Controller::Back         , 6  },  // +Y : Switch control mode (Fifa/CoD)
        { Xbox360Controller::Start        , 7  },  // +Y : Switch profile
        { Xbox360Controller::Guide        , 8  },
        { Xbox360Controller::LeftStick    , 9  },
        { Xbox360Controller::RightStick   , 10 }
};

struct JoyEntry {
    ::boost::optional<::boost::process::child> process;     // Holds the joy_node process
    ::boost::optional<ros::Subscriber> subscriber;          // Subscribes to the joy_node topic (/js0, /js1 etc etc)
    ::boost::optional<sensor_msgs::Joy> msg;                // Holds the latest message from the joy_node topic
    ::boost::optional<sensor_msgs::Joy> msg_prev;           // Holds the latest message from the joy_node topic

    std::string input;                          // js0
    int robotID;                                // 1 - 16
    const int MY_ID;                            // Holds the unique id

    joystick_profile profile;
    int profileCounter;

    // ==== Variables related to automatically running a RoleNode
    std::time_t timeLastReceived = std::time(0);// added by anouk
    ::boost::optional<::boost::process::child> processAuto;         // Holds the joy_node auto process
    bool dribblerOn = false;

    // ==== Variables related to driving ==== //
    std::map<Xbox360Controller, bool> btnState; // Holds the state of the buttons (pressed, not pressed)
    rtt::Vector2 speedState;                         // Holds the x-speed and y-speed of the robot.
    int genevaState;                            // Holds the state of the Geneva Drive. Range is [-2,2]
    bool controllerConnected = true;            // Holds if the corresponding controller is connected
    float orientation = 0.0;                    // Holds the last orientation of the robot
    float orientationOffset = 0.0;              // Holds the orientation offset
    bool useRelativeControl = true;             // Holds the control mode (relative or absolute)

    static int intSupplier;                     // Supplies ids to new instances of JoyEntry

    JoyEntry() : robotID{intSupplier}, MY_ID{intSupplier++} {
        this->profile = profile_default;
    }

    void init(){
        ROS_INFO_STREAM(input << " connected to robot " << robotID);
        setToInput("js" + std::to_string(MY_ID));
    }

    void setToInput(std::string newInput) {
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

    // Get the time between now and the last received message
    int getTimer(){
        return std::time(0) - timeLastReceived;
    }
    // Reset the timer to now
    void resetTimer(){
        timeLastReceived = std::time(0);
    }

    void receiveJoyMsg(const sensor_msgs::JoyConstPtr &msg) {
        this->resetTimer();             // Reset the timer
        this->msg_prev = this->msg;     // Store the current message in previous message
        this->msg = *msg;               // Only store the newest messages
    }

    void setRobotID(int id){
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

    void nextJoystickProfile(){
        profileCounter = (profileCounter + 1) % NUM_JOYSTICK_PROFILES;
        profile = joystick_profiles[profileCounter];
        ROS_INFO_STREAM(input << " using profile " << profileCounter << " " << joystick_profiles[profileCounter].DESCRIPTION);
    }

    void switchControlMode(){
        if(useRelativeControl) {
            useRelativeControl = false;
            ROS_INFO_STREAM(input << " using control mode absolute (Fifa)");
        } else {
            useRelativeControl = true;
            ROS_INFO_STREAM(input << " using control mode relative (Call of Duty)");
        }
    }

    void setControllerConnected(bool isConnected){
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

    void press(Xbox360Controller btn){
        btnState[btn] = true;
    }
    void release(Xbox360Controller btn){
        btnState[btn] = false;
    }
    bool isPressed(Xbox360Controller btn) {
        try {
            return btnState.at(btn);
        }catch(std::out_of_range ex){
            return false;
        }
    }
};


// ======================================================================== //


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
        if(getVal(msg.buttons, xbox360mapping.at(btn)) > 0){                // If Start is pressed
            if(!getVal(msg_prev.buttons, xbox360mapping.at(btn)) > 0){      // Check whether it was not already pressed before
                joy.nextJoystickProfile();                                  // If not, switch to next profile
            }
        }

        /* === Check if control mode has to be modified === */
        btn = Xbox360Controller::Back;
        if(getVal(msg.buttons, xbox360mapping.at(btn)) > 0){                // If Back is pressed
            if(!getVal(msg_prev.buttons, xbox360mapping.at(btn)) > 0){      // Check whether it was not already pressed before
                joy.switchControlMode();                                    // If not, switch to next control mode
            }
        }

        /* ==== Check if ID has to be switched lower ==== */
        btn = Xbox360Controller::DpadY;
        if(getVal(msg.axes, xbox360mapping.at(btn)) > 0){                   // If DpadDown is pressed
            if(!getVal(msg_prev.axes, xbox360mapping.at(btn)) > 0) {        // Check whether it was not already pressed before
                joy.setRobotID((joy.robotID + 1) % 16);                     // If not, increment id
            }
        }else
        if(getVal(msg.axes, xbox360mapping.at(btn)) < 0){                   // If DpadUp is pressed
            if(getVal(msg_prev.axes, xbox360mapping.at(btn)) == -0) {       // Check whether it was not already pressed before
                joy.setRobotID((joy.robotID + 15) % 16);                     // If not, decrement id
            }
        }

        /* ==== Rotate kicker (Geneva Drive)==== */
        btn = Xbox360Controller::DpadX;
        if(getVal(msg.axes, xbox360mapping.at(btn)) > 0){                   // If DpadLeft is pressed
            if(!getVal(msg_prev.axes, xbox360mapping.at(btn)) > 0) {        // Check whether it was not already pressed before
                if(!joy.genevaState == 1) {                                 // Check whether the state is not already minimal
                    joy.genevaState--;                                      // Decrease geneva state by 1
                }
            }
        }else
        if(getVal(msg.axes, xbox360mapping.at(btn)) < 0){                   // If DpadLeft is pressed
            if(!getVal(msg_prev.axes, xbox360mapping.at(btn)) < 0) {        // Check whether it was not already pressed before
                if(!joy.genevaState == 5) {                                 // Check whether the state is not already minimal
                    joy.genevaState++;                                      // Increase geneva state by 1
                }
            }
        }
    }
    /* ==== End DPad control ==== */

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
    if(RightTriggerVal < 0.9) {        						// If RightBumper is pressed
        command.chipper = true;                                 // activate chipper
        command.chipper_forced = true;							// Don't wait for ball sensor, chip immediately
    }
    /* ==================== */

    // ==== Set kicker velocity
    if(command.kicker || command.chipper) {
        command.kicker_vel = 2.0;
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