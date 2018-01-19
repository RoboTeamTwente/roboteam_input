#include <string>
#include <cmath>
#include <ctime>
#include <boost/optional.hpp>

namespace b = ::boost;

#include <boost/process.hpp>

namespace bp = ::boost::process;

#include <map>

#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "roboteam_msgs/RobotCommand.h"
#include "roboteam_msgs/World.h"
#include "roboteam_msgs/GeometryFieldSize.h"
#include "roboteam_msgs/GeometryData.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/world_analysis.h"

#include "joystick_enums.h"
#include "joystick_profiles.h"

namespace rtt {

    const int NUM_CONTROLLERS = 2;

    /* Maps the buttons, triggers, and sticks from the Xbox 360 controller to the messages received from joy_node*/
    const std::map<Xbox360Controller, int> xbox360mapping = {
            // Axes
            { Xbox360Controller::LeftStickX   , 0  },  // Drive forward / backward
            { Xbox360Controller::LeftStickY   , 1  },  // Strafe left / right
            { Xbox360Controller::LeftTrigger  , 2  },
            { Xbox360Controller::RightStickX  , 3  },  // Turn left / right
            { Xbox360Controller::RightStickY  , 4  },
            { Xbox360Controller::RightTrigger , 5  },
            { Xbox360Controller::DpadX        , 6  },  // Left : Turn Geneva Drive left / Right : Turn Geneva Drive right
            { Xbox360Controller::DpadY        , 7  },  // Up : ID +=1 / Down : ID -= 1

            // Buttons
            { Xbox360Controller::A, 0 },
            { Xbox360Controller::B, 1 },    // Turn counterclockwise
            { Xbox360Controller::X, 2 },    // Turn clockwise
            { Xbox360Controller::Y, 3 },    // Enables ID switching and profile switching as long as it is pressed
            { Xbox360Controller::LeftBumper   , 4  },  // Dribbler
            { Xbox360Controller::RightBumper  , 5  },  // Kicker
            { Xbox360Controller::Back         , 6  },
            { Xbox360Controller::Start        , 7  },  // +Y : Switch profile
            { Xbox360Controller::Guide        , 8  },
            { Xbox360Controller::LeftStick    , 9  },
            { Xbox360Controller::RightStick   , 10 }
    };



    struct JoyEntry {
        b::optional<bp::child> process;             // Holds the joy_node process
        b::optional<ros::Subscriber> subscriber;    // Subscribes to the joy_node topic (/js0, /js1 etc etc)
        b::optional<sensor_msgs::Joy> msg;          // Holds the latest message from the joy_node topic

        std::string input;                          // js0
        int robotID;                                // 1 - 16
        const int MY_ID;                            // Holds the unique id

        joystick_profile profile;
        int profileCounter;

        std::map<Xbox360Controller, bool> btnState; // Holds the state of the buttons (pressed, not pressed)
        Vector2 speedState;                         // Holds the x-speed and y-speed of the robot
        int genevaState;                            // Holds the state of the Geneva Drive. Range is [-2,2]

        static int intSupplier;                     // Supplies ids to new instances of JoyEntry

        JoyEntry() : robotID{intSupplier}, MY_ID{intSupplier++} {
            this->profile = profile_default;
        }

        void init(){
            std::cout << "[JoyEntry::init] " << input << " connected to robot " << robotID << std::endl;
            setToInput("js" + std::to_string(MY_ID));
        }

        void setToInput(std::string newInput) {

            std::cout << "[JoyEntry::setToInput] now listening to " << newInput << std::endl;

            if (newInput == input)
                return;

            input = newInput;

            // Remove the most recently received message to prevent stale values.
            msg = b::none;

            // Kill your darlings if needed
            if (process) {
                process = b::none;
                //std::cout << "[JoyEntry::setToInput] process killed" << std::endl;
            }

            if (subscriber) {
                subscriber->shutdown();
                subscriber = b::none;
                //std::cout << "[JoyEntry::setToInput] subscriber killed" << std::endl;
            }

            if (input == "") {
                return;
            }

            // Make new process
            std::string exec = "roslaunch";

            std::vector<std::string> args = {
                    "roboteam_input",
                    "joy_node.launch",
                    "jsTarget:=" + input
            };

            // If this ever starts throwing weird compile time errors about
            // exe_cmd_init being deleted, just go to posix/basic_cmd.hpp
            // and mark the exe_cmd_init(const ...) function as default
            // See: https://github.com/klemens-morgenstern/boost-process/issues/21
            //std::cout << "[JoyEntry::setToInput] Starting process" << std::endl;

            process = bp::child(
                    bp::search_path("roslaunch"),
                    "roboteam_input",
                    "joy_node.launch",
                    "jsTarget:=" + input,
                    "deadzone:=0.20",
                    "nodeName:=JoyStick_" + this->input,
                    bp::std_out > bp::null
            );

            // Make new subscriber
            ros::NodeHandle n;
            subscriber = n.subscribe(input, 1, &JoyEntry::receiveJoyMsg, this);
        }

        void receiveJoyMsg(const sensor_msgs::JoyConstPtr &msg) {
//             std::cout << "[JoyEntry::receiveJoyMsg " << this->input << " ] Message received : " << msg << std::endl;

            // Only store the newest messages
            this->msg = *msg;
        }

        void setRobotID(int id){
            std::cout << "[JoyEntry::setRobotID] " << input << " connected to robot " << id << std::endl;
            robotID = id;

            // Reset robot velocity
            speedState.x = 0;
            speedState.y = 0;
        }

        void nextJoystickProfile(){
            this->profileCounter = (this->profileCounter + 1) % NUM_JOYSTICK_PROFILES;
            this->profile = joystick_profiles[this->profileCounter];
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


    int JoyEntry::intSupplier = 0;
    std::array<JoyEntry, NUM_CONTROLLERS> joys;

    template<typename T> T getVal(const std::vector<T> &values, int index) {
        if (index < values.size()) {
            return values[index];
        }
        return T(0);
    }

    roboteam_msgs::RobotCommand makeRobotCommand(JoyEntry &joy, sensor_msgs::Joy const &msg) {
//        std::cout << "[makeRobotCommand] MY_ID: " << joy.MY_ID << " input: " << joy.input << " robotID: " << joy.robotID << std::endl;

        roboteam_msgs::RobotCommand command;

        command.id = joy.robotID;

        /* ==== DPad control is off by default. While Y is pressed it is enabled ==== */
        if(getVal(msg.buttons, xbox360mapping.at(Xbox360Controller::Y))) {

            Xbox360Controller btn;

            /* === Check if profile has to be modified === */
            btn = Xbox360Controller::Start;
            if(getVal(msg.buttons, xbox360mapping.at(btn)) > 0){  // if Start is pressed
                if(!joy.isPressed(btn)){                    // Check if it was already pressed before
                    joy.nextJoystickProfile();                  // If not, switch to next profile
                }
                joy.press(btn);                             // Set button state to pressed
            }else{
                joy.release(btn);                           // Set button state to released
            }

            /* ==== Check if ID has to be switched lower ==== */
            btn = Xbox360Controller::DpadY;
            if(getVal(msg.axes, xbox360mapping.at(btn)) > 0){    // If DpadUp is pressed
                bool pressed = joy.isPressed(btn);            // Store whether the button was pressed before
                joy.press(btn);                               // Set button state to pressed
                if(!pressed) {                                // Check if it was already pressed before
                    joy.setRobotID((joy.robotID + 1) % 16);                // If not, increment id
                    joy.genevaState = 0;                                   // Reset Geneva Drive
                    command.geneva_state = joy.genevaState;                // Reset Geneva Drive
                    return command;
                }
            }else
            if(getVal(msg.axes, xbox360mapping.at(btn)) < 0){   // If DpadDown is pressed
                bool pressed = joy.isPressed(btn);                  // Store whether the button was pressed before
                joy.press(btn);                                     // Set button state to pressed
                if(!pressed) {                                      // Check if it was already pressed before
                    joy.setRobotID((joy.robotID + 15) % 16);            // If not, decrement id
                    joy.genevaState = 0;                                // Reset Geneva Drive
                    command.geneva_state = joy.genevaState;             // Reset Geneva Drive
                    return command;
                }
            }else                                               // If Dpad is not pressed
                joy.release(btn);                                   // Set button state to released
            /* ============================================== */

            /* ==== Rotate kicker (Geneva Drive)==== */
            btn = Xbox360Controller::DpadX;
            if(getVal(msg.axes, xbox360mapping.at(btn)) > 0){    // If DpadLeft is pressed
                if(!joy.isPressed(btn)) {                               // Check if it was already pressed before
                    joy.genevaState--;                                    // Turn Geneva Drive to the left
                    if (joy.genevaState < -2) {                           // Geneva state cannot go below -2 (such state does not exist)
                        joy.genevaState = -2;                               // Set the Geneva state back to -2
                    }
                }
                joy.press(btn);                                         // Set button state to pressed
            } else if(getVal(msg.axes, xbox360mapping.at(btn)) < 0){ // If DpadRight is pressed
                if(!joy.isPressed(btn)) {                               // Check if it was already pressed before
                    joy.genevaState++;                                    // Turn Geneva Drive to the right
                    if (joy.genevaState > 2) {                           // Geneva state cannot go above 2 (such state does not exist)
                        joy.genevaState = 2;                               // Set the Geneva state back to 2
                    }
                }
                joy.press(btn);                                         // Set button state to pressed
            }else                                                // If Dpad is not pressed
                joy.release(btn);                                       // Set button state to released
            /* ============================================== */

        }
        /* ==== End DPad control ==== */



        /* ==== Turn robot clockwise ==== */
        if(getVal(msg.buttons, xbox360mapping.at(Xbox360Controller::X))){
            command.y_vel = 0.8;
            command.w = -2;
            return command;
        }
        /* ======================== */

        /* ==== Turn robot counterclockwise ==== */
        if(getVal(msg.buttons, xbox360mapping.at(Xbox360Controller::B))){
            command.y_vel = -0.8;
            command.w = 2;
            return command;
        }
        /* =============================== */



        /* ==== Smoothing ==== */
        /* Smooth x */
        double diffX = -joy.speedState.x + joy.profile.SPEED_MULTIPLIER * getVal(msg.axes, xbox360mapping.at(Xbox360Controller::LeftStickY));
        joy.speedState.x += joy.profile.SMOOTH_FACTOR * diffX;
        command.x_vel = joy.speedState.x;

        /* Smooth y */
        double diffY = -joy.speedState.y + joy.profile.SPEED_MULTIPLIER * getVal(msg.axes, xbox360mapping.at(Xbox360Controller::LeftStickX));
        joy.speedState.y += joy.profile.SMOOTH_FACTOR * diffY;
        command.y_vel = joy.speedState.y;
        /* =================== */

        // ==== Rotation
        double rot_mult = joy.profile.ROTATION_MULTIPLIER;// + joy.speedState.x * 2;         // maybe sqrt(x²+y²) ?
        command.w = getVal(msg.axes, xbox360mapping.at(Xbox360Controller::RightStickX)) * rot_mult;



        /* ==== Set Kicker ==== */
        Xbox360Controller btn;
        btn = Xbox360Controller::RightBumper;
        if(getVal(msg.buttons, xbox360mapping.at(btn))){        // If RightBumper is pressed
            if(!joy.isPressed(btn))                                 // Check if it was already pressed before
                command.kicker = true;                                  // If not, activate kicker
            joy.press(btn);                                         // Set button state to pressed
        }else                                                   // If RightBumper is not pressed
            joy.release(btn);                                       // Set button state to released
        /* ==================== */


        // ==== Set dribbler
        command.dribbler = getVal(msg.buttons, xbox360mapping.at(Xbox360Controller::LeftBumper)) == 1;

        // ==== Set geneva drive state
        command.geneva_state = joy.genevaState;

        // ==== Set kicker velocity
        if (command.kicker) {
            command.kicker_vel = 3;
            std::cout << "[makeRobotCommand] Kicker command for robot " << joy.robotID << std::endl;
        }

        /* ==== Check speed boundaries ==== */
        /* === Check x === */
          // If speed is below -SPEED_MAX
             if ( command.y_vel         < -joy.profile.SPEED_MAX                                 ) { command.y_vel = -joy.profile.SPEED_MAX; }
          // If speed is inbetween -SPEED_MAX and SPEED_MAX
        else if (-joy.profile.SPEED_MIN <  command.y_vel && command.y_vel < joy.profile.SPEED_MIN) { command.y_vel =  0.0; }
          // If speed is above SPEED_MAX
        else if ( joy.profile.SPEED_MAX <  command.y_vel                                         ) { command.y_vel =  joy.profile.SPEED_MAX; }

        /* === Check y === */
          // If speed is below -SPEED_MAX
             if ( command.x_vel         < -joy.profile.SPEED_MAX                                 ) { command.x_vel = -joy.profile.SPEED_MAX; }
          // If speed is inbetween -SPEED_MAX and SPEED_MAX
        else if (-joy.profile.SPEED_MIN <  command.x_vel && command.x_vel < joy.profile.SPEED_MIN) { command.x_vel =  0.0; }
          // If speed is above SPEED_MAX
        else if ( joy.profile.SPEED_MAX <  command.x_vel                                         ) { command.x_vel =  joy.profile.SPEED_MAX; }

        /* Check rotation */
        if      (fabs(command.w) < joy.profile.ROTATION_MIN) { command.w = 0.0; }
        /* ================================ */

        return command;
    }
} // rtt

int main(int argc, char **argv) {
    using namespace rtt;

    ros::init(argc, argv, "roboteam_input");
    ros::NodeHandle n;

    // Publish on robotcommands
    ros::Publisher pub = n.advertise<roboteam_msgs::RobotCommand>("robotcommands", 10);

    ros::Rate fps(60);

    for (auto &joy : joys) {
        joy.init();
    }

    while (ros::ok()) {

        std::time_t t = std::time(0);
//        std::cout << "\n=======================================" << t << "============================================\n";

        for (auto &joy : joys) {
            if (joy.msg) {
                auto command = makeRobotCommand(joy, *joy.msg);
                pub.publish(command);
            }
        }

        fps.sleep();

        ros::spinOnce();
    }

    return 0;
}
