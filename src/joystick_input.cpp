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
#include "roboteam_msgs/RoleDirective.h"
#include "roboteam_msgs/World.h"
#include "roboteam_msgs/GeometryFieldSize.h"
#include "roboteam_msgs/GeometryData.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/world_analysis.h"

#include "joystick_enums.h"

namespace rtt {

    const int NUM_CONTROLLERS = 1;
    const int TIMEOUT_SECONDS = 5;

    /* Maps the buttons, triggers, and sticks from the Xbox 360 controller to the messages received from joy_node*/
    const std::map<Xbox360Controller, int> xbox360mapping = {
            // Axes
            { Xbox360Controller::LeftStickX   , 0  },  // Drive forward / backward
            { Xbox360Controller::LeftStickY   , 1  },  // Strafe left / right
            { Xbox360Controller::LeftTrigger  , 2  },
            { Xbox360Controller::RightStickX  , 3  },  // Turn left / right
            { Xbox360Controller::RightStickY  , 4  },
            { Xbox360Controller::RightTrigger , 5  },  // Turbo
            { Xbox360Controller::DpadX        , 6  },  // Left : ID -=1 / Right : ID += 1
            { Xbox360Controller::DpadY        , 7  },

            // Buttons
            { Xbox360Controller::A, 0 },
            { Xbox360Controller::B, 1 },
            { Xbox360Controller::X, 2 },    // Turn clockwise
            { Xbox360Controller::Y, 3 },    // Turn counterclockwise
            { Xbox360Controller::LeftBumper   , 4  },  // Dribbler
            { Xbox360Controller::RightBumper  , 5  },  // Kicker
            { Xbox360Controller::Back         , 6  },
            { Xbox360Controller::Start        , 7  },
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

        // ==== Variables related to automatically running a RoleNode
        std::time_t timeLastReceived = std::time(0);       // added by anouk
        bool isRunningAuto = false;                 // added by anouk
        b::optional<bp::child> processAuto;         // Holds the joy_node auto process

        std::map<Xbox360Controller, bool> btnState; // Holds the state of the buttons (pressed, not pressed)
        Vector2 speedState;                         // Holds the x-speed and y-speed of the robot

        static int intSupplier;                     // Supplies ids to new instances of JoyEntry

        JoyEntry() : robotID{intSupplier}, MY_ID{intSupplier++} {}

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

        // Get the time between now and the last received message
        int getTimer(){
           return std::time(0) - timeLastReceived;
        }
        // Reset the timer to now
        void resetTimer(){
            timeLastReceived = std::time(0);
        }
        void updateAutoProcess(){
            // If timeout not yet reached
            if(getTimer() <3 - 3 + TIMEOUT_SECONDS) { // #Liefde #LoveLife #RoboTeamLife
                // If process is running, kill it
                if(processAuto && processAuto->running()){
                    ROS_INFO_STREAM_NAMED("input", "Joy " << robotID << " terminating process..");
                    processAuto->terminate();
                    processAuto = b::none;      // This is required, just calling terminate doesn't cut it. Without this, processAuto->running() returns false when starting a new process
                }
            }else
            // If timeout reached
            {
                // If no process is running, start it
                if(!processAuto || !processAuto->running()){
                    ROS_INFO_STREAM_NAMED("input", "Joy " << robotID << " starting process..");

                    boost::filesystem::path pathRosrun = bp::search_path("rosrun");
                    std::vector<std::string> args;
                    args.push_back("roboteam_tactics");
                    args.push_back("TestX");
                    args.push_back("rtt_bob/DemoAttacker");
                    args.push_back("string:GetBall__aimAt=ourgoal");
                    args.push_back("int:ROBOT_ID=" + std::to_string(robotID));
                    args.push_back("double:Kick__kickVel=8.0");

                    processAuto = bp::child(pathRosrun, args);
                }
            }
        }

        void receiveJoyMsg(const sensor_msgs::JoyConstPtr &msg) {
            this->resetTimer(); // Reset the timer
            this->msg = *msg;   // Only store the newest messages
        }

        void setRobotID(int id){
            ROS_INFO_STREAM_NAMED("input", input << " connected to robot " << id);
            robotID = id;
            // Reset robot velocity
            speedState.x = 0;
            speedState.y = 0;
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

    const double SMOOTH_FACTOR = 0.3;
    const double SPEED_MULTIPLIER = 1;
    const double SPEED_MIN = 0.5;
    const double SPEED_MAX = 2;

    const double ROTATION_MULTIPLIER = 5;
    const double ROTATION_MIN = 1.5;


    roboteam_msgs::RobotCommand makeRobotCommand(JoyEntry &joy, sensor_msgs::Joy const &msg) {
//        std::cout << "[makeRobotCommand] MY_ID: " << joy.MY_ID << " input: " << joy.input << " robotID: " << joy.robotID << std::endl;

        roboteam_msgs::RobotCommand command;

        command.id = joy.robotID;



        /* ==== Check if ID has to be switched lower ==== */
        Xbox360Controller btn;
        btn = Xbox360Controller::DpadX;
        if(getVal(msg.axes, xbox360mapping.at(btn)) > 0){    // If DpadLeft is pressed
            if(!joy.isPressed(btn))                                 // Check if it was already pressed before
                joy.setRobotID((joy.robotID + 15) % 16);                // If not, decrement id
            joy.press(btn);                                         // Set button state to pressed
        }else
        if(getVal(msg.axes, xbox360mapping.at(btn)) < 0){    // If DpadRight is pressed
            if(!joy.isPressed(btn))                                 // Check if it was already pressed before
                joy.setRobotID((joy.robotID + 1) % 16);             // If not, increment id
            joy.press(btn);                                         // Set button state to pressed
        }else                                                   // If Dpad is not pressed
            joy.release(btn);                                       // Set button state to released
        /* ============================================== */



        /* ==== Turn clockwise ==== */
        if(getVal(msg.buttons, xbox360mapping.at(Xbox360Controller::X))){
            command.y_vel = 0.8;
            command.w = -2;
            return command;
        }
        /* ======================== */

        /* ==== Turn counterclockwise ==== */
        if(getVal(msg.buttons, xbox360mapping.at(Xbox360Controller::B))){
            command.y_vel = -0.8;
            command.w = 2;
            return command;
        }
        /* =============================== */


        double speedmult = SPEED_MULTIPLIER;

        /* ==== Turbo ==== */
            /* For some reason, if the RightTrigger is released and hasn't been pressed yet, this reads 0 instead of 1 */
            /* When the trigger is pressed halfway, the value -0 is sent, distinguishing is from 0 */
//        double RightTriggerVal = getVal(msg.axes, xbox360mapping.at(Xbox360Controller::RightTrigger));
//        if(RightTriggerVal == 0 && !std::signbit(RightTriggerVal))
//            RightTriggerVal = 1;
//        speedmult = SPEED_MULTIPLIER + (1 - RightTriggerVal);


        /* ==== Smoothing ==== */
        /* Smooth x */
        double diffX = -joy.speedState.x + speedmult * getVal(msg.axes, xbox360mapping.at(Xbox360Controller::LeftStickY));
        joy.speedState.x += SMOOTH_FACTOR * diffX;
        command.x_vel = joy.speedState.x;
        /* Smooth y */
        double diffY = -joy.speedState.y + speedmult * getVal(msg.axes, xbox360mapping.at(Xbox360Controller::LeftStickX));
        joy.speedState.y += SMOOTH_FACTOR * diffY;
        command.y_vel = joy.speedState.y;
        /* =================== */

        /* Non-smoothing */
        //command.y_vel = 3 * getVal(msg.axes, xbox360mapping.at(Xbox360Controller::LeftStickY);
        //command.x_vel = 3 * getVal(msg.axes, xbox360mapping.at(Xbox360Controller::LeftStickX);

        // ==== Rotation
        double rot_mult = ROTATION_MULTIPLIER;// + joy.speedState.x * 2;         // maybe sqrt(x²+y²) ?
        command.w = getVal(msg.axes, xbox360mapping.at(Xbox360Controller::RightStickX)) * rot_mult;


        /* ==== Set Kicker ==== */
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


        // ==== Set kicker velocity
        if (command.kicker) {
            command.kicker_vel = 3;
            std::cout << "[makeRobotCommand] Kicker command for robot " << joy.robotID << std::endl;
        }

        /* ==== Check speed boundaries ==== */
        /* Check x */
        if      (fabs(command.y_vel) < SPEED_MIN) { command.y_vel = 0.0; }
        else if (fabs(command.y_vel) > SPEED_MAX) { command.y_vel = SPEED_MAX; }
        /* Check y */
        if      (fabs(command.x_vel) < SPEED_MIN) { command.x_vel = 0.0; }
        else if (fabs(command.x_vel) > SPEED_MAX) { command.x_vel = SPEED_MAX; }
        /* Check rotation */
        if      (fabs(command.w) < ROTATION_MIN) { command.w = 0.0; }
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

    ros::Rate fps(1);

    ROS_INFO_STREAM_NAMED("input", "Initializing NUM_CONTROLLERS controller(s)");
    for (auto &joy : joys) {
        joy.init();
    }


    while (ros::ok()) {
        ROS_INFO_STREAM_THROTTLE_NAMED(1, "input", "==========| Tick |==========");

        // Handle subscriber callbacks
        ros::spinOnce();

        for (auto &joy : joys) {

            joy.updateAutoProcess();

            if(joy.getTimer() < TIMEOUT_SECONDS){
                if (joy.msg) {
                    auto command = makeRobotCommand(joy, *joy.msg);
                    pub.publish(command);
                }

            }

//            else{
//                if(joy.processAuto)
//                    ROS_INFO_STREAM_NAMED("input", "running: " << joy.processAuto->running());
//                else
//                    ROS_INFO_STREAM_NAMED("input", "running: -");
//
//
//                if(!joy.isRunningAuto || !joy.processAuto || !joy.processAuto->running()){
//                    ROS_INFO_STREAM_NAMED("input", "Joy " << joy.robotID << " starting process..");
//
//                    boost::filesystem::path pathRosrun = bp::search_path("rosrun");
//                    std::vector<std::string> args;
//                    args.push_back("roboteam_tactics");
//                    args.push_back("TestX");
//                    args.push_back("rtt_bob/DemoAttacker");
//                    args.push_back("string:GetBall__aimAt=ourgoal");
//                    args.push_back("int:ROBOT_ID=" + std::to_string(joy.robotID));
//                    args.push_back("double:Kick__kickVel=8.0");
//
//                    std::string command = "";
//                    for (auto const& s : args) { command += s + " "; }
//
//                    ROS_INFO_STREAM_NAMED("input", command);
//
//                    joy.processAuto = bp::child(pathRosrun, args);
//
//                    joy.isRunningAuto=true;
//                }
//            }
        }
        fps.sleep();
    }

    return 0;
}