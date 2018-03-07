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
#include "diagnostic_msgs/DiagnosticArray.h"    // Used for listening to Joy on /diagnostics
#include "roboteam_msgs/RobotCommand.h"
#include "roboteam_msgs/RoleDirective.h"
#include "roboteam_msgs/World.h"
#include "roboteam_msgs/GeometryFieldSize.h"
#include "roboteam_msgs/GeometryData.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/world_analysis.h"
#include "roboteam_tactics/bt.hpp"

#include "joystick_enums.h"
#include "joystick_profiles.h"



namespace rtt {

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

        // ==== Variables related to automatically running a RoleNode
        std::time_t timeLastReceived = std::time(0);// added by anouk
        b::optional<bp::child> processAuto;         // Holds the joy_node auto process
        bool autoPlayOn = false;                    // Indicates if autoPlay should be started
        bool autoAttacker = true;
        bool dribblerOn = false;

        bool skillIsRunning = false;
        b::optional<bp::child> processSkill;


        std::map<Xbox360Controller, bool> btnState; // Holds the state of the buttons (pressed, not pressed)
        Vector2 speedState;                         // Holds the x-speed and y-speed of the robot.
        int genevaState;                            // Holds the state of the Geneva Drive. Range is [-2,2]
        bool controllerConnected = true;           // Holds if the corresponding controller is connected

        static int intSupplier;                     // Supplies ids to new instances of JoyEntry

        JoyEntry() : robotID{intSupplier}, MY_ID{intSupplier++} {
            this->profile = profile_slow;
        }

        void init(){
            ROS_INFO_STREAM(input << " connected to robot " << robotID);
            setToInput("js" + std::to_string(MY_ID));
        }

        void setToInput(std::string newInput) {
            ROS_INFO_STREAM(input << " now listening to " << newInput);

            // Remove the most recently received message to prevent stale values.
            msg = b::none;

            if(newInput == input)
                return;

            input = newInput;

            // Kill the current process
            if(process) {
                process->terminate();
                process = b::none;
            }

            // Kill the current subscriber
            if(subscriber) {
                subscriber->shutdown();
                subscriber = b::none;
            }

            if(input == "")
                return;

            // If this ever starts throwing weird compile time errors about
            // exe_cmd_init being deleted, just go to posix/basic_cmd.hpp
            // and mark the exe_cmd_init(const ...) function as default
            // See: https://github.com/klemens-morgenstern/boost-process/issues/21

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

        void stopAutoPlay(){
            // If the autoAttacker is also on (in addition to the keeper which is always on) ..
            // ..change to the strategy with only a keeper. The attacker will now have the Sleep Role..
            // .. but this role also avoids the keeper area, and possibly I'll add border avoidance to that.
            // Note that autoAttacker is initially set to true, such that this strategy starts immediately
            // PROBLEM: if the autoAttacker has started, and joystick takes over again, the defense area avoidance doesnt work anymore.
            if(autoAttacker) {
                boost::filesystem::path pathRosrun = bp::search_path("rosrun");
                std::vector<std::string> args;
                args.push_back("roboteam_tactics");
                args.push_back("TestX");
                args.push_back("rtt_jelle/DemoStratOnlyKeeper");

                ROS_INFO_STREAM(input << " starting rtt_jelle/DemoStratOnlyKeeper");
                processAuto = bp::child(pathRosrun, args);
                autoAttacker = false;
            }
            // // If process is running, kill it
            // if(processAuto){
            //     ROS_INFO_STREAM("Joy " << robotID << " terminating process..");
            //     processAuto->terminate();
            //     processAuto = b::none;      // This is required, just calling terminate doesn't cut it. Without this, processAuto->running() returns false when starting a new process
            // }
        }

        void startAutoPlay(){
            // // If no process is running, start it
            // // if(!processAuto || !processAuto->running()){
            //     ROS_INFO_STREAM("Joy " << robotID << " starting process..");

            // If autoAttacker was not on yet, start the strategy that includes the DemoAttacker
            if(!autoAttacker) {
                boost::filesystem::path pathRosrun = bp::search_path("rosrun");
                std::vector<std::string> args;
                args.push_back("roboteam_tactics");
                args.push_back("TestX");
                args.push_back("rtt_jelle/DemoStrat");

                //args.push_back("string:GetBall__aimAt=ourgoal");
                // args.push_back("int:ROBOT_ID=" + std::to_string(robotID));
                // args.push_back("double:Kick__kickVel=4.0");

                ROS_INFO_STREAM(input << " starting rtt_jelle/DemoStrat");
                processAuto = bp::child(pathRosrun, args);
                autoAttacker = true;
            }
        }

        void receiveJoyMsg(const sensor_msgs::JoyConstPtr &msg) {
            this->resetTimer(); // Reset the timer
            this->msg = *msg;   // Only store the newest messages
        }

        void setRobotID(int id){
            ROS_INFO_STREAM(input << " connected to robot " << id);
            robotID = id;
            // Reset robot velocity
            speedState.x = 0;
            speedState.y = 0;
            // Reset geneva drive
            genevaState = 0;
        }

        void nextJoystickProfile(){
            profileCounter = (profileCounter + 1) % NUM_JOYSTICK_PROFILES;
            profile = joystick_profiles[profileCounter];
            ROS_INFO_STREAM(input << " using profile " << profileCounter);
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

    void handleButtons(JoyEntry &joy, sensor_msgs::Joy const &msg){

        Xbox360Controller btn;

        /* ==== DPad control is off by default. While Y is pressed it is enabled ==== */
        if(getVal(msg.buttons, xbox360mapping.at(Xbox360Controller::Y))) {

            /* === Check if profile has to be modified === */
            btn = Xbox360Controller::Start;
            if(getVal(msg.buttons, xbox360mapping.at(btn)) > 0){    // If Start is pressed
                if(!joy.isPressed(btn)){                                // Check if it was already pressed before
                    joy.nextJoystickProfile();                              // If not, switch to next profile
                }
                joy.press(btn);                                         // Set button state to pressed
            }else{                                                  // Else
                joy.release(btn);                                       // Set button state to released
            }

            /* ==== Check if ID has to be switched lower ==== */
            btn = Xbox360Controller::DpadY;
            if(getVal(msg.axes, xbox360mapping.at(btn)) > 0){    // If DpadUp is pressed
                bool pressed = joy.isPressed(btn);                  // Store whether the button was pressed before
                joy.press(btn);                                     // Set button state to pressed
                if(!pressed) {                                      // Check if it was already pressed before
                    joy.setRobotID((joy.robotID + 1) % 16);             // If not, increment id
                }
            }else
            if(getVal(msg.axes, xbox360mapping.at(btn)) < 0){   // If DpadDown is pressed
                bool pressed = joy.isPressed(btn);                  // Store whether the button was pressed before
                joy.press(btn);                                     // Set button state to pressed
                if(!pressed) {                                      // Check if it was already pressed before
                    joy.setRobotID((joy.robotID + 15) % 16);            // If not, decrement id
                }
            }else                                               // If Dpad is not pressed
                joy.release(btn);                                   // Set button state to released
            /* ============================================== */

            /* ==== Rotate kicker (Geneva Drive)==== */
            btn = Xbox360Controller::DpadX;
            if(getVal(msg.axes, xbox360mapping.at(btn)) > 0){   // If DpadLeft is pressed
                if(!joy.isPressed(btn)) {                           // Check if it was already pressed before
                    joy.genevaState--;                                  // Turn Geneva Drive to the left
                    if(joy.genevaState < -2) {                         // Geneva state cannot go below -2 (such state does not exist)
                        joy.genevaState = -2;                               // Set the Geneva state back to -2
                    }
                }
                joy.press(btn);                                     // Set button state to pressed
            }else
            if(getVal(msg.axes, xbox360mapping.at(btn)) < 0){   // If DpadRight is pressed
                if(!joy.isPressed(btn)) {                           // Check if it was already pressed before
                    joy.genevaState++;                                  // Turn Geneva Drive to the right
                    if(joy.genevaState > 2) {                          // Geneva state cannot go above 2 (such state does not exist)
                        joy.genevaState = 2;                                // Set the Geneva state back to 2
                    }
                }
                joy.press(btn);                                     // Set button state to pressed
            }else                                                // If Dpad is not pressed
                joy.release(btn);                                   // Set button state to released
            /* ============================================== */

            /* ==== Enable / Disable autoPlay on LeftTrigger Click ==== */
            btn = Xbox360Controller::LeftStick;
            if(getVal(msg.buttons, xbox360mapping.at(btn)) > 0){   // If LeftStick is pressed
                if(!joy.isPressed(btn)){                                // Check if it was already pressed before
                    joy.autoPlayOn = !joy.autoPlayOn;                       // Toggle autoPlayOn
                    ROS_INFO_STREAM(joy.input << " autoPlay is now " << (joy.autoPlayOn ? "On" : "Off"));
                }
                joy.press(btn);                                     // Set button state to pressed
            }else{
                joy.release(btn);                                   // Set button state to released
            }
            /* ======================================================== */

        }
        /* ==== End DPad control ==== */




        /* ==== Enable / Disable getBall on A ==== */
        btn = Xbox360Controller::A;
        if(getVal(msg.buttons, xbox360mapping.at(btn)) > 0){    // If LeftStick is pressed
            if(!joy.isPressed(btn)){                                // Check if it was already pressed before

                if(joy.processSkill)                                    // If previous process is still running
                    joy.processSkill->terminate();                          // Terminate it
                joy.processSkill = b::none;                             // Remove the previous process

                boost::filesystem::path pathRosrun = bp::search_path("rosrun");
                std::vector<std::string> args;
                args.push_back("roboteam_tactics");
                args.push_back("TestX");
                args.push_back("rtt_jelle/DemoAttacker");
                args.push_back("int:ROBOT_ID=" + std::to_string(joy.robotID));
                args.push_back("bool:GetBall_A_dribblerOff=false");
                args.push_back("bool:GetBall_A_passOn=false");

                bt::Blackboard* bb = new bt::Blackboard();
                bb->SetInt("ROBOT_ID", joy.robotID);
                bb->SetBool("GetBall_A_dribblerOff", false);
                bb->SetBool("GetBall_A_passOn", false);


                ROS_INFO_STREAM(joy.input << " starting rtt_jelle/DemoAttacker");
                joy.processSkill = bp::child(pathRosrun, args);         // Start new process
                joy.skillIsRunning = true;

            }
            joy.press(btn);                                     // Set button state to pressed
        }else{
            if(joy.isPressed(btn)){                             // If button was pressed, but not anymore

                ROS_INFO_STREAM(joy.input << " stopping skill");

                if(joy.processSkill)                                // If previous process is still running
                    joy.processSkill->terminate();                      // Terminate it
                joy.processSkill = b::none;                         // Remove the previous process

                joy.skillIsRunning = false;
            }
            joy.release(btn);                                   // Set button state to released
        }
        /* ======================================================== */

    }

    roboteam_msgs::RobotCommand makeRobotCommand(JoyEntry &joy, sensor_msgs::Joy const &msg) {

        roboteam_msgs::RobotCommand command;

        command.id = joy.robotID;

        Xbox360Controller btn;

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
        double speedMultiplier = joy.profile.SPEED_MULTIPLIER;
        // If the dribbler is on, reduce the speed of the robot
        if(joy.dribblerOn){
            speedMultiplier = 0.5;
        }

        double speedXdesired = speedMultiplier * getVal(msg.axes, xbox360mapping.at(Xbox360Controller::LeftStickY));
        double speedXdiff = -joy.speedState.x + speedXdesired;
        joy.speedState.x += joy.profile.SMOOTH_FACTOR * speedXdiff;
        command.x_vel = joy.speedState.x;

        /* Smooth y */
        double speedYdesired = speedMultiplier * getVal(msg.axes, xbox360mapping.at(Xbox360Controller::LeftStickX));
        double speedYdiff = -joy.speedState.y + speedYdesired;
        joy.speedState.y += joy.profile.SMOOTH_FACTOR * speedYdiff;
        command.y_vel = joy.speedState.y;
        /* =================== */

        // ==== Rotation
        double rotationMultiplier = joy.profile.ROTATION_MULTIPLIER;// + joy.speedState.x * 2;         // maybe sqrt(x²+y²) ?
        // If the dribbler is on, reduce the rotation speed of the robot
        if(joy.dribblerOn) {
            rotationMultiplier = 2.2;
        }
        command.w = getVal(msg.axes, xbox360mapping.at(Xbox360Controller::RightStickX)) * rotationMultiplier;



        /* ==== Set Kicker ==== both right bumber and right trigger work ====*/ 
        btn = Xbox360Controller::RightBumper;
        /* If not pressed yet, returns 0. If not pressed anymore, returns 1. If pressed halfway, returns -0 */
        double RightTriggerVal = getVal(msg.axes, xbox360mapping.at(Xbox360Controller::RightTrigger));
        // If right trigger has not been pressed since starting the program
        if(RightTriggerVal == 0 && !std::signbit(RightTriggerVal)) {
			RightTriggerVal = 1;
		}
        if(getVal(msg.buttons, xbox360mapping.at(btn)) || RightTriggerVal<0.9){        // If RightBumper is pressed
            if(!joy.isPressed(btn))                                 // Check if it was already pressed before
                command.kicker = true;                                  // If not, activate kicker
            joy.press(btn);                                         // Set button state to pressed
        }else                                                   // If RightBumper is not pressed
            joy.release(btn);                                       // Set button state to released
        /* ==================== */


        // ==== Set dribbler ====
        btn = Xbox360Controller::LeftBumper;
        if(getVal(msg.buttons, xbox360mapping.at(btn))){
        	if(!joy.isPressed(btn)) {                               // Check if it was already pressed before
                joy.dribblerOn = !joy.dribblerOn;                       // If not, activate dribbler
            }
            joy.press(btn);                                         // Set button state to pressed
        } else {
        	joy.release(btn);                                       // Set button state to released
        }
        command.dribbler = joy.dribblerOn;
        /* ==================== */

        // ==== Set geneva drive state
        command.geneva_state = joy.genevaState;

        // ==== Set kicker velocity
        if(command.kicker) {
            command.kicker_vel = 2.5;
        }

        /* ==== Check speed boundaries ==== */
        /* === Check x === */
        // If speed is below -SPEED_MAX
        if(command.y_vel         < -joy.profile.SPEED_MAX                                 ) { command.y_vel = -joy.profile.SPEED_MAX; }
            // If speed is inbetween -SPEED_MAX and SPEED_MAX
        else if(-joy.profile.SPEED_MIN <  command.y_vel && command.y_vel < joy.profile.SPEED_MIN) { command.y_vel =  0.0; }
            // If speed is above SPEED_MAX
        else if(joy.profile.SPEED_MAX <  command.y_vel                                         ) { command.y_vel =  joy.profile.SPEED_MAX; }

        /* === Check y === */
        // If speed is below -SPEED_MAX
        if(command.x_vel         < -joy.profile.SPEED_MAX                                 ) { command.x_vel = -joy.profile.SPEED_MAX; }
            // If speed is inbetween -SPEED_MAX and SPEED_MAX
        else if(-joy.profile.SPEED_MIN <  command.x_vel && command.x_vel < joy.profile.SPEED_MIN) { command.x_vel =  0.0; }
            // If speed is above SPEED_MAX
        else if(joy.profile.SPEED_MAX <  command.x_vel                                         ) { command.x_vel =  joy.profile.SPEED_MAX; }

        /* Check rotation */
        if      (fabs(command.w) < joy.profile.ROTATION_MIN) { command.w = 0.0; }
        /* ================================ */

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

} // rtt



int main(int argc, char **argv) {
    using namespace rtt;


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


    bt::Blackboard bb;


    int tickCounter = 0;

    while (ros::ok()) {
        tickCounter++;
        ROS_INFO_STREAM_THROTTLE(5, "==========| Tick " << tickCounter << " |==========");

        // Handle subscriber callbacks
        ros::spinOnce();

        for (auto &joy : joys) {

            // If autoPlay is off, or timeout not yet reached
            if(!joy.autoPlayOn || joy.getTimer() <3 - 3 + TIMEOUT_SECONDS) { // #Liefde #LoveLife #RoboTeamLife

                // Stop autoplay if needed
                joy.stopAutoPlay();

                // If joystick message received
                if(joy.msg) {
                    // HandleButtons, such as ID switching
                    handleButtons(joy, *joy.msg);

                    // Send robotcommand if skill is not running atm
                    if(!joy.skillIsRunning) {
                        auto command = makeRobotCommand(joy, *joy.msg);
                        pub.publish(command);
                    }
                }

            }else
            // If timeout reached
            {
                // Start autoplay if needed
                joy.startAutoPlay();
            }
        }
        fps.sleep();
    }

    return 0;
}