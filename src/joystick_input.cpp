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

namespace rtt {

    const int NUM_CONTROLLERS = 2;

    /* Maps the buttons, triggers, and sticks from the Xbox 360 controller to the messages received from joy_node*/
    const std::map<Xbox360Controller, int> xbox360mapping = {
            { Xbox360Controller::LeftStickX   , 0  },  // Drive forward / backward
            { Xbox360Controller::LeftStickY   , 1  },  // Strafe left / right
            { Xbox360Controller::RightStickX  , 3  },  // Turn left / right
            { Xbox360Controller::LeftBumper   , 4  },  // Dribbler
            { Xbox360Controller::RightBumper  , 5  },  // Kicker
            { Xbox360Controller::RightTrigger , 5  },  // Turbo
            { Xbox360Controller::DpadLeft     , 11 },  // ID -= 1
            { Xbox360Controller::DpadRight    , 12 },  // ID += 1
    };

    struct JoyEntry {
        b::optional<bp::child> process;             // Holds the joy_node process
        b::optional<ros::Subscriber> subscriber;    // Subscribes to the joy_node topic (/js0, /js1 etc etc)
        b::optional<sensor_msgs::Joy> msg;          // Holds the latest message from the joy_node topic

        std::string input;                          // js0
        int robotID;                                // 1 - 16
        const int MY_ID;                            // Holds the unique id

        std::map<Xbox360Controller, bool> btnState; // Holds the state of the buttons (pressed, not pressed)
        Vector2 speedState;                         // Holds the x-speed and y-speed of the robot

        static int intSupplier;                     // Supplies ids to new instances of JoyEntry

        JoyEntry() : robotID{intSupplier}, MY_ID{intSupplier++} {}

        void init(){
            std::cout << "[JoyEntry::init]" << std::endl;
            setToInput("js" + std::to_string(MY_ID));
        }

        void setToInput(std::string newInput) {

            std::cout << "[JoyEntry::setToInput] " << newInput << std::endl;
            std::cout << "[JoyEntry::setToInput] previous: " << input << std::endl;

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

        /*void update() {
            std::cout << "[JoyEntry::update] Updating" << std::endl;

            std::string newInput;
            int newRobotID = -1;
            std::string newJoyType;
            std::string newMode;

            //ros::param::get("input" + std::to_string(MY_ID) + "/input", newInput);
            //ros::param::get("input" + std::to_string(MY_ID) + "/robot", newRobotID);
            //ros::param::get("input" + std::to_string(MY_ID) + "/joyType", newJoyType);
            //ros::param::get("input" + std::to_string(MY_ID) + "/mode", newMode);

            //setToInput(newInput);

            //robotID = newRobotID;

            //joyType = newJoyType;

            //mode = newMode;
        }*/

        /*template<typename T>
        void setParam(string key, T value){
            key = "input" + std::to_string(MY_ID) + "/" + key;
            std::cout << "[JoyEntry::setParam] " << key << " -> " << value << std::endl;
            ros::param::set(key, value);

            // Reset robot velocity
            speedState.x = 0;
            speedState.y = 0;

        }*/

        void setRobotID(int id){
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
        bool isPressed(Xbox360Controller btn){
            return btnState.at(btn);
        }
    };

    int JoyEntry::intSupplier = 0;

    std::array<JoyEntry, NUM_CONTROLLERS> joys;

    roboteam_msgs::World lastWorld;

    void callback_world_state(const roboteam_msgs::WorldConstPtr &world) {
        lastWorld = *world;
    }

    roboteam_msgs::GeometryFieldSize lastField;

    void callback_world_geometry(const roboteam_msgs::GeometryDataConstPtr &geom) {
        lastField = geom->field;
        std::cout << "Got geometry update!\n";
    }


    /*
    double cleanAngle(double angle) {
        if (angle <= -M_PI) {
            return fmod(angle - M_PI, (2 * M_PI)) + M_PI;
        } else if (angle > M_PI) {
            return fmod(angle + M_PI, (2 * M_PI)) - M_PI;
        } else {
            return angle;
        }
    }



    class RobotPosController {
    public:
        Vector2 positionController(Vector2 position, Vector2 target) {
            std::cout << "[RobotPosController.positionController] " << position << " " << target << std::endl;
            double maxSpeed = 4.0;

            double pGain = 2;
            double dGain = 4;
            double iGain = 0;

            auto posError = target - position;

            integrator = integrator + posError;

            Vector2 requiredSpeed = posError * pGain;
            requiredSpeed = requiredSpeed + (posError - prevPosError) * dGain;
            requiredSpeed = requiredSpeed + integrator * iGain;

            prevPosError = posError;

            if (requiredSpeed.length() > maxSpeed) {
                requiredSpeed = requiredSpeed.normalize() * maxSpeed;
            }

            return requiredSpeed;
        }


        double rotationController(double angle, double target) {
            std::cout << "[RobotPosController.rotationController] " << angle << " " << target << std::endl;
            double pGainRot = 4.0;
            double dGainRot = 6.0;
            double maxRotSpeed = 7.0;

            auto angleError = target - angle;

            angleError = cleanAngle(angleError);
            double requiredRotSpeed = angleError * pGainRot;
            requiredRotSpeed += (angleError - prevAngleError) * dGainRot;

            prevAngleError = angleError;

            if (fabs(requiredRotSpeed) > maxRotSpeed) {
                requiredRotSpeed = requiredRotSpeed / fabs(requiredRotSpeed) * maxRotSpeed;
            }

            return requiredRotSpeed;
        }

    private:
        Vector2 prevPosError;
        Vector2 integrator;
        double prevAngleError = 0;

    };

    Vector2 worldToRobotFrame(Vector2 requiredv, double rotation) {
        std::cout << "[worldToRobotFrame] " << requiredv << " " << rotation << std::endl;
        Vector2 robotRequiredv;
        robotRequiredv.x = requiredv.x * cos(-rotation) - requiredv.y * sin(-rotation);
        robotRequiredv.y = requiredv.x * sin(-rotation) + requiredv.y * cos(-rotation);
        return robotRequiredv;
    }
    */

    template<typename T> T getVal(const std::vector<T> &values, int index) {
        if (index < values.size()) {
            return values[index];
        }
        return T(0);
    }

    const double SMOOTH_FACTOR = 0.05;
    const double SPEED_MULTIPLIER = 2;
    const double SPEED_MIN = 0.5;
    const double SPEED_MAX = 5;

    const double ROTATION_MULTIPLIER = 5;
    const double ROTATION_MIN = 1.5;


    roboteam_msgs::RobotCommand makeRobotCommand(JoyEntry &joy, sensor_msgs::Joy const &msg) {
        std::cout << "[makeRobotCommand] MY_ID: " << joy.MY_ID << " input: " << joy.input << " robotID: " << joy.robotID << std::endl;

        roboteam_msgs::RobotCommand command;

        command.id = joy.robotID;

        /* ==== Check if ID has to be switched lower ==== */
        Xbox360Controller btn;
        btn = Xbox360Controller::DpadLeft;
        if(getVal(msg.buttons, xbox360mapping.at(btn))){        // If DpadLeft is pressed
            if(!joy.isPressed(btn))                                 // Check if it was already pressed before
                joy.setRobotID((joy.robotID + 15) % 16);         // If not, decrement id
            joy.press(btn);                                         // Set button state to pressed
        }else                                                   // If DpadLeft is not pressed
            joy.release(btn);                                       // Set button state to released

        btn = Xbox360Controller::DpadRight;
        if(getVal(msg.buttons, xbox360mapping.at(btn))){        // If DpadRight is pressed
            if(!joy.isPressed(btn))                                 // Check if it was already pressed before
                joy.setRobotID((joy.robotID + 1) % 16);          // If not, increment id
            joy.press(btn);                                         // Set button state to pressed
        }else                                                   // If DpadRight is not pressed
            joy.release(btn);                                       // Set button state to released
        /* ============================================== */



        /* ==== Smoothing ==== */
        double speedmult = SPEED_MULTIPLIER + (1 - getVal(msg.axes, xbox360mapping.at(Xbox360Controller::RightTrigger)));
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
        double rot_mult = ROTATION_MULTIPLIER + joy.speedState.x * 2;         // maybe sqrt(x²+y²) ?
        command.w = getVal(msg.axes, xbox360mapping.at(Xbox360Controller::RightStickX)) * rot_mult;

        // ==== Set dribbler and kicker
        command.dribbler = getVal(msg.buttons, xbox360mapping.at(Xbox360Controller::LeftBumper)) > 0;
        command.kicker   = getVal(msg.buttons, xbox360mapping.at(Xbox360Controller::RightBumper)) > 0;

        if (command.kicker) {
            std::cout << "[makeRobotCommand] Kicker command\n";
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

        std::cout << "[makeRobotCommand] x_vel: " << command.x_vel << std::endl;
        std::cout << "[makeRobotCommand] y_vel: " << command.y_vel << std::endl;
        std::cout << "[makeRobotCommand] w    : " << command.w     << std::endl;

        return command;
    }

    /*
    double keeperSpeed = 2.0;

    std::map<int, RobotPosController> angleControllers;
    std::map<int, double> targetAngles;

    roboteam_msgs::RobotCommand makeKeeperRobotCommand(JoyEntry &joy, sensor_msgs::Joy const &msg) {
        roboteam_msgs::World world = lastWorld;

        const JoystickMap &joystickMap = joystickTypeMap.at(joy.joyType);

        // Get the robot id
        int ROBOT_ID = joy.robotID;

        int const FPS = 5;
        double const FIELD_W = lastField.field_width;
        double const FIELD_L = lastField.field_length;

        double keeperYVel = getVal(msg.axes, joystickMap.yAxis) * keeperSpeed;

        //Disance from goal sideways
        double keeperXVel = getVal(msg.axes, joystickMap.xAxis) * keeperSpeed;

        Vector2 keeperPos = Vector2(keeperYVel, keeperXVel);

        roboteam_msgs::WorldRobot robot;

        if (auto robotOpt = lookup_our_bot(ROBOT_ID, &world)) {
            robot = *robotOpt;
        } else {
            roboteam_msgs::RobotCommand r;
            r.id = ROBOT_ID;
            std::cout << "[RobotHub] Keeper robot with id " << ROBOT_ID << " not found\n";
            return r;
        }

        Vector2 requiredSpeed = worldToRobotFrame(keeperPos, robot.angle);

        if (targetAngles.find(joy.MY_ID) == targetAngles.end()) {
            targetAngles[joy.MY_ID] = robot.angle;
        }

        targetAngles[joy.MY_ID] += getVal(msg.axes, joystickMap.rotationXAxis) * (2 * M_PI) * 0.6 * (1.0 / FPS);

        double requiredW = angleControllers[joy.MY_ID].rotationController(robot.angle, targetAngles[joy.MY_ID]);

        roboteam_msgs::RobotCommand command;
        command.id = ROBOT_ID;
        command.y_vel = requiredSpeed.y;
        command.x_vel = requiredSpeed.x;

        command.w = requiredW;
        command.dribbler = getVal(msg.buttons, joystickMap.dribblerAxis) > 0;

        command.kicker = getVal(msg.buttons, joystickMap.kickerAxis) > 0;
        if (command.kicker) {
            std::cout << "[RobotHub] Kicker command\n";
        }

        if (joystickMap.chipperAxis != -1) {
            command.chipper = getVal(msg.buttons, joystickMap.chipperAxis) > 0;
        } else if (joystickMap.chipperContinuousAxis != -1) {
            command.chipper = getVal(msg.axes, joystickMap.chipperContinuousAxis) <= -0.5;
        }

        if (command.chipper) {
            std::cout << "[RobotHub] Chipper command\n";
            command.chipper_vel = roboteam_msgs::RobotCommand::MAX_CHIPPER_VEL;
        }

        return command;
    }*/

} // rtt

int main(int argc, char **argv) {
    using namespace rtt;

    ros::init(argc, argv, "roboteam_input");
    ros::NodeHandle n;

    // Subscribe to world_state
    ros::Subscriber worldSub = n.subscribe<roboteam_msgs::World>("world_state", 10, callback_world_state);
    // Subscribe to world_geometry
    ros::Subscriber geomSub = n.subscribe<roboteam_msgs::GeometryData>("vision_geometry", 10, callback_world_geometry);
    // Publish on robotcommands
    ros::Publisher pub = n.advertise<roboteam_msgs::RobotCommand>("robotcommands", 10);

    ros::Rate fps(4);

    for (auto &joy : joys) {
        joy.init();
    }

    while (ros::ok()) {

        std::time_t t = std::time(0);
        std::cout << "\n=======================================" << t
                  << "============================================\n";

        for (auto &joy : joys) {
            std::cout << "\n----------------------------------\n";

            if (joy.msg) {
                auto command = makeRobotCommand(joy, *joy.msg);
                pub.publish(command);
            }

            /*
                if (joy.mode == "our keeper") {
                    auto command = makeKeeperRobotCommand(joy, *joy.msg);
                    pub.publish(command);
                } else
                if (joy.mode == "normal") {
                    auto command = makeRobotCommand(joy, *joy.msg);
                    pub.publish(command);
                } else {
                    auto command = makeRobotCommand(joy, *joy.msg);
                    pub.publish(command);
                }
            }*/

            // joy.msg = boost::none;
        }

        fps.sleep();

        ros::spinOnce();
    }

    return 0;
}