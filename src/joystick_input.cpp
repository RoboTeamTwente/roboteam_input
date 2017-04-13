#include <string>
#include <cmath>
#include <boost/optional.hpp>
namespace b = ::boost;
#include <boost/process.hpp>
namespace bp = ::boost::process;
#include <map>

#include <string>
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "roboteam_msgs/RobotCommand.h"
#include "roboteam_msgs/World.h"
#include "roboteam_msgs/GeometryFieldSize.h"
#include "roboteam_msgs/GeometryData.h"
#include "roboteam_utils/Vector2.h"
#include <typeinfo>
#include <cmath>
#include "roboteam_utils/world_analysis.h"

namespace rtt {

struct JoystickMap {
    int xAxis;
    int yAxis;
    int rotationXAxis;
    int rotationYAxis;
    int dribblerAxis;
    int kickerAxis;
    
    // These two are needed because the gioteck has a button (i.e. 0/1 input) for the right trigger,
    // while the right trigger of the xbox is continuus (i.e. -30000 when depressed and +30000 when pressed)
    // I hate the real world
    int chipperAxis;
    int chipperContinuousAxis;
} ;

const int NUM_CONTROLLERS = 4;

const std::map<std::string, JoystickMap> joystickTypeMap = {
    {
        {"xbox", {
            0,  // xAxis
            1,  // yAxis
            3,  // rotationXAxis
            4,  // rotationYAxis
            4,  // dribblerAxis
            5,  // kickerAxis
            -1, // chipperAxis
            5   // chipperContinuousAxis
        }},
        {"playstation", {
            0,  // xAxis
            1,  // yAxis
            2,  // rotationXAxis
            3,  // rotationYAxis
            10, // dribblerAxiscatkin
            11, // kickerAxis
            -1, // chipperAxis
            5   // chipperContinuousAxis
        }},
        {"gioteck", {
            0, // yAxis
            1, // xAxis
            2, // rotationXAxis
            3, // rotationYAxis
            4, // dribblerAxis
            5, // kickerAxis
            7, // chipperAxis
            -1 // chipperContinuousAxis
        }}
    }
};

struct JoyEntry {
    b::optional<bp::child>        process;
    b::optional<ros::Subscriber>  subscriber;

    b::optional<sensor_msgs::Joy> msg;

    std::string input;
    int robotID;
    std::string joyType;

    static int intSupplier;
    int const MY_ID;

    JoyEntry() : robotID{-1}, MY_ID{intSupplier++} {}

    void setToInput(std::string newInput) {
        if (newInput != input) {
            input = newInput;

            // Remove the most recently received message to prevent stale values.
            msg = b::none;

            // Kill your darlings if needed
            if (process) {
                process = b::none;
            }

            if (subscriber) {
                subscriber->shutdown();
                subscriber = b::none;
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
            std::cout << "Starting process\n";
            process = bp::child(
                bp::search_path("roslaunch"),
                "roboteam_input",
                "joy_node.launch",
                "jsTarget:=" + input,
                bp::std_out > bp::null
            );

            // Make new subscriber
            ros::NodeHandle n;
            subscriber = n.subscribe(input, 1, &JoyEntry::receiveJoyMsg, this);

        }

    }
    
    void receiveJoyMsg(const sensor_msgs::JoyConstPtr& msg) {
        // Only store the newest messages
        this->msg = *msg;
    }

    void update() {
        std::string newInput;
        int newRobotID = -1;
        std::string newJoyType;

        ros::param::get("input" + std::to_string(MY_ID) + "/input", newInput);
        ros::param::get("input" + std::to_string(MY_ID) + "/robot", newRobotID);
        ros::param::get("input" + std::to_string(MY_ID) + "/joyType", newJoyType);

        setToInput(newInput);

        robotID = newRobotID;

        joyType = newJoyType;
    }
} ;

int JoyEntry::intSupplier = 0;

std::array<JoyEntry, NUM_CONTROLLERS> joys;

roboteam_msgs::World lastWorld;
bool receivedFirstWorldMsg = false;
void callback_world_state(const roboteam_msgs::WorldConstPtr& world) {
    lastWorld = *world;
    receivedFirstWorldMsg = true;
}

roboteam_msgs::GeometryFieldSize lastField;
bool receivedFirstGeomMsg = false;
void callback_world_geometry(const roboteam_msgs::GeometryDataConstPtr& geom) {
    lastField = geom->field;

    receivedFirstGeomMsg = true;

    std::cout << "Got geometry update!\n";
}

double cleanAngle(double angle){
    if (angle <= -M_PI){
        return fmod(angle-M_PI, (2*M_PI))+M_PI;
    } else if(angle > M_PI){
        return fmod(angle+M_PI, (2*M_PI))-M_PI;
    } else {
        return angle;
    }
}

class RobotPosController {
public:
    Vector2 positionController(Vector2 position, Vector2 target) {
        double maxSpeed = 4.0;

        double pGain = 2;
        double dGain = 4;
        double iGain = 0;

        auto posError = target - position;

        integrator = integrator + posError;

        Vector2 requiredSpeed = posError*pGain;
        requiredSpeed = requiredSpeed + (posError - prevPosError) * dGain;
        requiredSpeed = requiredSpeed + integrator * iGain;

        prevPosError = posError;

        if (requiredSpeed.length() > maxSpeed) {
            requiredSpeed = requiredSpeed.normalize() * maxSpeed;
        }

        return requiredSpeed;
    }


    double rotationController(double angle, double target) {
        double pGainRot = 4.0;
        double dGainRot = 2.0;
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

} ;

Vector2 worldToRobotFrame(Vector2 requiredv, double rotation) {
    Vector2 robotRequiredv;
    robotRequiredv.x=requiredv.x*cos(-rotation)-requiredv.y*sin(-rotation);
    robotRequiredv.y=requiredv.x*sin(-rotation)+requiredv.y*cos(-rotation);
	return robotRequiredv;
}

std::map<int, double> lastAngles;

template <
    typename T
>
T getVal(const std::vector<T> & values, size_t index) {
    if (index < values.size()) {
        return values[index];
    }
    return T(0);
}

double speedValue(const double input) {
    double speedValue;
    if (input < 0.8) {
        speedValue = 0.1 * input;
    } else {
        speedValue = 9 * input - 9;
    }
    return speedValue;
}

bool kickerhack=false;

roboteam_msgs::RobotCommand makeRobotCommand(JoyEntry& joy, sensor_msgs::Joy const & msg) {
    roboteam_msgs::World world = lastWorld;

    const JoystickMap &joystickMap = joystickTypeMap.at(joy.joyType);

    // Get the robot id
    int ROBOT_ID = joy.robotID;

    roboteam_msgs::RobotCommand command;
    command.id = ROBOT_ID;
    command.y_vel = getVal(msg.axes, joystickMap.xAxis)*4;
    command.x_vel = getVal(msg.axes, joystickMap.yAxis)*4;

    if(fabs(command.x_vel) < 0.1) {
        command.x_vel=0;
    }
    if(fabs(command.y_vel) < 0.1) {
        command.y_vel=0;
    }

    command.w = getVal(msg.axes, joystickMap.rotationXAxis)*2;
    command.dribbler = getVal(msg.buttons, joystickMap.dribblerAxis) > 0;

    command.kicker = getVal(msg.buttons, joystickMap.kickerAxis) > 0;
    if (command.kicker) { 
        std::cout << "[RobotHub] Kicker command\n";
        command.kicker_forced = true;

        if(kickerhack){
            command.kicker_vel = 4.0;
            kickerhack=false;
        } else {
            command.kicker_vel = 5.0;
            kickerhack=true;
        }
    }

    if (joystickMap.chipperAxis != -1) {
        command.chipper = getVal(msg.buttons, joystickMap.chipperAxis) > 0;
    } else if (joystickMap.chipperContinuousAxis != -1) {
        command.chipper = getVal(msg.axes, joystickMap.chipperContinuousAxis) <= -0.5;
    }

    if (command.chipper) {
        std::cout << "[RobotHub] Chipper command\n";
        command.chipper_forced = true;
        command.chipper_vel = roboteam_msgs::RobotCommand::MAX_CHIPPER_VEL;
    }

    return command;
}

class UserKeeperController {
public:

    UserKeeperController(double speed = 2.0) : speed{speed} {}

    double speed;
    double keeperSpan = 0;
    double keeperDist = 0.5;
    RobotPosController controller;

    roboteam_msgs::RobotCommand makeRobotCommand(JoyEntry& joy, sensor_msgs::Joy const & msg) {
        roboteam_msgs::World world = lastWorld;

        const JoystickMap &joystickMap = joystickTypeMap.at(joy.joyType);

        // Get the robot id
        int ROBOT_ID = joy.robotID;
        
        int const FPS = 60;
        double const FIELD_W = lastField.field_width;
        double const FIELD_L = lastField.field_length ;

        // Distance from goal forward
        keeperDist += getVal(msg.axes, joystickMap.yAxis) * (speed * (1.0 / FPS));

        //Disance from goal sideways
        keeperSpan += getVal(msg.axes, joystickMap.xAxis) * (speed * (1.0 / FPS));

        if (keeperSpan < -FIELD_W / 2) keeperSpan = -FIELD_W / 2;
        if (keeperSpan > FIELD_W / 2) keeperSpan = FIELD_W / 2;
        if (keeperDist < -FIELD_L / 2) keeperDist = -FIELD_L / 2;
        if (keeperDist > FIELD_L / 2) keeperDist = FIELD_L / 2;

        Vector2 keeperPos = Vector2(keeperDist, keeperSpan);

        keeperPos = keeperPos + Vector2(lastField.field_length / -2, 0);

        roboteam_msgs::WorldRobot robot;

        if (auto robotOpt = lookup_our_bot(ROBOT_ID, &world)) {
            robot = *robotOpt;
        } else {
            roboteam_msgs::RobotCommand r;
            r.id = ROBOT_ID;
            std::cout << "[RobotHub] Keeper robot with id " << ROBOT_ID << " not found\n";
            return r;
        }

        Vector2 requiredSpeed = worldToRobotFrame(controller.positionController(robot.pos, keeperPos), robot.angle);

        roboteam_msgs::RobotCommand command;
        command.id = ROBOT_ID;
        command.y_vel = requiredSpeed.y;
        command.x_vel = requiredSpeed.x;

        command.w = getVal(msg.axes, joystickMap.rotationXAxis)*2;
        command.dribbler = getVal(msg.buttons, joystickMap.dribblerAxis) > 0;

        command.kicker = getVal(msg.buttons, joystickMap.kickerAxis) > 0;
        if (command.kicker) { 
            std::cout << "[RobotHub] Kicker command\n";
            command.kicker_forced = true;

            if(kickerhack){
                command.kicker_vel = 4.0;
                kickerhack=false;
            } else {
                command.kicker_vel = 5.0;
                kickerhack=true;
            }
        }

        if (joystickMap.chipperAxis != -1) {
            command.chipper = getVal(msg.buttons, joystickMap.chipperAxis) > 0;
        } else if (joystickMap.chipperContinuousAxis != -1) {
            command.chipper = getVal(msg.axes, joystickMap.chipperContinuousAxis) <= -0.5;
        }

        if (command.chipper) {
            std::cout << "[RobotHub] Chipper command\n";
            command.chipper_forced = true;
            command.chipper_vel = roboteam_msgs::RobotCommand::MAX_CHIPPER_VEL;
        }

        return command;
    }
} ;

} // rtt

int main(int argc, char **argv) {
    using namespace rtt;

    ros::init(argc, argv, "roboteam_input");
    ros::NodeHandle n;

    auto worldSub = n.subscribe<roboteam_msgs::World>("world_state", 10, callback_world_state);
    auto geomSub = n.subscribe<roboteam_msgs::GeometryData>("vision_geometry", 1, callback_world_geometry);
    auto pub = n.advertise<roboteam_msgs::RobotCommand>("robotcommands", 10);

    UserKeeperController keeperController;

    ros::Rate fps(60);

    while (ros::ok()) {
        for (auto& joy : joys) {
            joy.update();

            if (joy.msg) {
                // TODO: Make it parameter configurable whether or not keeper is on
                auto command = keeperController.makeRobotCommand(joy, *joy.msg);
                pub.publish(command);
            }
            
            // joy.msg = boost::none;
        }

        fps.sleep();

        ros::spinOnce();
    }

    return 0;
}
