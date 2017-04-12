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
        std::cout << "Received a joy message for " << MY_ID << "\n";
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

double cleanAngle(double angle){
    if (angle <= -M_PI){
        return fmod(angle-M_PI, (2*M_PI))+M_PI;
    } else if(angle > M_PI){
        return fmod(angle+M_PI, (2*M_PI))-M_PI;
    } else {
        return angle;
    }
}

Vector2 prevPosError;
Vector2 integrator;

Vector2 positionController(Vector2 posError) {
    double maxSpeed = 2.0;

    double pGain = 2;
    double dGain = 4;
    double iGain = 0.001;

    integrator = integrator + posError;

    Vector2 requiredSpeed = posError*pGain;
    requiredSpeed = requiredSpeed + (posError - prevPosError) * dGain;
    requiredSpeed = requiredSpeed + integrator * iGain;

    prevPosError = posError;

    if (requiredSpeed.length() > 1) {
        requiredSpeed = requiredSpeed.normalize();
    }

    // Slow down once we get close to the goal, otherwise go at maximum speed
    // if (posError.length() > 0.5) { // TODO: compute this distance depending on the maximum speed, so that there is no overshoot
        // if (requiredSpeed.length() > 0) {
            // requiredSpeed = requiredSpeed.scale(1/requiredSpeed.length() * maxSpeed);
        // } else {
            // requiredSpeed = Vector2(0.0, 0.0);
        // }
    // }
    return requiredSpeed;
}

double prevAngleError = 0;

double rotationController(double angleError) {
    double pGainRot = 4.0;
    double dGainRot = 2.0;
    double maxRotSpeed = 7.0;

    angleError = cleanAngle(angleError);
    double requiredRotSpeed = angleError * pGainRot;
    requiredRotSpeed += (angleError - prevAngleError) * dGainRot;

    prevAngleError = angleError;

    // if (fabs(requiredRotSpeed) > maxRotSpeed) {
        // requiredRotSpeed = requiredRotSpeed / fabs(requiredRotSpeed) * maxRotSpeed;
    // }

    return requiredRotSpeed;
}


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
    // Everything is within the roboteam_input node, in groups going from
    // input0 to inputN.
    roboteam_msgs::World world = lastWorld;

    // const std::string group = "input" + std::to_string(inputNum);

    // Get the joystick type
    // std::string joyType = "playstation";
    // ros::param::get(group + "/joyType", joyType);
    const JoystickMap &joystickMap = joystickTypeMap.at(joy.joyType);

    // Get the robot id
    int ROBOT_ID = joy.robotID;
    // ros::param::get(group + "/robot", ROBOT_ID);

    // Vector2 target_speed = Vector2(
        // -getVal(msg.axes, joystickMap.xAxis),
        // getVal(msg.axes, joystickMap.yAxis)
    // );

    /*
    if (target_speed.length() < 0.9) {
        target_speed = target_speed.scale(0.3);
    } else {
        target_speed = (target_speed - target_speed.scale((0.9 - 0.9*0.3)/target_speed.length())).scale(2);
    }

    double myAngle = world.us.at(ROBOT_ID).angle;
    double targetAngle;

    Vector2 requiredSpeed = positionController(target_speed);
    Vector2 requiredSpeedWF = worldToRobotFrame(requiredSpeed, myAngle);

    Vector2 myPos(world.us.at(ROBOT_ID).pos);
    Vector2 ballPos(world.ball.pos);
    double distanceToBall = (ballPos - myPos).length();
    if (distanceToBall < 1) {
        requiredSpeedWF = requiredSpeedWF.scale(distanceToBall + 0.2);
    }
    targetAngle = (ballPos - myPos).angle();

    double requiredRotSpeed = rotationController(targetAngle - myAngle);

    */

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

    //command.dribbler = true;
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
        std::cout << "Continuous value: " << getVal(msg.axes, joystickMap.chipperContinuousAxis);
        command.chipper = getVal(msg.axes, joystickMap.chipperContinuousAxis) <= -0.5;
    }

    if (command.chipper) {
        std::cout << "[RobotHub] Chipper command\n";
        command.chipper_forced = true;
        command.chipper_vel = roboteam_msgs::RobotCommand::MAX_CHIPPER_VEL;
    }

    return command;
}

// TODO: Probably want to make a small class/struct for this
// TODO: Same goes for the PID controller
double keeperDir = M_PI * 0.5;
double keeperDist = 0.5;

roboteam_msgs::RobotCommand makeKeeperRobotCommand(JoyEntry& joy, sensor_msgs::Joy const & msg) {
    // Everything is within the roboteam_input node, in groups going from
    // input0 to inputN.
    roboteam_msgs::World world = lastWorld;

    // const std::string group = "input" + std::to_string(inputNum);

    // Get the joystick type
    // std::string joyType = "playstation";
    // ros::param::get(group + "/joyType", joyType);
    const JoystickMap &joystickMap = joystickTypeMap.at(joy.joyType);

    // Get the robot id
    int ROBOT_ID = joy.robotID;
    
    int const FPS = 60;
    // TODO: This should be m/s, not rads. Now if you're far away you will go faster!
    double radsPerSecond = 0.5;
    keeperDir -= getVal(msg.axes, joystickMap.xAxis) * (radsPerSecond * (2 * M_PI)) * (1.0 / FPS);
    keeperDist += getVal(msg.axes, joystickMap.yAxis) * (radsPerSecond * (2 * M_PI)) * (1.0 / FPS);

    if (keeperDir < 0) keeperDir = 0;
    if (keeperDir > M_PI) keeperDir = M_PI;
    if (keeperDist < 0.1) keeperDist = 0.1;
    if (keeperDist > 2) keeperDist = 2;

    Vector2 keeperPos = Vector2(keeperDist, 0).rotate((M_PI - keeperDir) - (0.5 * M_PI));
    keeperPos = keeperPos + Vector2(-4.5, 0);

    roboteam_msgs::WorldRobot robot;

    if (auto robotOpt = lookup_our_bot(ROBOT_ID, &world)) {
        robot = *robotOpt;
    } else {
        roboteam_msgs::RobotCommand r;
        r.id = ROBOT_ID;
        std::cout << "[RobotHub] Keeper robot with id " << ROBOT_ID << " not found\n";
        return r;
    }

    Vector2 currentPos = robot.pos;

    double targetAngle;

    Vector2 requiredSpeed = positionController(keeperPos - currentPos);
    Vector2 requiredSpeedWF = worldToRobotFrame(requiredSpeed, robot.angle);

    targetAngle = (Vector2(world.ball.pos) - currentPos).angle();

    double requiredRotSpeed = rotationController(targetAngle - robot.angle);

    roboteam_msgs::RobotCommand command;
    command.id = ROBOT_ID;
    command.y_vel = requiredSpeedWF.y;
    command.x_vel = requiredSpeedWF.x;

    // command.w = requiredRotSpeed;
    command.w = getVal(msg.axes, joystickMap.rotationXAxis)*2;
    command.dribbler = getVal(msg.buttons, joystickMap.dribblerAxis) > 0;

    //command.dribbler = true;
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

} // rtt

int main(int argc, char **argv) {
    using namespace rtt;

    ros::init(argc, argv, "roboteam_input");
    ros::NodeHandle n;

    auto world_sub = n.subscribe<roboteam_msgs::World>("world_state", 10, callback_world_state);
    auto pub = n.advertise<roboteam_msgs::RobotCommand>("robotcommands", 10);

    ros::Rate fps(60);

    while (ros::ok()) {
        for (auto& joy : joys) {
            joy.update();

            if (joy.msg) {
                // TODO: Make it parameter configurable whether or not keeper is on
                auto command = makeRobotCommand(joy, *joy.msg);
                pub.publish(command);
            }
            
            // joy.msg = boost::none;
        }

        fps.sleep();

        ros::spinOnce();
    }

    return 0;
}
