#include <string>
#include <cmath>
#include <boost/optional.hpp>
#include <map>

#include "joystick_input.h"
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "roboteam_msgs/RobotCommand.h"
#include "roboteam_msgs/World.h"
#include "roboteam_utils/Vector2.h"
#include <typeinfo>
#include <cmath>

namespace rtt {

struct JoystickMap {
    int xAxis;
    int yAxis;
    int rotationXAxis;
    int rotationYAxis;
    int dribblerAxis;
    int kickerAxis;
} ;

const int NUM_CONTROLLERS = 4;

const std::map<std::string, JoystickMap> joystickTypeMap = {
    {
        {"xbox", {
            0, // xAxis
            1, // yAxis
            3, // rotationXAxis
            4, // rotationYAxis
            4, // dribblerAxis
            5 // kickerAxis
        }},
        {"playstation", {
            0, // xAxis
            1, // yAxis
            2, // rotationXAxis
            3, // rotationYAxis
            10, // dribblerAxiscatkin
            11 // kickerAxis
        }},
        {"gioteck", {
            0, // yAxis
            1, // xAxis
            2, // rotationXAxis
            3, // rotationYAxis
            4, // dribblerAxis
            5 // kickerAxis
        }}
    }
};

std::array<sensor_msgs::Joy, NUM_CONTROLLERS> joyMsgs;

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

roboteam_utils::Vector2 positionController(roboteam_utils::Vector2 posError) {
    double maxSpeed = 2.0;
    double P_Gain = 2.0;
    roboteam_utils::Vector2 requiredSpeed = posError*P_Gain;

    // Slow down once we get close to the goal, otherwise go at maximum speed
    if (posError.length() > 0.5) { // TODO: compute this distance depending on the maximum speed, so that there is no overshoot
        if (requiredSpeed.length() > 0) {
            requiredSpeed = requiredSpeed.scale(1/requiredSpeed.length() * maxSpeed);
        } else {
            requiredSpeed = roboteam_utils::Vector2(0.0, 0.0);
        }
    }
    return requiredSpeed;
}

double rotationController(double angleError) {
    double pGainRot = 6.0;
    double maxRotSpeed = 7.0;

    angleError = cleanAngle(angleError);
    double requiredRotSpeed = angleError * pGainRot;

    if (fabs(requiredRotSpeed) > maxRotSpeed) {
        requiredRotSpeed = requiredRotSpeed / fabs(requiredRotSpeed) * maxRotSpeed;
    }
    return requiredRotSpeed;
}


void receiveJoyMsg(int inputNum, const sensor_msgs::JoyConstPtr& msg) {
    // Only store the newest messages
    joyMsgs[inputNum] = *msg;
}

roboteam_utils::Vector2 worldToRobotFrame(roboteam_utils::Vector2 requiredv, double rotation) {
    roboteam_utils::Vector2 robotRequiredv;
    robotRequiredv.x=requiredv.x*cos(-rotation)-requiredv.y*sin(-rotation);
    robotRequiredv.y=requiredv.x*sin(-rotation)+requiredv.y*cos(-rotation);
	return robotRequiredv;
}

std::map<int, double> lastAngles;

template <
    typename T
>
T get_val(const std::vector<T> & values, size_t index) {
    if (index < values.size()) {
        return values[index];
    }
    return T(0);
}

roboteam_msgs::RobotCommand makeRobotCommand(const int inputNum, const sensor_msgs::Joy& msg) {
    // Everything is within the roboteam_input node, in groups going from
    // input0 to inputN.
    roboteam_msgs::World world = lastWorld;

    const std::string group = "~input" + std::to_string(inputNum);

    // Get the joystick type
    std::string joyType = "playstation";
    ros::param::get(group + "/joyType", joyType);
    const JoystickMap &joystickMap = joystickTypeMap.at(joyType);

    // Get the robot id
    int ROBOT_ID = 0;
    ros::param::get(group + "/robot", ROBOT_ID);

    roboteam_utils::Vector2 target_speed = roboteam_utils::Vector2(
        -pow(get_val(msg.axes, joystickMap.xAxis), 3) * 10,
        pow(get_val(msg.axes, joystickMap.yAxis), 3) * 10
    );

    
    roboteam_utils::Vector2 dirVector = roboteam_utils::Vector2(
        -get_val(msg.axes, joystickMap.rotationXAxis),
        get_val(msg.axes, joystickMap.rotationYAxis)
    );

    if (inputNum == 0) {
        ROS_INFO_STREAM("target_speed: " << target_speed.x << " " << target_speed.y << " dirVector: " << dirVector.x << " " << dirVector.y);
    }

    double myAngle = world.us.at(ROBOT_ID).angle;
    double targetAngle;
    if (dirVector.length() > 0.8) {
        targetAngle = dirVector.angle();
        lastAngles[ROBOT_ID] = targetAngle;
    } else {
        targetAngle = lastAngles[ROBOT_ID];
    }

    roboteam_utils::Vector2 requiredSpeed = positionController(target_speed);
    roboteam_utils::Vector2 requiredSpeedWF = worldToRobotFrame(requiredSpeed, myAngle);
    double requiredRotSpeed = rotationController(targetAngle - myAngle);

    if (inputNum == 0) {
        // ROS_INFO_STREAM("requiredSpeed: " << requiredSpeed.x << " " << requiredSpeed.y << " targetAngle: " << targetAngle);
        // ROS_INFO_STREAM("dribbler: " << get_val(msg.buttons, joystickMap.dribblerAxis) << " kicker: " << get_val(msg.buttons, joystickMap.kickerAxis));
    }


    roboteam_msgs::RobotCommand command;
    command.id = ROBOT_ID;
    command.x_vel = requiredSpeedWF.x;
    command.y_vel = requiredSpeedWF.y;
    command.w = requiredRotSpeed;
    command.dribbler = get_val(msg.buttons, joystickMap.dribblerAxis) > 0;
    command.kicker = get_val(msg.buttons, joystickMap.kickerAxis) > 0;
    if (command.kicker) {
        command.kicker_vel = 5.0;
    }
    return command;
}

} // rtt

int main(int argc, char **argv) {
    using namespace rtt;

    ros::init(argc, argv, "roboteam_input");
    ros::NodeHandle n;

    // For each input, construct a subscriber and store it in this vector
    // If we'd want to switch between inpus based on some topic, this vector
    // would have to be made global, and subscribers would have to be deleted
    // from the array and inserted again with proper callbacks.
    // At the same time, the joy nodes would also have to be (re)configured.
    // (right now this happens in the launch file roboteam_input.launch)
    // Right now the joy nodes have no "reconfigure" topics or anything, so
    // we'd have to write our own joystick code that reads from
    // /dev/input/js0 et al.
    std::vector<std::unique_ptr<ros::Subscriber>> subscribers;
    for (int i = 0; i < NUM_CONTROLLERS; ++i) {
        // Make a subscriber on the stack
        auto subscriber = n.subscribe<sensor_msgs::Joy>("js" + std::to_string(i), 1, boost::bind(receiveJoyMsg, i, _1));
        // Construct a copy on the heap
        std::unique_ptr<ros::Subscriber> subscriberHeap(new ros::Subscriber(subscriber));
        // Push it on the vector
        subscribers.push_back(std::move(subscriberHeap));
    }

    auto world_sub = n.subscribe<roboteam_msgs::World>("world_state", 10, callback_world_state);
    auto pub = n.advertise<roboteam_msgs::RobotCommand>("robotcommands", 10);

    // Create world & geom callbacks
    std::cout << "Waiting for first world_state...\n";
    while (!receivedFirstWorldMsg) {
        ros::spinOnce();
    }

    // TODO: Right now the roboteam_input's input names (js0, js1) are hardcoded
    // in the launch file. they should probably also be changeable through an event/topic
    // However, the joy & joy_node nodes are fixed, so not sure if this is possible
    // without writing our own joynode code.

    // Flush received messages every 1/30th second
    ros::Rate fps30(30);

    while (ros::ok()) {
        // if (lastWorld.us.size() > 0) {
            // For every controller...
            for (int i = 0; i < NUM_CONTROLLERS; ++i) {
                // If a message is present
                // if (joyMsgs[i]) {
                    // Make a command out of it
                    roboteam_msgs::RobotCommand command = makeRobotCommand(i, joyMsgs[i]);
                    // Send it
                    pub.publish(command);

                    // Reset the element in the array
                    // joyMsgs[i] = boost::none;
                // }
            }
        // }

        fps30.sleep();

        ros::spinOnce();
    }

    return 0;
}
