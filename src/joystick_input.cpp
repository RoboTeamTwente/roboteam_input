#include <string>
#include <cmath>
#include <boost/optional.hpp>

#include "roboteam_tactics/utils/LastWorld.h"

#include "joystick_input.h"
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "roboteam_msgs/RobotCommand.h"
#include "roboteam_msgs/World.h"
#include "roboteam_utils/Vector2.h"
#include <typeinfo>
#include <cmath>

#include "roboteam_tactics/skills/GoToPos.h"
#include "roboteam_tactics/Parts.h"

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
        // {"xbox", {
        //     1, // speedAxis
        //     0, // directionAxis
        //     3, // rotationAxis
        //     4, // dribblerAxis
        //     5 // kickerAxis
        // }},
        {"playstation", {
            1, // yAxis
            0, // xAxis
            2, // rotationXAxis
            3, // rotationYAxis
            10, // dribblerAxis
            11 // kickerAxis
        }},
        // {"gioteck", {
        //     1, // speedAxis
        //     0, // directionAxis
        //     2, // rotationAxis
        //     4, // dribblerAxis
        //     5 // kickerAxis
        // }}
    }
};

std::array<boost::optional<sensor_msgs::Joy>, NUM_CONTROLLERS> joyMsgs;

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

void sendRobotCommand(const int inputNum, const sensor_msgs::Joy& msg) {
    // Everything is within the roboteam_input node, in groups going from
    // input0 to inputN.
    roboteam_msgs::World world = LastWorld::get();

    const std::string group = "~input" + std::to_string(inputNum);

    // Get the joystick type
    std::string joyType = "playstation";
    ros::param::get(group + "/joyType", joyType);
    const JoystickMap &joystickMap = joystickTypeMap.at(joyType);

    // Get the robot id
    int ROBOT_ID = 0;
    ros::param::get(group + "/robot", ROBOT_ID);

    roboteam_utils::Vector2 target_speed = roboteam_utils::Vector2(
        pow(msg.axes[joystickMap.xAxis], 3) * 3,
        -pow(msg.axes[joystickMap.yAxis], 3) * 3
    );

    double targetAngle = roboteam_utils::Vector2(
        pow(msg.axes[joystickMap.rotationXAxis], 3),
        -pow(msg.axes[joystickMap.rotationYAxis], 3)
    ).angle();

    roboteam_utils::Vector2 myPos(world.us.at(ROBOT_ID).pos);
    roboteam_utils::Vector2 posTarget = myPos + target_speed;

    std::shared_ptr<bt::Blackboard> bb = std::make_shared<bt::Blackboard>();
    bb->SetDouble("xGoal", posTarget.x);
    bb->SetDouble("yGoal", posTarget.y);
    bb->SetDouble("wGoal", targetAngle);
    bb->SetInt("ROBOT_ID", ROBOT_ID);
    bb->SetBool("endPoint", true);
    bb->SetBool("dribbler", false);
    GoToPos goToPos("", bb);
    goToPos.Update();

    // Convert target speed to be relative to the robot.
    // float orientation = world.us.at(ROBOT_ID).angle;
    // speedCommand = worldToRobotFrame(speedCommand, orientation);

    // Construct the robot command
    // roboteam_msgs::RobotCommand command;
    // command.id = ROBOT_ID;
    // command.active = true;
    // command.x_vel= speedCommand.x;
    // command.y_vel= speedCommand.y;
    // command.w = pow(msg.axes[joystickMap.rotationAxis], 3) * 5;
    // command.dribbler = msg.buttons[joystickMap.dribblerAxis] > 0;
    // command.kicker = msg.buttons[joystickMap.kickerAxis] > 0;
    // if (command.kicker > 0) {
    //     command.kicker_vel = 8.0;
    // }

    // return command;
}

} // rtt

int main(int argc, char **argv) {
    using namespace rtt;

    ros::init(argc, argv, "roboteam_input");
    ros::NodeHandle n;

    // Make sure there are only "none's" in the array
    joyMsgs.fill(boost::none);

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

    // auto world_sub = n.subscribe<roboteam_msgs::World>("world_state", 10, callback_world_state);
    auto pub = n.advertise<roboteam_msgs::RobotCommand>("robotcommands", 10);

    // Create world & geom callbacks
    std::cout << "Waiting for first world_state...\n";
    rtt::WorldAndGeomCallbackCreator cb;
    rtt::LastWorld::wait_for_first_messages();

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
                if (joyMsgs[i]) {
                    // Make a command out of it
                    sendRobotCommand(i, *joyMsgs[i]);
                    // Send it
                    // pub.publish(command);

                    // Reset the element in the array
                    joyMsgs[i] = boost::none;
                }
            }
        // }

        fps30.sleep();

        ros::spinOnce();
    }

    return 0;
}
