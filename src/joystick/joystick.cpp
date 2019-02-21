#include <string>
#include <cmath>
#include <ctime>
#include <boost/optional.hpp>
#include <math.h>
#include <map>

#include "joystick_manager.h"

int joySticks::intSupplier = 0;
std::array<joySticks, NUM_CONTROLLERS> joys;

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
    ros::Publisher demo_pub = n.advertise<roboteam_msgs::DemoRobot>("demo_info", 10);

    // Listen to diagnostics
    ros::Subscriber sub = n.subscribe<diagnostic_msgs::DiagnosticArray>("diagnostics", 1, &handleDiagnostics);

    // Set ROS framerate
    ros::Rate fps(60);

    ROS_INFO_STREAM("Initializing NUM_CONTROLLERS controller(s)");
    // Loop over all joysticks
    for (auto &joy : joys) {
        joy.init();
    }

    int tickCounter = 0;

    while (ros::ok()) {
        tickCounter++;
        ROS_INFO_STREAM_THROTTLE(5, "==========| Tick " << tickCounter << " |==========");

        // Handle subscriber callbacks
        ros::spinOnce();

        // Loop over all joysticks
        for (auto &joy : joys) {
            // If joystick message is received
            if(joy.msg) {
                // If previous joystick message is received
                if (joy.previousMsg) {
                    // Handle buttons such as ID and control mode switching, and the geneva drive
                    handleButtons(joy, *joy.msg, *joy.previousMsg);
                    if (joy.toggleAutoPlay) {
                        roboteam_msgs::DemoRobot demoInfo;
                        demoInfo.id = joy.robotID;
                        demoInfo.reserve = !joy.autoPlay;
                        demo_pub.publish(demoInfo);
                        joy.toggleAutoPlay = false;
                        ROS_INFO_STREAM(joy.input << " : autoPlay " << (joy.autoPlay ? "on" : "off"));
                    }

                    if (!joy.autoPlay) {
                        // Create and publish robot command
                        auto command = makeRobotCommand(joy, *joy.msg, *joy.previousMsg);
                        command.use_angle = static_cast<unsigned char>(true);
                        pub.publish(command);
                    }
                }
                joy.previousMsg = joy.msg;
            }
        }
        fps.sleep();
    }

    return 0;
}