#include <string>
#include <vector>
#include <rosconsole/macros_generated.h>
#include <ros/ros.h>
#include <roboteam_msgs/RobotCommand.h>

/// SETTINGS
const std::vector<unsigned int> robotIDs = {0, 1, 2, 3};
const double angle = 0.5*M_PI;
const float xvel = 0.0;
const float yvel = 0.0;



/// main loop
int main(int argc, char **argv) {
    ros::init(argc, argv, "joystick_controller");
    ros::NodeHandle n;

    // Publish on robotcommands
    ros::Publisher pub = n.advertise<roboteam_msgs::RobotCommand>("robotcommands", 10);

    // Set ROS framerate
    ros::Rate fps(60);

    std::string str = "Sending to robot ID's: ";
    for (unsigned int i : robotIDs) {
        str += std::to_string(i);
        str += (i == robotIDs[robotIDs.size()-1]) ? " " : ", ";
    }
    ROS_INFO_STREAM(str);

    while (ros::ok()) {
        // Handle subscriber callbacks
        ros::spinOnce();

        for (unsigned int i : robotIDs) {
            roboteam_msgs::RobotCommand command;
            command.id = i;
            command.use_angle = 1;
            command.w = static_cast<float>(angle);
            command.x_vel = xvel;
            command.y_vel = yvel;
            pub.publish(command);
        }
        fps.sleep();
    }
    return 0;
}