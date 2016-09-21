#include <string>

#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "roboteam_robothub/robot_command.h"

/**
 * https://www.kernel.org/doc/Documentation/input/joystick-api.txt
 */

int speedaxis=1;
int directionaxis=0;
int rotationaxis=2;
int idupdownbutton=5;


int id=0;
bool active=true;
float x_vel;
float y_vel;
float w_vel;
bool dribbler=false;
float kick_vel=0.0;
float chip_vel=0.0;

ros::Publisher chatter_pub;

void joyCallback(const sensor_msgs::Joy& msg) {
    int previousIdUpDownState=0;

    std::string myJoy;
    ros::param::get("~joy", myJoy);

    std::string txt = "I heard a joy on joystick " + myJoy + "!";
    ROS_INFO(txt.c_str());
    x_vel= msg.axes[speedaxis]*2;
    y_vel= msg.axes[directionaxis]*2;
    w_vel= msg.axes[rotationaxis]*3;

    //for (int i = sizeof(msg.axes) - 1; i >= 0; i--) 
    //	ROS_INFO("axis %i, gives: %f",i, msg.axes[i]);
    if(previousIdUpDownState != msg.axes[idupdownbutton]){
	if (msg.axes[idupdownbutton] == 1){
	    id++;
	}
	else if(msg.axes[idupdownbutton]==-1){
	    id--;
	}
	previousIdUpDownState=msg.axes[idupdownbutton];
    }
    roboteam_robothub::robot_command command;

    command.id = id;
    command.active = active;
    command.x_vel = x_vel;
    command.y_vel = y_vel;
    command.w_vel = w_vel;
    command.dribbler = dribbler;
    command.kick_vel = kick_vel;
    command.chip_vel = chip_vel;

    ROS_INFO_STREAM("command sent for robot: " << command.id);

    chatter_pub.publish(command);

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "roboteam_input");

    ros::NodeHandle n;
    chatter_pub = n.advertise<roboteam_robothub::robot_command>("robotcommands", 1000);

    if (!ros::param::has("~joy")) {
        ROS_INFO("Private parameter joy not set. Shutting down.");
    }

    std::string myJoy;
    ros::param::get("~joy", myJoy);

    ROS_INFO("Registering");

    ros::Subscriber sub0 = n.subscribe(myJoy, 1, joyCallback);

    ros::spin();

    return 0;
}
