#include <string>
#include "joystick_input.h"
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "roboteam_msgs/RobotCommand.h"

namespace rtt{

ros::Publisher chatter1_pub;
ros::Publisher chatter2_pub;

void Joy::joyCallback1(const sensor_msgs::Joy& msg) {
    ros::param::get("/roboteam_input_1/robot1", id);
    x_vel= msg.axes[speedaxis]*2;
    y_vel= msg.axes[directionaxis]*2;
    w_vel= msg.axes[rotationaxis]*3;

    roboteam_msgs::RobotCommand command;

    command.id = id;
    command.active = active;
    command.x_vel = x_vel;
    command.y_vel = y_vel;
    command.w_vel = w_vel;
    command.dribbler = dribbler;
    // command.kick_vel = kick_vel;
    // command.chip_vel = chip_vel;

    chatter1_pub.publish(command);
}

void Joy::joyCallback2(const sensor_msgs::Joy& msg) {
	ros::param::get("/roboteam_input_1/robot2", id);
    x_vel= msg.axes[speedaxis]*2;
    y_vel= msg.axes[directionaxis]*2;
    w_vel= msg.axes[rotationaxis]*3;

    roboteam_msgs::RobotCommand command;

    command.id = id;
    command.active = active;
    command.x_vel = x_vel;
    command.y_vel = y_vel;
    command.w_vel = w_vel;
    command.dribbler = dribbler;
    // command.kick_vel = kick_vel;
    // command.chip_vel = chip_vel;

    chatter2_pub.publish(command);
}

void Joy::loop() {
    while (ros::ok()) {
        std::string myJoyNow1;
        ros::param::get("/roboteam_input_1/joy1", myJoyNow1);
        if (myJoyNow1 != myJoy1) {
        	if (myJoyNow1 == "none") {
        		ROS_INFO_STREAM("stopped listening for joystick 1");
        		sub1.shutdown();
        		myJoy1 = myJoyNow1;
        	} else {
	            ROS_INFO_STREAM("listening to topic " << myJoyNow1 << ", for joystick 1");
	            sub1.shutdown();
	            myJoy1 = myJoyNow1;
	            sub1 = n.subscribe(myJoy1, 1, &Joy::joyCallback1, this);
	        }
        }

        std::string myJoyNow2;
        ros::param::get("/roboteam_input_1/joy2", myJoyNow2);
        if (myJoyNow2 != myJoy2) {
            if (myJoyNow2 == "none") {
        		ROS_INFO_STREAM("stopped listening for joystick 2");
        		sub2.shutdown();
        		myJoy2 = myJoyNow2;
        	} else {
	            ROS_INFO_STREAM("listening to topic " << myJoyNow2 << ", for joystick 2");
	            sub2.shutdown();
	            myJoy2 = myJoyNow2;
	            sub2 = n.subscribe(myJoy2, 1, &Joy::joyCallback2, this);
	        }
        }

        ros::spinOnce();
    }
}

Joy::Joy(ros::NodeHandle& nh) {
    n = nh;
    ros::param::get("/roboteam_input_1/joy1", myJoy1);
    ros::param::get("/roboteam_input_1/joy2", myJoy2);
    ROS_INFO("startup ok");
    Joy::loop();
}

Joy::~Joy() {
    n.shutdown();
}

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "roboteam_input");
    ros::NodeHandle n;    
    rtt::chatter1_pub = n.advertise<roboteam_msgs::RobotCommand>("robotcommands", 1000);
    rtt::chatter2_pub = n.advertise<roboteam_msgs::RobotCommand>("robotcommands2", 1000);
    rtt::Joy joy(n);
    return 0;
}