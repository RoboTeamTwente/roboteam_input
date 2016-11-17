#include <string>
#include "joystick_input.h"
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "roboteam_msgs/RobotCommand.h"
#include <typeinfo>

namespace rtt{

ros::Publisher chatter1_pub;
ros::Publisher chatter2_pub;

void Joy::joyCallback1(const sensor_msgs::Joy& msg) {
    ros::param::get("/roboteam_input_1/robot1", id);
    x_vel= msg.axes[speedaxis];
    y_vel= msg.axes[directionaxis];
    w = msg.axes[rotationaxis]*3;
	ROS_INFO(typeid(msg.axes[dribbleraxis]).name());
    roboteam_msgs::RobotCommand command;
	ROS_INFO("dribbler: %f",msg.axes[dribbleraxis]);
    command.id = id;
    command.active = active;
    command.x_vel = x_vel;
    command.y_vel = y_vel;
    command.w = w;
    command.dribbler = dribbler;
	ROS_INFO("callback1");
    pub.publish(command);
}

void Joy::joyCallback2(const sensor_msgs::Joy& msg) {
	ros::param::get("/roboteam_input_1/robot2", id);
    x_vel= msg.axes[speedaxis];
    y_vel= msg.axes[directionaxis];
    w = msg.axes[rotationaxis]*3;

    roboteam_msgs::RobotCommand command;

    command.id = id;
    command.active = active;
    command.x_vel = x_vel;
    command.y_vel = y_vel;
    command.w = w;
    command.dribbler = dribbler;
	ROS_INFO("callback2");
    pub.publish(command);
}

void Joy::joyCallback3(const sensor_msgs::Joy& msg) {
	ros::param::get("/roboteam_input_1/robot3", id);
    x_vel= msg.axes[speedaxis];
    y_vel= msg.axes[directionaxis];
    w = msg.axes[rotationaxis]*3;

    roboteam_msgs::RobotCommand command;

    command.id = id;
    command.active = active;
    command.x_vel = x_vel;
    command.y_vel = y_vel;
    command.w = w;
    command.dribbler = dribbler;
	ROS_INFO("callback3");
    pub.publish(command);
}

void Joy::joyCallback4(const sensor_msgs::Joy& msg) {
	ros::param::get("/roboteam_input_1/robot4", id);
    x_vel= msg.axes[speedaxis];
    y_vel= msg.axes[directionaxis];
    w = msg.axes[rotationaxis]*3;

    roboteam_msgs::RobotCommand command;

    command.id = id;
    command.active = active;
    command.x_vel = x_vel;
    command.y_vel = y_vel;
    command.w = w;
    command.dribbler = dribbler;
	ROS_INFO("callback4");
    pub.publish(command);
}

void Joy::loop() {
    while (ros::ok()) {
        std::string myJoyNow1;
        ros::param::get("/roboteam_input_1/joy1", myJoyNow1);
        if (myJoyNow1 != myJoy1 or firstassignment) {
        	if (myJoyNow1 == "none") {
        		ROS_INFO_STREAM("stopped listening for joystick 1");
        		sub1.shutdown();
        		myJoy1 = myJoyNow1;
        	} else {
	            ROS_INFO_STREAM("listening to topic " << myJoyNow1 << ", for joystick 1");
	            sub1.shutdown();
	            myJoy1 = myJoyNow1;
                int a = 1;
	            sub1 = n.subscribe(myJoy1, 1, &Joy::joyCallback1, this);
	        }
        }

        std::string myJoyNow2;
        ros::param::get("/roboteam_input_1/joy2", myJoyNow2);
        if (myJoyNow2 != myJoy2 or firstassignment) {
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
        
        std::string myJoyNow3;
        ros::param::get("/roboteam_input_1/joy3", myJoyNow3);
        if (myJoyNow3 != myJoy3 or firstassignment) {
            if (myJoyNow3 == "none") {
        		ROS_INFO_STREAM("stopped listening for joystick 3");
        		sub3.shutdown();
        		myJoy3 = myJoyNow3;
        	} else {
	            ROS_INFO_STREAM("listening to topic " << myJoyNow3 << ", for joystick 2");
	            sub3.shutdown();
	            myJoy3 = myJoyNow3;
	            sub3 = n.subscribe(myJoy3, 1, &Joy::joyCallback3, this);
	        }
        }
        
        std::string myJoyNow4;
        ros::param::get("/roboteam_input_1/joy4", myJoyNow4);
        if (myJoyNow4 != myJoy4 or firstassignment) {
            if (myJoyNow4 == "none") {
        		ROS_INFO_STREAM("stopped listening for joystick 4");
        		sub4.shutdown();
        		myJoy4 = myJoyNow4;
        	} else {
	            ROS_INFO_STREAM("listening to topic " << myJoyNow4 << ", for joystick 4");
	            sub4.shutdown();
	            myJoy4 = myJoyNow4;
	            sub4 = n.subscribe(myJoy2, 1, &Joy::joyCallback4, this);
	        }
        }
        
        if(firstassignment){firstassignment = false;}

        ros::spinOnce();
    }
}

Joy::Joy(ros::NodeHandle nh) {
    n = nh;
    pub = n.advertise<roboteam_msgs::RobotCommand>("robotcommands", 1000);
    ros::param::get("/roboteam_input_1/joy1", myJoy1);
    ros::param::get("/roboteam_input_1/joy2", myJoy2);
    ros::param::get("/roboteam_input_1/joy3", myJoy3);
    ros::param::get("/roboteam_input_1/joy4", myJoy4);
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
    rtt::Joy joy(n);
    return 0;
}
