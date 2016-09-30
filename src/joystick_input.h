#ifndef JOYSTICK_INPUT
#define JOYSTICK_INPUT

#include <string>
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"

namespace rtt{

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

class Joy{
public:
	Joy(ros::NodeHandle&);
	~Joy();
private:
	void joyCallback1(const sensor_msgs::Joy& msg);
	void joyCallback2(const sensor_msgs::Joy& msg);
	void loop();
	ros::Subscriber sub1;
	ros::Subscriber sub2;
	std::string myJoy1;
	std::string myJoy2;
	ros::NodeHandle n;
};

}

#endif