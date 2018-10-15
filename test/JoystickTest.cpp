//
// Created by mrlukasbos on 18-9-18.
//

#include <gtest/gtest.h>
#include "sensor_msgs/Joy.h"

sensor_msgs::Joy fakeMsg;

TEST(JoystickTest, it_fakes_a_joymsg) {
    fakeMsg.axes.push_back(3);
    sensor_msgs::Joy msg = fakeMsg;

    ASSERT_TRUE(true);
}