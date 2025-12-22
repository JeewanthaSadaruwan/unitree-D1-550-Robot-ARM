#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/common/time/time_tool.hpp>
#include "msg/PubServoInfo_.hpp"
#include "msg/ArmString_.hpp"

#define TOPIC "current_servo_angle"
#define TOPIC1 "arm_Feedback"

using namespace unitree::robot;
using namespace unitree::common;

void Handler(const void* msg)
{
    const unitree_arm::msg::dds_::PubServoInfo_* pm = (const unitree_arm::msg::dds_::PubServoInfo_*)msg;

    std::cout << "servo0_data:" << pm->servo0_data_() << ", servo1_data:" << pm->servo1_data_() << ", servo2_data:" << pm->servo2_data_()<< ", servo3_data:" << pm->servo3_data_()<< ", servo4_data:" << pm->servo4_data_()<< ", servo5_data:" << pm->servo5_data_()<< ", servo6_data:" << pm->servo6_data_() << std::endl;
}

void Handler1(const void* msg)
{
    const unitree_arm::msg::dds_::ArmString_* pm = (const unitree_arm::msg::dds_::ArmString_*)msg;

    std::cout << "armFeedback_data:" << pm->data_() << std::endl;
}

int main()
{
    ChannelFactory::Instance()->Init(0);
    ChannelSubscriber<unitree_arm::msg::dds_::PubServoInfo_> subscriber(TOPIC);
    subscriber.InitChannel(Handler);

    ChannelSubscriber<unitree_arm::msg::dds_::ArmString_> subscriber1(TOPIC1);
    subscriber1.InitChannel(Handler1);

    while (true)
    {
        sleep(10);
    }

    return 0;
}
