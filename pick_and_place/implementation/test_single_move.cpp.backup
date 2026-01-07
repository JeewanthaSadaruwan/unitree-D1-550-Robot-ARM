#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/common/time/time_tool.hpp>
#include "msg/ArmString_.hpp"
#include <iostream>

#define TOPIC_COMMAND "rt/arm_Command"
#define TOPIC_FEEDBACK "rt/arm_Feedback"

using namespace unitree::robot;
using namespace unitree::common;

unitree_arm::msg::dds_::ArmString_ latestFeedback;
bool feedbackReceived = false;

void feedbackHandler(const void* message) {
    latestFeedback = *(unitree_arm::msg::dds_::ArmString_*)message;
    feedbackReceived = true;
}

int main()
{
    ChannelFactory::Instance()->Init(0);
    ChannelPublisher<unitree_arm::msg::dds_::ArmString_> publisher(TOPIC_COMMAND);
    ChannelSubscriber<unitree_arm::msg::dds_::ArmString_> subscriber(TOPIC_FEEDBACK);
    subscriber.InitChannel(feedbackHandler, 1);
    publisher.InitChannel();

    std::cout << "=========================================" << std::endl;
    std::cout << "Move to Safe Position" << std::endl;
    std::cout << "=========================================" << std::endl;
    
    // Wait for feedback
    std::cout << "Waiting for arm feedback..." << std::endl;
    int wait_count = 0;
    while (!feedbackReceived && wait_count < 50) {
        usleep(100000);  // 100ms
        wait_count++;
    }
    
    if (!feedbackReceived) {
        std::cout << "ERROR: No feedback from arm!" << std::endl;
        std::cout << "Check robot connection." << std::endl;
        return 1;
    }
    
    std::cout << "Feedback received." << std::endl;
    std::cout << "\nMoving to safe position in 2 seconds..." << std::endl;
    sleep(2);

    // Safe position - from the last position in your sequence
    std::string safePositionCmd = 
        "{\"seq\":1,\"address\":1,\"funcode\":2,\"data\":{\"mode\":1,"
        "\"angle0\":2.4,\"angle1\":-86.3,\"angle2\":88.5,\"angle3\":-16.6,"
        "\"angle4\":-7.8,\"angle5\":-164,\"angle6\":0}}";
    
    unitree_arm::msg::dds_::ArmString_ msg{};
    msg.data_() = safePositionCmd;
    publisher.Write(msg);
    
    std::cout << "\n✓ Safe position command sent!" << std::endl;
    std::cout << "Position: [2.4, -86.3, 88.5, -16.6, -7.8, -164.0]" << std::endl;
    std::cout << "\nArm will move to safe position." << std::endl;
    std::cout << "Wait for arm to complete movement..." << std::endl;
    
    // Wait a bit to ensure command is processed
    sleep(2);
    
    std::cout << "\n✓ Done!" << std::endl;
 
    return 0;
}
