#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/common/time/time_tool.hpp>
#include "../msg/ArmString_.hpp"

#define TOPIC "rt/arm_Command"

using namespace unitree::robot;
using namespace unitree::common;

int main()
{
    ChannelFactory::Instance()->Init(0);
    ChannelPublisher<unitree_arm::msg::dds_::ArmString_> publisher(TOPIC);
    publisher.InitChannel();

    unitree_arm::msg::dds_::ArmString_ msg{};
    
    while(true)
    {
        // Open gripper to 60
        std::cout << "Opening gripper to 60..." << std::endl;
        msg.data_() = "{\"seq\":4,\"address\":1,\"funcode\":1,\"data\":{\"id\":6,\"angle\":60,\"delay_ms\":0}}";
        publisher.Write(msg);
        sleep(1);  // Wait 1 second for movement to complete
        
        // Close gripper to 0
        std::cout << "Closing gripper to 0..." << std::endl;
        msg.data_() = "{\"seq\":4,\"address\":1,\"funcode\":1,\"data\":{\"id\":6,\"angle\":0,\"delay_ms\":0}}";
        publisher.Write(msg);
        sleep(1);  // Wait 1 second for movement to complete
    }
 
    return 0;
}
