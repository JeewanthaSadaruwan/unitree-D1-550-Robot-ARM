#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/common/time/time_tool.hpp>
#include "msg/ArmString_.hpp"
#include <iostream>
#include <cmath>
#include <sstream>

#define TOPIC_COMMAND "rt/arm_Command"
#define TOPIC_FEEDBACK "rt/arm_Feedback"

using namespace unitree::robot;
using namespace unitree::common;

// Global variable to store latest feedback
unitree_arm::msg::dds_::ArmString_ latestFeedback;
bool feedbackReceived = false;

void feedbackHandler(const void* message) {
    latestFeedback = *(unitree_arm::msg::dds_::ArmString_*)message;
    feedbackReceived = true;
}

// Parse joint angle from feedback JSON string
double getJointAngle(const std::string& json, int jointId) {
    std::string searchKey = "\"servo" + std::to_string(jointId) + "_data\":";
    size_t pos = json.find(searchKey);
    if (pos == std::string::npos) return 0.0;
    
    pos += searchKey.length();
    size_t endPos = json.find_first_of(",}", pos);
    std::string valueStr = json.substr(pos, endPos - pos);
    return std::stod(valueStr);
}

// Wait until arm reaches target position (within tolerance)
bool waitForPosition(double target0, double target1, double target2, double target3, 
                     double target4, double target5, double tolerance = 5.0, int maxWaitSec = 15) {
    std::cout << "  Waiting for arm to reach target position..." << std::endl;
    
    for (int i = 0; i < maxWaitSec * 10; i++) {  // Check 10 times per second
        if (feedbackReceived) {
            std::string feedback = latestFeedback.data_();
            
            double current0 = getJointAngle(feedback, 0);
            double current1 = getJointAngle(feedback, 1);
            double current2 = getJointAngle(feedback, 2);
            double current3 = getJointAngle(feedback, 3);
            double current4 = getJointAngle(feedback, 4);
            double current5 = getJointAngle(feedback, 5);
            
            double error0 = std::abs(current0 - target0);
            double error1 = std::abs(current1 - target1);
            double error2 = std::abs(current2 - target2);
            double error3 = std::abs(current3 - target3);
            double error4 = std::abs(current4 - target4);
            double error5 = std::abs(current5 - target5);
            
            // Debug: Show position errors every second
            if (i % 10 == 0) {
                std::cout << "  Errors: " << error0 << "° " << error1 << "° " 
                          << error2 << "° " << error3 << "° " << error4 << "° " << error5 << "°" << std::endl;
            }
            
            // Check if all joints are within tolerance
            if (error0 < tolerance && error1 < tolerance && error2 < tolerance &&
                error3 < tolerance && error4 < tolerance && error5 < tolerance) {
                std::cout << "  ✓ Position reached!" << std::endl;
                return true;
            }
        }
        usleep(100000);  // 100ms
    }
    
    std::cout << "  ⚠ Timeout waiting for position" << std::endl;
    return false;
}

void sendCommand(ChannelPublisher<unitree_arm::msg::dds_::ArmString_>& publisher, 
                 const std::string& jsonData, const std::string& description) {
    unitree_arm::msg::dds_::ArmString_ msg{};
    msg.data_() = jsonData;
    publisher.Write(msg);
    std::cout << description << std::endl;
    
    // Small delay to ensure DDS message is transmitted
    usleep(200000);  // 200ms delay for message delivery
}

int main()
{
    ChannelFactory::Instance()->Init(0);
    ChannelPublisher<unitree_arm::msg::dds_::ArmString_> publisher(TOPIC_COMMAND);
    ChannelSubscriber<unitree_arm::msg::dds_::ArmString_> subscriber(TOPIC_FEEDBACK);
    subscriber.InitChannel(feedbackHandler, 1);
    publisher.InitChannel();

    std::cout << "=== Starting Pick and Place Sequence ===" << std::endl;
    std::cout << "Robot is at zero position" << std::endl;
    sleep(3);

    // Step 1: Move to first position
    sendCommand(publisher, 
        "{\"seq\":1,\"address\":1,\"funcode\":2,\"data\":{\"mode\":1,\"angle0\":67.9,\"angle1\":73.2,\"angle2\":-22.6,\"angle3\":11.6,\"angle4\":35.7,\"angle5\":59.7,\"angle6\":0}}",
        "Step 1: Moving to first position");
    waitForPosition(67.9, 73.2, -22.6, 11.6, 35.7, 59.7);
    sleep(2);  // Hold for 2 seconds after reaching

    // Step 2: Open gripper to 65
    sendCommand(publisher, 
        "{\"seq\":2,\"address\":1,\"funcode\":1,\"data\":{\"id\":6,\"angle\":65,\"delay_ms\":2000}}",
        "Step 2: Opening gripper to 65");
    sleep(4);

    // Step 3: Move to pick position
    sendCommand(publisher, 
        "{\"seq\":3,\"address\":1,\"funcode\":2,\"data\":{\"mode\":1,\"angle0\":70,\"angle1\":88.3,\"angle2\":-30.2,\"angle3\":11.7,\"angle4\":31.9,\"angle5\":59.5,\"angle6\":65}}",
        "Step 3: Moving to pick position");
    waitForPosition(70, 88.3, -30.2, 11.7, 31.9, 59.5);
    sleep(2);

    // Step 4: Close gripper to 1 - GRAB!
    sendCommand(publisher, 
        "{\"seq\":4,\"address\":1,\"funcode\":1,\"data\":{\"id\":6,\"angle\":1,\"delay_ms\":2000}}",
        "Step 4: Closing gripper to 1 - GRAB CUP!");
    sleep(5);

    // Step 5: Move to transport position
    sendCommand(publisher, 
        "{\"seq\":5,\"address\":1,\"funcode\":2,\"data\":{\"mode\":1,\"angle0\":70.6,\"angle1\":54.7,\"angle2\":-14,\"angle3\":74.5,\"angle4\":-70.2,\"angle5\":133.8,\"angle6\":0}}",
        "Step 5: Moving to transport position");
    waitForPosition(70.6, 54.7, -14, 74.5, -70.2, 133.8);
    sleep(2);

    // Step 6: Move to intermediate position
    sendCommand(publisher, 
        "{\"seq\":6,\"address\":1,\"funcode\":2,\"data\":{\"mode\":1,\"angle0\":-57.1,\"angle1\":62.1,\"angle2\":-15.2,\"angle3\":-10.6,\"angle4\":37.6,\"angle5\":-164.1,\"angle6\":0}}",
        "Step 6: Moving to intermediate position");
    waitForPosition(-57.1, 62.1, -15.2, -10.6, 37.6, -164.1);
    sleep(2);

    // Step 7: Move to place position
    sendCommand(publisher, 
        "{\"seq\":7,\"address\":1,\"funcode\":2,\"data\":{\"mode\":1,\"angle0\":-59.3,\"angle1\":82.3,\"angle2\":-20.5,\"angle3\":-10.4,\"angle4\":29.1,\"angle5\":-164.1,\"angle6\":0}}",
        "Step 7: Moving to place position");
    waitForPosition(-59.3, 82.3, -20.5, -10.4, 29.1, -164.1);
    sleep(2);

    // Step 8: Open gripper to 65 - RELEASE!
    sendCommand(publisher, 
        "{\"seq\":8,\"address\":1,\"funcode\":1,\"data\":{\"id\":6,\"angle\":65,\"delay_ms\":2000}}",
        "Step 8: Opening gripper to 65 - RELEASE CUP!");
    sleep(4);

    // Step 9: Return to safe position
    sendCommand(publisher, 
        "{\"seq\":9,\"address\":1,\"funcode\":2,\"data\":{\"mode\":1,\"angle0\":2.4,\"angle1\":-86.3,\"angle2\":88.5,\"angle3\":-16.6,\"angle4\":-7.8,\"angle5\":-164,\"angle6\":0}}",
        "Step 9: Returning to safe position");
    waitForPosition(2.4, -86.3, 88.5, -16.6, -7.8, -164);
    sleep(2);

    

    std::cout << "=== Sequence Complete! ===" << std::endl;
 
    return 0;
}
