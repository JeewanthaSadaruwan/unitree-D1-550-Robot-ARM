#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/common/time/time_tool.hpp>
#include "msg/ArmString_.hpp"
#include <iostream>
#include <cmath>
#include <sstream>
#include <vector>
#include <array>
#include <mutex>
#include <chrono>
#include <thread>

#define TOPIC_COMMAND "rt/arm_Command"
#define TOPIC_FEEDBACK "rt/arm_Feedback"
#define NUM_JOINTS 6

using namespace unitree::robot;
using namespace unitree::common;

// Joint angles container
using JointAngles = std::array<double, NUM_JOINTS>;

// Thread-safe feedback storage
std::mutex feedbackMutex;
unitree_arm::msg::dds_::ArmString_ latestFeedback;
bool feedbackReceived = false;

void feedbackHandler(const void* message) {
    std::lock_guard<std::mutex> lock(feedbackMutex);
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

// Get current joint angles from feedback (thread-safe)
JointAngles getCurrentJointAngles() {
    std::lock_guard<std::mutex> lock(feedbackMutex);
    JointAngles angles;
    std::string feedback = latestFeedback.data_();
    for (int i = 0; i < NUM_JOINTS; i++) {
        angles[i] = getJointAngle(feedback, i);
    }
    return angles;
}

// Quintic polynomial coefficients for one joint
struct QuinticCoeffs {
    double a0, a1, a2, a3, a4, a5;
    
    // Evaluate position at time t
    double position(double t) const {
        return a0 + a1*t + a2*t*t + a3*t*t*t + a4*t*t*t*t + a5*t*t*t*t*t;
    }
    
    // Evaluate velocity at time t (optional, for debugging)
    double velocity(double t) const {
        return a1 + 2*a2*t + 3*a3*t*t + 4*a4*t*t*t + 5*a5*t*t*t*t;
    }
};

// Compute quintic polynomial coefficients
// Boundary conditions: q(0)=q0, q(T)=q1, v(0)=v(T)=0, a(0)=a(T)=0
QuinticCoeffs computeQuinticCoeffs(double q0, double q1, double T) {
    QuinticCoeffs c;
    double delta_q = q1 - q0;
    double T2 = T * T;
    double T3 = T2 * T;
    double T4 = T3 * T;
    double T5 = T4 * T;
    
    c.a0 = q0;
    c.a1 = 0.0;
    c.a2 = 0.0;
    c.a3 = 10.0 * delta_q / T3;
    c.a4 = -15.0 * delta_q / T4;
    c.a5 = 6.0 * delta_q / T5;
    
    return c;
}

// Compute segment duration based on maximum joint speed
double computeSegmentDuration(const JointAngles& start, const JointAngles& target, 
                               double vmax_deg_s, double min_duration = 2.0) {
    double max_time = min_duration;
    
    for (int i = 0; i < NUM_JOINTS; i++) {
        double delta_q = std::abs(target[i] - start[i]);
        double time_needed = delta_q / vmax_deg_s;
        if (time_needed > max_time) {
            max_time = time_needed;
        }
    }
    
    return max_time;
}

// Build JSON command string for joint angles
std::string buildJointCommand(int seq, const JointAngles& angles, double gripperAngle = 0.0) {
    std::ostringstream oss;
    oss << "{\"seq\":" << seq 
        << ",\"address\":1,\"funcode\":2,\"data\":{\"mode\":1"
        << ",\"angle0\":" << angles[0]
        << ",\"angle1\":" << angles[1]
        << ",\"angle2\":" << angles[2]
        << ",\"angle3\":" << angles[3]
        << ",\"angle4\":" << angles[4]
        << ",\"angle5\":" << angles[5]
        << ",\"angle6\":" << gripperAngle << "}}";
    return oss.str();
}

// Execute smooth quintic trajectory from current position to target
void executeQuinticMove(ChannelPublisher<unitree_arm::msg::dds_::ArmString_>& publisher,
                        const JointAngles& target, 
                        double vmax_deg_s = 30.0,
                        double dt = 0.01,  // 100 Hz
                        const std::string& description = "") {
    
    if (!description.empty()) {
        std::cout << description << std::endl;
    }
    
    // Get current position
    JointAngles start = getCurrentJointAngles();
    
    // Compute segment duration
    double T = computeSegmentDuration(start, target, vmax_deg_s, 1.0);
    std::cout << "  Trajectory duration: " << T << " seconds" << std::endl;
    
    // Compute quintic coefficients for each joint
    std::array<QuinticCoeffs, NUM_JOINTS> coeffs;
    for (int i = 0; i < NUM_JOINTS; i++) {
        coeffs[i] = computeQuinticCoeffs(start[i], target[i], T);
    }
    
    // Stream trajectory at fixed rate
    int num_steps = static_cast<int>(T / dt);
    auto start_time = std::chrono::steady_clock::now();
    
    for (int step = 0; step <= num_steps; step++) {
        double t = step * dt;
        if (t > T) t = T;  // Clamp to final time
        
        // Evaluate trajectory at time t
        JointAngles current_setpoint;
        for (int i = 0; i < NUM_JOINTS; i++) {
            current_setpoint[i] = coeffs[i].position(t);
        }
        
        // Publish position command
        std::string cmd = buildJointCommand(step + 1, current_setpoint);
        unitree_arm::msg::dds_::ArmString_ msg{};
        msg.data_() = cmd;
        publisher.Write(msg);
        
        // Progress indicator every 0.5 seconds
        if (step % static_cast<int>(0.5 / dt) == 0) {
            std::cout << "  Progress: " << (t/T*100.0) << "%" << std::endl;
        }
        
        // Wait for next timestep (precise timing)
        auto next_time = start_time + std::chrono::microseconds(static_cast<long long>((step + 1) * dt * 1e6));
        std::this_thread::sleep_until(next_time);
    }
    
    std::cout << "  ✓ Trajectory execution complete" << std::endl;
}

// Wait until arm reaches target position (within tolerance)
bool waitForPosition(const JointAngles& target, double tolerance = 5.0, int maxWaitSec = 10) {
    std::cout << "  Verifying final position..." << std::endl;
    
    for (int i = 0; i < maxWaitSec * 10; i++) {  // Check 10 times per second
        if (feedbackReceived) {
            JointAngles current = getCurrentJointAngles();
            
            bool all_within_tolerance = true;
            double max_error = 0.0;
            
            for (int j = 0; j < NUM_JOINTS; j++) {
                double error = std::abs(current[j] - target[j]);
                if (error > max_error) max_error = error;
                if (error > tolerance) {
                    all_within_tolerance = false;
                }
            }
            
            // Debug: Show max error every second
            if (i % 10 == 0) {
                std::cout << "  Max error: " << max_error << "°" << std::endl;
            }
            
            if (all_within_tolerance) {
                std::cout << "  ✓ Position verified!" << std::endl;
                return true;
            }
        }
        usleep(100000);  // 100ms
    }
    
    std::cout << "  ⚠ Timeout waiting for position" << std::endl;
    return false;
}

// Send one-shot command (for gripper control)
void sendOneShotCommand(ChannelPublisher<unitree_arm::msg::dds_::ArmString_>& publisher, 
                        const std::string& jsonData, const std::string& description) {
    unitree_arm::msg::dds_::ArmString_ msg{};
    msg.data_() = jsonData;
    publisher.Write(msg);
    std::cout << description << std::endl;
    
    // Delay for message delivery (only for one-shot commands)
    usleep(200000);  // 200ms
}

int main()
{
    ChannelFactory::Instance()->Init(0);
    ChannelPublisher<unitree_arm::msg::dds_::ArmString_> publisher(TOPIC_COMMAND);
    ChannelSubscriber<unitree_arm::msg::dds_::ArmString_> subscriber(TOPIC_FEEDBACK);
    subscriber.InitChannel(feedbackHandler, 1);
    publisher.InitChannel();

    std::cout << "=========================================" << std::endl;
    std::cout << "Pick and Place with Trajectory Planning" << std::endl;
    std::cout << "Using Quintic Polynomial Trajectories" << std::endl;
    std::cout << "=========================================" << std::endl;
    std::cout << "Waiting for feedback..." << std::endl;
    
    // Wait for initial feedback
    int wait_count = 0;
    while (!feedbackReceived && wait_count < 50) {
        usleep(100000);  // 100ms
        wait_count++;
    }
    
    if (!feedbackReceived) {
        std::cout << "ERROR: No feedback received from arm!" << std::endl;
        return 1;
    }
    
    std::cout << "Feedback received. Starting in 2 seconds..." << std::endl;
    sleep(2);

    // Define waypoints (all joint angles in degrees)
    JointAngles pos_home = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};  // Initial home position
    JointAngles pos_above_pick = {70.0, 73.2, -22.6, 11.7, 31.9, 59.5};  // Above pick location
    JointAngles pos_pick = {70.0, 88.3, -30.2, 11.7, 31.9, 59.5};  // Pick location (moved down)
    JointAngles pos_above_pick_lifted = {70.0, 73.2, -22.6, 11.7, 31.9, 59.5};  // Lift after pick
    JointAngles pos_above_place = {-59.3, 62.1, -15.2, -10.4, 29.1, -164.1};  // Above place location
    JointAngles pos_place = {-59.3, 82.3, -20.5, -10.4, 29.1, -164.1};  // Place location (moved down)
    JointAngles pos_above_place_lifted = {-59.3, 62.1, -15.2, -10.4, 29.1, -164.1};  // Lift after place
    JointAngles pos_safe = {0.0, -45.0, 45.0, 0.0, 0.0, 0.0};  // Safe position
    
    // Trajectory parameters - TUNED FOR SMOOTH MOTION
    double vmax = 15.0;  // max velocity in deg/s (reduced from 30 to minimize vibration)
    double vmax_vertical = 10.0;  // slower for vertical movements
    double dt = 0.02;    // 50 Hz streaming rate (reduced from 100Hz for better controller response)

    // Step 1: Move to above pick position
    std::cout << "\n=== Step 1: Moving to above pick location ===" << std::endl;
    executeQuinticMove(publisher, pos_above_pick, vmax, dt, "Executing smooth trajectory...");
    waitForPosition(pos_above_pick, 5.0, 10);
    sleep(2);

    // Step 2: Open gripper
    std::cout << "\n=== Step 2: Opening gripper ===" << std::endl;
    sendOneShotCommand(publisher, 
        "{\"seq\":100,\"address\":1,\"funcode\":1,\"data\":{\"id\":6,\"angle\":65,\"delay_ms\":2000}}",
        "Opening gripper to 65°");
    sleep(4);

    // Step 3: Move down vertically to pick position
    std::cout << "\n=== Step 3: Moving down vertically to pick ===" << std::endl;
    executeQuinticMove(publisher, pos_pick, vmax_vertical, dt, "Executing vertical movement...");
    waitForPosition(pos_pick, 5.0, 10);
    sleep(2);

    // Step 4: Close gripper - GRAB!
    std::cout << "\n=== Step 4: Closing gripper - GRAB! ===" << std::endl;
    sendOneShotCommand(publisher, 
        "{\"seq\":101,\"address\":1,\"funcode\":1,\"data\":{\"id\":6,\"angle\":1,\"delay_ms\":2000}}",
        "Closing gripper to 1° - GRABBING!");
    sleep(5);

    // Step 5: Move up vertically after grabbing
    std::cout << "\n=== Step 5: Moving up vertically after grab ===" << std::endl;
    executeQuinticMove(publisher, pos_above_pick_lifted, vmax_vertical, dt, "Executing vertical lift...");
    waitForPosition(pos_above_pick_lifted, 5.0, 10);
    sleep(2);

    // Step 6: Move to above place position (skip intermediate positions)
    std::cout << "\n=== Step 6: Moving to above place location ===" << std::endl;
    executeQuinticMove(publisher, pos_above_place, vmax, dt, "Executing smooth trajectory...");
    waitForPosition(pos_above_place, 5.0, 10);
    sleep(2);

    // Step 7: Move down vertically to place position
    std::cout << "\n=== Step 7: Moving down vertically to place ===" << std::endl;
    executeQuinticMove(publisher, pos_place, vmax_vertical, dt, "Executing vertical movement...");
    waitForPosition(pos_place, 5.0, 10);
    sleep(2);

    // Step 8: Open gripper - RELEASE!
    std::cout << "\n=== Step 8: Opening gripper - RELEASE! ===" << std::endl;
    sendOneShotCommand(publisher, 
        "{\"seq\":102,\"address\":1,\"funcode\":1,\"data\":{\"id\":6,\"angle\":65,\"delay_ms\":2000}}",
        "Opening gripper to 65° - RELEASING!");
    sleep(4);

    // Step 9: Move up vertically after releasing
    std::cout << "\n=== Step 9: Moving up vertically after release ===" << std::endl;
    executeQuinticMove(publisher, pos_above_place_lifted, vmax_vertical, dt, "Executing vertical lift...");
    waitForPosition(pos_above_place_lifted, 5.0, 10);
    sleep(2);

    // Step 10: Return to safe position
    std::cout << "\n=== Step 10: Returning to safe position ===" << std::endl;
    executeQuinticMove(publisher, pos_safe, vmax, dt, "Executing smooth trajectory...");
    waitForPosition(pos_safe, 5.0, 10);
    sleep(2);

    std::cout << "\n=========================================" << std::endl;
    std::cout << "✓ Pick and Place Sequence Complete!" << std::endl;
    std::cout << "=========================================" << std::endl;
 
    return 0;
}
