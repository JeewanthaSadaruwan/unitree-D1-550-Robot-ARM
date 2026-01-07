#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/common/time/time_tool.hpp>
#include "msg/ArmString_.hpp"
#include "msg/PubServoInfo_.hpp"
#include <iostream>
#include <cmath>
#include <sstream>
#include <vector>
#include <array>
#include <mutex>
#include <chrono>
#include <thread>

#define TOPIC_COMMAND "rt/arm_Command"
#define TOPIC_SERVO_ANGLE "current_servo_angle"
#define NUM_JOINTS 6

using namespace unitree::robot;
using namespace unitree::common;

// Joint angles container
using JointAngles = std::array<double, NUM_JOINTS>;

// Thread-safe servo angle storage
std::mutex servoMutex;
unitree_arm::msg::dds_::PubServoInfo_ latestServoInfo;
bool servoReceived = false;

void servoHandler(const void* message) {
    std::lock_guard<std::mutex> lock(servoMutex);
    latestServoInfo = *(unitree_arm::msg::dds_::PubServoInfo_*)message;
    servoReceived = true;
}

// Get current joint angles from servo topic (thread-safe)
JointAngles getCurrentJointAngles() {
    std::lock_guard<std::mutex> lock(servoMutex);
    JointAngles angles;
    angles[0] = latestServoInfo.servo0_data_();
    angles[1] = latestServoInfo.servo1_data_();
    angles[2] = latestServoInfo.servo2_data_();
    angles[3] = latestServoInfo.servo3_data_();
    angles[4] = latestServoInfo.servo4_data_();
    angles[5] = latestServoInfo.servo5_data_();
    return angles;
}

// Calibrate: Read and average current joint positions
JointAngles calibrateStartPosition(int num_samples = 10) {
    std::cout << "\n📍 Calibrating start position..." << std::endl;
    std::cout << "   Taking " << num_samples << " samples and averaging..." << std::endl;
    
    std::vector<JointAngles> samples;
    
    for (int i = 0; i < num_samples; i++) {
        if (!servoReceived) {
            std::cout << "   ⚠ Warning: No servo data during calibration" << std::endl;
            usleep(100000);  // Wait 100ms
            continue;
        }
        
        JointAngles current = getCurrentJointAngles();
        samples.push_back(current);
        
        // Print first 5 samples for verification
        if (i < 5) {
            std::cout << "   Sample " << (i+1) << ": [" << current[0] << ", " << current[1] << ", " 
                      << current[2] << ", " << current[3] << ", " << current[4] << ", " 
                      << current[5] << "]" << std::endl;
        }
        
        usleep(50000);  // 50ms between samples
    }
    
    if (samples.empty()) {
        std::cout << "   ❌ Failed to calibrate! No servo data received." << std::endl;
        return {0, 0, 0, 0, 0, 0};
    }
    
    // Calculate average
    JointAngles avg = {0, 0, 0, 0, 0, 0};
    for (const auto& sample : samples) {
        for (int i = 0; i < NUM_JOINTS; i++) {
            avg[i] += sample[i];
        }
    }
    for (int i = 0; i < NUM_JOINTS; i++) {
        avg[i] /= samples.size();
    }
    
    std::cout << "   ✓ Calibrated with " << samples.size() << " samples" << std::endl;
    std::cout << "   Start position: [";
    for (int i = 0; i < NUM_JOINTS; i++) {
        std::cout << avg[i];
        if (i < NUM_JOINTS - 1) std::cout << ", ";
    }
    std::cout << "]°" << std::endl;
    
    return avg;
}

// Forward kinematics - compute end-effector position from joint angles
struct EndEffectorPos {
    double x, y, z;
};

EndEffectorPos computeEndEffectorPosition(const JointAngles& angles) {
    // Convert to radians
    double q[NUM_JOINTS];
    for (int i = 0; i < NUM_JOINTS; i++) {
        q[i] = angles[i] * M_PI / 180.0;
    }
    
    // Link lengths from URDF (in meters)
    double base_height = 0.0738;
    double shoulder_z = 0.0578;
    double shoulder_y = -0.0276;
    double upper_arm_length = 0.27;
    double upper_arm_y = -0.0004;
    double forearm_x = 0.05;
    double forearm_y = 0.0275;
    double forearm_z = 0.041325;
    double wrist1_x = 0.15468;
    double wrist1_y = -0.0258;
    double tool_x = 0.0777;
    double tool_y = 0.025822;
    
    // Joint 0: Base rotation
    double c0 = cos(q[0]);
    double s0 = sin(q[0]);
    
    // Cumulative angles
    double a1 = q[1];
    double a2 = q[1] + q[2];
    double a3 = q[1] + q[2] + q[3];
    
    // Position in arm plane
    double x_local = 0;
    double z_local = base_height + shoulder_z;
    double y_local = shoulder_y;
    
    // Add upper arm
    x_local += upper_arm_length * cos(a1);
    z_local += upper_arm_length * sin(a1);
    y_local += upper_arm_y;
    
    // Add forearm
    x_local += forearm_x + forearm_z * cos(a2);
    z_local += forearm_z * sin(a2);
    y_local += forearm_y;
    
    // Add wrist
    x_local += wrist1_x * cos(a3);
    z_local += wrist1_x * sin(a3);
    y_local += wrist1_y;
    
    // Add tool
    x_local += tool_x * cos(a3);
    z_local += tool_x * sin(a3);
    y_local += tool_y;
    
    // Rotate to world coordinates
    EndEffectorPos pos;
    pos.x = x_local * c0 - y_local * s0;
    pos.y = x_local * s0 + y_local * c0;
    pos.z = z_local;
    
    return pos;
}

// Quintic polynomial coefficients for one joint
struct QuinticCoeffs {
    double a0, a1, a2, a3, a4, a5;
    
    double position(double t) const {
        return a0 + a1*t + a2*t*t + a3*t*t*t + a4*t*t*t*t + a5*t*t*t*t*t;
    }
};

// Compute quintic polynomial coefficients
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

// Compute segment duration
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

// Build JSON command string
std::string buildJointCommand(int seq, const JointAngles& angles) {
    std::ostringstream oss;
    oss << "{\"seq\":" << seq 
        << ",\"address\":1,\"funcode\":2,\"data\":{\"mode\":1"
        << ",\"angle0\":" << angles[0]
        << ",\"angle1\":" << angles[1]
        << ",\"angle2\":" << angles[2]
        << ",\"angle3\":" << angles[3]
        << ",\"angle4\":" << angles[4]
        << ",\"angle5\":" << angles[5]
        << ",\"angle6\":0}}";
    return oss.str();
}

// Execute smooth trajectory
void executeQuinticMove(ChannelPublisher<unitree_arm::msg::dds_::ArmString_>& publisher,
                        const JointAngles& start,
                        const JointAngles& target, 
                        double vmax_deg_s,
                        double dt) {
    
    double T = computeSegmentDuration(start, target, vmax_deg_s, 2.0);
    std::cout << "  Duration: " << T << " seconds" << std::endl;
    
    std::array<QuinticCoeffs, NUM_JOINTS> coeffs;
    for (int i = 0; i < NUM_JOINTS; i++) {
        coeffs[i] = computeQuinticCoeffs(start[i], target[i], T);
    }
    
    int num_steps = static_cast<int>(T / dt);
    auto start_time = std::chrono::steady_clock::now();
    
    for (int step = 0; step <= num_steps; step++) {
        double t = step * dt;
        if (t > T) t = T;
        
        JointAngles current_setpoint;
        for (int i = 0; i < NUM_JOINTS; i++) {
            current_setpoint[i] = coeffs[i].position(t);
        }
        
        std::string cmd = buildJointCommand(step + 1, current_setpoint);
        unitree_arm::msg::dds_::ArmString_ msg{};
        msg.data_() = cmd;
        publisher.Write(msg);
        
        if (step % static_cast<int>(0.5 / dt) == 0) {
            std::cout << "  Progress: " << (t/T*100.0) << "%" << std::endl;
        }
        
        auto next_time = start_time + std::chrono::microseconds(static_cast<long long>((step + 1) * dt * 1e6));
        std::this_thread::sleep_until(next_time);
    }
    
    std::cout << "  ✓ Complete" << std::endl;
}

int main()
{
    ChannelFactory::Instance()->Init(0);
    ChannelPublisher<unitree_arm::msg::dds_::ArmString_> publisher(TOPIC_COMMAND);
    ChannelSubscriber<unitree_arm::msg::dds_::PubServoInfo_> servoSubscriber(TOPIC_SERVO_ANGLE);
    
    servoSubscriber.InitChannel(servoHandler, 1);
    publisher.InitChannel();

    std::cout << "=========================================" << std::endl;
    std::cout << "Single Move with Trajectory Planning" << std::endl;
    std::cout << "=========================================" << std::endl;
    
    // Wait for servo data
    std::cout << "Waiting for servo data..." << std::endl;
    int wait_count = 0;
    while (!servoReceived && wait_count < 50) {
        usleep(100000);
        wait_count++;
    }
    
    if (!servoReceived) {
        std::cout << "ERROR: No servo data!" << std::endl;
        return 1;
    }
    
    // Calibrate: Read current position
    JointAngles start = calibrateStartPosition(10);
    
    // Compute and display end-effector position
    EndEffectorPos eePos = computeEndEffectorPosition(start);
    std::cout << "\n📍 End-Effector Position:" << std::endl;
    std::cout << "   X: " << eePos.x << " m" << std::endl;
    std::cout << "   Y: " << eePos.y << " m" << std::endl;
    std::cout << "   Z: " << eePos.z << " m" << std::endl;
    
    std::cout << "\nStarting movement in 5 seconds..." << std::endl;
    sleep(5);

    // Target position with YOUR QUINTIC TRAJECTORY PLANNING
    JointAngles target = {67.9, 73.2, -22.6, 11.6, 35.7, 59.7};
    
    // Trajectory parameters - adjust these to tune smoothness
    double vmax = 30.0;   // max velocity deg/s (lower = slower, smoother)
    double dt = 0.01;     // 100 Hz update rate
    
    std::cout << "\nMoving with YOUR quintic trajectory planning..." << std::endl;
    std::cout << "Target: [67.9, 73.2, -22.6, 11.6, 35.7, 59.7]" << std::endl;
    std::cout << "Max velocity: " << vmax << " deg/s" << std::endl;
    std::cout << "Update rate: " << (1.0/dt) << " Hz" << std::endl;
    
    executeQuinticMove(publisher, start, target, vmax, dt);
    
    std::cout << "\n✓ Movement complete!" << std::endl;
 
    return 0;
}
