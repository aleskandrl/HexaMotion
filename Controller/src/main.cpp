/**
 * @file main.cpp
 * @brief Main entry point for the HexaMotion headless robot controller.
 *
 * This application initializes all required motion control modules,
 * starts a network bridge to communicate with the HexaStudio HMI,
 * and runs a deterministic control loop to process commands and send feedback.
 * NO DYNAMIC MEMORY ALLOCATION (new/delete) OR EXCEPTIONS (try/catch) IN THE MAIN LOOP.
 */

#include <iostream>
#include <memory>
#include <thread>
#include <chrono>
#include <csignal> // For signal handling (Ctrl+C)

// --- Core Application Components ---
#include "HmiBridge.h"
#include "Logger.h"

// --- Robot Control Modules ---
#include "DataTypes.h"
#include "Units.h"
#include "RobotConfig.h"
#include "StateData.h"
#include "KinematicModel.h"
#include "KdlKinematicSolver.h"
#include "TrajectoryInterpolator.h"
#include "TrajectoryPlanner.h"
#include "MasterHardwareInterface.h"
#include "MotionManager.h"
#include "RobotController.h"
#include "RdtProtocol.h"

// FIXED: Add using namespace to resolve type name errors
using namespace RDT;
using namespace Rdt;
using namespace RDT::literals;
using namespace std::chrono_literals;

// --- Global state for graceful shutdown ---
std::atomic<bool> g_shutdown_requested = false;

void signalHandler([[maybe_unused]] int signum) {
    if (!g_shutdown_requested) {
        LOG_WARN("Main", "Interrupt signal received. Requesting shutdown...");
        g_shutdown_requested = true;
    }
}

// --- Function Prototypes ---
// FIXED: All types are now correctly namespaced or resolved via 'using'
void processHmiCommands(const ControlState& hmi_cmd, RobotController& robot_controller, StateData& state_data, const KinematicModel& model);
void publishRobotStatus(StateData& state_data, HmiBridge& hmi_bridge, const ControlState& last_hmi_cmd, const RobotController& robot_controller);
// REMOVED: Unnecessary and Qt-dependent function prototype was removed.

int main([[maybe_unused]] int argc, [[maybe_unused]] char* argv[]) {
    // --- Graceful shutdown setup ---
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    // --- 1. Initialization Phase ---
    Logger::setLogLevel(LogLevel::Debug);
    LOG_INFO("Main", "--- HexaMotion Controller Starting ---");

    // --- 1a. Configuration Loading ---
    InterfaceConfig hw_config;
    hw_config.realtime_type = InterfaceConfig::RealtimeInterfaceType::Udp;
    hw_config.motion_manager_cycle_ms = 4; // 250 Hz RT loop

    ControllerConfig ctrl_config;
    ctrl_config.control_loop_period = 20ms; // 50 Hz NRT loop for this main

    // --- 1b. Component Instantiation (using smart pointers for automatic resource management) ---
    LOG_INFO("Main", "Creating core components...");
    auto state_data = std::make_shared<StateData>();
    auto kuka_model = std::make_shared<KinematicModel>(KinematicModel::createKR6R900());
    
    // Pass const reference of model to components that don't take ownership
    auto robot_controller = std::make_shared<RobotController>(hw_config, ctrl_config, *kuka_model, state_data);
    
    // HmiBridge is stack-allocated as its lifetime is managed by main()
    HmiBridge hmi_bridge;

    // --- 1c. Backend Initialization ---
    LOG_INFO("Main", "Initializing Robot Controller backend...");
    if (!robot_controller->initialize(TrajectoryPoint{})) {
        LOG_CRITICAL("Main", "Failed to initialize RobotController. Shutting down.");
        return -1;
    }

    // --- 1d. Network Initialization ---
    const int HMI_PORT = 30002;
    if (!hmi_bridge.start(HMI_PORT)) {
        LOG_CRITICAL("Main", "Failed to start HMI Bridge. Is the port in use? Shutting down.");
        return -1;
    }

    // --- 2. Main Control Loop ---
    LOG_INFO("Main", "Initialization complete. Entering main control loop...");
    ControlState last_hmi_cmd; // Cache the last command state from HMI
    
    while (!g_shutdown_requested) {
        auto loop_start_time = std::chrono::steady_clock::now();

        // --- 2a. Poll for HMI Commands ---
        ControlState new_hmi_cmd = hmi_bridge.poll();
        
        // This logic correctly updates the cached command state ONLY if a new command was received.
        if (new_hmi_cmd.jogRequestId != last_hmi_cmd.jogRequestId || new_hmi_cmd.programCommand != 0 || new_hmi_cmd.setEStop || new_hmi_cmd.resetEStop) {
            last_hmi_cmd = new_hmi_cmd;
        }

        // --- 2b. Process HMI Commands ---
        processHmiCommands(last_hmi_cmd, *robot_controller, *state_data, *kuka_model);
        
        // --- 2c. Clear one-shot commands after processing ---
        last_hmi_cmd.setEStop = false;
        last_hmi_cmd.resetEStop = false;
        last_hmi_cmd.programCommand = 0;
        last_hmi_cmd.jogIncrement = 0.0;
        
        // --- 2d. Publish Robot Status to HMI ---
        publishRobotStatus(*state_data, hmi_bridge, last_hmi_cmd, *robot_controller);
        
        // --- 2e. Maintain Loop Frequency ---
        auto loop_end_time = std::chrono::steady_clock::now();
        auto loop_duration = loop_end_time - loop_start_time;
        if (loop_duration < ctrl_config.control_loop_period) {
            std::this_thread::sleep_for(ctrl_config.control_loop_period - loop_duration);
        }
    }

    // --- 3. Shutdown Phase ---
    LOG_INFO("Main", "Shutdown requested. Cleaning up resources...");
    // hmi_bridge.stop() will be called by its destructor.
    // Smart pointers will release their resources.
    // The jthread in RobotController will be automatically joined.
    return 0;
}

void processHmiCommands(const ControlState& hmi_cmd, RobotController& robot_controller, StateData& state_data, const KinematicModel& model) {
    static uint32_t processed_jog_id = 0;
    static bool last_real_mode_request = false;

    // --- Safety First ---
    if (hmi_cmd.setEStop) {
        robot_controller.emergencyStop();
    } else if (hmi_cmd.resetEStop) {
        robot_controller.reset();
    }

    // --- Mode Switching ---
    if (hmi_cmd.enableRealMode != last_real_mode_request) {
        if (robot_controller.requestModeSwitch(
            hmi_cmd.enableRealMode ? MasterHardwareInterface::ActiveMode::Realtime : MasterHardwareInterface::ActiveMode::Simulation
        ) != RobotController::SwitchRequestResult::Success) {
            LOG_WARN("CommandHandler", "Mode switch request failed or was not immediate.");
        }
        last_real_mode_request = hmi_cmd.enableRealMode;
    }
    
    // --- Program Commands ---
    if (hmi_cmd.programCommand == 4) { // Go Home
        LOG_INFO("CommandHandler", "Go Home command received.");
        TrajectoryPoint home_cmd;
        home_cmd.header.motion_type = MotionType::PTP;
        // FIXED: Get home position from the kinematic model directly.
        home_cmd.command.joint_target = model.getHomePositionJoints();
        home_cmd.command.speed_ratio = 0.25; // Safe speed for homing
        if (!robot_controller.executeMotionToTarget(home_cmd)) {
            LOG_WARN("CommandHandler", "Go Home command failed to execute.");
        }
    }

    // --- Jogging ---
    if (hmi_cmd.jogRequestId != processed_jog_id && hmi_cmd.jogAxis != -1 && hmi_cmd.jogIncrement != 0.0) {
        processed_jog_id = hmi_cmd.jogRequestId;

        TrajectoryPoint jog_cmd;
        jog_cmd.header.motion_type = MotionType::JOINT;
        jog_cmd.command.speed_ratio = hmi_cmd.speedOverride;

        AxisSet current_joints = state_data.getFbTrajectoryPoint().feedback.joint_actual;
        if(hmi_cmd.jogAxis >= 0 && hmi_cmd.jogAxis < static_cast<int>(ROBOT_AXES_COUNT)) {
            current_joints.at(hmi_cmd.jogAxis).angle += Degrees(hmi_cmd.jogIncrement).toRadians();
            jog_cmd.command.joint_target = current_joints;
            if (!robot_controller.executeMotionToTarget(jog_cmd)) {
                 LOG_WARN("CommandHandler", "Jog command failed to execute.");
            }
        }
    }
}

void publishRobotStatus(StateData& state_data, HmiBridge& hmi_bridge, const ControlState& last_hmi_cmd, const RobotController& robot_controller) {
    RobotStatus status_msg;

    TrajectoryPoint feedback_tp = state_data.getFbTrajectoryPoint();
    TrajectoryPoint command_tp = state_data.getCmdTrajectoryPoint();
    RobotMode current_mode = state_data.getRobotMode();

    // --- Populate MotionState ---
    for (size_t i = 0; i < ROBOT_AXES_COUNT; ++i) {
        status_msg.motion.actualJoints[i] = feedback_tp.feedback.joint_actual.at(i).angle.toDegrees().value();
        status_msg.motion.plannedJoints[i] = command_tp.command.joint_target.at(i).angle.toDegrees().value();
        
        status_msg.motion.actualTcp[i] = (i < 3) ? feedback_tp.feedback.cartesian_actual.get_value_at(i) * 1000.0
                                                 : feedback_tp.feedback.cartesian_actual.get_value_at(i) * (180.0 / UnitConstants::PI);
        status_msg.motion.plannedTcp[i] = (i < 3) ? command_tp.command.cartesian_target.get_value_at(i) * 1000.0
                                                  : command_tp.command.cartesian_target.get_value_at(i) * (180.0 / UnitConstants::PI);
    }
    
    status_msg.motion.isMoving = (current_mode == RobotMode::Running || current_mode == RobotMode::Jogging || current_mode == RobotMode::Homing);
    status_msg.motion.isSimulated = (robot_controller.getActiveMode() == MasterHardwareInterface::ActiveMode::Simulation);
    status_msg.motion.speedRatio = state_data.getGlobalSpeedRatio();

    // --- Populate SystemState & ProgramState ---
    status_msg.sys.isEStop = state_data.isEstopActive();
    status_msg.sys.isPowerOn = state_data.isPhysicallyConnected();
    status_msg.prog.isRunning = (current_mode == RobotMode::Running);
    status_msg.prog.isPaused = (current_mode == RobotMode::Paused);
    
    // --- Populate transaction ACKs and versions ---
    status_msg.processedProgramReqId = last_hmi_cmd.programUpdateReqId;
    status_msg.processedConfigReqId = last_hmi_cmd.configUpdateReqId;
    
    // TODO: Implement actual versioning from the controller logic
    status_msg.configVersion = 1;
    status_msg.trajVersion = 1;
    status_msg.programVersion = 1;

    // --- Broadcast ---
    hmi_bridge.broadcastStatus(status_msg);
}