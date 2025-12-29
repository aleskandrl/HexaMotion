/**
 * @file RdtJson.h
 * @brief Serialization/deserialization helpers for RdtProtocol structures using nlohmann::json.
 * This file acts as the "dictionary" for converting C++ objects to JSON and back.
 */

#ifndef RDT_JSON_H
#define RDT_JSON_H

#include "RdtProtocol.h"
#include "nlohmann/json.hpp"
#include <cstring> // For strncpy

using json = nlohmann::json;

namespace Rdt {

// --- Serialization for Controller -> HMI (to_json) ---

// These structures can be handled automatically by nlohmann::json using this macro
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(ToolData, id, name, offset)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(ControllerConfig, tools, bases, ipAddress, axisLimits)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(TrajectoryPoint, x, y, z)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(TrajectoryPath, points, waypoints)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(ProgramStep, id, type, targetJoints, speed, commandCode)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(ProgramData, name, steps)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(MotionState, actualJoints, plannedJoints, actualTcp, plannedTcp, monitorPose, isMoving, isSimulated, speedRatio)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(SystemState, isEStop, isPowerOn, activeErrorId, cpuLoad, controllerTemp, networkLatency)

// Custom serializer for ProgramState is required because of the fixed-size char array.
inline void to_json(json& j, const ProgramState& p) {
    j = json{
        {"isRunning", p.isRunning},
        {"isPaused", p.isPaused},
        {"currentLine", p.currentLine},
        {"programName", std::string(p.programName)} // Convert char array to string
    };
}

// Custom serializer for the main RobotStatus object to handle conditional serialization.
inline void to_json(json& j, const RobotStatus& p) {
    j = json{
        {"timestamp", p.timestamp},
        {"motion", p.motion},
        {"sys", p.sys},
        {"prog", p.prog},
        {"processedJogId", p.processedJogId},
        {"processedConfigReqId", p.processedConfigReqId},
        {"processedProgramReqId", p.processedProgramReqId},
        {"configVersion", p.configVersion},
        {"trajVersion", p.trajVersion},
        {"programVersion", p.programVersion}
    };
    // Conditionally add heavy payloads to save bandwidth
    if (!p.config.tools.empty() || !p.config.bases.empty()) {
        j["config"] = p.config;
    }
    if (!p.trajectory.points.empty() || !p.trajectory.waypoints.empty()) {
        j["trajectory"] = p.trajectory;
    }
    if (!p.loadedProgram.steps.empty()) {
        j["loadedProgram"] = p.loadedProgram;
    }
}


// --- Deserialization for HMI -> Controller (from_json) ---

// Only ControlState needs a deserializer on the controller side.
inline void from_json(const json& j, ControlState& p) {
    // A safe way to deserialize: check for key existence before getting value.
    if(j.contains("jogRequestId")) j.at("jogRequestId").get_to(p.jogRequestId);
    if(j.contains("jogAxis")) j.at("jogAxis").get_to(p.jogAxis);
    if(j.contains("jogIncrement")) j.at("jogIncrement").get_to(p.jogIncrement);

    if(j.contains("speedOverride")) j.at("speedOverride").get_to(p.speedOverride);
    if(j.contains("enableRealMode")) j.at("enableRealMode").get_to(p.enableRealMode);
    if(j.contains("monitorToolId")) j.at("monitorToolId").get_to(p.monitorToolId);
    if(j.contains("monitorBaseId")) j.at("monitorBaseId").get_to(p.monitorBaseId);

    if(j.contains("setEStop")) j.at("setEStop").get_to(p.setEStop);
    if(j.contains("resetEStop")) j.at("resetEStop").get_to(p.resetEStop);

    if(j.contains("programCommand")) j.at("programCommand").get_to(p.programCommand);
    if(j.contains("previewStepIndex")) j.at("previewStepIndex").get_to(p.previewStepIndex);

    if(j.contains("ackConfigVersion")) j.at("ackConfigVersion").get_to(p.ackConfigVersion);
    if(j.contains("ackTrajVersion")) j.at("ackTrajVersion").get_to(p.ackTrajVersion);
    if(j.contains("ackProgramVersion")) j.at("ackProgramVersion").get_to(p.ackProgramVersion);

    if(j.contains("configUpdateReqId")) j.at("configUpdateReqId").get_to(p.configUpdateReqId);
    if(j.contains("newConfig")) j.at("newConfig").get_to(p.newConfig);

    if(j.contains("programUpdateReqId")) j.at("programUpdateReqId").get_to(p.programUpdateReqId);
    if(j.contains("newProgram")) j.at("newProgram").get_to(p.newProgram);
}

} // namespace Rdt

#endif // RDT_JSON_H