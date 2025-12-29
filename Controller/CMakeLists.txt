# ============================================================================
# CMakeLists.txt for HexaMotion Controller
# ============================================================================
cmake_minimum_required(VERSION 3.18)

# ============================================================================
# Project Definition
# ============================================================================
project(HexaMotionController LANGUAGES CXX)

# ============================================================================
# Set path to local dependencies
# ============================================================================
set(LOCAL_DEPENDENCY_PREFIX "/home/rdt/Desktop/HexaMotion/libs")
list(APPEND CMAKE_PREFIX_PATH "${LOCAL_DEPENDENCY_PREFIX}")
message(STATUS "Adding custom search path for dependencies: ${LOCAL_DEPENDENCY_PREFIX}")


# ============================================================================
# C++ Standard Configuration
# ============================================================================
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ============================================================================
# Find Dependencies
# ============================================================================
find_package(Threads REQUIRED)
find_package(Eigen3 3.4 REQUIRED NO_MODULE)
find_package(orocos_kdl REQUIRED)

# ============================================================================
# Executable Target
# ============================================================================
add_executable(HexaMotion
    # Main entry point and network bridge
    src/main.cpp
    src/HmiBridge.cpp

    # Core Logic Modules (monolithic build)
    ../Modules/robot_controller_nrt/RobotController.cpp
    ../Modules/motion_manager_rt/MotionManager.cpp
    ../Modules/motion_interface_hal/MasterHardwareInterface.cpp
    ../Modules/motion_interface_hal/InternalSimulation.cpp
    ../Modules/motion_interface_hal/UdpInterface.cpp
    # *** NEW: Add EtherCAT source file to build ***
    ../Modules/motion_interface_hal/ethercat/EthercatInterface.cpp
    ../Modules/motion_interface_hal/ethercat/EthercatAxis.cpp
    ../Modules/trajectory_planner_nrt/TrajectoryPlanner.cpp
    ../Modules/interpolator_nrt/TrajectoryInterpolator.cpp
    ../Modules/interpolator_nrt/MotionProfile.cpp
    ../Modules/kinematic_solver_nrt/KinematicModel.cpp
    ../Modules/kinematic_solver_nrt/KdlKinematicSolver.cpp
    ../Modules/data_types_rdt/DataTypes.cpp
)

# ============================================================================
# Include Directories
# ============================================================================
target_include_directories(HexaMotion PRIVATE
    # Directory for main.cpp and HmiBridge.h
    src

    # Project-wide shared folder for protocol definitions
    ../Modules/shared

    # All required project modules
    ../Modules/data_types_rdt
    ../Modules/logger
    ../Modules/trajectory_queue_lf
    ../Modules/motion_interface_hal
    # *** FIX: Add subdirectory for EtherCAT headers ***
    ../Modules/motion_interface_hal/ethercat
    ../Modules/kinematic_solver_nrt
    ../Modules/interpolator_nrt
    ../Modules/trajectory_planner_nrt
    ../Modules/motion_manager_rt
    ../Modules/robot_controller_nrt
    ../Modules/state_data_nrt
    ../Modules/spatial_math_

    # External dependencies
    ${EIGEN3_INCLUDE_DIR}
    ${orocos_kdl_INCLUDE_DIRS}
)

# ============================================================================
# Link Libraries
# ============================================================================
target_link_libraries(HexaMotion PRIVATE
    Threads::Threads
    Eigen3::Eigen
    ${orocos_kdl_LIBRARIES}
    atomic # Required for std::atomic
)

# ============================================================================
# Compiler Flags and Options
# ============================================================================
if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(HexaMotion PRIVATE
        -Wall
        -Wextra
        -Wpedantic
        -Werror=unused-result # Treat [[nodiscard]] as an error
        -pthread # Ensure thread flags are passed
    )
endif()

# ============================================================================
# Installation (Optional but good practice)
# ============================================================================
install(TARGETS HexaMotion
    RUNTIME DESTINATION bin
)

message(STATUS "HexaMotion controller executable configured.")
message(STATUS "  C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "  Eigen3 found: ${EIGEN3_INCLUDE_DIR}")
message(STATUS "  Orocos-KDL found: ${orocos_kdl_LIBRARIES}")