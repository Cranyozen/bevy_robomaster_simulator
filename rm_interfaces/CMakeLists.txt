cmake_minimum_required(VERSION 3.8)
project(rm_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Armor.msg"
  "msg/Armors.msg"
  "msg/Target.msg"
  "msg/RuneTarget.msg"
  "msg/Point2d.msg"
  "msg/GimbalCmd.msg"
  "msg/ChassisCmd.msg"
  "msg/DebugLight.msg"
  "msg/DebugLights.msg"
  "msg/DebugArmor.msg"
  "msg/DebugArmors.msg"
  "msg/DebugRuneAngle.msg"
  "msg/Measurement.msg"
  "msg/JudgeSystemData.msg"
  "msg/OperatorCommand.msg"
  "msg/SerialReceiveData.msg"
  "msg/PlanGimbalCmd.msg"
  # Benchmark v2 ground truth and events. Same `rm_interfaces` package on
  # purpose: the simulator repo is the only thing a benchmark run builds, and a
  # second package would mean a second colcon workspace to keep in sync.
  "msg/SimRobotState.msg"
  "msg/SimRobotStates.msg"
  "msg/SimArmorState.msg"
  "msg/SimArmorStates.msg"
  "msg/SimEvent.msg"
  "msg/SimScenarioStatus.msg"
  # Benchmark v2, P1-B: runtime scenario control. `load` replaces the scenario and
  # stops the clock, `start` (re)starts it at t=0 and overrides the TOML trigger,
  # `reset` is idempotent and puts vehicles/clock/event state back.
  "srv/SimScenarioLoad.srv"
  "srv/SimScenarioStart.srv"
  "srv/SimScenarioReset.srv"
  "srv/SetMode.srv"
  "srv/RunHandEyeCalibration.srv"
  DEPENDENCIES
    std_msgs
    geometry_msgs
)


ament_package()
