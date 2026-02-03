cmake_minimum_required(VERSION 3.8)
project(robot_msgs)

# 复制自 auto_aim_interfaces 示例
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies (保留你自己的依赖)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# 按照 auto_aim_interfaces 的风格，直接列出所有文件
rosidl_generate_interfaces(${PROJECT_NAME}
  # 你自己的 MSG 文件列表
  "msg/VisionPts.msg"
  "msg/Vision.msg"
  "msg/RobotCtrl.msg"
  "msg/PTZYaw.msg"
  "msg/Barrel.msg"
  "msg/PTZPerception.msg"
  "msg/TrackReset.msg"
  "msg/YawDecision.msg"
  "msg/CompetitionInfo.msg"
  "msg/AttackBase.msg"
  "msg/MainYawCtrl.msg"
  "msg/UsbCamera.msg"
  "msg/OmniPerception.msg"
  "msg/RobotHp.msg"
  "msg/SentryVision.msg"
  "msg/EnemyInfo.msg"
  "msg/MainYawcommunication.msg"
  "msg/ModeControl.msg"
  "msg/Armor.msg"
  "msg/Armors.msg"
  "msg/Target.msg"
  "msg/TrackerInfo.msg"
  "msg/TimeInfo.msg"
  "msg/DebugArmor.msg"
  "msg/DebugArmors.msg"
  "msg/DebugLight.msg"
  "msg/DebugLights.msg"
  "msg/RfidStatus.msg"
  
  # 你自己的 SRV 文件列表
  "srv/ChassisMoveStatus.srv"

  # 按照 auto_aim_interfaces 的风格，在末尾声明依赖
  DEPENDENCIES
    std_msgs
    geometry_msgs
    builtin_interfaces # 你自己的依赖
)

ament_package()
