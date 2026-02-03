std_msgs/Header header
uint16 id           # 目标ID：1=蓝方 2=红方
float32 pitch       # 当前小云台pitch角度
float32 yaw         # 当前小云台yaw角度
float32 roll        # 当前roll角度
float32[] quaternion  # 四元数姿态
float32 shoot_spd   # 射速 m/s
float32 main_yaw    # 大云台yaw角度
