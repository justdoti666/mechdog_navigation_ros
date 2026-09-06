# UltrasonicArray.msg
# 4 颗 HC-SR04 超声波传感器阵列读数 (方案A: 独立 ultrasonic_node 发布)
#
# 方向命名与算法库 config.h get_ultrasonic_layout() 对齐:
#   front_left / front_center / front_right / bottom

# 毫秒级时间戳 (unix, 秒.微秒用 int32? 用 float64 统一)
builtin_interfaces/Time stamp

# 单颗读数 (4 组, 顺序=上方 directions 的顺序)
# 用数组长度固定 4
float64 front_left_cm      # 左前 距离(cm), 无效=NaN
float64 front_center_cm    # 正前 距离(cm), 无效=NaN
float64 front_right_cm     # 右前 距离(cm), 无效=NaN
float64 bottom_cm          # 底部朝下 距离(cm), 无效=NaN

# 各方向是否为有效读数 (fail-closed: 无效要明确标出, 避免被当 8.0m 兜底)
bool front_left_valid
bool front_center_valid
bool front_right_valid
bool bottom_valid

# 测量周期 (秒) / 本轮序号 (供消费方判断新鲜度)
float32 period_sec
uint32 seq
