# 传感器融合结果 (mechdog_navigation SensorFusion::fuse() 输出)
# 发布到 /fusion_result, 供调试与下游 (巡检决策) 使用

# 融合时间戳 (秒, system_clock epoch)
float64 timestamp

# 环境类型: 0=UNKNOWN 1=INDOOR 2=SEMI_INDOOR 3=OUTDOOR
int32 environment

# 悬崖/跌落风险 (底部超声波)
bool cliff_detected

# 前方最小障碍距离 (米)
float64 min_forward_distance_m

# 推荐动作: 0=STOP 1=BACKWARD 2=TURN_LEFT 3=TURN_RIGHT 4=SLOW_FORWARD 5=FORWARD 6=REACHED_GOAL
int32 recommended_action

# 环境自适应权重 (深度相机 vs 超声波)
float64 effective_astra_weight
float64 effective_ultrasonic_weight
