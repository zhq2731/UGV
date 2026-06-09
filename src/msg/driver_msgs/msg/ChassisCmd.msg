Header header

# 驾驶模式反馈   0:人工, 1:无人, 2:遥控 3 人工reserve
uint8 driving_mode
uint8 mode_flag
uint8 adu_mode
uint8 eps_mode
uint8 epb_mode
uint8 bcm_mode
uint8 xbr_mode
uint8 gear_mode #添加挡位线控，20230808

# 转向模式  0:前轮转向, 1:楔形转向, 2:原地转向, 3:无效
uint8 steer_mode

# steer
float64 steering_wheel_angle #方向盘转角 左正右负
float64 steering_wheel_angle_speed #方向盘转动速度 左正右负

# 前轮转角 单位: °
float64 front_wheel_angle
# 前轮转角速率 单位: °/s
float64 front_wheel_speed
# 后轮转角 单位: °
float64 rear_wheel_angle
# 后轮转角速率 单位: °/s
float64 rear_wheel_speed

# 油门 0-100% 
float64 throttle_pedal
# 刹车 0-100% 
float64 brake_pedal
# 加速度  单位:m/s^2 
float64 acc_target
float64 velocity_target

# 发动机转矩 单位: N
float64 engine_torque_target

# 发动机启动信号 0-熄火 1-点火 2 on 3 start 
uint8 engine_on_off

# 低压上电信号 0:断电, 1:加电, 2:无效
uint8 low_voltage_signal
# 高压上电信号 0:断电, 1:加电, 2:无效
uint8 high_voltage_signal

# 档位  空挡：0 前进档：1-6 倒档：7 无效：8
uint8 gear_location

# 驻车状态  true:驻车, false:解驻车
# 0 Parking brake not applied
# 1 1 Parking brake applied
# 2 Parking brake releasing
# 3 Parking brake applying
# 5 Secondary braking active

uint8 parking_brake
uint8 parking_brake_pct # 外部压力百分比 30~100%

# 急停状态  true:急停, false:急停关闭
bool emergency_stop

# 左转向灯状态  true: 开, false: 关
bool left_light
# 右转向灯状态  true: 开, false: 关
bool right_light

# 远光灯状态  true: 开, false: 关
bool high_light
# 近光灯状态  true: 开, false: 关
bool low_light

# 刹车灯状态  true: 开, false: 关
bool brake_light
# 危险报警灯状态  true: 开, false: 关
bool emergency_light
# 雾灯状态  true: 开, false: 关
bool front_foggy_light
bool rear_foggy_light

# 喇叭  true: 开, false: 关
bool horn

bool position_light

bool wiper

bool reverse_light

bool head_light

bool electromagnetic_silence

# 分动器 （xb项目添加 2024.3.11 yqx）
uint8 tran_gear
# 防空灯
bool air_lamp
#






