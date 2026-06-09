Header header
# 驾驶模式反馈   0:人工, 1:无人, 2:遥控
uint8 driving_mode
uint8 adu_mode
uint8 eps_mode
uint8 epb_mode
uint8 xbr_mode
uint8 bcm_mode

#轻卡
bool manual_steeringwheel
bool manual_brake_padel
bool manual_throttle_padel

#坦克
bool brake_intervene
bool estop_intervene
bool steer_intervene
bool timeout_status
uint8 remote_button_status

# Ready 后才会进入
uint8 ready

