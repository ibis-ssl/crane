builtin_interfaces/Time received_stamp
uint8 robot_id
float32 packet_frequency_hz
float32 receive_interval_mean_ms
float32 receive_interval_min_ms
float32 receive_interval_max_ms
float32 receive_interval_stddev_ms
float32 feedback_age_ms

uint32 valid_packet_count
uint32 invalid_packet_count
uint32 sync_error_count
uint32 checksum_error_count
uint32 size_mismatch_count
uint32 counter_jump_count

uint8 counter

uint16 kick_state

uint8 TEMP_MOTOR_1=0
uint8 TEMP_MOTOR_2=1
uint8 TEMP_MOTOR_3=2
uint8 TEMP_MOTOR_4=3
uint8 TEMP_FET=4
uint8 TEMP_COIL_1=5
uint8 TEMP_COIL_2=6

uint8[] temperatures

uint16 ERR_INFO_POWER_NONE           = 0x0000
uint16 ERR_INFO_POWER_UNDER_VOLTAGE  = 0x0001
uint16 ERR_INFO_POWER_OVER_VOLTAGE   = 0x0002
uint16 ERR_INFO_POWER_OVER_CURRENT   = 0x0004
uint16 ERR_INFO_POWER_SHORT_CIRCUIT  = 0x0008
uint16 ERR_INFO_POWER_CHARGE_TIME    = 0x0010
uint16 ERR_INFO_POWER_CHARGE_POWER   = 0x0020
uint16 ERR_INFO_POWER_DISCHARGE      = 0x0040
uint16 ERR_INFO_POWER_PARAMETER      = 0x0080
uint16 ERR_INFO_POWER_COMMAND        = 0x0100
uint16 ERR_INFO_POWER_NO_CAP         = 0x0200
uint16 ERR_INFO_POWER_DISCHARGE_FAIL = 0x0400
uint16 ERR_INFO_POWER_GD_POWER_FAIL  = 0x0800
uint16 ERR_INFO_POWER_COIL_OVER_HEAT = 0x1000
uint16 ERR_INFO_POWER_FET_OVER_HEAT  = 0x2000

uint16 ERR_INFO_BLDC_NONE            = 0x0000
uint16 ERR_INFO_BLDC_UNDER_VOLTAGE   = 0x0001
uint16 ERR_INFO_BLDC_OVER_CURRENT    = 0x0002
uint16 ERR_INFO_BLDC_MOTOR_OVER_HEAT = 0x0004
uint16 ERR_INFO_BLDC_OVER_LOAD       = 0x0008
uint16 ERR_INFO_BLDC_ENC_ERROR       = 0x0010
uint16 ERR_INFO_BLDC_OVER_VOLTAGE    = 0x0020
uint16 ERR_INFO_BLDC_FET_OVER_HEAT   = 0x0040

uint16 ERR_ID_POWER = 100
uint16 ERR_ID_BLDC_RF = 0
uint16 ERR_ID_BLDC_RB = 1
uint16 ERR_ID_BLDC_LB = 2
uint16 ERR_ID_BLDC_LF = 3

uint16 error_id
uint16 error_info
float32 error_value

uint8 CURRENT_1=0
uint8 CURRENT_2=1
uint8 CURRENT_3=2
uint8 CURRENT_4=3

float32[] motor_current
uint8[] ball_detection

bool ball_sensor

float32 yaw_angle
float32 diff_angle
float32[] odom
float32[] odom_speed
float32[] mouse_odom
float32[] mouse_vel

float32[] voltage

uint8 camera_pos_x_div2
uint8 camera_pos_y
uint8 camera_radius_div4
uint8 camera_fps
uint8 check_ver  # legacy alias of camera_pos_x_div2

uint8 VALUES_MOUSE_ODOM_1=0
uint8 VALUES_MOUSE_ODOM_2=1
uint8 VALUES_GLOBAL_VEL_1=2
uint8 VALUES_GLOBAL_VEL_2=3
uint8 VALUES_OUT_VEL_1=4
uint8 VALUES_OUT_VEL_2=5
uint8 VALUES_MOTOR_FEEDBACK_1=6
uint8 VALUES_MOTOR_FEEDBACK_2=7
uint8 VALUES_MOTOR_FEEDBACK_3=8
uint8 VALUES_MOTOR_FEEDBACK_4=9
uint8 VALUES_LOCAL_ODOM_SPEED_MVF_1=10
uint8 VALUES_LOCAL_ODOM_SPEED_MVF_2=11
uint8 VALUES_LOCAL_ODOM_SPEED_MVF_3=12
uint8 VALUES_MOUSE_QUALITY=13

float32[] values
