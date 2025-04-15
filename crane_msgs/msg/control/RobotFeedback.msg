builtin_interfaces/Time received_stamp
uint8 robot_id

uint8 counter

uint8 kick_state

uint8 TEMP_MOTOR_1=0
uint8 TEMP_MOTOR_2=1
uint8 TEMP_MOTOR_3=2
uint8 TEMP_MOTOR_4=3
uint8 TEMP_FET=4
uint8 TEMP_COIL_1=5
uint8 TEMP_COIL_2=6

uint8[] temperatures

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

uint8 check_ver

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
