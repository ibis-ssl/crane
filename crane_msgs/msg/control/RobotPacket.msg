uint8 robot_id

# local goalie system(goalie only)
bool local_goalie_enable false
# enable local control by FC sensor for centering ball in dribbler
bool enable_ball_centering_control false

crane_msgs/LocalPlannerConfig local_planner_config

uint8 CHECK = 0
uint8 VEL_SURGE_HIGH = 1
uint8 VEL_SURGE_LOW = 2
uint8 VEL_SWAY_HIGH = 3
uint8 VEL_SWAY_LOW = 4
uint8 VISION_GLOBAL_THETA_HIGH = 5
uint8 VISION_GLOBAL_THETA_LOW = 6
uint8 TARGET_GLOBAL_THETA_HIGH = 7
uint8 TARGET_GLOBAL_THETA_LOW = 8
uint8 KICK_POWER = 9
uint8 DRIBBLE_POWER = 10
uint8 KEEPER_ENABLE = 11
uint8 BALL_GLOBAL_X_HIGH = 12
uint8 BALL_GLOBAL_X_LOW = 13
uint8 BALL_GLOBAL_Y_HIGH = 14
uint8 BALL_GLOBAL_Y_LOW = 15
uint8 VISION_GLOBAL_X_HIGH = 16
uint8 VISION_GLOBAL_X_LOW = 17
uint8 VISION_GLOBAL_Y_HIGH = 18
uint8 VISION_GLOBAL_Y_LOW = 19
uint8 TARGET_GLOBAL_X_HIGH = 20
uint8 TARGET_GLOBAL_X_LOW = 21
uint8 TARGET_GLOBAL_Y_HIGH = 22
uint8 TARGET_GLOBAL_Y_LOW = 23
uint8 LOCAL_FEEDBACK_ENABLE = 24

# kick flag(chip / straight)
bool chip_enable false

# kick power(0.0 - 1.0)
float32 kick_power 0.0

# dribble power(0.0 - 1.0)
float32 dribble_power 0.0

# target[vx, vy, theta]
geometry_msgs/Pose2D target_velocity

geometry_msgs/Pose2D current_pose

bool enable_local_feedback

float32[] target_x
float32[] target_y
float32[] target_theta

float32 laytency_ms 0.0

float32 current_ball_x
float32 current_ball_y


# movement mode(transfer / motion)
bool motion_mode_enable
