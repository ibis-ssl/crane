uint8 robot_id

# local goalie system(goalie only)
bool local_goalie_enable false
# enable local control by FC sensor for centering ball in dribbler
bool enable_ball_centering_control false

crane_msgs/LocalPlannerConfig local_planner_config

# kick flag(chip / straight)
bool chip_enable false

# emergency stop flag (stop wheel motors immediately)
bool stop_flag false

# if this flag is true, the robot will lift up dribbler (but chip_enable has priority)
bool lift_up_dribbler_flag false

# kick power(0.0 - 1.0)
float32 kick_power 0.0

# dribble power(0.0 - 1.0)
float32 dribble_power 0.0

geometry_msgs/Pose2D current_velocity

geometry_msgs/Pose2D current_pose

bool enable_local_feedback

float32 target_theta
float32 omega_limit 10.0
# for local planner
float32 theta_tolerance 0.0

float32 latency_ms 0.0

uint16 elapsed_time_ms_since_last_vision 0

uint8 LOCAL_CAMERA_MODE=0
uint8 POSITION_TARGET_MODE=1
uint8 SIMPLE_VELOCITY_TARGET_MODE=2
uint8 VELOCITY_TARGET_MODE=3

uint8 control_mode

string skill_name

LocalCameraMode[<=1] local_camera_mode
PositionTargetMode[<=1] position_target_mode
SimpleVelocityTargetMode[<=1] simple_velocity_target_mode
VelocityTargetMode[<=1] velocity_target_mode
