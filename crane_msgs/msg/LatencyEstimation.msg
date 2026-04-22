uint8 robot_id
string source          # "world_model" or "robot_feedback"
float32 latency_ms     # NaN = not estimated (low variance or low correlation)
float32 correlation    # normalized cross-correlation peak value
uint32 samples_used
float32 cmd_stddev     # std dev of cmd signal (confidence indicator)
