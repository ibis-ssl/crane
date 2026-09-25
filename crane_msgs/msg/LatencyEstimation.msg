uint8 robot_id
string source          # "world_model" or "robot_feedback"
float32 latency_ms     # EMA-smoothed lag. An entry is published only when the current window is estimated, so never NaN
float32 correlation    # normalized cross-correlation peak value
uint32 samples_used    # number of raw cmd samples in the buffer (before resampling)
float32 cmd_stddev     # std dev of cmd signal (confidence indicator)
