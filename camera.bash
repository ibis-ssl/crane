device_id = 0
v4l2-ctl -d $device_id -c exposure_dynamic_framerate=0
v4l2-ctl -d $device_id -c auto_exposure=1
v4l2-ctl -d $device_id -c focus_automatic_continuous=0
v4l2-ctl -d $device_id -c focus_absolute=0


v4l2-ctl -d $device_id -c exposure_time_absolute=120
v4l2-ctl -d $device_id -c brightness=120
v4l2-ctl -d $device_id -c sharpness=400
