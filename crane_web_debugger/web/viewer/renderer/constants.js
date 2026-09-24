export const CONTROL_MODE_LONG = {
    0: 'LOCAL_CAMERA', 1: 'POSITION_TARGET', 2: 'SIMPLE_VELOCITY', 3: 'POLAR_VELOCITY'
};
export const ROBOT_HIT_RADIUS_M = 0.15;
export const BALL_HIT_RADIUS_M = 0.08;
export const FIELD_BOUNDARY_DEFAULT_M = 0.3;

export const ZOOM_MIN = 0.1;
export const ZOOM_MAX = 5.0;

// ロボットの警告閾値。レール・HUD・概要タブで同じ値を使う
export const VOLTAGE_CRIT_V = 21.0;
export const VOLTAGE_WARN_V = 22.5;
export const TEMP_CRIT_C = 75;
export const TEMP_WARN_C = 60;
export const FEEDBACK_STALE_MS = 500;
export const LATENCY_WARN_MS = 100;

// デフォルトフィールドサイズ (div-A: 12×9 m)
export const DEFAULT_FIELD_LENGTH_M = 12.0;
export const DEFAULT_FIELD_WIDTH_M = 9.0;
