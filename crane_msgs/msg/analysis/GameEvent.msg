# SSL Game Controller Game Event Message
# Unified message for all SSL game controller game events
# Uses NamedValueArray pattern to consolidate all event types into single message

# Event type (e.g., "BALL_LEFT_FIELD_TOUCH_LINE", "GOAL", "BOT_KICKED_BALL_TOO_FAST")
string event_type

# Origins of this game event
# Empty if it originates from game controller
# Contains autoRef name(s) if it originates from one or more autoRefs
string[] origin

# Common fields present in most events
string team        # Team name (YELLOW/BLUE/UNKNOWN)
uint32 robot_id    # Robot ID (0 if not applicable)

# Event-specific data using named value arrays
# This allows flexible storage of event-specific fields:
NamedBool[] bool_values
NamedInt[] int_values
NamedFloat[] float_values
NamedString[] string_values
NamedPosition[] position_values

