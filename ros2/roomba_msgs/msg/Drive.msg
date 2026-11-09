# Opcode 137: drive along an arc. Requires Safe or Full mode; subject to the cmd_vel timeout.
int16 RADIUS_STRAIGHT=32767
int16 RADIUS_TURN_CLOCKWISE=-1
int16 RADIUS_TURN_COUNTER_CLOCKWISE=1

int16 velocity_mm_s   # -500 ... 500
int16 radius_mm       # -2000 ... 2000, or one of the special values above
