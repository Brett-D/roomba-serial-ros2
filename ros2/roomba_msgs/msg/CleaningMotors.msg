# Main brush, side brush and vacuum. Requires Safe or Full mode.
# use_pwm = false: on/off at full speed (opcode 138).
# use_pwm = true:  variable speed (opcode 144).
bool use_pwm

bool main_brush
bool main_brush_outward
bool side_brush
bool side_brush_clockwise
bool vacuum

int8 main_brush_pwm   # -127 ... 127 (negative = reverse)
int8 side_brush_pwm   # -127 ... 127 (negative = reverse)
uint8 vacuum_pwm      # 0 ... 127
