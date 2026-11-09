# Every sensor the Roomba 500-series Open Interface reports (sensor group 100, packets 7-58).
# Field names match roomba_driver.oi decoded keys.

uint8 MODE_OFF=0
uint8 MODE_PASSIVE=1
uint8 MODE_SAFE=2
uint8 MODE_FULL=3

uint8 CHARGING_NOT_CHARGING=0
uint8 CHARGING_RECONDITIONING=1
uint8 CHARGING_FULL=2
uint8 CHARGING_TRICKLE=3
uint8 CHARGING_WAITING=4
uint8 CHARGING_FAULT=5

# Infrared characters (ir_omni / ir_left / ir_right). 0 = nothing received;
# 128-141 are remote-control buttons.
uint8 IR_NONE=0
uint8 IR_DOCK_FORCE_FIELD=161
uint8 IR_VIRTUAL_WALL=162
uint8 IR_DOCK_GREEN_BUOY=164
uint8 IR_DOCK_GREEN_BUOY_FORCE_FIELD=165
uint8 IR_DOCK_RED_BUOY=168
uint8 IR_DOCK_RED_BUOY_FORCE_FIELD=169
uint8 IR_DOCK_RED_GREEN_BUOY=172
uint8 IR_DOCK_RED_GREEN_BUOY_FORCE_FIELD=173

std_msgs/Header header

# Packet 7: bumps and wheel drops
bool bump_left
bool bump_right
bool wheel_drop_left
bool wheel_drop_right

# Packets 8-13
bool wall
bool cliff_left
bool cliff_front_left
bool cliff_front_right
bool cliff_right
bool virtual_wall

# Packet 14: wheel / brush overcurrents
bool overcurrent_left_wheel
bool overcurrent_right_wheel
bool overcurrent_main_brush
bool overcurrent_side_brush

# Packets 15, 17, 52, 53
uint8 dirt_detect
uint8 ir_omni
uint8 ir_left
uint8 ir_right

# Packet 18: buttons currently held
bool button_clean
bool button_spot
bool button_dock
bool button_minute
bool button_hour
bool button_day
bool button_schedule
bool button_clock

# Packets 19-20: firmware odometry since the previous packet (unreliable on some 500 firmware;
# use the encoder counts instead)
int16 distance_mm
int16 angle_deg

# Packets 21-26: battery
uint8 charging_state
uint16 voltage_mv
int16 current_ma          # negative = discharging
int8 temperature_c
uint16 charge_mah
uint16 capacity_mah

# Packets 27-31: raw IR signal strengths (0-4095)
uint16 wall_signal
uint16 cliff_left_signal
uint16 cliff_front_left_signal
uint16 cliff_front_right_signal
uint16 cliff_right_signal

# Packet 34: charging sources available
bool internal_charger_available
bool home_base_available

# Packets 35-42
uint8 oi_mode
uint8 song_number
bool song_playing
uint8 stream_packet_count
int16 requested_velocity_mm_s
int16 requested_radius_mm
int16 requested_right_velocity_mm_s
int16 requested_left_velocity_mm_s

# Packets 43-44: raw wheel encoder counts (wrap at 65536)
uint16 left_encoder_counts
uint16 right_encoder_counts

# Packet 45: light bumper detections
bool light_bump_left
bool light_bump_front_left
bool light_bump_center_left
bool light_bump_center_right
bool light_bump_front_right
bool light_bump_right

# Packets 46-51: light bumper signal strengths (0-4095)
uint16 light_bump_left_signal
uint16 light_bump_front_left_signal
uint16 light_bump_center_left_signal
uint16 light_bump_center_right_signal
uint16 light_bump_front_right_signal
uint16 light_bump_right_signal

# Packets 54-57: motor currents
int16 left_motor_current_ma
int16 right_motor_current_ma
int16 main_brush_current_ma
int16 side_brush_current_ma

# Packet 58: caster wheel is turning (robot is actually moving forward)
bool stasis
