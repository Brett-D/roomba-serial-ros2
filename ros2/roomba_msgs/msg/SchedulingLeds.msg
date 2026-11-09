# Opcode 162. Requires Safe or Full mode.
uint8 SUNDAY=1
uint8 MONDAY=2
uint8 TUESDAY=4
uint8 WEDNESDAY=8
uint8 THURSDAY=16
uint8 FRIDAY=32
uint8 SATURDAY=64

uint8 weekdays   # bitmask of the day constants above
bool schedule
bool clock
bool am
bool pm
bool colon
