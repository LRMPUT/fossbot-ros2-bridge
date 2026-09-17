# Reflectance line-following sensors, front of the robot.
std_msgs/Header header
float32 left            # normalised 0..1, 0 = dark (on line)
float32 middle
float32 right
uint16 left_raw         # raw MCP3008 counts, 0..1023
uint16 middle_raw
uint16 right_raw
bool on_line_left       # raw below the on_line_threshold parameter
bool on_line_middle
bool on_line_right
