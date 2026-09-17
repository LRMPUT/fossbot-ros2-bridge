# All 16 MCP3008 channels, unprocessed. Index 0..7 = U8 ch0..7 (spidev0.0),
# index 8..15 = U7 ch0..7 (spidev0.1). See pinmap.py on the robot.
std_msgs/Header header
uint16[16] counts       # 0..1023
float32[16] volts       # counts scaled to the 3.3 V reference
