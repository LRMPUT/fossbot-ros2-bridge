# Health of the PC <-> robot link, published by the bridge.
std_msgs/Header header
bool connected              # a telemetry frame arrived recently
float32 telemetry_hz        # measured receive rate
float32 latency_ms          # round trip: command sent -> applied -> echoed back
uint32 frames_received
uint32 frames_lost          # inferred from sequence-number gaps
float32 loss_percent
bool motors_enabled
bool estop
bool imu_ok
bool lidar_ok
float32 supply_volts        # 5V input rail at the Pi, from its PMIC. NaN if unknown.
float32 supply_amps         # total current across the PMIC rails
bool low_voltage            # rail below the warning threshold
string robot_host           # robot this bridge talks to
float32 duty_left           # motor duty the robot is applying, -1..1
float32 duty_right
