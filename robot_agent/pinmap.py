# FOSSBot v2 main PCB -- pin map extracted & verified from KiCad schematic
# RPi BCM GPIO numbering.

# --- I2C (bus 1, GPIO2=SDA / GPIO3=SCL) ---
I2C_BUS      = 1
MPU6050_ADDR = 0x68     # J22. Some boards carry an MPU-6500 here (WHO_AM_I 0x70).
SSD1306_ADDR = 0x3C     # SCR1 OLED

# --- SPI (bus 0): two MCP3008 ADCs ---
# CLK=GPIO11, MOSI(Din)=GPIO10, MISO(Dout)=GPIO9
ADC_U8_CE = 0   # spidev0.0  (CE0/GPIO8)
ADC_U7_CE = 1   # spidev0.1  (CE1/GPIO7)

# MCP3008 channel -> sensor
ADC_CHANNELS = {
    # U8 (spidev0.0)
    ("U8", 4): "DIST_BL (IR back-left)",
    ("U8", 5): "DIST_FL (IR front-left)",
    ("U8", 6): "FL_LEFT (line left)",
    ("U8", 7): "FL_M   (line middle)",
    # U7 (spidev0.1)
    ("U7", 0): "LDR (light)",
    ("U7", 1): "MIC (MAX4466)",
    ("U7", 2): "FL_RIGHT (line right)",
    ("U7", 3): "A_CH3 (spare header J10)",
    ("U7", 4): "PD (photodiode)",
    ("U7", 5): "DIST_BR (IR back-right)",
    ("U7", 6): "A_CH6 (spare header J11)",
    ("U7", 7): "DIST_FR (IR front-right)",
}

# --- Motor driver U1 = TB6612FNG (ROB-14450) ---
MOTOR_STBY = 6          # must be HIGH to enable driver
# Motor A -> J5 (MOTOR_A), drives the left wheel
MOTOR_A_PWM = 12
MOTOR_A_IN1 = 19
MOTOR_A_IN2 = 26
# Motor B -> J29 (MOTOR_B), drives the right wheel
MOTOR_B_PWM = 13
MOTOR_B_IN1 = 5
MOTOR_B_IN2 = 0

# --- Encoders / odometry ---
ENC_LEFT  = 1           # J6  L_ODO
ENC_RIGHT = 25          # J7  R_ODO

# --- RGB LED D1 (common cathode, active HIGH via 270R) ---
LED_R = 16
LED_G = 20
LED_B = 21

# --- Buzzer BZ1 (via NPN Q2, active HIGH) ---
BUZZER = 22

# --- Push buttons (active LOW, pull-up to 3V3) ---
BUTTONS = {
    "SW1": 27,
    "SW2": 17,
    "SW3": 4,
    "SW4": 14,
}

# --- Ultrasonic HC-SR04 (J26) : GPIO23 = trigger, GPIO24 = echo ---
ULTRA_A = 23
ULTRA_B = 24

# --- 3-pin expansion "D5" headers (servo/PWM/LED) ---
EXP_J24 = 18    # signal on GPIO18
EXP_J25 = 15    # signal on GPIO15
