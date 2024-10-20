#define SERVOMIN 50
#define SERVOMAX 1024
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

#if 0   // ESP32-s3-dev-board
#define BASE_ANALOG_PIN 1     // for arduino: A0
#define PIN_SDA   40
#define PIN_SCL   41
#define ONOFF_PIN 11
#define STEPPER_PULSE 36
#define STEPPER_DIR 37
#endif

#if 1   // ESP32-c3-mini-board
#define BASE_ANALOG_PIN 3     // for arduino: A0
#define PIN_SDA   8
#define PIN_SCL   9
#define ONOFF_PIN 10
#define STEPPER_PULSE 1
#define STEPPER_DIR 0

#define PS2_DAT 4
#define PS2_CMD 5
#define PS2_SEL 6
#define PS2_CLK 7


#define ANALOG_PIN_MAX  4095  // measured
#define ANALOG_PIN_MIN  815   // measured

#endif