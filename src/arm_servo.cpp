
#include "constants.h"

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "control.h"
// #include <OneButton.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_I2C_ADDRESS, Wire);

// #define SERVOMIN  103 // This is the 'minimum' pulse length count (out of 4096)
// #define SERVOMAX  512 // This is the 'maximum' pulse length count (out of 4096)

#include "ps2_utils.h"

PS2Controller ps2_controller;

// OneButton btnOnOff = OneButton(
//   ONOFF_PIN,  // Input pin for the button
//   true,        // Button is active LOW
//   true         // Enable internal pull-up resistor
// );

AccelStepper stepper(AccelStepper::DRIVER, STEPPER_PULSE, STEPPER_DIR); // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5

BaseRotate baseRotate( stepper, BASE_ANALOG_PIN, ANALOG_PIN_MIN, ANALOG_PIN_MAX);
bool diag_mode = false;

// our servo # counter
uint8_t servonum = 0;

PWMServo* servos[6] = {
  new PWMServo(pwm, 0, 120, 490),
  new PWMServo(pwm, 2, 120, 490),
  new PWMServo(pwm, 4, 150, 490),
  new PWMServo(pwm, 6, 120, 490,-1),  // reverse angle 
  new PWMServo(pwm, 8, 120, 480),  // combo servo
  new PWMServo(pwm, 12, 110, 450, -1), 
};

Joint* joints[5] = {
  new Joint(servos[0]),
  new Joint(servos[1]),
  new Joint(servos[2]),
  new Joint(servos[3]),
  new Joint(servos[4], servos[5]),
};

struct ArmState{
  float x;
  float y;
  float z_angle;
} arm_state;

static int servo_park_pos[] = {200,320,200,300,250};
static int servo_init_pos[] = {90,90,90,90,120};

void servo_reset() {
  for( int i=4;i>=0;i--) {
    // if( servo_init_pos[i] > 0)
    joints[i]->move_to( servo_init_pos[i]);
    delay(500);
  }
}

// void servo_park() {
//   servo_reset();
//   for( int i=4;i>=0;i--) {
//     if( servo_park_pos[i] > 0)
//       joints[i]->moveTo( servo_park_pos[i]);
//   }
// }
void handleReset() {
      // pwm.reset();
      servo_reset();
}



bool arm_move_to(float x, float y, int duration_in_ms = 1000) {
  static float a1 = 18.5, a2 = 10.5;
  double r = sqrt((x * x) + (y * y));
 double phi_2 = acos(((a2 * a2)-(a1 * a1)-(r * r))/(-2 * a1 * r)) * RAD_TO_DEG;
   double phi_1 = acos(((y * y)-(r * r)-(x * x))/(-2 * r * x)) * RAD_TO_DEG;
   double phi_3 = acos(((r * r)-(a1 * a1)-(a2 * a2))/(-2 * a1 * a2)) * RAD_TO_DEG;

  if( isnan(phi_1)) phi_1 = 90;

   double theta_1 = phi_2 + phi_1;
   double theta_2 = phi_3;
  
  Serial.println( "Target angles:" + String(theta_1) + "," + String(theta_2));
  if( isnan( theta_1) || isnan(theta_2) ) {
    return false;
  }

   double angles[5][2] = {{90}, {90}, {180}, {theta_2}, {theta_1}};  // target , step
  int steps = duration_in_ms / 20;
  if( steps ) {
    for( int i=0;i<5;i++)
      angles[i][1] = ( angles[i][0] - joints[i]->get_angle() ) / steps;
    for( int j=0;j<steps;j++) {
      String output;
      for( int i=0;i<5;i++) {
        double newv = joints[i]->get_angle() + angles[i][1];
        joints[i]->move_to( newv );
        if( i == 0) output = newv;
        else
          output += "," + String(newv);
      }
      Serial.println( output) ;
      delay(20);
    }
  }
  Serial.println("...");
  for( int i=0;i<5;i++) {
    Serial.println(i);
    joints[i]->move_to( angles[i][0] );
  }

  Serial.println("Done");
  return true;
}

int ps2_event(int id, int value) {
  Serial.println("ps2 event:" + String(id) + ",v="+ String(value));
  //
  switch(id) {
    case PSS_RY:
      arm_state.y += value * 0.5;
      break;
    case PSS_RX:
      arm_state.x += value * 0.5;
      break;
    
    case PSS_LX:
      arm_state.z_angle += value*1.0;
      break;
    
    case PSB_PAD_UP:
      arm_state = {15,15,0};
      break;
    case PSB_PAD_DOWN:
      arm_state = {15,0,0};
      break;

    default:
      return 0;
  }

  arm_move_to( arm_state.x, arm_state.y );
  Serial.println("arm_move_to done!");
  return 1;
}

void setup() {
  Serial.begin(115200);
  Serial.println("8 channel Servo test!");

  delay(100);
  ps2_controller.setup();
  ps2_controller.set_event_handler( ps2_event );

  // btnOnOff.attachClick(handleReset);

  Wire.setPins(PIN_SDA, PIN_SCL);
  pwm.begin();
  /*
   * In theory the internal oscillator (clock) is 25MHz but it really isn't
   * that precise. You can 'calibrate' this by tweaking this number until
   * you get the PWM update frequency you're expecting!
   * The int.osc. for the PCA9685 chip is a range between about 23-27MHz and
   * is used for calculating things like writeMicroseconds()
   * Analog servos run at ~50 Hz updates, It is importaint to use an
   * oscilloscope in setting the int.osc frequency for the I2C PCA9685 chip.
   * 1) Attach the oscilloscope to one of the PWM signal pins and ground o、n
   *    the I2C PCA9685 chip you are setting the value for.
   * 2) Adjust setOscillatorFrequency() until the PWM update frequency is the
   *    expected value (50Hz for most ESCs)
   * Setting the value here is specific to each individual I2C PCA9685 chip and
   * affects the calculations for the PWM update frequency. 
   * Failure to correctly set the int.osc value will cause unexpected PWM results
   */
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  // for  esp32, the value is 4096
  pinMode(BASE_ANALOG_PIN, INPUT);    // analog input
  delay(10);

  servo_reset();    // calc 
  arm_state.x = 15;
  arm_state.y = 15;
  arm_state.z_angle = 0;

  baseRotate.reset();
  Serial.println("start");
}

void control(char *buf) {
    char cmd;
    static char temp[20];
    cmd = buf[0];       // format: cmd ****
    char* ptr = buf+2;  // point to char after 'space' 
    switch(cmd) {
      case 'j': {
        int id,angle;
        if( 2 == sscanf(ptr, "%d %d", &id, &angle) ) {
            joints[id]->move_to(angle);
            goto success;
        }
        // error goes here
        break;
      }
      case 'b': {
        int len = SerialHelper::parse_next(ptr, temp);
        if( len ) {
          int angle = atoi(temp);
          baseRotate.setTargetPos( angle );
          goto success;
        }
        break;
      }
      case 'r': {
        // pwm.reset();
        servo_reset();
        goto success;
      }
      case 'm': {
        float x,y;
        if( 2 == sscanf(ptr, "%f %f", &x, &y) ) {
          if( arm_move_to(x,y)) 
            goto success;
          Serial.println( "Invalid position");
        }
        break;
      }

      case '*': {
        int id,pulse;
        if( 2 == sscanf(ptr, "%d %d", &id, &pulse) ) {
          pwm.setPWM(id,0,pulse);
          goto success;
        }
        break;
      }

      case 'd': {
        Serial.println("Joint status:");
        for( int i=0;i<5;i++ ) {
          Serial.println("#"+String(i)+" "+String(joints[i]->get_angle()));
        }
        baseRotate.dumpPos();
        Serial.println("===END===");
        goto success;
      }
    }
    Serial.println("error!");
    Serial.println(buf);
    return;

success:
    Serial.println(buf + String("ok!"));
}

void loop() {
  static char buf[80];

  // btnOnOff.tick();

  if(SerialHelper::readline(Serial.read(), buf, 80) > 0) {
    control( buf );
  }

  baseRotate.loop();

  ps2_controller.loop();
  delay(10);
}
