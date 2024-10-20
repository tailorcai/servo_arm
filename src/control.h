
#include <AccelStepper.h>
#include <elapsedMillis.h>
#include <Adafruit_PWMServoDriver.h>

#include "arm_utils.h"
// TODO: using <ramp.h >

#define CHECK_INVERVAL_S 2
#define SPEED 200


class BaseRotate {
  protected:
    long curPos;    // current position
    long targetPos;
    bool targetPosValid;
    int pinMeter;
    long minPos, maxPos;
    elapsedMillis sinceLastCheck;
    AccelStepper&  theStepper;
    MeanFilter meanFilter;
  public:
    BaseRotate(AccelStepper& stepper, int analogPin, long vMin, long vMax):
      theStepper( stepper), pinMeter(analogPin),
      minPos(vMin), maxPos(vMax),
      meanFilter(10),
      targetPosValid(false)
    {
      theStepper.setMaxSpeed(100);
    }

    void reset() {
      // 获取当前位置，通过均值过滤器，提升精度
      for(int i=0;i<20;i++)
        meanFilter.push( analogRead(pinMeter) );
      curPos = meanFilter.mean();

      // 0 degree 
      int mid = ( minPos + maxPos ) /2;

      // moving
      setTargetPos( mid );        

      dumpPos();
    }

    void setTargetPos(long newPos) {
      targetPos = max(minPos, min(newPos,maxPos));
      targetPosValid = true;
    }

    void dumpPos() {
      Serial.println("stepper pos:" + String(curPos) + "," + String(theStepper.currentPosition()));
    }

    void loop() {
      // update position
      meanFilter.push( analogRead(pinMeter) );
      curPos = meanFilter.mean();

      // a simple interpolation implement
      // it's better to use RAMP.h for enhanced movement control
      if( targetPosValid && sinceLastCheck > CHECK_INVERVAL_S ) {
        theStepper.setSpeed(SPEED);
        int target = (targetPos - curPos) * 1;  // in case the stepper's dir opposite, change to -1
        if( abs(target)>=2) { 
          theStepper.move(target);   // 目前是反向的
          // Serial.println("Move:" + String( target));
          theStepper.runSpeedToPosition();
        }

        sinceLastCheck = 0;  // reset timer

        if( target == 0 ) {
          targetPosValid = false;
          theStepper.stop();
        }
      }

      // dumpPos();

    }

    bool isRunning() {
      return targetPosValid;
    }
};

class PWMServo {
  protected:
    Adafruit_PWMServoDriver& pwm;
    int id;
    int vmin, vmax; // physical limitation for each servo
    double cur_val;
    bool angle_reverse; // in case dir is not consistent
  protected:
    void set(int pulselen) {
      pwm.setPWM( id, 0,pulselen );
    }
    
  public:
    PWMServo(Adafruit_PWMServoDriver& _pwm, int no, int min, int max, bool reverse = false):
      pwm(_pwm), id(no),vmin(min),vmax(max),angle_reverse(reverse)  {
    }

    void stop() {
      pwm.setPWM(id,0,0); // set 0 will cause the servo lose power at all, for debug usage
    }


    void moveTo(double angle) {
      cur_val = angle;
      // TODO: arm calibration need, not just 0-180...
      double value = angle_reverse? map(angle, 180, 0, vmin, vmax):
        map(angle, 0, 180, vmin, vmax);
      set(round(value));
    }
};

class Joint {
  protected:
    PWMServo* servo;
    PWMServo* servo2;
    double angle;
  public:
    Joint( PWMServo* _servo, PWMServo* _servo2 = NULL): servo(_servo), servo2(_servo2) {
    }
    
    double get_angle() { return angle; }
    void move_to(double angle) {
      this->angle = angle;
      if( servo ) servo->moveTo(angle);
      if( servo2 ) servo2->moveTo(angle);
    }
    void stop() {}
};
