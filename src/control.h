
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
      for(int i=0;i<20;i++)
        meanFilter.push( analogRead(pinMeter) );
      curPos = meanFilter.mean();

      int mid = ( minPos + maxPos ) /2;
      setTargetPos( mid );        

      dumpPos();
    }

    void setTargetPos(long newPos) {
      targetPos = max(minPos, min(newPos,maxPos));
      targetPosValid = true;
    }

    void dumpPos() {
      static elapsedMillis interval=0;
      if( interval > 1000 ) {
        Serial.println("stepper pos:" + String(curPos) + "," + String(theStepper.currentPosition()));
        interval = 0;
      }
    }

    void loop() {
      meanFilter.push( analogRead(pinMeter) );
      curPos = meanFilter.mean();

      if( targetPosValid && sinceLastCheck > CHECK_INVERVAL_S ) {
        theStepper.setSpeed(SPEED);
        int target = (targetPos - curPos) * 1;
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
};

class PWMServo {
  protected:
    Adafruit_PWMServoDriver& pwm;
    int id;
    int vmin;
    int vmax;
    double cur_val;

    bool angle_reverse;
  protected:
    void set(int pulselen) {
      pwm.setPWM( id, 0,pulselen );
    }
    
  public:
    PWMServo(Adafruit_PWMServoDriver& _pwm, int no, int min, int max, bool reverse = false):
      pwm(_pwm), id(no),vmin(min),vmax(max),angle_reverse(reverse)  {
    }

    void stop() {
      pwm.setPWM(id,0,0);
    }

    void moveTo(double angle) {
      cur_val = angle;
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
