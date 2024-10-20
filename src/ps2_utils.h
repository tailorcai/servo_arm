#pragma once

#include <PS2X_lib.h>  //for v1.6
#include <elapsedMillis.h>

class PS2Controller {
protected:
  PS2X ps2x; // create PS2 Controller Class
  static const bool pressures = false;
  static const bool rumble = false;
  static const int READ_INTERVAL = 200; //ms
  byte vibrate;
  elapsedMillis read_timer;
  std::function<int(int,int)> event_handler;
public:
  PS2Controller():vibrate(0) {}

  void set_event_handler( std::function<int(int,int)> func ) {
    event_handler = func;
  }

  void setup() {
    int error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);
  
    if(error == 0){
      Serial.print("Found Controller, configured successful ");
      Serial.print("pressures = ");
      if (pressures)
        Serial.println("true ");
      else
        Serial.println("false");
      Serial.print("rumble = ");
      if (rumble)
        Serial.println("true)");
      else
        Serial.println("false");
        Serial.println("Try out all the buttons, X will vibrate the controller, faster as you press harder;");
        Serial.println("holding L1 or R1 will print out the analog stick values.");
        Serial.println("Note: Go to www.billporter.info for updates and to report bugs.");

    }  
    else if(error == 1)
      Serial.println("No controller found, check wiring, see readme.txt to enable debug. visit www.billporter.info for troubleshooting tips");
    
    else if(error == 2)
      Serial.println("Controller found but not accepting commands. see readme.txt to enable debug. Visit www.billporter.info for troubleshooting tips");

    else if(error == 3)
      Serial.println("Controller refusing to enter Pressures mode, may not support it. ");
    
  //  Serial.print(ps2x.Analog(1), HEX);
    
    auto type = ps2x.readType(); 
    switch(type) {
      case 0:
        Serial.print("Unknown Controller type found ");
        break;
      case 1:
        Serial.print("DualShock Controller found ");
        break;
      case 2:
        Serial.print("GuitarHero Controller found ");
        break;
    case 3:
        Serial.print("Wireless Sony DualShock Controller found ");
        break;
    }
  }

  void loop() {
    // 检查时间间隔
    if( read_timer <= READ_INTERVAL) return;
    read_timer = 0;

    ps2x.read_gamepad(false, vibrate);

    for( int i=PSS_RX;i<=PSS_LY;i++) {
      byte v = ps2x.Analog(i);
      // Serial.println("ps2 " + String(v));
      if( v > 200 ) event_handler( i,  -1 );
      else if( v < 100 ) event_handler(i, 1);
    }

    static int btns[] = { PSB_START, PSB_SELECT, PSB_PAD_UP, PSB_PAD_RIGHT, PSB_PAD_LEFT, PSB_PAD_DOWN};
    for( int i=0;i<sizeof(btns)/sizeof(int);i++)
      if( ps2x.ButtonPressed(btns[i]))
          event_handler(btns[i], 1);
/*
    ps2x.read_gamepad(false, vibrate); //read controller and set large motor to spin at 'vibrate' speed
    
    if(ps2x.Button(PSB_START))         //will be TRUE as long as button is pressed
      Serial.println("Start is being held");
    if(ps2x.Button(PSB_SELECT))
      Serial.println("Select is being held");      

    if(ps2x.Button(PSB_PAD_UP)) {      //will be TRUE as long as button is pressed
      Serial.print("Up held this hard: ");
      Serial.println(ps2x.Analog(PSAB_PAD_UP), DEC);
    }
    if(ps2x.Button(PSB_PAD_RIGHT)){
      Serial.print("Right held this hard: ");
      Serial.println(ps2x.Analog(PSAB_PAD_RIGHT), DEC);
    }
    if(ps2x.Button(PSB_PAD_LEFT)){
      Serial.print("LEFT held this hard: ");
      Serial.println(ps2x.Analog(PSAB_PAD_LEFT), DEC);
    }
    if(ps2x.Button(PSB_PAD_DOWN)){
      Serial.print("DOWN held this hard: ");
      Serial.println(ps2x.Analog(PSAB_PAD_DOWN), DEC);
    }   

    vibrate = ps2x.Analog(PSAB_CROSS);  //this will set the large motor vibrate speed based on how hard you press the blue (X) button
    if (ps2x.NewButtonState()) {        //will be TRUE if any button changes state (on to off, or off to on)
      if(ps2x.Button(PSB_L3))
        Serial.println("L3 pressed");
      if(ps2x.Button(PSB_R3))
        Serial.println("R3 pressed");
      if(ps2x.Button(PSB_L2))
        Serial.println("L2 pressed");
      if(ps2x.Button(PSB_R2))
        Serial.println("R2 pressed");
      if(ps2x.Button(PSB_TRIANGLE))
        Serial.println("Triangle pressed");        
    }

    if(ps2x.ButtonPressed(PSB_CIRCLE))               //will be TRUE if button was JUST pressed
      Serial.println("Circle just pressed");
    if(ps2x.NewButtonState(PSB_CROSS))               //will be TRUE if button was JUST pressed OR released
      Serial.println("X just changed");
    if(ps2x.ButtonReleased(PSB_SQUARE))              //will be TRUE if button was JUST released
      Serial.println("Square just released");     

    if(ps2x.Button(PSB_L1) || ps2x.Button(PSB_R1)) { //print stick values if either is TRUE
      Serial.print("Stick Values:");
      Serial.print(ps2x.Analog(PSS_LY), DEC); //Left stick, Y axis. Other options: LX, RY, RX  
      Serial.print(",");
      Serial.print(ps2x.Analog(PSS_LX), DEC); 
      Serial.print(",");
      Serial.print(ps2x.Analog(PSS_RY), DEC); 
      Serial.print(",");
      Serial.println(ps2x.Analog(PSS_RX), DEC); 
    }  
    */

  }
};
   