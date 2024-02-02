// 1. Set up Motor Controllers and Define pin names for ease of use latter. 
//          * note the driver pins must be connected to Mega pins, you can look these up or there are ~ next to those pins. 
//            In this case its pins 2-13. 1 and 2 are also PWM pins but they are used for communication as well 
//          ** Write this in motors.ino for clarity later on 
#include "Vars.h"
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200); // by default people use 9600. In modern systems that is too slow. 115200 is typically a good starting number to prevent the serial from overflowing
  zeroEncoders();
  motorInit();
  control_pos(1); 
}

void loop() {
  // put your main code here, to run repeatedly:

  //start the loop by updating our encoder counts
      updateEncPos();
      //Select the lines of code below and press ctrl+/ this will uncomment/recomment the code. This is helpful for quick debugging. I reccomend after wiring everything
      //make sure when you turn each wheel by hand the correct encoder count goes up and down. This will ensure you've not crossed wires anywhere on accident. 
      // Serial.print(fl_enc_count);  
      // Serial.print(" ");
      // Serial.print(fr_enc_count);
      // Serial.print(" ");
      // Serial.print(bl_enc_count);
      // Serial.print(" ");
      // Serial.print(br_enc_count);
      // Serial.print(" ");

      // Serial.print(fl_vel());
      // Serial.print(" ");
      // Serial.print(fr_vel());
      // Serial.print(" ");
      // Serial.print(bl_vel());
      // Serial.print(" ");
      // Serial.print(br_vel());
      // Serial.print(" ");

      Serial.print(fl_pos());
      Serial.print(" ");
      Serial.print(fr_pos());
      Serial.print(" ");
      Serial.print(bl_pos());
      Serial.print(" ");
      Serial.print(br_pos());
      Serial.print(" ");

      Serial.println();  
  
  //Set the PWM Value
  //motor_set(-255); test motors to make sure they work
  //control_vel(.3, 0.0); make sure velocity controller is good
  if (1 - fl_pos() < .05  ){control_pos(0);}
  
  control_loop();
}

