// This is going to be were we write functions that control motor pins

// Steps to reach goals
// 1. Set up Motor Controllers and Define pin names for ease of use latter. 
//          * note the driver pins must be connected to Mega pins, you can look these up or there are ~ next to those pins. 
//            In this case its pins 2-13. 1 and 2 are also PWM pins but they are used for communication as well 
//Our motor driverss have 6 non power pins 
        //ENA - PWM signal 
        //IN1 - direction set 
        //IN2 - direction set (these 2 pins need to be set as low and high for CW motion, high low for CCW motion and High high or low low for braking, )
        //IN3 - same as in1 but for the second driver
        //IN4 - same as in2 but for the second driver
        //ENB - same as ENA but for the second driver
        //There are some random pins at the bottom of the board as well for measuring current and voltage. This is less common on other cheap motor drivers.

//Take a look at the motor. We have 6 pins, if you look close at the green you can see the pin names
    //M+ - this will be for the PWM+ signal from the motor driver
    //M- - this will be for the PWM- signal from the motor driver
    //GND - ground
    //Vcc - power
    //O-A - encoder A
    //O-B - encoder B

#define fl_PWM 9 //front left PWM pin positive (these plug into ENA/ENB)
#define fr_PWM 10
#define bl_PWM 11 
#define br_PWM 12


#define fl_IN1 22 //front left IN1 pin 
#define fl_IN2 23 //front left IN2 pin 
#define fr_IN1 24
#define fr_IN2 25

#define bl_IN1 26
#define bl_IN2 27

#define br_IN1 28
#define br_IN2 29

//intialize all motor pins
void motorInit(){
  pinMode(fl_PWM, OUTPUT);
  pinMode(fr_PWM, OUTPUT);
  pinMode(bl_PWM, OUTPUT);
  pinMode(br_PWM, OUTPUT);
  
  pinMode(fl_IN1, OUTPUT);
  pinMode(fr_IN1, OUTPUT);
  pinMode(bl_IN1, OUTPUT);
  pinMode(br_IN1, OUTPUT);

  pinMode(fl_IN2, OUTPUT);
  pinMode(fr_IN2, OUTPUT);
  pinMode(bl_IN2, OUTPUT);
  pinMode(br_IN2, OUTPUT);
}


//function for setting speed (PWM) and direction (+/-) of all the motors at once
void motor_set(int PWM){
  //To be good programmers we have made other functions to indvidually set the motor speeds.
  //This will be important later on when developing the controller.
  fl_motor_set(PWM);
  fr_motor_set(PWM);
  bl_motor_set(PWM);
  br_motor_set(PWM);
}

void fl_motor_set(int PWM){
  //1. set direction using a simple comparison
  bool dir = PWM > 0; //this reads true if PWM is a postive int and false otherwise so false will be reverse
  //2. Write the pins to the correct Values 
  digitalWrite(fl_IN1, dir); //if PWM is greater than 0 set HIGH
  digitalWrite(fl_IN2, !dir); //if PWM is greater than 0 set LOW, if you recall a HIGH LOW signal is CCW movement of the wheel. For the Right side we will want CC movement to go forward
  analogWrite(fl_PWM, abs(PWM));
}
void fr_motor_set(int PWM){
  bool dir = PWM > 0;
  digitalWrite(fr_IN1, !dir); 
  digitalWrite(fr_IN2, dir); 
  analogWrite(fr_PWM, abs(PWM));
}
void bl_motor_set(int PWM){
  bool dir = PWM > 0;
  digitalWrite(bl_IN1, !dir); 
  digitalWrite(bl_IN2, dir); 
  analogWrite(bl_PWM, abs(PWM));
}
void br_motor_set(int PWM){
  bool dir = PWM > 0;
  digitalWrite(br_IN1, dir); 
  digitalWrite(br_IN2, !dir); 
  analogWrite(br_PWM, abs(PWM));
}

//Lets make a way to calculate each wheels linear velocity first

//Note with our robot the wheels max speed is roughly .4 m/s
float fl_vel(){
    unsigned long myTime = micros(); //Velocity is a function of time so lets get the current time
    static float prevTime = 0; // and the previous time. Static just means when we leave the function the variable will retain its last value
    static int32_t prevCount = 0;
    updateEncPos(); // make sure we get the latestet encoder data
    float velocity = (1.0/ENCODER_COUNTS_PER_REV)*(PI*WHEEL_DIAMETER)*((fl_enc_count-prevCount)/((myTime-prevTime) * .000001));  
    prevCount = fl_enc_count; //update prev Count
    prevTime = myTime; // update prev Time
  return velocity;
}
float fr_vel(){
    unsigned long myTime = micros();
    static float prevTime = 0;
    static int32_t prevCount = 0;
    updateEncPos();
    float velocity = (1.0/ENCODER_COUNTS_PER_REV)*(PI*WHEEL_DIAMETER)*((fr_enc_count-prevCount)/((myTime-prevTime) * .000001));  
    prevCount = fr_enc_count; 
    prevTime = myTime; 
  return velocity;
}
float bl_vel(){
    unsigned long myTime = micros(); 
    static float prevTime = 0;
    static int32_t prevCount = 0;
    updateEncPos();
    float velocity = (1.0/ENCODER_COUNTS_PER_REV)*(PI*WHEEL_DIAMETER)*((bl_enc_count-prevCount)/((myTime-prevTime) * .000001));  
    prevCount = bl_enc_count; 
    prevTime = myTime;
  return velocity;
}
float br_vel(){
    unsigned long myTime = micros(); 
    static float prevTime = 0; 
    static int32_t prevCount = 0;
    updateEncPos(); 
    float velocity = (1.0/ENCODER_COUNTS_PER_REV)*(PI*WHEEL_DIAMETER)*((br_enc_count-prevCount)/((myTime-prevTime) * .000001));  
    prevCount = br_enc_count; 
    prevTime = myTime; 
  return velocity;
}

//Now we need a we need an actual way to set the velocities. I am going to implement that over in the controller.

// We also need to know the location of the wheels 
float fl_pos(){
  updateEncPos();
  static float prev_pos_fl = 0;
  static float pos = 0;
  pos += ((1.0/ENCODER_COUNTS_PER_REV)*(fl_enc_count-prev_pos_fl)*PI*WHEEL_DIAMETER);
  prev_pos_fl = fl_enc_count;
  return pos;
  }

float fr_pos(){
  updateEncPos();
  static float prev_pos_fr = 0;
  static float pos = 0;
  pos += (1.0/ENCODER_COUNTS_PER_REV)*(fr_enc_count-prev_pos_fr)*PI*WHEEL_DIAMETER;
  prev_pos_fr = fr_enc_count;
  return pos;
  }

float bl_pos(){
  updateEncPos();
  static float prev_pos_bl = 0;
  static float pos = 0;
  pos += (1.0/ENCODER_COUNTS_PER_REV)*(bl_enc_count-prev_pos_bl)*PI*WHEEL_DIAMETER;
  prev_pos_bl = bl_enc_count;
  return pos;
  }

float br_pos(){
  updateEncPos();
  static float prev_pos_br = 0;
  static float pos = 0;
  pos += (1.0/ENCODER_COUNTS_PER_REV)*(br_enc_count-prev_pos_br)*PI*WHEEL_DIAMETER;
  prev_pos_br = br_enc_count;
  return pos;
  }