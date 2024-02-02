//This one will look real scary, but its fairly simple. We are going to make a small PID loop that will adjust the motor PWM speed to hit a target velocity or postion
// If you need a PID refresher
//          In PID is the sum of three different types of error multiplied each by magic numbers called gains. Each part of the PID is good for a specfic thing.
//                            * P - this is the difference between our goal and actual values. If we want to be going .5 m/s and are currently going .2 m/s the value of P is .3 m/s. Its a good place to start, 
//                                  maybe by multipying by a K value of 100 we could get a decent PWM of 30. But what if we want to go .5 m/s and are at .48 m/s then our PWM value might be to small to actually move the robot
//                                  and so we add
//                            * I - this value stands for integral and it is keeping track of the total error up to this point. Lets go use our previous numbers, at time = 1 we want .5 m/s and are at .3 m/s the I term is .2
//                                  then we come to check our speed again at time = 2 and we are now at .48 m/s that means our integral term would be .22, our last error plus current.
//                                  The reason that this is good, is now if that .02 extra isn't enough to increase the speed of the robot then at time = 3 if we were still going .48 m/s our I term would become .24 and each
//                                  time we come back and haven't reached the goal I will grow and grow until we are sending our max value. The issue with these 2 is that if the P and I term are not well balanced what will happen
//                                  is we ask for .5 and then the robot will fly past .5 m/s and goto .6 m/s then  overcorrect itself and go back down to .42 m/s and back and forth until it (hopefully) reaches an equilibrim
//                            * D - thats why we use the D term, or derivative. It is trying to guess how fast we are approaching the goal and if we are approaching it very quickly it will help prevent over shoot. The way it does 
//                                  is it takes the current error (.5 m/s and we are at .48 m/s) and subtracts it from the previous error (.2 m/s for our previous examples) which would give -.18 m/s. 
//                                 
//                         Taking a look at 3 times we can see how values change to balance one another
//                                  Time       Target Vel         Actual Vel     P =  Tar-Act=     I = sum of errors*dt          D = (currErr - PrevErr)/dt       setPWM=Kp*P+Ki*I+Kd*D     Now test values for the K's until you system works
//                                  T = 0,   Vtarget= .5 m/s,   Vactual= 0 m/s,  P = .5 - 0  = .5, I = .5 =.5,                   D = .5      -    0    = .5,      setPWM=Kp*.5+Ki*.5+Kd*.5      If it starts too slow increase Kp
//                                  T = 1,   Vtarget= .5 m/s,   Vactual= .3 m/s, P = .5 - .3 = .2, I = .5 + .2 = .7,             D = .2      -   .5    = -.3,     setPWM=Kp*.2+Ki*.2+Kd*-.3     If it gets stuck not quite there increase Ki
//                                  T = 2,   Vtarget= .5 m/s,   Vactual= .6 m/s, P = .5 - .6 =  -.1, I = .5 + .2 + -.1 = .6 ,    D = -.1      -  -.3    = .2,     setPWM=Kp*-.1+Ki*-.1+Kd*.2    If it flies past the goal increase Kd
//                            * NOTE dt is change in time, for these examples the time change was always 1 so it filters out, but it out actual system we will call the control loop more frequently and so we will have a time change variable


const int Kp_vel = 3; //When tuning I turn off i and D and just start with p
const int Ki_vel = .35;
const int Kd_vel = .5;

const int Kp_pos = 3.3;
const int Ki_pos = 1;
const int Kd_pos = .5;

bool postional_mode = false; // This variable will say is we want to use the control loop to set a postion or velocity
uint32_t last_millis = 0; // we will use this below it needs to intialize to 0 though and keeps its previous value so we define it up here

void control_loop(){
  
  //Okay so lets start with a few variables to use
  float v_error = 0; //current error between target and goal for vel
  float p_error = 0; //current error between target and goal for pos
  static float fl_int = 0;  // I terms for postional control
  static float fr_int = 0;
  static float bl_int = 0;
  static float br_int = 0;

  static float fl_v_int = 0; //I terms for velocity control
  static float fr_v_int = 0;
  static float bl_v_int = 0;
  static float br_v_int = 0;

  static float fl_v_d = 0; // D term for velocity
  static float fr_v_d = 0; 
  static float bl_v_d = 0; 
  static float br_v_d = 0; 

  static float fl_p_d = 0; // D term for postoinal
  static float fr_p_d = 0; 
  static float bl_p_d = 0; 
  static float br_p_d = 0; 

  static float fl_last_error = 0;// second half of D term for postional
  static float fr_last_error = 0;
  static float bl_last_error = 0;
  static float br_last_error = 0;

  static float fl_v_last_error = 0; // second half of D term for Vel
  static float fr_v_last_error = 0;
  static float bl_v_last_error = 0;
  static float br_v_last_error = 0;
  
  float time_change = (millis() - last_millis)/1000.0f; //dt converted to seconds
  
    // skip down to the velocity controller first then back. They are the same with different K values
  if (postional_mode == true){
      p_error = fl_target_p - fl_pos();
      fl_int = max(-MAX_INTEGRAL, min(MAX_INTEGRAL, fl_int + p_error*time_change));
      fl_p_d = (p_error - fl_last_error)/time_change;
      fl_last_error = p_error;
      fl_target_v = Kp_pos*p_error+Ki_pos*fl_int+Kd_pos*fl_p_d;
  
      p_error = fr_target_p - fr_pos();
      fr_int = max(-MAX_INTEGRAL, min(MAX_INTEGRAL, fr_int + p_error*time_change));
      fr_p_d = (p_error - fr_last_error)/time_change;
      fr_last_error = p_error;
      fr_target_v = Kp_pos*p_error+Ki_pos*fr_int+Kd_pos*fr_p_d;

      p_error = bl_target_p - bl_pos();
      bl_int = max(-MAX_INTEGRAL, min(MAX_INTEGRAL, bl_int + p_error*time_change));
      bl_p_d = (p_error - bl_last_error)/time_change;
      bl_last_error = p_error;
      bl_target_v = Kp_pos*p_error+Ki_pos*bl_int+Kd_pos*bl_p_d;

      p_error = br_target_p - br_pos();
      br_int = max(-MAX_INTEGRAL, min(MAX_INTEGRAL, br_int + p_error*time_change));
      br_p_d = (p_error - br_last_error)/time_change;
      br_last_error = p_error;
      br_target_v = Kp_pos*p_error+Ki_pos*br_int+Kd_pos*br_p_d;
    }
  
 
  // this is the velocity controller
  v_error = fl_target_v - fl_vel(); //error
  fl_v_int = max(-MAX_V_INTEGRAL, min(MAX_V_INTEGRAL, fl_v_int+v_error*time_change)); //Calculate I term. The min and max function here are just limiting our integral to a max/min value of our choosing. 
  fl_v_d = (v_error - fl_v_last_error)/time_change; //Calculating D term
  fl_v_last_error = v_error;//updating Error
  fl_val = max(-MAX_VEL, min(MAX_VEL, Kp_vel*v_error + Kd_vel*fl_v_d+ Ki_vel*fl_v_int));//update the PWM value on the motor
  fl_motor_set(vel_to_pwm(fl_val));



  v_error = fr_target_v - fr_vel();
  fr_v_int = max(-MAX_V_INTEGRAL, min(MAX_V_INTEGRAL, fr_v_int+v_error*time_change));
  fr_v_d = (v_error - fr_v_last_error)/time_change; 
  fr_v_last_error = v_error;
  float fr_val = max(-MAX_VEL, min(MAX_VEL, Kp_vel*v_error + Kd_vel*fr_v_d+ Ki_vel*fr_v_int));
  fr_motor_set(vel_to_pwm(fr_val));

  v_error = bl_target_v - bl_vel(); //error
  bl_v_int = max(-MAX_V_INTEGRAL, min(MAX_V_INTEGRAL, bl_v_int+v_error*time_change)); //Calculate I term. The min and max function here are just limiting our integral to a max/min value of our choosing. 
  bl_v_d = (v_error - bl_v_last_error)/time_change; //Calculating D term
  bl_v_last_error = v_error;//updating Error
  float bl_val = max(-MAX_VEL, min(MAX_VEL, Kp_vel*v_error + Kd_vel*bl_v_d+ Ki_vel*bl_v_int));//update the PWM value on the motor
  bl_motor_set(vel_to_pwm(bl_val));


  v_error = br_target_v - br_vel();
  br_v_int = max(-MAX_V_INTEGRAL, min(MAX_V_INTEGRAL, br_v_int+v_error*time_change));
  br_v_d = (v_error - br_v_last_error)/time_change; 
  br_v_last_error = v_error;
  float br_val = max(-MAX_VEL, min(MAX_VEL, Kp_vel*v_error + Kd_vel*br_v_d+ Ki_vel*br_v_int));
  br_motor_set(vel_to_pwm(br_val));

  last_millis = millis(); //prev_time

  }//end control loop

// Here we set a linear and angular velocity for our robot
// this will be in m/s
  void control_vel(float v_linear, float v_angular){
    control_vel_fl(v_linear + (ROBOT_WIDTH/2.0f)*v_angular); 
    control_vel_fr(v_linear - (ROBOT_WIDTH/2.0f)*v_angular);
    control_vel_br(v_linear - (ROBOT_WIDTH/2.0f)*v_angular);
    control_vel_bl(v_linear + (ROBOT_WIDTH/2.0f)*v_angular);
  }

  //and a few setter functions
  void control_vel_fl(float v){
  fl_target_v = v;
  postional_mode = false;
  }

void control_vel_fr(float v){
  fr_target_v = v;
  postional_mode = false;
  }

void control_vel_bl(float v){
  bl_target_v = v;
  postional_mode = false;
  }

void control_vel_br(float v){
  br_target_v = v;
  postional_mode = false;
  }


void control_pos(float p){
  control_pos_fl(p);
  control_pos_fr(p);
  control_pos_br(p);
  control_pos_bl(p);
  }


void control_pos_fl(float p){
  fl_target_p = p;
  postional_mode = true;
  }


void control_pos_fr(float p){
  fr_target_p = p;
  postional_mode = true;
  }

void control_pos_bl(float p){
  bl_target_p = p;
  postional_mode = true;
  }

void control_pos_br(float p){
  br_target_p = p;
  postional_mode = true;
  }

void control_angle(float angle){
    float val = (ROBOT_WIDTH*angle/2.0f);
    control_pos_fl(fl_pos() + val);
    control_pos_fr(fr_pos() - val);
    control_pos_bl(bl_pos() + val);
    control_pos_br(br_pos() - val);

}

//This function will convert a desired velocity to a PWM. We know that at full power the max velocity is aproximently .4 m/s this means at half power or a PWM value of 127.5 we would expect to go .2
float vel_to_pwm(float vel){
  float pwm = (255/.4)*vel;
  return pwm;
}