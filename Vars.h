// Now I don't know if this is good practice. But I like to have a place to declare my global variables seperate from my indvidual sketches to prevent repeats and for clarity.

//technically you'd want a getter function for these variables so that they can only be updated by the encoders read function.
int32_t fl_enc_count = 0; 
int32_t fr_enc_count = 0; 
int32_t bl_enc_count = 0; 
int32_t br_enc_count = 0; 

float fl_target_v = 0;
float fr_target_v = 0;
float bl_target_v = 0;
float br_target_v = 0;

float fl_target_p = 0;
float fr_target_p = 0;
float bl_target_p = 0;
float br_target_p = 0;

#define WHEEL_DIAMETER .08 //80 mm
#define ENCODER_COUNTS_PER_REV 900 //this is a rough estimate. You can confirm it by rotating the wheel one full time and have it printing the enocder value
#define MAX_VEL .4
#define MAX_LINEAR_PWM 250
#define MAX_INTEGRAL 10000
#define MAX_V_INTEGRAL 10000
#define ROBOT_WIDTH .21 //this distance is wheel center to wheel center

//temps
float fl_val = 0;