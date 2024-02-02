//Here we will use the Encoder Library with Arduino, because while we could recreate the wheel. We could never do it quite like the proffesionals. 
//Recall from the motors the pins below
    //O-A
    //O-B
// magnets in the encoder spin past the 2 pins inducing a current, in one direction pin 1 will go high, then both, then it will go low while pin 2 is still high and then both are low
// while in the other pin 2 will go high first. Below is a visual.
//                           _______         _______       
//               Pin1 ______|       |_______|       |______ Pin1
// negative <---         _______         _______         __      --> positive
//               Pin2 __|       |_______|       |_______|   Pin2

		//	new	new	old	old
		//	pin2	pin1	pin2	pin1	Result
		//	----	----	----	----	------
		//	  0	    0	   0	    0	    no movement
		//	  0	    0	   0	    1	    +1
		//	  0	    0	   1	    0	    -1
		//	  0	    0	   1	    1	    +2  (assume pin1 edges only)
		//	  0	    1	   0	    0	    -1
		//	  0	    1	   0	    1	    no movement
		//	  0	    1	   1	    0	    -2  (assume pin1 edges only)
		//	  0	    1	   1	    1	    +1
		//	  1	    0	   0	    0	    +1
		//	  1	    0	   0	    1	    -2  (assume pin1 edges only)
		//	  1	    0	   1	    0	    no movement
		//	  1	    0	   1	    1	    -1
		//	  1	    1	   0	    0	    +2  (assume pin1 edges only)
		//	  1	    1	   0	    1	    -1
		//	  1	    1	   1	    0	    +1
		//	  1	    1	   1	    1	    no movement
//Credit: Paul Stoffregen

//Some other notes, its important that at least all the pin 1s are on intterupt pins. Optimally both pin 1 and pin 2s are but the arduino mega does not have enough to do both and for our speed
//and accuracy purposes just the first is enough.
#include <Encoder.h>

//declare the 8 encoding pins
#define fl_a 2 //the A pins should all be on interupts 
#define fl_b 53
#define fr_a 3
#define fr_b 51
#define bl_a 18
#define bl_b 49
#define br_a 19
#define br_b 47

// Now we look at the libraries documentation to determine we need to make the following objects
Encoder fl_enc(fl_a, fl_b);
Encoder fr_enc(fr_b, fr_a); // The left wheels and right wheels are not both spinning CC from the perspective of the encoders. So when moving forward one will be counting postive ticks and the other negative. 
Encoder bl_enc(bl_a, bl_b); // We can manually fix this in the code by swamping the a and b pins
Encoder br_enc(br_b, br_a);

// okay now for an Zeroing function. This will be used to make the motors 0. It will be used later
void zeroEncoders(){
  fl_enc.write(0);
  fr_enc.write(0);
  bl_enc.write(0);
  br_enc.write(0);
  }

void updateEncPos(){
  fl_enc_count = fl_enc.read();
  fr_enc_count = fr_enc.read();
  bl_enc_count = bl_enc.read();
  br_enc_count = br_enc.read();
}