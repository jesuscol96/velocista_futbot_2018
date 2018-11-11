/*
*   FUTBOT 2018
*   Project: Usain Bot, Velocista 
*
*/

 #include "WProgram.h"
 #include "mylib.h"
 #include "adc_driver.h"

 int main(){

 	//Timing / ts_d must be at least 10 times larger than ts_c
 	unsigned int ts_c=1000; //sample time in micros for pid controllers
 	unsigned int ts_d=10000; //sample time in micros for data acquisition

 	//Encoders
 	int pin1_enc1=1;
 	int pin2_enc1=2;
 	int pin1_enc2=3;
 	int pin2_enc2=4;
 	freq_encoder enc1(pin1_enc1,pin2_enc1);
 	freq_encoder enc2(pin1_enc2,pin2_enc2);

 	//PID controllers

 	//Velocity control
 	double vel_max=3; //Max speed in m/s
 	double Kp_v=1;
 	double Ki_v=1;
 	double Kd_v=1;
 	myPID pid_v(Kp_v,Ki_v,Kd_v,ts_c);

 	//Dir control
 	double Kp_a=1;
 	double Ki_a=1;
 	double Kd_a=1;
 	myPID pid_a(Kp_a,Ki_a,Kd_a,ts_c);

 	//PWM pins for motor control
 	int pin1_m1 = 0;
 	int pin2_m1 = 0;
 	int pin1_m2 = 0;
 	int pin2_m2 = 0;

 	//General control object
 	control_motors control_bot(&pid_v,&pid_a,&enc1,&enc2,pin1_m1, pin2_m1,pin1_m2,pin2_m2);

 	//SPI communication with sensor matrix. By default pinMOSI=11, pinMISO=12, pinSCK=13
 	int pin_EOC = 0;  
	int pin_SS1 = 0;  
	int pin_SS2 = 0;  
	int pin_SS3 = 0;  
	int data[49]={0}; //Array of data initialization
	matrix sensors(pin_SS1,pin_SS2,pin_SS3,pin_EOC,data);

	//Data analyzer object
	data_analyzer analyzer(data);

	//Some constant updating variables for loop
	bool turn_or_angle=ANGLE_CONTROL; //dir or angle
	double dir_value=0; //w or angle
	double error=0; //aprox error
	double mean=0; //Array's mean value
	int dir=0;  //Suggested turning direction
	double vel=0; // Vehicle's speed
	

	//Fixed variables for loop
	double max_error=10;    //Values obtained by algoritm testing on Octave
	double k_vel=2.5; //Adjust constant for speed
	double min_error=0.9;
	double min_mean=15;
	double turning_w=1.5;

	//Introduce delay
	int start_delay=10;  //seg delay before start
	delay(start_delay*1000); 

	//Init time
	unsigned int time_c=micros();
	unsigned int time_d=micros();
	

 	while(1){

 		//Data acquisition
 		if(ts_d>micros()-time_d){
 		
	 		//Update data
	 		sensors.eval_matrix();
	
	 		//Analyze received data
	 		error=analyzer.get_error();
	 		mean=analyzer.get_mean();
	 		dir=analyzer.get_dir();

	 		//Decide what to do
	 		if((error<min_error && mean<min_mean) || error>max_error ){  //Blank way or intersection

	 			vel=0; //Stop

	 			 //Use turning control 	
	 			turn_or_angle=TURNING_CONTROL;

	 			if(dir)
	 				dir_value=turning_w;
	 			else
	 				dir_value=-1*turning_w;
	 			
	 		}	 	
	 		else{	//Standard way

	 			//Determine vehicle's speed
	 			vel=vel_max*exp(-1*error/k_vel);

	 			 //Use angle control 	
	 			turn_or_angle=ANGLE_CONTROL;
	 			dir_value=analyzer.get_angle();			
	 		}	 		

	 		//Update time
	 		time_d=micros();

	 	}

	 	//Control action
	 	if (ts_c>micros()-time_c){
	 		
	 		//Take action
	 		control_bot.set(vel,dir_value,turn_or_angle);

	 		//Update time
	 		time_c=micros();
 		}

 	}

 	return 0;
 }
