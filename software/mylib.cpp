
	#include "mylib.h"

	//freq_encoder CLASS
	freq_encoder::freq_encoder(int pin1, int pin2):Encoder(pin1, pin2){

		this->write(0);
		count=0;
		time_track=micros();		
	}

	//Returns frequency in Hz
	float freq_encoder::eval_freq(){

		time=micros()-time_track;
		int read=this->read();		
		float freq= (float) (read-count);
		freq=(freq*2500.0)/( (float) time);	
		count=read;
		time_track=micros();

		return freq;	
	}
	//return W
	float freq_encoder::eval_afreq(){

		float ang_freq= this->eval_freq();
		ang_freq*=(2.0*pi);

		return ang_freq;
	}
	//Returns rot angle in rad.
	float freq_encoder::eval_ang(){

		float angle= (float) this->read();
		angle=(angle/400.0)*2.0*pi;

		return angle;
	}

	//End of freq_encoder CLASS


	//MyPID CLASS
	myPID::myPID(double Kp_, double Ki_, double Kd_,int ts_){

		Kp=Kp_;
		Ki=Ki_;
		Kd=Kd_;

		double ts= ((double) ts_)/1000;	//millis to secs
		//Values
		A=Kp+Ki*ts+(Kd/ts);		
		B=-Kp-(2*Kd/ts);
		C=Kd/ts;
	}	

	double myPID::eval_PID(double input_value){

		if(n<2){

			input[1]=input[0];
			input[0]=input_value;
			n++;
		}
		else{

			//Update output
			output+=A*input_value+B*input[0]+C*input[1];

			//Remember new values.
			input[1]=input[0];
			input[0]=input_value;

		}

		return output;
	}

	//End of myPID CLASS


	//control_motors CLASS

	control_motors::control_motors(myPID* vcomun_,myPID* vdif_,freq_encoder* enc_m1_, freq_encoder* enc_m2_,
				  				   int pin1_m1_, int pin2_m1_,int pin1_m2_,int pin2_m2_){

			vcomun =vcomun_;
			vdif =vdif_;
			enc_m1 = enc_m1_;
			enc_m2 = enc_m2_;
			pin1_m1 = pin1_m1_;  //Pos     Motor pos dir when current from pos to neg
			pin2_m1 = pin2_m1_;  //Neg 
			pin1_m2 = pin1_m2_;  //Pos 
			pin2_m2 = pin2_m2_;  //Neg 

			//Set pins as outputs
			pinMode(pin1_m1,OUTPUT);
			pinMode(pin1_m2,OUTPUT);
			pinMode(pin1_m2,OUTPUT);
			pinMode(pin2_m2,OUTPUT);

			//Set analog resolution
			analogWriteResolution(14);
			//Set PWM freq
			analogWriteFrequency(pin1_m1, max_pwm_freq);
			analogWriteFrequency(pin2_m1, max_pwm_freq);
			analogWriteFrequency(pin1_m2, max_pwm_freq);
			analogWriteFrequency(pin2_m2, max_pwm_freq);
			//
			analogWrite(pin1_m1,0);
			analogWrite(pin2_m1,0);
			analogWrite(pin1_m2,0);
			analogWrite(pin2_m2,0);

	}

	void control_motors::set(double sp_vel, double m_dir, bool control_type){

		//Measure real world
		double w_m1= (double) enc_m1->eval_afreq();
		double w_m2= (double) enc_m2->eval_afreq();

		//Mass's center vel
		 double cm_vel= 0.5*R* (w_m1+w_m2);

		 //Velocity controller implementation
		 double vel_error=sp_vel-cm_vel;
		 double c_signal1=vcomun->eval_PID(vel_error);

		 //Direction control:

		 double c_signal2;

		 if(control_type){ //Angle controller implementation
		 	
		 	c_signal2=vdif->eval_PID(-1*m_dir); //sp=0
		 }
		 else{ //Turning controller implementation

		 	double t_w=k_turning*(w_m1-w_m2);  //Clockwise dir is positive
		 	double dir_error=m_dir-t_w;//sp=m_dir
		 	c_signal2=vdif->eval_PID(dir_error);
		 }	


		 if(c_signal1>=Vc_sat) c_signal1=Vc_sat; //Common voltage saturation
		 if(c_signal2>=Vd_sat) c_signal2=Vd_sat; //Dif voltage saturation

		 //Voltage to each motor:
		 double vol_m1=c_signal1+c_signal2;
		 double vol_m2=c_signal1-c_signal2;	

		 //PWM equivalent

		 //M1 (Left motor)
		 if(vol_m1 >= 0){

		 	analogWrite(pin2_m1,0);
		 	int pwm =  (int)(res*vol_m1/Vmax);
		 	analogWrite(pin1_m1,pwm);
		 }
		 else{
		 	analogWrite(pin1_m1,0);
		 	int pwm = (int)(-1*res*vol_m1/Vmax);
		    analogWrite(pin2_m1,pwm);
		 }

		  //M2 (Right motor)
		 if(vol_m2 >= 0){

		 	analogWrite(pin2_m2,0);
		 	int pwm = (int)(res*vol_m2/Vmax);
		    analogWrite(pin1_m2,pwm);
		 }
		 else{
		 	analogWrite(pin1_m2,0);
		 	int pwm = (int)(-1*res*vol_m2/Vmax);
		    analogWrite(pin2_m2,pwm);
		 }	

	}

	//End of class control_motors

	// Class data analyzer

	data_analyzer::data_analyzer(int* sensor_){

		sensor=sensor_;
		
	}

	void data_analyzer::analyze(void){

		double X[size][2]={0}; //X matrix
		double D[49]={0};  //D vector
		double prob[49]={0};

		//Define		
		for(int i = 0; i < 7; i++)
			for(int j = 0; j < 7; j++){

				int k=7*i+j;
				prob[k]= ((double) *(sensor+k))/255.0;								
				X[k][0]=prob[k];
				X[k][1]=(i+1)*prob[k];
				D[k]=(j+1)*prob[k];				
			}	

		//Hard calculations
		double M[2][2]={0};

		//X'*X
		for(int i = 0; i < size; i++){			
			M[0][0]+=X[i][0]*X[i][0];
			M[0][1]+=X[i][0]*X[i][1];			
			M[1][1]+=X[i][1]*X[i][1];
		}
		    M[1][0]=M[0][1];		 

		//(X'*X)^-1		  
		 double det=M[0][0]*M[1][1]-M[1][0]*M[0][1];
		 double hold; //buffer
		 hold=M[0][0];
		 M[0][0]=M[1][1]/det;
		 M[0][1]=-1.0*M[0][1]/det;
		 M[1][0]=-1.0*M[1][0]/det;
		 M[1][1]=hold/det;
		 
		 //Reset slope, b
		 slope=0;
		 b=0;

		 //(X'*X)^-1 *X' * D / Get slope, b
		 for(int i = 0; i < size; i++){		 	
		 	b+=(M[0][0]*X[i][0]+M[0][1]*X[i][1])*D[i];
		 	slope+=(M[1][0]*X[i][0]+M[1][1]*X[i][1])*D[i];
		 }

		 //Error calculation
		 error=0;		
		 for(int i = 0; i < 7; i++){

		 	double y=(slope*(i+1)+b);

			for(int j = 0; j < 7; j++){	

				int k=7*i+j;
				hold=y-(j+1);
				error+=prob[k]*hold*hold;			
			}
		}	
			error*=0.5;

		//Angle
		angle=-1*atan(slope); 
	}

	double data_analyzer::get_slope(void){

		return slope;
	}

	double data_analyzer::get_angle(void){

		return angle;
	}

	double data_analyzer::get_b(void){

		return b;
	}

	double data_analyzer::get_error(void){

		return error;
	}	

	double data_analyzer::get_mean(void){

		int sum=0;

		for (int i = 0; i < size; i++)
			sum+=*(sensor+i);			
		
		return ((double) sum) / ((double) size);


	}

	double data_analyzer::get_half_mean(bool side){

		int ko, kf;
		int sum=0;

		if(side){
			ko=3; kf=7;
		}
		else{
			ko=0; kf=3;
		}

		for(int i = 0; i < 7; i++)
			for(int j = ko; j < kf; j++)			
				sum+=*(sensor+7*i+j);

		return ((double) sum) / ((double) 7*4);
	}

	double data_analyzer::sigmoid(double x){

		return 1.0/(1+exp(-1*x));
	}

	double data_analyzer::analyze_side(bool side){

		double mean=this->get_half_mean(side);
		mean*=Kadj;
		
		double *x, *y;  //Pointer to input and output data

		//Decide which side
		if(side){
			x=x_r; y=&y_past_r; 
		}
		else{
			x=x_l; y=&y_past_l; 
		}

		//Refresh		
		for(int i = 0; i < n-1; i++)
			*(x+n-1-i)=*(x+n-2-i);		
		*x=mean;		

		double output=0;

		for (int i = 0; i < n; i++)
			output+=(*(x+i))*wi[i];				

		output+=w_y*(*y);
		output=this->sigmoid(output);
		*y=output;

		return output;
	}


	int data_analyzer::get_dir(void){

		double dir_r=this->analyze_side(true);
		double dir_l=this->analyze_side(false);
		Serial.printf("Right: %d\n", (int) (dir_r*100));
		Serial.printf("Left: %d\n", (int) (dir_l*100));

		if(dir_r>dir_l)
			return 1;
		else
			return 0;		
	}

	//End of class data analyzer


	//scan string function

	void scan_s(char *buf){

	int size=0;
	int i=0;

	while(size==0){
		size=Serial.available();		
	}
	
	if(size!=0){
		for(i=0;i<size;i++)
			*(buf+i)=Serial.read();
	}
	*(buf+i)='\0';		
}
