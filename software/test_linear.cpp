/*
*   Este archivo permite probar el algoritmo de RF
*   como metodo para determinar el angulo formando en la matriz de sensores
*/


#include "WProgram.h"
#include "adc_driver.h"

int sensor[49]{	
	0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0
};




double m,b,R_sqr;

int main(){

	Serial.begin(9600);


	while(1){
	
		if(Serial.available()){

			if(Serial.read()=='y'){

					   get_line_parameters(sensor,&m, &b, &R_sqr);

						int m_int=int(m*10000);
						int b_int=int(b*10000);
						int R_sqr_int=int(R_sqr*10000);
			
						Serial.printf("\nm=%d\nb=%d\nMultiplicado por 10000:\nm=%d\nb=%d\nR_sqr=%d\nDone\n",int(m),int(b),m_int,b_int,R_sqr_int);
				}
		}
	}

}
