/*
IIR2 coefficient compuation with values for graphing
https://www.earlevel.com/main/2012/11/26/biquad-c-source-code/

// Line for compiling && executing--
gcc iir2.c -o iir2 -lm -Wall && ./iir2
*/
#include <math.h>
#include <stdio.h>

int main(void)
{
/* Specifications. */
double Fc = 0.05;
double Q = .707;

    double norm,a0,a1,a2,b1,b2;
    double K = tan(M_PI * Fc);

/* Compute coefficients. */
    norm = 1 / (1 + K / Q + K * K);
    a0 = 1; //K * K * norm;
    a1 = 2 * a0;
    a2 = a0;
    b1 = 2 * (K * K - 1) * norm;
    b2 = (1 - K / Q + K * K) * norm;

printf("Fc  %12.8f\n",Fc);
printf("Q   %12.8f\n\n",Q);

double KK = (K * K * norm);
printf("KK  %12.8f  %12.8f\n\n",K * K * norm, 1/(K * K * norm) );

printf("K   %12.8f\n",K);
printf("a0  %12.8f\n",a0);
printf("a1  %12.8f\n",a1);
printf("a2  %12.8f\n",a2);
printf("b1  %12.8f\n",b1);
printf("b2  %12.8f\n",b2);

/* Generate a table for graphing. */
x
double out;
double in = 4095;
double z1 = 0;
double z2 = 0;
int i;
	for (i = 0; i < 30; i++)
	{
// By making a0 = a2 = 1, and using KK for gain scaling
// there is a net reduction of one multiply.
		 out = in * a0 + z1;
   	 z1  = in * a1 + z2 - b1 * out;
   	 z2  = in * a2 - b2 * out;
		printf("%2i  %12.8f %14.8f %14.8f %12.8f\n",i,in,z1,z2,out*KK);
	}
return 0;
}

