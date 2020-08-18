#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "test.h"
#include "math_util.h"

struct vt vtol_dynamics();
float absolut_(float);
float modulo_(float, float);
float sign_(float);

int main()
{
	struct vt lako;
	lako = vtol_dynamics();
	printf("Main1: values of vao and gamma = %f and %f\n", lako.Va0, lako.gamma);
	return 0;
}

struct vt vtol_dynamics()
{
	struct vt kola;
	//float y[12] = {0,0,0,0,0,0,0,0,0,0,0,0};

	struct states states_in = {0,0,0,0,0,0,0,0,0,0,0,0};
	
	printf("Fn1: values of vao and gamma = %f and %f\n", vtol.Va0, vtol.gamma);
	kola.Va0 = vtol.Va0;
	kola.gamma = vtol.gamma;
	kola.gamma = 20.0*(cosf(30));
	printf("Fn2: values of vao and gamma = %f and %f\n", kola.Va0, kola.gamma);

	float _x, _y, _a;
	int _b;
	//absolute
	printf("Enter numbers _x, _y = ");
	scanf("%f %f", &_x, &_y);
	_a = modulo_(_x, _y);
	//_x = absolut_(_x);
	//_x = floor(_x);

	printf("\nDivision of %f/%f is = %f\n", _x, _y, _x/_y);
	printf("Modulus of %f/%f is = %f\n", _x, _y, _a);
	
	_b = sign_(_x);

	printf("Sign of %f is = %d\n", _x, _b);
	printf("Floor of %f is = %f\n", _x, floor(_x));

   //float i=5.1, j=5.9, k=-5.4, l=-6.9;
   //printf("floor of  %f is  %f\n", i, floor(i));
   //printf("floor of  %f is  %f\n", j, floor(j));
   //printf("floor of  %f is  %f\n", k, floor(k));
   //printf("floor of  %f is  %f\n", l, floor(l));

	//*y = *stf;
	//float y[] = {states_in.pn,states_in.pe,states_in.pd,states_in.u,states_in.v,states_in.w,states_in.phi,states_in.theta,states_in.psi,states_in.p,states_in.q,states_in.r};
	//float y[] = states_in;

	float R1[3][3];
	R1[0][0] = 0.0;
	R1[0][1] = 1.4;
	R1[0][2] = 2.4;
	
	R1[1][0] = 3.4;
	R1[1][1] = 2.4;
	R1[1][2] = 3.4;
	
	R1[2][0] = 2.4;
	R1[2][1] = 3.4;
	R1[2][2] = 2.4;

	float R2[3][3];
	R2[0][0] = 0.0;
	R2[0][1] = 1.4;
	R2[0][2] = 2.4;
	
	R2[1][0] = 3.4;
	R2[1][1] = 2.4;
	R2[1][2] = 3.4;
	
	R2[2][0] = 2.4;
	R2[2][1] = 3.4;
	R2[2][2] = 2.4;

	float R3[3][3];

	array_initd((float*)R1,9);
	array_initd((float*)R2,9);
	array_initd((float*)R3,9);

	MatrixMultiply(R1,3,3,R2,3,3,R3);

	printf("Matrix check %f\n", R3[2][1]);

	return kola;
}

float absolut_(float x)
{
	float a;
	if(x<0.0) {a = -x;}
	else {a = x;}
	printf("Absolute value of a is = %f\n", a);
	return a;
}

float modulo_(float x, float y)
{
	float a;
	if(y == 0.0) {a = x;}
	else{a = x - (y*floor(x/y));}
	return a;
}

float sign_(float x)
{
	int a;
	if(x<0) a = -1;
	else a = 1;
	return a;
}
