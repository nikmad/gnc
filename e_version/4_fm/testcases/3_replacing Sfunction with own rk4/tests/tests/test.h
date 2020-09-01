#include <math.h>

#ifndef MYLIB_CONSTANTS_H

struct vt{
	float Va0;
	float gamma;
};

struct states{
	float pn;
	float pe;
	float pd;
	float u;
	float v;
	float w;
	float phi;
	float theta;
	float psi;
	float p;
	float q;
	float r;
};

const struct vt vtol = 
{
	35.0,
	15
};

#define MYLIB_CONSTANTS_H
#endif
