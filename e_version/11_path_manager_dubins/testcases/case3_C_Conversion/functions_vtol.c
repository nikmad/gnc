
#include "math_util.h"
#include <math.h>
#define WAYPOINT_SIZE 100

//__________________________________________________________

float absolut_(float x)
{
	float a;
	if(x<0.0) {a = -x;}
	else {a = x;}
	return a;
}

//__________________________________________________________

float modulo_(float x, float y)
{
	float a;
	if(y == 0.0) {a = x;}
	else{a = x - (y*floor(x/y));}
	return a;
}

//__________________________________________________________

float sign_(float x)
{
	int a;
	if(x<0) a = -1;
	else a = 1;
	return a;
}

//__________________________________________________________

void cross_product(float Vec1[],float Vec2[], float Vec_prod[][3])
{
	Vec_prod[0][0]=Vec1[1]*Vec2[2]-Vec2[1]*Vec1[2] ;
	Vec_prod[0][1]=-(Vec1[0]*Vec2[2]-Vec2[0]*Vec1[2]);
	Vec_prod[0][2]=Vec1[0]*Vec2[1]-Vec2[0]*Vec1[1];
}

//__________________________________________________________

float dot_product(float Vec1[],float Vec2[])
{
	int i=0;
	float dot_prod_op=0.0;
	dot_prod_op=Vec1[i]*Vec2[i]+Vec1[i+1]*Vec2[i+1]+Vec1[i+2]*Vec2[i+2];
	return dot_prod_op;
}

//__________________________________________________________
	
int is_empty_matrix(float abb[5][WAYPOINT_SIZE])
{
	int i=0,j=0;
	float add=0.0,sub=0.0;
	
	for(i=0;i<5;i++)
	{
		for(j=0;j<(5*WAYPOINT_SIZE)+1;j++)
		{
			add=add+abb[i][j];
			sub=sub-abb[i][j];
			if((add!=0.0)&&(sub!=0.0))
			{
				return 0;
			}
			else
			{
				return 1;
			}
		}
	}
}

//__________________________________________________________

float norm_n(float a, float b, float c)
{
	float N;
	N = sqrtf(a*a+b*b+c*c);
	return N;
}

//__________________________________________________________