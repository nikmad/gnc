#ifndef MATH_UTIL_H
#define MATH_UTIL_H
#include<stdlib.h>
#include "math_util.h"

#define PI 3.1415927

void array_initd(float * arr,int num);

float sec(float );
int sign(float);

extern void MatrixMultiply(float *,unsigned int ,unsigned int ,float *,unsigned int ,unsigned int ,float *); 
extern void transposed3x3(float[][3]);
extern void transposedmxnAToB(int m, int n, float[][n],float[][m]);
extern void transposedlxmxnAToB(int l, int m, int n, float[l][m][n], float[n][m][l]);

extern void MatrixInverse3x3(float [][3],float [][3]);
extern void MatrixInverse4x4(float [4][4],float [4][4]);
extern void MatrixInverse6x6(float [][6],float [][6]);

#endif
