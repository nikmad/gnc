#include <stdio.h>
#include <math.h>
#include "math_util.h"
#pragma pack(1)

#define WAYPOINT_SIZE 100
float dot_product(float Vec1[],float Vec2[]);
int is_empty_matrix(float abb[5][WAYPOINT_SIZE]);
void cross_product(float Vec1[],float Vec2[], float Vec_prod[1][3]);

struct atp
{
	int size_waypoint_array;
	float R_min;
	float Va0;	
}atp1;

int main()
{
	atp1.size_waypoint_array = 100;
	atp1.R_min = 35*35/9.81;
	atp1.Va0 = 35;
	
	/*
	// path_follow() testing
	float out[4];
	float in[] = {2, 35, 10, 10, 10, 20, 10, 0, 100, 0, 0, 10, 1, 5, 5, 5, 30, 0.5, 0.1, 0.2, 0.2, 0.2, 0.1, 1, 0.4, 20, 3, 2, 0.4, 3};
	path_follow(in, atp1, out);
	printf("%f\n %f\n %f\n %f\n", out[0], out[1], out[2], out[3]);
	*/
	
	/* 
	//path_manager() testing
	float out[30];
	printf("Testiugskfg1\n");
	printf("%d\n %f\n %f\n", atp1.size_waypoint_array, atp1.R_min, atp1.Va0);
	float in[] = {4,
		0,    0,    -100,  0,           35,
		1200, 0,    -100,  45*PI/180,   35,
		0,    1200, -100,  45*PI/180,   35,
		1200, 1200, -100, -135*PI/180,  35,
		0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, 5, 5, 5, 30, 0.5, 0.1, 0.2, 0.2, 0.2, 0.1, 1, 0.4, 20, 3, 2, 0.4, 0};
	*/

	float in_pplanner[17] = {5, 5, 5, 30, 0.5, 0.1, 0.2, 0.2, 0.2, 0.1, 1, 0.4, 20, 3, 2, 0.4, 0}; //17 floats = 16 states + 1 time
	float out_pplanner[501];
	path_planner(in_pplanner, atp1, out_pplanner);

	float in_pmanager[518], out_pmanager[30];

	int i=0, j=0, NN=0;
	for(i=0; i<5*atp1.size_waypoint_array+1; i++)
	{
		in_pmanager[i] = out_pplanner[i];
	}

	NN = 5*atp1.size_waypoint_array+1; 

	j = 0;

	for(i=NN; i < NN+sizeof(in_pplanner)/sizeof(in_pplanner[0]); i++)
	{
		in_pmanager[i] = in_pplanner[j];
		j++;
	}

	path_manager(in_pmanager, atp1, out_pmanager);
	
	float in_pfollow[31], out_pfollow[4]; 

	for(i=0; i<sizeof(out_pmanager)/sizeof(out_pmanager[0]); i++)
	{
		in_pfollow[i] = out_pmanager[i];
	}

	/*
	printf("Testiugskfg2\n");
	printf("%f\n %f\n %f\n %f\n %f\n %f\n %f\n %f\n %f\n %f\n %f\n %f\n %f\n %f\n %f\n %f\n %f\n %f\n %f\n %f\n %f\n %f\n %f\n %f\n %f\n %f\n %f\n %f\n %f\n %f\n", out[0], out[1], out[2], out[3], out[4], out[5], out[6], out[7], out[8], out[9], out[10], out[11], out[12], out[13], out[14], out[15], out[16], out[17], out[18], out[19], out[20], out[21], out[22], out[23], out[24], out[25], out[26], out[27], out[28], out[29]);
	*/
	
	in_pfollow[sizeof(out_pmanager)/sizeof(out_pmanager[0])-1] = in_pplanner[sizeof(in_pplanner)/sizeof(in_pplanner[0])-1];

	path_follow(in_pfollow, atp1, out_pfollow);

	printf(" Va_c = %f\n h_c = %f\n chi_c = %f\n phi_ff = %f\n", out_pfollow[0], out_pfollow[1], out_pfollow[2], out_pfollow[3]);

	return 0;
}

void path_manager_fillet(float in[],struct atp atp1,int start_of_simulation,float waypoints[5][WAYPOINT_SIZE],float out[])
{
	static int ptr_a; // waypoint pointer
	static int ptr_b;
	static int state_transition;// state of transition state machine
    static int flag_need_new_waypoints;// % flag that request new waypoints from path planner
	
	int NN=0;
	float t=0.0;
	int i=0,j=0,k=0,l=0;
	static float waypoints_old[5][WAYPOINT_SIZE];// stored copy of old waypoints
	float state[16];
	int num_waypoints= (int)in[1+NN];
	
	float w_i[3][1],w_in[3][1],w_ip[3][1],temp3x1[3][1],q_n[3][1],q_p[3][1],q_n_trans[1][3],r[3][1],q[3][1],c[3][1];
	float norm_w_i_w_in=0.0,norm_w_ip_w_i=0.0;
	float temp_angle[1][1];
	float flag=0.0,Va_d=0.0;
	float rho = 0.0;
	float lambda = 0.0,halfplane[1][1];
	float z[3][1];
	float n_i[3][1];
	int match=0;
	float angle=0.0;
		
	NN = NN + 1 + 5*atp1.size_waypoint_array;
	float pn   = in[0+NN];
	float pe   = in[1+NN];
	float h    = in[2+NN];
	float temp=-1.0 ;
	
	for(i=0;i<16;i++)
	{
		state[i]=in[i+NN];		
	}

	NN=NN+16;
	t=in[NN];
	
	float p[3][1]={{pn},{pe},{-h}};
	float temp1x2[1][2],q_n_tem[2][1];
	float norm_qn_qp=0.0;
	
	if((start_of_simulation==1)||(is_empty_matrix(waypoints_old)==1))
	{
		for(i=0;i<5;i++)
		{
			for(j=0;j<WAYPOINT_SIZE;j++)
			{
				waypoints_old[i][j]=0.0;
			}
		}
		flag_need_new_waypoints=0;
	}
	
	/* Doubt and confusion
		
	if min(min(waypoints==waypoints_old))==0,
      ptr_a = 1;
      ptr_b = 2;
      waypoints_old = waypoints;
      state_transition = 1;
      flag_need_new_waypoints = 0;end
	
	*/
	for(i=0;i<5;i++)
	{
		for(j=0;j<WAYPOINT_SIZE;j++)
		{
			if(waypoints[i][j]!=waypoints_old[i][j])
			{
				ptr_a = 0;
				ptr_b = 1;
				for(k=0;k<5;k++)
				{
					for(l=0;l<WAYPOINT_SIZE;l++)
					{
						waypoints_old[k][l] = waypoints[k][l];
					}
				}
				state_transition = 1;
				flag_need_new_waypoints = 0;
				match=1;
				break;
			}
		}
		if(match==1)
		{
			break;
		}
	}
	
	for(i=0;i<3;i++)
	{
		w_i[i][0]=waypoints[i][ptr_a+1];
		w_in[i][0]=waypoints[i][ptr_a]; 
		w_ip[i][0]=waypoints[i][ptr_a+2];		
	}
	 norm_w_i_w_in=sqrtf(powf((w_i[0][0]-w_in[0][0]),2)+powf((w_i[1][0]-w_in[1][0]),2)+powf((w_i[2][0]-w_in[2][0]),2));
	 norm_w_ip_w_i=sqrtf(powf((w_ip[0][0]-w_i[0][0]),2)+powf((w_ip[1][0]-w_i[1][0]),2)+powf((w_ip[2][0]-w_i[2][0]),2));
	
	for(i=0;i<3;i++)
	{
		q_n[i][0]=(w_i[i][0]-w_in[i][0])/norm_w_i_w_in;
		q_p[i][0]=(w_ip[i][0]-w_i[i][0])/norm_w_ip_w_i;
		
	}
		
	q_n_trans[0][0]=q_n[0][0];
	q_n_trans[0][1]=q_n[1][0];
	q_n_trans[0][2]=q_n[2][0];
	
	for(i=0;i<3;i++)
	{
		q_n_trans[0][i] = (-1.0) * (q_n_trans[0][i]);
	}
	MatrixMultiply(q_n_trans,1,3,q_p,3,1,temp_angle);
	angle=acosf(temp_angle[0][0]);
	
	switch(state_transition)
	{
		case 1:
		
			flag   = 1; // % following straight line path
			Va_d   = waypoints[4][ptr_a]; // desired airspeed along waypoint path. ptr_a to computed as array starts from 0 in C
			  
			for(i=0;i<3;i++)
			{			  
			  r[i][0]=w_in[i][0];
			  q[i][0]=q_n[i][0];
	//         q      = q/norm(q);
			  c[i][0] = w_i[i][0];
			  
			}  
			  rho = 0.0;
			  lambda=0.0;
			  
			for(i=0;i<3;i++)
			{
				z[i][0] = w_i[i][0]-((atp1.R_min/tanf(angle/2))*q_n[i][0]);
			}
			
	//		halfplane = (p(1:2)-z(1:2))'*q_n(1:2);
			for(i=0;i<2;i++)
			{
				temp1x2[0][i]=p[i][0]-z[i][0];
			}
			MatrixMultiply(temp1x2,1,2,q_n_tem,2,1,halfplane);
			
			if(halfplane[0][0]>=0)
			{
			   state_transition=2;
			   flag_need_new_waypoints = 0;
			}
			break;
			
			
			
		case 2: //% follow orbit from wpp_a-wpp_b to wpp_b-wpp_c
			
			norm_qn_qp=sqrtf(powf((q_n[0][0]-q_p[0][0]),2)+powf((q_n[1][0]-q_p[1][0]),2)+powf((q_n[2][0]-q_p[2][0]),2));
			
			for(i=0;i<3;i++)
			{
				n_i[i][0]=(q_n[i][0]-q_p[i][0])/norm_qn_qp;
								
			}
			flag=2; //% following orbit
			Va_d   = waypoints[4][ptr_a];
			for(i=0;i<3;i++)
			{			  
			  r[i][0]=w_in[i][0];
			  q[i][0]=q_n[i][0];
	//         q      = q/norm(q);
			  c[i][0] = w_i[i][0]-((atp1.R_min/tanf(angle/2))*n_i[i][0]);
			  
			}  
			rho    = atp1.R_min;
			lambda = sign(q_n[0][0]*q_p[1][0]-q_n[1][0]*q_p[0][0]);
			
			 for(i=0;i<3;i++)
			  {
				z[i][0] = w_i[i][0]+((atp1.R_min/tanf(angle/2))*q_p[i][0]);
			  }
			 for(i=0;i<2;i++)
			{
				temp1x2[0][i]=p[i][0]-z[i][0];
			}
			MatrixMultiply(temp1x2,1,2,q_n_tem,2,1,halfplane); 
			if(halfplane[0][0]>=0)
			{
				 if((ptr_a<num_waypoints-1) && (ptr_a<ptr_b+1))
				 {
					ptr_a=ptr_a+1;
				 }
				flag_need_new_waypoints = 0;
				ptr_b=ptr_b+1;
				state_transition=1;//% state=1;
				
			}
			break;
			
		default:
			break;
			
	}
	out[0]=flag;
	out[1]=Va_d;
	out[2]=r[0][0];
	out[3]=r[1][0];
	out[4]=r[2][0];
	
	out[5]=q[0][0];
	out[6]=q[1][0];
	out[7]=q[2][0];
	
	out[8]=c[0][0];
	out[9]=c[1][0];
	out[10]=c[2][0];
	
	out[11]=rho;
	out[12]=lambda;
	
	for(i=0;i<16;i++)
	{
		out[i+13]=state[i];		
	}
	
	out[29]=(float)flag_need_new_waypoints;			  
}

struct L_idx{
	float L; //min(L1, L2, L3, L4)
	int indx; // index of the min L
};

struct L_idx min_4(float L1, float L2, float L3, float L4)
{
	struct L_idx Lmin;
	Lmin.L = L1<L2?(L1<L3?(L1<L4?L1:L4):(L3<L4?L3:L4)):(L2<L3?(L2<L4?L2:L4):(L3<L4?L3:L4));
	if(Lmin.L == L1)
		Lmin.indx = 1;
	else if(Lmin.L == L2)
		Lmin.indx = 2;
	else if(Lmin.L == L3)
		Lmin.indx = 3;
	else if(Lmin.L == L4)
		Lmin.indx = 4;
	return Lmin;
}

float norm_n(float a, float b, float c)
{
	float N;
	N = sqrtf(a*a+b*b+c*c);
	return N;
}

float modpi_(float theta)
{
	float a;
	a = theta - 2*PI*floor(theta/(2*PI));
	return a;
}

float* rotz(float theta)
{
	float *Rmat;
	Rmat = (float *) malloc(9*sizeof(float));

	*Rmat 	  = cosf(theta);
	*(Rmat+1) = -sinf(theta);
	*(Rmat+2) = 0;

	*(Rmat+3) = sinf(theta);
	*(Rmat+4) = cosf(theta);
	*(Rmat+5) = 0;

	*(Rmat+6) = 0;
	*(Rmat+7) = 0;
	*(Rmat+8) = 1;

	return Rmat;
	//free(Rmat);
}

struct dpath{
	float ps[3];
	float chis;
	float pe[3];
	float chie;
	float R;
	float L;
	float cs[3];
	float lams;
	float ce[3];
	float lame;
	float w1[3];
	float q1[3];
	float w2[3];
	float w3[3];
	float q3[3];
};

void dubinsParameters(float start_node[], float end_node[], float R_min, struct dpath *dubinspath)
{
	float ell;
	float cle[3], cls[3], cre[3], crs[3];
	float L1, L2, L3, L4, theta, theta2;
	int i;
	ell = sqrtf(powf((start_node[0]-end_node[0]),2) + powf((start_node[1]-end_node[1]),2));
	
	if(ell<2*R_min)
	{
		printf("The distance between nodes must be larger than 2R.\n");
		//dubinspath = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	}

	float ps[] = {start_node[0],start_node[1],start_node[2]};
	float chis = start_node[3];
	float pe[] = {end_node[0],end_node[1],end_node[2]};
	float chie = end_node[3];

	printf("Ps[] check = [%f %f %f]\n", ps[0], ps[1], ps[2]);
	printf("Rmin check = %f\n", R_min);

	//float *Rmat1, *Rmat2;
	float *Mat1, *Mat2;
	//float *test1;
	//test1 = (float *) malloc(9*sizeof(float));
	//Rmat2 = (float *) malloc(9*sizeof(float));
	Mat1 = (float *) malloc(3*sizeof(float));
	Mat2 = (float *) malloc(3*sizeof(float));

	//Rmat1 = rotz(PI/2, Rmat1);
	//test1 = rotz(-PI/2);

	//printf("Test1 = [%f %f %f %f %f %f %f %f %f]\n",*test1,*(test1+1),*(test1+2),*(test1+3),*(test1+4),*(test1+5),*(test1+6),*(test1+7),*(test1+8));

	*Mat1 = cosf(chis);
	*(Mat1+1) = sinf(chis);
	*(Mat1+2) = 0;

	*Mat2 = cosf(chie);
	*(Mat2+1) = sinf(chie);
	*(Mat2+2) = 0;

	float *temp3x1_1, *temp3x1_2, *temp3x1_3, *temp3x1_4;
	temp3x1_1 = (float *) malloc(3*sizeof(float));
	temp3x1_2 = (float *) malloc(3*sizeof(float));
	temp3x1_3 = (float *) malloc(3*sizeof(float));
	temp3x1_4 = (float *) malloc(3*sizeof(float));

	MatrixMultiply(rotz(PI/2),3,3,Mat1,3,1,temp3x1_1);
	MatrixMultiply(rotz(-PI/2),3,3,Mat1,3,1,temp3x1_2);
	MatrixMultiply(rotz(PI/2),3,3,Mat2,3,1,temp3x1_3);
	MatrixMultiply(rotz(-PI/2),3,3,Mat2,3,1,temp3x1_4);

    printf("temp3x1_2 = [%f %f %f]\n",temp3x1_2[0],temp3x1_2[1],temp3x1_2[2]);

	for(int i=0; i<3; i++)
	{
		crs[i] = ps[i] + R_min * temp3x1_1[i];
		cls[i] = ps[i] + R_min * temp3x1_2[i];
		cre[i] = pe[i] + R_min * temp3x1_3[i];
		cle[i] = pe[i] + R_min * temp3x1_4[i];
	}

	printf("ps[i] = [%f %f %f]\n",ps[0],ps[1],ps[2]);
	printf("cls[i] = [%f %f %f]\n",cls[0],cls[1],cls[2]);

	//compute L1
	theta = modpi_(atan2f(cre[1]-crs[1], cre[0]-crs[0]) + 2*PI);
	L1 = norm_n(crs[0]-cre[0], crs[1]-cre[1], crs[2]-cre[2]) + R_min*modpi_(2*PI+modpi_(theta-PI/2)-modpi_(chis-PI/2)) + R_min*modpi_(2*PI + modpi_(chie-PI/2) - modpi_(theta-PI/2));

	//compute L2
	ell = norm_n(cle[0]-crs[0], cle[1]-crs[1], cle[2]-crs[2]);
	theta = modpi_(atan2f(cle[1]-crs[1], cle[0]-crs[0]) + 2*PI);
	theta2 = theta - PI/2 + asinf(2*R_min/ell);

	if(ell==0)
		L2 = 9999;
	else 
	{
		L2 = sqrtf(ell*ell-4*R_min*R_min) + R_min*modpi_(2*PI+modpi_(theta2) - modpi_(chis-PI/2)) + R_min*modpi_(2*PI + modpi_(theta2+PI) - modpi_(chie+PI/2));
	}

	//compute L3
	ell = norm_n(cre[0]-cls[0], cre[1]-cls[1], cre[2]-cls[2]);
	theta = modpi_(atan2f(cls[1]-cre[1], cls[0]-cre[0])+2*PI);
	theta2 = acosf(2*R_min/ell);
	if(ell==0)
		L3 = 9999;
	else 
	{
		L3 = sqrtf(ell*ell - 4*R_min*R_min) + R_min*modpi_(2*PI + modpi_(chis+PI/2) - modpi_(theta+theta2)) + R_min*(2*PI + modpi_(chie-PI/2) - modpi_(theta+theta2-PI));
	}

	//compute L4
	theta = modpi_(atan2f(cls[1]-cle[1],cls[0]-cle[0])+2*PI);
	L4 = norm_n(cls[0]-cle[0], cls[1]-cle[1], cls[2]-cle[2]) + R_min*modpi_(2*PI+modpi_(chis+PI/2) - modpi_(theta+PI/2)) +R_min*modpi_(2*PI + modpi_(theta+PI/2) - modpi_(chie+PI/2));

	//Minimum distance of all 4 evaluations
	struct L_idx Lmin;
	Lmin = min_4(L1, L2, L3, L4);
	printf("Checking Lmin = %f %d\n", Lmin.L, Lmin.indx);

	float *e1, *_q1, *_q3, *temp3x1_5;
	e1 = (float *) malloc(3*sizeof(float));
	_q1 = (float *) malloc(3*sizeof(float));
	_q3 = (float *) malloc(3*sizeof(float));
	temp3x1_5 = (float *) malloc(3*sizeof(float));

	*e1 = 1;
	*(e1+1) = 0;
	*(e1+2) = 0;

	float cs[3], ce[3], _w1[3], _w2[3], _w3[3]; 
	int lams, lame;
	float norm_cecs;

	switch(Lmin.indx)
	{
		case 1:
			lams = 1;
			lame = 1;
			for(i=0; i<3; i++)
			{
				cs[i] = crs[i];
				ce[i] = cre[i];
			}
			norm_cecs = norm_n(ce[0]-cs[0], ce[1]-cs[1], ce[2]-cs[2]); 
			
			for(i=0; i<3; i++)
			{
				*(_q1+i) = (ce[i]-cs[i])/norm_cecs;
			}

			MatrixMultiply(rotz(-PI/2),3,3,_q1,3,1,temp3x1_5);

			for(i=0; i<3; i++)
			{
				_w1[i] = cs[i] + R_min*(*(temp3x1_5+i));
				_w2[i] = ce[i] + R_min*(*(temp3x1_5+i));
			}

		break;

		case 2:
			lams = 1;
			lame = -1;
			for(i=0; i<3; i++)
			{
				cs[i] = crs[i];
				ce[i] = cle[i];
			}
			ell = norm_n(ce[0]-cs[0], ce[1]-cs[1], ce[2]-cs[2]);
			theta = atan2f(ce[1]-cs[1], ce[0]-cs[0]+2*PI);
            theta2 = theta - PI/2 + asinf(2*R_min/ell);
            MatrixMultiply(rotz(theta2+PI/2),3,3,e1,3,1,_q1);
            MatrixMultiply(rotz(theta2),3,3,e1,3,1,temp3x1_5);
            for(i=0; i<3; i++)
			{
				_w1[i] = cs[i] + R_min*(*(temp3x1_5+i));
			}
			MatrixMultiply(rotz(theta2+PI),3,3,e1,3,1,temp3x1_5);
			for(i=0; i<3; i++)
			{
				_w2[i] = ce[i] + R_min*(*(temp3x1_5+i));
			}
		break;

		case 3:
			lams = -1;
			lame = 1;
			for(i=0; i<3; i++)
			{
				cs[i] = cls[i];
				ce[i] = cre[i];
			}
			ell = norm_n(ce[0]-cs[0], ce[1]-cs[1], ce[2]-cs[2]);
			theta = atan2f(ce[1]-cs[1],ce[0]-cs[0]+2*PI);
            theta2 = acosf(2*R_min/ell);
            MatrixMultiply(rotz(theta+theta2-PI/2),3,3,e1,3,1,_q1);
            MatrixMultiply(rotz(theta+theta2),3,3,e1,3,1,temp3x1_5);
            for(i=0; i<3; i++)
			{
				_w1[i] = cs[i] + R_min*(*(temp3x1_5+i));
			}
			MatrixMultiply(rotz(theta+theta2-PI),3,3,e1,3,1,temp3x1_5);
			for(i=0; i<3; i++)
			{
				_w2[i] = ce[i] + R_min*(*(temp3x1_5+i));
			}
		break;

		case 4:
			lams = -1;
			lame = -1;
			for(i=0; i<3; i++)
			{
				cs[i] = cls[i];
				ce[i] = cle[i];
			}
			norm_cecs = norm_n(ce[0]-cs[0], ce[1]-cs[1], ce[2]-cs[2]); 
			
			for(i=0; i<3; i++)
			{
				*(_q1+i) = (ce[i]-cs[i])/norm_cecs;
			}
			MatrixMultiply(rotz(PI/2),3,3,_q1,3,1,temp3x1_5);
            for(i=0; i<3; i++)
			{
				_w1[i] = cs[i] + R_min*(*(temp3x1_5+i));
			}
			MatrixMultiply(rotz(PI/2),3,3,_q1,3,1,temp3x1_5);
			for(i=0; i<3; i++)
			{
				_w2[i] = ce[i] + R_min*(*(temp3x1_5+i));
			}
		break;

		default:
		break;
	}

	MatrixMultiply(rotz(chie),3,3,e1,3,1,_q3);
	for(i=0; i<3; i++)
	{
		_w3[i] = pe[i];
	}

	dubinspath->chis = chis;
    dubinspath->chie = chie;
    dubinspath->R    = R_min;
    dubinspath->L    = Lmin.L;
    dubinspath->lams = lams;
    dubinspath->lame = lame;

    for(i=0;i<3;i++)
    {
	    dubinspath->ps[i]   = ps[i];
	    dubinspath->pe[i]   = pe[i];
	    dubinspath->cs[i]   = cs[i];
	    dubinspath->ce[i]   = ce[i];
	    dubinspath->w1[i]   = _w1[i];
	    dubinspath->q1[i]   = _q1[i];
	    dubinspath->w2[i]   = _w2[i];
	    dubinspath->w3[i]   = _w3[i];
	    dubinspath->q3[i]   = _q3[i];
	}

	//printf("q1[i] test1 = [%f %f %f]\n",dubinspath->q1[0],dubinspath->q1[1],dubinspath->q1[2]);
	printf("cs[i] test1 = [%f %f %f]\n",dubinspath->cs[0],dubinspath->cs[1],dubinspath->cs[2]);
	
	free(Mat1);
	free(Mat2);
	free(temp3x1_1);
	free(temp3x1_2);
	free(temp3x1_3);
	free(temp3x1_4);
	free(temp3x1_5);
	free(e1);
	free(_q1);
	free(_q3);

}

void path_manager_dubins(float in[],struct atp atp1,int start_of_simulation,float waypoints[5][WAYPOINT_SIZE],float out[])
{
  float state[16];
  int NN = 0;
  int i,j,k;
  int num_waypoints= (int)in[1+NN];
  //waypoints = reshape(in(2+NN:5*P.size_waypoint_array+1+NN),5,P.size_waypoint_array);
  NN = NN + 1 + 5*atp1.size_waypoint_array;
  float pn        = in[1+NN];
  float pe        = in[2+NN];
  float h         = in[3+NN];
  // Va      = in(4+NN);
  // alpha   = in(5+NN);
  // beta    = in(6+NN);
  // phi     = in(7+NN);
  // theta   = in(8+NN);
  float chi     = in[9+NN];
  // p       = in(10+NN);
  // q       = in(11+NN);
  // r       = in(12+NN);
  // Vg      = in(13+NN);
  // wn      = in(14+NN);
  // we      = in(15+NN);
  // psi     = in(16+NN);

  for(i=0;i<16;i++)
	{
		state[i]=in[i+NN];		
	}

  NN = NN + 16;
  int t  = in[NN]; 
  
 //float p[3][1]={{pn},{pe},{-h}};

  static float waypoints_old[5][WAYPOINT_SIZE];   // stored copy of old waypoints
  static int ptr_a, ptr_b;           // waypoint pointer
  static int state_transition; // state of transition state machine
  //static int start_of_simulation;
  static struct dpath *dubinspath;
  static int flag_need_new_waypoints; // flag that request new waypoints from path planner
  static int flag_first_time_in_state;
  
  dubinspath = (struct dpath*)malloc(sizeof(struct dpath));
  
  	if(start_of_simulation==1)
	{
		for(i=0;i<5;i++)
		{
			for(j=0;j<WAYPOINT_SIZE;j++)
			{
				waypoints_old[i][j]=0.0;
			}
		}
		flag_need_new_waypoints=0;
		state_transition = 0;
      	flag_first_time_in_state = 1;
	}

	int match;
	for(i=0;i<5;i++)
	{
		for(j=0;j<WAYPOINT_SIZE;j++)
		{
			if(waypoints[i][j]!=waypoints_old[i][j])
			{
				ptr_a = 0;
				ptr_b = 1;
				for(k=0;k<5;k++)
				{
					for(int l=0;l<WAYPOINT_SIZE;l++)
					{
						waypoints_old[k][l] = waypoints[k][l];
					}
				}
				state_transition = 1;
				flag_need_new_waypoints = 0;
				flag_first_time_in_state = 1;

				float start_node[] = {waypoints[0][ptr_a], waypoints[1][ptr_a], waypoints[2][ptr_a], waypoints[3][ptr_a], 0, 0};
				float end_node[]   = {waypoints[0][ptr_b], waypoints[1][ptr_b], waypoints[2][ptr_b], waypoints[3][ptr_b], 0, 0};
				dubinsParameters(start_node, end_node, atp1.R_min, dubinspath); 
					//printf("q1[i] test2 = [%f %f %f]\n",dubinspath->q1[0],dubinspath->q1[1],dubinspath->q1[2]);
					printf("cs[i] test2 = [%f %f %f]\n",dubinspath->cs[0],dubinspath->cs[1],dubinspath->cs[2]);

				match=1;
				break;
			}
		}
		if(match==1)
		{
			break;
		}
	}

	float _p[] = {pn,pe,-h};
	float *_p1, *_p2, *_p3;
	float *_p4, *_p5;
	float *_p6, *_p7, *_p8;
	_p1 = (float *) malloc(3*sizeof(float));
	_p2 = (float *) malloc(3*sizeof(float));
	_p3 = (float *) malloc(sizeof(float));
	_p4 = (float *) malloc(3*sizeof(float));
	_p5 = (float *) malloc(sizeof(float));
	_p6 = (float *) malloc(3*sizeof(float));
	_p7 = (float *) malloc(3*sizeof(float));
	_p8 = (float *) malloc(sizeof(float));
	
	int flag;
	float Va_d, rho, lambda;
	float _r[3], _q[3], _c[3];

	for(int i=0;i<3;i++){
		*(_p1+i)= _p[i]-dubinspath->w1[i];
		*(_p2+i)= dubinspath->q1[i];
		*(_p4+i)= _p[i]-dubinspath->w2[i];
		*(_p6+i)= _p[i]-dubinspath->w3[i];
		*(_p7+i)= dubinspath->q3[i];
	}

	MatrixMultiply(_p1,1,3,_p2,3,1,_p3);
	MatrixMultiply(_p4,1,3,_p2,3,1,_p5);
	MatrixMultiply(_p6,1,3,_p7,3,1,_p8);
 
	switch(state_transition)
	{

		case 0:
			flag = 2;
			Va_d = waypoints[4][ptr_a];
			rho = dubinspath->R;
			lambda = dubinspath->lams;
			
			for(int i=0;i<3;i++)
			{
				_r[i] = -999;
				_q[i] = -999;
				_c[i] = dubinspath->cs[i];
			}

			if(flag_first_time_in_state==1)
				{flag_first_time_in_state=0;}
		break;

		case 1:
			flag = 2;
			Va_d = waypoints[4][ptr_a];
			rho = dubinspath->R;
			lambda = dubinspath->lams;
			
			for(int i=0;i<3;i++)
			{
				_r[i] = -999;
				_q[i] = -999;
				_c[i] = dubinspath->cs[i];
			}
 	printf("_c[i] = [%f %f %f]\n",_c[0],_c[1],_c[2]);

			if((*_p3>=0)&&(flag_first_time_in_state==1)){
				state_transition = 2;
				flag_first_time_in_state = 1; 			
			}
			else if(*_p3>=0){
				state_transition = 3;
				flag_first_time_in_state = 1;
			}
			else
				flag_first_time_in_state = 0;
		break;

		case 2:
			flag = 2;
			Va_d = waypoints[4][ptr_a];
			rho = dubinspath->R;
			lambda = dubinspath->lams;
			
			for(int i=0;i<3;i++)
			{
				_r[i] = -999;
				_q[i] = -999;
				_c[i] = dubinspath->cs[i];
			}

			if(*_p3<0){
				state_transition = 1;
				flag_first_time_in_state = 1;
			}
			else 
				flag_first_time_in_state = 0;
		break;

		case 3:
			flag = 1;
			Va_d = waypoints[4][ptr_a];
			rho = -999;
			lambda = -999;
			for(int i=0;i<3;i++)
			{
				_r[i] = dubinspath->w1[i];
				_q[i] = dubinspath->q1[i];
				_c[i] = -999;
			}
			flag_first_time_in_state = 0;

			if(*_p5>=0){
				state_transition = 4;
				flag_first_time_in_state = 1;
			}
		break;

		case 4:
			flag = 2;
			Va_d = waypoints[4][ptr_a];
			rho = dubinspath->R;
			lambda = dubinspath->lame;
			
			for(int i=0;i<3;i++){
				_r[i] = -999;
				_q[i] = -999;
				_c[i] = dubinspath->ce[i];
			}
			flag_first_time_in_state = 0;

			if((*_p8 >= 0)&&(flag_first_time_in_state==1)){
				state_transition = 5;
				flag_first_time_in_state = 1;
			}
			else if(*_p8 >= 0){
				if(ptr_a == num_waypoints-2){
					flag_need_new_waypoints = 1;
					ptr_b = ptr_a+1;
				}
				else{
					ptr_a = ptr_a+1;
					//ptr_b = ptr_a+1; //this was the original part but seems to be wrong
					ptr_b = ptr_b+1;
					state_transition = 1;
					flag_first_time_in_state = 1;
				}
				float start_node[] = {waypoints[0][ptr_a], waypoints[1][ptr_a], waypoints[2][ptr_a], waypoints[3][ptr_a], 0, 0};
				float end_node[]   = {waypoints[0][ptr_b], waypoints[1][ptr_b], waypoints[2][ptr_b], waypoints[3][ptr_b], 0, 0};
				dubinsParameters(start_node, end_node, atp1.R_min, dubinspath);
			}
			else{
				flag_first_time_in_state = 0;
			}
		break;

		case 5:
			flag = 2;
			Va_d = waypoints[4][ptr_a];
			rho = dubinspath->R;
			lambda = dubinspath->lame;
			
			for(int i=0;i<3;i++){
				_r[i] = -999;
				_q[i] = -999;
				_c[i] = dubinspath->ce[i];
			}
			flag_first_time_in_state = 0;

			if(*_p8<0){
				state_transition = 4;
				flag_first_time_in_state = 1;
			}
			else{
				flag_first_time_in_state = 0;
			}
		break;

		default:
		break;
	}
	out[0]=flag;
	out[1]=Va_d;

	out[2]=_r[0];
	out[3]=_r[1];
	out[4]=_r[2];

	out[5]=_q[0];
	out[6]=_q[1];
	out[7]=_q[2];

	out[8]	=_c[0];
	out[9]	=_c[1];
	out[10]	=_c[2];

	out[11]=rho;
	out[12]=lambda;

	for(i=0;i<16;i++)
	{
		out[i+13]=state[i];		
	}

	out[29]=(float)flag_need_new_waypoints;	

	free(_p1);
	free(_p2);	
	free(_p3);
	free(_p4);
	free(_p5);
	free(_p6);
	free(_p7);
	free(_p8);

}

void path_planner(float in[], struct atp atp1,float out[])
//void path_planner(struct atp atp1,float out[])
{
	float NN=0.0;
	static int num_waypoints=0;
	float wpp[atp1.size_waypoint_array][5];
	
	int i=0,j=0,k=0;
	
	array_initd((float*)wpp,atp1.size_waypoint_array*5);
	
	static int flag_temp=0;
	
	//below if is not satisfied hence would go to else - nik.
	/*if(flag_temp==1)
	{
		wpp[0][0]=0.0;
		wpp[0][1]=0.0;
		wpp[0][2]=0.0;
		wpp[0][3]=-9999.0;
		wpp[0][4]=atp1.Va0;
		
		wpp[1][0]=1000.0;
		wpp[1][1]=0.0;
		wpp[1][2]=-100.0;
		wpp[1][3]=-9999.0;
		wpp[1][4]=atp1.Va0;
		
		wpp[2][0]=0.0;
		wpp[2][1]=1100.0;
		wpp[2][2]=-100.0;
		wpp[2][3]=-9999.0;
		wpp[2][4]=atp1.Va0;
		
		wpp[3][0]=1200.0;
		wpp[3][1]=1200.0;
		wpp[3][2]=-100.0;
		wpp[3][3]=-9999,0;
		wpp[3][4]=atp1.Va0;
		
		wpp[4][0]=0.0;
		wpp[4][1]=0.0;
		wpp[4][2]=0.0;
		wpp[4][3]=-9999.0;
		wpp[4][4]=atp1.Va0;
		
		num_waypoints=5; //starting from 0
	}
	else*/
	//{
		wpp[0][0]=0.0;
		wpp[0][1]=0.0;
		wpp[0][2]=-100.0;
		wpp[0][3]=0.0;
		wpp[0][4]=atp1.Va0;
		
		wpp[1][0]=1200.0;
		wpp[1][1]=0.0;
		wpp[1][2]=-100.0;
		wpp[1][3]=(45.0*PI)/180;
		wpp[1][4]=atp1.Va0;
		
		wpp[2][0]=0.0;
		wpp[2][1]=1200.0;
		wpp[2][2]=-100.0;
		wpp[2][3]=(45.0*PI)/180;
		wpp[2][4]=atp1.Va0;
		
		wpp[3][0]=1200.0;
		wpp[3][1]=1200.0;
		wpp[3][2]=-100.0;
		wpp[3][3]=(-135.0*PI)/180;
		wpp[3][4]=atp1.Va0;		
		
		num_waypoints=4; //starting from 0
	//}
	for(i=num_waypoints;i<atp1.size_waypoint_array;i++)
	{
		for(j=0;j<5;j++)
		{
			wpp[i][j]=-9999.0;
		}
	}

	k=1;
	out[0]=num_waypoints;
	for(i=0;i<num_waypoints;i++)
	{
		for(j=0;j<5;j++)
		{
			out[k]=wpp[i][j];
			k++;
		}
	}
}

void path_manager(float in[],struct atp atp1,float out[])
{
	static int start_of_simulation;
	float  state[16];
	
	int i=0,j=0;
	float waypoints[5][atp1.size_waypoint_array];
	int flag=0;
	float  Va_d =0.0;
	int  flag_need_new_waypoints=0.0;
	
	float pn=0.0;
	float pe=0.0;
	float h =0.0;
	float  chi=0.0;
	float  r[3][1]   = {{0.0},{0.0},{0.0}};
	float  q[3][1]   = {{0.0},{0.0},{0.0}};
	float  c[3][1]   = {{0.0},{0.0},{0.0}};
	float  rho       = 0.0;
	float  lambda    = 0.0;
	
	int size_of_in=sizeof(in)/sizeof(float);
	
	for(i=0;i<16;i++)
	{
		state[i]=0.0;
	}
	
	for(i=0;i<5;i++)// Initializaton of waypoint array
	{
		for(j=0;j<atp1.size_waypoint_array;j++)
		{
			waypoints[i][j]=0.0;
		}
	}
	
	if(in[size_of_in-1]==0.0)//t
	{
		start_of_simulation=1;
	}
		
	int NN=0;
	int  num_waypoints = (int)in[0+NN];
		printf("Testiugskfg3m1\n");
	if(num_waypoints==0)
	{
				printf("Testiugskfg3.0\n");
			flag   = 1; // % following straight line path
			Va_d   = atp1.Va0;// % desired airspeed along waypoint path
			NN     = NN + 1 + 5*atp1.size_waypoint_array;
		//       NN = 1+NN;
						
			pn        = in[0+NN];
			pe        = in[1+NN];
			h         = in[2+NN];
			
			chi       = in[8+NN];
			
			r[0][0]   = pn;
			r[1][0]   = pe;
			r[2][0]   = -h;	  
			  
			q[0][0]   = cosf(chi);
			q[1][0]   = sinf(chi);
			q[2][0]   = 0;
			 
			c[0][0]   = 0.0;
			c[1][0]   = 0.0;
			c[2][0]   = 0.0;
			  
			rho       = 0.0;
			lambda    = 0.0;
			
			for(i=0;i<16;i++)
			{
				state[i]=in[i+NN];
			}		
		
		//	state     =  in(1+NN:16+NN);
			flag_need_new_waypoints = 1;
			out[0]=	flag;
			out[1]=Va_d;
			out[2]=r[0][0];
			out[3]=r[1][0];
			out[4]=r[2][0];
			
			out[5]=q[0][0];
			out[6]=q[1][0];
			out[7]=q[2][0];
			
			out[8]=c[0][0];
			out[9]=c[1][0];
			out[10]=c[2][0];
			
			out[11]=rho;
			out[12]=lambda;
			
			for(i=0;i<16;i++)
			{
				out[i+13]=state[i];
			}
			printf("Testiugskfg3.0\n");
			//out[28]=(float)flag_need_new_waypoints;//doubt
			out[29]=(float)flag_need_new_waypoints;//doubt
	}
	else
	{
			printf("Testiugskfg3.1\n");
			//printf("atp1.size_waypoint_array = %d\n",atp1.size_waypoint_array);
		//waypoints matrix being used in two function.
		for(j=0;j<atp1.size_waypoint_array;j++)
		{
			//printf("Testiugskfg3.2\n");
			for(i=0;i<5;i++)
			{
				waypoints[i][j]=in[1+j*5+i];
			}
			
		}
					printf("Checking waypoints(0-4,0) = [%f %f %f %f]\n",waypoints[0][0],waypoints[1][0],waypoints[2][0],waypoints[3][0],waypoints[4][0]);

					printf("Checking waypoints(0-4,1) = [%f %f %f %f]\n",waypoints[0][1],waypoints[1][1],waypoints[2][1],waypoints[3][1],waypoints[4][1]);
						//printf("Testiugskfg3.2\n");
		
		//if(fabsf(waypoints[4][0]>=2*PI))	
		//if(fabsf(waypoints[3][0]>=2*PI))
		//if(1)
		//		//printf("Testiugskfg3.2\n");
		//{
		//	//printf("Testiugskfg3\n");
		//	path_manager_fillet(in,atp1, start_of_simulation, waypoints,out);
		//	start_of_simulation=0;
		//}
		//else
		//{
			// % follows Dubins paths between waypoint configurations
        path_manager_dubins(in,atp1,start_of_simulation,waypoints,out); 
        start_of_simulation=0;
        printf("check out[8,9,10] = [%f %f %f]\n", out[8], out[9], out[10]);
			
		//}
	}
}

void path_follow(float in[], struct atp atp1, float out[])
{
	float chi_inf = 60.0*PI/180.0;  //approach angle for large distance from straight-line path
    float k_path  = 0.01;        //proportional gain for path following
    float k_orbit = 2.5;       //proportional gain for orbit following
    float gravity = 9.81;

	int NN=0;
	int i=0,j=0;

	//PATH input to path_follow
	printf("check in[0] = %f\n", in[0]);
	int flag  = (int)in[0+NN];
    float Va_d = in[1+NN];
    float r_path[3][1] = {{in[2+NN]},{in[3+NN]},{in[4+NN]}};
    float q_path[3][1] = {{in[5+NN]},{in[6+NN]},{in[7+NN]}};
    float c_orbit[3][1]= {{in[8+NN]},{in[9+NN]},{in[10+NN]}};
    float rho_orbit = in[11+NN];
    float lam_orbit = in[12+NN];
	
    printf("checking in[8,9,10] = [%f %f %f]\n", in[8+NN], in[9+NN], in[10+NN]);

	float s_i[3]={0.0,0.0,0.0};
	
	float n_lon_transposed[1][3]={0.0,0.0,0.0};
	float unit_prod[3]={0.0,0.0,0.0};
	
	float epi[3]={0.0,0.0,0.0};
    
    NN = NN + 13;
    
    // STATE input to path_follow
    float pn = in[0+NN];
    float pe = in[1+NN];
    float h    = in[2+NN];
    float Va   = in[3+NN];
    float alpha= in[4+NN];
    float beta = in[5+NN];
    float phi  = in[6+NN];
    float theta   = in[7+NN];
    float chi     = in[8+NN];
    float p       = in[9+NN];
    float q       = in[10+NN];
    float r       = in[11+NN];
    float Vg      = in[12+NN];
    float wn      = in[13+NN];
    float we      = in[14+NN];
    float psi     = in[15+NN];
	
	float t = in[16+NN];

	printf("time check: %f\n", t);

	float chi_q=0.0;
	float chi_c=0.0;
	float prod[1][3]={0.0,0.0,0.0};

	float n_lon[3][1],temp1x3_1[3];
	float Va_c=0.0;
	float epi_dot=0.0;
	float s_n=0.0;
	float s_e=0.0;
	float s_d=0.0;
	float h_c=0.0;
	float d_s=0.0;
	float phi_ff=0.0;
	float epy[1][1];
	//float ch_c=0.0;
	float temp1x3_2[1][3];
	float temp3x1_1[3][1];
		
	NN = NN + 16;
      
    switch (flag)
	{
        case 1: //follow straight line path specified by r and q
          
            //compute wrapped version of path angle
            chi_q = atan2f(q_path[1][0],q_path[0][0]);
			
            while((chi_q - chi) < -PI)
			{
				chi_q = chi_q + 2*PI; 
			}
            while((chi_q - chi) > +PI)
			{
				chi_q = chi_q - 2*PI;
			}

           	n_lon[0][0]=-sinf(chi_q);
			n_lon[1][0]=cosf(chi_q);
			n_lon[2][0]=0.0;
			
			temp1x3_1[0]=0.0;
			temp1x3_1[1]=0.0;
			temp1x3_1[2]=1.0;
			
			temp1x3_2[0][0]=q_path[0][0];
			temp1x3_2[0][1]=q_path[1][0];
			temp1x3_2[0][2]=q_path[2][0];
            
			cross_product(temp1x3_2,temp1x3_1,prod);
			// prod=cross(q_path,[0;0;1]);
			
			unit_prod[0]=prod[0][0]/sqrtf(powf(prod[0][0],2)+powf(prod[0][1],2)+powf(prod[0][2],2));
			unit_prod[1]=prod[0][1]/sqrtf(powf(prod[0][0],2)+powf(prod[0][1],2)+powf(prod[0][2],2));
			unit_prod[2]=prod[0][2]/sqrtf(powf(prod[0][0],2)+powf(prod[0][1],2)+powf(prod[0][2],2));
			//unit_prod=prod/norm(prod);
            epi[0]=pn-r_path[0][0];
			epi[1]=pe-r_path[1][0];
			epi[2]=h-r_path[2][0];
            
			epi_dot=dot_product(epi,unit_prod);
			//			epi_dot=dot(epi,unit_prod);
           
			//  s_i=epi-(epi_dot*unit_prod);
			
			for(i=0;i<3;i++)
			{
				s_i[i]=epi[i]-(epi_dot*unit_prod[i]);
			}
			 s_n=s_i[0];
			 s_e=s_i[1];
			 s_d=s_i[2];

            h_c = -r_path[2][0]-sqrtf(powf(s_n,2)+powf(s_e,2))*(q_path[2][0]/sqrtf(powf(q_path[0][0],2)+powf(q_path[1][0],2)));
			// h_c = -r_path(3)-sqrt(s_n^2+s_e^2)*(q_path(3)/sqrt(q_path(1)^2+q_path(2)^2));
           			
			n_lon_transposed[0][0]=n_lon[0][0];
			n_lon_transposed[0][1]=n_lon[1][0];
			n_lon_transposed[0][2]=n_lon[2][0];
					
			temp3x1_1[0][0]=pn-r_path[0][0];
			temp3x1_1[1][0]=pe-r_path[1][0];
			temp3x1_1[2][0]=-h-r_path[2][0];
						
			MatrixMultiply(n_lon_transposed,1,3,temp3x1_1,3,1,epy);
		//	epy=n_lon'*([pn;pe;-h]-r_path);
        //   chi_c = chi_q-chi_inf*2*atan(k_path*epy)/pi; 
					
			chi_c=chi_q-chi_inf*2*atanf(k_path*epy[0][0])/PI ;
		
			phi_ff =-0*PI/180;
			//printf("test case 1 \n");
		break;
           
        case 2: // follow orbit specified by c, rho, lam
        
	        // commanded altitude is the height of the orbit  
	        h_c = -c_orbit[2][0];	
	        printf("checking C_rbit = [%f %f %f]\n", c_orbit[0][0], c_orbit[1][0], c_orbit[2][0]);
	        // distance from orbit center
	        d_s = sqrtf(powf((pn-c_orbit[0][0]),2)+powf((pe-c_orbit[1][0]),2)); 
	     
	        // the roll angle feedforward command
	        phi_ff = atan2f((pe-c_orbit[1][0]),(pn-c_orbit[0][0]));
	        //printf("test 1: %f\n", chi);
			 
			if((phi_ff-chi)<-PI)
			{
	            phi_ff=phi_ff+2*PI;
	        }
	        else if((phi_ff-chi)>PI)
			{
	            phi_ff=phi_ff-2*PI;
	        }
	         
	        //printf("test 2: %f\n", phi_ff);
	         // heading command
	        chi_c = phi_ff+lam_orbit*((PI/2)+(atanf((k_orbit*(d_s-rho_orbit)/rho_orbit))));
			//printf("test 3: %f\n", phi_ff);
		break;
		
		default:
		printf("Default case\n");
		break;
	}
  	//printf("test 4: %f\n", phi_ff);
    // command airspeed equal to desired airspeed
    Va_c = Va_d;
  
    // create output
    out[0]=Va_c;
	out[1]=h_c;
	out[2]=chi_c;
	//printf("test 4: %f\n", phi_ff);
	out[3]=phi_ff;
}

void cross_product(float Vec1[],float Vec2[], float Vec_prod[][3])
{
	Vec_prod[0][0]=Vec1[1]*Vec2[2]-Vec2[1]*Vec1[2] ;
	Vec_prod[0][1]=-(Vec1[0]*Vec2[2]-Vec2[0]*Vec1[2]);
	Vec_prod[0][2]=Vec1[0]*Vec2[1]-Vec2[0]*Vec1[1];
}

float dot_product(float Vec1[],float Vec2[])
{
	int i=0;
	float dot_prod_op=0.0;
	dot_prod_op=Vec1[i]*Vec2[i]+Vec1[i+1]*Vec2[i+1]+Vec1[i+2]*Vec2[i+2];
	return dot_prod_op;
}
	
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
