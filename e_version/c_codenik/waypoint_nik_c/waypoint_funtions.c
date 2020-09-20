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
	float out[4];
	float in[] = {2, 35, 10, 10, 10, 20, 10, 0, 100, 0, 0, 10, 1, 5, 5, 5, 30, 0.5, 0.1, 0.2, 0.2, 0.2, 0.1, 1, 1, 0.4, 20, 3, 2, 0.4};
	atp1.size_waypoint_array = 100;
	atp1.R_min = 35*35/9.81;
	atp1.Va0 = 35;
	path_follow(in, atp1, out);
	//printf("%d\n %f\n %f\n", atp1.size_waypoint_array, atp1.R_min, atp1.Va0);
	printf("%f\n %f\n %f\n %f\n", out[0], out[1], out[2], out[3]);
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
	
	for(i=0;i<17;i++)
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
			for(j=0;j>WAYPOINT_SIZE;j++)
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
      flag_need_new_waypoints = 0;
	end
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
	
	for(i=0;i<17;i++)
	{
		out[i+13]=state[i];		
	}
	
	out[28]=(float)flag_need_new_waypoints;
				  
			  
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
	for(i=0;i<15;i++)
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
	
	if(num_waypoints==0)
	{
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
			
			for(i=0;i<17;i++)
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
			
			for(i=0;i<17;i++)
			{
				out[i+13]=state[i];
			}
			out[28]=(float)flag_need_new_waypoints;//doubt
		
	}
	else
	{
		
		//waypoints matrix being used in two function.
		for(i=0;i<5;i++)
		{
			for(j=0;j<(5*atp1.size_waypoint_array)+1;j++)
			{
				waypoints[i][j]=in[1+(i*j)];
			}
		}
	
		
		if(fabsf(waypoints[4][0]>=2*PI))
		{
			path_manager_fillet(in,atp1, start_of_simulation, waypoints,out);
			start_of_simulation=0;
		}
		else
		{
			// % follows Dubins paths between waypoint configurations
        //out = path_manager_dubins(in,atp,start_of_simulation); 
        start_of_simulation=0;
			
		}
	}
}


void path_planner(float in[], struct atp atp1,float out[])
{
	float NN=0.0;
	static int num_waypoints=0;
	float wpp[atp1.size_waypoint_array][5];
	
	int i=0,j=0,k=0;
	
	array_initd((float*)wpp,atp1.size_waypoint_array*5);
	
	static int flag_temp;
	
	if(flag_temp==1)
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
		wpp[2][2]=-110.0;
		wpp[2][3]=-9999.0;
		wpp[2][4]=atp1.Va0;
		
		wpp[3][0]=1200.0;
		wpp[3][1]=1200.0;
		wpp[3][2]=-150.0;
		wpp[3][3]=-9999,0;
		wpp[3][4]=atp1.Va0;
		
		wpp[4][0]=0.0;
		wpp[4][1]=0.0;
		wpp[4][2]=0.0;
		wpp[4][3]=-9999.0;
		wpp[4][4]=atp1.Va0;
		
		num_waypoints=5; //starting from 0
	}
	else
	{
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
		wpp[2][2]=-110.0;
		wpp[2][3]=(45.0*PI)/180;
		wpp[2][4]=atp1.Va0;
		
		wpp[3][0]=1200.0;
		wpp[3][1]=1200.0;
		wpp[3][2]=-100.0;
		wpp[3][3]=(-135.0*PI)/180;
		wpp[3][4]=atp1.Va0;		
		
		num_waypoints=4; //starting from 0
	}
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

void path_follow(float in[], struct atp atp1, float out[])
{
	float chi_inf = 60.0*PI/180.0;  //approach angle for large distance from straight-line path
    float k_path  = 0.01;        //proportional gain for path following
    float k_orbit = 2.5;       //proportional gain for orbit following
    float gravity = 9.81;

	int NN=0;
	int i=0,j=0;

	//PATH input to path_follow
	int flag  = (int)in[0+NN];
    float Va_d = in[1+NN];
    float r_path[3][1] = {{in[2+NN]},{in[3+NN]},{in[4+NN]}};
    float q_path[3][1] = {{in[5+NN]},{in[6+NN]},{in[7+NN]}};
    float c_orbit[3][1]= {{in[8+NN]},{in[9+NN]},{in[10+NN]}};
    float rho_orbit = in[11+NN];
    float lam_orbit = in[12+NN];
	
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
	
	float t = in[0+NN];
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