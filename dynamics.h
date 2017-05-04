#ifndef DYNAMICS
#define DYNAMICS


#include "SystemParameter.h"

class dynamics
{
public:
	void GravityComp(float Gcomp[6] ,float angle[6])
	{
		float s0 = sin( angle[0] );
		float s1 = sin( angle[1] );
		float s2 = sin( angle[2] );
		float s3 = sin( angle[3] );
		float s4 = sin( angle[4] );
		float s5 = sin( angle[5] );
		float c0 = cos( angle[0] );
		float c1 = cos( angle[1] );
		float c2 = cos( angle[2] );
		float c3 = cos( angle[3] );
		float c4 = cos( angle[4] );
		float c5 = cos( angle[5] );

		float r1 = L1/5.0;
		float r2 = L2/3.0;

		//printf("%f\n",joint_pos[1][0] );

		Gcomp[0] = Mg1*c1*s0*r1 + Mg2*(0.3*c1*s0 - r2*(s3*(c0*c2 - s0*s1*s2) - c1*c3*s0));
		Gcomp[1] = Mg1*c0*s1*r1 + Mg2*(0.3*c0*s1 + r2*(c0*c3*s1 - c0*c1*s2*s3));
		Gcomp[2] = r2*Mg2*s3*(s0*s2 - c0*c2*s1);
		Gcomp[3] = -r2*Mg2*(c3*(c2*s0 + c0*s1*s2) - c0*c1*s3);
	}

};


#endif