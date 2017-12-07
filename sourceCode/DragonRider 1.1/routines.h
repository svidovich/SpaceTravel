#include <iostream>
#include <Eigen/Dense>
#include <math.h>
using namespace std;
using namespace Eigen;

void turnleftfromztox(Vector3d& yawcf, Vector3d currentmomentum, Vector3d& yawdf)
{
	if(currentmomentum(2) > 0)
	{
		yawcf(0) = -1;
	}
	if(currentmomentum(2) < 0)
	{
		yawdf(0) = 1;
	}
	if(currentmomentum(2) == 0)
	{
		yawcf(0) = 0;
		yawdf(0) = 0;
	}
}



// This function will make the ship's Z directional momentum increase.
void goStraightInZ(double& tvrightbfscale, double& tvrightcfscale, double& tvleftefscale, double& tvleftffscale, double& bvrightbfscale, double& bvrightcfscale, double& bvleftefscale, double& bvleftffscale, double force, Vector3d goalmomentum, Vector3d currentmomentum)
{
	if (goalmomentum(2) >= currentmomentum(2))
		{
			tvrightbfscale = force;
			tvrightcfscale = force;
			tvleftefscale = force;
			tvleftffscale = force;
			bvrightbfscale = force;
			bvrightcfscale = force;
			bvleftefscale = force;
			bvleftffscale = force;
		}
	else
		{
			tvrightbfscale = 0;
			tvrightcfscale = 0;
			tvleftefscale = 0;
			tvleftffscale = 0;
			bvrightbfscale = 0;
			bvrightcfscale = 0;
			bvleftefscale = 0;
			bvleftffscale = 0;
		}
}

// This function will make the ship drag its Z directional momentum to 0.
void zstop(double& tvrightafscale, double& tvleftdfscale, double& bvrightafscale, double& bvleftdfscale, double brakef, Vector3d currentmomentum)
{
	if (currentmomentum(2) > 0)
	{
		tvrightafscale = brakef;
		bvrightafscale = brakef;
		tvleftdfscale = brakef;
		bvleftdfscale = brakef;
	}
	if (currentmomentum(2) <= 0)
	{
		
		tvrightafscale = 0;
		bvrightafscale = 0;
		tvleftdfscale = 0;
		bvleftdfscale = 0;

	}

}
