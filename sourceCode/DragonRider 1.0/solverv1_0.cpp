#include <iostream>
#include <Eigen/Dense>
#include <math.h>
#include <fstream>
#include <string>
#include <cstdlib>
#include "hires_timer-1.c"
#include "functions.h"
#include "robsmath.h"
#include "routines.h"
#include <vector>
const double pi = 3.1415926536;
using namespace Eigen;
using namespace std;


// Solver function. Perhaps there's a better way of doing this, but who cares?
void solver(Matrix3d& F, Matrix3d& P, Matrix3d& J, Matrix3d& Q, Vector3d& p, Vector3d& x, double h, double m, double tol, int n);

ofstream fout, xout; // These are global, be careful.

int main()
{
	/* Initializations Begin here */

	int i;
	double ts, tf, h, m, error; // Timer Start | Timer Finish | Resolution | Mass | Error of Calculation
	int n;
	double tol = 0.0001; // Low Tolerance for low error...!
	// Initialize the matrices we need to do work on rotation
	Matrix3d J; // Inertia
	Matrix3d P; // Initialize skew-symmetric P
	P << 0, 0, 0,
	0, 0, 0,
	 0, 0, 0; // Currently no initial angular momentum.
	Matrix3d Q = Matrix3d::Identity(); // Rotation
	Matrix3d F;
	Matrix3d A;
	Matrix3d I = Matrix3d::Identity(); // Identity, for good measure
	Matrix3d E;
	// Initialize some vectors for the position work
	Vector3d x(0,0,0); // Initial position vector for body frame connection
	Vector3d p(0,0,0); // Initial momentum vector
	/* Initializations end here */


	thinkofthecustomer(fout,xout,n,m,tol);

	h = 720.0/n; // Numerator is flight duration in seconds.
	J << 8662.0, 0, 0,
		0, 8662.0, 0,
			0, 0, 49062.0;

	/* Starts the timer */   
	init_hires_timer();
	hires_timer(&ts);
	// Solve the MV equation, decide new position.
	solver(F,P,J,Q,p,x,h,m,tol,n); // amazing. Future me is going to punch me in the head, I'm sure.

	cout << "Final Q:\n" << Q << endl;
	cout << "Final x:\n" << x << endl;
	cout << "Final p:\n" << p << endl;
	E = (I-Q);
	error = E.norm();
   

	cout << endl << "Final Error: " << error << endl;
	hires_timer(&tf);
	cout << "Calculation time: " << tf - ts << " seconds." << endl;

	fout.close();
	xout.close();


	return 0;
}


// Solver function. Gives all of the new rotations, position, etc.
void solver(Matrix3d& F, Matrix3d& P, Matrix3d& J, Matrix3d& Q, Vector3d& p, Vector3d& x, double h, double m, double tol, int n)
{

	/* Initialization of SuperDraco Modules */

	// Initialize the force vector for the superdraco booster at position 0
	Vector3d fsuperd0(0,0,140);
	// Initialize beta for the superdraco booster at position 0
	Vector3d bsuper0(1.85,0,.8659);
	// Initialize the torque vector for the superdraco booster at position 0
	Vector3d tsuper0(0,0,0);


	// Initialize the force vector for the superdraco booster at position pi/2
	Vector3d fsuperdpi2(0,0,0);
	// Initialize beta for the superdraco booster at position pi/2
	Vector3d bsuperpi2(0,-1.85,.8659);
	// Initialize the torque vector for the superdraco booster at position pi/2
	Vector3d tsuperpi2(0,0,0);

	// Initialize the force vector for the superdraco booster at position pi
	Vector3d fsuperdpi(0,0,140);
	// Initialize beta for the superdraco booster at position pi
	Vector3d bsuperpi(-1.85,0,.8659);
	// Initialize the torque vector for the superdraco booster at position pi
	Vector3d tsuperpi(0,0,0);
	// Initialize the force vector for the superdraco booster at position 3pi/2

	Vector3d fsuperd3pi2(0,0,0);
	// Initialize beta for the superdraco booster at position 3pi/2
	Vector3d bsuper3pi2(0,1.85,.8659);
	// Initialize the torque vector for the superdraco booster at position 3pi/2
	Vector3d tsuper3pi2(0,0,0);

	// Get the sum of the forces
	Vector3d fsum(0,0,0);
	fsum  = fsuperd0 + fsuperdpi2 + fsuperdpi + fsuperd3pi2;


	/* Initialization of MiniDracos */


//	#######################################################################
				/* Top View MiniDraco Pods */
//	#######################################################################

	// Right hand top view minidraco pod
	Vector3d tvrt(0,0,0); // Total torque vector initialization
	// Points in the negative z direction. Unit vector for force is <0,0,-1>
	// This booster is part of braking.
	double tvrightafscale = 0;
	Vector3d tvrightaf(0,0,-2*tvrightafscale);
	Vector3d tvrightab(1.138,1.138,2.2383);
	Vector3d tvrightat(0,0,0);

	// Points in the positive z and negative x directions. Unit vector for force is <-sqrt(2)/2,0,sqrt(2)/2>
	// This booster is part of acceleration. It lies at 135 degrees from horizontal.
	double tvrightbfscale = 0;
	Vector3d tvrightbf(-tvrightbfscale*.707107,0,tvrightbfscale*.707107);
	Vector3d tvrightbb(1.138,1.138,2.2383);
	Vector3d tvrightbt(0,0,0);

	// Points in the positive z and positive x directions. Unit vector for force is <.259,0,.966>
	// This booster is part of acceleration. It lies at 75 degrees from horizontal.
	double tvrightcfscale = 0;
	Vector3d tvrightcf(tvrightcfscale*.259,0,tvrightcfscale*.966);
	Vector3d tvrightcb(1.138,1.138,2.2383);
	Vector3d tvrightct(0,0,0);

	// Left hand top view minidraco pod
	Vector3d tvlt(0,0,0); 
	// Points in the negative z direction. Unit vector for force is <0,0,-1>
	// This booster is part of braking.
	double tvleftdfscale = 0;
	Vector3d tvleftdf(0,0,-2*tvleftdfscale);
	Vector3d tvleftdb(-1.138,1.138,1.3724);
	Vector3d tvleftdt(0,0,0);

	// Points in the positive z and positive x directions. Unit vector for force is <sqrt(2)/2,0,sqrt(2)/2>
	// This booster is part of acceleration. It lies at 45 degrees from horizontal.
	double tvleftefscale = 0;
	Vector3d tvleftef(tvleftefscale*.707107,0,tvleftefscale*.707107);
	Vector3d tvlefteb(-1.138,1.138,1.3724);
	Vector3d tvleftet(0,0,0);

	// Points in the positive z and negative x directions. Unit vector for force is <-.259,0,.966>
	// This booster is part of acceleration. It lies at 105 degrees from horizontal.
	double tvleftffscale = 0;
	Vector3d tvleftff(-tvleftffscale*.259,0,tvleftffscale*.966);
	Vector3d tvleftfb(-1.138,1.138,1.3724);
	Vector3d tvleftft(0,0,0);

//	##################################################################################
				/* Bottom View MiniDraco Pods */
//	##################################################################################

	// Right hand bottom view minidraco pod
	Vector3d bvrt(0,0,0);
	// Points in the negative z direction. Unit vector for force is <0,0,-1>
	// This booster is part of braking.
	double bvrightafscale = 0;
	Vector3d bvrightaf(0,0,-2*bvrightafscale);
	Vector3d bvrightab(-1.138,-1.138,2.2383);
	Vector3d bvrightat(0,0,0);

	// Points in the positive z and positive x directions. Unit vector for force is <sqrt(2)/2,0,sqrt(2)/2>
	// This booster is part of acceleration. It lies at 45 degrees from horizontal.
	double bvrightbfscale = 0;
	Vector3d bvrightbf(bvrightbfscale*.707107,0,bvrightbfscale*.707107);
	Vector3d bvrightbb(-1.138,-1.138,2.2383);
	Vector3d bvrightbt(0,0,0);

	// Points in the positive z and negative x directions. Unit vector for force is <-.259,0,.966>
	// This booster is part of acceleration. It lies at 105 degrees from horizontal.
	double bvrightcfscale = 0;
	Vector3d bvrightcf(bvrightcfscale*-.259,0,bvrightcfscale*.966);
	Vector3d bvrightcb(-1.138,-1.138,2.2383);
	Vector3d bvrightct(0,0,0);

	// Left hand bottom view minidraco pod
	Vector3d bvlt(0,0,0);
	// Points in the negative z direction. Unit vectore for force is <0,0,-1>
	// This booster is part of braking.
	double bvleftdfscale = 0;
	Vector3d bvleftdf(0,0,-2*bvleftdfscale);
	Vector3d bvleftdb(1.138,-1.138,2.2383);
	Vector3d bvleftdt(0,0,0);

	// Points in the positive z and negative x directions. Unit vectore for force is <-sqrt(2)/2,0,sqrt(2)/2>
	// This booster is part of acceleration. It lies at 135 degrees from horizontal.
	double bvleftefscale = 0;
	Vector3d bvleftef(bvleftefscale*-.707107,0,bvleftefscale*.707107);
	Vector3d bvlefteb(1.138,-1.138,2.2383);
	Vector3d bvleftet(0,0,0);

	// Points in the positive z and positive x directions. Unit vectore for force is <.259,0,.966>
	// This booster is part of acceleration. It lies at 75 degrees from horizontal.
	double bvleftffscale = 0;
	Vector3d bvleftff(bvleftffscale*.259,0,bvleftffscale*.966);
	Vector3d bvleftfb(1.138,-1.138,2.2383);
	Vector3d bvleftft(0,0,0);
//	######################################################################################
//					Yaw and Pitch Mini Dracos
//	######################################################################################
	// Torque totals for yaw and pitch vectors.
	Vector3d yawt(0,0,0);
	Vector3d pitcht(0,0,0);

	// Pitch controller, rotation from z axis toward positive y.
	double paf = 0;
	Vector3d pitchaf(0,paf,0);
	Vector3d pitchab(0,1.07,3.5935);
	Vector3d pitchat(0,0,0);	

	// Pitch controller, rotation from z axis toward negative y.
	double pbf = 0;
	Vector3d pitchbf(0,-pbf,0);
	Vector3d pitchbb(0,-1.07,3.5925);
	Vector3d pitchbt(0,0,0);

	// Yaw controller, rotation from z axis toward negative x.
	double ycf = 0;
	Vector3d yawcf(-ycf,0,0);
	Vector3d yawcb(-1.07,0,3.5935);
	Vector3d yawct(0,0,0);

	// Yaw controller, rotation from z axis toward positive x.
	double ydf = 0;
	Vector3d yawdf(ydf,0,0);
	Vector3d yawdb(1.07,0,3.5935);
	Vector3d yawdt(0,0,0);


//	#####################################################################################
//       			Big Matrix F: 3x16, Magnitudes Vector U, 16x1
//	#####################################################################################

	Matrix<double,3,16> BigF; // This statement declares the big matrix itself.
	

//	######################################################################################
//					End MiniDracos
//	######################################################################################


	// Initialize the torque sum vector
	Vector3d t(0,0,0);
	// Initialize torque matrix
	Matrix3d T;

	Vector3d goalmomentum(0,0, 100);

    cout << "Solving Moser Veselov Equation...\n";
    for (int i=0; i<n; i++)
    {


	// Calculate each torque

/*	########### SuperDraco Torques ###########	*/
	tsuper0 = bsuper0.cross(fsuperd0);
	tsuperpi2 = bsuperpi2.cross(fsuperdpi2);
	tsuperpi = bsuperpi.cross(fsuperdpi);
	tsuper3pi2 = bsuper3pi2.cross(fsuperd3pi2);

/*	########### MiniDraco Torques ###########	*/

	// Top right
	tvrightat = tvrightab.cross(tvrightaf);
	tvrightbt = tvrightbb.cross(tvrightbf);
	tvrightct = tvrightcb.cross(tvrightcf);
	tvrt = tvrightat + tvrightbt + tvrightct;

	// Bottom right
	bvrightat = bvrightab.cross(bvrightaf);
	bvrightbt = bvrightbb.cross(bvrightbf);
	bvrightct = bvrightcb.cross(bvrightcf);
	bvrt = bvrightat + bvrightbt + bvrightct;

	// Top left
	tvleftdt = tvleftdb.cross(tvleftdf);
	tvleftet = tvlefteb.cross(tvleftef);
	tvleftft = tvleftfb.cross(tvleftff);
	tvlt = tvleftdt + tvleftet + tvleftft;

	// Bottom left
	bvleftdt = bvleftdb.cross(bvleftdf);
	bvleftet = bvlefteb.cross(bvleftef);
	bvleftft = bvleftfb.cross(bvleftff);
	bvlt = bvleftdt + bvleftet + bvleftft;

	// Pitch
	pitchat = pitchab.cross(pitchaf);
	pitchbt = pitchbb.cross(pitchbf);
	pitcht = pitchat + pitchbt;

	// Yaw
	yawct = yawcb.cross(yawcf);
	yawdt = yawdb.cross(yawdf);
	yawt = yawct + yawdt;





	if ( h*i < 180 ) // Accelerate the whole time
	{
goStraightInZ(tvrightbfscale, tvrightcfscale, tvleftefscale, tvleftffscale, bvrightbfscale, bvrightcfscale, bvleftefscale, bvleftffscale, 1, goalmomentum, p);
	}
	if ( h*i < 45) // 0-45, yaw
	{
		yawcf << -1,0,0;
	}
	if ( h*i >= 45 && h*i < 135) // 45-90, oppose original yaw
	{
		yawcf << 0,0,0;
		yawdf << 1,0,0;
	}
//	if ( h*i >= 90 && h*i < 135) // 90-135, original yaw
//	{
//		yawdf << 0,0,0;
//		yawcf << 1,0,0;
//	}
	if ( h*i >= 135 && h*i < 180) // 135-180, oppose original yaw
	{
		yawcf << -1,0,0;
		yawdf << 0,0,0;
	}
	if ( h*i >= 180 )
	{
		yawcf << 0,0,0;
	}
	/* ######################################################### */
	/* This is where you should put the routines you want to run */
	/* ######################################################### */




	// This piece will make the ship run with a constant momentum of 100.	
//	if (topmomentumgoal(2) < 100)
//	{
//		goStraightInZ(tvrightbfscale, tvrightcfscale, tvleftefscale, tvleftffscale, bvrightbfscale, bvrightcfscale, bvleftefscale, bvleftffscale, 40, topmomentumgoal, p);
//	}
	// This piece will (hopefully) make the ship turn from z to negative x.
//	if (topmomentumgoal(2) >= 100)
//	{
//		turnleftfromztox(yawcf, p, yawdf);
//	}

	/* ################################# */
	/* These are the state calculations. */
	/* ################################# */

	t = tsuper0 + tsuperpi2 + tsuperpi + tsuper3pi2 + tvrt + bvrt + tvlt + bvlt + yawt + pitcht;
	T = so3IsomorphismInverse(t);

	F = MoserVeselov(P*h, J, 10, tol);
	P = (F.transpose())*P*F + h*T;
	Q = Q*F;
	
	p = p + h*Q*fsum;
        x = x + (h/m)*p;
		
	
        // Output the matrices to file
	fout << Q(0,0) << "," << Q(0,1) << "," << Q(0,2) << "," << endl;
	fout << Q(1,0) << "," << Q(1,1) << "," << Q(1,2) << "," << endl;
	fout << Q(2,0) << "," << Q(2,1) << "," << Q(2,2) << "," << endl;
        // Output position vectors to file
        xout << x(0) << "," << x(1) << "," << x(2) << endl;
    }

}




