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
	Vector3d P; // Initialize vector P
	P << 0, 0, 0; // Currently no initial angular momentum.
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

	/* Initialization of MiniDracos */


//	#######################################################################
				/* Top View MiniDraco Pods */
//	#######################################################################

	// Right hand top view minidraco pod
	Vector3d tvrt(0,0,0); // Total torque vector initialization
	// Points in the negative z direction. Unit vector for force is <0,0,-1>
	// This booster is part of braking.
	Vector3d tvrightaf(0,0,-2);
	Vector3d tvrightab(1.138,1.138,2.2383);
	Vector3d tvrightat(0,0,0);

	// Points in the positive z and negative x directions. Unit vector for force is <-sqrt(2)/2,0,sqrt(2)/2>
	// This booster is part of acceleration. It lies at 135 degrees from horizontal.
	Vector3d tvrightbf(-.707107,0,.707107);
	Vector3d tvrightbb(1.138,1.138,2.2383);
	Vector3d tvrightbt(0,0,0);

	// Points in the positive z and positive x directions. Unit vector for force is <.259,0,.966>
	// This booster is part of acceleration. It lies at 75 degrees from horizontal.
	Vector3d tvrightcf(.259,0,.966);
	Vector3d tvrightcb(1.138,1.138,2.2383);
	Vector3d tvrightct(0,0,0);

	// Left hand top view minidraco pod
	Vector3d tvlt(0,0,0); 
	// Points in the negative z direction. Unit vector for force is <0,0,-1>
	// This booster is part of braking.
	Vector3d tvleftdf(0,0,-2);
	Vector3d tvleftdb(-1.138,1.138,2.2383);
	Vector3d tvleftdt(0,0,0);

	// Points in the positive z and positive x directions. Unit vector for force is <sqrt(2)/2,0,sqrt(2)/2>
	// This booster is part of acceleration. It lies at 45 degrees from horizontal.
	Vector3d tvleftef(.707107,0,.707107);
	Vector3d tvlefteb(-1.138,1.138,2.2383);
	Vector3d tvleftet(0,0,0);

	// Points in the positive z and negative x directions. Unit vector for force is <-.259,0,.966>
	// This booster is part of acceleration. It lies at 105 degrees from horizontal.
	Vector3d tvleftff(-.259,0,.966);
	Vector3d tvleftfb(-1.138,1.138,2.383);
	Vector3d tvleftft(0,0,0);

//	##################################################################################
				/* Bottom View MiniDraco Pods */
//	##################################################################################

	// Right hand bottom view minidraco pod
	Vector3d bvrt(0,0,0);
	// Points in the negative z direction. Unit vector for force is <0,0,-1>
	// This booster is part of braking.
	Vector3d bvrightaf(0,0,-2);
	Vector3d bvrightab(-1.138,-1.138,2.2383);
	Vector3d bvrightat(0,0,0);

	// Points in the positive z and positive x directions. Unit vector for force is <sqrt(2)/2,0,sqrt(2)/2>
	// This booster is part of acceleration. It lies at 45 degrees from horizontal.
	Vector3d bvrightbf(.707107,0,.707107);
	Vector3d bvrightbb(-1.138,-1.138,2.2383);
	Vector3d bvrightbt(0,0,0);

	// Points in the positive z and negative x directions. Unit vector for force is <-.259,0,.966>
	// This booster is part of acceleration. It lies at 105 degrees from horizontal.
	Vector3d bvrightcf(-.259,0,.966);
	Vector3d bvrightcb(-1.138,-1.138,2.2383);
	Vector3d bvrightct(0,0,0);

	// Left hand bottom view minidraco pod
	Vector3d bvlt(0,0,0);
	// Points in the negative z direction. Unit vectore for force is <0,0,-1>
	// This booster is part of braking.
	Vector3d bvleftdf(0,0,-2);
	Vector3d bvleftdb(1.138,-1.138,2.2383);
	Vector3d bvleftdt(0,0,0);

	// Points in the positive z and negative x directions. Unit vectore for force is <-sqrt(2)/2,0,sqrt(2)/2>
	// This booster is part of acceleration. It lies at 135 degrees from horizontal.
	Vector3d bvleftef(-.707107,0,.707107);
	Vector3d bvlefteb(1.138,-1.138,2.2383);
	Vector3d bvleftet(0,0,0);

	// Points in the positive z and positive x directions. Unit vectore for force is <.259,0,.966>
	// This booster is part of acceleration. It lies at 75 degrees from horizontal.
	Vector3d bvleftff(.259,0,.966);
	Vector3d bvleftfb(1.138,-1.138,2.2383);
	Vector3d bvleftft(0,0,0);
//	######################################################################################
//					Yaw and Pitch Mini Dracos
//	######################################################################################
	// Torque totals for yaw and pitch vectors.
	Vector3d yawt(0,0,0);
	Vector3d pitcht(0,0,0);

	// Pitch controller, rotation from z axis toward positive y.
	Vector3d pitchaf(0,1,0);
	Vector3d pitchab(0,1.07,3.5935);
	Vector3d pitchat(0,0,0);	

	// Pitch controller, rotation from z axis toward negative y.
	Vector3d pitchbf(0,-1,0);
	Vector3d pitchbb(0,-1.07,3.5935);
	Vector3d pitchbt(0,0,0);

	// Yaw controller, rotation from z axis toward negative x.
	Vector3d yawcf(-1,0,0);
	Vector3d yawcb(-1.07,0,3.5935);
	Vector3d yawct(0,0,0);

	// Yaw controller, rotation from z axis toward positive x.
	Vector3d yawdf(1,0,0);
	Vector3d yawdb(1.07,0,3.5935);
	Vector3d yawdt(0,0,0);


//	#####################################################################################
//       			Big Matrix Xi: 3x16, Magnitudes Vector U, 16x1
//	#####################################################################################

	Matrix<double,3,16> Xi; // This statement declares the big matrix itself.
	// Each column of Xi is one of the force vectors which control mini draco boost. 
	Xi.col(0) = tvrightaf; // ref: 128
	Xi.col(1) = tvrightbf; // ref: 135
	Xi.col(2) = tvrightcf; // ref: 142
	Xi.col(3) = tvleftdf; // ref: 151
	Xi.col(4) = tvleftef; // ref: 158
	Xi.col(5) = tvleftff; // ref: 165
	Xi.col(6) = bvrightaf; // ref: 178
	Xi.col(7) = bvrightbf; // ref: 185
	Xi.col(8) = bvrightcf; // ref:  192
	Xi.col(9) = bvleftdf; // ref: 201
	Xi.col(10) = bvleftef; // ref: 208
	Xi.col(11) = bvleftff; // ref: 215
	Xi.col(12) = pitchaf; // ref: 228
	Xi.col(13) = pitchbf; // ref: 234
	Xi.col(14) = yawcf; // ref: 240
	Xi.col(15) = yawdf; // ref: 246

//	Declare timestep sized instruction list U. It is initialized to zero.

	Matrix<double,16,1> UROLL;
	UROLL.setZero();
	UROLL(0)=.353553;
	UROLL(1)=1;
	UROLL(6)=.353553;
	UROLL(7)=1;
	UROLL = UROLL * 643.75;

	Matrix<double,16,1> upitch;
	upitch.setZero();
	upitch(0)=0.6125;
	upitch(2)=0.789877;
	upitch(3)=0.302743;
	upitch(4)=0.625589;	
	upitch(5)=1;
	upitch(6)=0.364415;
	upitch(7)=0.548625;
	upitch(14)=0.775873;
	
	Matrix<double,16,1> uxplus;
	uxplus.setZero();
	uxplus(2)=1;
	uxplus(3)=0.516298;
	uxplus(4)=0.0941802;
	uxplus(7)=0.460461;
	uxplus(9)=0.162798;


	Matrix<double,16,1> uyaw;
	uyaw.setZero();
	uyaw(0)=0.6125;
	uyaw(4)=0.366281;
	uyaw(5)=1;

	Matrix<double,16,100> U;
	U.setZero();
	for (int j = 0; j <= 100; j++)
	{
		U.col(i) = 400*uxplus;
	}
	
//	######################################################################################
//					End MiniDracos
//	######################################################################################

	// Calculate each torque

/*	########### MiniDraco Torques ###########	*/

	// Top right
	tvrightat = tvrightab.cross(tvrightaf);
	tvrightbt = tvrightbb.cross(tvrightbf);
	tvrightct = tvrightcb.cross(tvrightcf);
	tvrt = tvrightat + tvrightbt + tvrightct;

	// Top left
	tvleftdt = tvleftdb.cross(tvleftdf);
	tvleftet = tvlefteb.cross(tvleftef);
	tvleftft = tvleftfb.cross(tvleftff);
	tvlt = tvleftdt + tvleftet + tvleftft;

	// Bottom right
	bvrightat = bvrightab.cross(bvrightaf);
	bvrightbt = bvrightbb.cross(bvrightbf);
	bvrightct = bvrightcb.cross(bvrightcf);
	bvrt = bvrightat + bvrightbt + bvrightct;

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

//	################################################################
//			Declaration of torque matrix big T
//	################################################################

	// Big T, like Big F, begins as a 3x16 matrix. It gets multiplied
	// by u, making a 3-vector, then mapped to so(3)

	Matrix<double,3,16> BigT;

	BigT.col(0) = tvrightat;
	BigT.col(1) = tvrightbt; 
	BigT.col(2) = tvrightct; 
	BigT.col(3) = tvleftdt; 
	BigT.col(4) = tvleftet; 
	BigT.col(5) = tvleftft; 

	BigT.col(6) = bvrightat;
	BigT.col(7) = bvrightbt; 
	BigT.col(8) = bvrightct; 
	BigT.col(9) = bvleftdt; 
	BigT.col(10) = bvleftet; 
	BigT.col(11) = bvleftft; 

	BigT.col(12) = pitchat; 
	BigT.col(13) = pitchbt; 
	BigT.col(14) = yawct; 
	BigT.col(15) = yawdt; 
	

    cout << "Solving Moser Veselov Equation...\n";
    for (int i=0; i<n; i++)
    {

	/* ################################# */
	/* These are the state calculations. */
	/* ################################# */

        x = x + (h/m)*p;
	p = p + h*Q*Xi*U.col(i);
	F = MoserVeselov(Skew(P)*h, J, 10, tol);
	Q = Q*F;
	P = (F.transpose())*P + h*BigT*U.col(i);

	
        // Output the matrices to file
	fout << Q(0,0) << "," << Q(0,1) << "," << Q(0,2) << "," << endl;
	fout << Q(1,0) << "," << Q(1,1) << "," << Q(1,2) << "," << endl;
	fout << Q(2,0) << "," << Q(2,1) << "," << Q(2,2) << "," << endl;
        // Output position vectors to file
        xout << x(0) << "," << x(1) << "," << x(2) << endl;
	}

	cout << "Xi:\n";
	cout << Xi << endl;
	cout << "BigT:\n";
	cout << BigT << endl;
	hires_timer(&tf);
	cout << "Calculation time: " << tf - ts << " seconds." << endl;

	cout << "Final Q:\n" << Q << endl;
	cout << "Final x:\n" << x << endl;
	cout << "Final p:\n" << p << endl;

	fout.close();
	xout.close();
	return 0;


}




