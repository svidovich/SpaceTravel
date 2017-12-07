#include <iostream>
#include <Eigen/Dense>
#include <math.h>
#include <fstream>
#include <string>
#include <cstdlib>
#include "hires_timer-1.c"
#include "functions.h"
#include "robsmath.h"
const double pi = 3.1415926536;
using namespace Eigen;
using namespace std;


// Solver function. Perhaps there's a better way of doing this, but who cares?
void solver(Eigen::Matrix3d& F, Eigen::Matrix3d& P, Eigen::Matrix3d& J, Eigen::Matrix3d& Q, Eigen::Vector3d f, Eigen::Vector3d& p, Eigen::Vector3d& x, double h, double m, double tol, int n);

ofstream fout, xout; // These are global, be careful.

int main()
{
	/* Initializations Begin here */

	int i;
	double ts, tf, h, m, error; // Timer Start | Timer Finish | Resolution | Mass | Error of Calculation
	int n;
	double tol = 0.000000001; // Low Tolerance for low error...!
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
	Vector3d p(0,1,0); // Initial momentum vector
	Vector3d f(0,200000,0); // Force vector

	/* Initializations end here */


	thinkofthecustomer(fout,xout,n,m,tol);
	h = 10.0/n; // be careful. this has been changed.
	JDEF(J); // Function for defining the inertia matrix.

	/* Starts the timer */   
	init_hires_timer();
	hires_timer(&ts);

	// Solve the MV equation, decide new position.
	solver(F,P,J,Q,f,p,x,h,m,tol,n); // amazing. Future me is going to punch me in the head, I'm sure.

	cout << "Final Q:\n" << Q << endl;
	cout << "Final x:\n" << x << endl;
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
void solver(Matrix3d& F, Matrix3d& P, Matrix3d& J, Matrix3d& Q, Vector3d f, Vector3d& p, Vector3d& x, double h, double m, double tol, int n)
{

    /* The for loop that solves world hunger */

    cout << "Solving Moser Veselov Equation...\n";
    for (int i=0; i<n; i++)
    {
	F = MoserVeselov(P*h, J, 10, tol);
	P = (F.transpose()) * P * F;
	Q = Q*F;

	p = p + f*h;
        x = x + (h/m)*p + (pow(h,2)/m)*f;
	
	
        // Output the matrices to file
	fout << Q(0,0) << "," << Q(0,1) << "," << Q(0,2) << "," << endl;
	fout << Q(1,0) << "," << Q(1,1) << "," << Q(1,2) << "," << endl;
	fout << Q(2,0) << "," << Q(2,1) << "," << Q(2,2) << "," << endl;
        // Output position vectors to file
        xout << x(0) << "," << x(1) << "," << x(2) << endl;
    }

}




