#include <iostream>
#include <Eigen/Dense>
#include <math.h>
#include <fstream>
#include <string>
#include <cstdlib>
#include "hires_timer-1.c"
const double pi = 3.1415926536;
using namespace Eigen;
using namespace std;

ofstream fout;

/******************************************************************************
* so3Isomorphism is a linear map from so(3) to R3 which commutes with
* the respective Lie brackets (matrix bracket and cross-product)
******************************************************************************/
Vector3d so3Isomorphism(const Matrix3d A)
{
    Vector3d a;
//    cout << endl << "Sending vector a from so(3) to R3...\n"; 
    a << -A.array()(1,2) , A.array()(0,2) , -A.array()(0,1);
    return a;
}

/******************************************************************************
* so3IsomorphismInverse is a linear map from R3 to so(3) which is the
* inverse of so3Isomorphism
******************************************************************************/
Matrix3d so3IsomorphismInverse(const Vector3d a)
{
    Matrix3d A;
//    cout << endl << "Sending 3x3 matrix A from R3 to so(3)...\n";
    A <<            0, -a.array()(2),  a.array()(1),
         a.array()(2),             0, -a.array()(0),
        -a.array()(1),  a.array()(0),             0;
    return A;
}


/******************************************************************************
* Uses the Rodrigues formula to exponentiate A in so(3) to Q in SO(3)
******************************************************************************/
Matrix3d ExpSO3(const Matrix3d A)
{
    double sigma;
    Matrix3d E = Matrix3d::Identity();
    
    sigma = A.norm()/sqrt(2);
    
    E += (sin(sigma) / sigma) * A + ((1.0 - cos(sigma))/(sigma*sigma))*A*A;
    return E;
}

/******************************************************************************
* Uses the Rodrigues formula to exponentiate a in R3 to Q in SO(3)
******************************************************************************/
Matrix3d ExpSO3(const Vector3d a)
{
    double sigma;
    Matrix3d E = Matrix3d::Identity();
    Matrix3d A = so3IsomorphismInverse(a);
    
    sigma = a.norm();
    
    E += (sin(sigma) / sigma) * A + ((1.0 - cos(sigma))/(sigma*sigma))*A*A;
    return E;
}

/******************************************************************************
* Solves the equation P = QJ - JQ^T for Q in SO(3) using a modified Newton
* method.
*
* Initial guess is Q = Id
* Matrix J is assumed symmetric positive definite
* Matrix P is assumed skew-symmetric
* If iterate_bound is reached before Newton's method converges, most recent
*    estimate is returned
* tol is tolerance for residual as measured by Frobenius norm
******************************************************************************/
Matrix3d MoserVeselov(const Matrix3d P, const Matrix3d J, const int iterate_bound, const double tol)
{
    int i;

    Matrix3d Q;
    Vector3d a;
    Vector3d p = so3Isomorphism(P);
    Matrix3d X;
    Matrix3d Id = Matrix3d::Identity();

    double residual;

    //Initial guess is Q = Id
    Q = Id;
    cout << endl << "Solving Moser-Veselov equation...\n";
    //Check initial residual
    residual = (Q*J - J*Q.transpose() - P).norm();
    //cout << "Initial guess for Q is identity.\n";
    i = 0;
    while(i < iterate_bound && residual > tol)
    {
        X = J*Q;        
        a = (X.trace()*Id - X).inverse() *(Q.transpose() * p + so3Isomorphism(X.transpose() - X));

        Q = Q*ExpSO3(a);
        residual = (Q*J - J*Q.transpose() - P).norm();
        i++;
    }
    //cout << "Final Q:\n" << Q << endl;
    return Q;
}

int main()
{
    int i;
    string fname;

    double ts, tf, n, h;
    double tol = 0.000000001;

    Matrix3d J;
    Matrix3d P;
    Matrix3d Q = Matrix3d::Identity();
    Matrix3d F;
    Matrix3d A;
    Matrix3d I = Matrix3d::Identity();
    Matrix3d E;

    cout << "Welcome to matrix rotation v0.0! We will rotate your cube.\n";
    cout << "Enter a filename for output (include a file extension!): ";
    getline(cin, fname);
    fout.open(fname.c_str());
    if (fout.fail())
    {
	cout << "Opening output file failed. Shutting down...\n";
        exit(1);
    }
    cout << "Error tolerance is set to " << tol << ".\n";
    cout << "Please enter the step size: ";
    cin >> n;
    cout << "You chose a step size of " << n << ". Calculating...\n";

    /* A few Definitions. J is calculated with a cube. h is 2pi/n. */
    J << 0.5, 0, 0,
	 0,  0.5, 0,
	 0, 0,  0.5;

    P << 0, -1, 0,
	 1, 0, 0,
	 0, 0, 0;

    h = (2*pi)/n;

    /* Starts the timer */   
    init_hires_timer();
    hires_timer(&ts);

    /* The for loop that solves world hunger */

    cout << "Initial guess for Q is identity.\n";
    //fout << "Initial guess for Q is identity.\n";
    for (i=0; i<n; i++)
    {
	F = MoserVeselov(P*h, J, 10, tol);
	P = (F.transpose()) * P * F;
	Q = Q*F;
	//fout << "Moser Veselov solve #" << i << "/" << n << endl;
	//cout << "Moser Veselov solve #" << i << "/" << n << endl;
	//fout << "Q = \n";
	//fout << Q << endl;
	fout << Q(0,0) << "," << Q(0,1) << "," << Q(0,2) << "," << endl;
	fout << Q(1,0) << "," << Q(1,1) << "," << Q(1,2) << "," << endl;
	fout << Q(2,0) << "," << Q(2,1) << "," << Q(2,2) << "," << endl;
    }
    cout << "Final Q:\n" << Q << endl;
    E = (I-Q);
    double error;
    error = E.norm();
   

    //fout << endl << "Final Error: " << error << endl;
    hires_timer(&tf);
    cout << "Calculation time: " << tf - ts << " seconds." << endl;
    fout.close();
    return 0;
}
