#include <iostream>
#include <Eigen/Dense>
#include <math.h>
using namespace std;
using namespace Eigen;

/******************************************************************************
* so3Isomorphism is a linear map from so(3) to R3 which commutes with
* the respective Lie brackets (matrix bracket and cross-product)
******************************************************************************/
Vector3d so3Isomorphism(const Matrix3d A)
{
    Vector3d a;
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
