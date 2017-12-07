#include <iostream>
#include <Eigen/Dense>
#include <math.h>

/******************************************************************************
* so3Isomorphism is a linear map from so(3) to R3 which commutes with
* the respective Lie brackets (matrix bracket and cross-product)
******************************************************************************/
Eigen::Vector3d so3Isomorphism(const Eigen::Matrix3d A);

/******************************************************************************
* so3IsomorphismInverse is a linear map from R3 to so(3) which is the
* inverse of so3Isomorphism
******************************************************************************/
Eigen::Matrix3d so3IsomorphismInverse(const Eigen::Vector3d a);

/******************************************************************************
* Uses the Rodrigues formula to exponentiate A in so(3) to Q in SO(3)
******************************************************************************/
Eigen::Matrix3d ExpSO3(const Eigen::Matrix3d A);

/******************************************************************************
* Uses the Rodrigues formula to exponentiate a in R3 to Q in SO(3)
******************************************************************************/
Eigen::Matrix3d ExpSO3(const Eigen::Vector3d a);

/******************************************************************************/
/* Solves the equation P = QJ - JQ^T for Q in SO(3) using a modified Newton   */
/* method.								      */
/******************************************************************************/
Eigen::Matrix3d MoserVeselov(const Eigen::Matrix3d P, const Eigen::Matrix3d J, const int iterate_bound, const double tol);

