/**/
#include <fstream>
#include <iostream>
#include <string>

// Opens output streams, queries user for some information
void thinkofthecustomer(std::ofstream& fout, std::ofstream& xout, int& n, double& m, double tol);

void JDEF(Eigen::Matrix3d& J); // Function wrapper for definition of J

void JINPUT(Eigen::Matrix3d& J); // Function for user input of inertia matrix


