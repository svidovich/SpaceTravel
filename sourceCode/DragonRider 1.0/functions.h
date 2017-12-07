#include <iostream>
#include <fstream>
#include <cstdlib>
#include <string>
#include <Eigen/Dense>
#include <math.h>
using namespace std;
using namespace Eigen;


// Customer Service Implementation
void thinkofthecustomer(ofstream& fout, ofstream& xout, int& n, double& m, double tol)
{
    string fname, xname;
    cout << "Welcome to Dragon Ride v1.0! Let's fly like SpaceX.\n";
    cout << "Enter a filename for output of rotation matrices (include a file extension!): ";

    // File Openings.
    getline(cin, fname); // Get the name of the file for matrix output
    fout.open(fname.c_str()); // Open
    if (fout.fail()) // Procedural cautions...
    {
	cout << "Opening output file failed. Shutting down...\n";
        exit(1);
    }
    cout << "Enter a filename for output of position vectors (include a file extension!): "; 
    getline(cin, xname); // Get the name of file for vector output
    xout.open(xname.c_str()); // Open
    if (xout.fail()) // Procedural Cautions...
    {
	cout << "Opening output file failed. Shutting down...\n";
        exit(1);
    }

    // Customer Service Section
    cout << "Error tolerance is set to " << tol << ".\n"; // This can't be changed by the user, but they should know, right?
    cout << "Please enter the number of steps: ";
    cin >> n;
    cout << "You chose a step size of " << n << ".\n";
    cout << "Enter the mass: "; // Mass of object is needed only for momentum calculus.
    cin >> m;
    cout << endl;
}


