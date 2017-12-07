#include <iostream>
#include <fstream>
#include <cstdlib>
#include <string>
#include <Eigen/Dense>
#include <math.h>
using namespace std;
using namespace Eigen;


//Definition for function that allows user input for J. 
void JINPUT(Matrix3d& J)
{

    bool a = true;
    while(a){
		    double a11,a12,a13,a21,a22,a23,a31,a32,a33;
		    char correct;
		    cout << "Enter the entries of your inertia matrix as prompted. \n";
		    cout << "a11: ";
		    cin >> a11;
		    cout << "\na12: ";
		    cin >> a12;
		    cout << "\na13: ";
		    cin >> a13;

		    cout << "\na21: ";
		    cin >> a21;
		    cout << "\na22: ";
		    cin >> a22;
		    cout << "\na23: ";
		    cin >> a23;

		    cout << "\na31: ";
		    cin >> a31;
		    cout << "\na32: ";
		    cin >> a32;
		    cout << "\na33: ";
		    cin >> a33;



		    J << a11, a12, a13,
			 a21, a22, a23,
			 a31, a32, a33;

		    cout << "You entered the matrix\n" << J <<"\n Is this correct? (Y/N): ";
		    cin >> correct;
		    if (correct == 'Y' || correct == 'y')
		    {
			a = false;
		    }
		    else
		    {
			a = true;
		    }
	    }

}


//Definition for J definition wrapper
void JDEF(Matrix3d& J)
{
	int choice; // For the switch
	bool cont = true;  // For the While loop
	cout << "Please choose from the list to define your inertia matrix:\n";
	cout << "\t1. Enter my own matrix\n\t2. Choose Tie Fighter\n\t3. Choose Dragon\n\t4. Choose Cube\n";
	while(cont)
	{
		cout << "Enter a number: ";
		cin >> choice;
		switch(choice)
		{
			case 1: // If user inputs 1, they want to define their own.
				JINPUT(J);
				cont = false;
				break;
			case 2: // If user inputs 2, they want a tie fighter. J dense as as titanium.
				J << 1315380, 0, 0,
					0, 792102, 0,
						0, 0, 9489670;
				cont = false;
				break;
			case 3: // If user inputs 3, they want a dragon.
				J << 8662.0, 0, 0,
					0, 8662.0, 0,
						0, 0, 49062.0;
				cont = false;
				break;
			case 4: // If user inputs 4, they want a cube.
				J << 2.66666, 0, 0,
					0, 2.66666, 0,
					0, 0, 2.66666;
				cont = false;
				break;
			default: // The user input garbage.
				cout << "Invalid input.\n";
				 
		}
	}
}


// Customer Service Implementation
void thinkofthecustomer(ofstream& fout, ofstream& xout, int& n, double& m, double tol)
{
    string fname, xname;
    cout << "Welcome to matrix rotation v0.3! We will rotate your object and send it through space.\n";
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


