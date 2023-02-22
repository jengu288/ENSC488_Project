// ProgrammingDemo.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <stdio.h>
#include <conio.h>
#include "ensc-488.h"
#include <iostream>
#include <string>
using namespace std;

struct internalForm {
	double rotation[3][3];
	double position[3];
	double transform[4][4];
};

void printInternalForm(internalForm toPrint);
double* internalToUserForm(internalForm in);
internalForm transformMultiply(double A[4][4], double B[4][4]);
internalForm transformInvert(internalForm original);
internalForm  userToInternalForm(double x, double y, double z, double theta);

int main(int argc, char* argv[])
{
	JOINT configA = { 0, 0, -100, 0 }; //JOINT R R P R
	JOINT configB = { 0, 0, -200, 0 };
	printf("Keep this window in focus, and...\n");


	char ch;
	int c;

	const int ESC = 27;

	internalForm createdTestA = userToInternalForm(5, 8, 2, 83); // x, y, z, theta form input
	internalForm createdTestB = userToInternalForm(2, 4, 7, 12);
	double* returnedToUser = internalToUserForm(createdTestA);
	
	cout << "Testing for the internal to user form:\n";
	cout << returnedToUser[0] << " " << returnedToUser[1] << " " << returnedToUser[2] << " " << returnedToUser[3] << endl;
 

	transformInvert(createdTestA);
	internalForm createdTestOutput = transformMultiply(createdTestA.transform, createdTestB.transform);

	printf("1:Press any key to continue \n");
	printf("2:Press ESC to exit \n");

	c = _getch();

	while (1)
	{
		if (c != ESC)
		{
			printf("Press '1' or '2' \n");
			ch = _getch();

			if (ch == '1')
			{
				MoveToConfiguration(configA);
			}
			else if (ch == '2')
			{
				MoveToConfiguration(configB);
			}

			printf("Press any key to continue \n");
			printf("Press ESC to exit \n");
			c = _getch();
		}
		else
			break;


	}


	return 0;
}

void printInternalForm(internalForm toPrint) {
	cout << "\nRotation matrix\n";
	cout << toPrint.rotation[0][0] << " " << toPrint.rotation[0][1] << " " << toPrint.rotation[0][2] << endl;
	cout << toPrint.rotation[1][0] << " " << toPrint.rotation[1][1] << " " << toPrint.rotation[1][2] << endl;
	cout << toPrint.rotation[2][0] << " " << toPrint.rotation[2][1] << " " << toPrint.rotation[2][2] << endl;

	cout << "\nPosition vector\n";

	cout << toPrint.position[0] << endl << toPrint.position[1] << endl << toPrint.position[2] << endl;
}

double* internalToUserForm(internalForm in) {
	static double out[4];

	out[0] = in.position[0];
	out[1] = in.position[1];
	out[2] = in.position[2];

	out[3] = RAD2DEG(acos(in.rotation[0][0]));
	return out;
}

internalForm transformMultiply(double A[4][4], double B[4][4]) {
	double C[4][4];
	// Initialize elements of C to 0
	for (int i = 0; i < 4; i++) {   // rows of C
		for (int j = 0; j < 4; j++) { // columns of C
			C[i][j] = 0;
		}
	}

	// Calculate Matrix C
	for (int i = 0; i < 4; i++) { // rows of C
		for (int j = 0; j < 4; j++) { // columns of B
			for (int k = 0; k < 4; k++) { // columns of A, rows of B
				C[i][j] += A[i][k] * B[k][j];
			}
		}
	}

	internalForm output = {
		{{C[0][0], C[0][1], C[0][2]},
		{C[1][0], C[1][1], C[1][2]},
		{C[2][0], C[2][1], C[2][2]}},

		{C[0][3], C[1][3], C[2][3]},

		{{C[0][0], C[0][1], C[0][2], C[0][3]},
		{C[1][0], C[1][1], C[1][2], C[1][3]},
		{C[2][0], C[2][1], C[2][2], C[2][3]},
		{C[3][0], C[3][1], C[3][2], C[3][3]}}
	};

	printInternalForm(output);

	return output;
}

internalForm transformInvert(internalForm original) {
	//diagonal elements dont change
	internalForm transposed;
	transposed = original;

	transposed.rotation[0][1] = original.rotation[1][0];
	transposed.rotation[1][0] = original.rotation[0][1];
	transposed.rotation[0][2] = original.rotation[2][0];
	transposed.rotation[2][0] = original.rotation[0][2];
	transposed.rotation[1][2] = original.rotation[2][1];
	transposed.rotation[2][1] = original.rotation[1][2];

	transposed.position[0] = -(transposed.rotation[0][0] * original.position[0] + transposed.rotation[0][1] * original.position[1] + transposed.rotation[0][2] * original.position[2]);
	transposed.position[1] = -(transposed.rotation[1][0] * original.position[0] + transposed.rotation[1][1] * original.position[1] + transposed.rotation[1][2] * original.position[2]);
	transposed.position[2] = -(transposed.rotation[2][0] * original.position[0] + transposed.rotation[2][1] * original.position[1] + transposed.rotation[2][2] * original.position[2]);

	printInternalForm(transposed);
	return transposed;
}

internalForm  userToInternalForm(double x, double y, double z, double theta) {
	double angleInRad = DEG2RAD(theta);

	internalForm tester = {
		{{cos(angleInRad), -sin(angleInRad), 0},
		{sin(angleInRad), cos(angleInRad), 0},
		{0, 0, 1}},

		{x, y, z},

		{{cos(angleInRad), -sin(angleInRad), 0, x},
		{sin(angleInRad), cos(angleInRad), 0, y},
		{0, 0 , 1, z},
		{0, 0, 0, 1}}
	};

	printInternalForm(tester);
	return tester;
}

