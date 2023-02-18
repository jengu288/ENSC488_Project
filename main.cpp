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

bool internalToUserForm() {
	return false;
}

bool transformMultiply(double A[4][4], double B[4][4], double C[4][4]) {

	// Calculate Matrix C
	for (int i = 0; i < 4; i++) { // rows of C
		for (int j = 0; j < 4; j++) { // columns of B
			for (int k = 0; k < 4; k++) { // columns of A, rows of B
				C[i][j] += A[i][k] * B[k][j];
			}
		}
	}

	// Output Matrix C
	for (int i = 0; i < 4; i++) { // rows of C
		for (int j = 0; j < 4; j++) { // columns of C
			cout << C[i][j] << " ";
		}
		cout << endl;
	}

	return false;
}

void printInternalForm(internalForm tester) {
	cout << "\nrotation matrix\n";
	cout << tester.rotation[0][0] << " " << tester.rotation[0][1] << " " << tester.rotation[0][2] << endl;
	cout << tester.rotation[1][0] << " " << tester.rotation[1][1] << " " << tester.rotation[1][2] << endl;
	cout << tester.rotation[2][0] << " " << tester.rotation[2][1] << " " << tester.rotation[2][2] << endl;

	cout << "\nposition vector\n";

	cout << tester.position[0] << endl << tester.position[1] << endl << tester.position[2] << endl;
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

int main(int argc, char* argv[])
{
	JOINT q1 = { 0, 0, -100, 0 }; //JOINT R R P R
	JOINT q2 = { 0, 0, -200, 0 };
	printf("Keep this window in focus, and...\n");


	char ch;
	int c;

	const int ESC = 27;

	internalForm createdTestA = userToInternalForm(5, 8, 2, 83); // x, y, z, theta form input
	internalForm createdTestB = userToInternalForm(2, 4, 7, 12);

	internalForm createdTestOutput = userToInternalForm(0, 0, 0, 0);
	transformInvert(createdTestA);
	transformMultiply(createdTestA.transform, createdTestB.transform, createdTestOutput.transform);

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
				MoveToConfiguration(q1);
			}
			else if (ch == '2')
			{
				MoveToConfiguration(q2);
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
