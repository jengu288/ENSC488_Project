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
};


bool internalToUserForm() {
	return false;

}

bool transformMultiply() {
	return false;

}

bool transformInvert() {
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

internalForm inverter(internalForm original) {
	//diagonal elements dont change
	internalForm transposed;
	transposed = original;

	transposed.rotation[0][1] = original.rotation[1][0];
	transposed.rotation[1][0] = original.rotation[0][1];
	transposed.rotation[0][2] = original.rotation[2][0];
	transposed.rotation[2][0] = original.rotation[0][2];
	transposed.rotation[1][2] = original.rotation[2][1];
	transposed.rotation[2][1] = original.rotation[1][2];

	//transposed.position = matrixmult(transposed.rotation,original.position);
	//transposed.position = matrixmult(transposed.position, -1);

	transposed.position[0] = -(transposed.rotation[0][0] * original.position[0] + transposed.rotation[0][1] * original.position[1] + transposed.rotation[0][2] * original.position[2]);
	transposed.position[1] = -(transposed.rotation[1][0] * original.position[0] + transposed.rotation[1][1] * original.position[1] + transposed.rotation[1][2] * original.position[2]);
	transposed.position[2] = -(transposed.rotation[2][0] * original.position[0] + transposed.rotation[2][1] * original.position[1] + transposed.rotation[2][2] * original.position[2]);

	printInternalForm(transposed);
	return transposed;
	//original.rotation[0][1]
}

internalForm  userToInternalForm(double x, double y, double z, double theta) {
	double angleInRad = DEG2RAD(theta);

	internalForm tester = {
		{{cos(angleInRad), -sin(angleInRad), 0},
		{sin(angleInRad), cos(angleInRad), 0},
		{0, 0, 1}},

		{x, y, z}
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

	
	inverter(userToInternalForm(1, 2, 3, 90));

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
