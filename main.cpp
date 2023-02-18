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

internalForm  userToInternalForm(double x, double y, double z, double theta) {
	double angleInRad = DEG2RAD(theta);

	internalForm tester = {
		{{cos(angleInRad), -sin(angleInRad), 0},
		{sin(angleInRad), cos(angleInRad), 0},
		{0, 0, 1}},

		{x, y, z}
	};

	cout << tester.rotation[0][1] << endl << tester.rotation[1][1] << endl << tester.rotation[2][1] << endl;
	cout << tester.position[0] << endl << tester.position[1] << endl << tester.position[2] << endl;

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

	userToInternalForm(1, 2, 3, 90);

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
