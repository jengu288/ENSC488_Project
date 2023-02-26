// ProgrammingDemo.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <stdio.h>
#include <conio.h>
#include "ensc-488.h"
#include <iostream>
#include <string>
#include <vector>
using namespace std;

const int ROTATE_MATRIX_DIM = 3;
const double L1 = 405, L2 = 195, L3 = 70, L4 = 142, L5 = 270, L6 = 270; //in mm

typedef vector<vector<double>> matrixDouble;

class TransformMatrix
{
public:
	TransformMatrix() {
		//default constructor makes a 4x4 identity matrix
	};

	TransformMatrix(double x, double y, double z, double phi);
	TransformMatrix(matrixDouble matrix);

	matrixDouble getRotation();
	vector<double> getPosition();
	matrixDouble getTransform();
	TransformMatrix getInverseTransform();

	void setRotation(matrixDouble newRotate);
	void setPosition(vector<double> newPosition);
	void setTransform(matrixDouble newTransform);

	void printTransformMatrix();
	void printRotation();
	void printPosition();
	void printUserForm();

	void invert();

	static TransformMatrix userFormToTransformMatrix(double x, double y, double z, double theta);
	static vector<double> transformMatrixToUserForm(TransformMatrix transform);
	static TransformMatrix transformMatrixMultiply(TransformMatrix lh, TransformMatrix rh); //lh*rh since multiplication order matters for matrices
	static TransformMatrix kinBaseToWrist(JOINT jointParameters); //output is base to wrist transform matrix
	static TransformMatrix kinModules(JOINT jointParameters);

	TransformMatrix operator*(TransformMatrix rh); //overloaded operator to do this*rh


private:
	matrixDouble transform = { {1, 0, 0, 0},
							   {0, 1, 0, 0},
							   {0, 0, 1, 0},
							   {0, 0, 0, 1} };

};

int main(int argc, char* argv[])
{
	double theta1 = 0, theta2 = 0, d3 = -200, theta4 = 0; // here for now

	JOINT configA = { 0, 0, -200, 90 }; //JOINT R R P R
	JOINT configB = { 0, 0, -100, 0 };

	TransformMatrix::kinBaseToWrist(configA);
	TransformMatrix::kinModules(configA);

	TransformMatrix identityTest = TransformMatrix();
	matrixDouble rot = identityTest.getRotation();

	//test print functions
	cout << "Print identity from TransformMatrix default constructor" << endl;
	identityTest.printTransformMatrix();
	identityTest.printPosition();
	identityTest.printRotation();

	//test set rotation function
	cout << "Set the rotation[0][0] to 2 using setRotation()" << endl;
	rot[0][0] = rot[0][0] * 2;
	identityTest.setRotation(rot);
	identityTest.printTransformMatrix();

	//testing set transform function
	cout << "Set the transformation[1][1] to 37 using setTransformation()" << endl;
	matrixDouble modifiedTransform = identityTest.getTransform();
	modifiedTransform[1][1] = 37;
	identityTest.setTransform(modifiedTransform);
	identityTest.printTransformMatrix();

	//Testing using 2 simply cases
	TransformMatrix testA = TransformMatrix::userFormToTransformMatrix(337, 0, 135, 0); // 0 0 -200 0 -> check these
	TransformMatrix testB = TransformMatrix::userFormToTransformMatrix(337, 0, 35, 0); // 0 0 -100 0

	testA.printTransformMatrix();
	testB.printTransformMatrix();
	testA.printUserForm();
	testB.printUserForm();

	//testing both multiply options
	TransformMatrix testC = testA * testB;
	TransformMatrix testD = TransformMatrix::transformMatrixMultiply(testA, testB);
	testC.printTransformMatrix();
	testD.printTransformMatrix();

	//test invert function
	testA.invert();
	testA.printTransformMatrix();

	//custom test cases
	TransformMatrix createdTestI = TransformMatrix::userFormToTransformMatrix(5, 8, 2, 83); // x, y, z, phi form input
	TransformMatrix createdTestJ = TransformMatrix::userFormToTransformMatrix(2, 4, 7, 12);

	createdTestI.printTransformMatrix();
	createdTestJ.printTransformMatrix();


	char ch;
	int c;

	const int ESC = 27;
	printf("Keep this window in focus, and...\n");

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

TransformMatrix::TransformMatrix(double x, double y, double z, double phi)
{
	double angleInRad = DEG2RAD(phi);
	transform = { {cos(angleInRad), sin(angleInRad), 0, x},
				 {sin(angleInRad), -cos(angleInRad), 0, y},
				 {0, 0 , -1, z},
				 {0, 0, 0, 1} };

	// the rotation matrix was derived by observing the orientation of the base frame to the wrist frame 
	// where phi is the angle from the base x to the wrist x wrt to the base z
}

TransformMatrix::TransformMatrix(matrixDouble matrix)
{
	transform = matrix;
}

matrixDouble TransformMatrix::getRotation()
{
	matrixDouble rotation = { {0, 0, 0 },
		{0, 0, 0},
		{0, 0, 0} };

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			rotation[i][j] = transform[i][j];
		}
	}
	return rotation;
}

vector<double> TransformMatrix::getPosition()
{
	vector<double> position = { 0, 0, 0 };


	for (int i = 0; i < 3; i++) {
		position[i] = transform[i][3];
	}
	return position;
}

matrixDouble TransformMatrix::getTransform()
{
	return transform;
}

TransformMatrix TransformMatrix::getInverseTransform()
{
	TransformMatrix inverted = *this;
	inverted.invert();
	return inverted;
}

void TransformMatrix::setRotation(matrixDouble newRotate)
{
	for (int i = 0; i < ROTATE_MATRIX_DIM; i++)
	{
		for (int j = 0; j < ROTATE_MATRIX_DIM; j++)
		{
			transform[i][j] = newRotate[i][j];
		}
	}
}

void TransformMatrix::setPosition(vector<double> newPosition)
{
	for (int i = 0; i < newPosition.size(); i++)
	{
		transform[i][3] = newPosition[i];
	}
}

void TransformMatrix::setTransform(matrixDouble newTransform)
{
	transform = newTransform;
}

void TransformMatrix::printTransformMatrix()
{
	cout << "Transformation Matrix" << endl;

	for (int i = 0; i < transform.size(); i++)
	{
		for (int j = 0; j < transform[i].size(); j++)
		{
			cout << transform[i][j] << " ";
		}

		cout << endl;
	}
}

void TransformMatrix::printRotation()
{
	cout << "Rotation Matrix" << endl;

	matrixDouble rotationMat = getRotation();
	for (int i = 0; i < ROTATE_MATRIX_DIM; i++)
	{
		for (int j = 0; j < ROTATE_MATRIX_DIM; j++)
		{
			cout << rotationMat[i][j] << " ";
		}

		cout << endl;
	}
}

void TransformMatrix::printPosition()
{
	cout << "Position Vector" << endl;

	vector<double> positionVec = getPosition();
	for (int i = 0; i < positionVec.size(); i++)
	{
		cout << positionVec[i] << " " << endl;
	}
}

void TransformMatrix::printUserForm()
{
	vector<double> userForm = transformMatrixToUserForm(*this);
	cout << "User Form" << endl;
	for (int i = 0; i < userForm.size(); i++)
	{
		cout << userForm[i] << " ";
	}

	cout << endl;

}

TransformMatrix TransformMatrix::userFormToTransformMatrix(double x, double y, double z, double theta)
{
	return TransformMatrix(x, y, z, theta);
}

vector<double> TransformMatrix::transformMatrixToUserForm(TransformMatrix transformMat)
{
	vector<double> userForm;

	userForm.push_back(transformMat.getPosition()[0]);
	userForm.push_back(transformMat.getPosition()[1]);
	userForm.push_back(transformMat.getPosition()[2]);

	userForm.push_back(RAD2DEG(acos(transformMat.getRotation()[0][0])));

	return userForm;
}

TransformMatrix TransformMatrix::transformMatrixMultiply(TransformMatrix lh, TransformMatrix rh)
{
	return lh * rh; //uses overloaded operator
}

TransformMatrix TransformMatrix::kinBaseToWrist(JOINT jointParameters)
{
	double theta1 = DEG2RAD(jointParameters[0]), theta2 = DEG2RAD(jointParameters[1]), d3 = jointParameters[2], theta4 = DEG2RAD(jointParameters[3]);
	double phi = theta1 + theta2 - theta4;
	double theta12 = theta1 + theta2;

	TransformMatrix baseToWrist({ {cos(phi), sin(phi), 0, L4 * cos(theta12) + L2 * cos(theta1)},
							   {sin(phi),-cos(phi), 0, L4 * sin(theta12) + L2 * sin(theta1)},
							   {0, 0, -1, -d3 - L5 - L6 + L3 + L1},
							   {0, 0, 0, 1} });

	cout << "This is all together" << endl;
	baseToWrist.printTransformMatrix();
	return baseToWrist;
}

TransformMatrix TransformMatrix::kinModules(JOINT jointParameters)
{
	
	double theta1 = DEG2RAD(jointParameters[0]), theta2 = DEG2RAD(jointParameters[1]), d3 = jointParameters[2], theta4 = DEG2RAD(jointParameters[3]);
	TransformMatrix baseToOne({ {cos(theta1), -sin(theta1), 0, 0},
						   {sin(theta1), cos(theta1), 0, 0},
						   {0, 0, 1, L1},
						   {0, 0, 0, 1} });

	TransformMatrix oneToTwo({ {cos(theta2), -sin(theta2), 0, L2},
							   {sin(theta2), cos(theta2), 0, 0},
							   {0, 0, 1, L3},
							   {0, 0, 0, 1} });

	TransformMatrix twoToThree({ {1, 0, 0, L4},
								 {0, -1, 0, 0},
								 {0, 0, -1, -d3-L5},
								 {0, 0, 0, 1} });

	TransformMatrix threeToFour({ {cos(theta4), -sin(theta4), 0, 0},
								  {sin(theta4), cos(theta4), 0, 0},
								  {0, 0, 1, L6},
								  {0, 0, 0, 1} });

	TransformMatrix baseToTwo = baseToOne * oneToTwo;
	TransformMatrix twoToFour = twoToThree * threeToFour;

	TransformMatrix baseToFourModularized = baseToTwo * twoToFour;
	cout << "This is from all the  transforms multiplied" << endl;
	baseToFourModularized.printTransformMatrix();
	return baseToFourModularized;
}

TransformMatrix TransformMatrix::operator*(TransformMatrix rh)
{
	matrixDouble A = this->getTransform();
	matrixDouble B = rh.getTransform();

	// Initialize elements of C to 0
	matrixDouble C = { {0, 0, 0, 0 },
		{0, 0, 0, 0},
		{0, 0, 0, 0},
		{0, 0, 0, 0} };

	// Calculate Matrix C
	for (int i = 0; i < 4; i++) { // rows of C
		for (int j = 0; j < 4; j++) { // columns of B
			for (int k = 0; k < 4; k++) { // columns of A, rows of B
				C[i][j] += A[i][k] * B[k][j];
			}
		}
	}

	return TransformMatrix(C);
}

void TransformMatrix::invert()
{
	//diagonal elements dont change
	matrixDouble transposedRot = this->getRotation();
	matrixDouble orignalRot = this->getRotation();

	transposedRot[0][1] = orignalRot[1][0];
	transposedRot[1][0] = orignalRot[0][1];
	transposedRot[0][2] = orignalRot[2][0];
	transposedRot[2][0] = orignalRot[0][2];
	transposedRot[1][2] = orignalRot[2][1];
	transposedRot[2][1] = orignalRot[1][2];

	vector<double> inversePos = this->getPosition();
	vector<double> originalPos = this->getPosition();

	inversePos[0] = -(transposedRot[0][0] * originalPos[0] + transposedRot[0][1] * originalPos[1] + transposedRot[0][2] * originalPos[2]);
	inversePos[1] = -(transposedRot[1][0] * originalPos[0] + transposedRot[1][1] * originalPos[1] + transposedRot[1][2] * originalPos[2]);
	inversePos[2] = -(transposedRot[2][0] * originalPos[0] + transposedRot[2][1] * originalPos[1] + transposedRot[2][2] * originalPos[2]);

	this->setPosition(inversePos);
	this->setRotation(transposedRot);
}

