// main.cpp : Defines the entry point for the console application.

#include "stdafx.h"
#include <stdio.h>
#include <conio.h>
#include "ensc-488.h"
#include <iostream>
#include <string>
#include <vector>
#include <Windows.h>
#include <fstream>
using namespace std;

const int ROTATE_MATRIX_DIM = 3;
const double L1 = 405, L2 = 195, L3 = 70, L4 = 142, L5 = 270, L6 = 270; //in mm
const double j1MinLim = -(150), j1MaxLim = (150), j2MinLim = -(100), j2MaxLim = (100), j3MinLim = -200, j3MaxLim = -100, j4MinLim = -(160), j4MaxLim = (160);
const double j1MinVel = -(150), j1MaxVel = (150), j2MinVel = -(150), j2MaxVel = (150), j3MinVel = -50, j3MaxVel = 50, j4MinVel = -(150), j4MaxVel = (150);
const double j1MinAcc = -(600), j1MaxAcc = (600), j2MinAcc = -(600), j2MaxAcc = (600), j3MinAcc = -200, j3MaxAcc = 200, j4MinAcc = -(600), j4MaxAcc = (600);

const double minJointLimits[4] = { j1MinLim, j2MinLim, j3MinLim, j4MinLim };
const double maxJointLimits[4] = { j1MaxLim, j2MaxLim, j3MaxLim, j4MaxLim };
const double minVelocity[4] = { j1MinVel, j2MinVel, j3MinVel, j4MinVel }; //EDIT
const double maxVelocity[4] = { j1MaxVel, j2MaxVel, j3MaxVel, j4MaxVel }; //EDIT
const double minAcceleration[4] = { j1MinAcc, j2MinAcc, j3MinAcc, j4MinAcc }; //EDIT
const double maxAcceleration[4] = { j1MaxAcc, j2MaxAcc, j3MaxAcc, j4MaxAcc }; //EDIT

typedef vector<vector<double>> matrixDouble;
vector<matrixDouble> Planner(matrixDouble positions, double time, bool& canMove, vector<string>& issues);
vector<matrixDouble> generater(vector<matrixDouble> coeffMatrix, double trajTime, int samplingRate, vector<string>& issues);
void limitChecker(double pos, double vel, double acc, int joint, vector<string>& issues);

class TransformMatrix
{
public:
	TransformMatrix() {
		//default constructor makes a 4x4 identity matrix
	};

	TransformMatrix(double x, double y, double z, double phi); //This gives us the base to wrist transform from x y z phi
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

	//Requested functions for Demo1, names may have been modfied for clarification
	static TransformMatrix userFormToTransformMatrix(double x, double y, double z, double theta);
	static vector<double> transformMatrixToUserForm(TransformMatrix transform);

	static TransformMatrix transformMatrixMultiply(TransformMatrix lh, TransformMatrix rh); //lh*rh since multiplication order matters for matrices
	TransformMatrix operator*(TransformMatrix rh); //overloaded operator to do this*rh

	static TransformMatrix forKinBaseToWrist(JOINT jointParameters); //output is base to wrist transform matrix
	static TransformMatrix forKinModules(JOINT jointParameters); //output is base to wrist transform matrix
	static vector<double> where(JOINT joints, TransformMatrix TtoW, TransformMatrix StoB);

	static vector<vector<double>> invKinBaseToWrist(TransformMatrix wRelB, JOINT current);
	static vector<double> solve(double x, double y, double z, double phi, TransformMatrix StoB, TransformMatrix WtoT);

	//Helper Functions
	static double customRound(double num);
	static bool areJointsValid(double theta1, double theta2, double d3, double theta4);

private:
	matrixDouble transform = { {1, 0, 0, 0},
							   {0, 1, 0, 0},
							   {0, 0, 1, 0},
							   {0, 0, 0, 1} };

};

int main(int argc, char* argv[])
{
	matrixDouble wristToTool = { {1, 0, 0, 0},
							   {0, 1, 0, 0},
							   {0, 0, 1, 0}, //assign z as 10mm // make it 0 for testing
							   {0, 0, 0, 1} };
	TransformMatrix WtoT(wristToTool);

	matrixDouble stationToBase = { {1, 0, 0, 0},
						   {0, 1, 0, 0},
						   {0, 0, 1, 0},
						   {0, 0, 0, 1} };
	TransformMatrix StoB(stationToBase);

	JOINT homePose = {90, 0, -175, 0};
	char ch;
	int c;
	const int ESC = 27;

	// position matrix for trajectory planning
	matrixDouble positions = { {0, 0, 0, 0},
								{0, 0, 0, 0},
								{0, 0, 0, 0},
								{0, 0, 0, 0},
								{0, 0, 0, 0} };

	printf("\\:D/ Welcome to the ROBOSIM Controller Panel! \\:D/\n");
	printf("-------------------------------------------------\n");
	printf("1: Start \n");
	printf("E: Exit by Pressing ESC \n");
	printf("-------------------------------------------------\n");

	c = _getch();
	while (1) {
		if (c != ESC)
		{
			printf("Options:\n");
			printf("1: Specify joint values and move robot to that position. \n2: Specify a pose and move robot to that position. \n3: Grasp. \n4: Release. \n5: Return robot to home position. \n6: Trajectory Planning. \nE: Exit by Pressing ESC \n");
			printf("-------------------------------------------------\n");

			ch = _getch();

			if (ch == '1') //Joint Values Specified
			{
				printf("\n1: Specify Joint Values in degrees and mm in the order: theta1 theta2 d3 theta4\n");
				fflush(stdin);
				double jv1, jv2, jv3, jv4;
				fflush(stdin);
				scanf_s("%lf", &jv1);
				fflush(stdin);
				scanf_s("%lf", &jv2);
				fflush(stdin);
				scanf_s("%lf", &jv3);
				fflush(stdin);
				scanf_s("%lf", &jv4);
				fflush(stdin);
				if (jv1 < -150 || jv1 > 150 || jv2 < -100 || jv2 > 100 || jv3 < -200 || jv3 > -100 || jv4 < -160 || jv4 > 160) {
					printf("Specified joint values are not within joint limits.\n");
					continue;
				}
				else {
					JOINT configFK = { jv1, jv2, jv3, jv4 };
					printf("Moving to specified joint variables.\n");
					// Move robot to configuration
					MoveToConfiguration(configFK, true);
					// Compute and print pose
					vector<double> pose = TransformMatrix::where(configFK, WtoT, StoB);
					printf("Position and Orientation of the Tool (x,y,z,phi): %.2f, %.2f, %.2f, %.2f\n", pose[0], pose[1], pose[2], pose[3]);
				}
			}
			else if (ch == '2') //Inverse Kinematics: Pose Specified
			{
				printf("\n2: Specify Pose in degrees and mm in the order: x y z phi\n");
				double x, y, z, phi;
				fflush(stdin);
				scanf_s("%lf", &x);
				fflush(stdin);
				scanf_s("%lf", &y);
				fflush(stdin);
				scanf_s("%lf", &z);
				fflush(stdin);
				scanf_s("%lf", &phi);
				fflush(stdin);
				TransformMatrix specifiedTransform;
				specifiedTransform = specifiedTransform.userFormToTransformMatrix(x, y, z, phi);
				JOINT configIK;
				GetConfiguration(configIK);
				vector<vector<double>>retVec = specifiedTransform.invKinBaseToWrist(specifiedTransform, configIK);
				if (retVec[0][0] == 0) {
					printf("Sorry, no valid solutions.\n");
				}
				else if (retVec[0][0] == 1){
					configIK[0] = (retVec[1][0]);
					configIK[1] = (retVec[1][1]);
					configIK[2] = retVec[1][2];
					configIK[3] = (retVec[1][3]);
					printf("Calculated Joint Variables (theta1 theta2 d3 theta4): \n%.2f, %.2f ,%.2f, %.2f\n", configIK[0], configIK[1], configIK[2], configIK[3]);
					if (retVec.size() > 2) {
						if (specifiedTransform.areJointsValid(retVec[2][0], retVec[2][1], retVec[2][2], retVec[2][3])) {
							printf("Another worse solution: %.2f, %.2f, %.2f, %.2f\n", (retVec[2][0]), (retVec[2][1]), retVec[2][2], (retVec[2][3]));
						}
						
					}

					printf("Moving to calculated joint variables for closest solution.\n");
					MoveToConfiguration(configIK, true);
				}
			}
			else if (ch == '3') // grasp
			{
				printf("\n3: Grasp Object\n");
				Grasp(true);
			}
			else if (ch == '4') // release
			{
				printf("\n4: Release Grasped Object\n");
				Grasp(false);
			}
			else if (ch == '5') //return to home
			{
				printf("\n5: Return robot to home position\nMoving robot to home position.\n");

				MoveToConfiguration(homePose, true);
			}
			else if (ch == '6') // trajectory planning
			{
				// each row of the position matrix is an intermediate position
				// except the last row in the position matrix is the goal position and the first row is the current position
			
				// store current configuration into first row of position matrix
				JOINT configCurrent;
				GetConfiguration(configCurrent);  //returns curent config of joints in deg and mm
				for (int j = 0; j < 4; j++){
					positions[0][j] = configCurrent[j]; 
				}

				// set position matrix rows 1-3 to specified intermediate positions in joint space
				double x, y, z, phi, time, sampleRate;
				bool canMove = true;
				vector<string> issues;
				bool earlyExitFlag = false;


				for (int i = 1; i < 5; i++){
					// get intermediate positions from user
					if (i < 4) {
						printf("\n6: Specify the %i position in degrees and mm in the order: x y z phi\n", i);
					}
					// get goal position from user
					else{
						printf("\n6: Specify the goal position in degrees and mm in the order: x y z phi\n");
					}
					fflush(stdin);
					scanf_s("%lf", &x);
					fflush(stdin);
					scanf_s("%lf", &y);
					fflush(stdin);
					scanf_s("%lf", &z);
					fflush(stdin);
					scanf_s("%lf", &phi);
					fflush(stdin);
					
					// convert cartesian to joint values using invKin
					TransformMatrix specifiedPointTM = TransformMatrix::userFormToTransformMatrix(x, y, z, phi);
					matrixDouble specifiedPos = TransformMatrix::invKinBaseToWrist(specifiedPointTM, configCurrent);
					
					// check if solution possible for invKin
					if (specifiedPos.size() == 1)
					{
						printf("\nThe specified position is not mathematically possible.\n");
						earlyExitFlag = true;
						break;
					}

					// store joint space positions in position matrix
					for (int j = 0; j < 4; j++)
					{
						positions[i][j] = specifiedPos[1][j];
						configCurrent[j] = specifiedPos[1][j];
					}
				}

				if (earlyExitFlag) {
					continue;
				}

				// print position matrix
				cout << "Position Matrix" << endl;
				for (int i = 0; i < 5; i++)
				{
					for (int j = 0; j < 4; j++)
					{
						cout << positions[i][j] << " ";
					}
					cout << endl;
				}

				printf("Enter the desired trajectory duration\n");
				fflush(stdin);
				scanf_s("%lf", &time);
				printf("Enter the desired number of samples per polynomial\n");
				fflush(stdin);
				scanf_s("%lf", &sampleRate);
				vector<matrixDouble>coefMtx = Planner(positions, time, canMove, issues);

				//print coef matrix
				for (int i = 0; i < 4; i++)
				{
					printf("\nCoefficient Matrix for Joint Variable #%d\n", i + 1);
					for (int j = 0; j < 4; j++)
					{
						for (int k = 0; k < 4; k++)
						{
							cout << TransformMatrix::customRound(coefMtx[i][j][k]) << " ";
						}
						cout << endl;
					}
					cout << endl;
				}

				cout << "Testing generator with planner code\n";
				vector<matrixDouble> samples = generater(coefMtx, time, (int)sampleRate, issues);

				matrixDouble samplePos = samples[0];
				matrixDouble sampleVel = samples[1];
				matrixDouble sampleAcc = samples[2];

				double currentTime = clock() / CLOCKS_PER_SEC;
				double timeIncr = (time / (4)) / sampleRate;
				matrixDouble cartesian;

				//printing the values to a file
				ofstream outFile("cartesian.txt");
				if (!outFile.is_open()) {
					cout << "File could not be opened. That is weird.\n";
				}

				for (int i = 0; i < samplePos.size(); i++)
				{
					JOINT p = { samplePos[i][0], samplePos[i][1], samplePos[i][2], samplePos[i][3] };
					JOINT v = { sampleVel[i][0], sampleVel[i][1], sampleVel[i][2], sampleVel[i][3] };
					JOINT a = { sampleAcc[i][0], sampleAcc[i][1], sampleAcc[i][2], sampleAcc[i][3] };

					cartesian.push_back(TransformMatrix::transformMatrixToUserForm(TransformMatrix::forKinBaseToWrist(p)));
					outFile << cartesian[i][0] << " " << cartesian[i][1];
					outFile << endl;

					MoveWithConfVelAcc(p, v, a);
					/*while (clock() / CLOCKS_PER_SEC < currentTime + timeIncr) {
						
					}*/
					Sleep(timeIncr * 1000);
					JOINT actualCurrentConfig;
					GetConfiguration(actualCurrentConfig);
					//while (TransformMatrix::customRound(actualCurrentConfig[0])!= TransformMatrix::customRound(p[0]) && TransformMatrix::customRound(actualCurrentConfig[1])!= TransformMatrix::customRound(p[1]) && TransformMatrix::customRound(actualCurrentConfig[2]) != TransformMatrix::customRound(p[2]) && TransformMatrix::customRound(actualCurrentConfig[3])!= TransformMatrix::customRound(p[3])) {
					//	GetConfiguration(actualCurrentConfig);
					//}
					//currentTime = clock() / CLOCKS_PER_SEC;

				}



			}
			else if (ch == ESC) {
				break;
			}
			else {
				printf("Please enter a valid key. Try again.");
				continue;
			}
			printf("\n-------------------------------------------------\n");
			printf("C: Press any key to continue\nE: Exit by Pressing ESC \n");
			printf("-------------------------------------------------\n");

			c = _getch();
		}
		else
			break;
	}
	return 0;
}

vector<matrixDouble> generater(vector<matrixDouble> coeffMatrix, double trajTime, int samplingRate, vector<string>& issues) {

	matrixDouble position; 
	matrixDouble velocity;
	matrixDouble acceleration;
	vector<double> time;
	vector<double> empty(4);

	for (int i = 0; i < samplingRate*4 + 1; i++)// need to initalize matrices to have a row for each sample
	{
		position.push_back(empty);
		velocity.push_back(empty);
		acceleration.push_back(empty);

	}

	double timeSeg = (trajTime / 4) / samplingRate; //timeSeg is based on how many samples per intermediate poly
	for (int i = 0; i < 4; i++) //for each joint polynomial matrix in coeffMatrix
	{
		matrixDouble polynomialMat = coeffMatrix[i];
		int matrixOffset = 0;

		for (int j = 0; j < 4; j++) //there are always 4 intermediate polymonials for 3 via points + goal
		{
			double t = 0;
			//coeff extraction for each intermediate polymonial
			double a0 = polynomialMat[j][0];
			double a1 = polynomialMat[j][1];
			double a2 = polynomialMat[j][2];
			double a3 = polynomialMat[j][3];
			
			for (int k = 0; k < samplingRate; k++) //for the requested number of samples
			{
				clock_t startTime = clock();
				//add error checking for limits
				double calculatedPos = a0 + a1 * t + a2 * pow(t, 2) + a3 * pow(t, 3);
				double calculatedVel = a1 + 2 * a2 * t + 3 * a3 * pow(t, 2);
				double calculatedAcc = 2 * a2 + 6 * a3 * t;

				limitChecker(calculatedPos, calculatedVel, calculatedAcc, j, issues);

				position[k + matrixOffset][i] = TransformMatrix::customRound(calculatedPos);
				velocity[k + matrixOffset][i] = TransformMatrix::customRound(calculatedVel);
				acceleration[k + matrixOffset][i] = TransformMatrix::customRound(calculatedAcc);

				t += timeSeg;
				//while (startTime + 1000*timeSeg > clock()) {
				//	//not enough time has passed yet
				//}
				cout << startTime << "\n";
			}

			if (j == 3) { // for last polynomial, do one more point
				double calculatedPos = a0 + a1 * t + a2 * pow(t, 2) + a3 * pow(t, 3);
				double calculatedVel = a1 + 2 * a2 * t + 3 * a3 * pow(t, 2);
				double calculatedAcc = 2 * a2 + 6 * a3 * t;

				limitChecker(calculatedPos, calculatedVel, calculatedAcc, j, issues);

				position[samplingRate + matrixOffset][i] = TransformMatrix::customRound(calculatedPos);
				velocity[samplingRate + matrixOffset][i] = TransformMatrix::customRound(calculatedVel);
				acceleration[samplingRate + matrixOffset][i] = TransformMatrix::customRound(calculatedAcc);

				t += timeSeg;
			}
			matrixOffset += samplingRate;
		}

	}

	//printing the values to a file
	ofstream outFile("samples.txt");
	if (!outFile.is_open()) {
		cout << "File could not be opened. That is weird.\n";
	}
	
	double totalTime = 0;
	for (int i = 0; i < samplingRate*4 + 1; i++) //might change this to reflect # of samples per joint differently
	{
		if (i % samplingRate == 0) {
			cout << "\nNew poly\n";
		}

		outFile << totalTime << " ";
		cout << totalTime << " ";

		totalTime += timeSeg;

		for (double pos: position[i])
		{
			outFile << pos << " ";
			cout << pos << " ";
		}
		for (double vel : velocity[i])
		{
			outFile << vel << " ";
			cout << vel << " ";
		}
		for (double acc : acceleration[i])
		{
			outFile << acc << " ";
			cout << acc << " ";
		}

		outFile << endl;
		cout << endl;
	}

	outFile.close();
	vector<matrixDouble> out = { position, velocity, acceleration };
	return out;
}

void limitChecker(double pos, double vel, double acc, int joint, vector<string>& issues)
{
	if (pos > maxJointLimits[joint]) {
		string issueString = "Joint " + to_string(joint + 1) + "violates its max joint limit since the requested value is " + to_string(pos)
			+ " and the limit is " + to_string(maxJointLimits[joint]) + "\n";

		issues.push_back(issueString);
	}
	else if (pos < minJointLimits[joint]) {
		string issueString = "Joint " + to_string(joint + 1) + "violates its min joint limit since the requested value is " + to_string(pos) 
			+ " and the limit is " + to_string(minJointLimits[joint]) + "\n";
		issues.push_back(issueString);
	}

	if (vel > maxVelocity[joint]) {
		string issueString = "Joint " + to_string(joint + 1) + "violates its max joint velocity limit since the requested value is " + to_string(vel) 
			+ " and the limit is " +to_string(maxVelocity[joint]) + "\n";
		issues.push_back(issueString);
	}
	else if (vel < minVelocity[joint]) {
		string issueString = "Joint " + to_string(joint + 1) + "violates its min joint velocity limit since the requested value is " + to_string(vel) 
			+ " and the limit is " + to_string(minVelocity[joint]) + "\n";
		issues.push_back(issueString);
	}

	if (acc > maxAcceleration[joint]) {
		string issueString = "Joint " + to_string(joint + 1) + "violates its max joint acceleration limit since the requested value is " + to_string(acc) 
			+ " and the limit is " + to_string(maxAcceleration[joint]) + "\n";
		issues.push_back(issueString);
	}
	else if (acc < minAcceleration[joint]) {
		string issueString = "Joint " + to_string(joint + 1) + "violates its min joint acceleration limit since the requested value is " + to_string(acc)
			+ " and the limit is " + to_string(minAcceleration[joint]) + "\n";
		issues.push_back(issueString);
	}
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
			cout << customRound(transform[i][j]) << " ";
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
			cout << customRound(rotationMat[i][j]) << " ";
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
		cout << customRound(positionVec[i]) << " " << endl;
	}
}

void TransformMatrix::printUserForm()
{
	vector<double> userForm = transformMatrixToUserForm(*this);
	cout << "User Form" << endl;
	for (int i = 0; i < userForm.size(); i++)
	{
		cout << customRound(userForm[i]) << " ";
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

	double sinResult = transformMat.getRotation()[0][1];
	double cosResult = transformMat.getRotation()[0][0];
		
	userForm.push_back(RAD2DEG(atan2(sinResult, cosResult)));

	return userForm;
}

TransformMatrix TransformMatrix::transformMatrixMultiply(TransformMatrix lh, TransformMatrix rh)
{
	return lh * rh; //uses overloaded operator
}

TransformMatrix TransformMatrix::forKinBaseToWrist(JOINT jointParameters)
{
	//might be able to move this conversion to the user input part
	double theta1 = DEG2RAD(jointParameters[0]), theta2 = DEG2RAD(jointParameters[1]), d3 = jointParameters[2], theta4 = DEG2RAD(jointParameters[3]);
	double phi = theta1 + theta2 - theta4;
	double theta12 = theta1 + theta2;

	TransformMatrix baseToWrist({ {cos(phi), sin(phi), 0, L4 * cos(theta12) + L2 * cos(theta1)},
							   {sin(phi), -cos(phi), 0, L4 * sin(theta12) + L2 * sin(theta1)},
							   {0, 0, -1, -d3 - L5 - L6 + L3 + L1},
							   {0, 0, 0, 1} });

	return baseToWrist;
}

TransformMatrix TransformMatrix::forKinModules(JOINT jointParameters)
{
	//might be able to move this conversion to the user input part
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
	return baseToFourModularized;
}

vector<double> TransformMatrix::where(JOINT joints, TransformMatrix WtoT, TransformMatrix StoB) {
	TransformMatrix BtoW = forKinModules(joints); //defining frame 4 as wrist
	TransformMatrix StoT = StoB * BtoW * WtoT;
	matrixDouble transformStoT = StoT.getTransform();
	double phi = atan2(transformStoT[1][0], transformStoT[0][0]);
	vector<double> pose = { transformStoT[0][3], transformStoT[1][3], transformStoT[2][3], RAD2DEG(phi) };

	return pose;
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

double TransformMatrix::customRound(double num) {
	return double(round(100 * num)) / 100;
}

bool TransformMatrix::areJointsValid(double theta1, double theta2, double d3, double theta4) {//inputs must be in degrees
	bool jLimViolation = false;

	if (customRound((theta1)) > j1MaxLim || customRound((theta1)) < j1MinLim) {
		printf("There exists a solution that is invalid because it violates joint limits: theta1 = %.2lf degrees\n", (theta1));
		jLimViolation = true;
	}
	if (customRound((theta2)) > j2MaxLim || customRound((theta2)) < j2MinLim) {
		printf("There exists a solution that is invalid because it violates joint limits: theta2 = %.2lf degrees\n", (theta2));
		jLimViolation = true;
	}
	if (customRound(d3) > j3MaxLim || customRound(d3) < j3MinLim) {
		printf("There exists a solution that is invalid because it violates joint limits: d3 = %.2lf\n", d3);
		jLimViolation = true;
	}
	if (customRound((theta4)) > j4MaxLim || customRound((theta4)) < j4MinLim) {
		printf("There exists a solution that is invalid because it violates joint limits: theta4 = %.2lf degrees\n", (theta4));
		jLimViolation = true;
	}
	if (jLimViolation) {
		printf("Solution that violates joint limits (theta1 theta2 d3 theta4):\n%.2lf, %.2lf, %.2lf, %.2lf\n\n", (theta1), (theta2), d3, (theta4));
		jLimViolation = true;
	}
	return !jLimViolation;
}

vector<vector<double>> TransformMatrix::invKinBaseToWrist(TransformMatrix wRelB, JOINT current) {
	bool sol;
	double theta1, theta2, d3, theta4, x, y, z;
	vector<double> pos = wRelB.getPosition();
	x = pos[0];
	y = pos[1];
	z = pos[2];

	vector<double>existsSolution{0};
	vector<vector<double>> returnVec;
	returnVec.push_back(existsSolution);

	vector<vector<double>>solutions;
	for (size_t i = 0; i < 2; i++){
		if ((pow(x, 2) + pow(y, 2) - pow(L4, 2) - pow(L2, 2)) / (2 * L2 * L4) > 1) {
			//no solution.
			printf("There are no mathematically possible solutions\n");
			continue; //if theta 2 has no solutions, there are no solutions at all
		}
		
		//theta2 is in radians here
		theta2 = atan2(pow(-1,i)*sqrt(1 - pow(((pow(x, 2) + pow(y, 2) - pow(L4, 2) - pow(L2, 2)) / (2 * L2 * L4)),2)), (pow(x, 2) + pow(y, 2) - pow(L4, 2) - pow(L2, 2)) / (2 * L2 * L4));
		if (theta2 == 0 && i == 0) { //prevent duplicate solutions returned
			continue;
		}
		if (theta2 > PI) {
			theta2 = theta2 - 2*PI;
		}
		else if (theta2 < -PI) {
			theta2 = theta2 + 2 * PI;
		}
		
		double a = L4 * cos(theta2) + L2;
		double b = L4 * sin(theta2);
		if (a == 0 && b == 0) {
			if (x == 0 && y == 0) {
				theta1 = current[0];
			}
			else if (x != 0 || y != 0) {
				//zero solutions
				continue;
			}
		}
		else {//ax-by
			//theta 1 is in radians
			theta1 = atan2((a * y) - (b * x), (a * x) + (b * y));
		}
		if (theta1 > PI) {
			theta1 = theta1 - 2 * PI;
		}
		else if (theta1 < -PI) {
			theta1 = theta1 + 2 * PI;
		}
		
		double r11 = wRelB.getRotation()[0][0];
		double r21 = wRelB.getRotation()[1][0];
		if (r11 == 0) {
			//no solution
			continue;
		}
		else {
			theta4 = -atan2(r21, r11) + theta1 + theta2;
		}
		if (theta4 > PI) {
			theta4 = theta4 - 2 * PI;
		}
		else if (theta4 < -PI) {
			theta4 = theta4 + 2 * PI;
		}
		
		d3 = L1 - z + L3 - L5 - L6;

		//These solutionElements are in terms of deg and mm
		vector<double>solutionElements;
		solutionElements.push_back(RAD2DEG(theta1));
		solutionElements.push_back(RAD2DEG(theta2));
		solutionElements.push_back(d3);
		solutionElements.push_back(RAD2DEG(theta4));
		solutions.push_back(solutionElements);
		}

	//everything past this point is in degrees
	if (solutions.size() == 0){
		return returnVec;
	}
	if (solutions.size() == 1) {
		if (areJointsValid(solutions[0][0], solutions[0][1], solutions[0][2], solutions[0][3])) {
			returnVec[0] = { 1 }; //flag:exists solution
			returnVec.push_back(solutions[0]); //only one possible solution, returnVec will have 2 elements, flag and solution
			printf("Distance to solution is: %.2lf\n", sqrt(pow((current[0] - solutions[0][0]), 2) + pow((current[1] - solutions[0][1]), 2) + pow((current[2] - solutions[0][2]), 2) + pow((current[3] - solutions[0][3]), 2)));
			return returnVec;
		}
		else {
			returnVec[0] = { 0 }; //solution is invalid
			returnVec.push_back(solutions[0]);
			printf("Distance to invalid solution is: %.2lf\n", sqrt(pow((current[0]) - (solutions[0][0]), 2) + pow((current[1]) - (solutions[0][1]), 2) + pow(current[2] - solutions[0][2], 2) + pow((current[3]) - (solutions[0][3]), 2)));
			return returnVec;
		}

	}

	//if there are multiple solutions possible
	vector<double>distances;
	for (size_t i = 0; i < solutions.size(); i++){
		distances.push_back(sqrt(pow((current[0]) - solutions[i][0], 2) + pow((current[1]) - solutions[i][1], 2) + pow(current[2] - solutions[i][2], 2) + pow((current[3]) - solutions[i][3], 2)));
	}
	int minDistIndex = 0;
	if (areJointsValid(solutions[0][0], solutions[0][1], solutions[0][2], solutions[0][3]) == 0 && areJointsValid(solutions[1][0], solutions[1][1], solutions[1][2], solutions[1][3]) == 0) {
		minDistIndex = 0;
		for (size_t i = 0; i < distances.size(); i++) {
			if (distances[i] < distances[minDistIndex])
				minDistIndex = i;
			}
		returnVec[0] = { 0 }; //boolean:no valid solutions (due to joint limits)
		returnVec.push_back(solutions[minDistIndex]);
		if (minDistIndex == 1) {
			returnVec.push_back(solutions[0]);
			printf("Minimum distance for solution (with invalid joint values): %.4lf\nDistance for further solution(with invalid joint values): %.4lf\n\n", distances[1], distances[0]);
		}
		else {
			returnVec.push_back(solutions[1]);
			printf("Minimum distance for solution (with invalid joint values): %.4lf\nDistance for further solution(with invalid joint values): %.4lf\n\n", distances[0], distances[1]);
		}
	}
	else if (areJointsValid(solutions[0][0], solutions[0][1], solutions[0][2], solutions[0][3]) == 1 && areJointsValid(solutions[1][0], solutions[1][1], solutions[1][2], solutions[1][3]) == 1) {
		minDistIndex = 0;
		for (size_t i = 0; i < distances.size(); i++) {
			if (distances[i] < distances[minDistIndex])
				minDistIndex = i;
		}
		returnVec[0] = { 1 }; //boolean:exists valid solutions
		returnVec.push_back(solutions[minDistIndex]);
		if (minDistIndex == 1) {
			returnVec.push_back(solutions[0]);
			printf("Minimum distance for solution: %.4lf\nDistance for further solution: %.4lf\n\n", distances[1], distances[0]);
		}
		else {
			returnVec.push_back(solutions[1]);
			printf("Minimum distance for solution: %.4lf\nDistance for further solution: %.4lf\n\n", distances[0], distances[1]);
		}
	}
	else {
		returnVec[0] = { 1 }; //boolean:exists valid solutions
		if (areJointsValid(solutions[0][0], solutions[0][1], solutions[0][2], solutions[0][3]) == 1){ //this is the valid solution
			returnVec.push_back(solutions[0]);
			returnVec.push_back(solutions[1]);
			printf("distance for solution: %.4lf\nDistance for invalid solution: %.4lf\n\n", distances[0], distances[1]);

		}
		else {
			returnVec.push_back(solutions[1]);
			returnVec.push_back(solutions[0]);
			printf(" distance for solution: %.4lf\nDistance for invalid solution: %.4lf\n\n", distances[1], distances[0]);

		}	
	}	
	return returnVec;
}

vector<double> TransformMatrix::solve(double x, double y, double z, double phi, TransformMatrix StoB, TransformMatrix WtoT)
{
	TransformMatrix StoT = TransformMatrix::userFormToTransformMatrix(x, y, z, phi);
	TransformMatrix invStoB = StoB.getInverseTransform();
	TransformMatrix invWtoT = WtoT.getInverseTransform();
	TransformMatrix BtoW = invStoB * StoT * invWtoT;

	JOINT currentJointVars;
	GetConfiguration(currentJointVars);
	//output of invKin is in radians for joint angles
	vector<vector<double>> solutions = invKinBaseToWrist(BtoW, currentJointVars);

	if (solutions[0][0] != 0) {
		vector<double> closest = solutions[1];

		for (int i = 0; i < closest.size(); i++)
		{
			if (i != 2) {
				closest[i] = RAD2DEG(closest[i]);
			}
		}
		cout << "The closest solution that does not violate joint limits is:\n";
		cout << "Theta 1 = " << customRound(closest[0]);
		cout << "\nTheta 2 = " << customRound(closest[1]);
		cout << "\nDist 3 = " << customRound(closest[2]);
		cout << "\nTheta 4 = " << customRound(closest[3]) << endl << endl;

		return closest;
	}
	else {
		return { 0 };
	}
}

vector<matrixDouble> Planner(matrixDouble positions, double time, bool &canMove, vector<string> &issues) { //TODO::is member function?
	double timeSegment = time / 4;
	for (int viaPoint = 0; viaPoint < positions.size(); viaPoint++) {
		for (int joint = 0; joint < 4; joint++) {
			if (joint == 2) {
				if (positions[viaPoint][joint] > maxJointLimits[joint] || positions[viaPoint][joint] < minJointLimits[joint]) {
					canMove = false;
					string issueString = "via point " + to_string(viaPoint) + "violates the joint limits for joint" + to_string(joint) +
						". The requested value is " + to_string(positions[viaPoint][joint]) + "and it must be between " + to_string(maxJointLimits[joint]) + "and " + to_string(minJointLimits[joint]);
					issues.push_back(issueString);
				}
			}
			else {
				if (positions[viaPoint][joint] > maxJointLimits[joint] || (positions[viaPoint][joint]) < minJointLimits[joint]) {
					canMove = false;
					string issueString = "via point " + to_string(viaPoint) + "violates the joint limits for joint" + to_string(joint) +
						". The requested value is " + to_string(positions[viaPoint][joint]) + "and it must be between " + to_string(maxJointLimits[joint]) + "and " + to_string(minJointLimits[joint]);
					issues.push_back(issueString);
				}
			}
		}
	}
	matrixDouble theta1;
	matrixDouble theta2;
	matrixDouble d3;
	matrixDouble theta4;
	vector<matrixDouble>coefMtx = { theta1, theta2, d3, theta4 }; //CHANGE LOCATION OF DEFINITION

	//velocity calculations
	matrixDouble segVelocityMtx;
	for (int joint = 0; joint < 4; joint++) {
		vector<double> segmentVelocityPerJoint;
		for (size_t i = 0; i < 4; i++){
			double currentPosition = positions[i][joint];
			double nextPosition = positions[i + 1][joint];
			double delta = nextPosition - currentPosition;
			/*if( joint!= 2) {
				if (delta < -180) {
					delta = delta + 360;
				}
				if (delta > 180) {
					delta = delta - 360;
				}
			}*/
			double segmentVelocity = delta / timeSegment;
			segmentVelocityPerJoint.push_back(segmentVelocity);
		}
		segVelocityMtx.push_back(segmentVelocityPerJoint);
	}
	matrixDouble viaPointVelocityMtx;
	//calculate via point velocity
	for (int joint = 0; joint < 4; joint++) {
		vector<double> viaPointVelocity;
		viaPointVelocity.push_back(0);
		for (int i = 0; i < 3; i++) {
			double prevSegVel = segVelocityMtx[joint][i];
			double nextSegVel = segVelocityMtx[joint][i + 1];
			double pointVelocity = 0.5 * (prevSegVel + nextSegVel);
			viaPointVelocity.push_back(pointVelocity);
		}
		viaPointVelocity.push_back(0);
		viaPointVelocityMtx.push_back(viaPointVelocity);
	}


	for (int viaPoint = 0; viaPoint < 4; viaPoint++) {
		for (int joint = 0; joint < 4; joint++) {
			double current = positions[viaPoint][joint];
			double next = positions[viaPoint + 1][joint];
			double delta = next - current;
			//if (joint != 2) { //joint 3
			//	if (delta < -180) {
			//		delta = delta + 360;
			//	}
			//	if (delta > 180) {
			//		delta = delta - 360;
			//	}
			//}

			if (timeSegment < max(3 * abs(delta) / (2 * maxVelocity[joint]), sqrt(6 * abs(delta) / maxAcceleration[joint]))) {
				//timing not feasible
				canMove = false;
				string issuesString = "timing is not feasible for joint " + to_string(joint + 1) + "what more can i say idk\n";
				issues.push_back(issuesString);
			}
			double a0 = current;
			double a1 = viaPointVelocityMtx[joint][viaPoint];
			double a2 = ((delta * double(3)) / (pow(timeSegment, 2))) - ((2 * viaPointVelocityMtx[joint][viaPoint]) / timeSegment) - (viaPointVelocityMtx[joint][viaPoint + 1] / timeSegment); //check last time segment
			double a3 = ((-2 * delta) / (pow(timeSegment, 3))) + ((viaPointVelocityMtx[joint][viaPoint + 1] + viaPointVelocityMtx[joint][viaPoint]) / (pow(timeSegment, 2)));

			vector<double> coefficents = { a0, a1, a2, a3 };
			coefMtx[joint].push_back(coefficents);

		}
	}
	return coefMtx;
}
