#include <iostream>

#include "Kinematics.h"

#define ERROR_THRESHOLD 0.00001

std::vector<float> computeForwardKinematics(float q1, float q2, float L1, float L2);
std::vector<float> computeInverseKinematics(float x, float y, float L1, float L2);
std::vector<float> computeDifferentialKinematics(float q1, float q2, float L1, float L2);


bool testFK(float q1, float q2, float L1, float L2, std::vector<float> expectedEndEffectorPosition, float errorThreshold)
{
	std::vector<float> calculatedEndEffectorPosition = computeForwardKinematics(q1, q2, L1, L2);
	
	if (expectedEndEffectorPosition.size() != calculatedEndEffectorPosition.size())
	{
		std::cout << "[ERROR] (testFK) expected results and calculated ones have different sizes!" << std::endl;
		return false;
	}
	
	float error = 0.0f;
	for (int l_coord=0; l_coord < expectedEndEffectorPosition.size(); l_coord++)
	{
		error += abs(expectedEndEffectorPosition[l_coord] - calculatedEndEffectorPosition[l_coord]);
	}
	std::cout << "[INFO] (testFK) error = " << error << std::endl;
	
	if (error < errorThreshold)
		return true;
	else
		return false;
}

bool testIK(float x, float y, float L1, float L2, std::vector<float> expectedJointValues, float errorThreshold)
{
	
	std::vector<float> calculatedJointValues = computeInverseKinematics(x, y, L1, L2);
	
	if (expectedJointValues.size() != calculatedJointValues.size())
	{
		std::cout << "[ERROR] (testIK) expected results and calculated ones have different sizes!" << std::endl;
		return false;
	}
	
	float error = 0.0f;
	for (int l_coord=0; l_coord < expectedJointValues.size(); l_coord++)
	{
		error += abs(expectedJointValues[l_coord] - calculatedJointValues[l_coord]);
	}
	std::cout << "[INFO] (testIK) error = " << error << std::endl;
	
	if (error < errorThreshold)
		return true;
	else
		return false;
}

int main()
{
	// Tests Forward Kinematics
	std::cout << "====TESTING FORWARD KINEMATICS====" << std::endl;
	std::cout << "-> Test #1......"<< std::endl;
	float q1= deg2rad(0.0f); float q2 = deg2rad(0.0f); float L1 = 5.0f; float L2 = 6.0f; std::vector<float> expectedEndEffectorPosition{11.0f, 0.0f};
	bool isTestPassed = testFK(q1, q2, L1, L2, expectedEndEffectorPosition, ERROR_THRESHOLD);
	if (isTestPassed)
		std::cout << "PASSED!" << std::endl;
	else
		std::cout << "FAILED!" << std::endl;
	
	std::cout << "-> Test #2......"<< std::endl;
	q1= deg2rad(90.0f); q2 = deg2rad(0.0f); L1 = 5.0f; L2 = 6.0f; expectedEndEffectorPosition[0] = 0.0f;  expectedEndEffectorPosition[1] = 11.0f;
	isTestPassed = testFK(q1, q2, L1, L2, expectedEndEffectorPosition, ERROR_THRESHOLD);
	if (isTestPassed)
		std::cout << "PASSED!" << std::endl;
	else
		std::cout << "FAILED!" << std::endl;
	
	std::cout << "-> Test #3......"<< std::endl;
	q1= deg2rad(0.0f); q2 = deg2rad(90.0f); L1 = 5.0f; L2 = 6.0f; expectedEndEffectorPosition[0] = 5.0f;  expectedEndEffectorPosition[1] = 6.0f;
	isTestPassed = testFK(q1, q2, L1, L2, expectedEndEffectorPosition, ERROR_THRESHOLD);
	if (isTestPassed)
		std::cout << "PASSED!" << std::endl;
	else
		std::cout << "FAILED!" << std::endl;
	
	// Tests Inverse Kinematics
	std::cout << "====TESTING INVERSE KINEMATICS====" << std::endl;
	std::cout << "-> Test #1......"<< std::endl;
	float x= 11.0f; float y = 0.0f; L1 = 5.0f; L2 = 6.0f; expectedEndEffectorPosition.resize(3); expectedEndEffectorPosition[0] = 1.0f;  expectedEndEffectorPosition[1] = deg2rad(0.0f); expectedEndEffectorPosition[2] = deg2rad(0.0f); 
	isTestPassed = testIK(x, y, L1, L2, expectedEndEffectorPosition, ERROR_THRESHOLD);
	if (isTestPassed)
		std::cout << "PASSED!" << std::endl;
	else
		std::cout << "FAILED!" << std::endl;
	
	std::cout << "-> Test #2......"<< std::endl;
	x= 0.0f; y = 11.0f; L1 = 5.0f; L2 = 6.0f; expectedEndEffectorPosition.resize(3); expectedEndEffectorPosition[0] = 1.0f;  expectedEndEffectorPosition[1] = deg2rad(90.0f); expectedEndEffectorPosition[2] = deg2rad(0.0f); 
	isTestPassed = testIK(x, y, L1, L2, expectedEndEffectorPosition, ERROR_THRESHOLD);
	if (isTestPassed)
		std::cout << "PASSED!" << std::endl;
	else
		std::cout << "FAILED!" << std::endl;
	
	std::cout << "-> Test #3......"<< std::endl;
	x= 5.0f; y = 6.0f; L1 = 5.0f; L2 = 6.0f; expectedEndEffectorPosition.resize(5); expectedEndEffectorPosition[0] = 2.0f;  expectedEndEffectorPosition[1] = deg2rad(0.0f); expectedEndEffectorPosition[2] = deg2rad(90.0f); expectedEndEffectorPosition[3] = deg2rad(100.389f); expectedEndEffectorPosition[4] = deg2rad(-90.0f); 
	isTestPassed = testIK(x, y, L1, L2, expectedEndEffectorPosition, ERROR_THRESHOLD);
	if (isTestPassed)
		std::cout << "PASSED!" << std::endl;
	else
		std::cout << "FAILED!" << std::endl;
	
	std::cout << "-> Test #4......"<< std::endl;
	x= 12.0f; y = 0.0f; L1 = 5.0f; L2 = 6.0f; expectedEndEffectorPosition.resize(1); expectedEndEffectorPosition[0] = 0.0f;  
	isTestPassed = testIK(x, y, L1, L2, expectedEndEffectorPosition, ERROR_THRESHOLD);
	if (isTestPassed)
		std::cout << "PASSED!" << std::endl;
	else
		std::cout << "FAILED!" << std::endl;
	
	return 0;
}