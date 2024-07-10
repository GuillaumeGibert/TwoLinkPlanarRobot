#include <chrono>
#include <thread>

#include "Kinematics.h"
#include "DynamixelHandler.h"

// TO BE ADJUSTED IF NECESSARY
float _fDistanceThreshold = 0.1f; // in cm
float _pController = 0.1f;
float _fps = 20.0f; // in Hz

// Dynamixel config infos
DynamixelHandler _oDxlHandler;
std::string _robotDxlPortName = "/dev/ttyUSB0";
float _robotDxlProtocol = 2.0;
int _robotDxlBaudRate = 1000000;


void initRobot(DynamixelHandler& dxlHandler, std::string portName, float protocol, int baudRate)
{
	std::cout << "===Initialization of the Dynamixel Motor communication====" << std::endl;
	dxlHandler.setDeviceName(portName);
	dxlHandler.setProtocolVersion(protocol);
	dxlHandler.openPort();
	dxlHandler.setBaudRate(baudRate);
	dxlHandler.enableTorque(true);
	std::cout << std::endl;
}

void closeRobot(DynamixelHandler& dxlHandler)
{
	dxlHandler.enableTorque(false);
	dxlHandler.closePort();
}

int main(int argc, char** argv)
{
	if (argc == 5)
	{
		// Retrieves the args and stores them into variables
		float L1 = atof(argv[1]); // in cm
		float L2 = atof(argv[2]); // in cm
		float x = atof(argv[3]); // in cm
		float y = atof(argv[4]); // in cm
		float qpen = deg2rad(-90); // in rad
			
		// Initializes the robot
		initRobot(_oDxlHandler, _robotDxlPortName, _robotDxlProtocol, _robotDxlBaudRate);
		
		// Changes the control mode
		//_oDxlHandler.setControlMode(1); //wheel mode
		// VELOCITY CONTROL qdot
		
		// Gets the initial joint positions
		std::vector<float> l_vCurrentJointPosition;
		_oDxlHandler.readCurrentJointPosition(l_vCurrentJointPosition);
		
		// Inits joint velocities
		std::vector<float> l_vCurrentJointVelocity; l_vCurrentJointVelocity.push_back(0.0f); l_vCurrentJointVelocity.push_back(0.0f); l_vCurrentJointVelocity.push_back(0.0f);
		// VELOCITY CONTROL qdot
		
		// Inits the loop
		bool bIsFarFromTarget = true;
				
		while (bIsFarFromTarget)
		{
			// Estimates the position of the end-effector using FK
			std::vector<float> vEndEffectorPosition = computeForwardKinematics(l_vCurrentJointPosition[0], l_vCurrentJointPosition[2], L1, L2);
			// Computes deltaX
			std::vector<float> deltaX{x-vEndEffectorPosition[0], y - vEndEffectorPosition[1]};
			// Computes Jacobian matrix
			std::vector<float> vJacobianMatrix = computeDifferentialKinematics(l_vCurrentJointPosition[0], l_vCurrentJointPosition[2], L1, L2);
			// Computes Inverse of Jacobian matrix
			cv::Mat vInverseJacobianMatrix = computeInverseJacobianMatrix(vJacobianMatrix);
			// Determines deltaX
			cv::Mat1f oDeltaX(2, 1);
			oDeltaX.at<float>(0, 0) = _pController * deltaX[0];
			oDeltaX.at<float>(1, 0) = _pController * deltaX[1];
			// Computes deltaQ
			cv::Mat deltaQ = vInverseJacobianMatrix * oDeltaX;
			// Updates the current joint position
			l_vCurrentJointPosition[0] += deltaQ.at<float>(0,0);
			l_vCurrentJointPosition[2] += deltaQ.at<float>(1,0);
			// Uodates the target joint velocity
			l_vCurrentJointVelocity[0] = deltaQ.at<float>(0,0)*_fps;
			l_vCurrentJointVelocity[2] = deltaQ.at<float>(1,0)*_fps;
			// VELOCITY CONTROL qdot
			// Creates the target joint position vector
			std::vector<float> vTargetJointPosition;
				vTargetJointPosition.push_back(l_vCurrentJointPosition[0]); 
				vTargetJointPosition.push_back(qpen); 
				vTargetJointPosition.push_back(l_vCurrentJointPosition[2]); 
			// Sends the target joint position to actuate the robot
			_oDxlHandler.sendTargetJointPosition(vTargetJointPosition);
			
			// Creates the target joint velocity vector
			std::vector<float> vTargetJointVelocity;
				vTargetJointVelocity.push_back(l_vCurrentJointVelocity[0]); 
				vTargetJointVelocity.push_back(0.0f); // pen lift up/down
				vTargetJointVelocity.push_back(l_vCurrentJointVelocity[2]); 
			// Sends the target joint position to actuate the robot
			//_oDxlHandler.sendTargetJointVelocity(vTargetJointVelocity);
			// VELOCITY CONTROL qdot
			
			// Waits 1000/fps ms
			std::this_thread::sleep_for(std::chrono::milliseconds((long int)(1000.0f/_fps)));
			
			// Estimates the distance to the target
			float currentDistanceToTarget = sqrt(pow(y - vEndEffectorPosition[1],2) + pow(x - vEndEffectorPosition[0], 2));
			std::cout << "====LINEAR CONTROL====" << std::endl;
			std::cout << "Current Joint Position in deg (q1, q2)= (" << rad2deg(l_vCurrentJointPosition[0]) << ", " << rad2deg(l_vCurrentJointPosition[2]) << ")" << std::endl;
			std::cout << "Current Joint Velocity in deg/s (q1, q2)= (" << rad2deg(l_vCurrentJointVelocity[0]) << ", " << rad2deg(l_vCurrentJointVelocity[2]) << ")" << std::endl;
			std::cout << "Current Cartesian Position (x, y)= (" << vEndEffectorPosition[0] << ", " << vEndEffectorPosition[0] << ")" << std::endl;
			std::cout << "Current Distance To Target= " << currentDistanceToTarget << std::endl;
			std::cout << "==================" << std::endl;

			// Determines if the loop should continue
			if (currentDistanceToTarget < _fDistanceThreshold)
			{
				bIsFarFromTarget = false;
			}
		}
		
		// Waits 2s
		std::this_thread::sleep_for(std::chrono::milliseconds(2000));

		// Closes robot connection
		_oDxlHandler.closePort();
		
	}
	else
	{
		std::cout << "[WARNING] Cartesian control" << std::endl;
		std::cout << ">> linearControl L1(cm) L2(cm) x(cm) y(cm)"<< std::endl;
	}
		
	return 0;
}