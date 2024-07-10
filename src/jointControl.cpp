#include <chrono>
#include <thread>

#include "Kinematics.h"
#include "DynamixelHandler.h"

// Global variables
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
	if (argc == 6)
	{
		// Retrieves the args and stores them into variables
		float L1 = atof(argv[1]); // in cm
		float L2 = atof(argv[2]); // in cm
		float q1 = deg2rad(atof(argv[3])); // in rad
		float q2 = deg2rad(atof(argv[4])); // in rad
		float qpen = deg2rad(atof(argv[5])); // in rad
		
		// Initializes the robot
		initRobot(_oDxlHandler, _robotDxlPortName, _robotDxlProtocol, _robotDxlBaudRate);
		
		// Computes FK
		computeForwardKinematics(q1, q2, L1, L2);
		
		// Computes Differential Kinematics
		std::vector<float> vJacobianMatrix = computeDifferentialKinematics(q1, q2, L1, L2);
		int rank =  computeJacobianMatrixRank(vJacobianMatrix, 0.1);
		
		// Sends the target joint values received as args
		std::vector<float> vTargetJointPosition;
			vTargetJointPosition.push_back(q1); 
			vTargetJointPosition.push_back(qpen); 
			vTargetJointPosition.push_back(q2); 
		_oDxlHandler.sendTargetJointPosition(vTargetJointPosition);
		//std::cout << "vTargetJointPosition= " << vTargetJointPosition[0] << ", " <<  vTargetJointPosition[1] << ", " << vTargetJointPosition[2]<< std::endl;
		// Waits 2s
		std::this_thread::sleep_for(std::chrono::milliseconds(2000));

		// Closes robot connection
		_oDxlHandler.closePort();
	}
	else
	{
		std::cout << "[WARNING] Joint control"<< std::endl;
		std::cout << ">> jointControl L1(cm) L2(cm) q1(deg) q2(deg) qpen(deg)"<< std::endl;
	}
		
	return 0;
}