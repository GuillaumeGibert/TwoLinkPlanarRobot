#include <chrono>
#include <thread>

#include "Kinematics.h"
#include "ImageProcessing.h"


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
	if (argc == 4)
	{
		// Retrieves the args and stores them into variables
		float L1 = atof(argv[1]); // in cm
		float L2 = atof(argv[2]); // in cm
		std::string imgFilename = argv[3]; // image filename
			
		// Initializes the robot
		initRobot(_oDxlHandler, _robotDxlPortName, _robotDxlProtocol, _robotDxlBaudRate);
		
		// Loads the image
		cv::Mat l_oRawImg = cv::imread(imgFilename);
		if(l_oRawImg.empty())
		{
			std::cout << "[ERROR] Could not read the image: " << imgFilename << std::endl;
			return 1;
		}
		
		// Extracts its contours in the pixel reference frame
		std::vector<std::vector<cv::Point>> vContoursInPixel = imageProcessing(l_oRawImg);
		
		// Converts the contours to the robot reference frame
		float theta = -180.0f/180.0f*M_PI; float tx = 110; float ty = 20; int imageWidth =  l_oRawImg.cols; int imageHeight = l_oRawImg.rows;
		std::vector<std::vector<float>> vContoursInMm = convertContoursPixel2Mm(vContoursInPixel, theta, tx, ty, imageWidth, imageHeight);
		
		// Sends the contours to the robot 
		sendContours(vContoursInPixel, vContoursInMm, _oDxlHandler, L1, L2);
		
		// Waits 2s
		std::this_thread::sleep_for(std::chrono::milliseconds(2000));

		// Closes robot connection
		_oDxlHandler.closePort();
		
	}
	else
	{
		std::cout << "[WARNING] Draw image" << std::endl;
		std::cout << ">> drawImage L1(cm) L2(cm) img_filename"<< std::endl;
	}
		
	return 0;
}