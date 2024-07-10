# TwoLinkPlanarRobot

## About The Project

[![Project Screen Shot][project-screenshot]]()

This repository contains a series of code for controlling a 2-link planar robot using the [Dynamixel SDK](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/) to experiment with Forward, Inverse and Differential Kinematics. The [OpenCV library](https://opencv.org/) is also used to extract image contours that are then drawn by the robot.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

### Built With

* [![C++][cpp-shield]][cpp-url]
* [![OpenCV][opencv-shield]][opencv-url]

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- GETTING STARTED -->
## Getting Started

To set up the project locally, you need to install (if not already the case) some dependencies.
To get a local copy up and running follow these steps.

### Prerequisites

* C++ Compiler

Install the build-essential package
  ```sh
  sudo apt install build-essential 
  ```
  
* OpenCV

Install the dev library
  ```sh
  sudo apt install libopencv-dev 
  ```
  
  * Dynamixel SDK
  
 Download the Dynamixel SDK
 ```sh
 git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
 ```
 
 Go to the c++/build folder
  ```sh
 cd DynamixelSDK/c++/build/linux64
 ```

 Compile and install
  ```sh
 make
 sudo make install
 ```
 
 Upgrade USB access privileges by aading your account to the dialout group
   ```sh
  sudo usermod -aG dialout <your_account_id>
 ```
 

### Installation

1. Clone the repo
   ```sh
   git clone https://github.com/GuillaumeGibert/TwoLinkPlanarRobot.git
   ```
2. Open a terminal
3. Compile/Link by calling the makefile
 ```sh
   make
   ```

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- USAGE EXAMPLES -->
## Usage

### Joint control

1. Open a terminal
2. Launch the camera calibration executable
```sh
./bin/jointControl 5 6 0 0 -90
```
3. The arguments of this executable program are: L1 (in cm), L2 (in cm), q1 (in deg), q2 (in deg), qpen (in deg). qpen should only modify to lift up (0°) / down (-90°) the pen.


### Cartesian control

1. Open a terminal
2. Launch the cartesian control executable
```sh
./bin/cartControl 5 6 9 -2
```
The arguments of this executable program are: L1 (in cm), L2 (in cm), x (in cm), y (in cm).

### Linear control

1. Open a terminal
2. Launch the cartesian control executable
```sh
./bin/linearControl 5 6 9 2
```
The arguments of this executable program are: L1 (in cm), L2 (in cm), x (in cm), y (in cm).

### Image contour drawing

1. Open a terminal
2. Launch the color tracking executable
```sh
./bin/drawImage 5 6 ../data/python-logo.png
```
The arguments of this executable program are: L1 (in cm), L2 (in cm), the image filename.


<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- LICENSE -->
## License

Distributed under the GPL License. See `LICENSE.txt` for more information.

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- CONTACT -->
## Contact

Guillaume Gibert

Project Link: [https://github.com/GuillaumeGibert/TwoLinkPlanarRobot](https://github.com/GuillaumeGibert/TwoLinkPlanarRobot)

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[arduino-shield]: https://img.shields.io/badge/Arduino_IDE-00979D?style=for-the-badge&logo=arduino&logoColor=white
[arduino-url]: https://www.arduino.cc/
[python-shield]: https://img.shields.io/badge/Python-3776AB?style=for-the-badge&logo=python&logoColor=white
[python-url]: https://www.python.org/
[opencv-shield]: https://img.shields.io/badge/OpenCV-27338e?style=for-the-badge&logo=OpenCV&logoColor=white
[opencv-url]: https://opencv.org/
[cpp-shield]: https://img.shields.io/badge/-C++-blue?logo=cplusplus
[cpp-url]: https://isocpp.org/

[project-screenshot]: images/screenshot.png

[contributors-shield]: https://img.shields.io/github/contributors/GuillaumeGibert/TwoLinkPlanarRobot.svg?style=for-the-badge
[contributors-url]: https://github.com/GuillaumeGibert/TwoLinkPlanarRobot/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/GuillaumeGibert/TwoLinkPlanarRobot.svg?style=for-the-badge
[forks-url]: https://github.com/GuillaumeGibert/TwoLinkPlanarRobot/network/members
[stars-shield]: https://img.shields.io/github/stars/GuillaumeGibert/TwoLinkPlanarRobot.svg?style=for-the-badge
[stars-url]: https://github.com/GuillaumeGibert/TwoLinkPlanarRobot/stargazers
[issues-shield]: https://img.shields.io/github/issues/GuillaumeGibert/TwoLinkPlanarRobot.svg?style=for-the-badge
[issues-url]: https://github.com/GuillaumeGibert/TwoLinkPlanarRobot/issues
[license-shield]: https://img.shields.io/github/license/GuillaumeGibert/TwoLinkPlanarRobot.svg?style=for-the-badge
[license-url]: https://github.com/GuillaumeGibert/TwoLinkPlanarRobot/blob/master/LICENSE.txt
[linkedin-shield]: https://img.shields.io/badge/-LinkedIn-black.svg?style=for-the-badge&logo=linkedin&colorB=555
[linkedin-url]: https://linkedin.com/in/guillaume-gibert-06502ba4