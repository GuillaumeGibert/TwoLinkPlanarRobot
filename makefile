all: kinematics dynamics joint cartesian image test linear draw
	g++ -o bin/jointControl lib/jointControl.o lib/Kinematics.o lib/DynamixelHandler.o -L/usr/local/lib/ -ldxl_x64_cpp -lrt -L/usr/lib/x86_64-linux-gnu `pkg-config --libs opencv4`
	g++ -o bin/cartControl lib/cartControl.o lib/Kinematics.o lib/DynamixelHandler.o lib/ImageProcessing.o -L/usr/local/lib/ -ldxl_x64_cpp -lrt -L/usr/lib/x86_64-linux-gnu `pkg-config --libs opencv4`
	g++ -o bin/testKinematics lib/testKinematics.o lib/Kinematics.o `pkg-config --libs opencv4`
	g++ -o bin/linearControl lib/linearControl.o lib/Kinematics.o lib/DynamixelHandler.o lib/ImageProcessing.o -L/usr/local/lib/ -ldxl_x64_cpp -lrt -L/usr/lib/x86_64-linux-gnu `pkg-config --libs opencv4`
	g++ -o bin/drawImage lib/drawImage.o lib/Kinematics.o lib/DynamixelHandler.o lib/ImageProcessing.o -L/usr/local/lib/ -ldxl_x64_cpp -lrt -L/usr/lib/x86_64-linux-gnu `pkg-config --libs opencv4`
	
kinematics: src/Kinematics.cpp
	g++ -c src/Kinematics.cpp -o lib/Kinematics.o -I./include -I/usr/include/opencv4

test: src/testKinematics.cpp
	g++ -c src/testKinematics.cpp -o lib/testKinematics.o -I./include -I/usr/include/opencv4
	
joint: src/jointControl.cpp
	g++ -c src/jointControl.cpp -o lib/jointControl.o -I./include -I/usr/include/opencv4
	
cartesian: src/cartControl.cpp
	g++ -c src/cartControl.cpp -o lib/cartControl.o -I./include -I/usr/include/opencv4
	
linear: src/linearControl.cpp
	g++ -c src/linearControl.cpp -o lib/linearControl.o -I./include -I/usr/include/opencv4
	
image: src/ImageProcessing.cpp
	g++ -c src/ImageProcessing.cpp -o lib/ImageProcessing.o -I./include -I/usr/include/opencv4
	
dynamics: src/DynamixelHandler.cpp
	g++ -c src/DynamixelHandler.cpp -o lib/DynamixelHandler.o -I./include -I/usr/local/include
	
draw: src/drawImage.cpp
	g++ -c src/drawImage.cpp -o lib/drawImage.o -I./include -I/usr/include/opencv4
	
clean:
	rm lib/*.o
	rm bin/*