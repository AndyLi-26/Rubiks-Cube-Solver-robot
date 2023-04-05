#include <ctime>
#include <iostream>
// #include <raspicam/raspicam_cv.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <stdio.h>
#include <string>
//#include <wiringPi.h>
//#include <wiringSerial.h>
#include <unistd.h>
#include <bcm2835.h>

#include "ColorDetection.hpp"
#include "CameraVertices.hpp"
#include "CubeDisplay.hpp"

#include "solving_algorithm/CubeParser.hpp"
#include "solving_algorithm/FaceletCube.hpp"
#include "solving_algorithm/RubiksCube.hpp"
#include "solving_algorithm/Solver.hpp"

using namespace std; 
using namespace cv;
static const char* portName = "/dev/ttyS0";

char solution_global[100];


void bcm2835_set_debug	(uint8_t debug);	



int main ( int argc,char **argv ) {
   
	time_t timer_begin,timer_end;
	//-USB WEBCAM-//
	VideoCapture cap2; // top cam
	VideoCapture cap0; // btm cam
	
	cap2.open(0);
	cap0.open(2);
	
	Mat frame_top;
	Mat frame_btm;
	
	Mat rubiks_cube = Mat::zeros(280, 365, CV_8UC3 ); // Setup size
	rubiks_cube = Scalar(255,255,255); // Make window white
	drawFaces(rubiks_cube); //draw outline of facelets
	
	Mat img_top_hsv;
	Mat img_btm_hsv;
	//Mat image_thrs;

	//Start capture
	time ( &timer_begin );
	namedWindow("Top Camera", 1 );
	namedWindow("Bottom Camera", 1 );
	namedWindow("Rubiks Cube", 1);
	
	string facelets[6];
	
	unsigned int status; // Solver status    
 
	
	do {
		cap2.read(frame_top); //Returns true false
		cap0.read(frame_btm);
		
		// Convert from BGR to HSV colorspace
		cvtColor(frame_top, img_top_hsv, cv::COLOR_BGR2HSV);
		cvtColor(frame_btm, img_btm_hsv, cv::COLOR_BGR2HSV);
		
		// Detect the object based on HSV Range Values
		// openCV uses H: 0-179, S: 0-255, V: 0-255
			
	
		facelets[0] = FaceletToFaces(img_top_hsv,frame_top,0); // Scan U face
		facelets[1] = FaceletToFaces(img_btm_hsv,frame_btm,1); // Scan D face
		facelets[2] = FaceletToFaces(img_btm_hsv,frame_btm,2); // Scan F face ra
		facelets[3] = FaceletToFaces(img_top_hsv,frame_top,3); // Scan B face
		facelets[4] = FaceletToFaces(img_btm_hsv,frame_btm,4); // Scan L face
		facelets[5] = FaceletToFaces(img_top_hsv,frame_top,5); // Scan R face

		// Display detected colors on screen
		fillFacelet(rubiks_cube, facelets);
		
		// display video
		imshow("Top Camera", frame_top);
		imshow("Bottom Camera", frame_btm);
		
		// display rubiks cube layout
		imshow("Rubiks Cube", rubiks_cube);

		
	}while ( -1 == cv::waitKey(10) ); // pressing any key wil exit the loop
	cout<<"Stop camera..."<<endl;
	//Camera.release();
	//show time statistics
	time ( &timer_end ); /* get current time; same as: timer = time(NULL)  */
	double secondsElapsed = difftime ( timer_end,timer_begin );
	cout<< secondsElapsed<<" seconds elapsed" <<endl;
	
	// Initiate Solver
	// Parse the input and initialize FaceletCube
	FaceletCube faceletCube;
	CubeParser cubeParser;
	if((status = cubeParser.parseFacelets(facelets, faceletCube)) != CubeParser::VALID) {
	cout << cubeParser.ErrorText(status) << endl;
	return 1;
	}

	// Validate the FaceletCube
	RubiksCube cube;
	if((status = faceletCube.Validate(cube)) != FaceletCube::VALID) {
	cout << faceletCube.ErrorText(status) << endl;
	return 1;
	}
	// Cube is in a valid configuration at this point


	// Initialize tables and solve
	Solver solver;
	solver.InitializeTables();
	solver.Solve(cube);

	// Cube solved
	
	sleep(100); //
	// Save solution
	
	char moveset_sample[100];
    string solution_string;
    ifstream myfile("solution.txt");
    getline(myfile, solution_string);
    cout << "solution to be sent is: " <<endl;
    cout << solution_string << endl;
    strcpy(moveset_sample, solution_string.c_str());
    
	
	// UART comms to PSOC
	//wiringPiSetup ();
	//int serialPort; // File descriptor for serial port

	// Open the serial port 
	//serialPort = serialOpen(portName, 115200);
	//printf ("%i\n",serialPort) ;
	
	//int moveset_size = sizeof moveset_sample / sizeof *moveset_sample;
	//for(int i=0; i<moveset_size; i++){
		//char c = moveset_sample[i];
		/*if(c != ' '){
		serialPutchar(serialPort, c);
		delay(100);
		}
		
	}
	serialPutchar(serialPort, '+');

	cout << "solution sent" << endl;

	serialClose(serialPort);
	
	return 0;*/
	
	
	
	
	if (!bcm2835_init())
    {
      printf("bcm2835_init failed. Are you running as root??\n");
      return 1;
    }
 
    if (!bcm2835_spi_begin())
    {
      printf("bcm2835_spi_begin failed. Are you running as root??\n");
      return 1;
    }
    bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);      // The default
    bcm2835_spi_setDataMode(BCM2835_SPI_MODE0);                   // The default
    bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_65536); // The default
    bcm2835_spi_chipSelect(BCM2835_SPI_CS0);                      // The default
    bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, LOW);      // the default
    
    
	uint8_t send_data = 0x23;
    uint8_t read_data = bcm2835_spi_transfer(send_data);
    printf("Sent to SPI: 0x%02X. Read back from SPI: 0x%02X.\n", send_data, read_data);
    if (send_data != read_data)
      printf("Do you have the loopback from MOSI to MISO connected?\n");
    bcm2835_spi_end();
    bcm2835_close();
    return 0;

}

