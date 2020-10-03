#include "lidar.h"
#include <unistd.h>
#include <sstream>
#include <string>

using namespace std;
using namespace rp::standalone::rplidar;

#define PI 3.14159265

// Plages de vitesse 
#define DISTANCE_MAX_SPEED 700
#define DISTANCE_AVG_SPEED 600
#define DISTANCE_LOW_SPEED 450

// Etats des obstacles 
#define ETAT_OBSTACLE_NONE 3
#define ETAT_OBSTACLE_EXIST 2
#define ETAT_OBSTACLE_CLOSE 1
#define ETAT_OBSTACLE_DANGEROUS 0

// Vitesse 

// Angle de détection de 80°
#define ANGLE_ROBALLS 80

// Angle de détection de 95°
#define ANGLE_MILHABOT 95

// Angle de détection de 90°
#define ANGLE_KRABS 120

// Angle de détection de 90°
#define ANGLE_KIROULPA 90

// Etats de detections
#define DETECTION_AUCUNE  0
#define DETECTION_AVANT   1
#define DETECTION_ARRIERE 2







Lidar::Lidar()
{
	cout << "Création de l'objet Lidar..." << endl;
	
	drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);

	/*stringstream port;
	port << "/dev/ttyUSB" << PORT_USB_NUM;
	string strPort = port.str();
	char charPort[strPort.size() + 1];
	strPort.copy(charPort,strPort.size()+1);
	charPort[strPort.size()] = '\0';*/

	


	do 
	{
		if (IS_OK(drv->connect("/dev/ttyLIDAR", 115200))) //256000 sinon
		{
			op_result = drv->getDeviceInfo(devinfo);

			if (IS_OK(op_result))
			{
				cout << "lidar connected" << endl;
				//drv->startMotor();

			}
			else
			{
				delete drv;
				drv = NULL;
				cout << "lidar not connected" << endl;

			}
		}

		if (!checkRPLIDARHealth(drv)) {
			RPlidarDriver::DisposeDriver(drv);
			drv = NULL;
			cout << "Rip" << endl;
		}

	} while (!checkRPLIDARHealth(drv));

	drv->startMotor();
	
}

Lidar::~Lidar()
{
	drv->stopMotor();
	cout << "stop motor lidar Destructeur" << endl;
	RPlidarDriver::DisposeDriver(drv);

}

void Lidar::stopMotor()
{
	drv->stopMotor();
	cout << "stop motor lidar " << endl;
}




bool Lidar::checkRPLIDARHealth(RPlidarDriver * drv)
{
	u_result     op_result;
	rplidar_response_device_health_t healthinfo;


	op_result = drv->getHealth(healthinfo);
	if (IS_OK(op_result)) { // the macro IS_OK is the preperred way to judge whether the operation is succeed.
		printf("RPLidar health status : %d\n", healthinfo.status);
		if (healthinfo.status == RPLIDAR_STATUS_ERROR) {
			fprintf(stderr, "Error, rplidar internal error detected. Please reboot the device to retry.\n");
			// enable the following code if you want rplidar to be reboot by software
			 drv->reset();
			return false;
		}
		else {
			return true;
		}

	}
	else {
		fprintf(stderr, "Error, cannot retrieve the lidar health code: %x\n", op_result);
		return false;
	}
}


void Lidar::Scan()
{
		

	count = _countof(nodes);

	
    // start scan...
    drv->startScan(0,1);

	int i =0;
	
	int etat = 4;
	
	int distance_min;


	while(1){
		
		distance_min = 65000;
		op_result = drv->grabScanDataHq(nodes, count);
		if (IS_OK(op_result)) 
		{
			
			drv->ascendScanData(nodes, count);	
			for (int pos = 1; pos < (int)count ; ++pos) 
    		{				
            	if(distance_min  > (nodes[pos].dist_mm_q2 / cos(nodes[pos].angle_z_q14 * 90.f / 180 * PI)) / 4.0f &&
					(nodes[pos].dist_mm_q2 / cos(nodes[pos].angle_z_q14 * 90.f / 180 * PI)) / 4.0f != 0.0){
            		distance_min = nodes[pos].dist_mm_q2 / 4.0f; //obstacle le plus proche
            		distance = distance_min; 
            		angle = nodes[pos].angle_z_q14 * 90.f; //angle de l'obstacle le plus proche
            	}         		

            	if(distance_min >= DISTANCE_MAX_SPEED){
            		etat = ETAT_OBSTACLE_NONE; // Pas d'obstacle
            	}
            	else if(distance_min >= DISTANCE_AVG_SPEED){
            		etat = ETAT_OBSTACLE_EXIST; // Un obstacle dans le coin
            	}	
            	
            	else if(distance_min >= DISTANCE_LOW_SPEED){
            		etat = ETAT_OBSTACLE_CLOSE; // Un obstacle proche du robot
            	}
            	else {
            		etat = ETAT_OBSTACLE_DANGEROUS; // Un obstacle qui necessite de stoper les moteurs
            	}	
    		}
    	}

		
			if((angle < (ANGLE_KRABS / 2) || angle > (360 - ANGLE_KRABS / 2)) && Etat_Detection == 1){
                speed = etat;
            }
            else if(angle > (180 - ANGLE_KRABS / 2) && angle < (180 + ANGLE_KRABS / 2) && Etat_Detection == -1){
                speed = etat;
            }
            else{
                speed = ETAT_OBSTACLE_NONE; // Détection active mais on trouve rien dans la direction ou on va
            }
		
		
    	

    }
	
}


