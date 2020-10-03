#ifndef LIDAR_H
#define LIDAR_H


#include "rplidar.h" //RPLIDAR standard sdk, all-in-one header
#include <stdio.h>
#include <stdlib.h>
#include <ostream>
#include <iostream>
#include <signal.h>
#include <math.h>


using namespace std;
using namespace rp::standalone::rplidar;
#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif





class Lidar
{
public:
	
	size_t count;
	int Etat_Lidar;
	int Etat_Detection; //Avant, Arriere, ou Aucune
	int speed;
	
	//Constructeur et destruteur par defaut
	//Lidar();
	Lidar();
	~Lidar();

	//Verifie que le lidar est present et branche correctement
	bool checkRPLIDARHealth(RPlidarDriver* drv);

	//Lance un scan de 0 a 360 deg et stoque les valeurs dans nodes[]
	void Scan();
	rplidar_response_measurement_node_hq_t nodes[8192];
	
	void stopMotor();


private:
	RPlidarDriver * drv;
	u_result op_result;
	rplidar_response_device_info_t devinfo;
	
	float angle;
	float distance;
	
	int Obstacle_Proche_Distance;
	int Obstacle_Proche_Angle;
	



};

#endif // LIDAR_H