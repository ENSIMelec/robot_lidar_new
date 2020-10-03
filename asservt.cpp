#include <iostream>
#include <thread>
#include <string>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>

#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <sys/time.h>

using namespace std;

#include "Controller.h"
#include "SerialCodeurManager.h"
#include "Strategie.h"
#include "Odometry.h"

/*********************************** Define ************************************/

#define EXIT_SUCCESS 0
#define EXIT_FAIL_PARAM -1
#define EXIT_FAIL_I2C -2
#define EXIT_FORCING_STOP -99

/***************************** Variables globales ******************************/

bool forcing_stop;
unsigned int deltaAsservTimer;

/********************************* prototypes **********************************/

bool argc_control(int argc);
bool argv_contains_dummy(int argc, char** argv);

void jouerMatch(Controller& controller, Strategie& strat, MoteurManager& moteurs, Odometry& odometry);

void stopSignal(int signal);
void sleepMillis(int millis);


///////////////////////////// PROGRAMME PRINCIPAL ///////////////////////////////
int main(int argc, char **argv) {
/***************************** Début du programme ******************************/
	cout << "Debut du programme" << endl;

/*************************** Controle des paramètres ***************************/
	cout << "Controle des parametres" << endl;
	if(!argc_control(argc))
		return EXIT_FAIL_PARAM;

    // Interception du Ctrl-C pour l'arret des moteurs
	signal(SIGINT, stopSignal);

/***************** Déclaration et initialisation des variables *****************/
	// Initialise les GPIO de la Rasp
	// ATTENTION: à appeler absolument avant d’initialiser les managers
	wiringPiSetupGpio();

	Config config;
	config.loadFromFile("config.info"); //Charge la configuration à partir du fichier config.info
    int I2C_MOTEURS = 8;

	int i2cM = wiringPiI2CSetup(I2C_MOTEURS);
	
	if(i2cM < 0)
		return EXIT_FAIL_I2C;
	
	MoteurManager moteurs(i2cM);


	cout << "Déclaration et initialisation des variables" << endl;


	argv_contains_dummy(argc, argv); //En fonction des paramètres du main, on active les dummy motors ou non

	// Création du groupement de deux codeurs
	SerialCodeurManager codeurs(0);
	//reset du lancement précédent
	codeurs.Closes();
	codeurs.Initialisation();
	delay(100);

	char nomFile[100];
	sprintf(nomFile, "filepoint3.1/%s/", argv[1]); //Dossier contenant le fichier main.strat et les fichier .point
	Strategie strat(nomFile, argv[1]);

	//Setup Connexion udp to Serveur
	string ipServeur = config.getIpServeur();
	int portServeur = config.getPort();
	ClientUDP client(ipServeur, portServeur);

   	timer temps;

    deltaAsservTimer = config.getDeltaAsserv();
    Controller controller(codeurs, moteurs, config);
    Odometry odometry(codeurs);
	//asserv.initialiser(strat.getPointActuel());

/***************************** Départ du robot *********************************/
	cout << "Depart du robot" << endl;
	codeurs.reset();
	codeurs.reset();
	cout <<"Codeur reset"<<endl;

	jouerMatch(ref(controller), ref(strat), moteurs, odometry);

	if(forcing_stop) {
		cout << "Forcing stop" << endl;
	} else {
		cout << "Arrivee du robot" << endl;
	}

/************************ Libération de la mémoire *****************************/
	cout << "Liberation de la memoire" << endl;

	codeurs.Closes();
	close(i2cM);

/***************************** Fin du programme ********************************/
	cout << "Fin du programme" << endl;
	return forcing_stop ? EXIT_FORCING_STOP : EXIT_SUCCESS;
}


/////////////////////////// FIN PROGRAMME PRINCIPAL /////////////////////////////

void jouerMatch(Controller& controller, Strategie& strat, MoteurManager& moteurs, Odometry& odometry) {

    timer asservTimer;
    //controller.gotoPoint(1850,0,0);
    controller.gotoPoint(500,0,0);
	while(!forcing_stop && strat.isNotFinished()) {
	    //cout << "Moteur lancé " << endl;

        if(asservTimer.elapsed_ms() >= deltaAsservTimer) {
            //controller.update();
            odometry.update();
            odometry.debug();
            asservTimer.restart();

        }
        //odometry.update();
        //odometry.debug();
		//sleepMillis(10);
	}
    //asserv.stop();
    //sleepMillis(100); //Permet de laisser le temps de demander l'arrêt des moteurs :)
    //asserv.stop();
    sleepMillis(10);
    //strat.getPointActuel().display();
	return;
}


bool argc_control(int argc) {
/**
 * Renvoie EXIT_SUCCESS si tout va bien,
**/
	if(argc == 2)
		return true;
	else
		return false;
}

bool argv_contains_dummy(int argc, char** argv) {
/**
 * Renvoie true si un des arguments demande des dummy motors
**/
	//dummy motors
	const char* DUMMY_SHORTARG = "-dm";
	const char* DUMMY_LONGARG = "--dummy-motors";

	for(int i = 0; i < argc; i++)
	{
		if(strcmp(argv[i], DUMMY_SHORTARG) == 0 || strcmp(argv[i], DUMMY_LONGARG))
			return true;
	}
	return false;
}

void stopSignal(int signal) {
	if(signal)
		cout << "CTRL+C détecté" << endl;
	forcing_stop = true;
	return;
}

void sleepMillis(int millis) {
	std::this_thread::sleep_for(std::chrono::milliseconds(millis));
}
