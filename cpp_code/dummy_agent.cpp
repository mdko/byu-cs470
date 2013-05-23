#define _CRT_SECURE_NO_DEPRECATE 1
#include <iostream>
#include "470bot.h"
#include <ctime>
#include <chrono>
#include <sys/time.h>
#include <unistd.h>
#include <cstdlib>
using namespace std;
using namespace std::chrono;

const char *kDefaultServerName = "localhost";
const int kDefaultServerPort = 4000;

static long nextTurnSeconds;
static long nextTurnMillis;
static long nextShotMillis;
static long nextShotSeconds;

enum turningState {
	STRAIGHT,
	RIGHT,
	LEFT
};

static turningState nextTurn;

int main(int argc, char *argv[]) {
	const char *pcHost;
	int nPort;
	std::srand(std::time(0));

	nextTurnSeconds = 0;
	nextTurnMillis = 0;
	nextShotSeconds = 0;
	nextShotMillis = 0;
	
	nextTurn = STRAIGHT;

	if(argc < 2) {
		pcHost = kDefaultServerName;
	}
	else {
		pcHost = argv[1];
	}
    if(argc < 3) {
		nPort = kDefaultServerPort;
	}
	else {
        nPort = atoi(argv[2]);
    }

	BZRC MyTeam = BZRC(pcHost, nPort, false);
	if(!MyTeam.GetStatus()) {
		cout << "Can't connect to BZRC server." << endl;
		exit(1);
	}

	// Calling agent code
	world_init(&MyTeam);

	// Update loop
	while (true) {
		robot_pre_update(&MyTeam);
		robot_update(&MyTeam);
		robot_post_update(&MyTeam);
		usleep(50);
	}

	MyTeam.Close();
	free(&MyTeam);
	return 0;
}

void world_init(BZRC *my_team) 
{
}

// Make queries to the server
void robot_pre_update(BZRC *my_team) 
{
}

// Decide what we're going to do
void robot_update(BZRC *my_team) 
{
}

// Send actions to the server
void robot_post_update(BZRC *my_team)
{
	struct timeval now;
	gettimeofday(&now, NULL);
	long currentSeconds = now.tv_sec;
	long currentMillis = now.tv_usec;

	if (currentSeconds >= nextShotSeconds && currentMillis >= nextShotMillis)
	//if (true)
	{
		my_team->shoot(0);
		my_team->shoot(1);
		int ran_time = (rand() % 1000) + 500;
		nextShotSeconds = currentSeconds + 1 + ran_time / 1000;
		nextShotMillis = currentMillis + 0 + ran_time % 1000;
	}

	if (currentSeconds >= nextTurnSeconds && currentMillis >= nextTurnMillis)
	{
		
		switch(nextTurn) {
			case STRAIGHT:
			{
				my_team->speed(0, 1.0);
				my_team->angvel(0, 0.0);
				my_team->speed(1, 1.0);
				my_team->angvel(1, 0.0);
				int ran_time = (rand() % 5000);

				nextTurnSeconds = currentSeconds + 3 + ran_time / 1000;
				nextTurnMillis = currentMillis + 0 + ran_time % 1000;
				nextTurn = RIGHT;
				break;
			//case LEFT:
				// TODO.
				//break;
			}			
			case RIGHT:
			{
				my_team->speed(0, 0.5);
				my_team->angvel(0, -1.0);
				my_team->speed(1, 0.5);
				my_team->angvel(1, -1.0);
				nextTurnSeconds = currentSeconds + 0;
				nextTurnMillis = currentMillis + 300;
				nextTurn = STRAIGHT;
				break;
			}
			default:
				break;
		}

		nextTurnSeconds = currentSeconds + 3;
		nextTurnMillis = currentMillis + 0;
	}
}

// Send a command (action/query) to the server
// @return response the server gave
string query(string message)
{
	//MyTeam.SendLine(message);
}
