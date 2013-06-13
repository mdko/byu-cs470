#include <iostream>
#include "470bot.h"
#include <ctime>
#include <chrono>
#include <sys/time.h>
#include <unistd.h>
#include <cstdlib>
#include "Image.h"
#include <armadillo>

#define NUMBER_OF_DIRECTIONS 8

using namespace std;
using namespace arma;

const char *kDefaultServerName = "localhost";
const int kDefaultServerPort = 4000;
const int max_failures = 10; // A* Global

static bool debug = false;
static direction_t search_order[NUMBER_OF_DIRECTIONS]; // Defined in world_init
static grid_t world_grid;
static coordinate_t NULL_COORDINATE;
static coordinate_t enemy_flag_coor;
static int world_size;
static double shot_speed;
static grid_t visited_grid;
static direction_grid_t directional_grid;
static vector <tank_t> *my_tanks;
static vector<tank_brain_t> *tank_brains;
static vector<coordinate_t> *enemy_tanks_coors;
static vector<flag_t> enemy_flags;
static string my_team_color;
static bool shoot_bullets = true;

static double left_bounds;
static double right_bounds;
static double top_bounds;
static double bottom_bounds;

// A* Globals
static int recursive_counter = 0;
static int nodes_popped = 0;
static int final_cost = 0;
static int number_of_search_failures = 0;

// Constants initialize before kalmanFilter
static mat F;			// 6x6
static mat sigma_x;		// 6x6
static mat H;			// 2x6
static mat sigma_z;		// 2x2
static mat F_transpose;	// 6x6
static mat H_tranpose;	// 6x2
static mat I;			// 6x6
static float c = 0.1;
static float posit_conf = 25;
static float accel_conf = 30;

// Initialized at start of each run for kalmanFilter
vector<colvec> mu;//(6);// 6x1
vector<mat> sigma;		// 6x6

int main(int argc, char *argv[]) {
	define_constants();
	std::srand(std::time(0));
	
	const char *pcHost;
	int nPort;

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
	if(argc < 4) {
		world_size = 400;
	} else {
		world_size = atoi(argv[3]);
	}
	if(argc < 5) {
		shot_speed = 6;
	}
	else if (atoi(argv[4]) != 0)
	{
		shot_speed = atoi(argv[4]);
		shot_speed /= 100;
	}	
	if(argc < 6) {
		shoot_bullets = true;
	}
	else if (atoi(argv[5]) == 0)
	{
		shoot_bullets = false;
	}	
	if(argc < 7) {
		;
	}
	else if (atoi(argv[6]) != 0)
	{
		posit_conf = atoi(argv[6]);
		posit_conf /= 10;
	}	
	if(argc < 8) {
		;
	}
	else if (atoi(argv[7]) != 0)
	{
		accel_conf = atoi(argv[7]);
		accel_conf /= 10;
	}

	if(argc < 9) {
		my_team_color = "red";
	}
	else if (atoi(argv[8]) != 0)
	{
		my_team_color = argv[8];
	}

	BZRC MyTeam = BZRC(pcHost, nPort, false);
	if(!MyTeam.GetStatus()) {
		cout << "Can't connect to BZRC server." << endl;
		exit(1);
	}
	
	// Calling agent code
	world_init(&MyTeam);
}

void define_constants()
{
	NULL_COORDINATE.x = -1;
	NULL_COORDINATE.y = -1;	

	// "Please consider these states in the following order: left (x-1,y), right (x+1,y), up (x,y+1), down (x,y-1), diagonal up-left, up-right, down-left, down-right."
	// These coordinates are relative to the grid's storage in memory. Right increments X. Up decrements Y.
	// Left
	search_order[0].x = -1;
	search_order[0].y = 0;
	// Right
	search_order[1].x = 1;
	search_order[1].y = 0;
	// Up
	search_order[2].x = 0;
	search_order[2].y = -1;
	// Down
	search_order[3].x = 0;
	search_order[3].y = 1;
	// Up-Left
	search_order[4].x = search_order[0].x;
	search_order[4].y = search_order[2].y;
	// Up-Right
	search_order[5].x = search_order[1].x;
	search_order[5].y = search_order[2].y;
	// Down-Left
	search_order[6].x = search_order[0].x;
	search_order[6].y = search_order[3].y;
	// Down-Right
	search_order[7].x = search_order[1].x;
	search_order[7].y = search_order[3].y;
}

void world_init(BZRC *my_team) 
{
	my_tanks = new vector<tank_t>();
	my_tanks->clear();
	my_team->get_mytanks(my_tanks);

	my_team->get_occgrid(world_grid);
	printf("Now building the world_grid, using size %d.\n", world_size);
	
	enemy_tanks_coors = new vector<coordinate_t>();
	store_enemy_tanks_coors(my_team);

	store_enemy_flag(my_team);

	printf("Printing the world grid.");
	print_grid("obstacles.tga");
	//exit(0);
	//my_team->print_grid(world_grid);

	left_bounds = 0;
	right_bounds = world_grid.width -1;
	top_bounds = world_grid.height - 1;
	bottom_bounds = 0;
	
	return;
}

void store_enemy_tanks_coors(BZRC* my_team) 
{
	enemy_tanks_coors->clear();
	
	vector <otank_t> * other_tanks = new vector<otank_t>;
	my_team->get_othertanks(other_tanks);

	for (int tank_n = 0; tank_n < other_tanks->size(); tank_n++) {
		otank_t enemy_tank = other_tanks->at(tank_n);
		coordinate_t tank_coor;
		tank_coor.y = enemy_tank.pos[0]; // x-coor
		tank_coor.x = enemy_tank.pos[1]; // y-coor
		tank_coor.x += world_grid.width / 2;
		tank_coor.y += world_grid.height / 2;		
		enemy_tanks_coors->push_back(tank_coor);
	}
	return;
}

void store_enemy_flag(BZRC* my_team)
{	
	my_team->get_flags(&enemy_flags);
	assert(enemy_flags.size() > 0);

	for (int flag_n = 0; flag_n < enemy_flags.size(); flag_n++)
	{
		flag_t curr_flag = enemy_flags.at(flag_n);
		if (curr_flag.color.compare(my_team_color.c_str()) != 0)
		{
			enemy_flag_coor.x = curr_flag.pos[0];
			enemy_flag_coor.y = curr_flag.pos[1];
		}
	}
	enemy_flag_coor.x += world_grid.width / 2;
	enemy_flag_coor.y += world_grid.height / 2;
	printf("Enemy flag at: %f, %f\n", enemy_flag_coor.x, enemy_flag_coor.y);
}

void print_grid(const char* filename) 
{
	printf("Beginning grid export...\n");
	std::string targetFile = filename;
	
	// The +1 is because the center of the arena is 0,0
	TGAImage *img = new TGAImage(world_grid.width, world_grid.height);
	Colour c;
	c.a = 255; // Image will be 100% opaque.
	
	for (int y_n = 0; y_n < world_grid.height; y_n++) {
		for (int x_n = 0; x_n < world_grid.width; x_n++) {
			int curr_pixel = world_grid.obstacles.at(x_n).at(world_grid.height - y_n -1);
			
			if (curr_pixel == 1) { // is an obstacle
				c.r = 128;
				c.g = 128;
				c.b = 128;
			} else if (curr_pixel == 0) { // is empty
				c.r = 0;
				c.g = 0;
				c.b = 0;
			}
			img->setPixel(c,x_n,y_n);
		}
	}

	//write the image to disk
	img->WriteImage(filename);

	cerr << "Grid output to " << targetFile << + "\n";
	return;
}
