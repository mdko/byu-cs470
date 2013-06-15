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
const string dead_status = "dead";
const double PI = 3.141592653;

static bool debug = false;
static direction_t search_order[NUMBER_OF_DIRECTIONS]; // Defined in world_init
static grid_t world_grid;
static coordinate_t NULL_COORDINATE;
static coordinate_t enemy_flag_coor;
static coordinate_t home_base_coor;
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
static int tank_with_flag = -1;

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
	else
	{
		my_team_color = argv[8];
	}
	assert(my_team_color.length() > 0);

	BZRC MyTeam = BZRC(pcHost, nPort, false);
	if(!MyTeam.GetStatus()) {
		cout << "Can't connect to BZRC server." << endl;
		exit(1);
	}
	
	// Calling agent code
	world_init(&MyTeam);
	
	struct timeval now;
	struct timeval last_tick;
	last_tick = now;
	gettimeofday(&now, NULL);

	// Initialize the tank brains/goals
	{
		int number_of_tanks = my_tanks->size();
		tank_brains = new vector<tank_brain_t>();
		for (int tank_n = 0; tank_n < number_of_tanks; tank_n++)
		{
			tank_brain_t tb;
			tb.last_updated_s = 0;
			tb.current_goal = NULL_COORDINATE;
			tb.current_state = SETUP;
			tb.last_updated_s = now.tv_sec;
			tb.can_shoot = false;
			tb.current_path = NULL;
			tank_brains->push_back(tb);
		}
	}

	// TODO 
	//~ // Pad the observed_enemy_coordinates vector by 2 so that we don't flub on our first two calculations of leading the shot.
	//~ observed_enemy_coordinates.push_back(NULL_COORDINATE);
	//~ observed_enemy_coordinates.push_back(NULL_COORDINATE);

	// TODO Initialize Kalman filter for all the enemy tanks we track
	//~ resetKalmanFilterConstants();
	//~ resetKalmanFilterAfterEachRun(); // TODO call if the enemy tank dies...I think

	gettimeofday(&now, NULL);
	last_tick = now;

	long last_flag_update_s = now.tv_sec;

	while (true)
	{
		my_tanks->clear();
		MyTeam.get_mytanks(my_tanks);
		// TODO Track the positions of all of the enemy's tanks
		if (now.tv_sec >= last_flag_update_s + 5)
		{
			store_enemy_flag(&MyTeam);
			last_flag_update_s = now.tv_sec;
		}

		gettimeofday(&now, NULL);
		last_tick = now;

		for (int tank_n = 0; tank_n < tank_brains->size(); tank_n++)
		{
			printf("New frame.\n");
			tank_t current_tank = my_tanks->at(tank_n);
			tank_brain_t current_tank_brain = tank_brains->at(tank_n);			

			if (dead_status.compare(current_tank.status) == 0)
			{
				//printf("Tank %d is dead!\n", tank_n);
				continue;
			}
			
			switch (tank_brains->at(tank_n).current_state)
			{
				case SETUP:
				{
					if (current_tank_brain.last_updated_s + 20 >= now.tv_sec)
					{
						int tank_type = tank_n % 4;
						tank_brains->at(tank_n).can_shoot = false;
						switch (tank_type)
						{
							case 0:
							{
								tank_brains->at(tank_n).heading.x = 0;
								tank_brains->at(tank_n).heading.y = 1;
								break;
							}
							case 1:
							{
								tank_brains->at(tank_n).heading.x = 1;
								tank_brains->at(tank_n).heading.y = 0;
								break;
							}
							case 2:
							{
								tank_brains->at(tank_n).heading.x = 0;
								tank_brains->at(tank_n).heading.y = -1;
								break;
							}
							case 3:
							{
								tank_brains->at(tank_n).heading.x = -1;
								tank_brains->at(tank_n).heading.y = 0;
								break;
							}
						}
					}
					else
					{
						printf("Tank %d is switching to FLAG_DEFENSE mode.\n", tank_n);
						tank_brains->at(tank_n).current_state = FLAG_DEFENSE;
						tank_brains->at(tank_n).last_updated_s = now.tv_sec;
					}
					break;
				} // end case SETUP
				case FLAG_DEFENSE:
				{
					if (current_tank_brain.last_updated_s + 40 + tank_n*1 >= now.tv_sec)
					{
						for (int tank_n = 0; tank_n < tank_brains->size(); tank_n++)
						{
							// select your closest target
							int target_number = 0;
							// TODO only do this every X seconds
							{
								double distance = 10000000;
								for (int target_n = 0; target_n < enemy_tanks_coors->size(); target_n++)
								{
									coordinate_t current_target = enemy_tanks_coors->at(target_n);
									coordinate_t diff;
									diff.x = current_target.x - my_tanks->at(tank_n).pos[0];
									diff.y = current_target.y - my_tanks->at(tank_n).pos[1];
									double new_distance = sqrt(diff.x*diff.x + diff.y*diff.y);
									if (new_distance < distance)
									{
										target_number = target_n;
										distance = new_distance;
									}
								}
							}
							coordinate_t target_coor = enemy_tanks_coors->at(target_number);
							shoot_at_target(tank_n, &MyTeam, target_coor);
						}
					}
					else
					{
						printf("Tank %d is switching to ATTACK mode.\n", tank_n);
						tank_brains->at(tank_n).current_state = ATTACK;
						tank_brains->at(tank_n).last_updated_s = now.tv_sec;
					}
					break;
				} // end case FLAG_DEFENSE
				case ATTACK:
				{
					if (current_tank.flag.compare("-") != 0)
					{
						printf("Tank %d is switching to RUN_WITH_FLAG mode.\n", tank_n);
						tank_brains->at(tank_n).current_state = RUN_WITH_FLAG;
						tank_brains->at(tank_n).last_updated_s = now.tv_sec;						
						tank_brains->at(tank_n).current_path = NULL;
						break;
					}
					// TODO check if one of our tanks is carrying our flag, if they are switch to PROTECT_RUNNER
					// TODO check if our flag has been captured, if it has go kill the person carrying it and then pick it up

					coordinate_t current_position;
					current_position.x = current_tank.pos[0] + (world_grid.width / 2);
					current_position.y = world_grid.height - (current_tank.pos[1] + (world_grid.height / 2));

					if (current_tank_brain.last_updated_s + 5 <= now.tv_sec || current_tank_brain.current_path == NULL)
					{
						if (current_tank_brain.current_path != NULL)
						{
							delete current_tank_brain.current_path;
							tank_brains->at(tank_n).current_path = NULL;
						}
						
						tank_brains->at(tank_n).last_updated_s = now.tv_sec + (rand() % 4);

						all_straight(&MyTeam);
						printf("Now calculating path for Tank %d from %f, %f to %f, %f.\n",
							tank_n, enemy_flag_coor.x, enemy_flag_coor.y, current_position.x, current_position.y);

						fill_directional_grid(world_grid.width, world_grid.height);
						stack<coordinate_t> * path = best_first_search(enemy_flag_coor.x, enemy_flag_coor.y,
							current_position.x, current_position.y, true);
						tank_brains->at(tank_n).current_path = path;
						//display_path("astar.tga", path);
						if (path != NULL)
						{
							set_heading(tank_n, path);
						}
						tank_brains->at(tank_n).can_shoot = true;
					}
					else
					{
						set_heading(tank_n, current_tank_brain.current_path);						
					}
					break;
				} // end case ATTACK
				case RUN_WITH_FLAG:
				{
					coordinate_t current_position;
					current_position.x = current_tank.pos[0] + (world_grid.width / 2);
					current_position.y = world_grid.height - (current_tank.pos[1] + (world_grid.height / 2));

					if (current_tank_brain.last_updated_s + 3 <= now.tv_sec || current_tank_brain.current_path == NULL)
					{
						if (current_tank_brain.current_path != NULL)
						{
							delete current_tank_brain.current_path;
							current_tank_brain.current_path = NULL;
						}
						
						tank_brains->at(tank_n).last_updated_s = now.tv_sec + (rand() % 4);

						MyTeam.angvel(tank_n, 0);
						printf("Now calculating path for Tank %d from %f, %f to %f, %f.\n",
							tank_n, home_base_coor.x, home_base_coor.y, current_position.x, current_position.y);

						fill_directional_grid(world_grid.width, world_grid.height);
						stack<coordinate_t> * path = best_first_search(home_base_coor.x, home_base_coor.y,
							current_position.x, current_position.y, true);
						tank_brains->at(tank_n).current_path = path;
						//display_path("astar.tga", path);
						if (path != NULL)
						{
							set_heading(tank_n, path);
						}
						tank_brains->at(tank_n).can_shoot = true;
					}
					else
					{
						set_heading(tank_n, current_tank_brain.current_path);						
					}
					break;
				} // end case RUN_WITH_FLAG
				case PROTECT_RUNNER:
				{
					// TODO Check whether the tank runner is dead, if he is switch to ATTACK
					// TODO check if our flag has been captured, if it has go kill the person carrying it and then pick it up
				}
				// TODO RETRIEVE_OUR_FLAG
				default:
				{
					printf("Invalid tank state observed for %d.\n", tank_n);
					assert(false);
					break;
				}
			}
			
			for (int tank_n = 0; tank_n < tank_brains->size(); tank_n++)
			{
				follow_orders(tank_n, &MyTeam);
			}
			
			usleep(5000);
		}
	}
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
	home_base_coor.x = NULL_COORDINATE.x;
	home_base_coor.y = NULL_COORDINATE.y;
	
	my_tanks = new vector<tank_t>();
	my_tanks->clear();
	my_team->get_mytanks(my_tanks);

	my_team->get_occgrid(world_grid);
	printf("Now building the world_grid, using size %d.\n", world_size);
	
	enemy_tanks_coors = new vector<coordinate_t>();
	store_enemy_tanks_coors(my_team);

	store_enemy_flag(my_team);

	printf("Printing the world grid.\n");
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

	string enemy_color = "-";
	for (int flag_n = 0; flag_n < enemy_flags.size(); flag_n++)
	{
		flag_t curr_flag = enemy_flags.at(flag_n);
		//~ printf("Examining flag %s.\n", curr_flag.color.c_str());
		if (curr_flag.color.compare(my_team_color.c_str()) != 0)
		{
			enemy_flag_coor.x = curr_flag.pos[0];
			enemy_flag_coor.y = curr_flag.pos[1];
			enemy_color = curr_flag.color;
		}
		else if (home_base_coor.x == NULL_COORDINATE.x)
		{
			home_base_coor.x = curr_flag.pos[0] + world_grid.width / 2;
			home_base_coor.y = curr_flag.pos[1] + world_grid.height / 2;
		}
	}
	enemy_flag_coor.x += world_grid.width / 2;
	enemy_flag_coor.y += world_grid.height / 2;
	enemy_flag_coor.y = world_grid.height - enemy_flag_coor.y;
	printf("Enemy %s flag at: %f, %f\n", enemy_color.c_str(), enemy_flag_coor.x, enemy_flag_coor.y);
	printf("My flag at: %f, %f\n", home_base_coor.x, home_base_coor.y);
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

void set_heading(int tank_n, stack<coordinate_t> * path)
{
	const int lookahead_distance = 1;
	
	direction_t ret;

	if (path == NULL || path->empty())
	{
		printf("Received a bogus path for tank %d.\n", tank_n);
		tank_brains->at(tank_n).heading = ret;
		//tank_brains->at(tank_n).current_goal = NULL_COORDINATE;
		tank_brains->at(tank_n).current_path = NULL;
		return;
	}
	
	coordinate_t source = path->top();
	path->pop();
	for (int i = 0; i < lookahead_distance; i++)
	{
		if (path->empty())
		{
			break;
		}
		coordinate_t current_point = path->top();
		path->pop();
		
		double diff_x = current_point.x - source.x;
		double diff_y = current_point.y - source.y;
		
		// Emphasize
		//diff_x *= (lookahead_distance - i + 2.0);
		//diff_y *= (lookahead_distance - i + 2.0);
		
		ret.x += diff_x;
		ret.y += diff_y;
		
	}
	
	if (ret.x != 0 && ret.y != 0)
	{
		tank_brains->at(tank_n).heading = ret;
	}
	
	//~ string s = "tank_#_path.tga";
	//~ s.at(5) = 48 + tank_n; // Convert the tank number into a character, 0 is ascii 48
	//~ 
	//~ display_path(s.c_str(), path);
	
	return;
} // end set_heading()

void follow_orders(int tank_n, BZRC* my_team)
{
	const double turn_strength = 5.0;
	const double acceptable_difference = 0.5;	// As long as our impulse is within an arc this many radians wide, drive at full speed
	const double minimum_speed = 0.6;

	direction_t impulse = tank_brains->at(tank_n).heading;
	impulse.y *= -1; // stupid inverstion crap

	double randomness = ((rand() % 11) - 5) / 5.0 * acceptable_difference; // -5 to 5, scaled to somewhere within the acceptable_difference cone

	double speed;
	double turning;

	//printf("Tank %d is realigning its course towards %f, %f with heading %f, %f.\n", tank_n, tank_brains->at(tank_n).current_goal.x, tank_brains->at(tank_n).current_goal.y, impulse.x, impulse.y);

	// if heading is at 0,0, stop the tank because it means it got given a NULL path.
	if (impulse.x == 0 && impulse.y == 0)
	{
		speed = 0;
		turning = 0;
	}
	else
	{
		double strength = sqrt(pow(impulse.x, 2) + pow(impulse.y, 2));

		// Convert X and Y to an angle
		double new_rotation = atan2(impulse.y, impulse.x);

		//Normalize the difference between the two rotations by finding the angle between 4PI and -4PI that generates a difference between PI and -PI. 
		double r1 = new_rotation + 2*PI - my_tanks->at(tank_n).angle;
		double r2 = new_rotation		- my_tanks->at(tank_n).angle;
		double r3 = new_rotation - 2*PI - my_tanks->at(tank_n).angle;
		double difference = 10*PI; // A ludicrously high angle.
		if (fabs(r1) < fabs(difference))
		{
			difference = r1;
		}
		if (fabs(r2) < fabs(difference))
		{
			difference = r2;
		}
		if (fabs(r3) < fabs(difference))
		{
			difference = r3;
		}

		difference += randomness;

		// Make the tank slow down during sharp turns.
		if (fabs(difference) < acceptable_difference / 2)
		{
			// Full speed ahead.
			speed = 1.0;
			if (tank_brains->at(tank_n).can_shoot)
			{
				my_team->shoot(tank_n);
			}
		}
		else
		{
			// Set impulse to one-quarter speed until we're pointing in the right direction
			speed = minimum_speed;
		}	

		difference *= turn_strength;
		turning = difference;
		//if (iterations % 10 == 0)
		//{
		//	cout << "I am tank " << tank_n << " and my angle is " << my_tanks->at(tank_n).angle << " and I am turning " << difference << endl; 
		//}
	}

	my_team->angvel(tank_n, turning);
	my_team->speed(tank_n, speed);
	return;
} // end follow_orders()

void fill_directional_grid(int width, int height) {
	directional_grid.contents.clear();
	directional_grid.width = width;
	directional_grid.height = height;
	directional_grid.contents.resize(directional_grid.height);
	
	for (int height_n = 0; height_n < directional_grid.height; height_n++) {
		for (int width_n = 0; width_n < directional_grid.width; width_n++) {
			direction_t value;
			value.x = NULL_COORDINATE.x;
			value.y = NULL_COORDINATE.y;
			directional_grid.contents.at(height_n).push_back(value);
		}
	}
} // end fill_directional_grid()

// If use_heuristic is false, this is uniform cost. If use_heuristic is true, this is A-star.
stack<coordinate_t> * best_first_search(int target_x, int target_y, int start_x, int start_y, bool use_heuristic)
{
	if (target_x == NULL_COORDINATE.x && target_y == NULL_COORDINATE.y)
	{
		return NULL;
	}
	if (start_x == NULL_COORDINATE.x && start_y == NULL_COORDINATE.y)
	{
		return NULL;
	}
	
	assert(left_bounds == 0);
	assert(bottom_bounds == 0);
	assert(target_x >= 0);
	assert(target_y >= 0);
	assert(start_x >= 0);
	assert(start_y >= 0);

	const double SQRT_TWO = 1.414213562;

	priority_queue<prioritizable_node_t> next_locations;

	prioritizable_node_t current_location;
	current_location.x = start_x;
	current_location.y = start_y;
	current_location.prev_x = start_x;
	current_location.prev_y = start_y;
	current_location.cost = 0;
	if (use_heuristic)
	{
		current_location.heuristic = generate_heuristic(target_x, target_y, start_x, start_y);
	}
	else
	{
		current_location.heuristic = 0.0;
	}
	next_locations.push(current_location);
	
	bool found_path;
	int cycles_per_frame = 1000;
	//if (use_heuristic)
	//{
	//	cycles_per_frame = 201;
	//}
	//else
	//{
	//	cycles_per_frame = 501;
	//}
	
	int iterations = 0;
	while (next_locations.size() > 0)
	{
		current_location = next_locations.top();
		int current_x = current_location.x;
		int current_y = current_location.y;
		next_locations.pop();
		nodes_popped++;
		
		//printf("Cost here at %f, %f is %f.\n", current_location.x, current_location.y, current_location.cost);

		// Check if we're out of bounds first.
		if (current_x < left_bounds || current_x >= right_bounds
			|| current_y >= top_bounds || current_y < bottom_bounds)
		{
			//printf("Rejected because out of bounds.\n");
			continue;
		}
		
		// Check if we're in a wall
		//printf("WorldGrid Obstacles: %d, %d\n", world_grid.obstacles.size(), world_grid.obstacles.at(current_x).size());
		if (world_grid.obstacles.at(current_x).at(current_y) == 1)
		{
			//printf("Inside a wall, backing out.\n");
			continue;
		}

		// Check if we're in a location we've already been at. If not, mark it with a back-pointer.
		if (directional_grid.contents.at(current_x).at(current_y).x == NULL_COORDINATE.x &&
		    directional_grid.contents.at(current_x).at(current_y).y == NULL_COORDINATE.y
		 )
		{
			directional_grid.contents.at(current_x).at(current_y).x = current_location.prev_x;
			directional_grid.contents.at(current_x).at(current_y).y = current_location.prev_y;
		}
		else
		{
			//printf("This location was already visited.\n");
			continue;
		}

		if (target_x == current_x && target_y == current_y)
		{
			found_path = true;
			break;
		}
		
		// TODO calculate how much my travel is going to cost on account of walls and tanks.
		
		// Try adjacent spaces.
		for (int i = 0; i < NUMBER_OF_DIRECTIONS; i++)
		{
			prioritizable_node_t next_location;

			direction_t offset = search_order[i];
			next_location.x = current_location.x + offset.x;
			next_location.y = current_location.y + offset.y;
			next_location.prev_x = current_x;
			next_location.prev_y = current_y;
			next_location.cost = current_location.cost;

			//printf("Adding new location %f, %f to the stack.\n", next_location.x, next_location.y);
	
			double multiplier = 1.0;
	
			//~ if (penalized_mode)
			if (false)
			{
				bool this_has = has_adjacent_occupied(current_location.x, current_location.y);
				bool next_has = has_adjacent_occupied(next_location.x, next_location.y);
				
				if (this_has && next_has)
				{
					multiplier = 1.5;
				}
				else if (this_has) // && !next_has
				{
					multiplier = 1.1;
				}
				else if (next_has) // && !this_has
				{
					multiplier = 1.3;
				}
			}
			
			// The second half of the search order directions are diagonal.
			if (i < (NUMBER_OF_DIRECTIONS / 2))
			{
				//printf("Added orthagonal.\n");
				next_location.cost += multiplier;
			}
			else
			{
				//printf("Added diagonal.\n");
				next_location.cost += (multiplier * SQRT_TWO);
			}

			//~ if (avoid_tanks)
			//~ {
				//~ next_location.cost += tank_weights.weights.at(current_location.x).at(current_location.y);
			//~ }
			
			if (use_heuristic)
			{
				next_location.heuristic = generate_heuristic(target_x, target_y, next_location.x, next_location.y);
			}
			else
			{
				next_location.heuristic = 0.0;
			}

			next_locations.push(next_location);
		}
		
		//~ if (iterations % cycles_per_frame == 1)
		//~ {
			//~ string filename = "";
			//~ if (use_heuristic)
			//~ {
				//~ filename = "astar.tga";
			//~ }
			//~ else
			//~ {
				//~ filename = "ucost.tga";
			//~ }
			//~ display_pqueue_wavefront(filename.c_str(), next_locations);
		//~ }
		//~ iterations++;
		
	} // end while (next_locations.size() > 0)

	if (found_path)
	{
		final_cost = current_location.cost;
		stack<coordinate_t> * ret = new stack<coordinate_t>;
		int sanity = 0;
		while (!(current_location.x == start_x && current_location.y == start_y))
		{
			coordinate_t curr;
			curr.x = current_location.x;
			curr.y = current_location.y;
			ret->push(curr);

			if (curr.x == NULL_COORDINATE.x && curr.y == NULL_COORDINATE.y)
			{
				return NULL;
			}
			//printf("Found an A-star path to %f, %f.\n", curr.x, curr.y);
			current_location.x = directional_grid.contents.at(curr.x).at(curr.y).x;
			current_location.y = directional_grid.contents.at(curr.x).at(curr.y).y;
			if(sanity++ > 20000)
			{
				printf("Been backtracing too long, start panicking.\n");
				break;
			}
			//printf("Backtracing %f, %f.\nCurrent size is %d\n", current_location.x, current_location.y, ret->size());
		}
		return ret;
	}
	else
	{
		return NULL;
	}
	
} // end best_first_search()

double generate_heuristic(int target_x, int target_y, int start_x, int start_y)
{
	double dx = target_x - start_x;
	double dy = target_y - start_y;
	
	dx *= dx;
	dy *= dy;
	
	double ret = dx + dy;
	ret = sqrt(ret);
	
	return ret;
}

bool has_adjacent_occupied(int x, int y)
{
	for (int i = 0; i < NUMBER_OF_DIRECTIONS; i++)
	{
		coordinate_t next_location;

		direction_t offset = search_order[i];

		next_location.x = x + offset.x;
		next_location.y = y + offset.y;
		
		if (next_location.x < left_bounds || next_location.x >= right_bounds ||
			next_location.y < bottom_bounds || next_location.y >= top_bounds
		 )
		{
			return true;
		}
		
		if (world_grid.obstacles.at(next_location.x).at(next_location.y))
		{
			return true;
		}
	}
	return false;
} // end has_adjacent_occupied()

void display_path(const char* filename, stack<coordinate_t> * path)
{
	assert(path != NULL);

	//printf("Beginning path image export...\n");
	std::string targetFile = filename;
	
	int path_weighting = path->size();

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

	while (path->size() > 0)
	{
		coordinate_t current = path->top();
		path->pop();
		//printf("Path coordinate: %f, %f\n", current.x, current.y);
		//c = distance_to_color(path->size());
		c.r = 255;
		c.g = 255;
		c.b = 255;
		img->setPixel(c,current.x,world_grid.height - current.y - 1);
	}

	//write the image to disk
	img->WriteImage(filename);

	//printf("Path image output to %s\n", filename);
	return;
} // end display_path()

void all_straight(BZRC* my_team)
{
	printf("Telling all tanks to go straight.\n");
	for (int tank_n = 0; tank_n < tank_brains->size(); tank_n++)
	{
		if (my_tanks->at(tank_n).status.compare("dead") != 0)
		{
			my_team->angvel(tank_n, 0.0);
		}
	}
	return;
} // end all_straight()

void shoot_at_target(int tank_n, BZRC* my_team, coordinate_t target)
{
	const double turn_strength = 5.0;
	const double acceptable_difference = 0.05;	// As long as our impulse is within an arc this many radians wide, shoot

	double a = target.y;
	target.y = target.x;
	target.x = a;

	direction_t source; // The pair of numbers that comes in is inverted and in game coordinates.
	source.x = my_tanks->at(tank_n).pos[0] + world_grid.width / 2;
	source.y = my_tanks->at(tank_n).pos[1] + world_grid.height / 2;

	//direction_t impulse = tank_brains->at(tank_n).heading;
	direction_t impulse;
	impulse.x = target.x - source.x;
	impulse.y = target.y - source.y;

	//impulse.y *= -1; // stupid inverstion crap

	//printf("Source: %f, %f.\nTarget: %f, %f\nImpulse: %f, %f.\n", source.x, source.y, target.x, target.y, impulse.x, impulse.y);

	//double randomness = ((rand() % 11) - 5) / 5.0 * acceptable_difference; // -5 to 5, scaled to somewhere withing the acceptable_difference cone
	double randomness = 0;

	double speed = 0;
	double turning;

	//printf("Tank %d is realigning its course towards %f, %f with heading %f, %f.\n", tank_n, tank_brains->at(tank_n).current_goal.x, tank_brains->at(tank_n).current_goal.y, impulse.x, impulse.y);

	// if heading is at 0,0, stop the tank because it means it got given a NULL path.
	if (impulse.x == 0 && impulse.y == 0)
	{
		speed = 0;
		turning = 0;
	}
	else
	{
		double strength = sqrt(pow(impulse.x, 2) + pow(impulse.y, 2));

		// Convert X and Y to an angle
		double new_rotation = atan2(impulse.y, impulse.x);

		//Normalize the difference between the two rotations by finding the angle between 4PI and -4PI that generates a difference between PI and -PI. 
		double r1 = new_rotation + 2*PI - my_tanks->at(tank_n).angle;
		double r2 = new_rotation		- my_tanks->at(tank_n).angle;
		double r3 = new_rotation - 2*PI - my_tanks->at(tank_n).angle;
		double difference = 10*PI; // A ludicrously high angle.
		if (fabs(r1) < fabs(difference))
		{
			difference = r1;
		}
		if (fabs(r2) < fabs(difference))
		{
			difference = r2;
		}
		if (fabs(r3) < fabs(difference))
		{
			difference = r3;
		}

		difference += randomness;

		if (fabs(difference) < acceptable_difference / 2)
		{
			//speed = 1.0;
			//if (shoot_bullets && target.x > 0 && target.y > 0 && target.x < world_grid.width && target.y < world_grid.height)
			if (shoot_bullets)
			{
				printf("Boom!\n");
				my_team->shoot(tank_n);
			}
			// print_skeet_vision("skeet.tga", source.x, source.y, target.x, target.y);
		}
		else
		{
			// Set impulse to one-quarter speed until we're pointing in the right direction
			//speed = 0.25;
		}	
		speed = 0.0;

		difference *= turn_strength;
		turning = difference;
		//if (iterations % 10 == 0)
		//{
		//	cout << "I am tank " << tank_n << " and my angle is " << my_tanks->at(tank_n).angle << " and I am turning " << difference << endl; 
		//}
	}

	my_team->speed(tank_n, speed);
	my_team->angvel(tank_n, turning);
	return;
} // end shoot_at_target()
