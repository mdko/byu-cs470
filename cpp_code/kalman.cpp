// 6/3

#include <iostream>
#include "470bot.h"
#include <ctime>
#include <chrono>
#include <sys/time.h>
#include <unistd.h>
#include <cstdlib>
#include "Image.h"
#include <armadillo>

using namespace std;
using namespace arma;

static bool debug = false;

const char *kDefaultServerName = "localhost";
const int kDefaultServerPort = 4000;
const int tank_field_size = 4;
const int tank_field_strength = 2;

#define PI 3.141

#define NUMBER_OF_DIRECTIONS 8
static direction_t search_order[NUMBER_OF_DIRECTIONS]; // Defined in world_init

#define MAX_UPDATE_DELAY 0 // Update tank headings every 1-2 seconds. (Realistically it will probably go longer than this.)

#define UNEXPLORED_VALUE -1

#define INITIAL_OBSTACLE_PROBABILITY 0.5

#define CONF_THRESHOLD .6

static grid_t world_grid;
static coordinate_t NULL_COORDINATE;
static double left_bounds;
static double right_bounds;
static double top_bounds;
static double bottom_bounds;
static coordinate_t green_flag_coor;
static vector<coordinate_t> *enemy_tanks_coors;

static weight_grid_t explored_probabilities;

static int world_size;

static int shot_speed;

static grid_t visited_grid;
static direction_grid_t directional_grid;
static weight_grid_t tank_weights;

static vector <tank_t> *my_tanks;
static coordinate_t red_tank_coor;

static vector<tank_brain_t> *tank_brains;

static int recursive_counter = 0;

static int nodes_popped = 0;
static int final_cost = 0;

static int number_of_search_failures = 0;
const int max_failures = 10;

static vector<coordinate_t> observed_enemy_coordinates;
static vector<coordinate_t> shot_coordinates_record;

string searchType = "";

bool penalized_mode = true;
bool avoid_tanks = true;

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
		shot_speed /= 10;
	}	
	if(argc < 6) {
		debug = false;
	}
	else if (atoi(argv[5]) != 0)
	{
		debug = true;
	}

	BZRC MyTeam = BZRC(pcHost, nPort, false);
	if(!MyTeam.GetStatus()) {
		cout << "Can't connect to BZRC server." << endl;
		exit(1);
	}

	//printf("Hello, world!\n");

	// Calling agent code
	world_init(&MyTeam);
	
	//fill_directional_grid(world_grid.width, world_grid.height);	
	//printf("WorldGrid Size is %d, %d.\n", world_grid.width, world_grid.height);
	
	int number_of_tanks = my_tanks->size();

	// Initialize the tank brains/goals
	tank_brains = new vector<tank_brain_t>();
	//for (int tank_n = 0; tank_n < number_of_tanks; tank_n++)
	//{
	//	tank_brain_t tb;
	//	tb.last_updated_s = 0;
	//	tb.current_goal = NULL_COORDINATE;
	//	tank_brains->push_back(tb);
	//}
	
	// TODO Delete this when we update the code to look for unexplored areas. 
	//coordinate_t dummy_goal = green_flag_coor;
	

	//~ mat A = randu<mat>(4, 5);
	//~ mat B = randu<mat>(4, 5);
	//~ 
	//~ mat C = A*B.t();
	//~ C.print()

	// Pad the observed_enemy_coordinates vector by 2 so that we don't flub on our first two calculations of leading the shot.
	observed_enemy_coordinates.push_back(NULL_COORDINATE);
	observed_enemy_coordinates.push_back(NULL_COORDINATE);
	
	struct timeval now;
	struct timeval last_tick;
	gettimeofday(&now, NULL);
	last_tick = now;

	assert(shot_speed != 0);

	while (true)
	{
		my_tanks->clear();
		MyTeam.get_mytanks(my_tanks);

		store_enemy_tanks_coors(&MyTeam);

		last_tick = now;
		gettimeofday(&now, NULL);
		double time_delta = now.tv_sec;
		time_delta -= last_tick.tv_sec;
		time_delta *= 1000000;
		time_delta += now.tv_usec;
		time_delta -= last_tick.tv_usec;
		printf("Tick.\nTime delta: %f.\n", time_delta);
		
		coordinate_t target;
		target.x = enemy_tanks_coors->at(0).x;
		target.y = enemy_tanks_coors->at(0).y;
		if (observed_enemy_coordinates.size() > 50) // Discard older points.
		{
			observed_enemy_coordinates.erase(observed_enemy_coordinates.begin());
		}
		observed_enemy_coordinates.push_back(target);

		if (target.x != NULL_COORDINATE.x && target.y != NULL_COORDINATE.y)
		{
			printf("Target observed at %f, %f.\n", target.x, target.y);
		
			


















			// TODO Perform kDefaultServerPortman filtering on the target.
			float enemy_pos_x = 0;
			float enemy_pos_y = 50;
			float delta_t = 0.5;
			float c = 0.2;

			//-----------------------------------------------------------------------------
			// Constants initialized beforehand
			mat F;				// 6x6
			mat sigma_x;		// 6x6
			mat H;				// 2x6
			mat sigma_z;		// 2x2
			mat F_transpose;	// 6x6
			mat H_tranpose;		// 6x2
			mat I;				// 6x6

			// velocity given delta t
			float vdt = delta_t;
			// acceleration given delta t
			float adt = (delta_t * delta_t) / 2;

			// System transition matrix
			F << 1   << vdt << adt << 0	  << 0   << 0   << endr
			  << 0	 << 1	<< vdt << 0   << 0   << 0   << endr
			  << 0   << -c  << 1   << 0   << 0   << 0   << endr
			  << 0   << 0   << 0   << 1   << vdt << adt << endr
			  << 0   << 0   << 0   << 0   << 1   << vdt << endr
			  << 0   << 0   << 0   << 0   << -c  << 1   << endr;
			  
			// Noise
			sigma_x << 0.1 << 0   << 0   << 0   << 0   << 0   << endr
				    << 0   << 0.1 << 0   << 0   << 0   << 0   << endr
				    << 0   << 0   << 100 << 0   << 0   << 0   << endr
				    << 0   << 0   << 0   << 0.1 << 0   << 0   << endr
				    << 0   << 0   << 0   << 0   << 0.1 << 0   << endr
				    << 0   << 0   << 0   << 0   << 0   << 100 << endr; 

			// Observation matrix that selects out the two position values from state vector
			H << 1 << 0 << 0 << 0 << 0 << 0 << endr
			  << 0 << 0 << 0 << 1 << 0 << 0 << endr;

			// Covariance matrix
			sigma_z << 25 << 0  << endr
					<< 0  << 25 << endr;
			// F transpose
			F_transpose = F.t();

			// H tranpose
			H_tranpose = H.t();

			// Identity matrix
			I.eye(6,6);


			// TODO not sure where this needs to be initialized
			colvec X_t(6);			// 6x1
			// State vector
			X_t << enemy_pos_x << endr
				<< 0.1		   << endr
				<< 0.1		   << endr
				<< enemy_pos_y << endr
				<< 0.1		   << endr
				<< 0.1		   << endr;

			//-----------------------------------------------------------------------------
			// Initialized at start of each run
			colvec mu(6);			// 6x1
			mat sigma;				// 6x6

			// Mean estimate of the state vector
			mu.fill(0);

			// Covariance for the state vector
			sigma << 100 << 0   << 0   << 0   << 0   << 0   << endr
				  << 0	 << 0.1 << 0   << 0   << 0   << 0   << endr
				  << 0	 << 0   << 0.1 << 0   << 0   << 0   << endr
				  << 0	 << 0   << 0   << 100 << 0   << 0   << endr
				  << 0	 << 0   << 0   << 0   << 0.1 << 0   << endr
				  << 0	 << 0   << 0   << 0   << 0   << 0.1 << endr;
				  
			//-----------------------------------------------------------------------------
			// The following change with each time step
			mat A = (F * sigma * F_transpose) + sigma_x;			// 6x6
			colvec take_out_vec_Xt(6);								// 6x1
			colvec take_out_vec_Zt(2);								// 2x1
			take_out_vec_Xt.fill(1);
			take_out_vec_Zt.fill(1);

			// TODO the following extracts values from sigma_x/z, instead of adding
			// sigma_x/z like the original equation shows?
			X_t = (F * X_t) + (sigma_x * take_out_vec_Xt);			// 6x1
			colvec Z_t = (H * X_t) + (sigma_z * take_out_vec_Zt);	// 2x1						

			mat K_t_plus_1 = (A * H_tranpose * ((H * A * H_tranpose) + sigma_z)).i(); // 6x2
			mu = (F * mu) + (K_t_plus_1 * (Z_t - (H * F * mu))); 				 	  // 6x1 mut+1 = ...*mut*...
			sigma = (I - (K_t_plus_1 * H)) * A;					 				 	  // 6x6 sigmat+1 = ...*sigmat*..
	























		
			// Predict our target's future position based on distance.
			{
				coordinate_t previous = observed_enemy_coordinates.at(observed_enemy_coordinates.size()/2);
				coordinate_t diff;
				//printf("1\n");
				diff.x = target.x - previous.x;
				diff.y = target.y - previous.y;
				if (diff.x == 0 && diff.y == 0)
				{
					// Our current target is fine. Also, the following calculations will cause divide-by-zero errors.
				}
				else
				{
					double target_velocity = 25; // Assume the target is moving forward in a straight line for now.
					// TODO Use the Kalman filtering's best guess at the target's velocity.
					//target_velocity = sqrt(pow(diff.x, 2) + pow(diff.y, 2));
					//target_velocity *= (time_delta/1000000);
					printf("Calculated velocity is %f.\n", target_velocity);

					coordinate_t dist;
					dist.x = target.x - my_tanks->at(0).pos[1] - (world_grid.width/2);
					dist.y = target.y - my_tanks->at(0).pos[0] - (world_grid.height/2);
					double distance = sqrt(pow(dist.x, 2) + pow(dist.y, 2));
					printf("Calculated distance to target is %f.\n", distance);

					// Normalise the diff.
					double diff_len = sqrt(pow(diff.x, 2) + pow(diff.y, 2));
					diff.x /= diff_len;
					diff.y /= diff_len;

					diff.x *= distance / shot_speed;
					diff.y *= distance / shot_speed;
					printf("Predicted difference is %f, %f.\n", diff.x, diff.y);

					target.x += diff.x;
					target.y += diff.y;
					printf("Leading the shot to shoot at %f, %f.\n", target.x, target.y);
				}
			}
			
			shoot_at_target(0, &MyTeam, target);
			usleep(33000);
		}
		else
		{
			//printf("Can't track on target.\n");
			usleep(1000000);
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

void world_init(BZRC *my_team) {
	my_tanks = new vector<tank_t>();
	my_tanks->clear();
	my_team->get_mytanks(my_tanks);
	red_tank_coor.x = my_tanks->at(0).pos[0];
	red_tank_coor.y = my_tanks->at(0).pos[1];

	//my_team->get_occgrid(world_grid);
	printf("Now building the world_grid, using size %d.\n", world_size);
	populate_world_grid(world_size);
	printf("Just got WorldGrid, its size is %d, %d.\n", world_grid.width, world_grid.height);
	
	enemy_tanks_coors = new vector<coordinate_t>();
	store_enemy_tanks_coors(my_team);
	//populate_tank_grid();

	store_green_flag(my_team);
	//store_red_tank(my_team);

	printf("Printing the world grid.");
	print_grid("obstacles.tga");
	//exit(0);
	//my_team->print_grid(world_grid);

	left_bounds = 0;
	right_bounds = world_grid.width -1;
	top_bounds = world_grid.height - 1;
	bottom_bounds = 0;

	// Convert the flag and tank positions we got to grid coordinates.
	red_tank_coor.x += world_grid.width / 2;
	red_tank_coor.y += world_grid.height / 2;
	assert(red_tank_coor.x >= 0);
	assert(red_tank_coor.y >= 0);
	// TODO ought to be more detailed with these asserts
	
	green_flag_coor.x += world_grid.width / 2;
	green_flag_coor.y += world_grid.height / 2;
	printf("Enemy flag at: %f, %f\n", green_flag_coor.x, green_flag_coor.y);

	printf("My tank is at %f, %f.\n", red_tank_coor.x, red_tank_coor.y);

	return;
}

void store_red_tank(BZRC* my_team) {
	// Deprecated.
	vector <tank_t> *my_tanks = new vector<tank_t>();
	my_tanks->clear();
	bool got_tanks = my_team->get_mytanks(my_tanks);
	
	assert(got_tanks);
	assert(my_tanks->size() > 0);
	red_tank_coor.x = my_tanks->at(0).pos[0];
	red_tank_coor.y = my_tanks->at(0).pos[1];
	//printf("First tank %s at: %d, %d\n", my_tanks->at(0).callsign.c_str(), red_tank_coor.x, red_tank_coor.y);
	/*
	for (int tank_n = 0; tank_n < my_tanks->size(); tank_n++)
	{
		tank_t curr_tank = my_tanks->at(tank_n);
		if (curr_tank.index == 0)
		{
			red_tank_coor.x = curr_tank.pos[0];
			red_tank_coor.y = curr_tank.pos[1];
		}
	}
	*/
}

void store_green_flag(BZRC* my_team)
{
	vector<flag_t> * enemy_flags = new vector<flag_t>;
	my_team->get_flags(enemy_flags);
	assert(enemy_flags->size() > 0);

	for (int flag_n = 0; flag_n < enemy_flags->size(); flag_n++)
	{
		flag_t curr_flag = enemy_flags->at(flag_n);
		if (curr_flag.color.compare("green") == 0)
		{
			green_flag_coor.x = curr_flag.pos[0];
			green_flag_coor.y = curr_flag.pos[1];
		}
	}
}

void store_enemy_tanks_coors(BZRC* my_team) {
	enemy_tanks_coors->clear();
	
	vector <otank_t> * other_tanks = new vector<otank_t>;
	my_team->get_othertanks(other_tanks);

	// printf("There are %d other tanks.\n", other_tanks->size());
	
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

void fill_visited_grid(int width, int height) {
	visited_grid.obstacles.clear();
	visited_grid.width = width;
	visited_grid.height = height;
	visited_grid.obstacles.resize(visited_grid.height);
	
	for (int height_n = 0; height_n < visited_grid.height; height_n++) {
		for (int width_n = 0; width_n < visited_grid.width; width_n++) {
			int value = false;
			visited_grid.obstacles.at(height_n).push_back(value);
		}
	}

}

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

}

vector<vector<bool> * > *  get_empty_bool_grid(int width, int height)
{
	vector<vector<bool> * > * ret = new vector<vector<bool> * >(width*2);

	//printf("Mike is right.\n");
	ret->resize(width*2);
	assert(ret->capacity() >= width);
	//printf("Capacity of bool grid: %d\n", ret->capacity());
	for (int i = 0; i < width; i++)
	{
		vector<bool> * column = new vector<bool>(height*2);
		ret->push_back(column);
	}

	for (int w = 0; w < width; w++) {
		ret->at(w)->resize(height*2);
		assert(ret->at(w)->capacity() >= height);
		for (int h = 0; h < height; h++) {
			ret->at(w)->push_back(false);
		}
	}
	// TODO return a empty array of false booleans
	return ret;
}

stack<coordinate_t> * recursive_depth_first_search(
	int target_x, int target_y, int current_x, int current_y,
	vector<vector<bool> * > * visited_locations, stack<coordinate_t> * path_so_far,
	int max_depth
	)
{
	//printf("Searching from %d, %d to find %d, %d\n", current_x, current_y, target_x, target_y);
	const int frames_per_render = 200;

	// TODO use max_depth

	coordinate_t current_location;
	current_location.x = current_x;
	current_location.y = current_y;

	//stack<coordinate_t> null_stack;
	//assert(null_stack.size() == 0);

	// Check if we're out of bounds first, so that we can tell whether we're at a valid location in memory.
	// TODO This might ignore the left and bottom edge of the grid, I'm assuming that right_bounds = width - 1.
	if (current_x < left_bounds || current_x >= right_bounds
		|| current_y >= top_bounds || current_y < bottom_bounds)
	{
		//printf("Discarded an out of bounds point, %d, %d\n", current_x, current_y);
		return NULL;
	}
	
	// Check if we're in a location we've already been at. Mark it if we are.
	//if (visited_locations->at(current_x)->at(current_y))
	if (visited_grid.obstacles.at(current_x).at(current_y))
	{
		//printf("Discarded a previously visited point, %d, %d\n", current_x, current_y);
		return NULL;
	}
	else
	{
		//visited_locations->at(current_x)->at(current_y) = true;
		visited_grid.obstacles.at(current_x).at(current_y) = true;
	}

	// Check if our current location is an obstacle.
	if (world_grid.obstacles.at(current_x).at(current_y))
	{
		//printf("Bumped against a wall, %d, %d\n", current_x, current_y);
		return NULL;
	}
	else
	{
		; // We're fine, keep going.
	}

	// Now we're going to try to find a correct path from the current state.
	path_so_far->push(current_location);
	
	// Check if we've reached our target.
	if (target_x == current_x && target_y == current_y)
	{
		return path_so_far;
	}
	
	recursive_counter++;
	if (recursive_counter % frames_per_render == 1)
	{
		print_visited_grid("dfs.tga");
	}
	
	// Try adjacent spaces.
	for (int i = 0; i < NUMBER_OF_DIRECTIONS; i++)
	{
		direction_t offset = search_order[i];
		stack<coordinate_t> * ret = recursive_depth_first_search(target_x, target_y, current_x + offset.x, current_y + offset.y, visited_locations, path_so_far, max_depth);
		if (ret != NULL) // The null stack has a size of 0
		{
			return ret;
		}
		else
		{
			; // Try the next one.
		}
	}
	
	// Report that none of our children reached the goal.
	path_so_far->pop();
	nodes_popped++;
	return NULL;
}

void print_grid(const char* filename) {
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
			} else if (curr_pixel == UNEXPLORED_VALUE) {
				c.r = 0;
				c.g = 0;
				c.b = 255;
			}
			img->setPixel(c,x_n,y_n);
		}
	}

	//write the image to disk
	img->WriteImage(filename);

	cerr << "Grid output to " << targetFile << + "\n";
	return;
}

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
			} else if (curr_pixel == UNEXPLORED_VALUE) {
				c.r = 0;
				c.g = 0;
				c.b = 255;
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
}

/*
Colour c distance_to_colour(int distance)
{
	Colour ret;
	ret.a = 255;

}
*/

stack<coordinate_t> * breadth_first_search(int target_x, int target_y, int start_x, int start_y)
{
	assert(left_bounds == 0);
	assert(bottom_bounds == 0);
	assert(target_x >= 0);
	assert(target_y >= 0);
	assert(start_x >= 0);
	assert(start_y >= 0);

	queue<coordinate_t> next_locations;

	coordinate_t current_location;
	current_location.x = start_x;
	current_location.y = start_y;
	next_locations.push(current_location);
	
	// Locations are pushed and popped in pairs. The first one is the current location, the second is the prior location.
	coordinate_t prior_location = current_location;
	next_locations.push(prior_location);

	bool found_path;
	const int cycles_per_frame = 50;
	int iterations = 0;
	while (next_locations.size() > 0)
	{
		// Locations are pushed and popped in pairs. The first one is the current location, the second is the prior location.
		current_location = next_locations.front();
		int current_x = current_location.x;
		int current_y = current_location.y;
		next_locations.pop();
		nodes_popped++;

		prior_location = next_locations.front();
		next_locations.pop();

		// Check if we're out of bounds first.
		if (current_x < left_bounds || current_x >= right_bounds
			|| current_y >= top_bounds || current_y < bottom_bounds)
		{
			continue;
		}
		
		// Check if we're in a wall
		if (world_grid.obstacles.at(current_x).at(current_y))
		{
			continue;
		}

		// Check if we're in a location we've already been at. If not, mark it with a back-pointer.
		if (directional_grid.contents.at(current_x).at(current_y).x == NULL_COORDINATE.x &&
		    directional_grid.contents.at(current_x).at(current_y).y == NULL_COORDINATE.y
		 )
		{
			directional_grid.contents.at(current_x).at(current_y).x = prior_location.x;
			directional_grid.contents.at(current_x).at(current_y).y = prior_location.y;
		}
		else
		{
			continue;
		}

		if (target_x == current_x && target_y == current_y)
		{
			found_path = true;
			break;
		}
		
		// TODO add children to the queue
		// Try adjacent spaces.
		for (int i = 0; i < NUMBER_OF_DIRECTIONS; i++)
		{
			coordinate_t next_location;

			direction_t offset = search_order[i];
			next_location.x = current_location.x + offset.x;
			next_location.y = current_location.y + offset.y;

			next_locations.push(next_location);
			next_locations.push(current_location);
		}
		
		if (iterations % cycles_per_frame == 1)
		{
			display_wavefront("bfs.tga", next_locations);
		}
		iterations++;
		
	} // end while (next_locations.size() > 0)

	if (found_path)
	{
		stack<coordinate_t> * ret = new stack<coordinate_t>;
		int sanity = 0;
		while (!(current_location.x == start_x && current_location.y == start_y))
		{
			ret->push(current_location);

			coordinate_t curr;
			curr.x = current_location.x;
			curr.y = current_location.y;

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
	
} // end breadth_first_search()

void display_wavefront(const char* filename, queue<coordinate_t> locations)
{
	printf("Beginning wavefront image export...\n");
	std::string targetFile = filename;

	// The +1 is because the center of the arena is 0,0
	TGAImage *img = new TGAImage(world_grid.width, world_grid.height);
	Colour c;
	c.a = 255; // Image will be 100% opaque.

	// Draw the obstacles.
	for (int y_n = 0; y_n < world_grid.height; y_n++) {
		for (int x_n = 0; x_n < world_grid.width; x_n++) {
			int curr_pixel = world_grid.obstacles.at(x_n).at(world_grid.height - y_n -1);
			
			if (curr_pixel) { // is an obstacle
				c.r = 128;
				c.g = 128;
				c.b = 128;
			}
			else { // is empty
				c.r = 0;
				c.g = 0;
				c.b = 0;
			}
			img->setPixel(c,x_n,y_n);
		}
	}

	while (locations.size() > 0)
	{
		coordinate_t current = locations.front();
		//coordinate_t current.x = locations->front().x;
		//coordinate_t current.y = locations->front().y;
		locations.pop();
		//printf("Path coordinate: %f, %f\n", current.x, current.y);
		//c = distance_to_color(path->size());
		c.r = 255;
		c.g = 255;
		c.b = 255;
		img->setPixel(c,current.x,world_grid.height - current.y - 1);
	}

	//write the image to disk
	img->WriteImage(filename);

	printf("Path image output to %s\n", filename);
	return;
}

stack<coordinate_t> * iterative_deepening_depth_first_search(int target_x, int target_y, int current_x, int current_y)
{
	const int iterations_per_frame = 1;
	stack<coordinate_t> * ret = NULL;
	for (int depth_n = 0; true; depth_n++)
	{
		printf("Starting iteration %d\n", depth_n);
		fill_visited_grid(world_grid.width, world_grid.height);
		ret = new stack<coordinate_t>;
		ret = iddfs_recursor(target_x, target_y, current_x, current_y, ret, depth_n);
		if (ret != NULL)
		{
			return ret;
		}
		if (depth_n % iterations_per_frame == 0)
		{
			print_visited_grid("iddfs.tga");
		}
	}
	return NULL;
}

// TODO mark visited_grid
stack<coordinate_t> * iddfs_recursor(
	int target_x, int target_y, int current_x, int current_y,
	stack<coordinate_t> * path_so_far, int max_depth
	)
{
	//printf("Searching from %d, %d to find %d, %d\n", current_x, current_y, target_x, target_y);

	if (path_so_far->size() >= max_depth)
	{
		visited_grid.obstacles.at(current_x).at(current_y) = true;
		return NULL;
	}

	coordinate_t current_location;
	current_location.x = current_x;
	current_location.y = current_y;

	//stack<coordinate_t> null_stack;
	//assert(null_stack.size() == 0);

	// Check if we're out of bounds first, so that we can tell whether we're at a valid location in memory.
	// TODO This might ignore the left and bottom edge of the grid, I'm assuming that right_bounds = width - 1.
	if (current_x < left_bounds || current_x >= right_bounds
		|| current_y >= top_bounds || current_y < bottom_bounds)
	{
		//printf("Discarded an out of bounds point, %d, %d\n", current_x, current_y);
		return NULL;
	}
	
	// Check if we're in a location we've already been at. Mark it if we are.
	//if (visited_locations->at(current_x)->at(current_y))
	//if (visited_grid.obstacles.at(current_x).at(current_y))
	//{
		//printf("Discarded a previously visited point, %d, %d\n", current_x, current_y);
		//return NULL;
	//}
	//else
	//{
		//visited_locations->at(current_x)->at(current_y) = true;
		//visited_grid.obstacles.at(current_x).at(current_y) = true;
	//}

	// Check if our current location is an obstacle.
	if (world_grid.obstacles.at(current_x).at(current_y))
	{
		//printf("Bumped against a wall, %d, %d\n", current_x, current_y);
		return NULL;
	}
	else
	{
		; // We're fine, keep going.
	}

	// Now we're going to try to find a correct path from the current state.
	path_so_far->push(current_location);
	
	// Check if we've reached our target.
	if (target_x == current_x && target_y == current_y)
	{
		return path_so_far;
	}
	
	// Try adjacent spaces.
	for (int i = 0; i < NUMBER_OF_DIRECTIONS; i++)
	{
		direction_t offset = search_order[i];
		stack<coordinate_t> * ret = iddfs_recursor(target_x, target_y, current_x + offset.x, current_y + offset.y, path_so_far, max_depth);
		if (ret != NULL) // The null stack has a size of 0
		{
			return ret;
		}
		else
		{
			; // Try the next one.
		}
	}
	
	// Report that none of our children reached the goal.
	path_so_far->pop();
	nodes_popped++;
	return NULL;
}


void print_visited_grid(const char* filename)
{
	printf("Beginning visited grid image export...\n");
	std::string targetFile = filename;

	// The +1 is because the center of the arena is 0,0
	TGAImage *img = new TGAImage(world_grid.width, world_grid.height);
	Colour c;
	c.a = 255; // Image will be 100% opaque.

	// Draw the obstacles.
	for (int y_n = 0; y_n < world_grid.height; y_n++) {
		for (int x_n = 0; x_n < world_grid.width; x_n++) {
			int curr_pixel = world_grid.obstacles.at(x_n).at(world_grid.height - y_n -1);
			
			if (curr_pixel) { // is an obstacle
				c.r = 128;
				c.g = 128;
				c.b = 128;
			}
			else { // is empty
				c.r = 0;
				c.g = 0;
				c.b = 0;
			}
			
			// If the same location is marked on the visited grid, draw in white instead.
			if (visited_grid.obstacles.at(x_n).at(world_grid.height - y_n -1))
			{
				c.r = 255;
				c.g = 255;
				c.b = 255;
			}
			
			img->setPixel(c,x_n,y_n);
		}
	}

	//write the image to disk
	img->WriteImage(filename);

	printf("Path image output to %s\n", filename);
	return;
}

void populate_tank_grid()
{
	
	
	tank_weights.width = world_grid.width;
	tank_weights.height = world_grid.height;
	tank_weights.weights.resize(tank_weights.height);
	
	for (int height_n = 0; height_n < tank_weights.height; height_n++) {
		for (int width_n = 0; width_n < tank_weights.width; width_n++) {
			int value = 0;
			if (avoid_tanks)
			{
				for (int tank_n = 0; tank_n < enemy_tanks_coors->size(); tank_n++)
				{
					int diff_x = fabs(width_n - enemy_tanks_coors->at(tank_n).x);
					int diff_y = fabs(height_n - enemy_tanks_coors->at(tank_n).y);
					int distance = diff_x + diff_y; // This produces a diamond-shaped field around each tank.
					if (distance <= tank_field_size)
					{
						value += tank_field_strength;
					}
				}
			}
			tank_weights.weights.at(height_n).push_back(value);
		}
	}
	print_tank_weights("tanks.tga");
}

void print_tank_weights(const char* filename)
{
	printf("Beginning tank grid image export...\n");
	std::string targetFile = filename;

	// The +1 is because the center of the arena is 0,0
	TGAImage *img = new TGAImage(world_grid.width, world_grid.height);
	Colour c;
	c.a = 255; // Image will be 100% opaque.

	// Draw the obstacles.
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
			} else if (curr_pixel == UNEXPLORED_VALUE) {
				c.r = 0;
				c.g = 0;
				c.b = 255;
			}
			
			// Draw tank fields in red
			int tank_weight = tank_weights.weights.at(x_n).at(y_n);
			if (tank_weight > 0)
			{
				c.r = min(255, 128 + 10 * tank_weight);
			}
			
			img->setPixel(c,x_n,y_n);
		}
	}

	//write the image to disk
	img->WriteImage(filename);

	printf("Tank weights image output to %s\n", filename);
	return;
}

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
	
			if (penalized_mode)
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

void display_pqueue_wavefront(const char* filename, priority_queue<prioritizable_node_t> locations)
{
	printf("Beginning wavefront image export...\n");
	std::string targetFile = filename;

	// The +1 is because the center of the arena is 0,0
	TGAImage *img = new TGAImage(world_grid.width, world_grid.height);
	Colour c;
	c.a = 255; // Image will be 100% opaque.

	// Draw the obstacles.
	for (int y_n = 0; y_n < world_grid.height; y_n++) {
		for (int x_n = 0; x_n < world_grid.width; x_n++) {
			int curr_pixel = world_grid.obstacles.at(x_n).at(world_grid.height - y_n -1);
			
			if (curr_pixel) { // is an obstacle
				c.r = 128;
				c.g = 128;
				c.b = 128;
			}
			else { // is empty
				c.r = 0;
				c.g = 0;
				c.b = 0;
			}
			img->setPixel(c,x_n,y_n);
		}
	}

	while (locations.size() > 0)
	{
		prioritizable_node_t current = locations.top();
		//coordinate_t current.x = locations->front().x;
		//coordinate_t current.y = locations->front().y;
		locations.pop();
		//printf("Path coordinate: %f, %f\n", current.x, current.y);
		//c = distance_to_color(path->size());
		c.r = 255;
		c.g = 255;
		c.b = 255;
		img->setPixel(c,current.x,world_grid.height - current.y - 1);
	}

	//write the image to disk
	img->WriteImage(filename);

	printf("Path image output to %s\n", filename);
	return;
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
}

void set_tank_heading(int tank_n, stack<coordinate_t> * path, BZRC* my_team)
{
	const int lookahead_distance = 1;
	
	direction_t ret;

	if (path == NULL || path->empty())
	{
		printf("Received a bogus path for tank %d.\n", tank_n);
		tank_brains->at(tank_n).heading = ret;
		tank_brains->at(tank_n).current_goal = NULL_COORDINATE;
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
	
	tank_brains->at(tank_n).heading = ret;
	
	string s = "tank_#_path.tga";
	s.at(5) = 48 + tank_n; // Convert the tank number into a character, 0 is ascii 48
	
	display_path(s.c_str(), path);
	
	return;
}

void keep_tank_on_course(int tank_n, BZRC* my_team)
{
	const double turn_strength = 1.0;
	const double acceptable_difference = 1.0;	// As long as our impulse is within an arc this many radians wide, drive at full speed

	direction_t impulse = tank_brains->at(tank_n).heading;
	impulse.y *= -1; // stupid inverstion crap

	double randomness = ((rand() % 11) - 5) / 5.0 * acceptable_difference; // -5 to 5, scaled to somewhere withing the acceptable_difference cone

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
			//my_team->shoot(tank_n);
		}
		else
		{
			// Set impulse to one-quarter speed until we're pointing in the right direction
			speed = 0.25;
		}	

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
}

void populate_world_grid(int size)
{
	world_grid.width = size;
	world_grid.height = size;
	world_grid.obstacles.resize(size);

	explored_probabilities.width = size;
	explored_probabilities.height = size;
	explored_probabilities.weights.resize(size);

	for (int height_n = 0; height_n < size; height_n++) {
		for (int width_n = 0; width_n < size; width_n++) {
			world_grid.obstacles.at(height_n).push_back(UNEXPLORED_VALUE);
			explored_probabilities.weights.at(height_n).push_back(INITIAL_OBSTACLE_PROBABILITY);
		}
	}
}

void update_tank_vision(BZRC* my_team)
{
	for (int tank_n = 0; tank_n < my_tanks->size(); tank_n++)
	{
		grid_t ret;
		my_team->get_tank_vision_grid(ret, tank_n);
		
		ret.left += world_grid.width / 2;
		ret.top += world_grid.height / 2; // Is actually bottom

		//printf("Getting tank vision. Top: %d. Bottom: %d. Left: %d. Right: %d.\n", top, bottom, left, right);

		//printf("Got tank vision grid for tank %d. Top: %d, Left: %d, Width: %d, Height: %d.\n", tank_n, ret.top, ret.left, ret.width, ret.height);

		for (int x = 0; x < ret.width; x++)
		{
			for (int y = 0; y < ret.height; y++)
			{
				int current_x = ret.left + x;
				int current_y = y + world_grid.height - ret.height - ret.top;
				
				//printf("Printing vision for tank %d at pixel %d, %d.\n", tank_n, current_x, current_y);
				
				if (current_y < 0 || current_y >= world_grid.height || current_x < 0 || current_x >= world_grid.width)
				{
					printf("ERROR: Out of bounds on tank vision update at %d, %d.", current_x, current_y);
					continue;
				}
				assert(current_x >= 0);
				assert(current_y >= 0);
				assert(current_x < world_grid.width);
				assert(current_y < world_grid.height);

				int observed_value = ret.obstacles.at(x).at(y);

				update_world_obstacles(current_x, current_y, observed_value);
			}
		}
	}
	return;
} // end update_tank_vision()

void update_world_obstacles(int current_x, int current_y, int observed_value)
{
	const double true_positive = .97;
	const double false_negative = .03;
	const double false_positive = .1;
	const double true_negative = .9;

	int obstacle_ret = UNEXPLORED_VALUE;
	double historical_ret = INITIAL_OBSTACLE_PROBABILITY;
	
	double curr_value = explored_probabilities.weights.at(current_x).at(current_y);
	
	int curr_value_occupied = (curr_value > CONF_THRESHOLD) ? 1 : 0;
	
	double prob = 0.0;
	
	if (observed_value)
	{
		if (curr_value)
		{
			prob = true_positive;
		}
		else
		{
			prob = false_positive;
		}
	}
	else
	{
		if (curr_value)
		{
			prob = false_negative;
		}
		else
		{
			prob = true_negative;
		}
	}
	
	double B = 0.0;
	if (observed_value)
	{
		B = false_positive;
	}
	else
	{
		B = true_negative;
	}
	B *= (1 - curr_value);
	
	double A = prob * curr_value;
	historical_ret = A / (A + B);
	
	if (historical_ret > CONF_THRESHOLD)
	{
		obstacle_ret = 1;
	}
	else if (historical_ret < 1 - CONF_THRESHOLD)
	{
		obstacle_ret = 0;
	}
	
	world_grid.obstacles.at(current_x).at(current_y) = obstacle_ret;
	explored_probabilities.weights.at(current_x).at(current_y) = historical_ret;
	return;
} // end update_world_obstacles

void shoot_at_target(int tank_n, BZRC* my_team, coordinate_t target)
{
	const double turn_strength = 5.0;
	const double acceptable_difference = 0.03;	// As long as our impulse is within an arc this many radians wide, shoot

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

		// Make the tank slow down during sharp turns.
		if (fabs(difference) < acceptable_difference / 2)
		{
			//speed = 1.0;
			my_team->shoot(tank_n);
			print_skeet_vision("skeet.tga", source.x, source.y, target.x, target.y);
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
}

void print_skeet_vision(const char* filename, int my_tank_x, int my_tank_y, int shooting_target_x, int shooting_target_y)
{
	printf("Beginning target tracking image output.\n");
	std::string targetFile = filename;
	
	// The +1 is because the center of the arena is 0,0
	TGAImage *img = new TGAImage(world_grid.width, world_grid.height);
	Colour c;
	c.a = 255; // Image will be 100% opaque.
	
	for (int y_n = 0; y_n < world_grid.height; y_n++) {
		for (int x_n = 0; x_n < world_grid.width; x_n++) {
			c.r = 0;
			c.g = 0;
			c.b = 0;
			img->setPixel(c,x_n,y_n);
		}
	}

	coordinate_t draw;

	// My tank's location is red.
	c.r = 255;
	c.g = 0;
	c.b = 0;
	draw.x = my_tank_x;
	draw.y = my_tank_y;
	if (draw.x > 0 && draw.y > 0 && draw.x < world_grid.width && draw.y < world_grid.height)
		img->setPixel(c,draw.x,draw.y);
	
	// The last few points I thought the enemy tank was at are dark green
	// TODO
	c.r = 0;
	c.g = 64;
	c.b = 0;
	
	// The last few places I observed the enemy tank are blue
	c.r = 0;
	c.g = 0;
	c.b = 32;
	for (int place_n = 0; place_n < observed_enemy_coordinates.size(); place_n++)
	{
		coordinate_t next_place = observed_enemy_coordinates.at(place_n);
		draw.y = next_place.x;
		draw.x = next_place.y;
		c.b += 4;
		//c.r += 2;
		c.g += 2;
		//printf("Drawing previous observation at %f, %f.\n", draw.x, draw.y);
		if (draw.x > 0 && draw.y > 0 && draw.x < world_grid.width && draw.y < world_grid.height)
			img->setPixel(c,draw.x,draw.y);
	}
	
	// The point I think the enemy tank is at is light green
	// Using the previous For loop
	c.r = 0;
	c.g = 255;
	c.b = 0;
	printf("Drawing current tank observation at %f, %f.\n", draw.x, draw.y);
	if (draw.x > 0 && draw.y > 0 && draw.x < world_grid.width && draw.y < world_grid.height)
		img->setPixel(c,draw.x,draw.y);
	
	// The point I'm shooting at (accounting for velocity and acceleration) is yellow
	c.r = 255;
	c.g = 255;
	c.b = 0;
	draw.x = shooting_target_x;
	draw.y = shooting_target_y;
	if (draw.x > 0 && draw.y > 0 && draw.x < world_grid.width && draw.y < world_grid.height)
		img->setPixel(c,draw.x,draw.y);
	// Draw a crosshair
	draw.x--;
	if (draw.x > 0 && draw.y > 0 && draw.x < world_grid.width && draw.y < world_grid.height)
		img->setPixel(c,draw.x,draw.y);
	draw.x++;draw.x++;
	if (draw.x > 0 && draw.y > 0 && draw.x < world_grid.width && draw.y < world_grid.height)
		img->setPixel(c,draw.x,draw.y);
	draw.x--;
	draw.y++;
	if (draw.x > 0 && draw.y > 0 && draw.x < world_grid.width && draw.y < world_grid.height)
		img->setPixel(c,draw.x,draw.y);
	draw.y--;draw.y--;
	if (draw.x > 0 && draw.y > 0 && draw.x < world_grid.width && draw.y < world_grid.height)
		img->setPixel(c,draw.x,draw.y);

	//write the image to disk
	img->WriteImage(filename);

	printf("Skeet vision output to %s.\n", targetFile.c_str());
	return;
} // end print_skeet_vision
