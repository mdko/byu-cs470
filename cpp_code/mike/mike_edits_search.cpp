// 5/14 3 hours each
// 5/15 4 hours each

#include <iostream>
#include "470bot.h"
#include <ctime>
#include <chrono>
#include <sys/time.h>
#include <unistd.h>
#include <cstdlib>
#include "Image.h"

using namespace std;

const char *kDefaultServerName = "localhost";
const int kDefaultServerPort = 4000;

#define NUMBER_OF_DIRECTIONS 8
static direction_t search_order[NUMBER_OF_DIRECTIONS]; // Defined in world_init
static grid_t world_grid;
static coordinate_t NULL_COORDINATE;
static double left_bounds;
static double right_bounds;
static double top_bounds;
static double bottom_bounds;
static coordinate_t green_flag_coor;
static vector<coordinate_t> *enemy_tanks_coors;
static grid_t visited_grid;

static vector <tank_t> *my_tanks;
static coordinate_t red_tank_coor;

int main(int argc, char *argv[]) {
	define_constants();
	
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

	BZRC MyTeam = BZRC(pcHost, nPort, false);
	if(!MyTeam.GetStatus()) {
		cout << "Can't connect to BZRC server." << endl;
		exit(1);
	}

	// Calling agent code
	world_init(&MyTeam);

	// TODO let the user pass command line arguments to specify which search to perform

	stack<coordinate_t> path;
	path.push(red_tank_coor);
	path.push(red_tank_coor);
	
	fill_visited_grid(world_grid.width, world_grid.height);
	
	//vector<vector<bool> *> * empty_bool_grid = get_empty_bool_grid(world_grid.width, world_grid.height);
	
	depth_first_search(visited_grid, red_tank_coor.x, red_tank_coor.y);
	// Do depth-first search.
	/*stack<coordinate_t> ret = recursive_depth_first_search(
		// double target_x, double target_y,
		green_flag_coor.x, green_flag_coor.y,
		// double current_x, double current_y,
		red_tank_coor.x, red_tank_coor.y,
		// grid_t obstacles, vector<vector<bool> > * visited_locations,
		//world_grid, empty_bool_grid,
		world_grid, NULL,
		// stack<coordinate_t> path_so_far
		path
	);*/
	
	/*while (ret.size() > 0)
	{
		coordinate_t current = ret.top();
		ret.pop();
		printf("Path coordinate: %d, %d\n", current.x, current.y);
	}*/

	MyTeam.Close();
	return 0;
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
	search_order[4].y = search_order[2].x;
	// Up-Right
	search_order[5].x = search_order[1].x;
	search_order[5].y = search_order[2].x;
	// Down-Left
	search_order[6].x = search_order[0].x;
	search_order[6].y = search_order[3].x;
	// Down-Right
	search_order[7].x = search_order[1].x;
	search_order[7].y = search_order[3].x;
}

void world_init(BZRC *my_team) {
	my_tanks = new vector<tank_t>();
	my_tanks->clear();
	my_team->get_mytanks(my_tanks);
	printf("I have %d tanks.\n", my_tanks->size());
	red_tank_coor.x = my_tanks->at(0).pos[0];
	red_tank_coor.y = my_tanks->at(0).pos[1];
	printf("My tank is at %f, %f facing %d.\n", red_tank_coor.x, red_tank_coor.y, my_tanks->at(0).angle);

	my_team->get_occgrid(world_grid);
	enemy_tanks_coors = new vector<coordinate_t>();
	store_enemy_tanks_coors(my_team);
	store_green_flag(my_team);
	store_red_tank(my_team);
	
	//print_grid("obstacles.tga");
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

	return;
}

void store_red_tank(BZRC* my_team) {
	
	my_tanks = new vector<tank_t>();
	my_tanks->clear();
	bool got_tanks = my_team->get_mytanks(my_tanks);
	
	assert(got_tanks);
	assert(my_tanks->size() > 0);
	tank_t red_tank_0 = my_tanks->at(0);
	red_tank_coor.x = red_tank_0.pos[0];
	red_tank_coor.y = red_tank_0.pos[1];
	printf("First tank %s at: %d, %d (un-shifted coordinates)\n", my_tanks->at(0).callsign.c_str(), red_tank_coor.x, red_tank_coor.y);
	
	/*for (int tank_n = 0; tank_n < my_tanks->size(); tank_n++)
	{
		tank_t curr_tank = my_tanks->at(tank_n);
		if (curr_tank.index == 0)
		{
			red_tank_coor.x = curr_tank.pos[0];
			red_tank_coor.y = curr_tank.pos[1];
		}
	}*/
	
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
			printf("Enemy flag at: %f, %f\n", green_flag_coor.x, green_flag_coor.y);
		}
	}
}


void store_enemy_tanks_coors(BZRC* my_team) {
	enemy_tanks_coors->clear();
	
	vector <otank_t> * other_tanks = new vector<otank_t>;
	my_team->get_othertanks(other_tanks);
	
	for (int tank_n = 0; tank_n < other_tanks->size(); tank_n++) {
		otank_t enemy_tank = other_tanks->at(tank_n);
		coordinate_t tank_coor;
		tank_coor.x = enemy_tank.pos[0]; // x-coor
		tank_coor.y = enemy_tank.pos[1]; // y-coor
		enemy_tanks_coors->push_back(tank_coor);
	}
}

void fill_visited_grid(int width, int height) {
	visited_grid.width = width;
	visited_grid.height = height;
	visited_grid.obstacles.resize(visited_grid.height);
	
	for (int height_n = 0; height_n < visited_grid.height; height_n++) {
		for (int width_n = 0; width_n < visited_grid.width; width_n++) {
			int value = false; //48 is ascii value of 0
			visited_grid.obstacles.at(height_n).push_back(value);
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

// TODO place in the body of the CPP doc
// TODO you can make obstacles global static to save on memory and copy time
stack<coordinate_t> recursive_depth_first_search(
	int target_x, int target_y, int current_x, int current_y,
	grid_t obstacles, vector<vector<bool> * > * visited_locations, stack<coordinate_t> path_so_far
	)
{
	//printf("Searching from %f, %f to find %f, %f\n", current_x, current_y, target_x, target_y);

	coordinate_t current_location;
	current_location.x = current_x;
	current_location.y = current_y;

	stack<coordinate_t> null_stack;
	assert(null_stack.size() == 0);

	// Check if we're out of bounds first, so that we can tell whether we're at a valid location in memory.
	// TODO This might ignore the left and bottom edge of the grid, I'm assuming that right_bounds = width - 1.
	if (current_x < left_bounds || current_x >= right_bounds
		|| current_y < top_bounds || current_y >= bottom_bounds)
	{
		//printf("Discarded an out of bounds point, %f, %f\n", current_x, current_y);
		return null_stack;
	}
	
	// Check if we're in a location we've already been at. Mark it if we are.
	//if (visited_locations->at(current_x)->at(current_y))
	if (visited_grid.obstacles.at(current_x).at(current_y))
	{
		//printf("Discarded a previously visited point, %f, %f\n", current_x, current_y);
		return null_stack;
	}
	else
	{
		//visited_locations->at(current_x)->at(current_y) = true;
		visited_grid.obstacles.at(current_x).at(current_y) = true;
	}

	// Check if our current location is an obstacle.
	if (obstacles.obstacles.at(current_x).at(current_y))
	{
		printf("Bumped against a wall, %f, %f\n", current_x, current_y);
		return null_stack;
	}
	else
	{
		; // We're fine, keep going.
	}

	// Now we're going to try to find a correct path from the current state.
	path_so_far.push(current_location);
	
	// Check if we've reached our target.
	if (target_x == current_x && target_y == current_y)
	{
		return path_so_far;
	}
	
	// Try adjacent spaces.
	for (int i = 0; i < NUMBER_OF_DIRECTIONS; i++)
	{
		direction_t offset = search_order[i];
		stack<coordinate_t> ret = recursive_depth_first_search(target_x, target_y, current_x + offset.x, current_y + offset.y, obstacles, visited_locations, path_so_far);
		if (ret.size() != 0) // The null stack has a size of 0
		{
			return ret;
		}
		else
		{
			; // Try the next one.
		}
	}
	
	// Report that none of our children reached the goal.
	return null_stack;
}



/*result iterative_deepening_depth_first ()
{
	for (int depth_n = 0; depth_< 1000000000; depth_n++) 
	{
		result = depth_limited_search(problem, depth_n);
		if (result != cutoff)
			return result; // solution or failure
	}
}

result depth_limited_search ()
{
	return recursive_dls()
}*/


 /*void depth_first_search (grid_t visited_grid, double current_x, double current_y) 
 {
	  printf("Searching from %f, %f to find %f, %f\n", current_x, current_y, green_flag_coor.x, green_flag_coor.y);
	  if (current_x == green_flag_coor.x && current_y == green_flag_coor.y) {
		printf("We've reached the target!\n");
		return;
	  }
	  // "Visit" current square
      visited_grid.obstacles.at(current_x).at(current_y) = true;
      
      // Cycle through each neighbor of current square
      for (int edge_n = 0; edge_n < NUMBER_OF_DIRECTIONS; edge_n++)
      {
		  direction_t offset = search_order[edge_n];
		  
		  double new_x = current_x + offset.x;
		  double new_y = current_y + offset.y;
		  // Go to an adjacent square that hasn't been visited, is currently in bounds, and doesn't have an obstacle
		  if (visited_grid.obstacles.at(new_x).at(new_y) == false &&
				in_bounds(new_x, new_y) &&
				!contains_obstacle(new_x, new_y))
		  {
              depth_first_search(visited_grid, new_x, new_y);
			  visited_grid.obstacles.at(new_x).at(new_y) = true;
		  }
	  }
}*/

/*bool in_bounds(double x, double y)
{
	if (x >= left_bounds && 
		x < right_bounds && 
		y < top_bounds &&
		y >= bottom_bounds)
		return true;
	return false;
	
}*/

/*bool contains_obstacle(double x, double y)
{
	if (world_grid.obstacles.at(x).at(y) == true)
		return true;
	return false;
	
}*/
	

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

	//write the image to disk
	img->WriteImage(filename);

	cerr << "Grid output to " << targetFile << + "\n";
	return;
}










