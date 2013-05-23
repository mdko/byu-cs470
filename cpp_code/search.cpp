// 5/14 3 hours each
// 5/15 Mike: 1 hour
// 5/16 4 hours each
// 5/16 Thomas: 2 hours
// 5/17 8 hours 45 minutes each

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
static direction_grid_t directional_grid;
static weight_grid_t tank_weights;

static vector <tank_t> *my_tanks;
static coordinate_t red_tank_coor;

static int recursive_counter = 0;

static int nodes_popped = 0;
static int final_cost = 0;

string searchType = "";

bool penalized_mode;
bool avoid_tanks;

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
	if(argc < 4) {
		printf("Error! Specify what type of search to perform.\n");
		exit(0);
	}
	else
	{
		searchType = argv[3];
	}
		

	// You can add two numbers (1 is true, 0 is false) after the type to specify penalties.
	// If you specify them, you MUST specify both.
	// First one is walls. Second one is tanks.
	if (searchType == "astar" || searchType == "ucost")
	{
		if (argc < 5)
		{
			penalized_mode = true;
			avoid_tanks = true;
		}
		else
		{
			penalized_mode = atoi(argv[4]);
			avoid_tanks = atoi(argv[5]);
		}
	}
	else
	{
		penalized_mode = false;
		avoid_tanks = false;
	}

	BZRC MyTeam = BZRC(pcHost, nPort, false);
	if(!MyTeam.GetStatus()) {
		cout << "Can't connect to BZRC server." << endl;
		exit(1);
	}

	// Calling agent code
	world_init(&MyTeam);

	if (searchType == "dfs")
	{
		// Do depth-first search.

		int iterations = 10000;

		/*
		if (argc >= 5)
		{
			iterations = atoi(argv[4]);
			printf("Searching to a depth of %d iterations.\n", iterations);			
		}
		*/

		stack<coordinate_t> * path = new stack<coordinate_t>;
		//path->push(red_tank_coor);

		fill_visited_grid(world_grid.width, world_grid.height);
	
		//vector<vector<bool> *> * empty_bool_grid = get_empty_bool_grid(world_grid.width, world_grid.height);
	
		stack<coordinate_t> * ret = recursive_depth_first_search(
			// double target_x, double target_y,
			green_flag_coor.x, green_flag_coor.y,
			// double current_x, double current_y,
			red_tank_coor.x, red_tank_coor.y,
			// vector<vector<bool> > * visited_locations,
			//empty_bool_grid,
			NULL,
			// stack<coordinate_t> * path_so_far
			path, iterations
		);


		if (ret == NULL)
		{
			printf("Sorry!\nCould not find path to flag.\n");
			return 0;
		}

		final_cost = path->size();
		printf("Path found! Printing...");
		display_path("dfs.tga", path);

		//while (ret->size() > 0)
		//{
		//	coordinate_t current = ret->top();
		//	ret->pop();
		//	printf("Path coordinate: %f, %f\n", current.x, current.y);
		//}

	} // end if (searchType == "dfs")
	else if (searchType == "bfs")
	{
		fill_directional_grid(world_grid.width, world_grid.height);
		stack<coordinate_t> * path = breadth_first_search(green_flag_coor.x, green_flag_coor.y, red_tank_coor.x, red_tank_coor.y);
		printf("Path found! Printing...");
		final_cost = path->size();
		display_path("bfs.tga", path);
	} // end if (searchtype == "bfs")
	else if (searchType == "iddfs")
	{
		stack<coordinate_t> * path = iterative_deepening_depth_first_search(green_flag_coor.x, green_flag_coor.y, red_tank_coor.x, red_tank_coor.y);
		final_cost = path->size();
		display_path("iddfs.tga", path);
	}
	else if (searchType == "ucost")
	{
		fill_directional_grid(world_grid.width, world_grid.height);
		stack<coordinate_t> * path = best_first_search(green_flag_coor.x, green_flag_coor.y, red_tank_coor.x, red_tank_coor.y, false);
		display_path("ucost.tga", path);
	}
	else if (searchType == "astar")
	{
		fill_directional_grid(world_grid.width, world_grid.height);
		stack<coordinate_t> * path = best_first_search(green_flag_coor.x, green_flag_coor.y, red_tank_coor.x, red_tank_coor.y, true);
		display_path("astar.tga", path);
	}
	else
	{
		printf("Error, searchType %s is invalid.\n", searchType.c_str());
	}
	MyTeam.Close();
	
	printf("Nodes Popped: %d\n", nodes_popped);
	printf("Final Path Cost: %d\n", final_cost);

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

	my_team->get_occgrid(world_grid);
	enemy_tanks_coors = new vector<coordinate_t>();
	store_enemy_tanks_coors(my_team);
	populate_tank_grid();

	store_green_flag(my_team);
	//store_red_tank(my_team);
	
	print_grid("obstacles.tga");
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
	
	for (int tank_n = 0; tank_n < other_tanks->size(); tank_n++) {
		otank_t enemy_tank = other_tanks->at(tank_n);
		coordinate_t tank_coor;
		tank_coor.y = enemy_tank.pos[0]; // x-coor
		tank_coor.x = enemy_tank.pos[1]; // y-coor
		tank_coor.x += world_grid.width / 2;
		tank_coor.y += world_grid.height / 2;		
		enemy_tanks_coors->push_back(tank_coor);
	}
}

void fill_visited_grid(int width, int height) {
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
/*
 void depth_first_search DFS(grid_t visited_grid, double current_x, double current_y) {
      visited_grid.obstacles.at(current_x).at(current_y) = true;
      for (int edge_n = 0; edge_n < NUMBER_OF_DIRECTIONS; edge_n++) {
		  direction_t offset = search_order[i];
		  if (visited_grid.obstacles.at(current_x + offset.x).at(current_y _ offset_y) == false) {
              double new_x = current_x + offset.x;
              double new_y = current_y + offset.y;
              //if vertex w is unexplored then
				visited_grid.obstacles.at(new_x).at(new_y) = true;
                DFS(visited_grid, new_x, new_y);
              //else
              //   label e as a back edge
			 }
		 }
		*/	 


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

void display_path(const char* filename, stack<coordinate_t> * path)
{
	assert(path != NULL);

	printf("Beginning path image export...\n");
	std::string targetFile = filename;
	
	int path_weighting = path->size();

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

	printf("Path image output to %s\n", filename);
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
			printf("Backtracing %f, %f.\nCurrent size is %d\n", current_location.x, current_location.y, ret->size());
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
	const int tank_field_size = 25;
	const int tank_field_strength = 2;
	
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
	assert(left_bounds == 0);
	assert(bottom_bounds == 0);

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
	int cycles_per_frame;
	if (use_heuristic)
	{
		cycles_per_frame = 201;
	}
	else
	{
		cycles_per_frame = 501;
	}
	
	int iterations = 0;
	while (next_locations.size() > 0)
	{
		current_location = next_locations.top();
		int current_x = current_location.x;
		int current_y = current_location.y;
		next_locations.pop();
		nodes_popped++;
		
		//printf("Cost here is %f.\n", current_location.cost);

		// Check if we're out of bounds first.
		if (current_x < left_bounds || current_x >= right_bounds
			|| current_y >= top_bounds || current_y < bottom_bounds)
		{
			//printf("Rejected because out of bounds.\n");
			continue;
		}
		
		// Check if we're in a wall
		if (world_grid.obstacles.at(current_x).at(current_y))
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

			if (avoid_tanks)
			{
				next_location.cost += tank_weights.weights.at(current_location.x).at(current_location.y);
			}
			
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
		
		if (iterations % cycles_per_frame == 1)
		{
			string filename = "";
			if (use_heuristic)
			{
				filename = "astar.tga";
			}
			else
			{
				filename = "ucost.tga";
			}
			display_pqueue_wavefront(filename.c_str(), next_locations);
		}
		iterations++;
		
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

			current_location.x = directional_grid.contents.at(curr.x).at(curr.y).x;
			current_location.y = directional_grid.contents.at(curr.x).at(curr.y).y;
			if(sanity++ > 20000)
			{
				printf("Been backtracing too long, start panicking.\n");
				break;
			}
			printf("Backtracing %f, %f.\nCurrent size is %d\n", current_location.x, current_location.y, ret->size());
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
