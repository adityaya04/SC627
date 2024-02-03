/*=================================================================
 *
 * planner.c
 *
 *=================================================================*/
#include <math.h>
#include <random>
#include <vector>
#include <array>
#include <algorithm>

#include <tuple>
#include <string>
#include <stdexcept>
#include <regex> // For regex and split logic
#include <iostream> // cout, endl
#include <fstream> // For reading/writing files
#include <assert.h> 

/* Input Arguments */
#define	MAP_IN      prhs[0]
#define	ARMSTART_IN	prhs[1]
#define	ARMGOAL_IN     prhs[2]
#define	PLANNER_ID_IN     prhs[3]

/* Planner Ids */
#define RRT         0
#define RRTSTAR     1
#define PRM         2

/* Output Arguments */
#define	PLAN_OUT	plhs[0]
#define	PLANLENGTH_OUT	plhs[1]

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) (Y*XSIZE + X)

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define PI 3.141592654

//the length of each link in the arm
#define LINKLENGTH_CELLS 10

// Some potentially helpful imports
using std::vector;
using std::array;
using std::string;
using std::runtime_error;
using std::tuple;
using std::make_tuple;
using std::tie;
using std::cout;
using std::endl;

//*******************************************************************************************************************//
//                                                                                                                   //
//                                                GIVEN FUNCTIONS                                                    //
//                                                                                                                   //
//*******************************************************************************************************************//

/// @brief 
/// @param filepath 
/// @return map, x_size, y_size
tuple<double*, int, int> loadMap(string filepath) {
	std::FILE *f = fopen(filepath.c_str(), "r");
	if (f) {
	}
	else {
		printf("Opening file failed! \n");
		throw runtime_error("Opening map file failed!");
	}
	int height, width;
	if (fscanf(f, "height %d\nwidth %d\n", &height, &width) != 2) {
		throw runtime_error("Invalid loadMap parsing map metadata");
	}
	
	////// Go through file and add to m_occupancy
	double* map = new double[height*width];

	double cx, cy, cz;
	for (int y = 0; y < height; y++) {
		for (int x = 0; x < width; x++) {
			char c;
			do {
				if (fscanf(f, "%c", &c) != 1) {
					throw runtime_error("Invalid parsing individual map data");
				}
			} while (isspace(c));
			if (!(c == '0')) { 
				map[y+x*width] = 1; // Note transposed from visual
			} else {
				map[y+x*width] = 0;
			}
		}
	}
	fclose(f);
	return make_tuple(map, width, height);
}

// Splits string based on deliminator
vector<string> split(const string& str, const string& delim) {   
		// https://stackoverflow.com/questions/14265581/parse-split-a-string-in-c-using-string-delimiter-standard-c/64886763#64886763
		const std::regex ws_re(delim);
		return { std::sregex_token_iterator(str.begin(), str.end(), ws_re, -1), std::sregex_token_iterator() };
}


double* doubleArrayFromString(string str) {
	vector<string> vals = split(str, ",");
	double* ans = new double[vals.size()];
	for (int i = 0; i < vals.size(); ++i) {
		ans[i] = std::stod(vals[i]);
	}
	return ans;
}

bool equalDoubleArrays(double* v1, double *v2, int size) {
    for (int i = 0; i < size; ++i) {
        if (abs(v1[i]-v2[i]) > 1e-3) {
            cout << endl;
            return false;
        }
    }
    return true;
}

typedef struct {
	int X1, Y1;
	int X2, Y2;
	int Increment;
	int UsingYIndex;
	int DeltaX, DeltaY;
	int DTerm;
	int IncrE, IncrNE;
	int XIndex, YIndex;
	int Flipped;
} bresenham_param_t;


void ContXY2Cell(double x, double y, short unsigned int* pX, short unsigned int *pY, int x_size, int y_size) {
	double cellsize = 1.0;
	//take the nearest cell
	*pX = (int)(x/(double)(cellsize));
	if( x < 0) *pX = 0;
	if( *pX >= x_size) *pX = x_size-1;

	*pY = (int)(y/(double)(cellsize));
	if( y < 0) *pY = 0;
	if( *pY >= y_size) *pY = y_size-1;
}


void get_bresenham_parameters(int p1x, int p1y, int p2x, int p2y, bresenham_param_t *params) {
	params->UsingYIndex = 0;

	if (fabs((double)(p2y-p1y)/(double)(p2x-p1x)) > 1)
		(params->UsingYIndex)++;

	if (params->UsingYIndex)
		{
			params->Y1=p1x;
			params->X1=p1y;
			params->Y2=p2x;
			params->X2=p2y;
		}
	else
		{
			params->X1=p1x;
			params->Y1=p1y;
			params->X2=p2x;
			params->Y2=p2y;
		}

	 if ((p2x - p1x) * (p2y - p1y) < 0)
		{
			params->Flipped = 1;
			params->Y1 = -params->Y1;
			params->Y2 = -params->Y2;
		}
	else
		params->Flipped = 0;

	if (params->X2 > params->X1)
		params->Increment = 1;
	else
		params->Increment = -1;

	params->DeltaX=params->X2-params->X1;
	params->DeltaY=params->Y2-params->Y1;

	params->IncrE=2*params->DeltaY*params->Increment;
	params->IncrNE=2*(params->DeltaY-params->DeltaX)*params->Increment;
	params->DTerm=(2*params->DeltaY-params->DeltaX)*params->Increment;

	params->XIndex = params->X1;
	params->YIndex = params->Y1;
}

void get_current_point(bresenham_param_t *params, int *x, int *y) {
	if (params->UsingYIndex) {
        *y = params->XIndex;
        *x = params->YIndex;
        if (params->Flipped)
            *x = -*x;
    }
	else {
        *x = params->XIndex;
        *y = params->YIndex;
        if (params->Flipped)
            *y = -*y;
    }
}

int get_next_point(bresenham_param_t *params) {
	if (params->XIndex == params->X2) {
        return 0;
    }
	params->XIndex += params->Increment;
	if (params->DTerm < 0 || (params->Increment < 0 && params->DTerm <= 0))
		params->DTerm += params->IncrE;
	else {
        params->DTerm += params->IncrNE;
        params->YIndex += params->Increment;
	}
	return 1;
}



int IsValidLineSegment(double x0, double y0, double x1, double y1, double*	map,
			 int x_size, int y_size) {
	bresenham_param_t params;
	int nX, nY; 
	short unsigned int nX0, nY0, nX1, nY1;

	//printf("checking link <%f %f> to <%f %f>\n", x0,y0,x1,y1);
		
	//make sure the line segment is inside the environment
	if(x0 < 0 || x0 >= x_size ||
		x1 < 0 || x1 >= x_size ||
		y0 < 0 || y0 >= y_size ||
		y1 < 0 || y1 >= y_size)
		return 0;

	ContXY2Cell(x0, y0, &nX0, &nY0, x_size, y_size);
	ContXY2Cell(x1, y1, &nX1, &nY1, x_size, y_size);

	//printf("checking link <%d %d> to <%d %d>\n", nX0,nY0,nX1,nY1);

	//iterate through the points on the segment
	get_bresenham_parameters(nX0, nY0, nX1, nY1, &params);
	do {
		get_current_point(&params, &nX, &nY);
		if(map[GETMAPINDEX(nX,nY,x_size,y_size)] == 1)
			return 0;
	} while (get_next_point(&params));

	return 1;
}

int IsValidArmConfiguration(double* angles, int numofDOFs, double*	map,
			 int x_size, int y_size) {
    double x0,y0,x1,y1;
    int i;
		
	 //iterate through all the links starting with the base
	x1 = ((double)x_size)/2.0;
	y1 = 0;
	for(i = 0; i < numofDOFs; i++){
		//compute the corresponding line segment
		x0 = x1;
		y0 = y1;
		x1 = x0 + LINKLENGTH_CELLS*cos(2*PI-angles[i]);
		y1 = y0 - LINKLENGTH_CELLS*sin(2*PI-angles[i]);

		//check the validity of the corresponding line segment
		if(!IsValidLineSegment(x0,y0,x1,y1,map,x_size,y_size))
			return 0;
	}    
	return 1;
}

//*******************************************************************************************************************//
//                                                                                                                   //
//                                          DEFAULT PLANNER FUNCTION                                                 //
//                                                                                                                   //
//*******************************************************************************************************************//

static void planner(
			double* map,
			int x_size,
			int y_size,
			double* armstart_anglesV_rad,
			double* armgoal_anglesV_rad,
            int numofDOFs,
            double*** plan,
            int* planlength)
{
	//no plan by default
	*plan = NULL;
	*planlength = 0;
		
    //for now just do straight interpolation between start and goal checking for the validity of samples

    double distance = 0;
    int i,j;
    for (j = 0; j < numofDOFs; j++){
        if(distance < fabs(armstart_anglesV_rad[j] - armgoal_anglesV_rad[j]))
            distance = fabs(armstart_anglesV_rad[j] - armgoal_anglesV_rad[j]);
    }
    int numofsamples = (int)(distance/(PI/20));
    if(numofsamples < 2){
        printf("the arm is already at the goal\n");
        return;
    }
    *plan = (double**) malloc(numofsamples*sizeof(double*));
    int firstinvalidconf = 1;
    for (i = 0; i < numofsamples; i++){
        (*plan)[i] = (double*) malloc(numofDOFs*sizeof(double)); 
        for(j = 0; j < numofDOFs; j++){
            (*plan)[i][j] = armstart_anglesV_rad[j] + ((double)(i)/(numofsamples-1))*(armgoal_anglesV_rad[j] - armstart_anglesV_rad[j]);
        }
        if(!IsValidArmConfiguration((*plan)[i], numofDOFs, map, x_size, y_size) && firstinvalidconf) {
            firstinvalidconf = 1;
            printf("ERROR: Invalid arm configuration!!!\n");
        }
    }    
    *planlength = numofsamples;
    
    return;
}

//*******************************************************************************************************************//
//                                                                                                                   //
//                                              MY FUNCTIONS                                                         //
//                                                                                                                   //
//*******************************************************************************************************************//

double halton(int i, int b){
	double f = 1, r = 0;
	while ( i > 0)
	{
		f /= b;
		r += f * (i % b);
		i /= b;
	}
	return r;
}

double distance(double* node1, double* node2, int numOfDOFs){
	double d = 0.0;
	for(int i = 0; i < numOfDOFs; i++){
		double diff = fmod(node2[i] - node1[i] + M_PI, 2 * M_PI) - M_PI;
		d += diff * diff;
	}
	return sqrt(d);
}

void generateRandomSample(int numOfDOFs, double* sample){
	std::random_device rd;
	std::mt19937 gen(rd());
    std::uniform_real_distribution<> dist(0, 2 * M_PI);
	for(int i = 0; i < numOfDOFs; i++){
		if( i != 0) sample[i] = dist(gen);
		else sample[i] = dist(gen)/2;
	}
	return ;
}

bool validEdge(double *startConfig, double *endConfig, int numOfDOFs, int x_size, int y_size, double *map, int numChecks){
    for (int i = 0; i < numChecks; i++) {
        double alpha = halton(i + 1, 2); 
        double interpolatedConfig[numOfDOFs];
        for (int j = 0; j < numOfDOFs; j++) 
            interpolatedConfig[j] = startConfig[j] + alpha * (endConfig[j] - startConfig[j]);
        if (!IsValidArmConfiguration(interpolatedConfig, numOfDOFs, map, x_size, y_size)) 
            return false;
    }
    return true;
}

struct Node
{
	int id;
	vector<int> neighbours;
	Node(int nodeID) : id(nodeID) {}
};

struct StarNode
{
	int id, parent;
	double cost;
	StarNode(int nodeID) : id(nodeID) {}
};

bool edgeExists(Node& node, int neighbour){
	return std::find(node.neighbours.begin(), node.neighbours.end(), neighbour) != node.neighbours.end();
}

vector<int> KNN(const vector<double>& values, int k) {
    vector<int> indices(values.size());
    iota(indices.begin(), indices.end(), 0);
    partial_sort(indices.begin(), indices.begin() + k, indices.end(),
                      [&values](int i, int j) { return values[i] < values[j]; });
    return vector<int>(indices.begin() + 1, indices.begin() + k + 1);
}

std::vector<int> findPath(const std::vector<Node>& nodes, int start, int goal) {
    std::vector<bool> visited(nodes.size(), false);
    std::vector<int> parent(nodes.size(), -1);
    int front = 0, rear = 1;
    std::vector<int> queue;
    queue.push_back(start);
    visited[start] = true;
    while (front < rear) {
        int current = queue[front];
        front++;

        for (int neighbor : nodes[current].neighbours) {
            if (!visited[neighbor]) {
                visited[neighbor] = true;
                parent[neighbor] = current;
                queue.push_back(neighbor);
                rear++;

                if (neighbor == goal) {
                    std::vector<int> path;
                    while (neighbor != -1) {
                        path.push_back(neighbor);
                        neighbor = parent[neighbor];
                    }
                    std::reverse(path.begin(), path.end());
                    return path;
                }
            }
        }
    }
    return std::vector<int>();
}

void displayGraph(const std::vector<Node>& graph) {
    for (const Node& node : graph) {
        std::cout << "Node " << node.id << " : ";
        for (int neighbor : node.neighbours)
            std::cout << neighbor << " ";
        std::cout << std::endl;
    }
}


//*******************************************************************************************************************//
//                                                                                                                   //
//                                              RRT IMPLEMENTATION                                                   //
//                                                                                                                   //
//*******************************************************************************************************************//

static void plannerRRT(
    double *map,
    int x_size,
    int y_size,
    double *armstart_anglesV_rad,
    double *armgoal_anglesV_rad,
    int numofDOFs,
    double ***plan,
    int *planlength)
{
    /* TODO: Replace with your implementation */
	int numofsamples = 3000, numChecks = 200, K = 600;
	double STEP_SIZE = 0.3;
	const int iF = 2;
		const float ALPHA = 1.0 / iF;
	double Samples[numofsamples + 2][numofDOFs];
	vector<Node> nodes;
	int sampleCounter = 0;

	for(int i = 0; i < numofDOFs; i++){
		Samples[0][i] = armstart_anglesV_rad[i];
	}
	nodes.push_back(Node(0)); // Start
	while(sampleCounter < numofsamples){
		double qrand[numofDOFs], qnew[numofDOFs];
		generateRandomSample(numofDOFs, qrand);
		Node qnear = nodes[0];
		double closest_dist = distance(Samples[qnear.id], qrand, numofDOFs);
		for (Node node : nodes){
			double dist = distance(Samples[node.id], qrand, numofDOFs);
			if( dist < closest_dist){
				closest_dist = dist;
				qnear = node;
			}
		}
		for(int i = 0; i < numofDOFs; i++){
			qnew[i] = Samples[qnear.id][i] + STEP_SIZE * (qrand[i] - Samples[qnear.id][i]) / closest_dist;
			qnew[i] = fmod(qnew[i] , 2 * M_PI);
		}
		if (IsValidArmConfiguration(qnew, numofDOFs, map, x_size, y_size)){
			if(validEdge(Samples[qnear.id], qnew, numofDOFs, x_size, y_size, map, numChecks)){
				sampleCounter++;
				for(int j = 0; j < numofDOFs; j++) Samples[sampleCounter][j] = qnew[j];
				nodes.push_back(Node(sampleCounter));
				nodes[sampleCounter].neighbours.push_back(qnear.id);
				nodes[qnear.id].neighbours.push_back(sampleCounter);   
			}
		}
	}

	nodes.push_back(Node(sampleCounter + 1));
	for(int i = 0; i < numofDOFs; i++) Samples[sampleCounter + 1][i] = armgoal_anglesV_rad[i];
	vector<double> distances_goal;
	for(int i = 0; i < numofsamples + 1; i++)
		distances_goal.push_back(distance(Samples[i], Samples[numofsamples + 1], numofDOFs));
	vector<int> indices = KNN(distances_goal, K);
	for(int j = 0; j < K; j++){
		if(validEdge(Samples[numofsamples + 1], Samples[indices[j]], numofDOFs, x_size, y_size, map, numChecks)){
			nodes[numofsamples + 1].neighbours.push_back(indices[j]);
			nodes[indices[j]].neighbours.push_back(numofsamples + 1);
		}
	}
	// displayGraph(nodes);
	vector<int> path = findPath(nodes, 0, numofsamples + 1);
	if (!path.empty()) {
		std::cout << "Path found: ";
		for (int node : path) 
			std::cout << node << " ";
		std::cout << std::endl;

		*planlength = (path.size() - 1) * iF + 1;
		*plan = (double**)malloc(*planlength * sizeof(double*));

		for (int i = 0; i < *planlength; i++) {
			(*plan)[i] = (double*)malloc(numofDOFs * sizeof(double));

			int idx1 = i / iF;
			int idx2 = std::min(idx1 + 1, static_cast<int>(path.size()) - 1);

			for (int j = 0; j < numofDOFs; j++) {
				(*plan)[i][j] = Samples[path[idx1]][j] * (1.0 - ALPHA * (i % iF))
					+ Samples[path[idx2]][j] * ALPHA * (i % iF);
			}
		}
		std::cout << "Done" << std::endl;
	} 
	else std::cout << "No path found." << std::endl; // Modify this to sample more points and repeat
	return;
}


//*******************************************************************************************************************//
//                                                                                                                   //
//                                           RRT STAR IMPLEMENTATION                                                 //
//                                                                                                                   //
//*******************************************************************************************************************//

static void plannerRRTStar(
    double *map,
    int x_size,
    int y_size,
    double *armstart_anglesV_rad,
    double *armgoal_anglesV_rad,
    int numofDOFs,
    double ***plan,
    int *planlength)
{
    /* TODO: Replace with your implementation */
	int numofsamples = 1000, numChecks = 200, K = 200;
	double STEP_SIZE = 0.3, NBRradius = 0.5;
	double Samples[numofsamples + 2][numofDOFs];
	vector<StarNode> nodes;
	int sampleCounter = 0;
	const int iF = 2; 
	const float ALPHA = 1.0 / iF;

	for(int i = 0; i < numofDOFs; i++)
		Samples[0][i] = armstart_anglesV_rad[i];
	nodes.push_back(StarNode(0)); // Start
	nodes[0].cost = 0.0; // Start node cost
	while(sampleCounter < numofsamples){
		double qrand[numofDOFs], qnew[numofDOFs];
		generateRandomSample(numofDOFs, qrand);
		StarNode qnear = nodes[0];
		double closest_dist = distance(Samples[qnear.id], qrand, numofDOFs);
		for (StarNode node : nodes){
			double dist = distance(Samples[node.id], qrand, numofDOFs);
			if( dist < closest_dist){
				closest_dist = dist;
				qnear = node;
			}
		}
		for(int i = 0; i < numofDOFs; i++){
			qnew[i] = Samples[qnear.id][i] + STEP_SIZE * (qrand[i] - Samples[qnear.id][i]) / closest_dist;
			qnew[i] = fmod(qnew[i] , 2 * M_PI);
		}
		if (IsValidArmConfiguration(qnew, numofDOFs, map, x_size, y_size)){
			StarNode newNode(sampleCounter + 1);
			newNode.cost = qnear.cost + STEP_SIZE;
			bool new_has_parent = false;
			vector<int> neighbourhood;
			vector<double> nbr_distances;
			for(StarNode node : nodes){
				double dist = distance(Samples[node.id], qnew, numofDOFs);
				if (dist < NBRradius) {
					neighbourhood.push_back(node.id);
					nbr_distances.push_back(dist);
					if ((dist + node.cost) <= newNode.cost && validEdge(Samples[node.id], qnew, numofDOFs, x_size, y_size, map, numChecks)){
						newNode.parent = node.id;
						newNode.cost = dist + node.cost;
						new_has_parent = true;
					}
				}
			}
			
			if(new_has_parent){
				sampleCounter++;
				for(int j = 0; j < numofDOFs; j++) Samples[sampleCounter][j] = qnew[j];
				nodes.push_back(newNode);
				for(int j = 0; j < neighbourhood.size(); j++){
					if((newNode.cost + nbr_distances[j] < nodes[neighbourhood[j]].cost) && 
									validEdge(Samples[newNode.id], Samples[neighbourhood[j]], numofDOFs, x_size, y_size, map, numChecks)){
						if(neighbourhood[j] != newNode.parent){
							nodes[neighbourhood[j]].cost = newNode.cost + nbr_distances[j];
							nodes[neighbourhood[j]].parent = newNode.id;
						}
					}
				}
			}
		}
	}
	
	bool found_goal = false;
	nodes.push_back(StarNode(sampleCounter + 1));
	for(int i = 0; i < numofDOFs; i++) Samples[sampleCounter + 1][i] = armgoal_anglesV_rad[i];
	vector<double> distances_goal;
	for(int i = 0; i < numofsamples + 1; i++)
		distances_goal.push_back(distance(Samples[i], Samples[numofsamples + 1], numofDOFs));
	vector<int> indices = KNN(distances_goal, K);
	nodes[sampleCounter+1].parent = indices[0];
	nodes[sampleCounter+1].cost = nodes[indices[0]].cost + distances_goal[indices[0]];
	for(int j = 0; j < K; j++){
		if(validEdge(Samples[numofsamples + 1], Samples[indices[j]], numofDOFs, x_size, y_size, map, numChecks)){
			if(distances_goal[indices[j]] + nodes[indices[j]].cost <= nodes[numofsamples + 1].cost){
				nodes[numofsamples + 1].cost = distances_goal[indices[j]] + nodes[indices[j]].cost;
				nodes[numofsamples + 1].parent = indices[j];
			}
			found_goal = true;
		}
	}
	if(found_goal){
		vector<int> path;
		int current = numofsamples + 1;
		while (current != 0)
		{
			path.push_back(current);
			current = nodes[current].parent;
		}
		path.push_back(0);
		std::reverse(path.begin(), path.end());
		std::cout << "Path found: ";
		for (int node : path) 
			std::cout << node << " ";
		std::cout << std::endl;

		*planlength = (path.size() - 1) * iF + 1;
		*plan = (double**)malloc(*planlength * sizeof(double*));

		for (int i = 0; i < *planlength; i++) {
			(*plan)[i] = (double*)malloc(numofDOFs * sizeof(double));

			int idx1 = i / iF;
			int idx2 = std::min(idx1 + 1, static_cast<int>(path.size()) - 1);

			for (int j = 0; j < numofDOFs; j++) {
				(*plan)[i][j] = Samples[path[idx1]][j] * (1.0 - ALPHA * (i % iF))
					+ Samples[path[idx2]][j] * ALPHA * (i % iF);
			}
		}
		std::cout << "Done" << std::endl;
	}
	else std::cout << "No path found." << std::endl; // Modify this to sample more points and repeat
	return;
}

//*******************************************************************************************************************//
//                                                                                                                   //
//                                              PRM IMPLEMENTATION                                                   //
//                                                                                                                   //
//*******************************************************************************************************************//

static void plannerPRM(
    double *map,
    int x_size,
    int y_size,
    double *armstart_anglesV_rad,
    double *armgoal_anglesV_rad,
    int numofDOFs,
    double ***plan,
    int *planlength)
{
    /* TODO: Replace with your implementation */
	int numofsamples = 2000, K = 5, numChecks = 200;
	double Samples[numofsamples + 2][numofDOFs];
	vector<Node> nodes;
	int sampleCounter = 0;

	/*Creating Nodes*/
	for(int i = 0; i < numofDOFs; i++){
		Samples[0][i] = armstart_anglesV_rad[i];
		Samples[1][i] = armgoal_anglesV_rad[i];
	}
	nodes.push_back(Node(0)); // Start
	nodes.push_back(Node(1)); // Goal
	while(sampleCounter < numofsamples){
		double sample[numofDOFs];
		generateRandomSample(numofDOFs, sample);
		if(IsValidArmConfiguration(sample, numofDOFs, map, x_size, y_size)){
			for(int i = 0; i < numofDOFs; i++) 
				Samples[sampleCounter + 2][i] = sample[i];
			nodes.push_back(Node(sampleCounter + 2));
			sampleCounter++;
		}
	}

	/*Forming neighbourhood*/
	for(int i = 0; i < numofsamples + 2; i++){
		vector<double> distances;
		for(int j = 0; j < numofsamples + 2; j++)
			distances.push_back(distance(Samples[i], Samples[j], numofDOFs));
		vector<int> indices = KNN(distances, K);
		for(int j = 0; j < K; j++)
			if(!edgeExists(nodes[i], j) && i!=j)
				if(validEdge(Samples[i], Samples[j], numofDOFs, x_size, y_size, map, numChecks)){
					nodes[i].neighbours.push_back(j);
					nodes[j].neighbours.push_back(i);
				}
	}

	vector<int> path = findPath(nodes, 0, 1);
	if (!path.empty()) {
		std::cout << "Path found: ";
		for (int node : path) 
			std::cout << node << " ";
		std::cout << std::endl;

		const int iF = 20; 
		const float ALPHA = 1.0 / iF;
		*planlength = (path.size() - 1) * iF + 1;
		*plan = (double**)malloc(*planlength * sizeof(double*));

		for (int i = 0; i < *planlength; i++) {
			(*plan)[i] = (double*)malloc(numofDOFs * sizeof(double));

			int idx1 = i / iF;
			int idx2 = std::min(idx1 + 1, static_cast<int>(path.size()) - 1);

			for (int j = 0; j < numofDOFs; j++) {
				(*plan)[i][j] = Samples[path[idx1]][j] * (1.0 - ALPHA * (i % iF))
					+ Samples[path[idx2]][j] * ALPHA * (i % iF);
			}
		}
		std::cout << "Done" << std::endl;
	} 
	else std::cout << "No path found." << std::endl; // Modify this to sample more points and repeat
	return;
}

//*******************************************************************************************************************//
//                                                                                                                   //
//                                                MAIN FUNCTION                                                      //
//                                                                                                                   //
//*******************************************************************************************************************//

/** Your final solution will be graded by an grading script which will
 * send the default 6 arguments:
 *    map, numOfDOFs, commaSeparatedStartPos, commaSeparatedGoalPos, 
 *    whichPlanner, outputFilePath
 * An example run after compiling and getting the planner.out executable
 * >> ./planner.out map1.txt 5 1.57,0.78,1.57,0.78,1.57 0.392,2.35,3.14,2.82,4.71 0 output.txt
 * See the hw handout for full information.
 * If you modify this for testing (e.g. to try out different hyper-parameters),
 * make sure it can run with the original 6 commands.
 * Programs that do not will automatically get a 0.
 * */
int main(int argc, char** argv) {
	double* map;
	int x_size, y_size;

	tie(map, x_size, y_size) = loadMap(argv[1]);
	const int numOfDOFs = std::stoi(argv[2]);
	double* startPos = doubleArrayFromString(argv[3]);
	double* goalPos = doubleArrayFromString(argv[4]);
	int whichPlanner = std::stoi(argv[5]);
	string outputFile = argv[6];

	if(!IsValidArmConfiguration(startPos, numOfDOFs, map, x_size, y_size)||
			!IsValidArmConfiguration(goalPos, numOfDOFs, map, x_size, y_size)) {
		throw runtime_error("Invalid start or goal configuration!\n");
	}

	///////////////////////////////////////
	//// Feel free to modify anything below. Be careful modifying anything above.

	double** plan = NULL;
	int planlength = 0;

    // Call the corresponding planner function
    if (whichPlanner == PRM)
    {
        plannerPRM(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);
    }
    else if (whichPlanner == RRT)
    {
        plannerRRT(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);
    }
    else if (whichPlanner == RRTSTAR)
    {
        plannerRRTStar(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);
    }
    else
    {
        planner(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);
    }

	//// Feel free to modify anything above.
	//// If you modify something below, please change it back afterwards as my 
	//// grading script will not work and you will recieve a 0.
	///////////////////////////////////////

    // Your solution's path should start with startPos and end with goalPos
    if (!equalDoubleArrays(plan[0], startPos, numOfDOFs) || 
    	!equalDoubleArrays(plan[planlength-1], goalPos, numOfDOFs)) {
		throw std::runtime_error("Start or goal position not matching");
	}

	/** Saves the solution to output file
	 * Do not modify the output log file output format as it is required for visualization
	 * and for grading.
	 */
	std::ofstream m_log_fstream;
	m_log_fstream.open(outputFile, std::ios::trunc); // Creates new or replaces existing file
	if (!m_log_fstream.is_open()) {
		throw std::runtime_error("Cannot open file");
	}
	m_log_fstream << argv[1] << endl; // Write out map name first
	/// Then write out all the joint angles in the plan sequentially
	for (int i = 0; i < planlength; ++i) {
		for (int k = 0; k < numOfDOFs; ++k) {
			m_log_fstream << plan[i][k] << ",";
		}
		m_log_fstream << endl;
	}
}
