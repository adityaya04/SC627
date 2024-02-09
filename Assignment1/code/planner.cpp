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

struct Node
{
	int id;
	vector<int> neighbours;
	vector<double> angles;
	Node(int nodeID) : id(nodeID) {}
};

struct StarNode
{
	int id, parent;
	double cost;
	vector<double> angles;
	StarNode(int nodeID) : id(nodeID) {}
};

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

double distance(StarNode node1, StarNode node2, int numOfDOFs){
	double d = 0.0;
	for(int i = 0; i < numOfDOFs; i++){
		double diff = fmod(node2.angles[i] - node1.angles[i] + M_PI, 2 * M_PI) - M_PI;
		d += diff * diff;
	}
	return sqrt(d); 
}

double distance(Node node1, Node node2, int numOfDOFs){
	double d = 0.0;
	for(int i = 0; i < numOfDOFs; i++){
		double diff = fmod(node2.angles[i] - node1.angles[i] + M_PI, 2 * M_PI) - M_PI;
		d += diff * diff;
	}
	return sqrt(d); 
}

Node generateRandomSample(int numOfDOFs, int dummy){
	Node node(-1);
	std::random_device rd;
	std::mt19937 gen(rd());
    std::uniform_real_distribution<> dist(0, 2 * M_PI);
	for(int i = 0; i < numOfDOFs; i++){
		if( i != 0) node.angles.push_back(dist(gen));
		else node.angles.push_back(dist(gen) / 2.0);
	}
	return node; 
}

StarNode generateRandomSample(int numOfDOFs){
	StarNode node(-1);
	std::random_device rd;
	std::mt19937 gen(rd());
    std::uniform_real_distribution<> dist(0, 2 * M_PI);
	for(int i = 0; i < numOfDOFs; i++){
		if( i != 0) node.angles.push_back(dist(gen));
		else node.angles.push_back(dist(gen) / 2.0);
	}
	return node; 
}

bool validEdge(StarNode node1, StarNode node2, int numOfDOFs, int x_size, int y_size, double *map, int numChecks){
    for (int i = 0; i < numChecks; i++) {
        double alpha = halton(i + 1, 2); 
        double interpolatedConfig[numOfDOFs];
        for (int j = 0; j < numOfDOFs; j++) 
            interpolatedConfig[j] = node1.angles[j] + alpha * (node2.angles[j] - node1.angles[j]);
        if (!IsValidArmConfiguration(interpolatedConfig, numOfDOFs, map, x_size, y_size)) 
            return false;
    }
    return true; 
}

bool validEdge(Node node1, Node node2, int numOfDOFs, int x_size, int y_size, double *map, int numChecks){
    for (int i = 0; i < numChecks; i++) {
        double alpha = halton(i + 1, 2); 
        double interpolatedConfig[numOfDOFs];
        for (int j = 0; j < numOfDOFs; j++) 
            interpolatedConfig[j] = node1.angles[j] + alpha * (node2.angles[j] - node1.angles[j]);
        if (!IsValidArmConfiguration(interpolatedConfig, numOfDOFs, map, x_size, y_size)) 
            return false;
    }
    return true; 
}

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

vector<int> findPath(const vector<Node>& nodes, int start, int goal) {
    vector<bool> visited(nodes.size(), false);
    vector<int> parent(nodes.size(), -1);
    int front = 0, rear = 1;
    vector<int> queue;
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
                    vector<int> path;
                    while (neighbor != -1) {
                        path.push_back(neighbor);
                        neighbor = parent[neighbor];
                    }
                    reverse(path.begin(), path.end());
                    return path;
                }
            }
        }
    }
    return vector<int>();
}

//*******************************************************************************************************************//
//                                                                                                                   //
//                                              RRT IMPLEMENTATION                                                   //
//                                                                                                                   //
//*******************************************************************************************************************//

vector<Node> createRRT(vector<Node> nodes, int numofsamples, double *map, int x_size, int y_size, int numofDOFs){
	int sampleCounter = nodes.size();
	double STEP_SIZE = 0.7;
	int numChecks = 50;

	while(sampleCounter < numofsamples){
		Node qrand = generateRandomSample(numofDOFs, 0);
		Node qnear = nodes[0];
		Node qnew(sampleCounter);
		double closest_dist = distance(qnear, qrand, numofDOFs);
		for (Node node : nodes){
			double dist = distance(node, qrand, numofDOFs);
			if( dist < closest_dist){
				closest_dist = dist;
				qnear = node;
			}
		}
		for(int i = 0; i < numofDOFs; i++){
			qnew.angles.push_back(qnear.angles[i] + STEP_SIZE * (qrand.angles[i] - qnear.angles[i]) / closest_dist);
			qnew.angles[i] = fmod(qnew.angles[i] , 2 * M_PI);
		}
		if(validEdge(qnear, qnew, numofDOFs, x_size, y_size, map, numChecks)){
			qnew.neighbours.push_back(qnear.id);
			nodes[qnear.id].neighbours.push_back(sampleCounter);   
			nodes.push_back(qnew);
			sampleCounter++;
		}
		
	}
	return nodes;
}

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
	int numChecks = 50, K = 400;
	double STEP_SIZE = 0.7;
	const int iF = 1;
		const float ALPHA = 1.0 / iF;
	vector<Node> nodes;

	nodes.push_back(Node(0)); // Start
	for(int i = 0; i < numofDOFs; i++){
		nodes[0].angles.push_back(armstart_anglesV_rad[i]);
	}
	
	while(1){
		int start_size = nodes.size();
		if (start_size == 1) nodes = createRRT(nodes, 1000, map, x_size, y_size, numofDOFs);
		else{
			nodes = createRRT(nodes, start_size + 200, map, x_size, y_size, numofDOFs);
			// cout << "Taking more nodes " << start_size << endl;	
		} 

		vector<Node> graph = nodes;
		int goal = graph.size();
		Node Goal(goal);
		for(int i = 0; i < numofDOFs; i++) Goal.angles.push_back(armgoal_anglesV_rad[i]);
		vector<double> distances_goal;
		for(int i = 0; i < goal; i++)
			distances_goal.push_back(distance(graph[i], Goal, numofDOFs));
		vector<int> indices = KNN(distances_goal, K);
		bool found_goal = false;
		for(int j = 0; j < K; j++){
			if(validEdge(Goal, graph[indices[j]], numofDOFs, x_size, y_size, map, numChecks)){
				Goal.neighbours.push_back(indices[j]);
				graph[indices[j]].neighbours.push_back(goal);
				found_goal = true;
			}
		}
		if(found_goal){
			graph.push_back(Goal);
			vector<int> path = findPath(graph, 0, goal);
			if (!path.empty()) {
				// cout << "Path found: ";
				// for (int node : path) 
				// 	cout << node << " ";
				// cout << endl;
				double cost = 0.0;
				for(int i = 0; i < path.size() - 1; i++)
					cost += distance(graph[path[i]], graph[path[i+1]], numofDOFs);
				// cout << "Cost of RRT : " << cost << endl;
				// cout << "Nodes : " << graph.size() << endl;

				*planlength = (path.size() - 1) * iF + 1;
				*plan = (double**)malloc(*planlength * sizeof(double*));

				for (int i = 0; i < *planlength; i++) {
					(*plan)[i] = (double*)malloc(numofDOFs * sizeof(double));

					int idx1 = i / iF;
					int idx2 = std::min(idx1 + 1, static_cast<int>(path.size()) - 1);

					for (int j = 0; j < numofDOFs; j++) {
						(*plan)[i][j] = graph[path[idx1]].angles[j] * (1.0 - ALPHA * (i % iF))
							+ graph[path[idx2]].angles[j] * ALPHA * (i % iF);
					}
				}
				return;
			}
		} 
	}
	return;
}


//*******************************************************************************************************************//
//                                                                                                                   //
//                                           RRT STAR IMPLEMENTATION                                                 //
//                                                                                                                   //
//*******************************************************************************************************************//

vector<StarNode> createRRTstar(vector<StarNode> nodes, int numofsamples, double *map, int x_size, int y_size, int numofDOFs){
	int sampleCounter = nodes.size();
	int numChecks = 50;
	double STEP_SIZE = 0.7, NBRradius = 0.9;
	while(sampleCounter < numofsamples){
		StarNode qrand = generateRandomSample(numofDOFs);
		StarNode qnear = nodes[0];
		vector<int> neighbourhood;
		vector<double> nbr_distances;
		StarNode qnew(sampleCounter);
		double closest_dist = distance(qnear, qrand, numofDOFs);
		for (StarNode node : nodes){
			double dist = distance(node, qrand, numofDOFs);
			if( dist < closest_dist){
				closest_dist = dist;
				qnear = node;
			}
		}

		for(int i = 0; i < numofDOFs; i++){
			qnew.angles.push_back(qnear.angles[i] + STEP_SIZE * (qrand.angles[i] - qnear.angles[i]) / closest_dist); 
			qnew.angles[i] = fmod(qnew.angles[i] , 2 * M_PI);
		}

		
		for(StarNode node : nodes){
			double dist = distance(qnew, node, numofDOFs); 
			if(dist < NBRradius){
				neighbourhood.push_back(node.id);
				nbr_distances.push_back(dist);
			}
		}
		qnew.cost = -1;
		for(int i = 0; i < neighbourhood.size(); i++){
			if(validEdge(qnew, nodes[neighbourhood[i]], numofDOFs, x_size, y_size, map, numChecks))
			{
				if(qnew.cost < 0){
					qnew.cost = nodes[neighbourhood[i]].cost + nbr_distances[i];
					qnew.parent = neighbourhood[i];
				}
				else {
					if(nodes[neighbourhood[i]].cost + nbr_distances[i] < qnew.cost){
						qnew.cost = nodes[neighbourhood[i]].cost + nbr_distances[i];
						qnew.parent = neighbourhood[i];
					}	
				}
			}
		}
		if(qnew.cost > 0){
			nodes.push_back(qnew);
			sampleCounter++;

			for(int i = 0; i < neighbourhood.size(); i++){
				if(neighbourhood[i] != qnew.parent){
					if(validEdge(qnew, nodes[neighbourhood[i]], numofDOFs, x_size, y_size, map, numChecks))
					{
						if(qnew.cost + nbr_distances[i] < nodes[neighbourhood[i]].cost){
							nodes[neighbourhood[i]].parent = qnew.id;
							nodes[neighbourhood[i]].cost = qnew.cost + nbr_distances[i];
						}
					}
				}
			}
		}
	}
	return nodes;
}

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
	int numChecks = 50, K = 400;
	double STEP_SIZE = 0.7, NBRradius = 0.9;
	vector<StarNode> nodes;
	int sampleCounter = 0;
	const int iF = 1; 
	const float ALPHA = 1.0 / iF;

	nodes.push_back(StarNode(0)); // Start
	nodes[0].cost = 0.0; // Start node cost
	for(int i = 0; i < numofDOFs; i++)
		nodes[0].angles.push_back(armstart_anglesV_rad[i]);

	while(1){
		int start_size = nodes.size();
		if(start_size == 1) nodes = createRRTstar(nodes, 1000, map, x_size, y_size, numofDOFs);
		else nodes = createRRTstar(nodes, start_size + 200, map, x_size, y_size, numofDOFs);

		vector<StarNode> graph = nodes;
		int goal = graph.size();
		StarNode Goal(goal);
		Goal.cost = -1;
		for(int i = 0; i < numofDOFs; i++) Goal.angles.push_back(armgoal_anglesV_rad[i]);
		vector<double> distances_goal;
		for(int i = 0; i < goal; i++)
			distances_goal.push_back(distance(graph[i], Goal, numofDOFs));
		vector<int> indices = KNN(distances_goal, K);
		for(int j = 0; j < K; j++){
			if(validEdge(Goal, graph[indices[j]], numofDOFs, x_size, y_size, map, numChecks)){
				if(Goal.cost < 0) {
					Goal.parent = indices[j];
					Goal.cost = distances_goal[indices[j]] + graph[indices[j]].cost; 
				}
				else if (distances_goal[indices[j]] + graph[indices[j]].cost < Goal.cost){
					Goal.parent = indices[j];
					Goal.cost = distances_goal[indices[j]] + graph[indices[j]].cost; 
				}
			}
		}
		if(Goal.cost > 0){
			graph.push_back(Goal);
			vector<int> path;
			int current = goal;
			while (current != 0)
			{
				path.push_back(current);
				current = graph[current].parent;
			}
			path.push_back(0);
			reverse(path.begin(), path.end());
			// cout << "Path found: ";
			// for (int node : path) 
			// 	cout << node << " ";
			// cout << endl;
			double cost = 0.0;
			for(int i = 0; i < path.size() - 1; i++)
				cost += distance(graph[path[i]], graph[path[i+1]], numofDOFs);
			// cout << "Cost of RRT* : " << cost << endl;
			// cout << "Nodes : " << graph.size() << endl;

			*planlength = (path.size() - 1) * iF + 1;
			*plan = (double**)malloc(*planlength * sizeof(double*));

			for (int i = 0; i < *planlength; i++) {
				(*plan)[i] = (double*)malloc(numofDOFs * sizeof(double));

				int idx1 = i / iF;
				int idx2 = std::min(idx1 + 1, static_cast<int>(path.size()) - 1);

				for (int j = 0; j < numofDOFs; j++) {
					(*plan)[i][j] = graph[path[idx1]].angles[j] * (1.0 - ALPHA * (i % iF))
						+ graph[path[idx2]].angles[j] * ALPHA * (i % iF);
				}
			}
			return;
		}
	}
	
	return;
}

//*******************************************************************************************************************//
//                                                                                                                   //
//                                              PRM IMPLEMENTATION                                                   //
//                                                                                                                   //
//*******************************************************************************************************************//
vector<Node> createPRM(vector<Node> nodes, int numofsamples, double *map, int x_size, int y_size, int numofDOFs){
	int sampleCounter = nodes.size();
	int K = 50, numChecks = 200;

	while(sampleCounter < numofsamples){
		Node qnew = generateRandomSample(numofDOFs, 0);
		double sample[numofDOFs];
		for(int i = 0; i < numofDOFs; i++) sample[i] = qnew.angles[i];
		if(IsValidArmConfiguration(sample, numofDOFs, map, x_size, y_size)){
			qnew.id = sampleCounter++;
			nodes.push_back(qnew);
		}
	}

	for(int i = 0; i < nodes.size(); i++){
		vector<double> distances;
		for(int j = 0; j < nodes.size(); j++)
			if (i != j)
				distances.push_back(distance(nodes[i], nodes[j], numofDOFs));
		vector<int> indices = KNN(distances, K);
		for(int j = 0; j < K; j++)
			if(!edgeExists(nodes[i], j) && i!=j)
				if(validEdge(nodes[i], nodes[j], numofDOFs, x_size, y_size, map, numChecks)){
					nodes[i].neighbours.push_back(j);
					nodes[j].neighbours.push_back(i);
				}
	}
	return nodes;
}

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
	int K = 50, numChecks = 200;
	vector<Node> nodes;
	Node Start(-1), Goal(-1);
	for(int i = 0; i < numofDOFs; i++){
		Start.angles.push_back(armstart_anglesV_rad[i]);
		Goal.angles.push_back(armgoal_anglesV_rad[i]);
	}
	
	while (1){
		int start_size = nodes.size();
		if(start_size == 0) nodes = createPRM(nodes, 1000, map, x_size, y_size, numofDOFs);
		else{
			// cout << "Taking more nodes " << start_size << endl;
			nodes = createPRM(nodes, start_size + 300, map, x_size, y_size, numofDOFs);
		} 
		vector<Node> graph = nodes;
		int start = graph.size();
		int goal = start + 1;
		Goal.id = goal;
		Start.id = start;
		graph.push_back(Start);
		graph.push_back(Goal);

		vector<double> distances_goal, distances_start;
		for(int i = 0; i < start; i++){
			distances_goal.push_back(distance(Goal, graph[i], numofDOFs));
			distances_start.push_back(distance(Start, graph[i], numofDOFs));
		}
		vector<int> goal_indices = KNN(distances_goal, K);
		vector<int> start_indices = KNN(distances_start, K);


		for(int j = 0; j < K; j++){
			if(validEdge(Goal, graph[goal_indices[j]], numofDOFs, x_size, y_size, map, numChecks)){
				graph[goal].neighbours.push_back(goal_indices[j]);
				graph[goal_indices[j]].neighbours.push_back(goal);	
			}
			if(validEdge(Start, graph[start_indices[j]], numofDOFs, x_size, y_size, map, numChecks)){
				graph[start].neighbours.push_back(start_indices[j]);
				graph[start_indices[j]].neighbours.push_back(start);	
			}
		}
		vector<int> path = findPath(graph, start, goal);
		if (!path.empty()) {
			// cout << "Path found: ";
			// for (int node : path) 
			// 	cout << node << " ";
			// cout << endl;
			double cost = 0.0;
			for(int i = 0; i < path.size() - 1; i++)
				cost += distance(graph[path[i]], graph[path[i+1]], numofDOFs);
			// cout << "Cost of PRM : " << cost << endl;
			// cout << "Nodes : " << goal << endl;

			const int iF = 2; 
			const float ALPHA = 1.0 / iF;
			*planlength = (path.size() - 1) * iF + 1;
			*plan = (double**)malloc(*planlength * sizeof(double*));

			for (int i = 0; i < *planlength; i++) {
				(*plan)[i] = (double*)malloc(numofDOFs * sizeof(double));

				int idx1 = i / iF;
				int idx2 = std::min(idx1 + 1, static_cast<int>(path.size()) - 1);

				for (int j = 0; j < numofDOFs; j++) {
					(*plan)[i][j] = graph[path[idx1]].angles[j] * (1.0 - ALPHA * (i % iF))
						+ graph[path[idx2]].angles[j] * ALPHA * (i % iF);
				}
			}
			return;
		} 
	}

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