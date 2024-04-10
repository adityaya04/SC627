/*=================================================================
 *
 * planner.cpp
 *
 *=================================================================*/
#include "planner.h"
#include <math.h>
#include <iostream>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <stack>
#include <memory>
#include <chrono>
using namespace std;

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y-1)*XSIZE + (X-1))

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define NUMOFDIRS 8
#define inf INT32_MAX
int DX[9] = {-1, -1, -1,  0,  0,  1, 1, 1, 0};
int DY[9] = {-1,  0,  1, -1,  1, -1, 0, 1, 0};

chrono::time_point<chrono::steady_clock> T_starting;
unordered_map<int, int> Target_nodes;
unordered_map<int, pair<int, int>> Heuristic_2d;
stack<int> final_path;

int robot_prev_X, robot_prev_Y;
bool initialzed = true;
bool debug_bool = true;

struct Node
{
    shared_ptr<Node> parent;
    int _index;
    int _time;
    int _h;
    int _g;
    int _f;

    Node() : parent(nullptr), _g(inf), _f(inf) {}
    Node(int index, int time, int h) : parent(nullptr), _g(inf), _f(inf)
    {
        _index = index;
        _time = time;
        _h = h;
    }
};

auto GREATER = [](shared_ptr<Node> n1, shared_ptr<Node> n2)
{
    return (n1->_f > n2->_f);
};


priority_queue<shared_ptr<Node>, vector<shared_ptr<Node>>, decltype(GREATER)> OPEN(GREATER);
unordered_set<int> CLOSED;
unordered_map<int, shared_ptr<Node>> Nodes;

pair<int, int> getXY(int x_size, int index)
{
    pair<int, int> XY;
    XY.first = (int)(index % x_size) + 1;
    XY.second = (int)(index / x_size) + 1;
    return XY;
}

void Dijkstra(
        int* map,
        int collision_thresh,
        int x_size,
        int y_size
        )
{
    while(!OPEN.empty())
    {
        shared_ptr<Node> active_node = OPEN.top();
        OPEN.pop();
        if(Heuristic_2d.find(active_node ->_index) != Heuristic_2d.end()) continue; 
        Heuristic_2d[active_node ->_index] = pair<int, int>(active_node ->_g, active_node ->_time); 

        pair<int, int> gridXY = getXY(x_size, active_node ->_index);
        int grid_x = gridXY.first;
        int grid_y = gridXY.second;
        
        for(int i = 0; i < NUMOFDIRS; i++){
            int x_temp = grid_x + DX[i];
            int y_temp = grid_y + DY[i];
            int temp_index = (int) GETMAPINDEX(x_temp, y_temp, x_size, y_size);
            if(Heuristic_2d.find(temp_index) == Heuristic_2d.end()){
                if(x_temp >= 1 && x_temp <= x_size &&
                   y_temp >= 1 && y_temp <= y_size){
                    int cost = (int) map[temp_index];
                    if((cost >= 0) && (cost < collision_thresh)) {
                        if(Nodes.find(temp_index) == Nodes.end()){
                            shared_ptr<Node> new_node = make_shared<Node>(temp_index, active_node ->_time, 0);
                            Nodes[temp_index] = new_node;
                        }
                        if(Nodes[temp_index]->_g > active_node ->_g + cost){
                            Nodes[temp_index]->_g = active_node ->_g + cost;
                            Nodes[temp_index]->_f = Nodes[temp_index]->_g + Nodes[temp_index]->_h;
                            Nodes[temp_index]->parent = active_node;
                            OPEN.push(Nodes[temp_index]);
                        }
                    }
               }
            }
        }
    }
}

void A_star(
        int* map,
        int* target_traj,
        int target_steps,
        int x_size,
        int y_size,
        int collision_thresh
        )
{
    while(!OPEN.empty()){
        int timeElapsed = chrono::duration_cast<chrono::seconds>(chrono::steady_clock::now() - T_starting).count();
        int len_time;
        shared_ptr<Node> active_node = OPEN.top();
        OPEN.pop();
        if(active_node ->_time == 0) len_time = 0;
        else len_time = (int) (log10(active_node ->_time) + 1);
        int hash_key = active_node ->_index * ((int)pow(10, len_time)) + active_node ->_time; 
        if(CLOSED.find(hash_key) != CLOSED.end()) continue; 
        CLOSED.insert(hash_key);

        pair<int, int> gridXY = getXY(x_size, active_node ->_index);
        int grid_x = gridXY.first;
        int grid_y = gridXY.second;

        if(Target_nodes.find(active_node ->_index) != Target_nodes.end() and
           active_node ->_time == (Target_nodes[active_node ->_index] - timeElapsed)){
            while(active_node){
                final_path.push(active_node ->_index);
                active_node = active_node ->parent;
            }
            final_path.pop();
            return;
        }
        int time = active_node ->_time + 1;
        if(time > target_steps) continue;
        for(int i = 0; i < (NUMOFDIRS + 1); i++){
            int x_temp = grid_x + DX[i];
            int y_temp = grid_y + DY[i];
            int temp_index = (int) GETMAPINDEX(x_temp, y_temp, x_size, y_size);
            if(time == 0) len_time = 0;
            else len_time = (int) (log10(time) + 1);
            hash_key = temp_index * ((int)pow(10, len_time)) + time; 

            if(CLOSED.find(hash_key) == CLOSED.end()){
                if(x_temp >= 1 and y_temp >= 1 and
                   x_temp <= x_size and y_temp <= y_size){
                    int cost = (int) map[temp_index];
                    if((cost >= 0) and (cost < collision_thresh)){
                        if(Nodes.find(hash_key) == Nodes.end()){
                            int totalTime = timeElapsed + time;
                            int H = 0;
                            if (Target_nodes.find(temp_index) != Target_nodes.end() and totalTime <= Target_nodes[temp_index])
                                H = cost*(Target_nodes[temp_index] - totalTime);
                            else H =  Heuristic_2d[temp_index].first + abs(Heuristic_2d[temp_index].second - totalTime);
                            shared_ptr<Node> new_node = make_shared<Node>(temp_index, time, H);
                            Nodes[hash_key] = new_node;
                        }
                        if(Nodes[hash_key]->_g > active_node ->_g + cost){ 
                            Nodes[hash_key]->_g = active_node ->_g + cost;
                            Nodes[hash_key]->_f = Nodes[hash_key]->_g + 1.8*Nodes[hash_key]->_h; 
                            Nodes[hash_key]->parent = active_node;
                            OPEN.push(Nodes[hash_key]);
                        }
                    }
                }
            }
        }
    }
}

void planner(
    int* map,
    int collision_thresh,
    int x_size,
    int y_size,
    int robotposeX,
    int robotposeY,
    int target_steps,
    int* target_traj,
    int targetposeX,
    int targetposeY,
    int curr_time,
    int* action_ptr
    )   
{
    int goalposeX = (int) target_traj[target_steps - 1];
    int goalposeY = (int) target_traj[target_steps - 1 + target_steps];
    robot_prev_X = robotposeX; 
    robot_prev_Y = robotposeY;
    if(initialzed){
        T_starting = chrono::steady_clock::now();
        initialzed = false;
        int goal_idx;
        for(int i = 0; i < target_steps; i++){
            goal_idx = GETMAPINDEX((int) target_traj[i], (int) target_traj[target_steps + i], x_size, y_size);
            Target_nodes[goal_idx] = i;
            if(i > (target_steps/2)){
                shared_ptr<Node> temp = make_shared<Node>(goal_idx, i, 0);
                temp ->_g = 0;
                temp ->_f = temp ->_h;
                OPEN.push(temp);
                Nodes[goal_idx] = temp;
            }
        }
        Dijkstra(map, collision_thresh, x_size, y_size);
        Nodes.clear();
        int index = GETMAPINDEX(robotposeX, robotposeY, x_size, y_size);
        int H = Heuristic_2d[index].first;
        shared_ptr<Node> START = make_shared<Node>(index, 0, H);
        START ->_g = 0;
        START ->_f = START ->_h;
        OPEN.push(START);
        Nodes[index] = START;
        A_star(map, target_traj, target_steps, x_size, y_size, collision_thresh);
    }
    if(!final_path.empty()){
        int next_step = final_path.top();
        final_path.pop();
        pair<int, int> prevXY = getXY(x_size, next_step);
        robot_prev_X = prevXY.first;
        robot_prev_Y = prevXY.second;
    }

    action_ptr[0] = robot_prev_X;
    action_ptr[1] = robot_prev_Y;
    return;
}