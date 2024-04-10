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
#include <time.h>
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
int dX[NUMOFDIRS + 1] = {-1, -1, -1,  0,  0,  1, 1, 1, 0};
int dY[NUMOFDIRS + 1] = {-1,  0,  1, -1,  1, -1, 0, 1, 0};

chrono::time_point<chrono::steady_clock> T_starting;

int robot_prev_X, robot_prev_Y;
bool initialzed = true;
bool debug_bool = true;

struct node
{
    int _index;
    int _g;
    int _h;
    int _f;
    int _time;
    shared_ptr<node> parent;

    node() : parent(nullptr), _g(inf), _f(inf) {}
    node(int index, int time, int h) : parent(nullptr), _g(inf), _f(inf)
    {
        _index = index;
        _time = time;
        _h = h;
    }
};

auto GREATER = [](shared_ptr<node> n1, shared_ptr<node> n2)
{
    return (n1->_f > n2->_f);
};

unordered_map<int, int> Target_nodes;
unordered_map<int, pair<int, int>> heuristics;
priority_queue<shared_ptr<node>, vector<shared_ptr<node>>, decltype(GREATER)> OPEN(GREATER);
unordered_set<int> CLOSED;
unordered_map<int, shared_ptr<node>> nodes;
stack<int> final_path;

// pair<int, int> getXY(int x_size, shared_ptr<node> Node)
// {
//     pair<int, int> XY;
//     XY.first = (int)(Node -> _index % x_size) + 1;
//     XY.second = (int)(Node -> _index / x_size) + 1;
//     return XY;
// }

void Dijkstra(
        int* map,
        int collision_thresh,
        int x_size,
        int y_size
        )
{
    while(!OPEN.empty())
    {
        shared_ptr<node> s = OPEN.top();
        OPEN.pop();
        if(heuristics.find(s->_index) != heuristics.end()) continue; 
        heuristics[s->_index] = pair<int, int>(s->_g, s->_time); 

        // pair<int, int> gridXY = getXY(x_size, s);
        // int grid_x = gridXY.first;
        // int grid_y = gridXY.second;
        int grid_x = (int)(s->_index % x_size) + 1;
        int grid_y = (int)(s->_index / x_size) + 1;


        for(int i = 0; i < NUMOFDIRS; i++){
            int x_temp = grid_x + dX[i];
            int y_temp = grid_y + dY[i];
            int temp_index = (int) GETMAPINDEX(x_temp, y_temp, x_size, y_size);

            if(x_temp >= 1 and x_temp <= x_size and
               y_temp >= 1 and y_temp <= y_size and 
               heuristics.find(temp_index) == heuristics.end()){
                int cost = (int) map[temp_index];
                if((cost >= 0) and (cost < collision_thresh)) {
                    if(nodes.find(temp_index) == nodes.end()){
                        shared_ptr<node> n = make_shared<node>(temp_index, s->_time, 0);
                        nodes[temp_index] = n;
                    }
                    if(nodes[temp_index]->_g > s->_g + cost){
                        nodes[temp_index]->_g = s->_g + cost;
                        nodes[temp_index]->_f = nodes[temp_index]->_g + nodes[temp_index]->_h;
                        nodes[temp_index]->parent = s;
                        OPEN.push(nodes[temp_index]);
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
        shared_ptr<node> s = OPEN.top();
        OPEN.pop();
        if(time == 0) len_time = 0;
        else len_time = (int) (log10(s->_time) + 1);
        int hash_key = s->_index * ((int)pow(10, len_time)) + s->_time; 
        if(CLOSED.find(hash_key) != CLOSED.end()) continue; 
        CLOSED.insert(hash_key);

        // pair<int, int> gridXY = getXY(x_size, s);
        // int grid_x = gridXY.first;
        // int grid_y = gridXY.second;
        int grid_x = (int)(s->_index % x_size) + 1;
        int grid_y = (int)(s->_index / x_size) + 1;

        if(Target_nodes.find(s->_index) != Target_nodes.end() and s->_time == (Target_nodes[s->_index] - timeElapsed)){
            while(s){
                final_path.push(s->_index);
                s = s->parent;
            }
            final_path.pop();
            return;
        }
         
        int time = s->_time + 1;
        if(time > target_steps) continue;
        for(int i = 0; i < (NUMOFDIRS + 1); i++){
            int x_temp = grid_x + dX[i];
            int y_temp = grid_y + dY[i];
            int temp_index = (int) GETMAPINDEX(x_temp, y_temp, x_size, y_size);
            if(time == 0) len_time = 0;
            else len_time = (int) (log10(s->_time) + 1);
            hash_key = temp_index * ((int)pow(10, len_time)) + time; 
            
            if(x_temp >= 1 and y_temp >= 1 and
               x_temp <= x_size and y_temp <= y_size
               and CLOSED.find(hash_key) == CLOSED.end()){
                int cost = (int) map[temp_index];
                if((cost >= 0) and (cost < collision_thresh)){
                    if(nodes.find(hash_key) == nodes.end()){
                        int totalTime = timeElapsed + time;
                        int h = 0;
                        if (Target_nodes.find(temp_index) != Target_nodes.end() and totalTime <= Target_nodes[temp_index])
                            h = cost*(Target_nodes[temp_index] - totalTime);
                        else h =  heuristics[temp_index].first + abs(heuristics[temp_index].second - totalTime);
                        shared_ptr<node> n = make_shared<node>(temp_index, time, h);
                        nodes[hash_key] = n;
                    }
                    if(nodes[hash_key]->_g > s->_g + cost){ 
                        nodes[hash_key]->_g = s->_g + cost;
                        nodes[hash_key]->_f = nodes[hash_key]->_g + 1.8*nodes[hash_key]->_h; 
                        nodes[hash_key]->parent = s;
                        OPEN.push(nodes[hash_key]);
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
                shared_ptr<node> a = make_shared<node>(goal_idx, i, 0);
                a->_g = 0;
                a->_f = a->_h;
                nodes[goal_idx] = a;
                OPEN.push(a);
            }
        }
        Dijkstra(map, collision_thresh, x_size, y_size);
        nodes.clear();
        int index = GETMAPINDEX(robotposeX, robotposeY, x_size, y_size);
        int h = heuristics[index].first;
        shared_ptr<node> b = make_shared<node>(index, 0, h);
        b->_g = 0;
        b->_f = b->_h;
        nodes[index] = b;
        OPEN.push(b);
        A_star(map, target_traj, target_steps, x_size, y_size, collision_thresh);
    }
    if(!final_path.empty()){
        int action = final_path.top();
        final_path.pop();
        robot_prev_X = (action % x_size) + 1;
        robot_prev_Y = (action / x_size) + 1;
    }

    action_ptr[0] = robot_prev_X;
    action_ptr[1] = robot_prev_Y;
    return;
}