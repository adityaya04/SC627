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

static auto compare = [](shared_ptr<node> n1, shared_ptr<node> n2)
{
    return n1->_f > n2->_f;
};

bool initialized = true;
unordered_map<int, shared_ptr<node>> nodes;
unordered_map<int, int> goals;
unordered_map<int, pair<int, int>> heuristics;
unordered_set<int> closed;
priority_queue<shared_ptr<node>, vector<shared_ptr<node>>, decltype(compare)> openQueue(compare);
stack<int> actionStack;
chrono::time_point<chrono::steady_clock> startTime;

int prevX, prevY;

static void Dijkstra(
        int* map,
        int collision_thresh,
        int x_size,
        int y_size
        )
{
    while(!openQueue.empty())
    {
        shared_ptr<node> s = openQueue.top();
        openQueue.pop();
        if(heuristics.find(s->_index) != heuristics.end()) continue; // skip repetitions in open list
        heuristics[s->_index] = pair<int, int>(s->_g, s->_time); // closed list. stores optimal g-vals

        int rX = (int)(s->_index % x_size) + 1;
        int rY = (int)(s->_index / x_size) + 1;

        for(int dir = 0; dir < NUMOFDIRS; ++dir)
        {
            int newx = rX + dX[dir];
            int newy = rY + dY[dir];
            int newIndex = (int) GETMAPINDEX(newx, newy, x_size, y_size);

            if(newx >= 1 and newx <= x_size and newy >= 1 and newy <= y_size and heuristics.find(newIndex) == heuristics.end())
            {
                int cost = (int) map[newIndex];
                if((cost >= 0) and (cost < collision_thresh)) // cell is free
                {
                    if(nodes.find(newIndex) == nodes.end()) // create a new node, if it does not exist
                    {
                        shared_ptr<node> n = make_shared<node>(newIndex, s->_time, 0);
                        nodes[newIndex] = n;
                    }
                    if(nodes[newIndex]->_g > s->_g + cost) // compare g values and cost, update parent if needed
                    {
                        nodes[newIndex]->_g = s->_g + cost;
                        nodes[newIndex]->_f = nodes[newIndex]->_g + nodes[newIndex]->_h;
                        nodes[newIndex]->parent = s;
                        openQueue.push(nodes[newIndex]);
                    }
                }
            }
        }
    }
}

static void A_star(
        int* map,
        int* target_traj,
        int target_steps,
        int collision_thresh,
        int x_size,
        int y_size
        )
{
    while(!openQueue.empty())
    {
        int timeElapsed = chrono::duration_cast<chrono::seconds>(chrono::steady_clock::now() - startTime).count();
        int newx, newy, newIndex, digits, newIndexForMap, cost;
        shared_ptr<node> s = openQueue.top();
        openQueue.pop();
        digits = (s->_time == 0) ? 0 : (int)(log10(s->_time) + 1);
        newIndexForMap = s->_index * ((int)pow(10, digits)) + s->_time; // concatenate time value to the end of index for unique key
        if(closed.find(newIndexForMap) != closed.end()) continue; // skip repetitions in open list
        closed.insert(newIndexForMap);

        int rX = (int)(s->_index % x_size) + 1;
        int rY = (int)(s->_index / x_size) + 1;


        if(goals.find(s->_index) != goals.end() and s->_time == (goals[s->_index] - timeElapsed))
        {
            // goal reached, add all to action stack and return
            while(s)
            {
                actionStack.push(s->_index);
                s = s->parent;
            }
            actionStack.pop(); // remove start node
            return;
        }
        
        int time = s->_time + 1;
        if(time > target_steps)
        {
            continue;
        }
        for(int dir = 0; dir < (NUMOFDIRS + 1); ++dir)
        {
            newx = rX + dX[dir];
            newy = rY + dY[dir];
            newIndex = (int) GETMAPINDEX(newx, newy, x_size, y_size);
            digits = (time == 0) ? 0 : (int)(log10(time) + 1);
            newIndexForMap = newIndex * ((int)pow(10, digits)) + time; // concatenate time value to the end of index for unique key
            
            if(newx >= 1 and newx <= x_size and newy >= 1 and newy <= y_size and closed.find(newIndexForMap) == closed.end())
            {
                cost = (int) map[newIndex];
                if((cost >= 0) and (cost < collision_thresh)) // cell is free
                {
                    if(nodes.find(newIndexForMap) == nodes.end()) // create a new node, if it does not exist
                    {
                        int totalTime = timeElapsed + time;
                        int h = (goals.find(newIndex) != goals.end() and totalTime <= goals[newIndex]) ? cost*(goals[newIndex] - totalTime) : heuristics[newIndex].first + abs(heuristics[newIndex].second - totalTime);
                        shared_ptr<node> n = make_shared<node>(newIndex, time, h);
                        nodes[newIndexForMap] = n;
                    }
                    if(nodes[newIndexForMap]->_g > s->_g + cost) // compare g values and cost, update parent if needed
                    {
                        nodes[newIndexForMap]->_g = s->_g + cost;
                        nodes[newIndexForMap]->_f = nodes[newIndexForMap]->_g + 1.8*nodes[newIndexForMap]->_h; // weighted A*
                        nodes[newIndexForMap]->parent = s;
                        openQueue.push(nodes[newIndexForMap]);
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

    int goalposeX = (int) target_traj[target_steps-1];
    int goalposeY = (int) target_traj[target_steps-1+target_steps];

    prevX = robotposeX;
    prevY = robotposeY;

    if(initialized) 
    {
        startTime = chrono::steady_clock::now();
        initialized = false;

        int gIndex;
        for(int i = 0; i < target_steps; ++i) 
        {
            gIndex = GETMAPINDEX((int) target_traj[i], (int) target_traj[target_steps + i], x_size, y_size);

            goals[gIndex] = i;

            if(i > (target_steps/2))
            {
                shared_ptr<node> a = make_shared<node>(gIndex, i, 0);
                a->_g = 0;
                a->_f = a->_h;
                nodes[gIndex] = a;
                openQueue.push(a);
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
        openQueue.push(b);
        A_star(map, target_traj, target_steps, collision_thresh, x_size, y_size);
    }
    if(!actionStack.empty())
    {
        int nextIndex = actionStack.top();
        actionStack.pop();
        prevX = (nextIndex % x_size) + 1;
        prevY = (nextIndex / x_size) + 1;
    }

    action_ptr[0] = prevX;
    action_ptr[1] = prevY;

    return;
}