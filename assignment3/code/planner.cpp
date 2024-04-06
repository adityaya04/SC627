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
#define INT_MAX INT32_MAX

bool debug_bool = true;

struct node
{
    int mapIndex;
    int g;
    int h;
    int f;
    int time;
    std::shared_ptr<node> parent;

    node() : parent(nullptr), g(INT_MAX), f(INT_MAX) {}
    node(int ind, int hVal, int t) : parent(nullptr), g(INT_MAX), f(INT_MAX)
    {
        mapIndex = ind;
        h = hVal;
        time = t;
    }
};

static auto compare = [](std::shared_ptr<node> n1, std::shared_ptr<node> n2)
{
    return n1->f > n2->f;
};

bool firstCall = true;
std::unordered_map<int, std::shared_ptr<node>> nodes;
std::unordered_map<int, int> goals;
std::unordered_map<int, std::pair<int, int>> heuristics;
std::unordered_set<int> closed;
std::priority_queue<std::shared_ptr<node>, std::vector<std::shared_ptr<node>>, decltype(compare)> openQueue(compare);
std::stack<int> actionStack;
std::chrono::time_point<std::chrono::steady_clock> startTime;

int prevX, prevY;

static void computeHeuristics(
        int x_size,
        int y_size,
        int dX[],
        int dY[],
        int* map,
        int collision_thresh
        )
{
    while(!openQueue.empty())
    {
        std::shared_ptr<node> s = openQueue.top();
        openQueue.pop();
        if(heuristics.find(s->mapIndex) != heuristics.end()) continue; // skip repetitions in open list
        heuristics[s->mapIndex] = std::pair<int, int>(s->g, s->time); // closed list. stores optimal g-vals

        int rX = (int)(s->mapIndex % x_size) + 1;
        int rY = (int)(s->mapIndex / x_size) + 1;

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
                        std::shared_ptr<node> n = std::make_shared<node>(newIndex, 0, s->time);
                        nodes[newIndex] = n;
                    }
                    if(nodes[newIndex]->g > s->g + cost) // compare g values and cost, update parent if needed
                    {
                        nodes[newIndex]->g = s->g + cost;
                        nodes[newIndex]->f = nodes[newIndex]->g + nodes[newIndex]->h;
                        nodes[newIndex]->parent = s;
                        openQueue.push(nodes[newIndex]);
                    }
                }
            }
        }
    }
}

static void computePath(
        int x_size,
        int y_size,
        int dX[],
        int dY[],
        int* map,
        int collision_thresh,
        int* target_traj,
        int target_steps
        )
{
    while(!openQueue.empty())
    {
        int timeElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - startTime).count();
        int newx, newy, newIndex, digits, newIndexForMap, cost;
        std::shared_ptr<node> s = openQueue.top();
        openQueue.pop();
        digits = (s->time == 0) ? 0 : (int)(std::log10(s->time) + 1);
        newIndexForMap = s->mapIndex * ((int)std::pow(10, digits)) + s->time; // concatenate time value to the end of index for unique key
        if(closed.find(newIndexForMap) != closed.end()) continue; // skip repetitions in open list
        closed.insert(newIndexForMap);

        int rX = (int)(s->mapIndex % x_size) + 1;
        int rY = (int)(s->mapIndex / x_size) + 1;


        if(goals.find(s->mapIndex) != goals.end() and s->time == (goals[s->mapIndex] - timeElapsed))
        {
            // goal reached, add all to action stack and return
            while(s)
            {
                actionStack.push(s->mapIndex);
                s = s->parent;
            }
            actionStack.pop(); // remove start node
            return;
        }
        
        int time = s->time + 1;
        if(time > target_steps)
        {
            continue;
        }
        for(int dir = 0; dir < (NUMOFDIRS + 1); ++dir)
        {
            newx = rX + dX[dir];
            newy = rY + dY[dir];
            newIndex = (int) GETMAPINDEX(newx, newy, x_size, y_size);
            digits = (time == 0) ? 0 : (int)(std::log10(time) + 1);
            newIndexForMap = newIndex * ((int)std::pow(10, digits)) + time; // concatenate time value to the end of index for unique key
            
            if(newx >= 1 and newx <= x_size and newy >= 1 and newy <= y_size and closed.find(newIndexForMap) == closed.end())
            {
                cost = (int) map[newIndex];
                if((cost >= 0) and (cost < collision_thresh)) // cell is free
                {
                    if(nodes.find(newIndexForMap) == nodes.end()) // create a new node, if it does not exist
                    {
                        int totalTime = timeElapsed + time;
                        int h = (goals.find(newIndex) != goals.end() and totalTime <= goals[newIndex]) ? cost*(goals[newIndex] - totalTime) : heuristics[newIndex].first + abs(heuristics[newIndex].second - totalTime);
                        std::shared_ptr<node> n = std::make_shared<node>(newIndex, h, time);
                        nodes[newIndexForMap] = n;
                    }
                    if(nodes[newIndexForMap]->g > s->g + cost) // compare g values and cost, update parent if needed
                    {
                        nodes[newIndexForMap]->g = s->g + cost;
                        nodes[newIndexForMap]->f = nodes[newIndexForMap]->g + 1.8*nodes[newIndexForMap]->h; // weighted A*
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

    int dX[NUMOFDIRS + 1] = {-1, -1, -1,  0,  0,  1, 1, 1, 0};
    int dY[NUMOFDIRS + 1] = {-1,  0,  1, -1,  1, -1, 0, 1, 0};

    int goalposeX = (int) target_traj[target_steps-1];
    int goalposeY = (int) target_traj[target_steps-1+target_steps];

    prevX = robotposeX;
    prevY = robotposeY;

    if(firstCall) // init s_start, g(start) = 0, add to the open set, and node map
    {
        startTime = std::chrono::steady_clock::now();
        firstCall = false;

        int gIndex;
        for(int i = 0; i < target_steps; ++i) // setup multi-goal map
        {
            gIndex = GETMAPINDEX((int) target_traj[i], (int) target_traj[target_steps + i], x_size, y_size);
            // atleast 1 second will be skipped after execution (ceiling function). so subtract 1 sec from goal times
            // goals[gIndex] = MAX(0, i - 1);
            goals[gIndex] = MAX(0, i);

            if(i > (target_steps/2))
            {
                std::shared_ptr<node> a = std::make_shared<node>(gIndex, 0, MAX(0, i - 1));
                a->g = 0;
                a->f = a->g + a->h;
                nodes[gIndex] = a;
                openQueue.push(a);
            }
        }

        computeHeuristics(x_size, y_size, dX, dY, map, collision_thresh);
        nodes.clear();

        int index = GETMAPINDEX(robotposeX, robotposeY, x_size, y_size);
        int h = heuristics[index].first;
        std::shared_ptr<node> b = std::make_shared<node>(index, h, 0);
        b->g = 0;
        b->f = b->g + b->h;
        nodes[index] = b;
        openQueue.push(b);
        // call compute path
        computePath(x_size, y_size, dX, dY, map, collision_thresh, target_traj, target_steps);
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

    if(debug_bool){
        debug_bool = false;
        // cout << "iter1 " << endl;
        // for(int i = 0; i < target_steps ; i++){
        //     cout << target_traj[i] << ',' << target_traj[i + target_steps] << endl;
        // }
        // cout << curr_time << endl;
    }
    return;
}