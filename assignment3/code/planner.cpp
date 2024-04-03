/*=================================================================
 *
 * planner.cpp
 *
 *=================================================================*/
#include "planner.h"
#include <math.h>
#include <iostream>
#include <chrono>
#include <time.h>
#include <queue>
#include <vector>
#include <stack>
#include <unordered_map>

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

struct ArrayHasher {
    std::size_t operator()(const std::array<int, 3>& a) const {
        std::size_t h = 0;
        for (auto e : a) {
            h ^= std::hash<int>{}(e)  + 0x9e3779b9 + (h << 6) + (h >> 2); 
        }
        return h;
    }   
};

struct cell2d{

    pair<int,int> parent;

    int f, g, h;
    cell2d()
        : parent(-1, -1)
        , f(-1)
        , g(-1)
        , h(-1)
    {
    }
};

struct cell{

    vector<int> parent;
    int f, g, h;

    cell()
        : parent({-1, -1, -1})
        , f(-1)
        , g(-1)
        , h(-1)
    {
    }
};

typedef pair<int, vector<int>> listPair;
typedef pair<int, pair<int, int>> listPair2d;

stack<pair<int,int>> Path;
queue<pair<int,int>> Path2d;
bool have_path = false, better_2dpath = false;
vector<vector<cell2d>> grid2d;
int path_size_2d = INT_MAX;
int cost_2d = INT_MAX;
int trace_idx = 0;

bool isValid(int x, int y, int x_size, int y_size, int* map, int collision_thresh){
    if(((int)map[GETMAPINDEX(x,y,x_size,y_size)] >= 0) && ((int)map[GETMAPINDEX(x,y,x_size,y_size)] < collision_thresh))
        return true;
    return false;
}

vector<int> getPath2d(vector<vector<cell2d>> &grid, int goalposeX, int goalposeY){
    int row = goalposeX;
    int col = goalposeY;
    path_size_2d = 0;
    while (!(grid[row][col].parent.first == row
            && grid[row][col].parent.second == col)){
        Path2d.push(make_pair(row, col));
        int temp_row = grid[row][col].parent.first;
        int temp_col = grid[row][col].parent.second;
        row = temp_row;
        col = temp_col;
        path_size_2d++;
    }
    Path2d.push(make_pair(row, col));
    pair<int,int> p = Path2d.front();

    return {p.first, p.second};
}

vector<int> getPath(unordered_map<array<int,3>, cell, ArrayHasher> &grid, int goalposeX, int goalposeY, int goalT){
    int row = goalposeX;
    int col = goalposeY;
    int t = goalT;
    while (!(grid[{row,col,t}].parent[0] == row
             && grid[{row,col,t}].parent[1] == col && grid[{row,col,t}].parent[2] == t)){
        Path.push(make_pair(row, col));
        int temp_row = grid[{row,col,t}].parent[0];
        int temp_col = grid[{row,col,t}].parent[1];
        int temp_time = grid[{row,col,t}].parent[2];
        row = temp_row;
        col = temp_col;
        t = temp_time;
    }
    pair<int, int> p = Path.top();
    return {p.first, p.second};
}

vector<int> search_2d(
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
    int curr_time
    )   
{
    int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};
    int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};
    int i, j, epsilon=1;

    int goalposeX = robotposeX;
    int goalposeY = robotposeY;
    vector<vector<bool>> closed(x_size+1, vector<bool> (y_size+1, false));
    vector<vector<bool>> opened(x_size+1, vector<bool> (y_size+1, false));
    grid2d.clear();
    grid2d.resize(x_size+1, vector<cell2d>(y_size+1));
    for (i = 1; i <=x_size; i++) {
        for (j = 1; j <=y_size; j++) {
            grid2d[i][j].f = INT_MAX;
            grid2d[i][j].g = INT_MAX;
            grid2d[i][j].h = INT_MAX;
            grid2d[i][j].parent = make_pair(-1, -1);
        }
    }
    robotposeX = (int) target_traj[target_steps-1]; 
    robotposeY = (int) target_traj[target_steps-1+target_steps];
    i=robotposeX, j=robotposeY;
    priority_queue<listPair2d, vector<listPair2d>, greater<listPair2d>> open2d;
    for(int k=target_steps/2; k<target_steps; k++){
        grid2d[target_traj[k]][target_traj[k+target_steps]].f = 0;
        grid2d[target_traj[k]][target_traj[k+target_steps]].g = 0;
        grid2d[target_traj[k]][target_traj[k+target_steps]].h = 0;
        grid2d[target_traj[k]][target_traj[k+target_steps]].parent = make_pair(i, j);
        open2d.push(make_pair(0, make_pair(target_traj[k], target_traj[k+target_steps])));
    } 
    int newx, newy;
    vector<int> new_pose={robotposeX, robotposeY};
    bool found_path = false;
    if(robotposeX != goalposeX || robotposeY != goalposeY){
        while (!open2d.empty()){   
            bool next = false;
            while (!next){
                listPair2d curr = open2d.top();
                open2d.pop();
                i = curr.second.first;
                j = curr.second.second;
                if(closed[i][j]==false) next = true;
            }
            closed[i][j] = true;
            int gNew, hNew, fNew;
            if(i == goalposeX && j == goalposeY){
                cost_2d = grid2d[i][j].g;
                new_pose = getPath2d(grid2d, goalposeX, goalposeY);
                found_path = true;
            }
            for(int dir = 0; dir < NUMOFDIRS; dir++){
                newx = i + dX[dir];
                newy = j + dY[dir];
                if(newx >= 1 && newx <=x_size && newy >= 1 && newy <=y_size){
                    if(closed[newx][newy]==false && isValid(newx, newy, x_size, y_size, map, collision_thresh)){
                        gNew = grid2d[i][j].g + (int)map[GETMAPINDEX(newx, newy, x_size, y_size)];
                        hNew = 0;
                        fNew = gNew + hNew;
                        if(grid2d[newx][newy].g == INT_MAX || grid2d[newx][newy].g > gNew){
                            open2d.push(make_pair(fNew, make_pair(newx, newy)));
                            grid2d[newx][newy].f = fNew;  
                            grid2d[newx][newy].g = gNew;
                            grid2d[newx][newy].h = hNew;
                            grid2d[newx][newy].parent = make_pair(i, j);
                        }
                    }
                }
            }
        }
    }
    return new_pose;
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
    if(curr_time == 0) {
        have_path = false;
        Path = stack<pair<int,int>>();
        Path2d = queue<pair<int,int>>();
        path_size_2d = INT_MAX;
        cost_2d = INT_MAX;
        better_2dpath = false;
        trace_idx = target_steps - 1;
    }
    if(have_path){
        if(!better_2dpath){
            if(Path.size() > 1){
                Path.pop();
                pair<int, int> p = Path.top();
                action_ptr[0] = p.first;
                action_ptr[1] = p.second;
            }
            else{
                action_ptr[0] = robotposeX;
                action_ptr[1] = robotposeY;
            }
            return;
        }
        else{
            if(Path2d.size()>1){
                Path2d.pop();
                pair<int, int> p = Path2d.front();
                action_ptr[0] = p.first;
                action_ptr[1] = p.second;
            }
            else{
                action_ptr[0] = target_traj[trace_idx];
                action_ptr[1] = target_traj[trace_idx+target_steps];
                trace_idx--;
            }
            return;
        }
    }

    vector<int> new_pose2d = {robotposeX, robotposeY};
    new_pose2d = search_2d(map, collision_thresh, x_size, y_size, robotposeX, robotposeY, target_steps, target_traj, targetposeX, targetposeY, curr_time);

    int dX[NUMOFDIRS+1] = {0, -1, -1, -1,  0,  0,  1, 1, 1};
    int dY[NUMOFDIRS+1] = {0, -1,  0,  1, -1,  1, -1, 0, 1};
    int dT = 1;
    vector<int> new_pose={robotposeX, robotposeY};
    clock_t tStart = clock();
    double epsilon = 2;
    int buffer_time = (int) 5*(double)MAX(x_size,y_size)/200;

    int goalposeX = (int) target_traj[target_steps-1];
    int goalposeY = (int) target_traj[target_steps-1+target_steps];
    
    unordered_map<array<int,3> , cell, ArrayHasher >  grid;
    unordered_map<array<int,3> , bool, ArrayHasher >  closed;
    unordered_map<array<int,3> , bool, ArrayHasher >  opened;
    priority_queue<listPair, vector<listPair>, greater<listPair>> open;
    int i = robotposeX, j = robotposeY, k = curr_time;
    cell c = {};
    c.parent = vector<int> {i, j, k};
    c.g = 0;
    c.f = 0;
    c.h = 0;
    grid[{i,j,k}] = c;

    open.push(make_pair(0, c.parent));
    int newx, newy, newt;
    bool found_path = false;
    int num_expanded = 0;
    while(!open.empty()){
        bool next = false;
        while (!next){
            listPair curr = open.top();
            open.pop();
            i = curr.second[0];
            j = curr.second[1];
            k = curr.second[2];
            if((closed.find({i,j,k}) == closed.end() || closed[{i,j,k}]==false))
                next=true;            
        }
        closed[{i, j, k}] = true;
        int time_elapsed = buffer_time + (int)((clock() - tStart)/CLOCKS_PER_SEC);
        int target_x = (int) target_traj[MIN(curr_time+k+time_elapsed, target_steps-1)];
        int target_y = (int) target_traj[MIN(curr_time+k+time_elapsed+target_steps, 2*target_steps-1)];
        int gNew, hNew, fNew;
        num_expanded++;
        if(i == target_x && j == target_y && curr_time + time_elapsed + k <= target_steps){
            new_pose = getPath(grid, target_x, target_y, k);
            found_path = true;
            have_path = true;
        }

        for(int dir = 0; dir < NUMOFDIRS; dir++){
            newx = i + dX[dir];
            newy = j + dY[dir];
            newt = k + dT;
            if(newx >= 1 && newx <= x_size && newy >=1 && newy <= y_size && curr_time+newt+time_elapsed <= target_steps){
                if( (closed.find({newx,newy,newt}) == closed.end() || closed[{newx,newy,newt}]==false) && isValid(newx,newy,x_size,y_size,map,collision_thresh)){
                    gNew = grid[{i,j,k}].g + (int)map[GETMAPINDEX(newx,newy,x_size,y_size)];
                    hNew = (int) epsilon*grid2d[newx][newy].g;
                    fNew = gNew + hNew;

                    if(grid.find({newx, newy, newt}) == grid.end()){
                        cell c = {};
                        c.parent = vector<int> {-1, -1, -1};
                        c.g = INT_MAX;
                        c.h = INT_MAX;
                        c.f = INT_MAX;
                        grid[{newx,newy,newt}] = c;
                    }
                    if(grid[{newx, newy, newt}].g == INT_MAX || grid[{newx,newy,newt}].g > gNew){
                        open.push(make_pair(fNew, vector<int> {newx, newy, newt}));
                        grid[{newx,newy,newt}].f = fNew;  
                        grid[{newx,newy,newt}].g = gNew;                                                                // update g(s')
                        grid[{newx,newy,newt}].h = hNew;
                        grid[{newx,newy,newt}].parent =  vector<int> {i,j,k};
                    }
                }
            }
        }
        if(found_path) break;
        
    }
    if(!found_path){
        new_pose = search_2d(map, collision_thresh, x_size, y_size, robotposeX, robotposeY, target_steps, target_traj, targetposeX, targetposeY, curr_time);
        have_path = true;
        better_2dpath = true;
    }

    robotposeX = new_pose[0];
    robotposeY = new_pose[1];
    action_ptr[0] = robotposeX;
    action_ptr[1] = robotposeY; 

    if(debug_bool){
        debug_bool = false;
        cout << "iter1 " << endl;
        // for(int i = 0; i < target_steps ; i++){
        //     cout << target_traj[i] << ',' << target_traj[i + target_steps] << endl;
        // }
        // cout << curr_time << endl;
    }
    cout << "Time taken " << (double) ((clock() - tStart)/CLOCKS_PER_SEC) << endl;
    return;
}