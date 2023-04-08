#include <vector>
#include <array>
#include <iostream>
#include <math.h>
#include <cstring>
#include "tree.h"
#include "map.h"

#define EPSILON 0.2

#define TRAPPED 0
#define ADVANCED 1
#define REACHED 2

using std::vector;
using std::array;
using std::cout;
using std::endl;

bool isValid(const Map* map, double x, double y, const std::vector<int>* obstacle_ids)
{
    for (int obstacle_id : *obstacle_ids)
    {
        if (obstacle_id < 0 || obstacle_id >= map->num_obstacles())
        {
            std::cerr << "Invalid obstacle ID: " << obstacle_id << std::endl;
            continue;
        }

        const auto& obstacle = map->obstacles()[obstacle_id];
        double dx = obstacle.x - x;
        double dy = obstacle.y - y;
        double dist_sq = dx * dx + dy * dy;
        if (dist_sq <= obstacle.r * obstacle.r)
        {
            // The point is inside the obstacle
            return false;
        }
    }
    // The point is not inside any obstacles
    return true;
}

// function to generate random point in the map
double* generateRandomPoint(const Map* map, const std::vector<int>* obstacle_ids)
{
    double* point = new double[2];    
    do
    {
        point[0] = map->x_min() + (map->x_max() - map->x_min()) * (double)rand() / RAND_MAX;
        point[1] = map->y_min() + (map->y_max() - map->y_min()) * (double)rand() / RAND_MAX;
    } while (!isValid(map, point[0], point[1], obstacle_ids));
    return point;
}

bool equalDoubleArrays(double* v1, double *v2, int size) {
    for (int i = 0; i < size; ++i) {
        if (abs(v1[i]-v2[i]) > 1e-3) {
            return false;
        }
    }
    return true;
}