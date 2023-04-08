#include <iostream>
#include <vector>
#include <yaml-cpp/yaml.h>

class Map
{
public:
    struct Obstacle
    {
        double x;
        double y;
        double r;
    };

    Map(const std::string& filename)
    {
        // Load the YAML file
        YAML::Node config = YAML::LoadFile(filename);

        // Read the number of obstacles
        num_obstacles_ = config["num_obstacles"].as<int>();

        // Read the map bounds
        x_min_ = config["x_min"].as<double>();
        x_max_ = config["x_max"].as<double>();
        y_min_ = config["y_min"].as<double>();
        y_max_ = config["y_max"].as<double>();

        // Read the obstacles
        YAML::Node obstacles = config["obstacles"];
        for (int i = 0; i < num_obstacles_; i++)
        {
            YAML::Node obstacle = obstacles[i];

            // Read the obstacle properties
            double x = obstacle["x"].as<double>();
            double y = obstacle["y"].as<double>();
            double r = obstacle["r"].as<double>();

            // Add the obstacle to the list
            obstacles_.push_back({x, y, r});
        }
    }

    int num_obstacles() const { return num_obstacles_; }
    double x_min() const { return x_min_; }
    double x_max() const { return x_max_; }
    double y_min() const { return y_min_; }
    double y_max() const { return y_max_; }

    const std::vector<Obstacle>& obstacles() const { return obstacles_; }

    std::vector<Obstacle> find_obstacles(const std::vector<int>& obstacle_ids) const
    {
        std::vector<Obstacle> result;
        for (int id : obstacle_ids)
        {
            if (id < 0 || id >= num_obstacles_)
            {
                std::cerr << "Invalid obstacle ID " << id << std::endl;
                continue;
            }
            result.push_back(obstacles_[id]);
        }
        return result;
    }

private:
    int num_obstacles_;
    std::vector<Obstacle> obstacles_;
    double x_min_;
    double x_max_;
    double y_min_;
    double y_max_;
};

// int main()
// {
//     Map config("config.yaml");

//     // Print all obstacles
//     std::cout << "Number of obstacles: " << config.num_obstacles() << std::endl;
//     for (int i = 0; i < config.num_obstacles(); i++)
//     {
//         const Map::Obstacle& obstacle = config.obstacles()[i];
//         std::cout << "Obstacle " << i << ":" << std::endl;
//         std::cout << "  x = " << obstacle.x << std::endl;
//         std::cout << "  y = " << obstacle.y << std::endl;
//         std::cout << "  r = " << obstacle.r << std::endl;
//     }

//     // Find specific obstacles
//     std::vector<int> ids_to_find = {0, 2, 4};
//     std::vector<Map::Obstacle> found_obstacles = config.find_obstacles(ids_to_find);
//     for (const Map::Obstacle& obstacle : found_obstacles)
//     {
//         std::cout << "Found obstacle:" << std::endl;
//         std::cout << "  x = " << obstacle.x << std::endl;
//         std::cout << "  y = " << obstacle.y << std::endl;
//         std::cout << "  r = " << obstacle.r << std::endl;
//     }

//     return 0;
// }
