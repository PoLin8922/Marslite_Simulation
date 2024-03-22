#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <queue>
#include <vector>
#include "hyper_system/Navigability.h"

// Structure to represent a cell in the grid
struct Cell {
    int x, y;
    int value;
    Cell(int x_, int y_, int value_) : x(x_), y(y_), value(value_) {}
};

class LocalMapAnalyzer {
private:
    ros::NodeHandle nh_;
    ros::Subscriber map_sub_;
    ros::ServiceServer service_;
    nav_msgs::OccupancyGrid local_map_;
    bool map_received_;
    int room_space_threshold_ =  95;
    int navigable_space_threshold_ = 10;
    int map_size_;

public:
    LocalMapAnalyzer() : map_received_(false) {
        map_sub_ = nh_.subscribe("/move_base/local_costmap/costmap", 1, &LocalMapAnalyzer::mapCallback, this);
        service_ = nh_.advertiseService("navigability_srv", &LocalMapAnalyzer::analyzeLocalMapService, this);
    }

    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
        local_map_ = *msg;
        map_received_ = true;
        map_size_ = local_map_.info.width*local_map_.info.height;
    }

    // Service callback function
    bool analyzeLocalMapService(hyper_system::Navigability::Request& req,
                                hyper_system::Navigability::Response& res) {
        float navigability = analyzeLocalMap();
        res.navigability = navigability;
        return true;
    }

    // Dijkstra's Algorithm to traverse the grid and calculate the ratio of cells with value < 99
    float analyzeLocalMap() {
        if (!map_received_) {
            ROS_WARN("Local map not received yet!");
            return 0.0;
        }

        int center_x = local_map_.info.width / 2;
        int center_y = local_map_.info.height / 2;

        std::vector<std::vector<bool>> visited(local_map_.info.width, std::vector<bool>(local_map_.info.height, false));
        std::queue<Cell> q;
        q.push(Cell(center_x, center_y, local_map_.data[center_y * local_map_.info.width + center_x]));

        int navigable_space = 0;

        while (!q.empty()) {
            Cell current = q.front();
            q.pop();

            if (visited[current.x][current.y])
                continue;

            visited[current.x][current.y] = true;
                
            if (current.value < navigable_space_threshold_)
                navigable_space++;

            // Check neighbors
            for (int dx = -1; dx <= 1; dx++) {
                for (int dy = -1; dy <= 1; dy++) {
                    int nx = current.x + dx;
                    int ny = current.y + dy;
                    if (nx >= 0 && nx < local_map_.info.width && ny >= 0 && ny < local_map_.info.height &&
                        !visited[nx][ny] && local_map_.data[ny * local_map_.info.width + nx] >= 0 &&
                        local_map_.data[ny * local_map_.info.width + nx] < room_space_threshold_){
                        q.push(Cell(nx, ny, local_map_.data[ny * local_map_.info.width + nx]));
                    }
                }
            }
        }
        
        // printf("map_size:%d, navigable_space:%d \n", map_size_, navigable_space);
        float ratio = static_cast<float>(navigable_space) /  map_size_;
        return ratio;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "navigability_srv__node");
    LocalMapAnalyzer analyzer;
    ros::spin();
    return 0;


    // ros::Rate loop_rate(10);

    // while (ros::ok()) {
    //     float navigability = analyzer.analyzeLocalMap();
    //     ROS_INFO("navigability: %.2f", navigability);
    //     ros::spinOnce();
    //     loop_rate.sleep();
    // }

    // return 0;
}
