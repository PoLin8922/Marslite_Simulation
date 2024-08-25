#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Point.h"

// define scene map color
#define MALL 98
#define CORRIDER 120
#define WAREHOUSE 10 

class ScenePublisher {
public:
    ScenePublisher() : tfListener_(tfBuffer_){
        // Subscribe to the map topic
        map_subscriber_ = nh_.subscribe("/scene_map", 1, &ScenePublisher::mapCallback, this);

        // Subscribe to the odometry topic
        odom_subscriber_ = nh_.subscribe("/odom", 1, &ScenePublisher::odomCallback, this);

        // Publish to scene topic
        scene_publisher_ = nh_.advertise<std_msgs::String>("/scenario", 10);

        // Publish robot position in map frame
        position_publisher_ = nh_.advertise<geometry_msgs::Point>("/robot_position_map", 10);
    }

    ~ScenePublisher() {
        // Free allocated memory
        delete[] data;
    }

    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg) {
        // Process map data
        ROS_INFO("Received map data");
        
        resolution = map_msg->info.resolution;
        origin_x = map_msg->info.origin.position.x;
        origin_y = map_msg->info.origin.position.y;
        width = map_msg->info.width;
        ROS_INFO("Resolution: %f", resolution);
        ROS_INFO("Origin: [%f, %f]", origin_x , origin_y);
        ROS_INFO("Width : %f", width);
        ROS_INFO("Data size: %zu", map_msg->data.size());
        
        data = new double[map_msg->data.size()];
        for (size_t i = 0; i < map_msg->data.size(); ++i) 
            data[i] = static_cast<double>(map_msg->data[i]);
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg) {
        getRobotPositionInMap();
        // printf("roobt px : %f, py: %f \n", robot_px, robot_py);

        int index = (int)((robot_px - origin_x)/resolution) + (int)(((robot_py - origin_y)/resolution - 1))*width;
        // ROS_INFO("Robot's Position in Map Frame: x=%f, y=%f", robot_px, robot_py);
        // ROS_INFO("Index : %d", index);
        // ROS_INFO("Map data : %f", data[index]);

        std::string scenario;
        if(data[index] == MALL)
            scenario = "mall";
        else if(data[index] == CORRIDER)
            scenario = "corrider";
        else if(data[index] == WAREHOUSE)
            scenario = "warehouse";
        else
            scenario = "unknown";

        std_msgs::String msg;
        msg.data = scenario;
        scene_publisher_.publish(msg);

        geometry_msgs::Point position_msg;
        position_msg.x = robot_px;
        position_msg.y = robot_py;
        position_msg.z = 0.0;  

        position_publisher_.publish(position_msg);
    }

    void getRobotPositionInMap() {
        geometry_msgs::PoseStamped robot_pose_base;

        // Set the frame ID of the robot's pose to the base frame
        robot_pose_base.header.frame_id = "base_link";
        robot_pose_base.header.stamp = ros::Time(0);

        try {
            geometry_msgs::PoseStamped robot_pose_map;
            tfBuffer_.transform(robot_pose_base, robot_pose_map, "map");
            robot_px = robot_pose_map.pose.position.x;
            robot_py = robot_pose_map.pose.position.y;
        } 
        catch (tf2::TransformException& ex) {
            ROS_WARN("Could not transform robot's pose: %s", ex.what());
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber map_subscriber_;
    ros::Subscriber odom_subscriber_;
    ros::Publisher scene_publisher_;
    ros::Publisher position_publisher_;

    double resolution;
    double *data, origin_x, origin_y, width;;
    double robot_px, robot_py;

    // Transform Listener for TF transformations
    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "map_and_odom_reader");
    ScenePublisher scene_pub;

    ros::spin();  // Keep the program alive

    return 0;
}
