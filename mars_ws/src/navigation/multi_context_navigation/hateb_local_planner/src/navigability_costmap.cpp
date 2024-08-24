#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>

class NavigabilityCostmap
{
public:
    NavigabilityCostmap(tf2_ros::Buffer& tf) : tf_(tf)
    {
        ros::NodeHandle private_nh("~");
        
        costmap_ros_ = new costmap_2d::Costmap2DROS("navigability_costmap", tf_);
        costmap_ros_->pause();

        costmap_ros_->start();
    }

    ~NavigabilityCostmap()
    {
        delete costmap_ros_;
    }

private:
    tf2_ros::Buffer& tf_;
    costmap_2d::Costmap2DROS* costmap_ros_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "navigability_costmap");
    tf2_ros::Buffer buffer(ros::Duration(1));
    tf2_ros::TransformListener tf(buffer);
    NavigabilityCostmap navigability_costmap(buffer);
    ros::spin();
    return 0;
}
