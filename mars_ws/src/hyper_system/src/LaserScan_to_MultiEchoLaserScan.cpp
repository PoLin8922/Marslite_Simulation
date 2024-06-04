#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/MultiEchoLaserScan.h"

class convert_LaserScan_to_MultiEchoLaserScan{
    public:
        convert_LaserScan_to_MultiEchoLaserScan(){
            //Topic you want to publish
            pub_ = n_.advertise<sensor_msgs::MultiEchoLaserScan>("/horizontal_laser_2d", 1000);

            //Topic you want to subscribe
            sub_ = n_.subscribe("/scan", 1000, &convert_LaserScan_to_MultiEchoLaserScan::callback, this);
        }


        void callback(const sensor_msgs::LaserScan::ConstPtr& input){
            //sensor_msgs::LaserScan output;

            sensor_msgs::MultiEchoLaserScan muls;
            for (int i = 0; i < input->ranges.size(); ++i) {
            const float &range = input->ranges[i];
            ROS_INFO("ranges: [%f]",range);
            sensor_msgs::LaserEcho echo;
            echo.echoes.push_back(range);
            muls.ranges.push_back(echo);
            }

            pub_.publish(muls);
        }

    private:
    ros::NodeHandle n_;
    ros::Publisher pub_;
    ros::Subscriber sub_;

};//End of class convert_LaserScan_to_MultiEchoLaserScan


int main(int argc, char **argv)
{  
    //Initiate ROS
    ros::init(argc, argv, "convert_LaserScan_to_MultiEchoLaserScan");

    //Create an object of class SubscribeAndPublish that will take care of everything
    convert_LaserScan_to_MultiEchoLaserScan SAPObject;

    ros::spin();



    return 0;
}