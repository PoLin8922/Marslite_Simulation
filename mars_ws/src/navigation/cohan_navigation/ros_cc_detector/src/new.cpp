void Detector::scanCallback(const sensor_msgs::LaserScanConstPtr &msg, std::string topic) {
    float x1, x2, y1, y2, inbetween_dist, prev_dist, dist1, dist2, x_temp, y_temp;
    float occlusion = 0;
    bool negativ_jump = 0;
    vector<int> new_cc_lifetimes;
    pcl::PointCloud<pcl::PointXYZ> new_cc_cloud;
    
    // Transform
    try {
        listener.lookupTransform(output_frame_, msg->header.frame_id, ros::Time(), transform);
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
        return; // Exit early if transform fails
    }

    // Convert ranges to 2D points
    sensor_msgs::PointCloud2 cloud_ros;
    projector_.projectLaser(*msg, cloud_ros);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_output(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(cloud_ros, *cloud);

    // Transform point cloud
    pcl_ros::transformPointCloud(*cloud, *cloud_output, transform);

    // Save modification state of current scan
    for (int j = 0; j < input_topics.size(); ++j) {
        if (topic.compare(input_topics[j]) == 0) {
            new_cc_cloud.clear();

            // Processing steps
            for (size_t i = 1; i < cloud->points.size(); i++) {
                x1 = cloud->points[i - 1].x;
                x2 = cloud->points[i].x;
                y1 = cloud->points[i - 1].y;
                y2 = cloud->points[i].y;

                inbetween_dist = sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
                dist1 = sqrt(pow(x1, 2) + pow(y1, 2));
                dist2 = sqrt(pow(x2, 2) + pow(y2, 2));

                if ((dist2 - dist1) > dist_threshold && occlusion > min_occlusion) {
                    float x = cloud_output->points[i - 1].x;
                    float y = cloud_output->points[i - 1].y;
                    
                    float theta_sens = atan2(cloud_base->points[i].y + transform.getOrigin().y(), cloud_base->points[i].x + transform_to_base.getOrigin().x());
                    float theta_diff = theta_vel - theta_sens;
                    theta_diff = abs(atan2(sin(theta_diff), cos(theta_diff)));

                    if (theta_diff < 1.6) {
                        new_cc_cloud.push_back(pcl::PointXYZ(x, y, 0));
                    }
                }

                if (abs(inbetween_dist) < dist_tolerance) {
                    occlusion += inbetween_dist;
                } else {
                    occlusion = 0;
                    negativ_jump = 0;
                }

                if ((dist1 - dist2) > dist_threshold) {
                    negativ_jump = 1;
                    x_temp = cloud_output->points[i].x;
                    y_temp = cloud_output->points[i].y;
                }

                if (negativ_jump == 1 && occlusion > min_occlusion) {
                    negativ_jump = 0;
                    float theta_sens = atan2(cloud_base->points[i].y + transform.getOrigin().y(), cloud_base->points[i].x + transform_to_base.getOrigin().x());
                    float theta_diff = theta_vel - theta_sens;
                    theta_diff = abs(atan2(sin(theta_diff), cos(theta_diff)));

                    if (theta_diff < 1.6) {
                        new_cc_cloud.push_back(pcl::PointXYZ(x_temp, y_temp, 0));
                    }
                }
            }

            // Initialize lifetimes
            new_cc_lifetimes.resize(new_cc_cloud.size() + cc_clouds[j].points.size(), k_death);

            // Update cc_clouds and cc_lifetimes
            for (const auto& new_point : new_cc_cloud.points) {
                bool found = false;
                size_t index = 0;
                for (const auto& point : cc_clouds[j].points) {
                    if (sqrt(pow(point.x - new_point.x, 2) + pow(point.y - new_point.y, 2)) < dist_tolerance) {
                        found = true;
                        break;
                    }
                    index++;
                }
                if (!found && index < cc_lifetimes.size()) {
                    new_cc_cloud.push_back(cc_clouds[j].points[index]);
                    new_cc_lifetimes[new_cc_cloud.size() - 1] = cc_lifetimes[index] - 1;
                }
            }

            if (j < cc_clouds.size()) {
                cc_clouds[j] = new_cc_cloud;
                cc_lifetimes = new_cc_lifetimes;
            }

            clouds_modified[j] = true;
        }
    }

    // Publish critical corners
    int totalClouds = count(clouds_modified.begin(), clouds_modified.end(), true);
    if (totalClouds == clouds_modified.size()) {
        pcl::PointCloud<pcl::PointXYZ> cc_cloud;

        costmap_converter::ObstacleArrayMsg critical_corners_temp;
        critical_corners = critical_corners_temp;
        critical_corners.header.frame_id = output_frame_;
        critical_corners.header.stamp = ros::Time::now();

        for (size_t i = 0; i < clouds_modified.size(); ++i) {
            for (const auto& point : cc_clouds[i].points) {
                cc_cloud.push_back(point);

                costmap_converter::ObstacleMsg corner;
                corner.header.frame_id = output_frame_;
                corner.header.stamp = ros::Time::now();
                corner.radius = 0;
                geometry_msgs::Point32 corner_point;
                corner_point.x = point.x;
                corner_point.y = point.y;
                corner.polygon.points.push_back(corner_point);

                critical_corners.obstacles.push_back(corner);
            }

            clouds_modified[i] = false; // reset
        }

        sensor_msgs::PointCloud2 cloud_vis;
        pcl::toROSMsg(cc_cloud, cloud_vis);
        cloud_vis.header.frame_id = output_frame_;
        cloud_vis.header.stamp = ros::Time::now();
        pub_vis.publish(cloud_vis);
    }
}