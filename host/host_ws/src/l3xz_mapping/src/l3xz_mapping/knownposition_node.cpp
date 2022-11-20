#include <l3xz_mapping/map/map_lidar.hpp>

std::shared_ptr<MapLidar> map;

ros::Subscriber subOdom;
ros::Publisher pubGrid;

void callbackOdomIn(const nav_msgs::Odometry &msg) { map->setOdometry(msg); }

void callbackLidarIn(const sensor_msgs::LaserScanConstPtr msg)
{
    map->add(*msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "knownposition");
    ros::NodeHandle nh("~");
    map = std::make_shared<MapLidar>(100, -1, 2.5, 180.0, nh.param("pixel_x", 600), nh.param("pixel_y", 600),
                                nh.param("resolution", 0.1));
    subOdom =
        nh.subscribe(nh.param("odometry_topic", std::string("/odom_slam")), 1, callbackOdomIn);
    pubGrid =
        nh.advertise<nav_msgs::OccupancyGrid>(nh.param("grid_topic", std::string("/grid")), 1);

    std::string lidars = nh.param("lidar_sources", std::string("/road_detector/road_lidar, 100, -1, 2.5"));
    lidars.erase(std::remove(lidars.begin(), lidars.end(), ' '), lidars.cend());
    lidars.erase(std::remove(lidars.begin(), lidars.end(), '\n'), lidars.cend());
    std::vector<std::string> configs;

    size_t pos = 0;
    while ((pos = lidars.find(",")) != std::string::npos)
    {
        configs.push_back(lidars.substr(0, pos));
        lidars.erase(0, pos + 1);
    }
    configs.push_back(lidars.substr(0, pos));

    std::vector<ros::Subscriber> lidar_subscribers;
    for (uint32_t i = 0; i < configs.size(); i = i + 4)
    {
        int8_t block = static_cast<int8_t>(std::stoi(configs.at(i + 1)));
        int8_t unblock = static_cast<int8_t>(std::stoi(configs.at(i + 2)));
        double dist = static_cast<double>(std::stod(configs.at(i + 3)));
        ROS_INFO("Subscribing %s", configs.at(0).c_str());
        ROS_INFO("Block: %i, unblock: %i, max_dist: %f", block, unblock, dist);
        lidar_subscribers.push_back(nh.subscribe<sensor_msgs::LaserScan>(
            configs.at(0), 1, boost::bind(callbackLidarIn, _1)));
    }

    ros::Rate r(nh.param("rate_hz", 5));
    while (ros::ok())
    {
        if (nh.param("show", false))
        {
            cv::imshow("map", map->getMat());
            cv::waitKey(10);
        }
        pubGrid.publish(*map->getMap("map"));
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
