#include <l3xz_mapping/map/map_lidar.hpp>

std::shared_ptr<MapLidar> map;

ros::Subscriber subOdom;
ros::Publisher pubGrid;

void callbackOdomIn(const nav_msgs::Odometry &msg) { map->setOdometry(msg); }

void callbackLidarIn(const sensor_msgs::LaserScanConstPtr msg) { map->add(*msg); }

int main(int argc, char **argv)
{
    ros::init(argc, argv, "knownposition");
    ros::NodeHandle nh("~");
    map = std::make_shared<MapLidar>("lidar", 100, -1, 2.5, 180.0, nh.param("pixel_x", 600),
                                     nh.param("pixel_y", 600), nh.param("resolution", 0.1));
    map->setDebug(true);
    subOdom =
        nh.subscribe(nh.param("odometry_topic", std::string("/odom_slam")), 1, callbackOdomIn);
    pubGrid =
        nh.advertise<nav_msgs::OccupancyGrid>(nh.param("grid_topic", std::string("/grid")), 1);

    ros::Subscriber lidar_subscriber = nh.subscribe<sensor_msgs::LaserScan>(
        nh.param("lidar_topic", std::string("/road_detector/road_lidar")), 1,
        boost::bind(callbackLidarIn, _1));

    ros::Rate r(nh.param("rate_hz", 5));
    while (ros::ok())
    {
        if (nh.param("show", false))
        {
            cv::imshow("map", map->getMat());
            cv::waitKey(10);
        }
        pubGrid.publish(*map->getMap());
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
