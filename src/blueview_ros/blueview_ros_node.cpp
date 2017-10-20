#include <iostream>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/LaserScan.h>

#include <blueview_ros/Sonar.h>

using std::cout;
using std::endl;

// Create a Sonar
Sonar sonar;

void MinRangeCallback(const std_msgs::Float32::ConstPtr& msg)
{
    sonar.setMinRange(msg->data);
}

void MaxRangeCallback(const std_msgs::Float32::ConstPtr& msg)
{
    sonar.setMaxRange(msg->data);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bluview_ros_node");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");



    ///////////////////////////////////////////////////
    // Acquire params from paramserver
    ///////////////////////////////////////////////////
    double min_dist = 0,
            max_dist = 20, tick_rate;
    std::string mode,
            color_map,
            net_or_file, sonar_file, ip_addr;

    // Grab distance range
    nh.getParam("min_dist", min_dist);
    nh.getParam("tick_rate", tick_rate);

    nh.getParam("max_dist", max_dist);

    // Grab mode (image or range)
    nh.getParam("mode", mode);

    // Grab color map filename
    nh.getParam("color_map", color_map);

    nh.getParam("net_or_file", net_or_file);
    nh.getParam("ip_addr", ip_addr);
    nh.getParam("sonar_file", sonar_file);



    // Determine if a live "net" sonar will be used or if we are reading
    // from a file

    if (net_or_file == "net")
    {
        sonar.setMode(Sonar::net);
        sonar.setIpAddr(ip_addr);
    }
    else
    {
        sonar.setMode(Sonar::sonar_file);
        sonar.setInputSonFilename(sonar_file);
    }

    sonar.setRange(min_dist, max_dist);
    if (mode == "range")
        sonar.setDataMode(Sonar::range);
    else
        sonar.setDataMode(Sonar::image);

    sonar.setColorMap(color_map);


    cout << "min_dist: " << min_dist << endl;
    cout << "max_dist: " << max_dist << endl;
    cout << "color_map: " << color_map << endl;
    cout << "sonar_file: " << sonar_file << endl;

    // Initialize the sonar
    sonar.init();

    //Publish opencv image of sonar
    image_transport::ImageTransport it(n);
    image_transport::Publisher image_pub = it.advertise("sonar_image", 1);
    ros::Publisher scan_pub = n.advertise<sensor_msgs::LaserScan>("scan", 1);

    //Subscribe to range commands
    ros::Subscriber min_range_sub = nh.subscribe("sonar_min_range", 1, MinRangeCallback);
    ros::Subscriber max_range_sub = nh.subscribe("sonar_max_range", 1, MaxRangeCallback);


    // cv bridge static settings:
    cv_bridge::CvImage cvi;
    cvi.header.frame_id = "image";
    cvi.encoding = "bgra8"; // sonar image is four channels

    // Image sensor message
    sensor_msgs::Image msg_img;
    sensor_msgs::LaserScan msg_laser;
    msg_laser.angle_min = sonar.getBearingMinAngle();
    msg_laser.angle_max = sonar.getBearingMaxAngle();
    msg_laser.angle_increment = sonar.getBearingResolution();
    msg_laser.range_min = sonar.getRangeMin();
    msg_laser.range_max = sonar.getRangeMax();

    ros::Time ros_time;
    ros::Rate rate(tick_rate);

    cv::Mat img;
    std::vector<double> ranges;
    int status;
    while (ros::ok())
    {
        status = sonar.getNextSonarData();
        status = sonar.getSonarImage(img);
        status = sonar.getSonarScan(ranges);
        if (status == Sonar::Success)
        {
            try
            {
                // convert OpenCV image to ROS message
                ros_time = ros::Time::now();
                cvi.header.stamp = ros_time;
                cvi.image = img;
                cvi.toImageMsg(msg_img);

                // Publish the image
                image_pub.publish(msg_img);

                msg_laser.ranges.clear();
                msg_laser.ranges.insert(msg_laser.ranges.begin(), ranges.begin(),ranges.end());
                scan_pub.publish(msg_laser);

            } catch (cv_bridge::Exception& e) {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return -1;
            }
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
