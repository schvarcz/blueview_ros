#include <iostream>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Float32.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <geometry_msgs/Point32.h>

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
            max_dist = 40, tick_rate = 10;
    std::string mode,
            color_map,
            net_or_file, address;

    // Grab distance range
    nh.getParam("min_dist", min_dist);
    nh.getParam("tick_rate", tick_rate);

    nh.getParam("max_dist", max_dist);

    // Grab mode (image or range)
    nh.getParam("mode", mode);

    // Grab color map filename
    nh.getParam("color_map", color_map);

    nh.getParam("net_or_file", net_or_file);
    nh.getParam("address", address);

    // Determine if a live "net" sonar will be used or if we are reading
    // from a file
    if (net_or_file == "net")
        sonar.setMode(Sonar::net);
    else
        sonar.setMode(Sonar::sonar_file);

    sonar.setAddress(address);

    sonar.setRange(min_dist, max_dist);
    if (mode == "range")
        sonar.setDataMode(Sonar::range);
    else
        sonar.setDataMode(Sonar::image);

    sonar.setColorMap(color_map);

    cout << "min_dist: " << min_dist << endl;
    cout << "max_dist: " << max_dist << endl;
    cout << "color_map: " << color_map << endl;
    cout << "address: " << address << endl;

    // Initialize the sonar
    sonar.init();

    //Publish opencv image of sonar
    image_transport::ImageTransport it(n);
    image_transport::Publisher image_pub = it.advertise("sonar_image", 1);
    image_transport::Publisher image_pub_colored = it.advertise("sonar_image_colored", 1);
    ros::Publisher scan_pub = n.advertise<sensor_msgs::LaserScan>("scan", 1);

    //Subscribe to range commands
    ros::Subscriber min_range_sub = nh.subscribe("sonar_min_range", 1, MinRangeCallback);
    ros::Subscriber max_range_sub = nh.subscribe("sonar_max_range", 1, MaxRangeCallback);

    tf::TransformBroadcaster tf_broadcaster;


    // cv bridge static settings:
    cv_bridge::CvImage cvi;
    cvi.header.frame_id = "image";
    cvi.encoding = "mono16";
    cv_bridge::CvImage cvic;
    cvic.header.frame_id = "image_colored";
    cvic.encoding = "bgra8"; // sonar image is four channels

    // Image sensor message
    sensor_msgs::Image msg_img, msg_img_colored;
    sensor_msgs::LaserScan msg_laser;
    msg_laser.header.frame_id = "bv_rangedata";

    ros::Time current_time;
    ros::Rate rate(tick_rate);

    cv::Mat img, imgColored;
    std::vector<double> ranges;
    int status;
    while (ros::ok())
    {
        status = sonar.getNextSonarData();
        status = sonar.getSonarImage(img);
        status = sonar.getSonarColoredImage(imgColored);
        status = sonar.getSonarScan(ranges);
        if (status == Sonar::Success)
        {
            try
            {
                current_time = ros::Time::now();

                // convert OpenCV image to ROS message
                cvi.header.stamp = current_time;
                cvi.image = img;
                cvi.toImageMsg(msg_img);

                cvic.header.stamp = current_time;
                cvic.image = imgColored;
                cvic.toImageMsg(msg_img_colored);

                // Publish the image
                image_pub.publish(msg_img);
                image_pub_colored.publish(msg_img_colored);

                //Publish Range Data
                msg_laser.header.stamp = current_time;
                msg_laser.angle_min = sonar.getBearingMinAngle()*M_PI/180.;
                msg_laser.angle_max = sonar.getBearingMaxAngle()*M_PI/180.;
                msg_laser.angle_increment = sonar.getBearingResolution()*M_PI/180.;
                msg_laser.range_min = sonar.getRangeMin();
                msg_laser.range_max = sonar.getRangeMax();

                msg_laser.ranges.clear();
                msg_laser.ranges.insert(msg_laser.ranges.begin(), ranges.begin(),ranges.end());
                scan_pub.publish(msg_laser);

                geometry_msgs::TransformStamped range_trans;
                range_trans.header.stamp = current_time;
                range_trans.header.frame_id = "bv_rangedata";
                range_trans.child_frame_id = "base_link";

                range_trans.transform.translation.x = 0;
                range_trans.transform.translation.y = 0;
                range_trans.transform.translation.z = 0.0;
                range_trans.transform.rotation = tf::createQuaternionMsgFromYaw(0);
                tf_broadcaster.sendTransform(range_trans);

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
