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

void ThresRangeCallback(const std_msgs::Float32::ConstPtr& msg)
{
    sonar.setThresholdRangeData(msg->data);
}

sensor_msgs::PointCloud cv2pointCloud(cv::Mat image)
{
    sensor_msgs::PointCloud pc_msg;
    sensor_msgs::ChannelFloat32 ch_msg;
    ch_msg.name = "intensity";

    cv::Mat img, gray, pts;
    image.convertTo(img, CV_32F);
    cv::threshold(img, gray, sonar.getThresholdRangeData(), 255, cv::THRESH_BINARY);
    gray.convertTo(gray, CV_8U);
    cv::findNonZero(gray, pts);

    cv::Point2i sonarCenter(sonar.width()/2,sonar.height());
    double curDist, curBearing;
    cv::Point curPoint;
    geometry_msgs::Point32 point;
    for(int i = 0; i< pts.rows; i++)
    {
        curPoint = pts.at<cv::Point>(i);

        curDist = sqrt(pow(curPoint.x - sonarCenter.x ,2) + pow(sonar.height() - curPoint.y,2));
        curBearing = asin((curPoint.x - sonarCenter.x)/curDist);
        curDist *= sonar.getRangeResolution();

        point.x = curDist*cos(curBearing);
        point.y = curDist*sin(curBearing);
        point.z = 0;
        pc_msg.points.push_back(point);

        ch_msg.values.push_back(image.at<short>(curPoint));
    }

    pc_msg.channels.push_back(ch_msg);

    return pc_msg;
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
            max_dist = 40, tick_rate = 10, threshold = 1000;
    std::string color_map,
            net_or_file, address;
    bool enable_range_data = false, enable_gray_image = false, enable_colored_image = false, enable_point_cloud = false;

    // Grab distance range
    nh.getParam("min_dist", min_dist);
    nh.getParam("max_dist", max_dist);

    nh.getParam("tick_rate", tick_rate);
    nh.getParam("threshold", threshold);

    // Grab mode (image or range)
    nh.getParam("enable_range_data", enable_range_data);
    nh.getParam("enable_gray_image", enable_gray_image);
    nh.getParam("enable_colored_image", enable_colored_image);
    nh.getParam("enable_point_cloud", enable_point_cloud);

    // Grab color map filename
    nh.getParam("color_map", color_map);

    nh.getParam("net_or_file", net_or_file);
    nh.getParam("address", address);

    // Determine if a live "net" sonar will be used or if we are reading from a file
    if (net_or_file == "net")
        sonar.setMode(Sonar::net);
    else
        sonar.setMode(Sonar::sonar_file);

    sonar.setAddress(address);

    sonar.setRange(min_dist, max_dist);

    sonar.setColorMap(color_map);
    sonar.setThresholdRangeData(threshold);

    // Initialize the sonar
    sonar.init();

    //Publish opencv image of sonar
    image_transport::ImageTransport it(n);
    image_transport::Publisher image_pub, image_pub_colored;
    ros::Publisher scan_pub, pc_pub;
    if (enable_gray_image)
        image_pub = it.advertise("sonar_image", 1);
    if (enable_colored_image)
        image_pub_colored = it.advertise("sonar_image_colored", 1);
    if (enable_range_data)
        scan_pub = n.advertise<sensor_msgs::LaserScan>("scan", 1);
    if (enable_point_cloud)
        pc_pub = n.advertise<sensor_msgs::PointCloud>("point_cloud", 1);

    //Subscribe to range commands
    ros::Subscriber min_range_sub = nh.subscribe("sonar_min_range", 1, MinRangeCallback);
    ros::Subscriber max_range_sub = nh.subscribe("sonar_max_range", 1, MaxRangeCallback);
    ros::Subscriber thres_range_sub = nh.subscribe("threshold_range_data", 1, ThresRangeCallback);

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
    sensor_msgs::PointCloud pc_msg;
    msg_laser.header.frame_id = "bv_rangedata";
    pc_msg.header.frame_id = "bv_pointcloud";

    ros::Time current_time;
    ros::Rate rate(tick_rate);

    cv::Mat img, imgColored;
    std::vector<double> ranges;
    int status;
    while (ros::ok())
    {
        status = sonar.getNextSonarData();

        if (enable_gray_image || enable_point_cloud)
            status = sonar.getSonarImage(img);

        if (enable_colored_image)
            status = sonar.getSonarColoredImage(imgColored);

        if (enable_range_data)
            status = sonar.getSonarScan(ranges);

        if (status == Sonar::Success)
        {
            try
            {
                current_time = ros::Time::now();

                // Publish the images
                if (enable_gray_image)
                {
                    cvi.header.stamp = current_time;
                    cvi.image = img;
                    cvi.toImageMsg(msg_img);
                    image_pub.publish(msg_img);
                }

                if (enable_colored_image)
                {
                    cvic.header.stamp = current_time;
                    cvic.image = imgColored;
                    cvic.toImageMsg(msg_img_colored);
                    image_pub_colored.publish(msg_img_colored);
                }

                //Publish Range Data
                if (enable_range_data)
                {
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
                    range_trans.header.frame_id = "base_link";
                    range_trans.child_frame_id = "bv_rangedata";

                    range_trans.transform.translation.x = 0.0;
                    range_trans.transform.translation.y = 0.0;
                    range_trans.transform.translation.z = 0.0;
                    range_trans.transform.rotation = tf::createQuaternionMsgFromYaw(0);
                    tf_broadcaster.sendTransform(range_trans);
                }

                //Publish Point Cloud
                if (enable_point_cloud)
                {
                    pc_msg = cv2pointCloud(img);
                    pc_msg.header.stamp = current_time;
                    pc_msg.header.frame_id = "bv_pointcloud";
                    pc_pub.publish(pc_msg);

                    geometry_msgs::TransformStamped pc_trans;
                    pc_trans.header.stamp = current_time;
                    pc_trans.header.frame_id = "base_link";
                    pc_trans.child_frame_id = "bv_pointcloud";

                    pc_trans.transform.translation.x = 0.0;
                    pc_trans.transform.translation.y = 0.0;
                    pc_trans.transform.translation.z = 0.0;
                    pc_trans.transform.rotation = tf::createQuaternionMsgFromYaw(0);
                    tf_broadcaster.sendTransform(pc_trans);
                }

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
