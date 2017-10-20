#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <blueview_ros/Sonar.h>
#include <iostream>
#include <fstream>
#include <sstream>

#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/LaserScan.h>

#include <boost/filesystem.hpp>

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

//// Open camera for recording
cv::VideoWriter record_;
bool record_initialized_ = false;
std::string video_filename_ = "";
bool logging_enabled_ = false;
cv_bridge::CvImagePtr cv_img_ptr_;
bool cv_ptr_valid_ = false;

std::string notes_filename_ = "";
std::fstream notes_file_;
std::string log_filename_ = "";
std::fstream log_file_;
int log_frame_num_ = 0;

void videoCallback(const sensor_msgs::ImageConstPtr &msg)
{
    cv_img_ptr_ = cv_bridge::toCvCopy(msg, "bgr8");
    cv_ptr_valid_ = true;
}

void EnableSonarLoggingCallback(const std_msgs::Bool::ConstPtr& msg)
{
    logging_enabled_ = msg->data;

    sonar.setSonarLogEnable(msg->data);
    if (logging_enabled_)
    {
        // Generate the avi file name
        video_filename_ = sonar.getCurrentSonarFileName() + ".avi";
        log_filename_ = sonar.getCurrentSonarFileName() + ".txt";
        notes_filename_ = sonar.getCurrentSonarFileName() + ".notes";
        log_frame_num_ = 0;
    }
    else
    {
        log_file_.close();
        notes_file_.close();
    }

    record_initialized_ = false;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bluview_ros_node");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    //Publish opencv image of sonar
    image_transport::ImageTransport it(n);
    image_transport::Publisher image_pub = it.advertise("sonar_image", 1);
    ros::Publisher scan_pub = n.advertise<sensor_msgs::LaserScan>("scan", 1);

    ///////////////////////////////////////////////////
    // Acquire params from paramserver
    ///////////////////////////////////////////////////
    // Grab minimum distance
    double min_dist=-2;
    nh.getParam("min_dist", min_dist);

    // Grab maximim distance
    double max_dist;
    nh.getParam("max_dist", max_dist);

    // Set sonar range
    sonar.setRange(min_dist, max_dist);

    // Grab mode (image or range)
    std::string mode;
    nh.getParam("mode", mode);
    if (mode == "range")
    {
        sonar.setDataMode(Sonar::range);
    }
    else
    {
        sonar.setDataMode(Sonar::image);
    }

    // Grab color map filename
    std::string color_map;
    nh.getParam("color_map", color_map);
    sonar.setColorMap(color_map);

    //  // Grab sonar save directory
    // //  std::string save_directory;
    // //  node.get_param("~save_directory", save_directory);
    // //  sonar.set_save_directory(save_directory);
    //
    //
    // //  // Create the save_directory if it doesn't exist
    // //  boost::filesystem::path dir(save_directory);
    // //  if( !(boost::filesystem::exists(dir))) {
    // //       if(boost::filesystem::create_directory(dir)) {
    // //            std::cout << "Created new sonar save directory: " <<
    // //                 save_directory << endl;
    // //       } else {
    // //            cout << "Failed to create directory: " << save_directory << endl;
    // //       }
    // //  }
    //
    // Determine if a live "net" sonar will be used or if we are reading
    // from a file
    std::string net_or_file;
    // Grab sonar file name
    std::string sonar_file;
    // Grab suggested ip address
    std::string ip_addr;
    nh.getParam("net_or_file", net_or_file);

    if (net_or_file == "net")
    {
        nh.getParam("ip_addr", ip_addr);
        sonar.setMode(Sonar::net);
        sonar.setIpAddr(ip_addr);
    }
    else
    {
        nh.getParam("sonar_file", sonar_file);
        sonar.setMode(Sonar::sonar_file);
        sonar.setInputSonFilename(sonar_file);
    }

    // Initialize the sonar
    sonar.init();

    //Subscribe to range commands
    ros::Subscriber min_range_sub = nh.subscribe("sonar_min_range", 1, MinRangeCallback);
    ros::Subscriber max_range_sub = nh.subscribe("sonar_max_range", 1, MaxRangeCallback);

    cout << "min_dist: " << min_dist << endl;
    cout << "max_dist: " << max_dist << endl;
    cout << "color_map: " << color_map << endl;
    cout << "sonar_file: " << sonar_file << endl;
    //  ros::Subscriber enable_log_sub = nh_.subscribe("sonar_enable_log", 1,
    //                                                EnableSonarLoggingCallback);
    //
    //

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
    // //
    // //  cv::Mat temp;
    // //  sonar.getNextSonarImage(temp);
    // //
    // //  //cv::VideoWriter record_;
    // //  //record_.open("/home/videoray/sonar_log/video.avi", CV_FOURCC('D','I','V','X'), 10, temp.size(), true);
    // //
    ros::Time ros_time;

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
    }

    return 0;
}
