#include <blueview_ros/Sonar.h>

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

#include <math.h>
#include "opencv2/opencv.hpp"

using std::cout;
using std::endl;

//Create a Sonar
blueview::Sonar sonar;

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
        point.y = -curDist*sin(curBearing);
        point.z = 0;
        pc_msg.points.push_back(point);

        ch_msg.values.push_back(image.at<short>(curPoint));
    }

    pc_msg.channels.push_back(ch_msg);

    return pc_msg;
}


void imageFilterInit()
{
  cv::namedWindow("control");
  int value = 1;
  cv::createTrackbar( "Surface", "control", &value, 10 );
  value = 8;
  cv::createTrackbar( "Density", "control", &value, 100);
  value = 20;
  cv::createTrackbar( "DensityDistance", "control", &value, 100);
  value = 5;
  cv::createTrackbar( "MedianWindowSize", "control", &value, 30);
  value = 1;
  cv::createTrackbar( "MedianThreshold", "control", &value, 100);
  value = sonar.getThresholdRangeData();
  cv::createTrackbar("Threshold", "control", &value, 30000 );
  cv::waitKey(33);

  cv::moveWindow("control", 2500, 400);
}

void imageFilter(cv::Mat img)
{
  cv::Mat mask;
  img.convertTo(img, CV_32F);

  double surface = cv::getTrackbarPos( "Surface", "control");
  double threshold = cv::getTrackbarPos( "Threshold", "control");

  double minn, maxx;
  cv::minMaxIdx(img, &minn, &maxx);
  cv::threshold(img, mask, threshold, 0, cv::THRESH_TOZERO);
  cv::threshold(mask, mask, 0, 1, cv::THRESH_BINARY);

  cv::circle(mask, cv::Point(img.cols/2,img.rows),surface/sonar.getRangeResolution(), cv::Scalar(0), -1);



  cout << "Min: " << minn << " Max: " << maxx << endl;
  cout << "Resolution: " << sonar.getRangeResolution() << endl;
  cout << "ThresholdRangeData: " << threshold << endl;

  img = (img-minn)/(maxx-minn);

  cv::imshow("img", img);
  cv::imshow("filtered", img.mul(mask));
  cv::imshow("filteredmask", mask);
  cv::waitKey(33);
}

void densityFilter(cv::Mat img, sensor_msgs::LaserScan msg_laser)
{
    cv::Mat ptsImg = cv::Mat::zeros(img.rows, img.cols, CV_8U);

    for(int i =0; i< msg_laser.ranges.size(); i++)
    {
        double range = msg_laser.ranges.at(i)/sonar.getRangeResolution();
        double angle = msg_laser.angle_min + i*msg_laser.angle_increment;

        double dx = range*sin(angle), dy = range*cos(angle);

        ptsImg.at<uchar>(cv::Point(img.cols/2 + dx,img.rows - dy)) = 1;

    }

    int kernel_size = 9;
    cv::Mat kernel = cv::Mat::ones( kernel_size, kernel_size, CV_32F );

    /// Apply filter
    cv::filter2D(ptsImg, ptsImg, -1 , kernel);
    cv::threshold(ptsImg, ptsImg, 8, 255, cv::THRESH_BINARY);
    cv::imshow("Density", ptsImg);
    cv::waitKey(33);
}

void densityFilter2(sensor_msgs::LaserScan &msg_laser)
{
  int densityMax = cv::getTrackbarPos( "Density", "control");
  double densityMaxDist = ((double)cv::getTrackbarPos( "DensityDistance", "control")/10);
  for(int i =0; i< msg_laser.ranges.size(); i++)
  {
    double angle = msg_laser.angle_min + i*msg_laser.angle_increment;
    cv::Point compPt(msg_laser.ranges.at(i)*sin(angle), msg_laser.ranges.at(i)*cos(angle));
    int density = 0;
    for(int j =0; j< msg_laser.ranges.size(); j++)
    {
      angle = msg_laser.angle_min + j*msg_laser.angle_increment;
      cv::Point curPt(msg_laser.ranges.at(j)*sin(angle), msg_laser.ranges.at(j)*cos(angle));

      if (sqrt(pow(curPt.x - compPt.x,2) + pow(curPt.y - compPt.y,2)) < densityMaxDist)
        density++;
    }

    if (density < densityMax)
      msg_laser.ranges[i] = msg_laser.range_max;
  }
}

void medianFilter(sensor_msgs::LaserScan &msg_laser)
{
  int halfWindowSize = (int)cv::getTrackbarPos( "MedianWindowSize", "control")/2;
  double medianThreshold = ((double)cv::getTrackbarPos( "MedianThreshold", "control"))/10.;
  for(int i =0; i< msg_laser.ranges.size(); i++)
  {
    std::vector<double> values;
    for(int j=std::max(0,i-halfWindowSize); j<std::min((int)msg_laser.ranges.size(),i+halfWindowSize); j++)
      if(msg_laser.ranges[j] != 1000)
        values.push_back(msg_laser.ranges.at(j));

    std::sort(values.begin(), values.end());

    double median = values[(int)values.size()/2];

    if ( abs(median-msg_laser.ranges.at(i)) > medianThreshold)
      msg_laser.ranges[i] = 1000; //msg_laser.range_max;
  }
}

void surfaceFilter(sensor_msgs::LaserScan &msg_laser)
{
  double surface = cv::getTrackbarPos( "Surface", "control");
  for(int i =0; i< msg_laser.ranges.size(); i++)
  {
    if (msg_laser.ranges.at(i) < surface)
    {
      msg_laser.ranges.at(i) = 1000; //msg_laser->range_max;
    }
  }
}

cv::Mat drawScan(cv::Mat &img, sensor_msgs::LaserScan msg_laser, cv::Scalar color)
{
    for(int i =0; i< msg_laser.ranges.size(); i++)
    {
        double range = msg_laser.ranges.at(i)/sonar.getRangeResolution();
        double angle = msg_laser.angle_min + i*msg_laser.angle_increment;

        double dx = range*sin(angle), dy = range*cos(angle);

        cv::circle(img, cv::Point(img.cols/2 + dx,img.rows - dy), 5, color, -1);

    }

    return img;
}

void plotScan(cv::Mat img, sensor_msgs::LaserScan msg_laser, cv::Scalar color)
{
    img.convertTo(img, CV_32F);
    double minn, maxx;
    cv::minMaxIdx(img, &minn, &maxx);
    img = (img-minn)/(maxx-minn);
    cv::Mat colored;
    cv::cvtColor(img, colored, CV_GRAY2BGR);

    for(int i =0; i< msg_laser.ranges.size(); i++)
    {
        double range = msg_laser.ranges.at(i)/sonar.getRangeResolution();
        double angle = msg_laser.angle_min + i*msg_laser.angle_increment;

        double dx = range*sin(angle), dy = range*cos(angle);

        cv::circle(colored, cv::Point(img.cols/2 + dx,img.rows - dy), 5, color, -1);

    }

    cv::imshow("Ranges", colored);
    cv::waitKey(33);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bluview_ros_node");

    cout << "OpenCV " << CV_VERSION << endl;


    ros::NodeHandle n;
    ros::NodeHandle nh("~");
    ros::Time start_time = ros::Time::now();


    ///////////////////////////////////////////////////
    // Acquire params from paramserver
    ///////////////////////////////////////////////////
    double min_dist = 0,
            max_dist = 40, tick_rate = 10, threshold = 1000, timeDiff = 0, firstTime = 0;
    std::string color_map,
            net_or_file, address, base_link_name = "base_link";
    bool enable_range_data = false, enable_gray_image = false, enable_colored_image = false, enable_point_cloud = false;

    bool new_range_data = false, new_gray_image = false, new_colored_image = false, new_point_cloud = false;

    // Grab distance range
    nh.getParam("min_dist", min_dist);
    nh.getParam("max_dist", max_dist);

    nh.getParam("tick_rate", tick_rate);
    nh.getParam("threshold", threshold);

    nh.getParam("time_diff", timeDiff);
    nh.getParam("first_time", firstTime);

    // Grab mode (image or range)
    nh.getParam("enable_range_data", enable_range_data);
    nh.getParam("enable_gray_image", enable_gray_image);
    nh.getParam("enable_colored_image", enable_colored_image);
    nh.getParam("enable_point_cloud", enable_point_cloud);

    // Grab color map filename
    nh.getParam("color_map", color_map);

    nh.getParam("net_or_file", net_or_file);
    nh.getParam("address", address);

    nh.getParam("base_link_name", base_link_name);

    // Determine if a live "net" sonar will be used or if we are reading from a file
    if (net_or_file == "net")
        sonar.setMode(blueview::Sonar::net);
    else
        sonar.setMode(blueview::Sonar::sonar_file);

    sonar.setAddress(address);

    sonar.setRange(min_dist, max_dist);

    sonar.setColorMap(color_map);
    sonar.setThresholdRangeData(threshold);

    // Initialize the sonar
    sonar.init();
    imageFilterInit();
    //Publish opencv image of sonar
//    image_transport::ImageTransport it(n);
//    image_transport::Publisher image_pub, image_pub_colored;
    ros::Publisher scan_pub, pc_pub;
//    if (enable_gray_image)
//        image_pub = it.advertise("sonar_image", 1);
//    if (enable_colored_image)
//        image_pub_colored = it.advertise("sonar_image_colored", 1);
    if (enable_range_data)
        scan_pub = n.advertise<sensor_msgs::LaserScan>("scan", 1);
    if (enable_point_cloud)
        pc_pub = n.advertise<sensor_msgs::PointCloud>("point_cloud", 1);

    //Subscribe to range commands
    ros::Subscriber min_range_sub = nh.subscribe("sonar_min_range", 1, MinRangeCallback);
    ros::Subscriber max_range_sub = nh.subscribe("sonar_max_range", 1, MaxRangeCallback);
    ros::Subscriber thres_range_sub = nh.subscribe("threshold_range_data", 1, ThresRangeCallback);

    tf::TransformBroadcaster tf_broadcaster;

//    // cv bridge static settings:
//    cv_bridge::CvImage cvi;
//    cvi.header.frame_id = "image";
//    cvi.encoding = "mono16";
//    cv_bridge::CvImage cvic;
//    cvic.header.frame_id = "image_colored";
//    cvic.encoding = "bgra8"; // sonar image is four channels

//    // Image sensor message
//    sensor_msgs::Image msg_img, msg_img_colored;
    sensor_msgs::LaserScan msg_laser;
    sensor_msgs::PointCloud pc_msg;
    msg_laser.header.frame_id = base_link_name;
    pc_msg.header.frame_id = base_link_name;

    ros::Time current_time;
    ros::Rate rate(tick_rate);

    cv::Mat img, imgColored;
    std::vector<double> ranges;
    int status;
    double days = 0, hours = 0, minutes = 0, seconds = 0;
    while (ros::ok())
    {
        status = sonar.getNextSonarData();

        if (status == blueview::Sonar::Success)
        {
          std::cout << "New data." << endl;

          seconds = sonar.getTimeStamp();
          minutes = floor(seconds/60.);
          hours = floor(minutes/60.);
          days = floor(hours/24.);
          seconds -= minutes*60;
          minutes -= hours*60;
          hours -= days*24;

          std::cout << "TimeStamp: " << days << " " << hours << ":" << minutes << ":" << seconds << endl;
          std::cout << "TimeZone: " << sonar.getTimeZoneOffset() << endl;

          seconds = ((hours*60 + minutes)*60 + seconds);
          std::cout << "Elapsed time: " << (seconds + timeDiff) - firstTime << endl;
          if(firstTime == 0)
            firstTime = seconds;

            current_time.fromSec(start_time.toSec() + (seconds + timeDiff) - firstTime);
            current_time = ros::Time::now();
//            ros::Time::sleepUntil(current_time);

            if (enable_gray_image || enable_point_cloud)
            {
                new_gray_image = sonar.getSonarImage(img);
                new_point_cloud = new_gray_image;
            }

            if (enable_colored_image)
                new_colored_image = sonar.getSonarColoredImage(imgColored);

            if (enable_range_data)
                new_range_data = sonar.getSonarScan(ranges);

            try
            {
                // Publish the images
//                if (enable_gray_image && (new_gray_image  == blueview::Sonar::Success))
//                {
//                    cvi.header.stamp = current_time;
//                    cvi.image = img;
//                    cvi.toImageMsg(msg_img);
//                    image_pub.publish(msg_img);
//                }

//                if (enable_colored_image && (new_colored_image  == blueview::Sonar::Success))
//                {
//                    cvic.header.stamp = current_time;
//                    cvic.image = imgColored;
//                    cvic.toImageMsg(msg_img_colored);
//                    image_pub_colored.publish(msg_img_colored);
//                }

            } catch (cv_bridge::Exception& e) {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return -1;
            }

            //Publish Range Data
            if (enable_range_data && (new_range_data  == blueview::Sonar::Success))
            {
                msg_laser.header.stamp = current_time;
                msg_laser.angle_min = sonar.getBearingMinAngle()*M_PI/180.;
                msg_laser.angle_max = sonar.getBearingMaxAngle()*M_PI/180.;
                msg_laser.angle_increment = sonar.getBearingResolution()*M_PI/180.;
                msg_laser.range_min = sonar.getRangeMin();
                msg_laser.range_max = sonar.getRangeMax();

                msg_laser.ranges.clear();
                msg_laser.ranges.insert(msg_laser.ranges.begin(), ranges.rbegin(),ranges.rend());

                for(int i =0; i< msg_laser.ranges.size(); i++)
                {
//                    if(msg_laser.ranges.at(i) == 1000)
//                        msg_laser.ranges[i] = numeric_limits<float>::nan();
                }
                scan_pub.publish(msg_laser);
            }

            //Publish Point Cloud
            if (enable_point_cloud && (new_point_cloud  == blueview::Sonar::Success))
            {
                pc_msg = cv2pointCloud(img);
                pc_msg.header.stamp = current_time;
                pc_msg.header.frame_id = base_link_name;
                pc_pub.publish(pc_msg);
            }

            imageFilter(img);
            cv::Mat colored;
            img.convertTo(img, CV_32F);
            double minn, maxx;
            cv::minMaxIdx(img, &minn, &maxx);
            img = (img-minn)/(maxx-minn);
            cv::cvtColor(img, colored, CV_GRAY2BGR);
            drawScan(colored, msg_laser,cv::Scalar(0,0,255));
            densityFilter2(msg_laser);
            drawScan(colored, msg_laser,cv::Scalar(0, 255, 255));
            medianFilter(msg_laser);
            drawScan(colored, msg_laser,cv::Scalar(255,0,255));
            surfaceFilter(msg_laser);
            drawScan(colored, msg_laser,cv::Scalar(255,0,0));

            cv::imshow("Ranges", colored);
            cv::waitKey(33);
//            densityFilter(img, msg_laser);

            rate.sleep();

        }
        else
          std::cout << "No data." << endl;

        ros::spinOnce();
    }

    return 0;
}
