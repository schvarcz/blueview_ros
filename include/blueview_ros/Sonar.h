#ifndef _SONAR_H_
#define _SONAR_H_

#define ENABLE_SONAR 1

#include <opencv2/opencv.hpp>

#if ENABLE_SONAR == 1
#include <bvt_sdk.h>
#endif

class Sonar {
public:
     typedef enum SonarMode{
          net = 0,
          sonar_file
     }SonarMode_t;

     typedef enum DataMode{
          image = 0,
          range
     }DataMode_t;

     typedef enum Status{
          Success = 0,
          Failure
     }Status_t;

     Sonar();
     ~Sonar();
     Status_t init();

     int getNumPings();
     int getCurrentPingNum();

     double getBearingMinAngle();
     double getBearingMaxAngle();
     double getBearingResolution();
     double getRangeMin();
     double getRangeMax();
     double getRangeResolution();


     Status_t getNextSonarData();
     Status_t getSonarData(int index);
     Status_t getSonarImage(cv::Mat &image);
     Status_t getSonarScan(std::vector<double> &ranges);
     int reset();

     void setFrameNum(int num);
     void set_mode(SonarMode_t mode);
     void set_data_mode(DataMode_t data_mode);
     void set_ip_addr(const std::string &ip_addr);
     void set_input_son_filename(const std::string &fn);
     void set_range(double min_range, double max_range);
     void set_max_range(double max_range);
     void set_min_range(double min_range);
     void set_color_map(const std::string &color_map);
     void set_save_directory(const std::string &save_directory);

     Status_t SonarLogEnable(bool enable);
     const std::string& current_sonar_file();


     int height();
     int width();

protected:
     bool initialized_;
     std::string fn_;
     std::string ip_addr_;
     bool logging_;

     SonarMode_t mode_;
     DataMode_t data_mode_;

     std::string cur_log_file_;
     std::string save_directory_;


#if ENABLE_SONAR == 1
     BVTHead head_;
     BVTSonar son_;
     BVTPing ping_;
     BVTSDK::RangeData range_;

     std::string color_map_;

     // Sonar file save / logger members
     BVTSonar son_logger_;
     BVTHead out_head_;
     std::vector<double> ranges_;

#endif

     int heads_;
     int pings_;

     int cur_ping_;

     double min_range_;
     double max_range_;

     int height_;
     int width_;
};

#endif
