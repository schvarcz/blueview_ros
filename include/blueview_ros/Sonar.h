#ifndef _SONAR_H_
#define _SONAR_H_

#define ENABLE_SONAR 1

#include <opencv2/opencv.hpp>

#if ENABLE_SONAR == 1
#include <bvt_sdk.h>
#endif

class Sonar
{
public:
    typedef enum SonarMode
    {
        net = 0,
        sonar_file
    } SonarMode_t;

    typedef enum DataMode
    {
        image = 0,
        range
    } DataMode_t;

    typedef enum Status
    {
        Success = 0,
        Failure
    } Status_t;

    Sonar();
    ~Sonar();
    Status_t init();

    Status_t getNextSonarData();
    Status_t getSonarData(int index);
    Status_t getSonarImage(cv::Mat &image);
    Status_t getSonarScan(std::vector<double> &ranges);

    int reset();
    int getNumPings();
    int getCurrentPingNum();
    void setPingNum(int num);

    double getBearingMinAngle();
    double getBearingMaxAngle();
    double getBearingResolution();

    double getRangeMin();
    void setMinRange(double min_range);
    double getRangeMax();
    void setMaxRange(double max_range);
    void setRange(double min_range, double max_range);
    double getRangeResolution();

    SonarMode_t getMode();
    void setMode(SonarMode_t mode);

    DataMode_t getDataMode();
    void setDataMode(DataMode_t data_mode);

    std::string getAddress();
    void setAddress(const std::string &address);

    std::string getColorMap();
    void setColorMap(const std::string &color_map);

    std::string getSaveDirectory();
    void setSaveDirectory(const std::string &save_directory);

    Status_t setSonarLogEnable(bool enable);
    const std::string& getCurrentSonarFileName();

    int height();
    int width();

protected:
    bool initialized_;
    std::string addr_;
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
