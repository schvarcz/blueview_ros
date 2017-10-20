#include <blueview_ros/Sonar.h>
#include <iostream>
#include <stdio.h>
#include <unistd.h>
#include <fstream>
#define ENABLE_SONAR 1

using std::cout;
using std::endl;

Sonar::Sonar()
    : initialized_(false), addr_(""), logging_(false),
      mode_(Sonar::net), data_mode_(Sonar::image), min_range_(0),
      max_range_(40), color_map_(""), save_directory_("./")
{
}

Sonar::~Sonar()
{
#if ENABLE_SONAR == 1

#if (BVTSDK_VERSION >= 4)
    if (head_)
        BVTHead_Destroy(head_);
#endif // (BVTSDK_VERSION >= 4)

    if (son_)
        BVTSonar_Destroy(son_);

    // Close logging file
    this->setSonarLogEnable(false);
#endif
}

Sonar::Status_t Sonar::init()
{
#if ENABLE_SONAR == 1
    logging_ = false;
    cur_ping_ = 0;

    son_ = BVTSonar_Create();
    if ( son_ == NULL )
    {
        printf("BVTSonar_Create: failed\n");
        return Sonar::Failure;
    }

    int ret;
    if (mode_ == Sonar::net)
    {
        /////////////////////////////////////////
        // Reading from physical sonar
        /////////////////////////////////////////

        // If the ip address string is set, try to manually connect
        // to sonar first.
        bool manual_sonar_found = false;
        if (addr_ != "" && addr_ != "0.0.0.0")
        {
            ret = BVTSonar_Open(son_, "NET", addr_.c_str());
            if( ret != 0 )
                printf("Couldn't find sonar at defined IP address: %s", addr_.c_str());
            else
                manual_sonar_found = true;
        }

#if (BVTSDK_VERSION >= 4)
         if (!manual_sonar_found) {
              //Create the discovery agent
              BVTSonarDiscoveryAgent agent = BVTSonarDiscoveryAgent_Create();
              if( agent == NULL ) {
                   printf("BVTSonarDiscoverAgent_Create: failed\n");
                   return Sonar::Failure;
              }

              // Kick off the discovery process
              ret = BVTSonarDiscoveryAgent_Start(agent);

              //Let the discovery process run for a short while (5 secs)
              cout << "Searching for available sonars..." << endl;
              sleep(5);

              // See what we found
              int numSonars = 0;
              numSonars = BVTSonarDiscoveryAgent_GetSonarCount(agent);

              char SonarIPAddress[20];

              for(int i = 0; i < numSonars; i++) {
                   ret = BVTSonarDiscoveryAgent_GetSonarInfo(agent, i, &SonarIPAddress[0], 20);
                   printf("Found Sonar: %d, IP address: %s\n", i, SonarIPAddress);
              }

              if(numSonars == 0) {
                   printf("No Sonars Found\n");
                   return Sonar::Failure;
              }

              // Open the sonar
              //ret = BVTSonar_Open(son_, "NET", "192.168.1.45");
              ret = BVTSonar_Open(son_, "NET", SonarIPAddress);
              if( ret != 0 ) {
                   printf("BVTSonar_Open: ret=%d\n", ret);
                   return Sonar::Failure;
              }
         }
#endif // (BVTSDK_VERSION >= 4)

    }
    else
    {
        /////////////////////////////////////////
        // Reading from sonar file
        /////////////////////////////////////////

        // Open the sonar
        ret = BVTSonar_Open(son_, "FILE", addr_.c_str());
        if (ret != 0 )
        {
            printf("BVTSonar_Open: ret=%d\n", ret);
            return Sonar::Failure;
        }
    }

    // Make sure we have the right number of heads_
    heads_ = -1;
    heads_ = BVTSonar_GetHeadCount(son_);
    printf("BVTSonar_GetHeadCount: %d\n", heads_);

    // Get the first head
    head_ = NULL;
    ret = BVTSonar_GetHead(son_, 0, &head_);
    if (ret != 0 )
    {
        // Some sonar heads start at 1
        ret = BVTSonar_GetHead(son_, 1, &head_);
        if (ret != 0)
        {
            printf( "BVTSonar_GetHead: ret=%d\n", ret) ;
            return Sonar::Failure;
        }
    }

    // Check the ping count
    pings_ = -1;
    pings_ = BVTHead_GetPingCount(head_);
    printf("BVTHead_GetPingCount: %d\n", pings_);

    // Set the range window
    this->setRange(min_range_, max_range_);

    initialized_ = true;

    return Sonar::Success;
#else
    return Sonar::Failure;
#endif // (ENABLE_SONAR == 1)

}

Sonar::Status_t Sonar::setSonarLogEnable(bool enable)
{
    // Whether enable is true or false, if we enter the function here,
    // we should properly close the current file if currently logging
    if (logging_ && son_logger_)
        BVTSonar_Destroy(son_logger_);

    if (!enable)
    {
        // If logging is disabled, exit.
        logging_ = false;
        return Sonar::Success;
    }

    son_logger_ = BVTSonar_Create();
    if (son_logger_ == NULL)
    {
        printf("BVTSonar_Create: failed\n");
        return Sonar::Failure;
    }

    // Create the sonar file
    //  cur_log_file_ = save_directory_ + "/" + syllo::get_time_string() + ".son";
    cur_log_file_ = save_directory_ + "/son.son";

    int ret = BVTSonar_CreateFile(son_logger_, cur_log_file_.c_str(), son_, "");

    if (ret != 0)
    {
        printf("BVTSonar_CreateFile: ret=%d\n", ret);
        return Sonar::Failure;
    }

    // Get the first head of the file output
    out_head_ = NULL ;
    ret = BVTSonar_GetHead(son_logger_, 0, &out_head_);
    if (ret != 0)
    {
        printf("BVTSonar_GetHead: ret=%d\n" ,ret);
        return Sonar::Failure;
    }

    logging_ = true;

    return Sonar::Success;
}

Sonar::Status_t Sonar::getNextSonarData()
{
    Status_t status = Sonar::Failure;
    if (mode_ == Sonar::net)
    {
        status = getSonarData(-1);
    }
    else if (cur_ping_ < pings_)
    {
        status = getSonarData(cur_ping_++);
    }
    else
    {
        status = Sonar::Failure;
    }
    return status;
}

Sonar::Status_t Sonar::getSonarData(int index)
{
#if ENABLE_SONAR == 1

    if (!initialized_)
    {
        cout << "Sonar wasn't initialized." << endl;
        return Sonar::Failure;
    }

    if(ping_)
        BVTPing_Destroy(ping_);

    int ret = BVTHead_GetPing(head_, index, &ping_);

    if(ret != 0)
    {
        printf("BVTHead_GetPing: ret=%d\n", ret);
        return Sonar::Failure;
    }

    // Logging is enabled, write to file
    if (logging_)
    {
        ret = BVTHead_PutPing(out_head_, ping_);
        if (ret != 0)
        {
            printf("BVTHead_PutPing: ret=%d\n", ret);
            return Sonar::Failure;
        }
    }

    return Sonar::Success;
#else
    return Sonar::Failure;
#endif
}

Sonar::Status_t Sonar::getSonarImage(cv::Mat &image)
{
    BVTMagImage img;

    int ret;
#if (BVTSDK_VERSION >= 4)
    BVTImageGenerator imager = BVTImageGenerator_Create();
    BVTImageGenerator_SetHead(imager, head_);
    ret = BVTImageGenerator_GetImageXY(imager, ping_, &img);
    BVTImageGenerator_Destroy(imager);
    if (ret != 0)
    {
        printf("BVTPing_GetImage: ret=%d\n", ret);
        return Sonar::Failure;
    }
    BVTMagImage_GetHeight(img, &height_);
    BVTMagImage_GetWidth(img, &width_);
#else
    ret = BVTPing_GetImage(ping_, &img);
    if (ret != 0)
    {
        printf("BVTPing_GetImage: ret=%d\n", ret);
        return Sonar::Failure;
    }
    height_ = BVTMagImage_GetHeight(img);
    width_ = BVTMagImage_GetWidth(img);
#endif // (BVTSDK_VERSION >= 4)


    unsigned int len = height_*width_;
    unsigned short* data = new unsigned short[len];
    BVTMagImage_CopyBits(img, data, len);
    cv::Mat imgcv(height_, width_, CV_16UC1, data, width_*2);
    image = imgcv.clone();

    delete[] data;
    BVTMagImage_Destroy(img);
    return Sonar::Success;
}

Sonar::Status_t Sonar::getSonarColoredImage(cv::Mat &image_colored)
{
    BVTMagImage img;
    BVTColorImage cimg;

    // Build a color mapper
    BVTColorMapper mapper = BVTColorMapper_Create();
    if (mapper == NULL)
    {
        printf("BVTColorMapper_Create: failed\n");
        return Sonar::Failure;
    }

    // Load the bone colormap
    int ret = BVTColorMapper_Load(mapper, color_map_.c_str());
    if(ret != 0)
    {
        if (color_map_ == "")
        {
            printf("Color map not set.\n");
        }
        printf("BVTColorMapper_Load: ret=%d\n", ret);
        return Sonar::Failure;
    }

#if (BVTSDK_VERSION >= 4)
    BVTImageGenerator imager = BVTImageGenerator_Create();
    BVTImageGenerator_SetHead(imager, head_);
    ret = BVTImageGenerator_GetImageXY(imager, ping_, &img);
    BVTImageGenerator_Destroy(imager);
    if (ret != 0)
    {
        printf("BVTPing_GetImage: ret=%d\n", ret);
        return Sonar::Failure;
    }
    BVTMagImage_GetHeight(img, &height_);
    BVTMagImage_GetWidth(img, &width_);
#else
    ret = BVTPing_GetImage(ping_, &img);
    if (ret != 0)
    {
        printf("BVTPing_GetImage: ret=%d\n", ret);
        return Sonar::Failure;
    }
    height_ = BVTMagImage_GetHeight(img);
    width_ = BVTMagImage_GetWidth(img);
#endif // (BVTSDK_VERSION >= 4)

    // Perform the colormapping
    ret = BVTColorMapper_MapImage(mapper, img, &cimg);
    if (ret != 0)
    {
        printf("BVTColorMapper_MapImage: ret=%d\n", ret);
        return Sonar::Failure;
    }

    unsigned int len = height_*width_;
    unsigned int* data = new unsigned int[len*4];
    BVTColorImage_CopyBits(cimg, data, len);
    cv::Mat imgcv(height_, width_, CV_8UC4, data, width_*4);
    image_colored = imgcv.clone();

    delete[] data;
    BVTMagImage_Destroy(img);
    BVTColorMapper_Destroy(mapper);
    BVTColorImage_Destroy(cimg);
    return Sonar::Success;
}

Sonar::Status_t Sonar::getSonarScan(std::vector<double> &ranges)
{
    Status_t status = Sonar::Failure;
    if(cur_ping_> 0) //For the real BlueView is not going to work
    {
        status = Sonar::Success;

        BVTPing_GetRangeData( ping_, range_ );
        ranges.resize(range_.GetCount());
        for(int j=0; j<range_.GetCount(); j++)
            ranges[j] = range_.GetRangeValue(j);
    }

    return status;
}


int Sonar::reset()
{
    cur_ping_ = 0;
    return cur_ping_;
}

int Sonar::getNumPings()
{
    return pings_;
}

int Sonar::getCurrentPingNum()
{
    return cur_ping_;
}

void Sonar::setPingNum(int num)
{
    cur_ping_ = num;
}

double Sonar::getBearingMinAngle()
{
    BVTPing_GetRangeData( ping_, range_ );
    return range_.GetFOVMinAngle();
}

double Sonar::getBearingMaxAngle()
{
    BVTPing_GetRangeData( ping_, range_ );
    return range_.GetFOVMaxAngle();
}

double Sonar::getBearingResolution()
{
    BVTPing_GetRangeData( ping_, range_ );
    return range_.GetBearingResolution();
}

double Sonar::getRangeMin()
{
    return BVTHead_GetMinimumRange(head_);
}

void Sonar::setMinRange(double min_range)
{
    this->setRange(min_range, max_range_);
}

double Sonar::getRangeMax()
{
    return BVTHead_GetMaximumRange(head_);
}

void Sonar::setMaxRange(double max_range)
{
    this->setRange(min_range_, max_range);
}

void Sonar::setRange(double min_range, double max_range)
{
    min_range_ = min_range;
    max_range_ = max_range;

    if (min_range_ < 0 || min_range_ > max_range_ )
        min_range = 0;

    if (max_range_ < 0 || max_range_ <= min_range_+1)
        max_range_ = min_range_ + 2;

    BVTHead_SetRange(head_, min_range_, max_range_);
}

double Sonar::getRangeResolution()
{
    BVTPing_GetRangeData( ping_, range_ );
    return range_.GetRangeResolution();
}

Sonar::SonarMode_t Sonar::getMode()
{
    return mode_;
}

void Sonar::setMode(SonarMode_t mode)
{
    mode_ = mode;
}

Sonar::DataMode_t Sonar::getDataMode()
{
    return data_mode_;
}

void Sonar::setDataMode(DataMode_t data_mode)
{
    data_mode_ = data_mode;
}

std::string Sonar::getAddress()
{
    return addr_;
}

void Sonar::setAddress(const std::string &address)
{
    addr_ = address;
}

std::string Sonar::getColorMap()
{
    return color_map_;
}

void Sonar::setColorMap(const std::string &color_map)
{
    color_map_ = color_map;
}

std::string Sonar::getSaveDirectory()
{
    return save_directory_;
}

void Sonar::setSaveDirectory(const std::string &save_directory)
{
    save_directory_ = save_directory;
}

const std::string& Sonar::getCurrentSonarFileName()
{
    return cur_log_file_;
}

int Sonar::width()
{
    return width_;
}

int Sonar::height()
{
    return height_;
}
