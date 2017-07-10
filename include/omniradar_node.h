#include <mutex>
#include <thread>

#include <ros/ros.h>

#include <omniradar.h>
#include <omniradar/RadarEcho.h>
#include <omniradar/omniradarConfig.h>


class OmniradarNode
{
public:
    OmniradarNode();
    ~OmniradarNode();
    
    void dynamic_reconfigure_callback_function (omniradar::omniradarConfig &config, uint32_t level);
    
private:
    Omniradar *rdk = nullptr;
    std::mutex mtx_rdk;
    omniradar::RadarEcho msg;
    std::mutex mtx_msg;
    
    void build_and_publish_msg(std::vector< std::vector<uint8_t> > &echo);
};
