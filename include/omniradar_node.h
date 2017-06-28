#include <mutex>

#include <ros/ros.h>

#include <omniradar.h>

#include <omniradar/omniradarConfig.h>


class OmniradarNode
{
public:
    OmniradarNode();
    ~OmniradarNode();
    
    void dynamic_reconfigure_callback_function (omniradar::omniradarConfig &config, uint32_t level);
    
private:
    Omniradar *rdk = nullptr;
    int num_sweeps = 2;
    std::mutex mtx_rdk;
};
