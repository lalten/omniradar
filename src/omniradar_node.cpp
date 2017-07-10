#include <omniradar/RadarEcho.h>
#include <string>
#include <memory>
#include <pthread.h> // set thread names

// #include <chrono>

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

#include <omniradar_node.h>
#include <omniradar/RadarEcho.h>

#include "vco_tune.h"


OmniradarNode::OmniradarNode()
{
    msg.n_sweeps = 1;
    msg.t_sweep = 5e-3;
    msg.ric_config = "06-FE-00-69-60-7A-C0-00-00-00-00-2C-00-00-00-00-01-DC-64";

    try
    {
        // Set up ROS.
        dynamic_reconfigure::Server<omniradar::omniradarConfig> dynamic_reconfigure_server;
        dynamic_reconfigure::Server<omniradar::omniradarConfig>::CallbackType dynamic_reconfigure_callback;
        dynamic_reconfigure_callback = boost::bind(&OmniradarNode::dynamic_reconfigure_callback_function, this, _1, _2);
        dynamic_reconfigure_server.setCallback(dynamic_reconfigure_callback);

        ros::NodeHandle nh("~");

        // Create a publisher and name the topic.
        ros::Publisher pub = nh.advertise<omniradar::RadarEcho>("radar_raw", 10);

        // Set up Radar
        {
            std::lock_guard<std::mutex> lock(mtx_rdk);
            rdk = new Omniradar();
            rdk->ConfigureRadar(msg.ric_config);
            rdk->setVCOTune(vco_tune);
            rdk->setSweepTime(msg.t_sweep);
        }
        
        ROS_INFO_STREAM("RDK ready");

        while(nh.ok())
        {
            try
            {
                std::lock_guard<std::mutex> lock_rdk(mtx_rdk);
                auto t_echo = ros::Time::now();
                auto p_echo = rdk->AcquireEcho(msg.n_sweeps);

                std::thread t (
                    [=] ()
                    {
                        std::lock_guard<std::mutex> lock_msg(mtx_msg);

                        // Name this thread for debugging
                        std::string thread_name = "Pub msg #";
                        thread_name.append(std::to_string(msg.header.seq));
                        pthread_setname_np(pthread_self(), thread_name.c_str());
                        
                        msg.header.stamp = t_echo;
                        msg.packed_echo.resize(p_echo->size());
                        std::copy(p_echo->begin(), p_echo->end(), msg.packed_echo.begin());
                        pub.publish(msg);
                        msg.header.seq++;
                    }
                );
                t.detach();

            }
            catch (OmniradarException ex)
            {
                if(std::string(ex.what()).find("RDK_NO_RESPONSE") != std::string::npos)
                {
                    ROS_WARN_STREAM("RDK_NO_RESPONSE");
                    continue;
                }
                else
                {
                    throw;
                }
            }

            ros::spinOnce();

        }
    }
    catch(OmniradarException ex)
    {
        ROS_ERROR_STREAM("Omniradar exception " << ex.what());
        throw;
    }
}

OmniradarNode::~OmniradarNode()
{
    delete rdk;
}


void OmniradarNode::dynamic_reconfigure_callback_function (omniradar::omniradarConfig &config, uint32_t level)
{
    if(!rdk)
        return;
    
    std::lock_guard<std::mutex> lock(mtx_rdk);
    rdk->ConfigureRadar(config.config_str);
//     rdk->setVCOTune(vco_tune);
    rdk->setSweepTime(config.t_sweep / 1000.0);
    
    std::lock_guard<std::mutex> lock_msg(mtx_msg);
    msg.ric_config = config.config_str;
    msg.t_sweep = config.t_sweep / 1000.0;
    msg.n_sweeps = config.n_sweeps;
    
    ROS_INFO_STREAM("Set sweep to " << config.n_sweeps << " x " << config.t_sweep << "ms");
    
}
