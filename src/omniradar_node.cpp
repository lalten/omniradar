#include <string>
// #include <chrono>

#include <ros/ros.h>
#include <std_msgs/ByteMultiArray.h>
#include <dynamic_reconfigure/server.h>

#include <omniradar_node.h>


#include "vco_tune.h"


OmniradarNode::OmniradarNode()
{
    try
    {
        // Set up ROS.
        dynamic_reconfigure::Server<omniradar::omniradarConfig> dynamic_reconfigure_server;
        dynamic_reconfigure::Server<omniradar::omniradarConfig>::CallbackType dynamic_reconfigure_callback;
        dynamic_reconfigure_callback = boost::bind(&OmniradarNode::dynamic_reconfigure_callback_function, this, _1, _2);
        dynamic_reconfigure_server.setCallback(dynamic_reconfigure_callback);

        ros::NodeHandle nh("~");

        // Create a publisher and name the topic.
        ros::Publisher pub = nh.advertise<std_msgs::ByteMultiArray>("radar_raw", 10);

        std_msgs::ByteMultiArray msg;
        msg.layout.dim.resize(2);
        msg.layout.dim[0].label = "channel";
        msg.layout.dim[0].size = 4;
        msg.layout.dim[1].label = "sample";

        // Set up Radar
        {
            std::lock_guard<std::mutex> lock(mtx_rdk);
            rdk = new Omniradar();
            rdk->ConfigureRadar("06-FE-00-69-60-7A-C0-00-00-00-00-2C-00-00-00-00-01-DC-64");
            rdk->setVCOTune(vco_tune);
            rdk->setSweepTime(20e-3);
        }
        std::vector< std::vector<uint8_t> > echo;
        
        ROS_INFO_STREAM("RDK ready");

        while(nh.ok())
        {
            try
            {
                std::lock_guard<std::mutex> lock(mtx_rdk);
                echo = rdk->AcquireEcho(num_sweeps);
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

            // copy data into message
//             auto start = std::chrono::system_clock::now();
            msg.layout.dim[1].size = echo[0].size();
            msg.layout.dim[1].stride = msg.layout.dim[1].size;
            msg.layout.dim[0].stride = msg.layout.dim[0].size * msg.layout.dim[1].size;
            
            msg.data.resize(msg.layout.dim[0].size * msg.layout.dim[1].size);
            for(uint8_t channel_nr=0; channel_nr<4; channel_nr++)
            {
                std::copy(echo[channel_nr].begin(), echo[channel_nr].end(), msg.data.begin() + msg.layout.dim[1].stride * channel_nr);
            }
//             auto end = std::chrono::system_clock::now();
//             auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
//             ROS_INFO_STREAM("copying took " << elapsed.count() << "ms");
            pub.publish(msg);
            
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
    num_sweeps = config.n_sweeps;
    ROS_INFO_STREAM("set sweep to " << num_sweeps << " x " << config.t_sweep << "ms");
}
