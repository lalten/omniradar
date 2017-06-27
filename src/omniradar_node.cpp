#include <ros/ros.h>
#include <std_msgs/ByteMultiArray.h>

#include <omniradar.h>

#include "vco_tune.h"


int main (int argc, char** argv)
{
    try
    {
        // Set up ROS.
        ros::init(argc, argv, "omniradar_node");
        ros::NodeHandle nh("~");

        // Create a publisher and name the topic.
        ros::Publisher pub = nh.advertise<std_msgs::ByteMultiArray>("radar_raw", 10);

        std_msgs::ByteMultiArray msg;
        msg.layout.dim.resize(2);
        msg.layout.dim[0].label = "channel";
        msg.layout.dim[0].size = 4;
        msg.layout.dim[1].label = "sample";

        // Set up Radar
        Omniradar rdk;
        rdk.ConfigureRadar("06-FE-00-69-60-7A-C0-00-00-00-00-2C-00-00-00-00-01-DC-64");
        rdk.setVCOTune(vco_tune);
        rdk.setSweepTime(20e-3);

        std::vector< std::vector<bool> > echo;

        while(nh.ok())
        {
            echo = rdk.AcquireEcho(2);

            // copy data into message
            msg.layout.dim[1].size = echo[0].size();
            msg.layout.dim[1].stride = msg.layout.dim[1].size;
            msg.layout.dim[0].stride = msg.layout.dim[0].size * msg.layout.dim[1].size;
            
            msg.data.resize(msg.layout.dim[0].size * msg.layout.dim[1].size);
            for(uint8_t channel_nr=0; channel_nr<4; channel_nr++)
            {
                std::copy(echo[channel_nr].begin(), echo[channel_nr].end(), msg.data.begin() + msg.layout.dim[1].stride * channel_nr);
            }
            pub.publish(msg);

        }
    }
    catch(OmniradarException ex)
    {
        ROS_ERROR_STREAM("Omniradar exception " << ex.what());
        throw;
    }
    return 0;
}
