#include "Configuration.h"

namespace proc_fault
{
    Configuration* Configuration::instance = nullptr;

    Configuration::Configuration(const ros::NodeHandlePtr &nh)
        : nh(nh)
    {
        Deserialize();
    }

    Configuration::~Configuration() {}

    void Configuration::Deserialize() {

        ROS_INFO("Deserialize params");

        FindParameter("/navigation/dvl/timestamp_ms", dvlTimestampsMs);
        FindParameter("/navigation/dvl/enable", dvlEnable);

        FindParameter("/navigation/depth/timestamp_ms", depthTimestampsMs);
        FindParameter("/navigation/depth/enable", depthEnable);

        FindParameter("/navigation/imu/timestamp_ms", imuTimestampsMs);
        FindParameter("/navigation/imu/enable", imuEnable);

        FindParameter("/navigation/control/timestamp_ms", controlTimestampsMs);
        FindParameter("/navigation/control/enable", controlEnable);

        FindParameter("/initial_node_sleep_second", initialNodeSleepSecond);

        ROS_INFO("End deserialize params");
    }

    template <typename TType>
    void Configuration::FindParameter(const std::string &paramName, TType &attribute) {
        if (nh->hasParam("/proc_fault" + paramName)) {
            nh->getParam("/proc_fault" + paramName, attribute);
        } else {
            ROS_WARN_STREAM("Did not find /proc_fault" + paramName
                                    << ". Using default.");
        }
    }

}
