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

        FindParameter("/vision/camera/timestamp_ms", cameraTimestampsMs);
        FindParameter("/vision/camera/enable", cameraEnable);

        FindParameter("/vision/proc_image_processing/enable", procImageProcessingEnable);

        FindParameter("/mapping/sonar/timestamp_ms", cameraTimestampsMs);
        FindParameter("/mapping/sonar/enable", cameraEnable);

        FindParameter("/mapping/procMapping/timestamp_ms", mappingTimestampsMs);
        FindParameter("/mapping/procMapping/enable", mappingEnable);

        FindParameter("/hydro/providerHydro/timestamp_ms", providerHydroTimestampsMs);
        FindParameter("/hydro/providerHydro/enable", providerHydroEnable);

        FindParameter("/hydro/procHydro/timestamp_ms", procHydroTimestampsMs);
        FindParameter("/hydro/procHydro/enable", procHydroEnable);

        FindParameter("/power/providerPower/timestamp_ms", powerTimestampsMs);
        FindParameter("/power/providerPower/enable", powerEnable);

        FindParameter("/internal_com/rs485/timestamp_ms", interfaceTimestampsMs);
        FindParameter("/internal_com/rs485/enable", interfaceEnable);

        FindParameter("/initial_node_sleep_second", initialNodeSleepSecond);
        FindParameter("/loop_sleep_time_second", loopSleepTimeSecond);

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
