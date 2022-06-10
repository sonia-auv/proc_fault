//
// Created by coumarc9 on 7/24/17. and modified by Lucas ^^
//

#ifndef INTERFACE_CONFIGURATION_H
#define INTERFACE_CONFIGURATION_H

#include <cstdint>
#include <cmath>
#include <ros/ros.h>

namespace proc_fault
{
    class Configuration {

    public:

        static void createInstance(const ros::NodeHandlePtr &nh)
        {
            instance = new Configuration(nh);
        }

        static Configuration* getInstance()
        {
            return instance;
        }

        int dvlTimestampsMs = 1000;
        bool dvlEnable = true;

        int depthTimestampsMs = 1000;
        bool depthEnable = true;

        int imuTimestampsMs = 1000;
        bool imuEnable = true;

        int controlTimestampsMs = 1000;
        bool controlEnable = true;

        int cameraTimestampsMs = 1000;
        bool cameraEnable = true;

        bool procImageProcessingEnable = true;

        int sonarTimestampsMs = 1000;
        bool sonarEnable = true;

        int providerHydroTimestampsMs = 1000;
        bool providerHydroEnable = true;

        int procHydroTimestampsMs = 1000;
        bool procHydroEnable = true;

        int powerTimestampsMs = 1000;
        bool powerEnable = true;

        int interfaceTimestampsMs = 1000;
        bool interfaceEnable = true;

        int mappingTimestampsMs = 1000;
        bool mappingEnable = true;

        int initialNodeSleepSecond = 30;
        int loopSleepTimeSecond = 1;

    private:

        Configuration(const ros::NodeHandlePtr &nh);
        ~Configuration();

        ros::NodeHandlePtr nh;
        static Configuration* instance; 

        void Deserialize();

        template <typename TType>
        void FindParameter(const std::string &paramName, TType &attribute);


        };
}

#endif