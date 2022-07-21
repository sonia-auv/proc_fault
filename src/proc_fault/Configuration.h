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
            if(instance == nullptr)
            {
                ROS_ERROR("didn't succeed to create a new configuration instance");
            }
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

        int boardEscTimestampsMs = 5000;
        bool boardEscEnable = true;

        int cameraTimestampsMs = 1000;
        bool cameraEnable = true;

        bool procImageProcessingEnable = true;

        bool procDetectionEnable = true;

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

        bool providerActuatorEnable = true;

        bool procActuatorEnable = true;

        int boardIoTimestampsMs = 5000;
        bool boardIoEnable = true;

        bool providerUnderwaterComEnable = true;

        bool providerThrusterEnable = true;

        int boardPowerSupplyTimestampsMs = 5000;
        bool boardPowerSupplyEnable = true;

        int boardKillMissionTimestampsMs = 5000;
        bool boardKillMissionEnable = true;

        bool enableMotorResetModule = true;

        bool enableControlCheckerModule = true;

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