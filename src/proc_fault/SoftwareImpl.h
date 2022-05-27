#ifndef PROC_FAULT_SOFTWARE_IMPLEMENTATION_H
#define PROC_FAULT_SOFTWARE_IMPLEMENTATION_H

#include <ros/ros.h>
#include <chrono>

#include "SoftwareInterface.h"

namespace procFault
{
    // ------------------- Common function ------------------

    class CommonSoftware
    {
        public:
            const unsigned int depthTimestampsMs = 1000; 
            const unsigned int dvlTimestampsMs = 1000;
            const unsigned int imuTimestampsMs = 1000;

            static std::chrono::milliseconds getCurrentTimeMs()
            {
                return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
            }

            static bool timeDetectionAlgorithm(std::chrono::milliseconds lastTimestamp, unsigned int MaxMs)
            {
                if((CommonSoftware::getCurrentTimeMs() - timestamp).count() > CommonSoftware::depthTimestampsMs)
                {
                    return false;
                }

                return true;
            }

            static void restartContainer(std::string containerName)
            {
                system("docker restart" + containerName);
            }
    };

    // -------------- Software Implementation ---------------
/*
    class providerVisionSoftware: SoftwareInterface
    {
        public:
            const providerVisionMs = 1000;

            providerVisionSoftware()
            {
                provider_vision_front_topic = ros::NodeHandle("~").subscribe("/camera_array", 100, &DeepFilter::callbackBoundingBox, this);
                provider_vision_bottom_topic = ros::NodeHandle("~").subscribe("/camera_array", 100, &DeepFilter::callbackBoundingBox, this);
            }
            
            bool Detection()
            {

            }

            bool Correction()
            {

            }
        
        private:

        image_transport::Subscriber provider_vision_front_topic;
        image_transport::Subscriber provider_vision_bottom_topic;

    };

    class procImageProcessingSoftware: SoftwareInterface
    {
        public:
            bool Detection()
            {

            }

            bool Correction()
            {

            }
    };
*/

    class ProcControl: SoftwareInterface
    {
        public:
            bool detection()
            {

            }

            bool correction()
            {

            }
    };

    class ProviderImu: SoftwareInterface
    {
        public:

            ProviderDvl()
            {
                depthSubscriber = ros::NodeHandle("~").subscribe("/provider_imu/imu_info", 10, &ProviderImu::callbackImu, this);
            }

            void callbackImu(const sensor_msgs::Imu &receivedData)
            {
                timestamp = CommonSoftware::getCurrentTimeMs();
            }

            bool detection()
            {
                return CommonSoftware::timeDetectionAlgorithm(timestamp, CommonSoftware::imuTimestampsMs);
            }

            bool correction()
            {
                
            }

        private:
            ros::Subscriber imuSubscriber;
            std::chrono::milliseconds timestamp;
    };

    class ProviderDvl: SoftwareInterface
    {
        public:

            ProviderDvl()
            {
                depthSubscriber = ros::NodeHandle("~").subscribe("/provider_dvl/dvl_velocity", 10, &ProviderDvl::callbackDvl, this);
            }

            void callbackDvl(const sonia_common::BodyVelocityDVL &receivedData)
            {
                timestamp = CommonSoftware::getCurrentTimeMs();
            }
        
            bool detection()
            {
                return CommonSoftware::timeDetectionAlgorithm(timestamp, CommonSoftware::dvlTimestampsMs);
            }

            bool correction()
            {

            }
        
        private:
            ros::Subscriber depthSubscriber;
            std::chrono::milliseconds timestamp;
    };

    class ProviderDepth: SoftwareInterface
    {
        public:
            ProviderDepth()
            {
                depthSubscriber = ros::NodeHandle("~").subscribe("/provider_depth/depth", 10, &ProviderDepth::callbackDepth, this);
            }

            void callbackDepth(const std_msgs::Float32::ConstPtr &receivedData)
            {
                timestamp = CommonSoftware::getCurrentTimeMs();
            }

            bool detection()
            {
                return CommonSoftware::timeDetectionAlgorithm(timestamp, CommonSoftware::depthTimestampsMs);
            }

            bool correction()
            {

            }

        private:
            ros::Subscriber depthSubscriber;
            std::chrono::milliseconds timestamp;
    };

}

#endif