#ifndef PROC_FAULT_SOFTWARE_IMPLEMENTATION_H
#define PROC_FAULT_SOFTWARE_IMPLEMENTATION_H

#include <sensor_msgs/Imu.h>
#include <sonia_common/BodyVelocityDVL.h>
#include <std_msgs/Float32.h>

#include <ros/ros.h>
#include <chrono>

#include "SoftwareInterface.h"

namespace proc_fault
{
    // ------------------- Common function ------------------

    class CommonSoftware
    {
        public:
            static const unsigned int depthTimestampsMs = 1000; 
            static const unsigned int dvlTimestampsMs = 1000;
            static const unsigned int imuTimestampsMs = 1000;

            static std::chrono::milliseconds getCurrentTimeMs()
            {
                return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
            }

            static bool timeDetectionAlgorithm(std::chrono::milliseconds lastTimestamp, unsigned int MaxMs)
            {
                if((CommonSoftware::getCurrentTimeMs() - lastTimestamp).count() > MaxMs)
                {
                    return false;
                }

                return true;
            }

            static void restartContainer(std::string containerName)
            {
                //system("docker restart" + containerName);
            }
    };

    // -------------- Software Implementation ---------------
/*
    class providerVisionSoftware: public SoftwareInterface
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

    class procImageProcessingSoftware: public SoftwareInterface
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

    class ProcControl: public SoftwareInterface
    {
        public:
            bool detection()
            {

            }

            bool correction()
            {

            }
    };

    class ProviderImu: public SoftwareInterface
    {
        public:

            ProviderImu()
            {
                imuSubscriber = ros::NodeHandle("~").subscribe("/provider_imu/imu_info", 10, &ProviderImu::callbackImu, this);
            }

            ~ProviderImu()
            {
                
            }

            void callbackImu(const sensor_msgs::Imu::ConstPtr &receivedData)
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

    class ProviderDvl: public SoftwareInterface
    {
        public:

            ProviderDvl()
            {
                dvlSubscriber = ros::NodeHandle("~").subscribe("/provider_dvl/dvl_velocity", 10, &ProviderDvl::callbackDvl, this);
            }

            void callbackDvl(const sonia_common::BodyVelocityDVL::ConstPtr &receivedData)
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
            ros::Subscriber dvlSubscriber;
            std::chrono::milliseconds timestamp;
    };

    class ProviderDepth: public SoftwareInterface
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