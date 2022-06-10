#ifndef PROC_FAULT_SOFTWARE_IMPLEMENTATION_H
#define PROC_FAULT_SOFTWARE_IMPLEMENTATION_H

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/UInt16MultiArray.h>

#include <std_srvs/Empty.h>

#include <sonia_common/SendRS485Msg.h>
#include <sonia_common/BodyVelocityDVL.h>
#include <sonia_common/PointCloud2Extended.h>
#include <sonia_common/PingMsg.h>

#include <sonia_common/GetInformationList.h>
#include <sonia_common/ChangeNetwork.h>
#include <sonia_common/ActuatorDoActionSrv.h>
#include <sonia_common/ModemSendCmd.h>

#include <ros/ros.h>
#include <chrono>

#include "SoftwareInterface.h"
#include "Configuration.h"

namespace proc_fault
{
    // ------------------- Common function ------------------

    class CommonSoftware
    {
        public:
            static std::chrono::milliseconds getCurrentTimeMs()
            {
                return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
            }

            static bool timeDetectionAlgorithm(std::chrono::milliseconds lastTimestamp, unsigned int MaxMs)
            {
                unsigned int diffTimestamp = (CommonSoftware::getCurrentTimeMs() - lastTimestamp).count();
                if(diffTimestamp > MaxMs)
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

    class ProviderCom : public SoftwareInterface
    {
        public:
            ProviderCom()
            {
                service = ros::NodeHandle("~").serviceClient<sonia_common::ModemSendCmd>("/provider_underwater_com/request");
            }

            bool detection()
            {
                return service.exists();
            }

            bool correction()
            {
                return true;
            }

        private:
            ros::ServiceClient service;
    };

    class ProviderThruster : public SoftwareInterface
    {
        public:
            ProviderThruster()
            {
                service = ros::NodeHandle("~").serviceClient<std_srvs::Empty>("/provider_thruster/dry_test");
            }

            bool detection()
            {
                return service.exists();
            }

            bool correction()
            {
                return true;
            }

        private:
            ros::ServiceClient service;
    };

    class ProcActuator : public SoftwareInterface
    {
        public:
            ProcActuator()
            {
                service = ros::NodeHandle("~").serviceClient<sonia_common::ActuatorDoActionSrv>("/proc_actuators/cm_action_srv");
            }

            bool detection()
            {
                return service.exists();
            }

            bool correction()
            {
                return true;
            }

        private:
            ros::ServiceClient service;
    };

    class ProviderActuator : public SoftwareInterface
    {
        public:
            ProviderActuator()
            {
                service = ros::NodeHandle("~").serviceClient<sonia_common::ActuatorDoActionSrv>("/provider_actuators/do_action_srv");
            }

            bool detection()
            {
                return service.exists();
            }

            bool correction()
            {
                return true;
            }

        private:
            ros::ServiceClient service;
    };

    class ProcDetection : public SoftwareInterface
    {
        public:
            ProcDetection()
            {
                service = ros::NodeHandle("~").serviceClient<sonia_common::ChangeNetwork>("/proc_detection/change_network");
            }

            bool detection()
            {
                return service.exists();
            }

            bool correction()
            {
                return true;
            }

        private:
            ros::ServiceClient service;
    };

    class ProcImageProcessing : public SoftwareInterface
    {
        public:
            ProcImageProcessing()
            {
                service = ros::NodeHandle("~").serviceClient<sonia_common::GetInformationList>("/proc_image_processing/get_information_list");
            }

            bool detection()
            {
                return service.exists();
            }

            bool correction()
            {
                return true;
            }

        private:
            ros::ServiceClient service;
    };

/*
    class ProcMapping: public SoftwareInterface
    {
        public:
            ProcMapping()
            {
                service = ros::NodeHandle("~").serviceClient<sonia_common::PingerLocationService>("/proc_mapping/pinger_location_service");
            }
            
            bool detection()
            {
                return service.exists();
            }

            bool correction()
            {
                return true;
            }
        
        private:

        ros::ServiceClient service;
        std::chrono::milliseconds timestamp;
    };
*/
    class InterfaceRs485: public SoftwareInterface
    {
        public:
            InterfaceRs485()
            {
                tx_topic = ros::NodeHandle("~").subscribe("/interface_rs485/dataTx", 10, &InterfaceRs485::interfaceCallback, this);
            }

            void interfaceCallback(const sonia_common::SendRS485Msg &receivedData)
            {
                timestamp = CommonSoftware::getCurrentTimeMs();
            }
            
            bool detection()
            {
                return CommonSoftware::timeDetectionAlgorithm(timestamp, Configuration::getInstance()->interfaceTimestampsMs);
            }

            bool correction()
            {
                return true;
            }
        
        private:

        ros::Subscriber tx_topic;
        std::chrono::milliseconds timestamp;
    };

    class ProviderPower: public SoftwareInterface
    {
        public:
            ProviderPower()
            {
                provider_power_topic = ros::NodeHandle("~").subscribe("/provider_power/voltage", 10, &ProviderPower::powerCallback, this);
            }

            void powerCallback(const std_msgs::Float64MultiArray &receivedData)
            {
                timestamp = CommonSoftware::getCurrentTimeMs();
            }
            
            bool detection()
            {
                return CommonSoftware::timeDetectionAlgorithm(timestamp, Configuration::getInstance()->powerTimestampsMs);
            }

            bool correction()
            {
                return true;
            }
        
        private:

        ros::Subscriber provider_power_topic;
        std::chrono::milliseconds timestamp;
    };

    class ProcHydrophone: public SoftwareInterface
    {
        public:
            ProcHydrophone()
            {
                proc_hydro_topic = ros::NodeHandle("~").subscribe("/proc_hydrophone/ping", 10, &ProcHydrophone::hydroCallback, this);
            }

            void hydroCallback(const sonia_common::PingMsg &receivedData)
            {
                timestamp = CommonSoftware::getCurrentTimeMs();
            }
            
            bool detection()
            {
                return CommonSoftware::timeDetectionAlgorithm(timestamp, Configuration::getInstance()->procHydroTimestampsMs);
            }

            bool correction()
            {
                return true;
            }
        
        private:

        ros::Subscriber proc_hydro_topic;
        std::chrono::milliseconds timestamp;
    };

    class ProviderHydrophone: public SoftwareInterface
    {
        public:
            ProviderHydrophone()
            {
                provider_hydro_topic = ros::NodeHandle("~").subscribe("/provider_hydrophone/ping", 10, &ProviderHydrophone::hydroCallback, this);
            }

            void hydroCallback(const sonia_common::PingMsg &receivedData)
            {
                timestamp = CommonSoftware::getCurrentTimeMs();
            }
            
            bool detection()
            {
                return CommonSoftware::timeDetectionAlgorithm(timestamp, Configuration::getInstance()->providerHydroTimestampsMs);
            }

            bool correction()
            {
                return true;
            }
        
        private:

        ros::Subscriber provider_hydro_topic;
        std::chrono::milliseconds timestamp;
    };

    class ProviderSonar: public SoftwareInterface
    {
        public:
            ProviderSonar()
            {
                provider_sonar_topic = ros::NodeHandle("~").subscribe("/provider_sonar/point_cloud2_extended", 10, &ProviderSonar::sonarCallback, this);
            }

            void sonarCallback(const sonia_common::PointCloud2Extended &receivedData)
            {
                timestamp = CommonSoftware::getCurrentTimeMs();
            }
            
            bool detection()
            {
                return CommonSoftware::timeDetectionAlgorithm(timestamp, Configuration::getInstance()->sonarTimestampsMs);
            }

            bool correction()
            {
                return true;
            }
        
        private:

        ros::Subscriber provider_sonar_topic;
        std::chrono::milliseconds timestamp;
    };

    class ProviderVision: public SoftwareInterface
    {
        public:
            ProviderVision()
            {
                provider_vision_front_topic = ros::NodeHandle("~").subscribe("/camera_array/image_raw/front", 10, &ProviderVision::frontCallback, this);
                provider_vision_bottom_topic = ros::NodeHandle("~").subscribe("/camera_array/image_raw/bottom", 10, &ProviderVision::bottomCallback, this);
            }

            void frontCallback(const sensor_msgs::Image &receivedData)
            {
                topTimestamp = CommonSoftware::getCurrentTimeMs();
            }

            void bottomCallback(const sensor_msgs::Image &receivedData)
            {
                bottomTimestamp = CommonSoftware::getCurrentTimeMs();
            }
            
            bool detection()
            {
                bool topTest = CommonSoftware::timeDetectionAlgorithm(topTimestamp, Configuration::getInstance()->cameraTimestampsMs);
                bool bottomTest = CommonSoftware::timeDetectionAlgorithm(bottomTimestamp, Configuration::getInstance()->cameraTimestampsMs);
                return topTest || bottomTest;
            }

            bool correction()
            {
                return true;
            }
        
        private:

        ros::Subscriber provider_vision_front_topic;
        ros::Subscriber provider_vision_bottom_topic;

        std::chrono::milliseconds topTimestamp;
        std::chrono::milliseconds bottomTimestamp;
    };

/*
    class ProcImageProcessingSoftware: public SoftwareInterface
    {
        public:
            ProcImageProcessingSoftware()
            {

            }

            bool detection()
            {

            }

            bool correction()
            {

            }
    };
    */

    class ProcControl: public SoftwareInterface
    {
        public:
            ProcControl()
            {
                controlSubscriber = ros::NodeHandle("~").subscribe("/provider_thruster/thruster_pwm", 10, &ProcControl::callbackControl, this);
            }

            void callbackControl(const std_msgs::UInt16MultiArray &receivedData)
            {
                timestamp = CommonSoftware::getCurrentTimeMs();
            }

            bool detection()
            {
                return CommonSoftware::timeDetectionAlgorithm(timestamp, Configuration::getInstance()->controlTimestampsMs);
            }

            bool correction()
            {
                return true;
            }
        private:
            ros::Subscriber controlSubscriber;
            std::chrono::milliseconds timestamp;
    };

    class ProviderImu: public SoftwareInterface
    {
        public:

            ProviderImu()
            {
                imuSubscriber = ros::NodeHandle("~").subscribe("/provider_imu/imu_info", 10, &ProviderImu::callbackImu, this);
            }

            void callbackImu(const sensor_msgs::Imu::ConstPtr &receivedData)
            {
                timestamp = CommonSoftware::getCurrentTimeMs();
            }

            bool detection()
            {
                return CommonSoftware::timeDetectionAlgorithm(timestamp, Configuration::getInstance()->imuTimestampsMs);
            }

            bool correction()
            {
                return true;
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
                return CommonSoftware::timeDetectionAlgorithm(timestamp, Configuration::getInstance()->dvlTimestampsMs);
            }

            bool correction()
            {
                return true;
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
                return CommonSoftware::timeDetectionAlgorithm(timestamp, Configuration::getInstance()->depthTimestampsMs);
            }

            bool correction()
            {
                return true;
            }

        private:
            ros::Subscriber depthSubscriber;
            std::chrono::milliseconds timestamp;
    };

}

#endif